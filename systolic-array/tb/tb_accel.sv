`timescale 1ns/1ps

// System test for accel_top. The testbench plays the RV32I core: it lays A and
// B out in the buffers the way a driver would, writes CONFIG / LEN, starts the
// job, polls STATUS, reads C back over MMIO and compares it with C = A x B from
// the reference model.
// Covers every dataflow and precision, random and full-depth lengths,
// worst-case magnitudes, random stalls, back-to-back jobs without a reset,
// config errors, and the write lock while a job runs.
module tb_accel #(
    parameter int N = 4
);
    import pe_pkg::*;
    import accel_pkg::*;
    import tb_ref_pkg::*;

    // ------------------------------------------------------------ DUT
    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic               reset;
    logic               mmio_we = 1'b0;
    logic               mmio_re = 1'b0;
    logic [MMIO_AW-1:0] mmio_addr;
    logic [31:0]        mmio_wdata;
    logic [31:0]        mmio_rdata;
    logic               mmio_rvalid;
    logic               irq;
    logic               stall = 1'b0;

    accel_top #(.N(N)) dut (
        .clk         (clk),
        .reset       (reset),
        .mmio_we     (mmio_we),
        .mmio_re     (mmio_re),
        .mmio_addr   (mmio_addr),
        .mmio_wdata  (mmio_wdata),
        .mmio_rdata  (mmio_rdata),
        .mmio_rvalid (mmio_rvalid),
        .irq         (irq),
        .stall       (stall)
    );

    // Random stalls: while stall_on is set, stall about one cycle in four
    bit stall_on = 1'b0;
    always @(negedge clk) stall <= stall_on && ($urandom_range(3) == 0);

    // ------------------------------------------------------------ address map
    // Written out as documented for software, not taken from accel_pkg, so the
    // test also checks the map itself.
    localparam int unsigned CTRL = 'h00, STATUS = 'h04, CONFIG = 'h08,
                            LEN  = 'h0C, CYCLES = 'h10, INFO   = 'h14;

    function automatic int unsigned load_addr(input int r, input int c);
        return 'h100 + 4*(r*N + c);
    endfunction
    function automatic int unsigned a_addr(input int bank, input int off);
        return 'h08000 + 'h800*bank + 4*off;
    endfunction
    function automatic int unsigned b_addr(input int bank, input int off);
        return 'h10000 + 'h800*bank + 4*off;
    endfunction
    function automatic int unsigned out_addr(input int bank, input int off, input int word);
        return 'h18000 + 'h1000*bank + 8*off + 4*word;
    endfunction

    // ------------------------------------------------------------ bus tasks
    int total_errors = 0;
    int jobs_run     = 0;

    // Every bus task starts and ends just after a falling edge
    task automatic bus_write(input int unsigned addr, input logic [31:0] data);
        mmio_we    = 1'b1;
        mmio_addr  = addr;
        mmio_wdata = data;
        @(negedge clk);
        mmio_we    = 1'b0;
        mmio_addr  = 'x;
        mmio_wdata = 'x;
    endtask

    task automatic bus_read(input int unsigned addr, output logic [31:0] data);
        mmio_re   = 1'b1;
        mmio_addr = addr;
        @(negedge clk);
        mmio_re   = 1'b0;
        mmio_addr = 'x;
        if (mmio_rvalid !== 1'b1) begin
            $display("  no rvalid for the read of %h", addr);
            total_errors++;
        end
        data = mmio_rdata;
    endtask

    // ------------------------------------------------------------ one job
    function automatic logic [DATA_W-1:0] operand(input bit worst, input logic dual);
        // worst: the most negative value in every lane, so every product is the
        // largest positive one and the sums reach the top of the guard bits
        if (worst) return dual ? 16'h8080 : 16'h8000;
        return 16'($urandom);
    endfunction

    // C = A x B. The dataflow decides which dimension is the streamed one:
    //   WS: A is len x N, B is N x N  (B stationary, rows of A stream)
    //   IS: A is N x N,   B is N x len (A stationary, columns of B stream)
    //   OS: A is N x len, B is len x N (both stream, len = K)
    task automatic run_job(input dataflow_e df, input logic dual, input int len,
                           input bit worst, input bit stalls);
        logic [DATA_W-1:0] A [][];
        logic [DATA_W-1:0] B [][];
        logic [PSUM_W-1:0] C [][];
        logic [PSUM_W-1:0] acc;
        logic [63:0]       exp_word, got;
        logic [31:0]       status, rd, lo, hi;
        int                rows, inner, cols, bank, off, polls, errors, exp_cycles;
        string             name;

        name = $sformatf("%s %-6s len %3d%s%s", df.name(), dual ? "dual" : "single", len,
                         worst ? " worst" : "", stalls ? " +stalls" : "");
        errors = 0;

        case (df)
            DF_WS:   begin rows = len; inner = N;   cols = N;   end
            DF_IS:   begin rows = N;   inner = N;   cols = len; end
            default: begin rows = N;   inner = len; cols = N;   end
        endcase

        A = new[rows];
        foreach (A[r]) begin
            A[r] = new[inner];
            for (int k = 0; k < inner; k++) A[r][k] = operand(worst, dual);
        end
        B = new[inner];
        foreach (B[k]) begin
            B[k] = new[cols];
            for (int c = 0; c < cols; c++) B[k][c] = operand(worst, dual);
        end
        C = new[rows];
        foreach (C[r]) begin
            C[r] = new[cols];
            for (int c = 0; c < cols; c++) begin
                acc = '0;
                for (int k = 0; k < inner; k++) acc = ref_mac(acc, A[r][k], B[k][c], dual);
                C[r][c] = acc;
            end
        end

        // What a driver does: put each operand where its dataflow wants it
        case (df)
            DF_WS: begin
                // B[k][j] stays in PE (k, j); bank k streams column k of A
                for (int k = 0; k < N; k++)
                    for (int j = 0; j < N; j++) bus_write(load_addr(k, j), B[k][j]);
                for (int k = 0; k < N; k++)
                    for (int m = 0; m < len; m++) bus_write(a_addr(k, m), A[m][k]);
            end
            DF_IS: begin
                // A[j][k] stays in PE (k, j) (transposed); bank k streams row k of B
                for (int k = 0; k < N; k++)
                    for (int j = 0; j < N; j++) bus_write(load_addr(k, j), A[j][k]);
                for (int k = 0; k < N; k++)
                    for (int n = 0; n < len; n++) bus_write(b_addr(k, n), B[k][n]);
            end
            default: begin
                // A bank i streams row i of A, B bank j streams column j of B
                for (int i = 0; i < N; i++)
                    for (int k = 0; k < len; k++) bus_write(a_addr(i, k), A[i][k]);
                for (int j = 0; j < N; j++)
                    for (int k = 0; k < len; k++) bus_write(b_addr(j, k), B[k][j]);
            end
        endcase

        bus_write(CONFIG, {29'd0, dual, df});
        bus_write(LEN, len);
        stall_on = stalls;
        bus_write(CTRL, 32'h1);

        // Busy now: these must all be ignored. The operand writes hit the last
        // offset, which the stream hasn't read yet, so a leak shows up in C.
        bus_write(a_addr(0, len - 1), 32'hDEAD);
        bus_write(b_addr(0, len - 1), 32'hBEEF);
        bus_write(CONFIG,             32'h7);
        bus_write(LEN,                32'h3);
        bus_write(load_addr(0, 0),    32'h5A5A);

        polls = 0;
        do begin
            bus_read(STATUS, status);
            if (++polls > 20000) begin
                $display("FAIL  %s: never finished, STATUS = %h", name, status);
                $finish;
            end
        end while (!status[1]);
        stall_on = 1'b0;

        if (status[0] || status[2] || irq !== 1'b1) begin
            $display("  STATUS %h / irq %b after done", status, irq);
            errors++;
        end

        bus_read(CONFIG, rd);
        if (rd !== {29'd0, dual, df}) begin $display("  CONFIG changed to %h", rd); errors++; end
        bus_read(LEN, rd);
        if (rd !== len)               begin $display("  LEN changed to %0d", rd);  errors++; end

        // Without stalls the job length is fixed: SETUP + STREAM + WAIT (+ DRAIN) + DONE
        exp_cycles = (df == DF_OS) ? len + 3*N + 2 : len + 2*N + 3;
        bus_read(CYCLES, rd);
        if (!stalls && rd !== exp_cycles) begin
            $display("  CYCLES = %0d, expected %0d", rd, exp_cycles);
            errors++;
        end

        // Read C back: bank = column of C and offset = row (WS, OS), swapped for IS
        foreach (C[r, c]) begin
            bank = (df == DF_IS) ? r : c;
            off  = (df == DF_IS) ? c : r;
            bus_read(out_addr(bank, off, 0), lo);
            bus_read(out_addr(bank, off, 1), hi);
            got      = {hi, lo};
            exp_word = ref_unpack(C[r][c], dual);
            if (got !== exp_word) begin
                if (errors < 5)
                    $display("  C[%0d][%0d] (bank %0d off %0d): got %h expected %h",
                             r, c, bank, off, got, exp_word);
                errors++;
            end
        end

        bus_write(CTRL, 32'h2);                // clear done: irq must drop
        if (irq !== 1'b0) begin $display("  irq still high after CLEAR"); errors++; end

        $display("%s  %s", errors == 0 ? "PASS" : "FAIL", name);
        total_errors += errors;
        jobs_run++;
    endtask

    // ------------------------------------------------------------ config errors
    // A bad config must set ERROR and never start
    task automatic expect_error(input logic [2:0] cfg, input int len, input string name);
        logic [31:0] status;
        bus_write(CONFIG, cfg);
        bus_write(LEN, len);
        bus_write(CTRL, 32'h1);
        bus_read(STATUS, status);
        if (status[2:0] !== 3'b100) begin
            $display("FAIL  rejects %s (STATUS = %h)", name, status);
            total_errors++;
        end else begin
            bus_write(CTRL, 32'h2);
            bus_read(STATUS, status);
            if (status[2] !== 1'b0) begin
                $display("FAIL  CLEAR drops ERROR after %s", name);
                total_errors++;
            end else
                $display("PASS  rejects %s", name);
        end
    endtask

    // ------------------------------------------------------------ test list
    initial begin
        static dataflow_e dfs [3] = '{DF_WS, DF_IS, DF_OS};
        logic [31:0] rd;
        dataflow_e   df;

        reset = 1'b1;
        repeat (3) @(negedge clk);
        reset = 1'b0;

        $display("accel_top N = %0d", N);

        bus_read(INFO, rd);
        if (rd !== {8'(K_MAX), 16'(BUF_DEPTH), 8'(N)}) begin
            $display("FAIL  INFO = %h", rd);
            total_errors++;
        end else
            $display("PASS  INFO");

        expect_error({1'b0, DF_WS}, 0,             "LEN = 0");
        expect_error({1'b0, DF_WS}, BUF_DEPTH + 1, "LEN > BUF_DEPTH");
        expect_error({1'b1, DF_OS}, K_MAX + 1,     "OS LEN > K_MAX");
        expect_error(3'b011,        2,             "dataflow = 3");

        // Every mode, twice each, with and without stalls
        for (int st = 0; st < 2; st++)
            for (int d = 0; d < 3; d++)
                for (int p = 0; p < 2; p++)
                    repeat (2)
                        run_job(dfs[d], p[0],
                                dfs[d] == DF_OS ? $urandom_range(K_MAX, 1) : $urandom_range(3*N, 1),
                                1'b0, st[0]);

        // Largest magnitudes at the longest reduction each mode allows
        for (int d = 0; d < 3; d++)
            for (int p = 0; p < 2; p++)
                run_job(dfs[d], p[0], dfs[d] == DF_OS ? K_MAX : 5, 1'b1, 1'b0);

        // Full-depth streams
        run_job(DF_WS, 1'b0, BUF_DEPTH, 1'b0, 1'b0);
        run_job(DF_IS, 1'b1, BUF_DEPTH, 1'b0, 1'b1);

        // Random mix, back to back, no reset
        repeat (12) begin
            df = dfs[$urandom_range(2)];
            run_job(df, 1'($urandom), df == DF_OS ? $urandom_range(K_MAX, 1) : $urandom_range(40, 1),
                    1'b0, 1'($urandom));
        end

        if (total_errors == 0) $display("\nALL TESTS PASSED (N = %0d, %0d jobs)", N, jobs_run);
        else                   $display("\n%0d ERRORS (N = %0d)", total_errors, N);
        $finish;
    end

endmodule
