`timescale 1ns/1ps

// Checks systolic_array (and every PE in it) by running matrix multiplies in
// every dataflow (WS / IS / OS) and precision (single / dual), with and
// without random stalls, against a lane-exact reference model.
// Override the size from vsim (-gROWS=3 -gCOLS=5) to check a non-square array.
module tb_pe_array #(
    parameter int ROWS = 4,
    parameter int COLS = 4
);
    import pe_pkg::*;
    import tb_ref_pkg::*;

    localparam int M      = 6;      // vectors streamed per WS / IS tile
    localparam int K      = 4;      // reduction length per OS tile
    localparam int MAX_RC = (ROWS > COLS) ? ROWS : COLS;

    // ------------------------------------------------------------ DUT
    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic      reset;
    dataflow_e cfg_dataflow;
    logic      cfg_dual;
    logic      en, stat_load, acc_clear, drain;

    logic [ROWS-1:0][COLS-1:0][DATA_W-1:0] load_in;
    logic [ROWS-1:0][DATA_W-1:0]           west_h;
    logic [ROWS-1:0]                       west_hv;
    logic [COLS-1:0][DATA_W-1:0]           north_v;
    logic [COLS-1:0]                       north_vv;
    logic [COLS-1:0][PSUM_W-1:0]           south_p;
    logic [COLS-1:0]                       south_pv;

    systolic_array #(.ROWS(ROWS), .COLS(COLS)) dut (
        .clk              (clk),
        .reset            (reset),
        .cfg_dataflow     (cfg_dataflow),
        .cfg_dual         (cfg_dual),
        .en               (en),
        .stat_load        (stat_load),
        .acc_clear        (acc_clear),
        .drain            (drain),
        .load_in          (load_in),
        .west_h           (west_h),
        .west_h_valid     (west_hv),
        .north_v          (north_v),
        .north_v_valid    (north_vv),
        .south_psum       (south_p),
        .south_psum_valid (south_pv)
    );

    // ------------------------------------------------------------ scoreboard
    logic [PSUM_W-1:0] exp_q [COLS][$];
    logic [PSUM_W-1:0] got_q [COLS][$];
    int cap_mode;          // 0 off, 1 every valid psum (WS/IS), 2 only while draining (OS)
    bit stall_mode;        // drop en at random while running
    int total_errors = 0;

    // Output buffer model: takes the south edge on every enabled clock
    always @(posedge clk) begin
        if (!reset && en && cap_mode != 0) begin
            for (int j = 0; j < COLS; j++)
                if (south_pv[j] && (cap_mode == 1 || drain))
                    got_q[j].push_back(south_p[j]);
        end
    end

    // ------------------------------------------------------------ helpers
    // Hold the current inputs until an enabled clock edge takes them
    task automatic commit();
        bit taken;
        do begin
            en    = stall_mode ? ($urandom_range(3) != 0) : 1'b1;
            taken = en;
            @(negedge clk);
        end while (!taken);
    endtask

    task automatic idle_streams();
        west_h  = 'x;  west_hv  = '0;
        north_v = 'x;  north_vv = '0;
    endtask

    task automatic start(input dataflow_e df, input logic dual, input bit stall);
        cfg_dataflow = df;
        cfg_dual     = dual;
        stall_mode   = stall;
        cap_mode     = 0;
        en = 1'b1;  stat_load = 1'b0;  acc_clear = 1'b0;  drain = 1'b0;
        idle_streams();
        load_in = 'x;
        reset = 1'b1;
        repeat (2) @(negedge clk);
        reset = 1'b0;
    endtask

    task automatic check(input string name);
        int errors = 0;
        for (int j = 0; j < COLS; j++) begin
            if (got_q[j].size() != exp_q[j].size()) begin
                $display("  col %0d: got %0d results, expected %0d",
                         j, got_q[j].size(), exp_q[j].size());
                errors++;
            end else begin
                for (int k = 0; k < exp_q[j].size(); k++)
                    if (got_q[j][k] !== exp_q[j][k]) begin
                        $display("  col %0d item %0d: got %h expected %h",
                                 j, k, got_q[j][k], exp_q[j][k]);
                        errors++;
                    end
            end
            exp_q[j].delete();
            got_q[j].delete();
        end
        $display("%s  %s", errors == 0 ? "PASS" : "FAIL", name);
        total_errors += errors;
    endtask

    // ------------------------------------------------------------ WS / IS tile
    // Load the stationary grid in one cycle, stream M vectors with a one-step
    // skew per row, collect the column sums at the south edge.
    //   WS: stat[i][j] = B[i][j], row i streams A[m][i] -> column j gives C[m][j]
    //   IS: stat[i][j] = A[j][i], row i streams B[i][n] -> column j gives C[j][n]
    // Both reduce to out[j][m] = sum_i strm[i][m] * stat[i][j].
    task automatic stationary_tile();
        logic [DATA_W-1:0] stat [ROWS][COLS];
        logic [DATA_W-1:0] strm [ROWS][M];
        logic [PSUM_W-1:0] acc;

        foreach (stat[i, j]) stat[i][j] = $urandom;
        foreach (strm[i, m]) strm[i][m] = $urandom;

        for (int j = 0; j < COLS; j++)
            for (int m = 0; m < M; m++) begin
                acc = '0;
                for (int i = 0; i < ROWS; i++) acc = ref_mac(acc, strm[i][m], stat[i][j], cfg_dual);
                exp_q[j].push_back(acc);
            end

        // Every stationary operand goes in on the same clock
        foreach (stat[i, j]) load_in[i][j] = stat[i][j];
        stat_load = 1'b1;
        commit();
        stat_load = 1'b0;
        load_in   = 'x;

        cap_mode = 1;
        for (int s = 0; s < M + ROWS - 1; s++) begin
            for (int i = 0; i < ROWS; i++) begin
                int m;
                m = s - i;
                if (m >= 0 && m < M) begin west_hv[i] = 1'b1; west_h[i] = strm[i][m]; end
                else                 begin west_hv[i] = 1'b0; west_h[i] = 'x;         end
            end
            commit();
        end
        idle_streams();
        repeat (ROWS + COLS + 2) commit();
        cap_mode = 0;
    endtask

    // ------------------------------------------------------------ OS tile
    // Row i streams A[i][k], column j streams B[k][j], each skewed one step.
    // Once the wavefront has passed, drain ROWS times: column j comes out bottom
    // row first, C[ROWS-1][j] ... C[0][j].
    // worst_case fills every operand with the most negative value so every
    // product is the largest positive one; that checks the guard bits.
    task automatic os_tile(input int k_len, input bit worst_case);
        logic [DATA_W-1:0] a [ROWS][16];
        logic [DATA_W-1:0] b [16][COLS];
        logic [DATA_W-1:0] min_word;
        logic [PSUM_W-1:0] acc;
        longint            exact;

        min_word = cfg_dual ? 16'h8080 : 16'h8000;
        foreach (a[i, k]) a[i][k] = worst_case ? min_word : 16'($urandom);
        foreach (b[k, j]) b[k][j] = worst_case ? min_word : 16'($urandom);

        for (int j = 0; j < COLS; j++)
            for (int i = ROWS - 1; i >= 0; i--) begin
                acc = '0;
                for (int k = 0; k < k_len; k++) acc = ref_mac(acc, a[i][k], b[k][j], cfg_dual);
                exp_q[j].push_back(acc);
            end

        if (worst_case) begin
            exact = cfg_dual ? k_len * 64'sd16384 : k_len * (64'sd1 << 30);
            if (cfg_dual ? ($signed(acc[LANE_ACC_W-1:0]) != exact ||
                            $signed(acc[PSUM_W-1:LANE_ACC_W]) != exact)
                         : ($signed(acc) != exact)) begin
                $display("  worst case sum %0d does not fit the accumulator", exact);
                total_errors++;
            end
        end

        acc_clear = 1'b1;
        commit();
        acc_clear = 1'b0;

        for (int s = 0; s < k_len + MAX_RC - 1; s++) begin
            for (int i = 0; i < ROWS; i++) begin
                int k;
                k = s - i;
                if (k >= 0 && k < k_len) begin west_hv[i] = 1'b1; west_h[i] = a[i][k]; end
                else                     begin west_hv[i] = 1'b0; west_h[i] = 'x;      end
            end
            for (int j = 0; j < COLS; j++) begin
                int k;
                k = s - j;
                if (k >= 0 && k < k_len) begin north_vv[j] = 1'b1; north_v[j] = b[k][j]; end
                else                     begin north_vv[j] = 1'b0; north_v[j] = 'x;      end
            end
            commit();
        end
        idle_streams();
        repeat (ROWS + COLS) commit();  // let the wavefront reach PE(ROWS-1, COLS-1)

        cap_mode = 2;
        drain    = 1'b1;
        repeat (ROWS) commit();
        drain    = 1'b0;
        cap_mode = 0;
    endtask

    // ------------------------------------------------------------ test list
    initial begin
        static dataflow_e dfs [3] = '{DF_WS, DF_IS, DF_OS};
        string            name;

        $display("systolic_array %0d x %0d", ROWS, COLS);
        for (int st = 0; st < 2; st++)
            for (int d = 0; d < 3; d++)
                for (int p = 0; p < 2; p++) begin
                    start(dfs[d], p[0], st[0]);
                    // two tiles back to back without a reset in between
                    for (int t = 0; t < 2; t++) begin
                        if (dfs[d] == DF_OS) os_tile(K, 1'b0);
                        else                 stationary_tile();
                        name = $sformatf("%s %-6s %-8s tile %0d", dfs[d].name(),
                                         p ? "dual" : "single", st ? "+stalls" : "", t);
                        check(name);
                    end
                end

        // Worst-case magnitudes at the longest supported reduction
        for (int p = 0; p < 2; p++) begin
            start(DF_OS, p[0], 1'b0);
            os_tile(K_MAX, 1'b1);
            check($sformatf("DF_OS %-6s worst case K = %0d", p ? "dual" : "single", K_MAX));
        end

        if (total_errors == 0) $display("\nALL TESTS PASSED (%0d x %0d)", ROWS, COLS);
        else                   $display("\n%0d ERRORS (%0d x %0d)", total_errors, ROWS, COLS);
        $finish;
    end

endmodule
