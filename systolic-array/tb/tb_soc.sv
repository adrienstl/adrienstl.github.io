`timescale 1ns/1ps

// System test for soc_top: the RV32I core runs sw/firmware.S, which drives the
// accelerator with nothing but lw / sw. The testbench
//   - checks every instruction the core runs against the ISA model; loads from
//     the accelerator and system registers take the word the core saw
//   - on every MARK write, reads that job's A, B and C out of data memory and
//     checks C = A x B with its own arithmetic (and the firmware's own Cref)
//   - checks TOHOST = 1 and no fault at the end, and prints the cycle counts
//   - writes a cycle-by-cycle trace of the first job's accelerator part to
//     BUILD/trace_job0_n<N>.csv and a per-job cycle count for every pc to
//     BUILD/profile_n<N>.csv (the sources of the trace and profile in
//     docs/soc_overview.html)
module tb_soc #(
    parameter int    N     = 4,
    parameter string BUILD = "build"
);
    import tb_rv32i_iss_pkg::*;

    localparam int          IMEM_WORDS = 1024;
    localparam int          DMEM_WORDS = 2048;
    localparam logic [31:0] RAM_BASE   = 32'h1000_0000;
    localparam int          MAX_CYCLES = 5_000_000;

    // ------------------------------------------------------------ DUT
    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic        reset = 1'b1;
    logic        halted, fault, accel_irq;
    logic [31:0] tohost;
    logic [9:0]  led;

    soc_top #(
        .N          (N),
        .IMEM_WORDS (IMEM_WORDS),
        .DMEM_WORDS (DMEM_WORDS),
        .IMEM_INIT  ({BUILD, "/firmware.hex"}),
        .DMEM_INIT  ({BUILD, "/firmware_data.hex"})
    ) dut (
        .clk       (clk),
        .reset     (reset),
        .halted    (halted),
        .fault     (fault),
        .tohost    (tohost),
        .led       (led),
        .accel_irq (accel_irq)
    );

    // ------------------------------------------------------------ ISA model in lockstep
    class soc_iss extends rv32i_iss;
        function new();
            super.new(32'h0, RAM_BASE, DMEM_WORDS * 4);
        endfunction
        // The bus decoder's rules (soc_bus)
        virtual function bit bus_error(input logic [31:0] addr, input logic [3:0] be);
            case (addr[31:28])
                4'h1:    return (addr - RAM_BASE) >= DMEM_WORDS * 4;
                4'h2:    return (addr[27:17] != 0) || (be != 4'hF);
                4'h3:    return (addr[27:4]  != 0) || (be != 4'hF);
                default: return 1'b1;
            endcase
        endfunction
    endclass

    soc_iss iss;

    // Sample point for every check below: 1 ns after the falling edge. The data
    // RAM updates its read data AT the falling edge, so by now everything in
    // this cycle has settled, and the next rising edge is still 4 ns away.
    event sample;
    always @(negedge clk) begin
        #1;
        -> sample;
    end

    always @(sample) begin
        if (!reset && iss != null && (dut.u_core.tr_valid || dut.u_core.tr_trap)) begin
            dut_t d;
            d.valid    = dut.u_core.tr_valid;    d.trap     = dut.u_core.tr_trap;
            d.pc       = dut.u_core.tr_pc;       d.insn     = dut.u_core.tr_insn;
            d.next_pc  = dut.u_core.tr_next_pc;
            d.rd       = dut.u_core.tr_rd;       d.rd_wdata = dut.u_core.tr_rd_wdata;
            d.req      = dut.dbus_req;           d.we       = dut.dbus_we;
            d.addr     = dut.dbus_addr;          d.be       = dut.dbus_be;
            d.wdata    = dut.dbus_wdata;         d.rdata    = dut.dbus_rdata;
            void'(iss.check(d, "firmware"));
        end
    end

    // ------------------------------------------------------------ data memory, back door
    function automatic logic [31:0] rd32(input logic [31:0] a);
        return dut.u_dmem.mem[(a - RAM_BASE) >> 2];
    endfunction

    function automatic logic [15:0] rd16(input logic [31:0] a);
        logic [31:0] w;
        w = rd32(a);
        return a[1] ? w[31:16] : w[15:0];
    endfunction

    // ------------------------------------------------------------ job check
    // Also appends one line per job to BUILD/jobs_n<N>.csv
    int jobs_checked = 0, job_errors = 0, jobs_fd = 0;

    task automatic check_job(input logic [31:0] j);
        int          df, dual, m, k, p, fill, fw_errs, runs, bad, bad_ref;
        logic [31:0] pa, pb, pc, pr, cyc_acc, cyc_sw, busy;
        logic [15:0] a16, b16;
        logic [63:0] want, got, got_ref;
        longint      acc;
        int          l0, l1;
        string       dfn, prec, data;

        df   = rd32(j + 0);   dual = rd32(j + 4);   m = rd32(j + 8);  k = rd32(j + 12);
        p    = rd32(j + 16);  fill = rd32(j + 20);
        pa   = rd32(j + 28);  pb   = rd32(j + 32);  pc = rd32(j + 36); pr = rd32(j + 40);
        cyc_acc = rd32(j + 44); cyc_sw = rd32(j + 48); fw_errs = rd32(j + 52);
        runs    = rd32(j + 56); busy   = rd32(j + 60);

        bad = 0;  bad_ref = 0;
        for (int i = 0; i < m; i++) begin
            for (int c = 0; c < p; c++) begin
                acc = 0;  l0 = 0;  l1 = 0;
                for (int kk = 0; kk < k; kk++) begin
                    a16 = rd16(pa + 2 * (i * k + kk));
                    b16 = rd16(pb + 2 * (kk * p + c));
                    if (dual) begin
                        l0 += int'($signed(a16[7:0]))  * int'($signed(b16[7:0]));
                        l1 += int'($signed(a16[15:8])) * int'($signed(b16[15:8]));
                    end else begin
                        acc += longint'($signed(a16)) * longint'($signed(b16));
                    end
                end
                want    = dual ? {l1, l0} : acc;
                got     = {rd32(pc + 8 * (i * p + c) + 4), rd32(pc + 8 * (i * p + c))};
                got_ref = {rd32(pr + 8 * (i * p + c) + 4), rd32(pr + 8 * (i * p + c))};
                if (got !== want) begin
                    if (bad < 3) $display("  C[%0d][%0d] = %h, expected %h", i, c, got, want);
                    bad++;
                end
                if (got_ref !== want) bad_ref++;
            end
        end

        // Strings from if / else: a ?: between literals of different lengths pads
        // the shorter one with a zero byte
        if (df == 0) dfn = "WS"; else if (df == 1) dfn = "IS"; else dfn = "OS";
        if (dual) prec = "dual"; else prec = "single";
        if (fill) data = "worst"; else data = "random";
        $display("%s  %s %-6s %2d x %2d x %2d %-6s  accel %6d cyc (%3d runs, array busy %5d)  core alone %8d cyc  %5.1fx",
                 (bad == 0 && bad_ref == 0 && fw_errs == 0) ? "PASS" : "FAIL",
                 dfn, prec, m, k, p, data,
                 cyc_acc, runs, busy, cyc_sw, real'(cyc_sw) / real'(cyc_acc));
        if (bad_ref != 0 || fw_errs != 0)
            $display("  firmware's own check: %0d words wrong, software reference off in %0d elements", fw_errs, bad_ref);
        job_errors += bad + bad_ref + fw_errs;

        if (jobs_fd == 0) begin
            jobs_fd = $fopen($sformatf("%s/jobs_n%0d.csv", BUILD, N), "w");
            $fdisplay(jobs_fd, "job,dataflow,dual,m,k,p,fill,accel_cycles,runs,array_busy,core_cycles,errors");
        end
        $fdisplay(jobs_fd, "%0d,%s,%0d,%0d,%0d,%0d,%s,%0d,%0d,%0d,%0d,%0d", jobs_checked, dfn, dual, m, k, p,
                  data, cyc_acc, runs, busy, cyc_sw, bad + bad_ref + fw_errs);
        jobs_checked++;
    endtask

    // ------------------------------------------------------------ profile
    // Clock cycles spent at each pc (stall cycles included), per job. Written
    // to BUILD/profile_n<N>.csv at each MARK as "job,pc,cycles"; the firmware
    // listing maps the pcs to functions.
    int unsigned pc_cycles [IMEM_WORDS];
    int          prof_fd = 0;

    always @(sample) begin
        if (!reset && !halted) pc_cycles[dut.u_core.tr_pc[11:2]]++;
    end

    task automatic dump_profile(input int job);
        if (prof_fd == 0) begin
            prof_fd = $fopen($sformatf("%s/profile_n%0d.csv", BUILD, N), "w");
            $fdisplay(prof_fd, "job,pc,cycles");
        end
        foreach (pc_cycles[i])
            if (pc_cycles[i] != 0) $fdisplay(prof_fd, "%0d,%h,%0d", job, 4 * i, pc_cycles[i]);
        foreach (pc_cycles[i]) pc_cycles[i] = 0;
    endtask

    // MARK is written once per job, after its results are in memory
    always @(sample) begin
        if (!reset && dut.sys_we && dut.dbus_addr[3:0] == 4'hC) begin
            check_job(dut.dbus_wdata);
            dump_profile(jobs_checked - 1);
        end
    end

    // ------------------------------------------------------------ trace of job 0
    // From the first CYCLE read of the first job (the accelerator part starts)
    // to the second (it ends): one line per clock.
    int  trace_fd = 0, cycle_reads = 0, cycle = 0;
    bit  tracing = 0;

    always @(sample) begin
        if (!reset) begin
            cycle++;
            if (dut.dbus_req && !dut.dbus_we && dut.dbus_addr == 32'h3000_0004 && dut.dbus_ready) begin
                cycle_reads++;
                if (cycle_reads == 1) begin
                    trace_fd = $fopen($sformatf("%s/trace_job0_n%0d.csv", BUILD, N), "w");
                    $fdisplay(trace_fd, "cycle,pc,insn,retire,bus,addr,wdata,rdata,phase");
                    tracing = 1;
                end else if (cycle_reads == 2 && tracing) begin
                    tracing = 0;
                    $fclose(trace_fd);
                end
            end
            if (tracing)
                $fdisplay(trace_fd, "%0d,%h,%h,%0d,%s,%h,%h,%h,%0d",
                          cycle, dut.u_core.tr_pc, dut.u_core.tr_insn, dut.u_core.tr_valid,
                          !dut.dbus_req ? "-" : dut.dbus_we ? "W" : dut.dbus_ready ? "R" : "r",
                          dut.dbus_req ? dut.dbus_addr : 32'h0,
                          (dut.dbus_req && dut.dbus_we) ? dut.dbus_wdata : 32'h0,
                          (dut.dbus_req && !dut.dbus_we && dut.dbus_ready) ? dut.dbus_rdata : 32'h0,
                          dut.u_accel.phase);
        end
    end

    // ------------------------------------------------------------ run
    initial begin
        logic [31:0] img  [IMEM_WORDS];
        logic [31:0] dimg [DMEM_WORDS];
        int          cycles;

        $display("soc_top N = %0d: firmware on the RV32I core drives the accelerator", N);

        iss = new();
        foreach (img[i])  img[i]  = 32'h0;
        foreach (dimg[i]) dimg[i] = 32'h0;
        $readmemh({BUILD, "/firmware.hex"}, img);
        $readmemh({BUILD, "/firmware_data.hex"}, dimg);
        foreach (img[i]) iss.prog[i] = img[i];
        foreach (dimg[i])
            for (int b = 0; b < 4; b++) iss.set_byte(RAM_BASE + 4 * i + b, dimg[i][8*b +: 8]);

        repeat (3) @(posedge clk);
        #1 reset = 1'b0;

        cycles = 0;
        while (!halted && cycles < MAX_CYCLES) begin
            @(negedge clk);
            cycles++;
        end

        $display("");
        if (!halted)         $display("FAIL  no halt after %0d cycles (pc %h)", cycles, dut.u_core.tr_pc);
        else if (fault)      $display("FAIL  core stopped on a fault at pc %h", dut.u_core.tr_pc);
        else if (tohost != 1) $display("FAIL  firmware reports TOHOST = %h", tohost);
        else                 $display("PASS  firmware finished: TOHOST = 1, %0d cycles", cycles);
        $display("%s  %0d instructions checked against the ISA model",
                 iss.errors == 0 ? "PASS" : "FAIL", iss.retired);

        if (prof_fd != 0) $fclose(prof_fd);
        if (jobs_fd != 0) $fclose(jobs_fd);
        if (halted && !fault && tohost == 1 && iss.errors == 0 && job_errors == 0 && jobs_checked == 14)
            $display("\nALL TESTS PASSED (N = %0d, %0d jobs)", N, jobs_checked);
        else
            $display("\nERRORS (N = %0d): %0d job errors, %0d lockstep mismatches, %0d / 14 jobs checked",
                     N, job_errors, iss.errors, jobs_checked);
        $finish;
    end

endmodule
