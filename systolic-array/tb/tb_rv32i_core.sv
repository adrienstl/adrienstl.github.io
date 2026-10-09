`timescale 1ns/1ps

// Core-level test for rv32i_core, with the real instruction memory and a
// behavioural data bus that adds random wait states, so loads and stores also
// exercise the stall path. The ISA model (tb_rv32i_iss_pkg) checks every
// instruction in lockstep: pc, instruction, next pc, register write, bus access.
//
//   1. sw/isa_test.S (built by sw/rvasm.py): every instruction, self-checking
//   2. faults: illegal encodings, misaligned accesses and jumps, bus errors
//   3. constrained-random programs, with and without wait states
//
// Bus map the core sees here:
//   0x1000_0000  RAM, 16 KB, mirrored by the model
//   0x4000_0000  I/O, 4 KB: reads return words the model can't predict, so it
//                takes the core's word, the same way MMIO works in the SoC
//   anything else: dbus_err
module tb_rv32i_core #(
    parameter string BUILD      = "build",     // where rvasm wrote isa_test.hex
    parameter int    N_RANDOM   = 150,         // random programs
    parameter int    RANDOM_LEN = 300          // instructions in each
);
    import tb_rv32i_iss_pkg::*;

    localparam logic [31:0] RAM_BASE   = 32'h1000_0000;
    localparam int          RAM_BYTES  = 16384;
    localparam logic [31:0] IO_BASE    = 32'h4000_0000;
    localparam int          IO_BYTES   = 4096;
    localparam logic [31:0] ERR_BASE   = 32'h2000_0000;     // unmapped
    localparam int          IMEM_WORDS = 4096;
    localparam logic [31:0] EBREAK     = 32'h0010_0073;

    // ------------------------------------------------------------ DUT
    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic        reset = 1'b1;
    logic [31:0] imem_addr, imem_rdata;
    logic        dbus_req, dbus_we, dbus_ready, dbus_err, halted, fault;
    logic [31:0] dbus_addr, dbus_wdata, dbus_rdata;
    logic [3:0]  dbus_be;
    logic        tr_valid, tr_trap;
    logic [31:0] tr_pc, tr_insn, tr_next_pc, tr_rd_wdata;
    logic [4:0]  tr_rd;

    rv32i_core dut (
        .clk         (clk),
        .reset       (reset),
        .imem_addr   (imem_addr),
        .imem_rdata  (imem_rdata),
        .dbus_req    (dbus_req),
        .dbus_we     (dbus_we),
        .dbus_addr   (dbus_addr),
        .dbus_be     (dbus_be),
        .dbus_wdata  (dbus_wdata),
        .dbus_rdata  (dbus_rdata),
        .dbus_ready  (dbus_ready),
        .dbus_err    (dbus_err),
        .halted      (halted),
        .fault       (fault),
        .tr_valid    (tr_valid),
        .tr_trap     (tr_trap),
        .tr_pc       (tr_pc),
        .tr_insn     (tr_insn),
        .tr_next_pc  (tr_next_pc),
        .tr_rd       (tr_rd),
        .tr_rd_wdata (tr_rd_wdata)
    );

    imem #(.WORDS(IMEM_WORDS)) u_imem (
        .clk   (clk),
        .addr  (imem_addr),
        .rdata (imem_rdata)
    );

    // ------------------------------------------------------------ data bus model
    logic [7:0]  ram [RAM_BYTES];
    logic [31:0] io_salt;
    int          max_wait = 0;        // each access waits 0 .. max_wait cycles
    int          draw, left, waits_now, wait_cycles;
    bit          in_flight;
    logic        hit_ram, hit_io;

    assign hit_ram    = (dbus_addr >= RAM_BASE) && (dbus_addr < RAM_BASE + RAM_BYTES);
    assign hit_io     = (dbus_addr >= IO_BASE)  && (dbus_addr < IO_BASE + IO_BYTES);
    assign dbus_err   = dbus_req && !(hit_ram || hit_io);
    assign waits_now  = in_flight ? left : draw;
    assign dbus_ready = dbus_req && !dbus_err && (waits_now == 0);

    // Read data is only good in the cycle ready is high; before that the bus
    // carries junk, so a core that uses it early gets caught
    always_comb begin
        int unsigned off;
        off = {dbus_addr[31:2], 2'b00} - RAM_BASE;
        if (!dbus_ready)  dbus_rdata = 32'hBAD0_0000 ^ io_salt;
        else if (hit_io)  dbus_rdata = {dbus_addr[31:2], 2'b00} * 32'h9E37_79B1 ^ io_salt;
        else if (hit_ram) dbus_rdata = {ram[off + 3], ram[off + 2], ram[off + 1], ram[off]};
        else              dbus_rdata = 32'hDEAD_BEEF;
    end

    always @(posedge clk) begin
        if (reset) begin
            in_flight <= 1'b0;
            draw      <= 0;
            io_salt   <= 32'h1234_5678;
        end else if (dbus_req && !dbus_err) begin
            if (dbus_ready) begin
                in_flight <= 1'b0;
                draw      <= $urandom_range(max_wait);
                if (dbus_we && hit_ram)
                    for (int k = 0; k < 4; k++)
                        if (dbus_be[k]) ram[{dbus_addr[31:2], 2'b00} - RAM_BASE + k] <= dbus_wdata[8*k +: 8];
                if (hit_io) io_salt <= io_salt * 32'd1664525 + 32'd1013904223;
            end else begin
                in_flight   <= 1'b1;
                left        <= waits_now - 1;
                wait_cycles <= wait_cycles + 1;
            end
        end
    end

    // ------------------------------------------------------------ lockstep check
    class core_iss extends rv32i_iss;
        function new();
            super.new(32'h0, RAM_BASE, RAM_BYTES);
        endfunction
        virtual function bit bus_error(input logic [31:0] addr, input logic [3:0] be);
            return !((addr >= RAM_BASE && addr < RAM_BASE + RAM_BYTES) ||
                     (addr >= IO_BASE  && addr < IO_BASE + IO_BYTES));
        endfunction
    endclass

    core_iss iss;
    string   run_name;
    int      total_errors = 0, total_retired = 0, runs = 0;

    always @(negedge clk) begin
        if (!reset && iss != null) begin
            if (tr_valid || tr_trap) begin
                dut_t d;
                d.valid    = tr_valid;    d.trap     = tr_trap;
                d.pc       = tr_pc;       d.insn     = tr_insn;    d.next_pc = tr_next_pc;
                d.rd       = tr_rd;       d.rd_wdata = tr_rd_wdata;
                d.req      = dbus_req;    d.we       = dbus_we;    d.addr    = dbus_addr;
                d.be       = dbus_be;     d.wdata    = dbus_wdata; d.rdata   = dbus_rdata;
                void'(iss.check(d, run_name));
            end
            // Once stopped, the core must stay quiet
            if (halted && (tr_valid || tr_trap || dbus_req)) begin
                $display("  %s: activity after halt at pc %h", run_name, tr_pc);
                iss.errors++;
            end
        end
    end

    // ------------------------------------------------------------ encoders
    localparam int OP_LUI = 'h37, OP_AUIPC = 'h17, OP_JAL = 'h6F, OP_JALR = 'h67, OP_BRANCH = 'h63,
                   OP_LOAD = 'h03, OP_STORE = 'h23, OP_IMM = 'h13, OP_REG = 'h33;

    function automatic logic [31:0] enc_r(int f7, int rs2, int rs1, int f3, int rd, int op);
        return (f7 << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | op;
    endfunction
    function automatic logic [31:0] enc_i(int imm, int rs1, int f3, int rd, int op);
        return ((imm & 'hFFF) << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | op;
    endfunction
    function automatic logic [31:0] enc_s(int imm, int rs2, int rs1, int f3);
        return (((imm >> 5) & 'h7F) << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | ((imm & 'h1F) << 7) | OP_STORE;
    endfunction
    function automatic logic [31:0] enc_b(int off, int rs2, int rs1, int f3);
        return (((off >> 12) & 1) << 31) | (((off >> 5) & 'h3F) << 25) | (rs2 << 20) | (rs1 << 15) |
               (f3 << 12) | (((off >> 1) & 'hF) << 8) | (((off >> 11) & 1) << 7) | OP_BRANCH;
    endfunction
    function automatic logic [31:0] enc_u(int imm20, int rd, int op);
        return ((imm20 & 'hFFFFF) << 12) | (rd << 7) | op;
    endfunction
    function automatic logic [31:0] enc_j(int off, int rd);
        return (((off >> 20) & 1) << 31) | (((off >> 1) & 'h3FF) << 21) | (((off >> 11) & 1) << 20) |
               (((off >> 12) & 'hFF) << 12) | (rd << 7) | OP_JAL;
    endfunction
    function automatic int hi20(logic [31:0] v); return ((v + 'h800) >> 12) & 'hFFFFF; endfunction
    function automatic int lo12(logic [31:0] v); return int'($signed(v[11:0])); endfunction

    // ------------------------------------------------------------ running a program
    typedef logic [31:0] word_q [$];

    // Load program and RAM, clear registers, release reset. RAM bytes come from
    // data[] (words) when given, random otherwise.
    task automatic start(input string name, input word_q prog, input word_q data, input int waits);
        run_name = name;
        max_wait = waits;
        reset    = 1'b1;
        iss      = new();
        foreach (u_imem.mem[i]) u_imem.mem[i] = (i < prog.size()) ? prog[i] : 32'h0;
        for (int i = 0; i < IMEM_WORDS; i++) iss.prog[i] = u_imem.mem[i];
        for (int b = 0; b < RAM_BYTES; b++) begin
            ram[b] = (b / 4 < data.size()) ? data[b / 4][8*(b % 4) +: 8]
                   : (data.size() == 0)   ? 8'($urandom) : 8'h00;
            iss.set_byte(RAM_BASE + b, ram[b]);
        end
        for (int r = 0; r < 32; r++) dut.u_rf.regs[r] = '0;
        wait_cycles = 0;
        // Release just after a rising edge, so the checker's next falling-edge
        // sample already sees the first instruction (no race with it)
        repeat (2) @(posedge clk);
        #1 reset = 1'b0;
    endtask

    // Run until the core stops; returns the clock cycles the program took
    task automatic finish(input int max_cycles, output int cycles);
        cycles = 0;
        while (!halted && cycles < max_cycles) begin
            @(negedge clk);
            cycles++;
        end
        if (halted) cycles--;                         // halted shows one sample after the EBREAK's cycle
        repeat (3) @(negedge clk);                    // it must stay stopped
        if (!halted) begin
            $display("  %s: no halt after %0d cycles (pc %h)", run_name, max_cycles, tr_pc);
            iss.errors++;
        end
    endtask

    function automatic void report(input bit ok, input string detail = "");
        ok = ok && iss.errors == 0;
        $display("%s  %s%s", ok ? "PASS" : "FAIL", run_name, detail);
        total_errors += ok ? 0 : (iss.errors == 0 ? 1 : iss.errors);
        total_retired += iss.retired;
        runs++;
    endfunction

    // ------------------------------------------------------------ 1. ISA test
    task automatic isa_test(input int waits);
        logic [31:0] img [IMEM_WORDS];
        logic [31:0] dimg [RAM_BYTES / 4];
        word_q       prog, data;
        int          cycles;

        foreach (img[i])  img[i]  = 32'h0;
        foreach (dimg[i]) dimg[i] = 32'h0;
        $readmemh({BUILD, "/isa_test.hex"}, img);
        $readmemh({BUILD, "/isa_test_data.hex"}, dimg);
        foreach (img[i])  prog.push_back(img[i]);
        foreach (dimg[i]) data.push_back(dimg[i]);

        start($sformatf("isa_test, wait states 0..%0d", waits), prog, data, waits);
        finish(20000, cycles);
        if (dut.u_rf.regs[10] !== 32'd1)
            $display("  isa_test reports a0 = %0d: test %0d failed", dut.u_rf.regs[10], dut.u_rf.regs[10] >> 1);
        report(halted && !fault && dut.u_rf.regs[10] === 32'd1,
               $sformatf("  (%0d instructions, %0d cycles, %0d wait)", iss.retired, cycles, wait_cycles));
        if (waits == 0 && cycles != iss.retired)
            $display("  note: %0d cycles for %0d instructions", cycles, iss.retired);
    endtask

    // ------------------------------------------------------------ 2. faults
    // Prologue x31 = RAM base, x30 = unmapped, x5 = 1, x1 = 0; then the
    // instruction under test at 0x0C, then EBREAKs (jump targets land there)
    task automatic fault_case(input string name, input logic [31:0] insn, input bit expect_fault);
        word_q prog, none;
        int    cycles;

        prog = '{enc_u(RAM_BASE >> 12, 31, OP_LUI), enc_u(ERR_BASE >> 12, 30, OP_LUI),
                 enc_i(1, 0, 0, 5, OP_IMM), insn, EBREAK, EBREAK, EBREAK, EBREAK, EBREAK, EBREAK};
        start({expect_fault ? "fault: " : "no fault: ", name}, prog, none, 1);
        finish(100, cycles);
        report(halted && fault == expect_fault &&
               (!expect_fault || tr_pc == 32'h0C) &&                  // stopped on it
               dut.u_rf.regs[5] === 32'd1 && dut.u_rf.regs[1] === 32'd0);   // wrote nothing
    endtask

    task automatic fault_tests();
        fault_case("all-zero word",                32'h0000_0000,                   1);
        fault_case("all-ones word",                32'hFFFF_FFFF,                   1);
        fault_case("csrrw (no Zicsr)",             32'h3000_1073,                   1);
        fault_case("mret",                         32'h3020_0073,                   1);
        fault_case("ecall with rd != 0",           32'h0000_00F3,                   1);
        fault_case("mul (no M extension)",         enc_r(1, 6, 5, 0, 7, OP_REG),    1);
        fault_case("sll with funct7 0100000",      enc_r('h20, 6, 5, 1, 7, OP_REG), 1);
        fault_case("slli with shamt[5] set",       enc_i('h020, 5, 1, 7, OP_IMM),   1);
        fault_case("srai with bad funct7",         enc_i('h610, 5, 5, 7, OP_IMM),   1);
        fault_case("load funct3 011 (ld)",         enc_i(0, 31, 3, 7, OP_LOAD),     1);
        fault_case("load funct3 110 (lwu)",        enc_i(0, 31, 6, 7, OP_LOAD),     1);
        fault_case("store funct3 011 (sd)",        enc_s(0, 5, 31, 3),              1);
        fault_case("branch funct3 010",            enc_b(8, 0, 0, 2),               1);
        fault_case("jalr funct3 001",              enc_i(0, 31, 1, 7, OP_JALR),     1);
        fault_case("fence funct3 010",             32'h0000_200F,                   1);
        fault_case("opcode 1011011 (custom-2)",    32'h0000_005B,                   1);
        fault_case("misaligned lw",                enc_i(1, 31, 2, 5, OP_LOAD),     1);
        fault_case("misaligned lh",                enc_i(3, 31, 1, 5, OP_LOAD),     1);
        fault_case("misaligned lhu",               enc_i(1, 31, 5, 5, OP_LOAD),     1);
        fault_case("misaligned sw",                enc_s(2, 5, 31, 2),              1);
        fault_case("misaligned sh",                enc_s(1, 5, 31, 1),              1);
        fault_case("jal to pc + 6",                enc_j(6, 1),                     1);
        fault_case("taken beq to pc + 6",          enc_b(6, 0, 0, 0),               1);
        fault_case("jalr to RAM base + 2",         enc_i(2, 31, 0, 1, OP_JALR),     1);
        fault_case("load from unmapped address",   enc_i(0, 30, 2, 5, OP_LOAD),     1);
        fault_case("store to unmapped address",    enc_s(0, 5, 30, 0),              1);
        fault_case("load just below RAM",          enc_i(-4, 31, 2, 5, OP_LOAD),    1);
        fault_case("not-taken beq to pc + 6",      enc_b(6, 5, 0, 0),               0);
        fault_case("ecall halts cleanly",          32'h0000_0073,                   0);
        fault_case("misaligned byte is fine: lb",  enc_i(3, 31, 0, 6, OP_LOAD),     0);
    endtask

    // ------------------------------------------------------------ 3. random programs
    function automatic logic [31:0] rnd_val();
        case ($urandom_range(9))
            0:       return 32'h0;
            1:       return 32'h1;
            2:       return 32'hFFFF_FFFF;
            3:       return 32'h8000_0000;
            4:       return 32'h7FFF_FFFF;
            5:       return $urandom_range(31);
            6:       return -$urandom_range(64);
            default: return $urandom;
        endcase
    endfunction

    function automatic int rnd_imm12();
        case ($urandom_range(7))
            0:       return 0;
            1:       return -1;
            2:       return 2047;
            3:       return -2048;
            default: return int'($urandom_range(4095)) - 2048;
        endcase
    endfunction

    // Every register gets a value first (x31 = RAM base + 2 KB, x30 = I/O
    // base + 2 KB, never overwritten, so +-2 KB offsets stay in range; x29 is
    // only written by the auipc of a jalr pair). Branches and jumps only go
    // forward, so every program ends at its EBREAK. Nothing may jump into the
    // middle of an auipc + jalr or mv + load pair: the second half would use a
    // stale x29 / x28.
    function automatic word_q gen_random(input int body_len);
        word_q p;
        bit    targeted [int];
        int    total, i, kind, rd, rs1, rs2, size, off, j, f3, span;
        int    r_f7 [10] = '{0, 'h20, 0, 0, 0, 0, 0, 'h20, 0, 0};
        int    r_f3 [10] = '{0, 0, 1, 2, 3, 4, 5, 5, 6, 7};
        int    i_f3 [6]  = '{0, 2, 3, 4, 6, 7};
        int    b_f3 [6]  = '{0, 1, 4, 5, 6, 7};
        int    l_f3 [5]  = '{0, 1, 2, 4, 5};

        for (int r = 1; r < 32; r++) begin
            logic [31:0] v;
            v = (r == 31) ? RAM_BASE + 2048 : (r == 30) ? IO_BASE + 2048 : rnd_val();
            p.push_back(enc_u(hi20(v), r, OP_LUI));
            p.push_back(enc_i(lo12(v), r, 0, r, OP_IMM));
        end

        total = p.size() + body_len;                 // index of the EBREAK
        i = p.size();
        while (i < total) begin
            kind = $urandom_range(99);
            rd   = $urandom_range(28);               // x0 .. x28
            rs1  = $urandom_range(31);
            rs2  = $urandom_range(31);
            span = (total - i - 1 < 15) ? total - i - 1 : 15;
            if (kind < 25) begin                                         // OP
                j = $urandom_range(9);
                p.push_back(enc_r(r_f7[j], rs2, rs1, r_f3[j], rd, OP_REG));
            end else if (kind < 45) begin                                // OP-IMM
                p.push_back(enc_i(rnd_imm12(), rs1, i_f3[$urandom_range(5)], rd, OP_IMM));
            end else if (kind < 52) begin                                // shifts by immediate
                case ($urandom_range(2))
                    0:       p.push_back(enc_i($urandom_range(31), rs1, 1, rd, OP_IMM));
                    1:       p.push_back(enc_i($urandom_range(31), rs1, 5, rd, OP_IMM));
                    default: p.push_back(enc_i('h400 | $urandom_range(31), rs1, 5, rd, OP_IMM));
                endcase
            end else if (kind < 56) begin                                // LUI / AUIPC
                p.push_back(enc_u($urandom, rd, $urandom_range(1) ? OP_LUI : OP_AUIPC));
            end else if (kind < 66) begin                                // loads, mostly RAM
                f3   = l_f3[$urandom_range(4)];
                size = 1 << (f3 & 3);
                off  = size * (int'($urandom_range(4095 / size)) - 2048 / size);
                p.push_back(enc_i(off, $urandom_range(5) ? 31 : 30, f3, rd, OP_LOAD));
            end else if (kind < 69 && span >= 1 && !targeted.exists(i + 1)) begin   // rd == rs1
                f3   = l_f3[$urandom_range(4)];                          // (x28 = x31, then lw x28, off(x28))
                size = 1 << (f3 & 3);
                off  = size * (int'($urandom_range(4095 / size)) - 2048 / size);
                p.push_back(enc_i(0, 31, 0, 28, OP_IMM));
                p.push_back(enc_i(off, 28, f3, 28, OP_LOAD));
                i++;
            end else if (kind < 81) begin                                // stores
                f3   = $urandom_range(2);
                size = 1 << f3;
                off  = size * (int'($urandom_range(4095 / size)) - 2048 / size);
                p.push_back(enc_s(off, rs2, $urandom_range(7) ? 31 : 30, f3));
            end else if (kind < 91 && span >= 1) begin                   // forward branch
                j = i + 1 + $urandom_range(span - 1);
                targeted[j] = 1;
                p.push_back(enc_b(4 * (j - i), rs2, rs1, b_f3[$urandom_range(5)]));
            end else if (kind < 94 && span >= 1) begin                   // forward jal
                j = i + 1 + $urandom_range(span - 1);
                targeted[j] = 1;
                p.push_back(enc_j(4 * (j - i), rd));
            end else if (kind < 97 && span >= 2 && !targeted.exists(i + 1)) begin   // auipc x29 + jalr
                j = i + 2 + $urandom_range(span - 2);
                targeted[j] = 1;
                p.push_back(enc_u(0, 29, OP_AUIPC));
                p.push_back(enc_i(4 * (j - i), 29, 0, rd, OP_JALR));
                i++;
            end else begin                                               // fence
                p.push_back(32'h0FF0_000F);
            end
            i++;
        end
        p.push_back(EBREAK);
        return p;
    endfunction

    task automatic random_tests();
        word_q prog, none;
        int    cycles, waits, fails_before, retired_before, n;

        fails_before   = total_errors;
        retired_before = total_retired;
        for (n = 0; n < N_RANDOM; n++) begin
            waits = (n % 2) ? $urandom_range(3) : 0;
            prog  = gen_random(RANDOM_LEN);
            start($sformatf("random %0d", n), prog, none, waits);
            finish(20 * RANDOM_LEN, cycles);
            if (!halted || fault || iss.errors != 0) report(0, $sformatf("  (wait states 0..%0d)", waits));
            else begin
                total_retired += iss.retired;               // report() skipped: keep the log short
                if (waits == 0 && cycles != iss.retired)
                    $display("  random %0d: %0d cycles for %0d instructions with no wait states",
                             n, cycles, iss.retired);
            end
        end
        $display("%s  %0d random programs, %0d instructions checked in lockstep",
                 total_errors == fails_before ? "PASS" : "FAIL", N_RANDOM, total_retired - retired_before);
    endtask

    // ------------------------------------------------------------ test list
    initial begin
        $display("rv32i_core tests");
        isa_test(0);
        isa_test(3);
        fault_tests();
        random_tests();

        if (total_errors == 0) $display("\nALL TESTS PASSED (%0d instructions checked)", total_retired);
        else                   $display("\n%0d ERRORS", total_errors);
        $finish;
    end

endmodule
