// Single-cycle RV32I core: fetch, decode, execute, memory and write-back all
// happen between two clock edges, so every instruction takes one cycle (CPI 1).
// The one exception is a load from a slave that answers late: it holds the
// core with dbus_ready = 0 (the accelerator's MMIO reads take one extra cycle).
//
//   pc -> imem -> instr -+-> decoder -> control word for every mux below
//                        +-> regfile (rs1, rs2) -+-> ALU -> rd
//                        |                       +-> rs1 + offset -> dbus address
//                        |                       +-> branch compare -> next pc
//                        +-> imm gen ------------+-> ALU b, pc + imm
//
// Instruction memory is a synchronous RAM (block RAM on the FPGA). The core
// hands it the address of the NEXT instruction, and it registers that word on
// the same edge that loads the pc, so imem_rdata always holds mem[pc].
//
// Data bus: one access per cycle at most. The request (address, write data,
// byte mask) holds while dbus_ready is low; the access completes in the
// cycle dbus_ready is high. dbus_err ends it with a fault instead.
//
// There are no traps (no Zicsr). ECALL / EBREAK stop the core (halted). An
// illegal instruction, a misaligned access or jump target, or a bus error
// stop it with fault set as well; nothing that instruction would have written
// is written, and pc stays on it. Only reset restarts the core.
module rv32i_core
    import rv32i_pkg::*;
#(
    parameter logic [31:0] RESET_PC = 32'h0000_0000
)(
    input  logic        clk,
    input  logic        reset,          // synchronous, active high

    // Instruction memory
    output logic [31:0] imem_addr,      // address of the next instruction
    input  logic [31:0] imem_rdata,     // the instruction at pc

    // Data bus
    output logic        dbus_req,
    output logic        dbus_we,
    output logic [31:0] dbus_addr,
    output logic [3:0]  dbus_be,
    output logic [31:0] dbus_wdata,
    input  logic [31:0] dbus_rdata,
    input  logic        dbus_ready,
    input  logic        dbus_err,

    output logic        halted,         // stopped (ECALL, EBREAK or fault)
    output logic        fault,          // stopped on an error

    // Retire trace for the testbench (RVFI-style); nothing reads it in synthesis
    output logic        tr_valid,       // an instruction completes this cycle
    output logic        tr_trap,        // the instruction this cycle faults instead
    output logic [31:0] tr_pc,
    output logic [31:0] tr_insn,
    output logic [31:0] tr_next_pc,
    output logic [4:0]  tr_rd,          // 0 = no register written
    output logic [31:0] tr_rd_wdata
);

    // ------------------------------------------------------------ fetch + decode
    logic [31:0] pc, instr;
    ctrl_t       ctrl;

    assign instr = imem_rdata;

    rv32i_decoder u_dec (
        .instr (instr),
        .ctrl  (ctrl)
    );

    logic [4:0] rs1, rs2, rd;
    logic [2:0] funct3;

    assign rs1    = instr[19:15];
    assign rs2    = instr[24:20];
    assign rd     = instr[11:7];
    assign funct3 = instr[14:12];

    // ------------------------------------------------------------ operands
    logic [31:0] rs1_val, rs2_val, imm, wb_data;
    logic        rf_we;

    rv32i_regfile u_rf (
        .clk    (clk),
        .raddr1 (rs1),
        .rdata1 (rs1_val),
        .raddr2 (rs2),
        .rdata2 (rs2_val),
        .we     (rf_we),
        .waddr  (rd),
        .wdata  (wb_data)
    );

    rv32i_imm_gen u_imm (
        .instr (instr),
        .sel   (ctrl.imm_sel),
        .imm   (imm)
    );

    // ------------------------------------------------------------ execute
    logic [31:0] alu_a, alu_b, alu_y;

    always_comb begin
        case (ctrl.a_sel)
            A_PC:    alu_a = pc;
            A_ZERO:  alu_a = 32'd0;
            default: alu_a = rs1_val;
        endcase
    end

    assign alu_b = ctrl.b_imm ? imm : rs2_val;

    rv32i_alu u_alu (
        .op (ctrl.alu_op),
        .a  (alu_a),
        .b  (alu_b),
        .y  (alu_y)
    );

    logic taken;

    rv32i_branch u_br (
        .funct3 (funct3),
        .a      (rs1_val),
        .b      (rs2_val),
        .taken  (taken)
    );

    // ------------------------------------------------------------ next pc
    // Branch and JAL targets get their own adder; JALR's is the ALU's rs1 + imm
    logic [31:0] pc_plus4, pc_target, new_pc;
    logic        redirect;

    assign pc_plus4  = pc + 32'd4;
    assign pc_target = pc + imm;
    assign redirect  = ctrl.jal | ctrl.jalr | (ctrl.branch & taken);

    always_comb begin
        if (ctrl.jalr)                               new_pc = {alu_y[31:1], 1'b0};
        else if (ctrl.jal || (ctrl.branch && taken)) new_pc = pc_target;
        else                                         new_pc = pc_plus4;
    end

    // No compressed instructions, so a target must be a multiple of 4
    logic jump_misaligned;
    assign jump_misaligned = redirect & new_pc[1];

    // ------------------------------------------------------------ memory
    // Loads and stores get their own address adder (rs1 + offset). The data
    // RAM samples the address at the falling edge, so this path has half a
    // cycle; going around the ALU's operand and result muxes saves ~3 ns.
    // The offset is I-type for loads and S-type for stores (instr[5] tells them apart).
    logic [31:0] ls_offset, mem_addr, load_data;
    logic        mem_misaligned;

    assign ls_offset = instr[5] ? { {20{instr[31]}}, instr[31:25], instr[11:7] }
                                : { {20{instr[31]}}, instr[31:20] };
    assign mem_addr  = rs1_val + ls_offset;

    rv32i_lsu u_lsu (
        .funct3     (funct3),
        .active     (ctrl.mem_re | ctrl.mem_we),
        .addr       (mem_addr),
        .store_data (rs2_val),
        .wdata      (dbus_wdata),
        .be         (dbus_be),
        .misaligned (mem_misaligned),
        .rdata      (dbus_rdata),
        .load_data  (load_data)
    );

    // Faults known from the instruction alone; a faulting access never reaches
    // the bus. (The decoder already clears mem_re / mem_we on an illegal
    // instruction, and a load or store is never a jump, so the request only
    // waits on the alignment check, not on the branch compare.)
    logic pre_fault, bus_fault, trap, stall;

    assign pre_fault = ctrl.illegal | mem_misaligned | jump_misaligned;

    assign dbus_req  = (ctrl.mem_re | ctrl.mem_we) & ~mem_misaligned & ~halted;
    assign dbus_we   = ctrl.mem_we;
    assign dbus_addr = mem_addr;

    assign bus_fault = dbus_req & dbus_err;
    assign stall     = dbus_req & ~dbus_ready & ~dbus_err;       // wait for a slow slave
    assign trap      = ~halted & (pre_fault | bus_fault);

    // ------------------------------------------------------------ write-back
    always_comb begin
        case (ctrl.wb_sel)
            WB_MEM:  wb_data = load_data;
            WB_PC4:  wb_data = pc_plus4;
            default: wb_data = alu_y;
        endcase
    end

    logic retire;
    assign retire = ~halted & ~stall & ~trap;
    assign rf_we  = ctrl.reg_we & retire;

    // ------------------------------------------------------------ state
    // pc stays put while stalled, halted, or on the instruction that halts
    logic [31:0] pc_next;
    assign pc_next   = (retire && !ctrl.halt) ? new_pc : pc;
    assign imem_addr = reset ? RESET_PC : pc_next;

    always_ff @(posedge clk) begin
        if (reset) begin
            pc     <= RESET_PC;
            halted <= 1'b0;
            fault  <= 1'b0;
        end else begin
            pc <= pc_next;
            if (trap) begin
                halted <= 1'b1;
                fault  <= 1'b1;
            end else if (retire && ctrl.halt) begin
                halted <= 1'b1;
            end
        end
    end

    // ------------------------------------------------------------ trace
    assign tr_valid    = retire;
    assign tr_trap     = trap;
    assign tr_pc       = pc;
    assign tr_insn     = instr;
    assign tr_next_pc  = new_pc;
    assign tr_rd       = ctrl.reg_we ? rd : 5'd0;
    assign tr_rd_wdata = (ctrl.reg_we && rd != 5'd0) ? wb_data : 32'd0;

endmodule
