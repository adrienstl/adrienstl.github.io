// Control unit: one RV32I instruction word in, the control word for this cycle
// out. Pure combinational, since the core finishes every instruction in one cycle.
//
// Anything outside RV32I (bad opcode, reserved funct3 / funct7, CSR
// instructions since there is no Zicsr) sets illegal, and the core stops with
// a fault instead of guessing. FENCE and FENCE.I are no-ops: there is one
// hart, no cache and no write buffer, so memory is always in order.
module rv32i_decoder
    import rv32i_pkg::*;
(
    input  logic [31:0] instr,
    output ctrl_t       ctrl
);

    logic [6:0] opcode, funct7;
    logic [2:0] funct3;

    assign opcode = instr[6:0];
    assign funct3 = instr[14:12];
    assign funct7 = instr[31:25];

    // ALU operation named by funct3, shared by OP and OP-IMM. funct7 only
    // matters for ADD / SUB and SRL / SRA.
    alu_op_e f3_op;

    always_comb begin
        case (funct3)
            3'b000:  f3_op = ALU_ADD;
            3'b001:  f3_op = ALU_SLL;
            3'b010:  f3_op = ALU_SLT;
            3'b011:  f3_op = ALU_SLTU;
            3'b100:  f3_op = ALU_XOR;
            3'b101:  f3_op = (funct7 == 7'b0100000) ? ALU_SRA : ALU_SRL;
            3'b110:  f3_op = ALU_OR;
            default: f3_op = ALU_AND;
        endcase
    end

    always_comb begin
        ctrl = '0;                         // no writes, ALU adds rs1 + rs2, imm I

        case (opcode)
            OP_LUI: begin                  // rd = imm
                ctrl.reg_we  = 1'b1;
                ctrl.a_sel   = A_ZERO;
                ctrl.b_imm   = 1'b1;
                ctrl.imm_sel = IMM_U;
            end

            OP_AUIPC: begin                // rd = pc + imm
                ctrl.reg_we  = 1'b1;
                ctrl.a_sel   = A_PC;
                ctrl.b_imm   = 1'b1;
                ctrl.imm_sel = IMM_U;
            end

            OP_JAL: begin                  // rd = pc + 4, pc = pc + imm
                ctrl.reg_we  = 1'b1;
                ctrl.jal     = 1'b1;
                ctrl.imm_sel = IMM_J;
                ctrl.wb_sel  = WB_PC4;
            end

            OP_JALR: begin                 // rd = pc + 4, pc = (rs1 + imm) & ~1
                ctrl.reg_we  = 1'b1;
                ctrl.jalr    = 1'b1;
                ctrl.b_imm   = 1'b1;
                ctrl.wb_sel  = WB_PC4;
                ctrl.illegal = (funct3 != 3'b000);
            end

            OP_BRANCH: begin               // pc = pc + imm if the compare holds
                ctrl.branch  = 1'b1;
                ctrl.imm_sel = IMM_B;
                ctrl.illegal = (funct3 == 3'b010) || (funct3 == 3'b011);
            end

            OP_LOAD: begin                 // rd = mem[rs1 + imm]
                ctrl.reg_we  = 1'b1;
                ctrl.mem_re  = 1'b1;
                ctrl.b_imm   = 1'b1;
                ctrl.wb_sel  = WB_MEM;
                ctrl.illegal = (funct3 == 3'b011) || (funct3 == 3'b110) || (funct3 == 3'b111);
            end

            OP_STORE: begin                // mem[rs1 + imm] = rs2
                ctrl.mem_we  = 1'b1;
                ctrl.b_imm   = 1'b1;
                ctrl.imm_sel = IMM_S;
                ctrl.illegal = funct3[2] || (funct3[1:0] == 2'b11);
            end

            OP_IMM: begin                  // rd = rs1 op imm
                ctrl.reg_we  = 1'b1;
                ctrl.b_imm   = 1'b1;
                ctrl.alu_op  = f3_op;
                // Shift amounts are 5 bits; the top of the immediate must be
                // 0000000 (SLLI, SRLI) or 0100000 (SRAI)
                if (funct3 == 3'b001)
                    ctrl.illegal = (funct7 != 7'b0000000);
                else if (funct3 == 3'b101)
                    ctrl.illegal = (funct7 != 7'b0000000) && (funct7 != 7'b0100000);
            end

            OP_REG: begin                  // rd = rs1 op rs2
                ctrl.reg_we  = 1'b1;
                ctrl.alu_op  = f3_op;
                if (funct7 == 7'b0100000) begin
                    if (funct3 == 3'b000)      ctrl.alu_op  = ALU_SUB;
                    else if (funct3 != 3'b101) ctrl.illegal = 1'b1;    // only SUB and SRA
                end else if (funct7 != 7'b0000000) begin
                    ctrl.illegal = 1'b1;                              // M extension and others
                end
            end

            OP_FENCE: begin                // FENCE, FENCE.I: nothing to order
                ctrl.illegal = funct3[2] || funct3[1];
            end

            OP_SYSTEM: begin               // ECALL = 0x00000073, EBREAK = 0x00100073
                if (instr[31:21] == 11'd0 && instr[19:7] == 13'd0)
                    ctrl.halt    = 1'b1;
                else
                    ctrl.illegal = 1'b1;       // CSR instructions, MRET, WFI, ...
            end

            default: ctrl.illegal = 1'b1;
        endcase

        // An illegal instruction has no side effects: the core just stops
        if (ctrl.illegal) begin
            ctrl.reg_we = 1'b0;
            ctrl.mem_re = 1'b0;
            ctrl.mem_we = 1'b0;
            ctrl.branch = 1'b0;
            ctrl.jal    = 1'b0;
            ctrl.jalr   = 1'b0;
        end
    end

endmodule
