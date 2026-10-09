// Immediate generator: rebuilds the sign-extended 32-bit immediate from the
// bits each format scatters across the instruction word. The sign bit is
// always instr[31], which is why RISC-V puts it there in every format.
//
//   I  {20{s}}, instr[31:20]                                  loads, OP-IMM, JALR
//   S  {20{s}}, instr[31:25], instr[11:7]                     stores
//   B  {19{s}}, s, instr[7], instr[30:25], instr[11:8], 0     branches (+-4 KB)
//   U  instr[31:12], 12'b0                                    LUI, AUIPC
//   J  {11{s}}, s, instr[19:12], instr[20], instr[30:21], 0   JAL (+-1 MB)
module rv32i_imm_gen
    import rv32i_pkg::*;
(
    input  logic [31:0] instr,
    input  imm_sel_e    sel,
    output logic [31:0] imm
);

    always_comb begin
        case (sel)
            IMM_S:   imm = { {20{instr[31]}}, instr[31:25], instr[11:7] };
            IMM_B:   imm = { {19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0 };
            IMM_U:   imm = { instr[31:12], 12'b0 };
            IMM_J:   imm = { {11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0 };
            default: imm = { {20{instr[31]}}, instr[31:20] };
        endcase
    end

endmodule
