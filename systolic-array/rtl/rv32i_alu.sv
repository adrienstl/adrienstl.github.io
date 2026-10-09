// ALU: the ten RV32I integer operations. It also forms the JALR target
// (rs1 + imm), LUI (0 + imm) and AUIPC (pc + imm). Load / store addresses have
// their own adder in the core. Shifts use the low five bits of b, as the ISA says.
module rv32i_alu
    import rv32i_pkg::*;
(
    input  alu_op_e     op,
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic [31:0] y
);

    always_comb begin
        case (op)
            ALU_SUB:  y = a - b;
            ALU_SLL:  y = a << b[4:0];
            ALU_SLT:  y = {31'd0, $signed(a) < $signed(b)};
            ALU_SLTU: y = {31'd0, a < b};
            ALU_XOR:  y = a ^ b;
            ALU_SRL:  y = a >> b[4:0];
            ALU_SRA:  y = $unsigned($signed(a) >>> b[4:0]);
            ALU_OR:   y = a | b;
            ALU_AND:  y = a & b;
            default:  y = a + b;
        endcase
    end

endmodule
