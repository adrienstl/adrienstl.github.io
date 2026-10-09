// Branch unit: compares rs1 with rs2 for the six conditional branches. It has
// its own comparators, so the taken decision reads the register file directly
// instead of waiting on the ALU's operand muxes (the target, pc + imm, has its
// own adder in the core).
module rv32i_branch
    import rv32i_pkg::*;
(
    input  logic [2:0]  funct3,
    input  logic [31:0] a,        // rs1
    input  logic [31:0] b,        // rs2
    output logic        taken
);

    logic eq, lt, ltu;

    assign eq  = (a == b);
    assign lt  = $signed(a) < $signed(b);
    assign ltu = a < b;

    always_comb begin
        case (funct3)
            F3_BEQ:  taken =  eq;
            F3_BNE:  taken = ~eq;
            F3_BLT:  taken =  lt;
            F3_BGE:  taken = ~lt;
            F3_BLTU: taken =  ltu;
            F3_BGEU: taken = ~ltu;
            default: taken = 1'b0;
        endcase
    end

endmodule
