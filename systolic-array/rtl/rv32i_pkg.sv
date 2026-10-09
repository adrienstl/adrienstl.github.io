// RV32I base ISA: opcodes, funct3 codes, and the control word the decoder
// hands to the rest of the single-cycle datapath.
package rv32i_pkg;

    // ------------------------------------------------------------ opcodes (instr[6:0])
    localparam logic [6:0] OP_LUI    = 7'b0110111;
    localparam logic [6:0] OP_AUIPC  = 7'b0010111;
    localparam logic [6:0] OP_JAL    = 7'b1101111;
    localparam logic [6:0] OP_JALR   = 7'b1100111;
    localparam logic [6:0] OP_BRANCH = 7'b1100011;
    localparam logic [6:0] OP_LOAD   = 7'b0000011;
    localparam logic [6:0] OP_STORE  = 7'b0100011;
    localparam logic [6:0] OP_IMM    = 7'b0010011;
    localparam logic [6:0] OP_REG    = 7'b0110011;
    localparam logic [6:0] OP_FENCE  = 7'b0001111;
    localparam logic [6:0] OP_SYSTEM = 7'b1110011;

    // ------------------------------------------------------------ funct3
    // Branches
    localparam logic [2:0] F3_BEQ  = 3'b000;
    localparam logic [2:0] F3_BNE  = 3'b001;
    localparam logic [2:0] F3_BLT  = 3'b100;
    localparam logic [2:0] F3_BGE  = 3'b101;
    localparam logic [2:0] F3_BLTU = 3'b110;
    localparam logic [2:0] F3_BGEU = 3'b111;

    // Loads / stores: funct3[1:0] is the size, funct3[2] means zero-extend
    localparam logic [1:0] SZ_B = 2'b00;
    localparam logic [1:0] SZ_H = 2'b01;
    localparam logic [1:0] SZ_W = 2'b10;

    // ------------------------------------------------------------ control word
    typedef enum logic [3:0] {
        ALU_ADD, ALU_SUB, ALU_SLL, ALU_SLT, ALU_SLTU,
        ALU_XOR, ALU_SRL, ALU_SRA, ALU_OR,  ALU_AND
    } alu_op_e;

    typedef enum logic [2:0] { IMM_I, IMM_S, IMM_B, IMM_U, IMM_J } imm_sel_e;

    typedef enum logic [1:0] {
        A_RS1,       // most instructions
        A_PC,        // AUIPC
        A_ZERO       // LUI: 0 + imm
    } a_sel_e;

    typedef enum logic [1:0] {
        WB_ALU,      // ALU result
        WB_MEM,      // load data
        WB_PC4       // JAL / JALR link address
    } wb_sel_e;

    typedef struct packed {
        logic     reg_we;    // write rd
        a_sel_e   a_sel;     // ALU input a
        logic     b_imm;     // ALU input b: 1 = immediate, 0 = rs2
        alu_op_e  alu_op;
        imm_sel_e imm_sel;
        wb_sel_e  wb_sel;
        logic     mem_re;    // load
        logic     mem_we;    // store
        logic     branch;    // conditional branch (taken if the compare says so)
        logic     jal;
        logic     jalr;
        logic     halt;      // ECALL / EBREAK: stop the core
        logic     illegal;   // not an RV32I instruction: stop the core with a fault
    } ctrl_t;

endpackage
