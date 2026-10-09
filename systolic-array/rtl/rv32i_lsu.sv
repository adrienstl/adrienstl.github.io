// Load / store unit: moves data between a register and its byte lanes on the
// 32-bit bus. The bus always carries whole aligned words plus a byte mask.
//
//   store: SB copies rs2[7:0] to all four lanes, SH copies rs2[15:0] to both
//          halves, and the mask picks the lanes that are written
//   load : the raw word is shifted down by the byte offset, then sign- or
//          zero-extended (funct3[2] = 1 for LBU / LHU)
//
// There is no trap handler (no Zicsr), so an access that isn't aligned to its
// size is reported as misaligned and the core stops with a fault.
module rv32i_lsu
    import rv32i_pkg::*;
(
    input  logic [2:0]  funct3,
    input  logic        active,        // this instruction is a load or a store
    input  logic [31:0] addr,          // byte address from the ALU

    // Store side
    input  logic [31:0] store_data,    // rs2
    output logic [31:0] wdata,
    output logic [3:0]  be,            // byte lanes the access touches (loads too)
    output logic        misaligned,

    // Load side
    input  logic [31:0] rdata,         // raw word from the bus
    output logic [31:0] load_data      // what goes into rd
);

    logic [1:0] size;
    assign size = funct3[1:0];

    always_comb begin
        case (size)
            SZ_B: begin
                be         = 4'b0001 << addr[1:0];
                wdata      = {4{store_data[7:0]}};
                misaligned = 1'b0;
            end
            SZ_H: begin
                be         = addr[1] ? 4'b1100 : 4'b0011;
                wdata      = {2{store_data[15:0]}};
                misaligned = addr[0];
            end
            default: begin
                be         = 4'b1111;
                wdata      = store_data;
                misaligned = (addr[1:0] != 2'b00);
            end
        endcase
        misaligned = misaligned & active;
    end

    logic [31:0] shifted;
    assign shifted = rdata >> {addr[1:0], 3'b000};

    always_comb begin
        case (size)
            SZ_B:    load_data = funct3[2] ? {24'd0, shifted[7:0]}  : {{24{shifted[7]}},  shifted[7:0]};
            SZ_H:    load_data = funct3[2] ? {16'd0, shifted[15:0]} : {{16{shifted[15]}}, shifted[15:0]};
            default: load_data = rdata;
        endcase
    end

endmodule
