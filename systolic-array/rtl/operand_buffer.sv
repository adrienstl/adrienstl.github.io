// Operand buffer: one bank per array row (or column), each a BUF_DEPTH x 16
// simple dual-port RAM, so one M10K per bank on Cyclone V.
//   write port: CPU, the bank field enables one bank
//   read port : read address counter, the same offset to every bank at once,
//               data one cycle later; the output holds while re = 0 (stall)
module operand_buffer
    import pe_pkg::*, accel_pkg::*;
#(
    parameter int BANKS = 4
)(
    input  logic                         clk,

    input  logic                         we,
    input  logic [BANK_W-1:0]            wbank,
    input  logic [OFF_W-1:0]             woff,
    input  logic [DATA_W-1:0]            wdata,

    input  logic                         re,
    input  logic [OFF_W-1:0]             roff,
    output logic [BANKS-1:0][DATA_W-1:0] rdata
);

    genvar b;

    generate
        for (b = 0; b < BANKS; b++) begin : g_bank
            logic [DATA_W-1:0] mem [BUF_DEPTH];
            logic [DATA_W-1:0] q;

            always_ff @(posedge clk) begin
                if (we && wbank == BANK_W'(b)) mem[woff] <= wdata;
                if (re)                        q <= mem[roff];
            end

            assign rdata[b] = q;
        end
    endgenerate

endmodule
