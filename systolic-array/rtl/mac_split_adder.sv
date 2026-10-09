// Accumulator adder: one full-width add (single mode) or two independent
// lane adds (dual mode). In dual mode the carry out of lane 0 is killed so the
// lanes can't leak into each other. Two's complement addition is the same for
// signed and unsigned, so no casts are needed.
module mac_split_adder #(
    parameter int LANE_ACC_W = 18
)(
    input  logic                    dual,
    input  logic [2*LANE_ACC_W-1:0] x,
    input  logic [2*LANE_ACC_W-1:0] y,
    output logic [2*LANE_ACC_W-1:0] sum
);
    logic [LANE_ACC_W:0]   lo;      // extra MSB is the carry out of lane 0
    logic [LANE_ACC_W-1:0] hi;
    logic                  carry;

    assign lo    = x[LANE_ACC_W-1:0] + y[LANE_ACC_W-1:0];
    assign carry = dual ? 1'b0 : lo[LANE_ACC_W];
    assign hi    = x[2*LANE_ACC_W-1:LANE_ACC_W] + y[2*LANE_ACC_W-1:LANE_ACC_W] + carry;
    assign sum   = {hi, lo[LANE_ACC_W-1:0]};

endmodule
