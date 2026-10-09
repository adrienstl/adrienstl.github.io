module mac_multiplyer(

    input  logic  mode,

    input  logic signed [15:0]  x,
    input  logic signed [15:0]  y,
    output  logic signed [15:0]  Z_lane0,   // x[7:0]  * y[7:0]  (dual mode)
    output  logic signed [15:0]  Z_lane1,   // x[15:8] * y[15:8] (dual mode)
    output  logic signed [31:0]  Z_single


);
    localparam DUAL   = 1'b1;
    localparam SINGLE = 1'b0;

    always_comb begin

        Z_lane0  = '0;      // default
        Z_lane1  = '0;      // default
        Z_single = '0;      // default
    if (mode == DUAL) begin
        Z_lane0 = $signed(x[7:0])  * $signed(y[7:0]);
        Z_lane1 = $signed(x[15:8]) * $signed(y[15:8]);
       end
    else if (mode == SINGLE)begin

        Z_single = x * y;
        end
    end

endmodule