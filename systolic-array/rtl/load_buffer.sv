// Load buffer: N x N registers holding the stationary operands for WS / IS.
// The CPU writes them one at a time (index r*N + c); they all drive the array
// at once, so a single stat_load cycle fills every PE.
module load_buffer
    import pe_pkg::*, accel_pkg::*;
#(
    parameter int N = 4
)(
    input  logic                            clk,
    input  logic                            reset,
    input  logic                            we,
    input  logic [LOAD_IDX_W-1:0]           idx,     // r*N + c
    input  logic [DATA_W-1:0]               wdata,
    output logic [N-1:0][N-1:0][DATA_W-1:0] q        // [r][c] goes to PE (r, c)
);

    genvar r, c;

    generate
        for (r = 0; r < N; r++) begin : g_row
            for (c = 0; c < N; c++) begin : g_col
                logic [DATA_W-1:0] val;

                always_ff @(posedge clk) begin
                    if (reset)                                    val <= '0;
                    else if (we && idx == LOAD_IDX_W'(r*N + c))   val <= wdata;
                end

                assign q[r][c] = val;
            end
        end
    endgenerate

endmodule
