// Skew: delays lane i by i cycles, so the operands of one item reach PE (i, j)
// at the same time as the partial sum (or the other operand) they meet there.
// Lane 0 is a wire; lane i is a chain of i registers, N(N-1)/2 in all.
// Used twice: west edge (one lane per row) and north edge (one per column).
//
// Registers shift on en only. Like the PE pass registers, the valid bit always
// moves and the data only loads when it is real.
module skew #(
    parameter int LANES = 4,
    parameter int W     = 16
)(
    input  logic                    clk,
    input  logic                    reset,
    input  logic                    en,
    input  logic [LANES-1:0][W-1:0] in_data,
    input  logic [LANES-1:0]        in_valid,
    output logic [LANES-1:0][W-1:0] out_data,
    output logic [LANES-1:0]        out_valid
);

    genvar l;

    generate
        for (l = 0; l < LANES; l++) begin : g_lane
            if (l == 0) begin : g_wire
                assign out_data[l]  = in_data[l];
                assign out_valid[l] = in_valid[l];

            end else begin : g_chain
                // Each lane's chain is sized by its own genvar at elaboration: l stages
                logic [W-1:0] d [l];
                logic         v [l];

                always_ff @(posedge clk) begin
                    if (reset) begin
                        for (int s = 0; s < l; s++) v[s] <= 1'b0;
                    end else if (en) begin
                        v[0] <= in_valid[l];
                        for (int s = 1; s < l; s++) v[s] <= v[s-1];
                    end
                end

                always_ff @(posedge clk) begin
                    if (reset) begin
                        for (int s = 0; s < l; s++) d[s] <= '0;
                    end else if (en) begin
                        if (in_valid[l]) d[0] <= in_data[l];
                        for (int s = 1; s < l; s++)
                            if (v[s-1]) d[s] <= d[s-1];
                    end
                end

                assign out_data[l]  = d[l-1];
                assign out_valid[l] = v[l-1];
            end
        end
    endgenerate

endmodule
