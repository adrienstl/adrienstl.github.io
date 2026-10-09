// Output collector: takes the psums falling out of the south edge and writes
// them into the output buffer, one bank per array column.
//
//   de-skew : each column has its own write pointer, so item m of every column
//             lands at offset m even though column j arrives j cycles later
//             (no register triangle needed)
//   reorder : OS drain emits the bottom row first, C[N-1][j] .. C[0][j], so the
//             offset counts down: row i lands at offset i
//   unpack  : each 36-bit psum becomes two sign-extended 32-bit words the CPU
//             can use directly
//               single: {sext(psum[35:32]), psum[31:0]}  (one 64-bit value)
//               dual  : {sext(lane 1), sext(lane 0)}     (two int32)
module output_collector
    import pe_pkg::*, accel_pkg::*;
#(
    parameter int N = 4
)(
    input  logic                     clk,
    input  logic                     reset,
    input  logic                     job_start,   // zero the write pointers
    input  logic                     en,
    input  logic                     collect,     // from the controller
    input  dataflow_e                cfg_dataflow,
    input  logic                     cfg_dual,

    // South edge of the array
    input  logic [N-1:0][PSUM_W-1:0] psum,
    input  logic [N-1:0]             psum_valid,

    // Output buffer write ports, one per bank
    output logic [N-1:0]             wr_en,
    output logic [N-1:0][OFF_W-1:0]  wr_addr,
    output logic [N-1:0][OUT_W-1:0]  wr_data
);

    function automatic logic [OUT_W-1:0] unpack(input logic [PSUM_W-1:0] p,
                                                input logic              dual);
        if (dual)
            return { {(32-LANE_ACC_W){p[PSUM_W-1]}},     p[PSUM_W-1:LANE_ACC_W],
                     {(32-LANE_ACC_W){p[LANE_ACC_W-1]}}, p[LANE_ACC_W-1:0] };
        return { {(OUT_W-PSUM_W){p[PSUM_W-1]}}, p };
    endfunction

    logic is_os;
    assign is_os = (cfg_dataflow == DF_OS);

    genvar j;

    generate
        for (j = 0; j < N; j++) begin : g_col
            logic [OFF_W-1:0] ptr;     // results taken from this column so far

            // Same edge rule as the PE registers: take a value on an enabled clock
            assign wr_en[j] = en & collect & psum_valid[j];

            always_ff @(posedge clk) begin
                if (reset || job_start) ptr <= '0;
                else if (wr_en[j])      ptr <= ptr + 1'b1;
            end

            assign wr_addr[j] = is_os ? OFF_W'(N - 1) - ptr : ptr;
            assign wr_data[j] = unpack(psum[j], cfg_dual);
        end
    endgenerate

endmodule
