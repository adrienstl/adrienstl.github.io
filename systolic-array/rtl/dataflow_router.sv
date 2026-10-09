// Dataflow router: picks which operand buffer feeds which edge of the array.
//
//            west (h) stream     north (v) stream    stationary (load buffer)
//   WS       A                   -                   B
//   IS       B                   -                   A (transposed)
//   OS       A                   B                   -
//
// It also turns off the read of a buffer the mode doesn't use.
module dataflow_router
    import pe_pkg::*;
#(
    parameter int N = 4
)(
    input  dataflow_e                cfg_dataflow,

    // Stream read request from the controller
    input  logic                     rd_en,
    input  logic                     rd_valid,
    output logic                     a_re,
    output logic                     b_re,

    // Bank outputs
    input  logic [N-1:0][DATA_W-1:0] a_data,
    input  logic [N-1:0][DATA_W-1:0] b_data,

    // To the skews
    output logic [N-1:0][DATA_W-1:0] west_data,
    output logic [N-1:0]             west_valid,
    output logic [N-1:0][DATA_W-1:0] north_data,
    output logic [N-1:0]             north_valid
);

    assign a_re = rd_en & (cfg_dataflow != DF_IS);      // A streams in WS and OS
    assign b_re = rd_en & (cfg_dataflow != DF_WS);      // B streams in IS and OS

    assign west_data   = (cfg_dataflow == DF_IS) ? b_data : a_data;
    assign west_valid  = {N{rd_valid}};

    assign north_data  = b_data;
    assign north_valid = {N{rd_valid & (cfg_dataflow == DF_OS)}};

endmodule
