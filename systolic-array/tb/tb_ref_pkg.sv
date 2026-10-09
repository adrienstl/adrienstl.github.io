// Reference model shared by the testbenches: the arithmetic exactly as the
// hardware should do it, lane by lane, written without looking at the RTL.
package tb_ref_pkg;
    import pe_pkg::*;

    // One MAC: acc + x*y, wrapping at the psum width (per lane in dual mode)
    function automatic logic [PSUM_W-1:0] ref_mac(input logic [PSUM_W-1:0] acc,
                                                  input logic [DATA_W-1:0] x,
                                                  input logic [DATA_W-1:0] y,
                                                  input logic              dual);
        longint p0, p1, p;
        logic [LANE_ACC_W-1:0] l0, l1;
        if (dual) begin
            p0 = longint'($signed(x[LANE_W-1:0]))      * longint'($signed(y[LANE_W-1:0]));
            p1 = longint'($signed(x[DATA_W-1:LANE_W])) * longint'($signed(y[DATA_W-1:LANE_W]));
            l0 = acc[LANE_ACC_W-1:0]      + p0[LANE_ACC_W-1:0];
            l1 = acc[PSUM_W-1:LANE_ACC_W] + p1[LANE_ACC_W-1:0];
            return {l1, l0};
        end
        p = longint'($signed(x)) * longint'($signed(y));
        return acc + p[PSUM_W-1:0];
    endfunction

    // What the CPU should read for one psum: {high word, low word}
    //   single: the 36-bit sum as a sign-extended 64-bit value
    //   dual  : lane 1 and lane 0, each a sign-extended int32
    function automatic logic [63:0] ref_unpack(input logic [PSUM_W-1:0] p,
                                               input logic              dual);
        int     lane0, lane1;
        longint whole;
        if (dual) begin
            lane0 = $signed(p[LANE_ACC_W-1:0]);
            lane1 = $signed(p[PSUM_W-1:LANE_ACC_W]);
            return {lane1, lane0};
        end
        whole = $signed(p);
        return whole;
    endfunction

endpackage
