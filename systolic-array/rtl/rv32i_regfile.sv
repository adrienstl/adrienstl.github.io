// Register file: x0 - x31, two combinational read ports and one write port.
// x0 reads as zero and is never written.
//
// A write lands on the clock edge that ends the instruction, so an
// instruction reading the register the previous one wrote sees the new value
// with no forwarding: that is the point of a single-cycle core.
module rv32i_regfile (
    input  logic        clk,

    input  logic [4:0]  raddr1,
    output logic [31:0] rdata1,
    input  logic [4:0]  raddr2,
    output logic [31:0] rdata2,

    input  logic        we,
    input  logic [4:0]  waddr,
    input  logic [31:0] wdata
);

    logic [31:0] regs [32];

    // RISC-V leaves x1 - x31 undefined at reset. Simulation starts them at
    // zero so the ISA model and the core agree; software must not rely on it.
    // synthesis translate_off
    initial begin
        for (int i = 0; i < 32; i++) regs[i] = '0;
    end
    // synthesis translate_on

    // Plain always: always_ff forbids the initial block above from touching regs
    always @(posedge clk) begin
        if (we && waddr != 5'd0) regs[waddr] <= wdata;
    end

    assign rdata1 = (raddr1 == 5'd0) ? 32'd0 : regs[raddr1];
    assign rdata2 = (raddr2 == 5'd0) ? 32'd0 : regs[raddr2];

endmodule
