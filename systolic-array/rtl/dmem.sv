// Data memory: WORDS x 32 RAM with a byte mask on writes, in block RAM (M10K).
//
// Block RAM only reads through a register, and a single-cycle core needs a
// load's data in the same cycle as its address. So this RAM works on the
// FALLING edge: the core has the address, write data and mask ready by
// mid-cycle, the RAM reads or writes at the falling edge, and the read data
// has the second half of the cycle to reach the register file at the next
// rising edge. Every load and store still takes one cycle; the cost is that
// the address path must fit in half a clock period.
//
//   rising edge   falling edge   rising edge
//   |-- decode, regfile, ALU --|-- RAM out, LSU, write-back --|
//                              ^ RAM registers address / data
module dmem #(
    parameter int WORDS     = 2048,
    parameter     INIT_FILE = ""          // $readmemh image (.data section)
)(
    input  logic        clk,
    input  logic        we,
    input  logic [3:0]  be,
    input  logic [31:0] addr,             // byte address; the bus decoder checks the range
    input  logic [31:0] wdata,
    output logic [31:0] rdata             // valid from the falling edge on
);

    localparam int AW = $clog2(WORDS);

    // One instruction never reads and writes in the same cycle, so there is
    // no read-during-write case to preserve
    (* ramstyle = "no_rw_check" *) logic [3:0][7:0] mem [WORDS];
    logic [AW-1:0] a;

    assign a = addr[AW+1:2];

    initial begin
        // synthesis translate_off
        for (int i = 0; i < WORDS; i++) mem[i] = '0;
        // synthesis translate_on
        if (INIT_FILE != "") $readmemh(INIT_FILE, mem);
    end

    // Plain always: always_ff forbids the initial block (and testbench
    // back-door loads) from touching mem
    always @(negedge clk) begin
        if (we) begin
            if (be[0]) mem[a][0] <= wdata[7:0];
            if (be[1]) mem[a][1] <= wdata[15:8];
            if (be[2]) mem[a][2] <= wdata[23:16];
            if (be[3]) mem[a][3] <= wdata[31:24];
        end
        rdata <= mem[a];
    end

endmodule
