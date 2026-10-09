// Instruction memory: WORDS x 32 ROM with a synchronous read, so it maps to
// block RAM (M10K) on the FPGA.
//
// The core drives the address of the NEXT instruction (its pc_next), and the
// ROM registers that word on the same edge that loads the pc. So rdata always
// holds the instruction at the current pc, and the core still fetches and
// executes in one cycle.
//
// Addresses wrap every WORDS words. Words not in the image read as zero, which
// is an illegal instruction, so running off the end of a program faults.
module imem #(
    parameter int WORDS     = 1024,
    parameter     INIT_FILE = ""          // $readmemh image, one word per line
)(
    input  logic        clk,
    input  logic [31:0] addr,             // byte address of the next instruction
    output logic [31:0] rdata
);

    localparam int AW = $clog2(WORDS);

    logic [31:0] mem [WORDS];

    initial begin
        // synthesis translate_off
        for (int i = 0; i < WORDS; i++) mem[i] = '0;
        // synthesis translate_on
        if (INIT_FILE != "") $readmemh(INIT_FILE, mem);
    end

    always_ff @(posedge clk) begin
        rdata <= mem[addr[AW+1:2]];
    end

endmodule
