// Output buffer: one bank per array column, BUF_DEPTH x 64 bits each.
//   write port: output collector, every bank on its own
//   read port : CPU, address = bank | offset | word, data one cycle later
module output_buffer
    import accel_pkg::*;
#(
    parameter int N = 4
)(
    input  logic                    clk,

    input  logic [N-1:0]            wr_en,
    input  logic [N-1:0][OFF_W-1:0] wr_addr,
    input  logic [N-1:0][OUT_W-1:0] wr_data,

    input  logic                    rd_en,
    input  logic [BANK_W-1:0]       rd_bank,
    input  logic [OFF_W-1:0]        rd_off,
    input  logic                    rd_word,     // 0 = low word, 1 = high word
    output logic [31:0]             rd_data
);

    logic [N-1:0][OUT_W-1:0] bank_q;

    genvar b;

    generate
        for (b = 0; b < N; b++) begin : g_bank
            logic [OUT_W-1:0] mem [BUF_DEPTH];
            logic [OUT_W-1:0] q;

            always_ff @(posedge clk) begin
                if (wr_en[b])                         mem[wr_addr[b]] <= wr_data[b];
                if (rd_en && rd_bank == BANK_W'(b))   q <= mem[rd_off];
            end

            assign bank_q[b] = q;
        end
    endgenerate

    // Remember which bank and word were asked for, to pick them next cycle
    logic [BANK_W-1:0] bank_sel;
    logic              word_sel;

    always_ff @(posedge clk) begin
        if (rd_en) begin
            bank_sel <= rd_bank;
            word_sel <= rd_word;
        end
    end

    logic [OUT_W-1:0] entry;
    assign entry   = (bank_sel < BANK_W'(N)) ? bank_q[bank_sel] : '0;
    assign rd_data = word_sel ? entry[63:32] : entry[31:0];

endmodule
