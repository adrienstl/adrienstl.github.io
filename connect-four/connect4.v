// =============================================
// Connect-4 (single file, Quartus-safe)
// - No arrays
// - No functions / tasks
// - No 'integer'
// - Simple if/else
// - Each reg driven in exactly one always block
// - KEY0=reset_n (active-low), KEY1=clear, KEY2=drop, SW[2:0]=column (0..6)
// LEDs: [0]=error_full, [1]=current_player, [2]=game_over, [3]=winner, [9]=heartbeat
// =============================================

module connect4_top (
    input      CLOCK_50,
    input [2:0] SW,
    input [3:0] KEY,
    output [9:0] LEDR
);
    // Heartbeat to prove clock is alive
    reg [25:0] hb;
    always @(posedge CLOCK_50) hb <= hb + 26'd1;
    assign LEDR[9] = hb[25];

    // Active-low keys (DE1-SoC)
    wire reset_n = KEY[0];    // 1 when not pressed
    wire clear_n = KEY[1];    // 0 when pressed
    wire drop_n = KEY[2];     // 0 when pressed

    // 2-FF sync + edge detect -> one-cycle pulses for clear/drop
    reg clr_d1, clr_d2, drp_d1, drp_d2;
    always @(posedge CLOCK_50) begin
        clr_d1 <= ~clear_n; clr_d2 <= clr_d1; // ~KEY1 = 1 on press
        drp_d1 <= ~drop_n; drp_d2 <= drp_d1; // ~KEY2 = 1 on press
    end
    wire clear_pulse = clr_d1 & ~clr_d2;
    wire drop_pulse = drp_d1 & ~drp_d2;

    // Interconnects
    wire      current_player;
    wire      clear_board;
    wire      place;
    wire      error_full;
    wire      column_full;
    wire      game_over;
    wire      winner_player;

    // Controller
    connect4_controller u_ctrl (
        .CLOCK_50         (CLOCK_50),
        .reset_n      (reset_n),
        .clear      (clear_pulse),
        .drop       (drop_pulse),
        .column_full (column_full),
        .game_over       (game_over),
        .current_player (current_player),
        .clear_board (clear_board),
        .place       (place),
        .error_full (error_full)
    );

    // Board + winner-detect
    connect4_board u_board (
        .CLOCK_50        (CLOCK_50),
        .reset_n     (reset_n),
        .clear_board (clear_board),
        .place      (place),
        .column_sel (SW[2:0]),
        .current_player (current_player),
        .column_full (column_full),
        .game_over      (game_over),
        .winner_player (winner_player)
    );

    // LEDs (drive per-bit only; do NOT drive LEDR as a whole anywhere)
    assign LEDR[3] = winner_player; // 1 = player1 wins, 0 = player0 wins
    assign LEDR[2] = game_over;          // 1 = game ended
    assign LEDR[1] = current_player; // 0 = P0 turn, 1 = P1 turn
    assign LEDR[0] = error_full;    // tried to drop into a full column
    // LEDR[8:4] unused (left at 0 by default)
endmodule

// ==========================
// Tiny Controller FSM
// ==========================
module connect4_controller (
    input     CLOCK_50,
    input     reset_n,       // async, active-low
    input     clear,      // 1-cycle pulse
    input     drop,        // 1-cycle pulse
    input     column_full, // from board
    input     game_over,         // from board
    output reg current_player, // 0/1
    output reg clear_board, // 1-cycle pulse
    output reg place,          // 1-cycle pulse
    output reg error_full
);
    // States
    localparam S_RESET           = 2'b00;
    localparam S_WAIT_INPUT = 2'b01;
    localparam S_CHECK_COL = 2'b10;
    localparam S_PLACE           = 2'b11;

    reg [1:0] cs, ns;

    // State register
    always @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) cs <= S_RESET;
        else        cs <= ns;
    end

    // Next-state + outputs
    always @* begin
        clear_board = 1'b0;
        place      = 1'b0;
        error_full = 1'b0;
        ns       = cs;
        case (cs)
            S_RESET: begin
                clear_board = 1'b1;        // clear board on entry
                ns = S_WAIT_INPUT;
            end

            S_WAIT_INPUT: begin
                if (clear)          ns = S_RESET;
                else if (!game_over && drop) ns = S_CHECK_COL;
            end

            S_CHECK_COL: begin
                if (game_over) begin
                    ns = S_WAIT_INPUT;
                end else if (column_full) begin
                    error_full = 1'b1;
                    ns = S_WAIT_INPUT;
                end else begin
                    ns = S_PLACE;
                end
            end

            S_PLACE: begin
                if (!game_over) place = 1'b1; // one-cycle write strobe
                ns = S_WAIT_INPUT;
            end

            default: ns = S_RESET;
        endcase
    end

    // Current player toggles only after a successful place (not after game over)
    always @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n)         current_player <= 1'b0;
        else if (cs == S_RESET)      current_player <= 1'b0;
        else if (cs == S_PLACE && !game_over)
            current_player <= ~current_player;
    end
endmodule

// ==========================
// Board + Winner (NO arrays)
// cells = 84-bit packed: 42 cells * 2 bits (00 empty, 01 P0, 10 P1)
// Indexing: row 0..5 (top..bottom), col 0..6, each row is 14 bits.
// ==========================
module connect4_board (
    input         CLOCK_50,
    input         reset_n,
    input         clear_board,
    input         place,
    input    [2:0] column_sel, // 0..6
    input         current_player, // 0 or 1
    output reg       column_full, // top cell non-empty
    output reg       game_over,      // latched when someone wins
    output reg       winner_player // 0 for P0, 1 for P1
);
    // Packed board vector (no arrays)
    reg [83:0] cells;

    // Piece value (no ternary)
    reg [1:0] piece_value;
    always @* begin
        if (current_player) piece_value = 2'b10; else piece_value = 2'b01;
    end

    // Last placement (for winner check)
    reg [2:0] last_row; // 0..5
    reg [2:0] last_col; // 0..6
    reg [1:0] last_piece; // 01 / 10

    // Fire winner-check exactly one cycle after a write
    reg check_arm, check_fire;

    // Row base bit indices (14 bits per row)
    localparam [6:0] RB0 = 7'd0;
    localparam [6:0] RB1 = 7'd14;
    localparam [6:0] RB2 = 7'd28;
    localparam [6:0] RB3 = 7'd42;
    localparam [6:0] RB4 = 7'd56;
    localparam [6:0] RB5 = 7'd70;
    localparam [6:0] STRIDE = 7'd14; // row stride in bits

    // 2*column for bit-selects
    wire [6:0] col2 = {4'b0000, column_sel} << 1;

    // Column-full combinational (top row)
    always @* begin
        column_full = (cells[RB0 + col2 +: 2] != 2'b00);
    end

    // ====== Write / Clear / Arm-Fire (ONLY block that writes cells/last_* and check_*) ======
    always @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) begin
            cells      <= {84{1'b0}};
            last_row <= 3'd0;
            last_col <= 3'd0;
            last_piece <= 2'b00;
            check_arm <= 1'b0;
            check_fire <= 1'b0;
        end else if (clear_board) begin
            cells      <= {84{1'b0}};
            last_row <= 3'd0;
            last_col <= 3'd0;
            last_piece <= 2'b00;
            check_arm <= 1'b0;
            check_fire <= 1'b0;
        end else begin
            // 1-cycle delayed checker pulse
            check_fire <= check_arm;
            check_arm <= 1'b0;

            if (place && !game_over) begin
                // Drop into lowest empty slot in column (rows 5..0)
                if    (cells[RB5 + col2 +: 2] == 2'b00) begin
                    cells[RB5 + col2 +: 2] <= piece_value; last_row<=3'd5; last_col<=column_sel; last_piece<=piece_value; check_arm<=1'b1;
                end else if (cells[RB4 + col2 +: 2] == 2'b00) begin
                    cells[RB4 + col2 +: 2] <= piece_value; last_row<=3'd4; last_col<=column_sel; last_piece<=piece_value; check_arm<=1'b1;
                end else if (cells[RB3 + col2 +: 2] == 2'b00) begin
                    cells[RB3 + col2 +: 2] <= piece_value; last_row<=3'd3; last_col<=column_sel; last_piece<=piece_value; check_arm<=1'b1;
                end else if (cells[RB2 + col2 +: 2] == 2'b00) begin
                    cells[RB2 + col2 +: 2] <= piece_value; last_row<=3'd2; last_col<=column_sel; last_piece<=piece_value; check_arm<=1'b1;
                end else if (cells[RB1 + col2 +: 2] == 2'b00) begin
                    cells[RB1 + col2 +: 2] <= piece_value; last_row<=3'd1; last_col<=column_sel; last_piece<=piece_value; check_arm<=1'b1;
                end else if (cells[RB0 + col2 +: 2] == 2'b00) begin
                    cells[RB0 + col2 +: 2] <= piece_value; last_row<=3'd0; last_col<=column_sel; last_piece<=piece_value; check_arm<=1'b1;
                end
            end
        end
    end

    // Base bits for last_row (combinational)
    reg [6:0] base_bits;
    always @* begin
        case (last_row)
            3'd0: base_bits = RB0;
            3'd1: base_bits = RB1;
            3'd2: base_bits = RB2;
            3'd3: base_bits = RB3;
            3'd4: base_bits = RB4;
            default: base_bits = RB5; // 3'd5
        endcase
    end

    // Bit index of last piece (start of its 2-bit field)
    wire [6:0] last_col2 = {4'b0000, last_col} << 1;
    wire [6:0] idx0_bits = base_bits + last_col2;

    // Neighbor matches (combinational)
    reg l1,l2,l3,r1,r2,r3;
    reg d1v,d2v,d3v;
    reg ul1,ul2,ul3, dr1,dr2,dr3;
    reg ur1,ur2,ur3, dl1,dl2,dl3;

    always @* begin
        l1=0; l2=0; l3=0; r1=0; r2=0; r3=0;
        d1v=0; d2v=0; d3v=0;
        ul1=0; ul2=0; ul3=0; dr1=0; dr2=0; dr3=0;
        ur1=0; ur2=0; ur3=0; dl1=0; dl2=0; dl3=0;

        if (last_piece != 2'b00) begin
            // horizontal
            if (last_col >= 3'd1) if (cells[base_bits + (({4'b0000,(last_col-3'd1)}<<1)) +: 2] == last_piece) l1 = 1;
            if (last_col >= 3'd2) if (cells[base_bits + (({4'b0000,(last_col-3'd2)}<<1)) +: 2] == last_piece) l2 = 1;
            if (last_col >= 3'd3) if (cells[base_bits + (({4'b0000,(last_col-3'd3)}<<1)) +: 2] == last_piece) l3 = 1;
            if (last_col <= 3'd5) if (cells[base_bits + (({4'b0000,(last_col+3'd1)}<<1)) +: 2] == last_piece) r1 = 1;
            if (last_col <= 3'd4) if (cells[base_bits + (({4'b0000,(last_col+3'd2)}<<1)) +: 2] == last_piece) r2 = 1;
            if (last_col <= 3'd3) if (cells[base_bits + (({4'b0000,(last_col+3'd3)}<<1)) +: 2] == last_piece) r3 = 1;

            // vertical (down)
            if (last_row <= 3'd4) if (cells[idx0_bits + STRIDE +: 2] == last_piece) d1v = 1;
            if (last_row <= 3'd3) if (cells[idx0_bits + (STRIDE<<1) +: 2] == last_piece) d2v = 1;
            if (last_row <= 3'd2) if (cells[idx0_bits + (STRIDE + (STRIDE<<1)) +: 2] == last_piece) d3v = 1;

            // "\" diagonal
            if (last_row >= 3'd1 && last_col >= 3'd1) if (cells[idx0_bits - STRIDE - 7'd2 +: 2] == last_piece) ul1 = 1;
            if (last_row >= 3'd2 && last_col >= 3'd2) if (cells[idx0_bits - (STRIDE<<1) - 7'd4 +: 2] == last_piece) ul2 = 1;
            if (last_row >= 3'd3 && last_col >= 3'd3) if (cells[idx0_bits - (STRIDE + (STRIDE<<1)) - 7'd6 +: 2] == last_piece) ul3 = 1;

            if (last_row <= 3'd4 && last_col <= 3'd5) if (cells[idx0_bits + STRIDE + 7'd2 +: 2] == last_piece) dr1 = 1;
            if (last_row <= 3'd3 && last_col <= 3'd4) if (cells[idx0_bits + (STRIDE<<1) + 7'd4 +: 2] == last_piece) dr2 = 1;
            if (last_row <= 3'd2 && last_col <= 3'd3) if (cells[idx0_bits + (STRIDE + (STRIDE<<1)) + 7'd6 +: 2] == last_piece) dr3 = 1;

            // "/" diagonal
            if (last_row >= 3'd1 && last_col <= 3'd5) if (cells[idx0_bits - STRIDE + 7'd2 +: 2] == last_piece) ur1 = 1;
            if (last_row >= 3'd2 && last_col <= 3'd4) if (cells[idx0_bits - (STRIDE<<1) + 7'd4 +: 2] == last_piece) ur2 = 1;
            if (last_row >= 3'd3 && last_col <= 3'd3) if (cells[idx0_bits - (STRIDE + (STRIDE<<1)) + 7'd6 +: 2] == last_piece) ur3 = 1;

            if (last_row <= 3'd4 && last_col >= 3'd1) if (cells[idx0_bits + STRIDE - 7'd2 +: 2] == last_piece) dl1 = 1;
            if (last_row <= 3'd3 && last_col >= 3'd2) if (cells[idx0_bits + (STRIDE<<1) - 7'd4 +: 2] == last_piece) dl2 = 1;
            if (last_row <= 3'd2 && last_col >= 3'd3) if (cells[idx0_bits + (STRIDE + (STRIDE<<1)) - 7'd6 +: 2] == last_piece) dl3 = 1;
        end
    end

    // Counts + any-win
    wire [3:0] h_count = 4'd1 + l1 + l2 + l3 + r1 + r2 + r3;
    wire [3:0] v_count = 4'd1 + d1v + d2v + d3v;
    wire [3:0] d0_count = 4'd1 + ul1 + ul2 + ul3 + dr1 + dr2 + dr3; // "\"
    wire [3:0] d1_count = 4'd1 + ur1 + ur2 + ur3 + dl1 + dl2 + dl3; // "/"

    wire h_win = (h_count >= 4'd4);
    wire v_win = (v_count >= 4'd4);
    wire d0_win = (d0_count >= 4'd4);
    wire d1_win = (d1_count >= 4'd4);
    wire any_win = (last_piece != 2'b00) & (h_win | v_win | d0_win | d1_win);

    // Register result so we can latch cleanly next cycle
    reg any_win_q;
    always @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n)      any_win_q <= 1'b0;
        else if (clear_board) any_win_q <= 1'b0;
        else             any_win_q <= any_win;
    end

    // ====== Winner latch (ONLY block that writes game_over / winner_player) ======
    always @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) begin
            game_over <= 1'b0;
            winner_player <= 1'b0;
        end else if (clear_board) begin
            game_over <= 1'b0;
            winner_player <= 1'b0;
        end else if (check_fire && !game_over) begin
            if (any_win_q) begin
                game_over <= 1'b1;
                winner_player <= (last_piece == 2'b10); // 1 = Player1, 0 = Player0
            end
        end
    end
endmodule
