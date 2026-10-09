// Instruction-set model of RV32I for the testbenches: what one instruction
// should do, written from the ISA manual rather than from the RTL.
//
// The testbench steps it once for every instruction the core finishes (or
// faults on) and compares pc, instruction, next pc, register write and bus
// access. The model keeps its own copy of the program and of data RAM, so a
// store the core gets wrong shows up at the store, and a load that reads the
// wrong bytes shows up at the load. Loads from anything else (MMIO) take the
// word the core saw on the bus; the model still does its own byte selection
// and extension.
package tb_rv32i_iss_pkg;

    // What the model says one instruction does
    typedef struct {
        bit          trap;          // faults: nothing is written, pc stays
        bit          halt;          // ECALL / EBREAK
        logic [31:0] pc, insn, next_pc;
        logic [4:0]  rd;            // 0 = no register write
        logic [31:0] rd_wdata;
        bit          mem_re, mem_we;
        logic [31:0] mem_addr;
        logic [3:0]  mem_be;
        logic [31:0] mem_wdata;     // only the lanes in mem_be matter
    } step_t;

    // What the core did this cycle (trace port + data bus)
    typedef struct {
        bit          valid, trap;
        logic [31:0] pc, insn, next_pc;
        logic [4:0]  rd;
        logic [31:0] rd_wdata;
        bit          req, we;
        logic [31:0] addr;
        logic [3:0]  be;
        logic [31:0] wdata, rdata;
    } dut_t;

    function automatic logic [31:0] sext(input logic [31:0] v, input int bits);
        return 32'($signed(v << (32 - bits)) >>> (32 - bits));
    endfunction

    function automatic logic [31:0] lane_mask(input logic [3:0] be);
        return {{8{be[3]}}, {8{be[2]}}, {8{be[1]}}, {8{be[0]}}};
    endfunction

    class rv32i_iss;
        logic [31:0] x [32];
        logic [31:0] pc;
        logic [31:0] prog [int unsigned];       // word index -> instruction
        logic [7:0]  ram  [int unsigned];       // byte address -> byte
        logic [31:0] ram_base, ram_size;        // the range this model owns
        int          retired, errors;
        int          max_msgs = 10;

        function new(input logic [31:0] reset_pc, input logic [31:0] ram_base,
                     input logic [31:0] ram_size);
            this.pc       = reset_pc;
            this.ram_base = ram_base;
            this.ram_size = ram_size;
            foreach (x[i]) x[i] = '0;
        endfunction

        // The system around the core decides which accesses are bus errors
        virtual function bit bus_error(input logic [31:0] addr, input logic [3:0] be);
            return 1'b0;
        endfunction

        function bit in_ram(input logic [31:0] addr);
            return addr >= ram_base && addr - ram_base < ram_size;
        endfunction

        function void set_byte(input logic [31:0] addr, input logic [7:0] v);
            ram[addr] = v;
        endfunction

        function logic [7:0] get_byte(input logic [31:0] addr);
            return ram.exists(addr) ? ram[addr] : 8'h00;
        endfunction

        function void load_prog(input logic [31:0] words [], input logic [31:0] base);
            foreach (words[i]) prog[(base >> 2) + i] = words[i];
        endfunction

        // ------------------------------------------------------------ one instruction
        function step_t step(input logic [31:0] ext_rdata);
            step_t       s;
            logic [31:0] insn, a, b, val, next, addr, word, sh;
            logic [6:0]  opc, f7;
            logic [2:0]  f3;
            logic [4:0]  rd;
            int          size;                     // bytes
            bit          wr, taken, jump;

            s = '{default: '0};
            insn = prog.exists(pc >> 2) ? prog[pc >> 2] : 32'h0;
            opc  = insn[6:0];
            f3   = insn[14:12];
            f7   = insn[31:25];
            rd   = insn[11:7];
            a    = x[insn[19:15]];
            b    = x[insn[24:20]];

            s.pc = pc;  s.insn = insn;
            next = pc + 4;
            wr   = 0;   jump = 0;   val = '0;

            case (opc)
                7'b0110111: begin wr = 1; val = {insn[31:12], 12'h000}; end                     // LUI
                7'b0010111: begin wr = 1; val = pc + {insn[31:12], 12'h000}; end                // AUIPC
                7'b1101111: begin                                                                // JAL
                    wr = 1; val = pc + 4; jump = 1;
                    next = pc + sext({insn[31], insn[19:12], insn[20], insn[30:21], 1'b0}, 21);
                end
                7'b1100111: begin                                                                // JALR
                    if (f3 != 0) s.trap = 1;
                    wr = 1; val = pc + 4; jump = 1;
                    next = (a + sext(insn[31:20], 12)) & ~32'd1;
                end
                7'b1100011: begin                                                                // branches
                    case (f3)
                        3'b000: taken = (a == b);
                        3'b001: taken = (a != b);
                        3'b100: taken = ($signed(a) <  $signed(b));
                        3'b101: taken = ($signed(a) >= $signed(b));
                        3'b110: taken = (a <  b);
                        3'b111: taken = (a >= b);
                        default: begin taken = 0; s.trap = 1; end
                    endcase
                    if (taken) begin
                        jump = 1;
                        next = pc + sext({insn[31], insn[7], insn[30:25], insn[11:8], 1'b0}, 13);
                    end
                end
                7'b0000011: begin                                                                // loads
                    addr = a + sext(insn[31:20], 12);
                    case (f3)
                        3'b000, 3'b100: size = 1;
                        3'b001, 3'b101: size = 2;
                        3'b010:         size = 4;
                        default: begin size = 1; s.trap = 1; end
                    endcase
                    s.mem_re = 1; s.mem_addr = addr;
                    s.mem_be = (size == 4) ? 4'b1111 : (size == 2) ? (4'b0011 << addr[1:0]) : (4'b0001 << addr[1:0]);
                    if (addr % size != 0 || bus_error(addr, s.mem_be)) s.trap = 1;
                    if (!s.trap) begin
                        word = in_ram(addr) ? {get_byte({addr[31:2], 2'd3}), get_byte({addr[31:2], 2'd2}),
                                               get_byte({addr[31:2], 2'd1}), get_byte({addr[31:2], 2'd0})}
                                            : ext_rdata;
                        sh = word >> (8 * addr[1:0]);
                        case (f3)
                            3'b000:  val = sext(sh, 8);
                            3'b001:  val = sext(sh, 16);
                            3'b100:  val = sh & 32'hFF;
                            3'b101:  val = sh & 32'hFFFF;
                            default: val = word;
                        endcase
                        wr = 1;
                    end
                end
                7'b0100011: begin                                                                // stores
                    addr = a + sext({insn[31:25], insn[11:7]}, 12);
                    case (f3)
                        3'b000:  size = 1;
                        3'b001:  size = 2;
                        3'b010:  size = 4;
                        default: begin size = 1; s.trap = 1; end
                    endcase
                    s.mem_we = 1; s.mem_addr = addr;
                    s.mem_be = (size == 4) ? 4'b1111 : (size == 2) ? (4'b0011 << addr[1:0]) : (4'b0001 << addr[1:0]);
                    s.mem_wdata = b << (8 * addr[1:0]);
                    if (addr % size != 0 || bus_error(addr, s.mem_be)) s.trap = 1;
                end
                7'b0010011: begin                                                                // OP-IMM
                    wr = 1;
                    case (f3)
                        3'b000: val = a + sext(insn[31:20], 12);
                        3'b010: val = ($signed(a) < $signed(sext(insn[31:20], 12))) ? 1 : 0;
                        3'b011: val = (a < sext(insn[31:20], 12)) ? 1 : 0;
                        3'b100: val = a ^ sext(insn[31:20], 12);
                        3'b110: val = a | sext(insn[31:20], 12);
                        3'b111: val = a & sext(insn[31:20], 12);
                        3'b001: begin
                            if (f7 != 7'b0000000) s.trap = 1;
                            val = a << insn[24:20];
                        end
                        default: begin                                                           // 101
                            if (f7 == 7'b0000000)      val = a >> insn[24:20];
                            else if (f7 == 7'b0100000) val = 32'($signed(a) >>> insn[24:20]);
                            else                       s.trap = 1;
                        end
                    endcase
                end
                7'b0110011: begin                                                                // OP
                    wr = 1;
                    case ({f7, f3})
                        {7'b0000000, 3'b000}: val = a + b;
                        {7'b0100000, 3'b000}: val = a - b;
                        {7'b0000000, 3'b001}: val = a << b[4:0];
                        {7'b0000000, 3'b010}: val = ($signed(a) < $signed(b)) ? 1 : 0;
                        {7'b0000000, 3'b011}: val = (a < b) ? 1 : 0;
                        {7'b0000000, 3'b100}: val = a ^ b;
                        {7'b0000000, 3'b101}: val = a >> b[4:0];
                        {7'b0100000, 3'b101}: val = 32'($signed(a) >>> b[4:0]);
                        {7'b0000000, 3'b110}: val = a | b;
                        {7'b0000000, 3'b111}: val = a & b;
                        default:              s.trap = 1;
                    endcase
                end
                7'b0001111: if (f3 > 3'b001) s.trap = 1;                                        // FENCE, FENCE.I
                7'b1110011: begin                                                                // ECALL / EBREAK
                    if (insn == 32'h0000_0073 || insn == 32'h0010_0073) s.halt = 1;
                    else                                                 s.trap = 1;
                end
                default: s.trap = 1;
            endcase

            if (jump && next[1:0] != 2'b00) s.trap = 1;       // no C extension: targets are 4-aligned
            s.next_pc = next;

            if (s.trap) begin
                s.rd = 0; s.rd_wdata = 0; s.mem_re = 0; s.mem_we = 0;
                return s;                                     // nothing changes, pc stays
            end

            if (s.mem_we && in_ram(s.mem_addr))
                for (int k = 0; k < 4; k++)
                    if (s.mem_be[k]) set_byte({s.mem_addr[31:2], 2'(k)}, s.mem_wdata[8*k +: 8]);

            s.rd       = wr ? rd : 5'd0;
            s.rd_wdata = (s.rd != 0) ? val : 32'd0;
            if (s.rd != 0) x[s.rd] = val;
            pc = next;
            retired++;
            return s;
        endfunction

        // ------------------------------------------------------------ compare with the core
        // Called for every cycle where the core finishes or faults on an instruction.
        // Returns the number of mismatches (0 or more) and prints the first few.
        function int check(input dut_t d, input string tag = "");
            step_t e;
            string msg;
            int    n;

            e   = step(d.rdata);
            msg = "";
            n   = 0;

            if (d.trap !== e.trap) begin
                msg = {msg, $sformatf(" trap %0b / model %0b;", d.trap, e.trap)}; n++;
            end
            if (d.pc !== e.pc)     begin msg = {msg, $sformatf(" pc %h / model %h;", d.pc, e.pc)}; n++; end
            if (d.insn !== e.insn) begin msg = {msg, $sformatf(" insn %h / model %h;", d.insn, e.insn)}; n++; end

            if (!e.trap && !d.trap) begin
                if (d.next_pc !== e.next_pc) begin
                    msg = {msg, $sformatf(" next pc %h / model %h;", d.next_pc, e.next_pc)}; n++;
                end
                if (d.rd !== e.rd || d.rd_wdata !== e.rd_wdata) begin
                    msg = {msg, $sformatf(" x%0d = %h / model x%0d = %h;", d.rd, d.rd_wdata, e.rd, e.rd_wdata)}; n++;
                end
                if (d.req !== (e.mem_re | e.mem_we) || (d.req && d.we !== e.mem_we)) begin
                    msg = {msg, $sformatf(" bus req %0b we %0b / model re %0b we %0b;", d.req, d.we, e.mem_re, e.mem_we)}; n++;
                end else if (d.req) begin
                    if (d.addr !== e.mem_addr || d.be !== e.mem_be) begin
                        msg = {msg, $sformatf(" bus %h be %b / model %h be %b;", d.addr, d.be, e.mem_addr, e.mem_be)}; n++;
                    end
                    if (d.we && (d.wdata & lane_mask(d.be)) !== (e.mem_wdata & lane_mask(e.mem_be))) begin
                        msg = {msg, $sformatf(" store data %h / model %h;", d.wdata, e.mem_wdata)}; n++;
                    end
                end
            end

            if (n != 0) begin
                if (errors < max_msgs)
                    $display("  %s MISMATCH at pc %h (%h):%s", tag, e.pc, e.insn, msg);
                errors += n;
            end
            return n;
        endfunction
    endclass

endpackage
