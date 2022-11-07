module spandex_fsm(
    input logic clk,
    input logic rst,
    input logic do_flush_next,
    input logic do_rsp_next,
    input logic do_fwd_next,
    input logic do_ongoing_flush_next,
    input logic do_cpu_req_next,
    input logic is_flush_all,
    input logic l2_rd_rsp_ready_int,
    input logic l2_req_out_ready_int,
    input logic l2_rsp_out_ready_int,
    input logic l2_inval_ready_int, 
    input logic fwd_stall,
    input logic evict_stall,
    input logic ongoing_flush,
    input logic set_fwd_stall,
    input logic clr_fwd_stall, 
    input logic reqs_hit,
    input logic reqs_hit_next,
    input logic set_conflict,
    input logic set_set_conflict_reqs,
    input logic ongoing_atomic,
    input logic clr_set_conflict_reqs, 
    input logic tag_hit_next,
    input logic empty_way_found_next,
    input logic tag_hit, 
    input logic incr_flush_set, 
    input var logic [`REQS_BITS-1:0] reqs_i,
    input var logic [`REQS_BITS-1:0] reqs_i_next, 
    input var logic [`L2_SET_BITS:0] flush_set,
    input var logic [`L2_WAY_BITS:0] flush_way, 
    input l2_way_t way_hit, 
    input l2_way_t empty_way, 
    input l2_way_t evict_way_buf, 
    input l2_way_t way_hit_next,
    input line_t line_out, 
    input var reqs_buf_t reqs[`N_REQS],
    input var state_t states_buf[`L2_WAYS], 
    input var state_t rd_data_state[`L2_NUM_PORTS],
    input var hprot_t rd_data_hprot[`L2_NUM_PORTS],
    input var line_t lines_buf[`L2_WAYS],
    input var l2_tag_t tags_buf[`L2_WAYS],
        
    line_breakdown_l2_t.in line_br, 
    line_breakdown_l2_t.in line_br_next, 
    addr_breakdown_t.in addr_br, 
    addr_breakdown_t.in addr_br_next,
    l2_rsp_in_t.in l2_rsp_in,
    l2_fwd_in_t.in l2_fwd_in, 
    l2_cpu_req_t.in l2_cpu_req, 
    
    output logic decode_en,
    output logic lookup_en,
    output logic rd_mem_en,
    output logic lookup_mode, 
    output logic wr_rst,
    output logic wr_en_state,
    output logic fill_reqs,
    output logic wr_en_line, 
    output logic wr_req_state,
    output logic wr_req_state_atomic,
    output logic wr_req_invack_cnt,
    output logic wr_req_line,
    output logic wr_en_put_reqs,
    output logic wr_req_tag,
    output logic put_reqs_atomic,
    output logic wr_en_evict_way,
    output logic l2_rd_rsp_valid_int,
    output logic l2_req_out_valid_int,
    output logic l2_rsp_out_valid_int,
    output logic l2_inval_valid_int, 
    output logic incr_reqs_cnt,
    output logic set_ongoing_atomic,
    output logic incr_flush_way,
    output logic clr_evict_stall, 
    output logic set_fwd_in_stalled,
    output logic clr_fwd_stall_ended,
    output logic set_set_conflict_fsm,
    output logic clr_set_conflict_fsm,
    output logic set_cpu_req_conflict,
    output logic clr_ongoing_atomic,
    output logic fill_reqs_flush, 
    output logic set_evict_stall, 
    output logic [2:0] reqs_op_code,
    output logic[`REQS_BITS-1:0]  reqs_atomic_i,
    output state_t wr_data_state,
    output unstable_state_t state_wr_data_req,
    output line_t wr_data_line, 
    output line_t line_wr_data_req,
    output hprot_t wr_data_hprot, 
    output hprot_t hprot_wr_data_req, 
    output l2_tag_t wr_data_tag, 
    output l2_tag_t tag_estall_wr_data_req, 
    output l2_tag_t tag_wr_data_req,
    output invack_cnt_calc_t invack_cnt_wr_data_req,
    output hsize_t hsize_wr_data_req,
    output word_t word_wr_data_req,
    output cpu_msg_t cpu_msg_wr_data_req, 
    output l2_set_t set_in,
    output l2_way_t way, 
    output l2_way_t way_wr_data_req, 
    output l2_way_t wr_data_evict_way,
    output line_addr_t l2_inval_o,
    output word_t word_in, 
    output word_offset_t w_off_in, 
    output byte_offset_t b_off_in, 
    output hsize_t hsize_in, 
    output line_t line_in,
    
    addr_breakdown_t.out addr_br_reqs,
    l2_rd_rsp_t.out l2_rd_rsp_o, 
    l2_rsp_out_t.out l2_rsp_out_o, 
    l2_req_out_t.out l2_req_out_o 
 
`ifdef STATS_ENABLE
    , input logic l2_stats_ready_int, 
    output logic l2_stats_o,
    output logic l2_stats_valid_int
`endif
   );

    localparam RESET = 5'b00000; 
    localparam DECODE = 5'b00001; 
    localparam RSP_LOOKUP = 5'b00010;
    localparam RSP_E_DATA_ISD = 5'b00011;
    localparam RSP_DATA_XMAD = 5'b00100;
    localparam RSP_DATA_XMADW = 5'b00101;
    localparam RSP_INVACK = 5'b00110; 
    localparam FWD_REQS_LOOKUP = 5'b00111;
    localparam FWD_TAG_LOOKUP = 5'b01000;
    localparam FWD_PUTACK = 5'b01001;
    localparam FWD_STALL = 5'b01010; 
    localparam FWD_HIT = 5'b01011; 
    localparam FWD_HIT_2 = 5'b01100;
    localparam FWD_NO_HIT = 5'b01101; 
    localparam FWD_NO_HIT_2 = 5'b01110; 
    localparam ONGOING_FLUSH_LOOKUP = 5'b01111; 
    localparam ONGOING_FLUSH_PROCESS = 5'b10000;
    localparam CPU_REQ_REQS_LOOKUP = 5'b10001;
    localparam CPU_REQ_ATOMIC_OVERRIDE = 5'b10010;
    localparam CPU_REQ_ATOMIC_CONTINUE_READ = 5'b10011;
    localparam CPU_REQ_ATOMIC_CONTINUE_WRITE = 5'b10100;
    localparam CPU_REQ_SET_CONFLICT = 5'b10101; 
    localparam CPU_REQ_TAG_LOOKUP = 5'b10110;
    localparam CPU_REQ_READ_READ_ATOMIC_EM  =  5'b10111;
    localparam CPU_REQ_READ_ATOMIC_WRITE_S = 5'b11000; 
    localparam CPU_REQ_WRITE_EM = 5'b11001; 
    localparam CPU_REQ_EMPTY_WAY = 5'b11010; 
    localparam CPU_REQ_EVICT = 5'b11011; 

    logic [4:0] state, next_state;
    always_ff @(posedge clk or negedge rst) begin 
        if (!rst) begin 
            state <= RESET; 
        end else begin 
            state <= next_state; 
        end 
    end

    logic rst_en;
    assign rst_en = (state == RESET); 
    assign decode_en = (state == DECODE); 

    l2_set_t rst_set; 
    always_ff @(posedge clk or negedge rst) begin 
        if (!rst) begin 
            rst_set <= 0; 
        end else if (rst_en) begin 
            rst_set <= rst_set + 1; 
        end
    end 
    
    logic update_atomic;
    line_addr_t atomic_line_addr;
    always_ff @(posedge clk or negedge rst) begin 
        if (!rst) begin 
            atomic_line_addr <= 0; 
        end else if (update_atomic) begin 
            atomic_line_addr <= addr_br.line_addr; 
        end
    end
    
    always_ff @(posedge clk or negedge rst) begin 
        if (!rst) begin 
            reqs_atomic_i <= 0; 
        end else if (update_atomic) begin 
            reqs_atomic_i <= reqs_i; 
        end
    end

    logic[1:0] ready_bits; 
    always_ff @(posedge clk or negedge rst) begin 
        if (!rst) begin 
            ready_bits <= 0; 
        end else if (state == DECODE) begin 
            ready_bits <= 0; 
        end else if ((state == CPU_REQ_EVICT || state == FWD_NO_HIT || state == FWD_HIT) && l2_inval_ready_int) begin 
            ready_bits[0] <= 1'b1; 
        end else if ((state == CPU_REQ_EVICT && l2_req_out_ready_int) || ((state == FWD_NO_HIT || state == FWD_HIT) && l2_rsp_out_ready_int)) begin 
            ready_bits[1] <= 1'b1; 
        end
    end

    always_comb begin
        next_state = state
        case(state)
            RESET : begin
                if (rst_set == `L2_SETS - 1) begin 
                    next_state = DECODE;
                end
            end
            DECODE : begin
                if(do_fence) begin
                    next_state = DO_FENCE;
                end else if(do_flush) begin
                    next_state = DO_FLUSH;
                end else if(do_flush_once) begin
                    next_state = DECODE;
                end else if(do_rsp) begin
                    next_state = RSP_LOOKUP;
                end else if(do_fwd) begin
                    next_state = FWD_REQS_LOOKUP;
                end else if(do_cpu_req) begin
                    next_state = CPU_REQ_REQS_LOOKUP;
                end
            end
            DO_FENCE : begin
                next_state = DECODE;
            end
            DO_FLUSH : begin
                next_state = DECODE;
            end
            RSP_LOOKUP : begin 
                case(rsp_in.coh_msg) 
                    `RSP_O : begin
                        next_state = RSP_O;
                    end
                    `RSP_Odata : begin
                        next_state = RSP_Odata;
                    end
                    `RSP_WTdata: begin
                        next_state = RSP_WTdata;
                    end
                    `RSP_V : begin
                        next_state = RSP_V;
                    end
                    `RSP_NACK : begin
                        case(reqs[reqs_hit_i].state)
                            `SPX_XRV : begin
                                next_state = SPX_XRV;
                            end
                            `SPX_IV_DCS : begin
                                next_state = SPX_IV_DCS;
                            end
                            `SPX_IV : begin
                                next_state = SPX_IV;
                            end
                            default: begin
                                next_state = DECODE;
                            end
                        endcase
                    end
                    `RSP_S : begin
                        next_state = RSP_S;
                    end
                    `RSP_WB_ACK : begin
                        next_state = RSP_WB_ACK;
                    end
                    default : begin
                        next_state = DECODE;
                    end
                endcase
            end
            RSP_O : begin
                next_state = DECODE;
            end
            RSP_Odata : begin 
                next_state = DECODE;
            end
            RSP_WTdata : begin
                next_state = DECODE;
            end
            RSP_V : begin
                next_state = DECODE;
            end
            SPX_XRV : begin
                next_state = DECODE;
            end
            SPX_IV_DCS : begin
                next_state = DECODE;
            end 
            SPX_IV : begin
                next_state = DECODE ;
            end
            RSP_S : begin
                next_state = DECODE;
            end
            RSP_WB_ACK : begin
                next_state = DECODE ;
            end
            FWD_TAG_LOOKUP : begin
                next_state = FWD_REQS_LOOKUP;
            FWD_REQS_LOOKUP: begin
                if(fwd_stall) begin
                    next_state = FWD_STALL
                end else if(fwd_not_stall) begin
                    next_state = FWD_NOT_STALL
            FWD_STALL : begin 
                next_state = DECODE
            end
            FWD_NOT_STALL : begin
                next_state = DECODE
            end
            CPU_REQ_REQS_LOOKUP : begin 
                if (ongoing_atomic) begin 
                    if (atomic_line_addr != addr_br.line_addr) begin 
                        next_state = CPU_REQ_ATOMIC_OVERRIDE;
                    end else begin 
                        if (l2_cpu_req.cpu_msg == `READ || l2_cpu_req.cpu_msg == `READ_ATOMIC) begin 
                            next_state = CPU_REQ_ATOMIC_CONTINUE_READ;
                        end else begin 
                            next_state = CPU_REQ_ATOMIC_CONTINUE_WRITE;
                        end
                    end    
                end else if ((set_conflict | set_set_conflict_reqs) & !clr_set_conflict_reqs) begin 
                    next_state = CPU_REQ_SET_CONFLICT; 
                end else begin 
                    next_state = CPU_REQ_TAG_LOOKUP;
                end
            end
            CPU_REQ_ATOMIC_OVERRIDE : begin 
                next_state = DECODE;
            end 
            CPU_REQ_ATOMIC_CONTINUE_READ : begin 
                if (l2_rd_rsp_ready_int) begin 
                    next_state = DECODE;
                end
            end
            CPU_REQ_ATOMIC_CONTINUE_WRITE : begin 
                next_state = DECODE;
            end
            CPU_REQ_SET_CONFLICT : begin 
                next_state = DECODE; 
            end
            CPU_REQ_TAG_LOOKUP : begin 
                if (tag_hit_next) begin 
                    if (l2_cpu_req.cpu_msg == `READ || (l2_cpu_req.cpu_msg == `READ_ATOMIC 
                            && (states_buf[way_hit_next] == `EXCLUSIVE || states_buf[way_hit_next] == `MODIFIED)))  begin
                        next_state = CPU_REQ_READ_READ_ATOMIC_EM; 
                   end else if ((l2_cpu_req.cpu_msg == `READ_ATOMIC && states_buf[way_hit_next] == `SHARED) 
                            || (l2_cpu_req.cpu_msg == `WRITE && states_buf[way_hit_next] == `SHARED)) begin 
                        next_state = CPU_REQ_READ_ATOMIC_WRITE_S;
                    end else if (l2_cpu_req.cpu_msg == `WRITE && (states_buf[way_hit_next] == `EXCLUSIVE 
                            || states_buf[way_hit_next] == `MODIFIED)) begin  
                        next_state = CPU_REQ_WRITE_EM; 
                    end else begin 
                        next_state = DECODE; 
                    end
                end else if (empty_way_found_next) begin 
                    next_state = CPU_REQ_EMPTY_WAY;
                end else begin 
                    next_state = CPU_REQ_EVICT;
                end
            end
            CPU_REQ_READ_READ_ATOMIC_EM : begin 
                if (l2_rd_rsp_ready_int) begin 
                    next_state = DECODE;
                end
            end
            CPU_REQ_READ_ATOMIC_WRITE_S : begin 
                if (l2_req_out_ready_int) begin 
                    next_state = DECODE;
                end
            end
            CPU_REQ_WRITE_EM : begin 
                next_state = DECODE; 
            end
            CPU_REQ_EMPTY_WAY : begin 
                if (l2_req_out_ready_int) begin 
                    next_state = DECODE; 
                end
            end
            CPU_REQ_EVICT : begin 
                if (l2_inval_ready_int && l2_req_out_ready_int) begin 
                    next_state = DECODE; 
                end else if (ready_bits[0] && l2_req_out_ready_int) begin 
                    next_state = DECODE;
                end else if (l2_inval_ready_int && ready_bits[1]) begin 
                    next_state = DECODE;
                end
            end
        endcase
    end
addr_t addr_tmp;
    line_addr_t line_addr_tmp;
    unstable_state_t state_tmp;
    coh_msg_t coh_msg_tmp;

    always_comb begin 
        wr_rst = 1'b0; 
        wr_data_state = 0; 
        reqs_op_code = `L2_REQS_IDLE; 
        lookup_en = 1'b0; 
        lookup_mode = 1'b0; 
        wr_req_state = 1'b0;
        wr_req_state_atomic = 1'b0; 
        wr_req_line = 1'b0;
        wr_req_invack_cnt = 0;
        wr_req_tag = 1'b0; 
        wr_en_put_reqs = 1'b0;
        wr_en_state = 1'b0; 
        wr_en_line = 1'b0; 
        wr_en_evict_way = 1'b0; 
        set_in = 0; 
        way = 0; 
        way_wr_data_req = 0;
        wr_data_tag = 0;
        wr_data_hprot = 0; 
        wr_data_line = 0; 
        wr_data_state = 0; 
        wr_data_evict_way = 0; 
        incr_reqs_cnt = 1'b0;
        set_ongoing_atomic = 1'b0; 
        rd_mem_en = 1'b0; 
        incr_flush_way = 1'b0;
        addr_tmp = 0; 
        line_addr_tmp = 0; 
        state_tmp = 0; 
        coh_msg_tmp = 0; 
        fill_reqs = 1'b0;
        fill_reqs_flush = 1'b0; 
            
        set_fwd_in_stalled = 1'b0; 
        clr_fwd_stall_ended = 1'b0;
        set_set_conflict_fsm = 1'b0;
        clr_set_conflict_fsm = 1'b0; 
        set_cpu_req_conflict = 1'b0; 
        clr_ongoing_atomic  = 1'b0; 
        update_atomic = 1'b0;
        clr_evict_stall = 1'b0;
        set_evict_stall = 1'b0;
        put_reqs_atomic = 1'b0; 

        l2_rd_rsp_o.line = 0; 
        l2_rd_rsp_valid_int = 1'b0;
        l2_inval_o = 0; 
        l2_inval_valid_int = 1'b0; 

        addr_br_reqs.line = 0;
        addr_br_reqs.line_addr = 0;
        addr_br_reqs.word = 0;
        addr_br_reqs.tag = 0;
        addr_br_reqs.set = 0;
        addr_br_reqs.w_off = 0;
        addr_br_reqs.b_off = 0; 
        
        cpu_msg_wr_data_req = 0;
        tag_estall_wr_data_req = 0;
        invack_cnt_wr_data_req = 0; 
        hsize_wr_data_req = 0;
        state_wr_data_req = 0;
        hprot_wr_data_req = 0; 
        word_wr_data_req = 0;
        line_wr_data_req = 0;
        tag_wr_data_req = 0; 

        l2_req_out_valid_int = 1'b0;
        l2_req_out_o.coh_msg = 0;
        l2_req_out_o.hprot = 0;
        l2_req_out_o.addr = 0; 
        l2_req_out_o.line = 0;
                   
        l2_rsp_out_valid_int = 1'b0;
        l2_rsp_out_o.coh_msg = 0; 
        l2_rsp_out_o.req_id = 0; 
        l2_rsp_out_o.to_req = 1'b0; 
        l2_rsp_out_o.addr = 0; 
        l2_rsp_out_o.line = 0; 

        word_in = 0; 
        w_off_in = 0; 
        b_off_in = 0; 
        hsize_in = 0; 
        line_in = 0;

    `ifdef STATS_ENABLE
        l2_stats_o = 1'b0; 
        l2_stats_valid_int = 1'b0; 
    `endif
        case (state)
            RESET : begin 
                wr_rst = 1'b1;
                wr_data_state = `INVALID;
                set_in = rst_set; 
            end
            DECODE : begin 
                if (do_ongoing_flush_next) begin 
                    if (incr_flush_set) begin 
                        set_in = flush_set + 1;
                    end else begin 
                        set_in = flush_set;
                    end
                end else if (do_fwd_next) begin 
                    set_in = line_br_next.set;
                end else if (do_cpu_req_next) begin 
                    set_in = addr_br_next.set;
                end
                
            end
            RSP_LOOKUP : begin
                reqs_op_code = `L2_REQS_LOOKUP;
            end
            RSP_O : begin
                if(reqs_hit) begin 
                    reqs[reqs_hit_i].word_mask = (reqs[reqs_hit_i].word_mask) && (~(rsp_in.word_mask));
                    if(reqs[reqs_hit_i].word_mask == 0) begin
                        reqs[reqs_hit_i].state = SPX_I;
                        reqs_cnt += 1 ;
                    end
                    if(fwd_stall && reqs_fwd_stall_i == reqs_hit_i) begin
                        fwd_stall_ended = 1'b1 ;
            
                    end
                end
            end
            RSP_Odata : begin
                for(int i=0; i< WORDS_PER_LINE; i++) begin
                    if(rsp_in.word_mask & (1 << i)) begin
                        reqs[reqs_hit_i].line.range(((i+1) * BITS_PER_WORD)-1,i * BITS_PER_WORD) = rsp_in.line.range(((i+1) * BITS_PER_WORD)- 1, i * BITS_PER_WORD);
                    end
                end
                if(reqs[reqs_hit_i].hsize < BYTE_BITS && reqs[reqs_hit_i].state != SPX_AMO) begin
                   line_in = reqs[reqs_hit_i].line;
                   word_in = cpu_req_conflict.word;
                   b_off_in = reqs[reqs_hit_i].b_off;
                   w_off_in = reqs[reqs_hit_it].w_off;
                   hsize_in = reqs[reqs_hit_i].hsize;
                end
                reqs[reqs_hit_i].word_mask &= ~(rsp_in.word_mask)
                if(reqs[reqs_hit_i].word_mask == 0) begin
                    if(reqs[reqs_hit_i].state == SPX_AMO) begin
                        l2_rd_rsp_valid_int = 1'b1;
                        l2_rd_rsp_o.line = reqs[reqs_hit_i].line;
                        if(reqs[reqs_hit_i].cpu_msg != READ_ATOMIC) begin
                            line_in = reqs[reqs_hit_i].line;
                            word_in = cpu_rec_conflict.word;
                            b_off_in = reqs[reqs_it_i].b_off;
                            w_off_in = reqs[reqs_it_i].w_off;
                            hsize_in = reqs[reqs_i_next].hsize;

                            
                        end
                        set_conflict = 1'b0
                    end
                if(reqs[reqs_hit_i].hsize < BYTE_BITS && reqs[reqs_hit_i].state != SPX_AMO) begin
                    set_conflict = 1'b0;
                if(reqs[reqs_hit_i].state = SPX_IV || reqs[reqs_hit_i].cpu_msg == READ) begin
                    12_rd_rsp_valid_int = 1'b1;
                    12_rd_rsp_o.line = reqs[reqs_hit_i].line;
                reqs[reqs_hit_i].state = SPX_I;
                reqs_cnt += 1;
                wr_en_put_reqs = 1'b1;
                set_in = line_br.set ;
                way = reqs[reqs_hit_i];
                wr_data_tag = line_br.tag;
                wr_data_line = reqs[reqs_hit_i].line;
                wr_data_hprot = reqs[reqs_hit_i].hprot;
                wr_data_state = SPX_XRV;
                incr_reqs_cnt = 1'b1;
            end
            RSP_WTdata : begin 
                line_t line = 0 
                for (int i = 0; I<WORDS_PER_LINE; i++) begin
                    if(rsp_in.word_mask & 1 << i) begin
                        line.range(BITS_PER_WORD,0) = rsp_in.line.range((i+1) * BITS_PER_WORD - 1, i * BITS_PER_WORD);
                        break;
                    end
                end
                l2_rd_rsp_valid_int = 1'b1;
                12_rd_rsp_o.line = line;
                reqs[reqs_hit_i].state = SPX_I;
            end
            RSP_V : begin 
                for(int i=0; i<WORDS_PER_LINE;i++) begin
                    if(rsp_in.word_mask & (1 << i)) begin
                        reqs[reqs_hit_i].line.range((i+1) * BITS_PER_WORD-1, i * BITS_PER_WORD) = rsp_in.line.range((i+1) * BITS_PER_WORD-1, i * BITS_PER_WORD);
                        reqs[reqs_hit_i].word_mask |= 1 << i;
                    end
                end
                if(reqs[reqs_hit_i].word_mask == WORD_MASK_ALL) begin
                    l2_rd_rsp_valid_int = 1'b1;
                    12_rd_rsp_o.line = reqs[reqs_hit_i].line;
                    wr_en_put_reqs = 1'b1;
                    set_in = line_br.set ;
                    way = reqs[reqs_hit_i];
                    wr_data_tag = line_br.tag;
                    wr_data_line = reqs[reqs_hit_i].line;
                    wr_data_hprot = reqs[reqs_hit_i].hprot;
                    wr_data_state = current_valid_state;
                    incr_reqs_cnt = 1'b1;
                end
            end

            SPX_XRV : begin
                l2_req_out_valid_int = 1'b1;
                l2_req_out_o.hprot = reqs[reqs_hit_i].hprot;
                l2_req_o.coh_msg = REQ_V;
                l2_req_o.addr = (reqs[reqs_hit_i].tag) << `L2_SET_BITS | reqs[reqs_hit_i].set;
                l2_req_o.line = reqs[reqs_hit_i].line;
                l2_req_o.word_mask = rsp_in.word_mask;
            end
            SPX_IV_DCS : begin 
                l2_req_out_valid_int = 1'b1;
                l2_req_out_o.hprot = reqs[reqs_hit_i].hprot;
                l2_req_o.coh_msg = REQ_V;
                l2_req_o.addr = (reqs[reqs_hit_i].tag) << `L2_SET_BITS | reqs[reqs_hit_i].set;
                l2_req_o.line = 1'b0
                l2_req_o.word_mask = ~rsp_in.word_mask
                reqs[reqs_hit_i].state = SPX_IV;
            end
            SPX_IV : begin
                reqs[reqs_hit_i].retry++ ; 
                coh_msg_t retry_msg;
                if(reqs[reqs_hit_i].retry < MAX_RETRY) begin 
                    retry_msg = REQ_V ;
                end else begin 
                    retry_msg = REQ_Odata ;
                end
                l2_req_out_valid_int = 1'b1;
                l2_req_out_o.hprot = reqs[reqs_hit_i].hprot;
                l2_req_o.coh_msg = retry_msg;
                l2_req_o.addr = (reqs[reqs_hit_i].tag) << `L2_SET_BITS | reqs[reqs_hit_i].set;
                l2_req_o.line = 1'b0;
                l2_req_o.word_mask = ~rsp_in.word_mask;
                reqs_word_mask_in[reqs_hit_i] = ~reqs[reqs_hit_i].word_mask;
            end
            RSP_S : begin
                if(reqs[reqs_hit_i].state == SPX_IS) begin 
                    for(int i = 0; i<WORDS_PER_LINE; i++) begin
                        if(rsp_in.word_mask & (1 << i)) begin 
                          reqs[reqs_hit_i].line.range((i + 1) * BITS_PER_WORD - 1, i * BITS_PER_WORD) = rsp_in.line.range((i + 1) * BITS_PER_WORD - 1, i * BITS_PER_WORD);
                          reqs[reqs_hit_i].word_mask |= 1 << i;
                        end
                    end
                    if(reqs[reqs_hit_i].word_mask == WORD_MASK_ALL) begin
                        l2_rd_rsp_valid_int = 1'b1;
                        12_rd_rsp_o.line = reqs[reqs_hit_i].line;
                        reqs[reqs_hit_i] = SPX_I;
                        set_in = line_br.set ;
                        way = reqs[reqs_hit_i];
                        wr_data_tag = line_br.tag;
                        wr_data_line = reqs[reqs_hit_i].line;
                        wr_data_hprot = reqs[reqs_hit_i].hprot;
                        wr_data_state = SPX_S;
                        incr_reqs_cnt = 1'b1;
                    end
                end else if(reqs[reqs_hit_i].state = SPX_II) begin
                    for(int i = 0; i<WORDS_PER_LINE; i++) begin
                        if(rsp_in.word_mask & (1 << i)) begin 
                          reqs[reqs_hit_i].line.range((i + 1) * BITS_PER_WORD - 1, i * BITS_PER_WORD) = rsp_in.line.range((i + 1) * BITS_PER_WORD - 1, i * BITS_PER_WORD);
                          reqs[reqs_hit_i].word_mask |= 1 << i;
                        end
                    end
                    if(reqs[reqs_hit_i].word_mask == WORD_MASK_ALL) begin
                        l2_rd_rsp_valid_int = 1'b1;
                        12_rd_rsp_o.line = reqs[reqs_hit_i].line;
                        reqs[reqs_hit_i] = SPX_I;
                        set_in = line_br.set ;
                        way = reqs[reqs_hit_i];
                        wr_data_tag = line_br.tag;
                        wr_data_line = reqs[reqs_hit_i].line;
                        wr_data_hprot = reqs[reqs_hit_i].hprot;
                        wr_data_state = SPX_I;
                        incr_reqs_cnt = 1'b1;
                    end
                end
            end
            RSP_WB_ACK : begin
                if(reqs[reqs_hit_i].state == SPX_RI || reqs[reqs_hit_i].state == SPX_II) begin
                    reqs[reqs_hit_i].state = SPX_I;
                    incr_reqs_cnt = 1'b1;
                end
            end
            FWD_TAG_LOOKUP : begin 
                lookup_en = 1'b1;
                lookup_mode = `L2_LOOKUP_FWD
            end
            FWD_STALL : begin 
                fwd_in_stalled = 1'b1
            end
            FWD_IN_STALL : begin 
                if(fwd_in.coh_msg == FWD_INV_SPDX && !ready_bits[1])begin
                    if(reqs[reqs_fwd_stall_i.state = SPX_IS]) begin 
                        reqs[reqs_fwd_stall_i].state = SPX_II
                    end
                        l2_rsp_out_valid_int = 1'b1;
                        l2_rsp_out_o.coh_msg = `RSP_INV_ACK_SPDX; 
                        l2_rsp_out_o.req_id = 1'b0; 
                        l2_rsp_out_o.to_req = 1'b0; 
                        l2_rsp_out_o.addr = l2_fwd_in.addr;
                        l2_rsp_out_o.line = 0;
                        l2_rsp_out_o.word_mask = WORD_MASK_ALL
                    if(!ready_bits[0]) begin
                        l2_inval_valid_int = 1'b1;
                        l2_inval_data = DATA;
                        l2_inval_o = l2_fwd_in_addr ;
                    end
                end else if(fwd_in.coh_msg == FWD_REQ_O) begin 
                    if(reqs[reqs_fwd_stall_i].state = SPX_RI)begin
                        l2_rsp_out_valid_int = 1'b1;
                        l2_rsp_out_o.coh_msg = `RSP_O; 
                        l2_rsp_out_o.req_id = l2_fwd_in.req_id; 
                        l2_rsp_out_o.to_req = 1'b1; 
                        l2_rsp_out_o.addr = l2_fwd_in.addr;
                        l2_rsp_out_o.line = 0;
                        l2_rsp_out_o.word_mask = fwd_in.word_mask;
                    end else if(reqs[reqs_fwd_stall_i].state == SPR_XR || reqs[reqs_fwd_stall_i].state == SPX_AMO ) begin
                        word_mask_t rsp_mask = 0
                        if(tag_hit) begin
                            for (int i=0; i<WORDS_PER_LINE; i++) begin
                                if((fwd_in.word_mask & (1 << i)) && state_buf[reqs[reqs_fwd_stall_i].way][i] == SPX_AMO) begin
                                    rsp_mask |= 1<<i ;
                                    state_buf[reqs[reqs_fwd_stall_i].way][i] = SPX_I
                                end
                            end
                            if(rsp_mask) begin 
                                l2_rsp_out_valid_int = 1'b1;
                                l2_rsp_out_o.coh_msg = `RSP_O; 
                                l2_rsp_out_o.req_id = l2_fwd_in.req_id; 
                                l2_rsp_out_o.to_req = 1'b1; 
                                l2_rsp_out_o.addr = l2_fwd_in.addr;
                                l2_rsp_out_o.line = line_buf[reqs[reqs_fwd_stall_i].way];
                                l2_rsp_out_o.word_mask = rsp_mask;
                            end
                        end
                    end
                    if(!ready_bits[0]) begin 
                        l2_inval_valid_int = 1'b1;
                        l2_inval_data = DATA;
                        l2_inval_o = l2_fwd_in_addr ;
                    end
                end else if(fwd_in.coh_msg = FWD_REQ_Odata) begin
                    if(reqs[reqs_fwd_stall_i].state == SPX_RI) begin
                        l2_rsp_out_valid_int = 1'b1;
                        l2_rsp_out_o.coh_msg = `RSP_Odata; 
                        l2_rsp_out_o.req_id = l2_fwd_in.req_id; 
                        l2_rsp_out_o.to_req = 1'b1; 
                        l2_rsp_out_o.addr = l2_fwd_in.addr;
                        l2_rsp_out_o.line =  reqs[reqs_fwd_stall_i].line;
                        l2_rsp_out_o.word_mask = fwd_in.word_mask; 
                    end else if(reqs[reqs_fwd_stall_i].state == SPX_XR || reqs[reqs_fwd_stall_i].state == SPX_AMO) begin
                        word_mask_t rsp_mask = 0;
                        if(tag_hit) begin
                            for(int i=0; i<WORDS_PER_LINE; i++) begin
                                if ((fwd_in.word_mask & (1 << i)) && state_buf[reqs[reqs_fwd_stall_i].way][i] == SPX_R) begin
                                    rsp_mask |= 1 << i;
                                    state_buf[reqs[rews_fwd_stall_i].way][i] = SPX_I;
                                end
                            end
                            if(rsp_mask) begin 
                                l2_rsp_out_valid_int = 1'b1;
                                l2_rsp_out_o.coh_msg = `RSP_Odata; 
                                l2_rsp_out_o.req_id = l2_fwd_in.req_id; 
                                l2_rsp_out_o.to_req = 1'b1; 
                                l2_rsp_out_o.addr = l2_fwd_in.addr;
                                l2_rsp_out_o.line =   line_buf[reqs[reqs_fwd_stall_i].way];
                                l2_rsp_out_o.word_mask = rsp_mask;
                            end
                        end
                    end
                    if(!ready_bits[0]) begin
                        l2_inval_valid_int = 1'b1;
                        l2_inval_data = DATA;
                        l2_inval_o = l2_fwd_in_addr ;
                    end
                end else if(fwd_in.coh_msg == FWD_REQ_V) begin
                    word_mask_t nack_mask = 0;
                    word_mask_t ack_mask = 0;
                    for(int i=0; i < WORDS_PER_LINE; i++) begin
                        if(fwd_in.word_mask & (1 << i)) begin
                           if((reqs[reqs_fwd_stall_i].word_mask & (1 << i)) && reqs[reqs_fwd_stall_i].state == SPX_RI) begin
                            ack_mask |= 1 << i;
                           end else begin
                            nack_mask |= 1 << i;
                           end
                        end
                    end
                    if(ack_mask) begin
                        l2_rsp_out_valid_int = 1'b1;
                        l2_rsp_out_o.coh_msg = `RSP_V; 
                        l2_rsp_out_o.req_id = l2_fwd_in.req_id; 
                        l2_rsp_out_o.to_req = 1'b1; 
                        l2_rsp_out_o.addr = l2_fwd_in.addr;
                        l2_rsp_out_o.line =   reqs[reqs_fwd_stall_i].line;
                        l2_rsp_out_o.word_mask = ack_mask;
                    end
                    if(nack_mask and !ready_bits[1]) begin
                        l2_rsp_out_valid_int = 1'b1;
                        l2_rsp_out_o.coh_msg = `RSP_V; 
                        l2_rsp_out_o.req_id = l2_fwd_in.req_id; 
                        l2_rsp_out_o.to_req = 1'b1; 
                        l2_rsp_out_o.addr = l2_fwd_in.addr;
                        l2_rsp_out_o.line = 1'b0 ;
                        l2_rsp_out_o.word_mask = nack_mask;
                    end
                end else if(fwd_om.word_mask == FWD_RVK_O) begin
                    if(reqs[reqs_fwd_stall_i].state == SPX_RI) begin
                        word_mask_t rsp_mask =  reqs[reqs_fwd_stall_i].word_mask & fwd_in.word_mask;
                        reqs[reqs_fwd_stall_i].word_mask &= ~rsp_mask;
                        if(rsp_mask) begin
                            l2_rsp_out_valid_int = 1'b1;
                            l2_rsp_out_o.coh_msg = `RSP_RVK; 
                            l2_rsp_out_o.req_id = 1'b0 
                            l2_rsp_out_o.to_req = 1'b0; 
                            l2_rsp_out_o.addr = l2_fwd_in.addr;
                            l2_rsp_out_o.line =   reqs[reqs_fwd_stall_i].line;
                            l2_rsp_out_o.word_mask = rsp_mask;
                        end
                        if (!reqs[reqs_fwd_stall_i].word_mask) begin
                            reqs[reqs_fwd_stall_i].state = SPX_II;
                        end
                    end
                    else if(reqs[reqs_fwd_stall_i].state == SPX_XR || reqs[reqs_fwd_stall_i].state == SPX_AMO) begin
                        word_mask_t rsp_mark = 0;
                        if(tag_hit) begin
                            for (int i=0 ; i < WORDS_PER_LINE ; i++) begin
                                if((fwd_in.word_mask & (1<<i)) &&state_buf[reqs[reqs_fwd_stall_i].way][i] == SPX_R ) begin
                                    rsp_mask |= 1 << i;
                                    state_buf[reqs[reqs_fwd_stall_i].way][i] = SPX_I;
                                end
                            end
                            if(rsp_mask) begin
                                l2_rsp_out_valid_int = 1'b1;
                                l2_rsp_out_o.coh_msg = `RSP_RVK_O; 
                                l2_rsp_out_o.req_id = l2_fwd_in.req_id
                                l2_rsp_out_o.to_req = 1'b0; 
                                l2_rsp_out_o.addr = l2_fwd_in.addr;
                                l2_rsp_out_o.line = line_buf[reqs[reqs_fwd_stall_i].way];
                                l2_rsp_out_o.word_mask = rsp_mask;
                            end
                        end
                    end
                    if(!ready_bits[0]) begin
                        l2_inval_valid_int = 1'b1;
                        l2_inval_data = DATA;
                        l2_inval_o = l2_fwd_in_addr ;
                    end
                end else if(fwd_in.coh_msg = FWD_REQ_S) begin
                    if(reqs[reqs_fwd_stall_i] == SPX_RI) begin
                        l2_rsp_out_valid_int = 1'b1;
                        l2_rsp_out_o.coh_msg = `RSP_S; 
                        l2_rsp_out_o.req_id = l2_fwd_in.req_id
                        l2_rsp_out_o.to_req = 1'b1; 
                        l2_rsp_out_o.addr = l2_fwd_in.addr;
                        l2_rsp_out_o.line = reqs[reqs_fwd_stall_i].line;
                        l2_rsp_out_o.word_mask = fwd_in.word_mask;
                        if(!ready_bits[1]) begin
                            l2_rsp_out_valid_int = 1'b1;
                            l2_rsp_out_o.coh_msg = `RSP_RVK_O 
                            l2_rsp_out_o.req_id = l2_fwd_in.req_id
                            l2_rsp_out_o.to_req = 1'b0; 
                            l2_rsp_out_o.addr = l2_fwd_in.addr;
                            l2_rsp_out_o.line = reqs[reqs_fwd_stall_i].line;
                            l2_rsp_out_o.word_mask = fwd_in.word_mask;
                        end
                        




                                


                    
                            
                            
                            
                           













                
            
               
             







                









 
            

                


             
                


