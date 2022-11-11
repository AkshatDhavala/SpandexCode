module fwd_fsm(
    input clk,
    input rst,
    input logic fwd_stall,
    input var logic [`REQS_BITS-1:0] reqs_i
    input logic reqs_fwd_stall_i

    l2_fwd_in_t l2_fwd_in,
    output logic l2_rsp_out_valid_int,
    l2_rsp_out_t.out l2_rsp_out_o
    `ifdef STATS_ENABLE
        input logic l2_stats_ready_int, 
        output logic l2_stats_o,
        output logic l2_stats_valid_int
    `endif
    );
    localparam RESET = 5'b00000;
    localparam DECODE = 5'b00001;
    localparam FWD_REQ_LOOKUP = 5'b00010;
    localparam FWD_STALL = 5'b00011;
    localparam FWD_IN_STALL = 5'b00100;
    localparam FWD_NOT_IN_STALL = 5'b00101;
    localparam FWD_INV_SPDX = 5'b00110;
    localparam RSP_INV_ACK_SPDX = 5'b00111;
    localparam SPX_IS = 5'b01000;
    localparam SPX_II = 5'b01001;
    localparam WORD_MASK_ALL = 5'b01010;
    localparam DATA = 5'b01011;
    localparam FWD_REQ_O = 5'b01100;
    localparam SPX_RI = 5'b01101;
    localparam RSP_O = 5'b01110;
    localparam SPX_XR = 5'b01111;
    localparam SPX_AMO = 5'b10000;
    localparam SPX_I = 5'b10001;
    localparam FWD_REQ_Odata = 5'b10010;
    localparam RSP_Odata  = 5'b10011;
    localparam SPX_R = 5'b10100;
    localparam FWD_REQ_V = 5'b10101;
    localparam RSP_V = 5'b10110;
    localparam FWD_RVK_O = 5'b10111;
    localparam RSP_RVK = 5'b11000;
    localparam RSP_RVK_O = 5'b11001;
    localparam FWD_REQ_S = 5'b11010;
    localparam RSP_S = 5'b11011;
    localparam FWD_WTfwd = 5'b11100;
    localparam RSP_NACK = 5'b11101;
    logic [4:0] state, next_state;
    always_ff @(posedge clk or negedge rst) begin 
        if (!rst) begin 
            state <= RESET; 
        end else begin 
            state <= next_state; 
        end 
    end
    l2_set_t rst_set; 
    always_ff @(posedge clk or negedge rst) begin 
        if (!rst) begin 
            rst_set <= 0; 
        end else if (rst_en) begin 
            rst_set <= rst_set + 1; 
        end
    end 
    logic[1:0] ready_bits; 
    always_ff @(posedge clk or negedge rst) begin
        if(!rst) begin
            ready_bits <= 0;
        end else if(state == DECODE) begin
            ready_bits <= 0;
        end else if(l2_inval_ready_int) begin
            ready_bits[0] <= 1;
        end else if(l2_rsp_out_valid_int) begin
            ready_bits[1] <= 1;
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
                if(fwd_stall) begin
                    next_state = FWD_STALL;
                end
                else begin
                    next_state = FWD_NOT_STALL;
                end
            end
            FWD_STALL : begin 
                next_state = DECODE;
            end 
            FWD_NOT_STALL : begin
                next_state - DECODE ;
            end
        endcase
    end
    always_comb begin
        l2_rsp_out_valid_int = 1'b0;
        l2_rsp_out_o.coh_msg = 0; 
        l2_rsp_out_o.req_id = 0; 
        l2_rsp_out_o.to_req = 1'b0; 
        l2_rsp_out_o.addr = 0; 
        l2_rsp_out_o.line = 0; 

        l2_inval_o = 0; 
        l2_inval_valid_int = 1'b0; 
        `ifdef STATS_ENABLE
        l2_stats_o = 1'b0; 
        l2_stats_valid_int = 1'b0; 
    `   `endif
        case(state) begin 
            RESET : begin
                
            end
            DECODE: begin
            
            end
            FWD_STALL : begin
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
                            l2_rsp_out_o.coh_msg = `RSP_RVK_O ;
                            l2_rsp_out_o.req_id = l2_fwd_in.req_id;
                            l2_rsp_out_o.to_req = 1'b0; 
                            l2_rsp_out_o.addr = l2_fwd_in.addr;
                            l2_rsp_out_o.line = reqs[reqs_fwd_stall_i].line;
                            l2_rsp_out_o.word_mask = fwd_in.word_mask;
                        end
                    end else if(reqs[reqs_fwd_stall_i].state = SPX_XR || reqs[reqs_fwd_stall_i].line == SPX_AMO) begin 
                        word_mask_t rsp_mask = 0;
                        if(tag_hit) begin
                            for(int i = 0; i < WORDS_PER_LINE ; i++) begin
                                if((fwd_in.word_mask & (1 << i)) && states_buf[reqs[reqs_fwd_stall_i].way][i] == SPX_R ) begin
                                rsp_mask |= 1 << i ; 
                                state_buf[reqs[reqs_fwd_stall_i].way][i] = SPX_I;
                                end
                            end
                            if(rsp_mask) begin
                                l2_rsp_out_valid_int = 1'b1;
                                l2_rsp_out_o.coh_msg = `RSP_S; 
                                l2_rsp_out_o.req_id = l2_fwd_in.req_id
                                l2_rsp_out_o.to_req = 1'b1; 
                                l2_rsp_out_o.addr = l2_fwd_in.addr;
                                l2_rsp_out_o.line = line_buf[reqs[reqs_fwd_stall_i].way];
                                l2_rsp_out_o.word_mask = rsp_mask;
                                if(!ready_bits[1]) begin 
                                    l2_rsp_out_valid_int = 1'b1;
                                    l2_rsp_out_o.coh_msg = `RSP_S; 
                                    l2_rsp_out_o.req_id = l2_fwd_in.req_id
                                    l2_rsp_out_o.to_req = 1'b0; 
                                    l2_rsp_out_o.addr = l2_fwd_in.addr;
                                    l2_rsp_out_o.line = line_buf[reqs[reqs_fwd_stall_i].way];
                                    l2_rsp_out_o.word_mask = rsp_mask; 
                                end
                            end
                        end
                    end
                    if(!ready_bits[0]) begin
                        l2_inval_valid_int = 1'b1;
                        l2_inval_data = DATA;
                        l2_inval_o = l2_fwd_in_addr ;
                    end
                end else if(fwd_in.coh_msg == FWD_WTfwd) begin 
                    word_mask_t nack_mask = 0;
                    for(int i = 0 ; i < WORDS_PER_LINE; i++) begin 
                        if(fwd_in.word_mask & (1<<i)) begin
                            if((reqs[reqs_fwd_stall_i].word_mask & (1 << i)) && reqs[reqs_fwd_stall_i].state = SPX_RI) begin
                                nack_mask |= 1 << i ;
                            end
                        end
                    end
                    if(nack_mask) begin 
                        if(!ready_bits[1]) begin 
                            l2_rsp_out_valid_int = 1'b1;
                            l2_rsp_out_o.coh_msg = `RSP_NACK; 
                            l2_rsp_out_o.req_id = l2_fwd_in.req_id
                            l2_rsp_out_o.to_req = 1'b0; 
                            l2_rsp_out_o.addr = l2_fwd_in.addr;
                            l2_rsp_out_o.line = line_buf[reqs[reqs_fwd_stall_i].way];
                            l2_rsp_out_o.word_mask = rsp_mask; 
                        end
                    end
                end
            end
            FWD_NOT_STALL : begin
                case(fwd_in.coh_msg) 
                    `FWD_INV_SPDX : begin 
                        if(tag_hit) begin
                            for (int i = 0; i<WORDS_PER_LINE ; i++) begin 
                                if ((fwd_in.word_mask & (1 << i)) && state_buf[way_hit][i] == SPX_S) begin
                                    state_buf[way_hit][i] = SPX_I;
                                end
                            end
                        end
                        l2_rsp_out_valid_int = 1'b1;
                        l2_rsp_out_o.coh_msg = `RSP_INV_ACK_SPDX; 
                        l2_rsp_out_o.req_id = 1'b0
                        l2_rsp_out_o.to_req = 1'b0; 
                        l2_rsp_out_o.addr = l2_fwd_in.addr;
                        l2_rsp_out_o.line = 0;
                        l2_rsp_out_o.word_mask = WORD_MASK_ALL;
                        if(!ready_bits[0]) begin
                            l2_inval_valid_int = 1'b1;
                            l2_inval_data = DATA;
                            l2_inval_o = l2_fwd_in_addr ;
                        end
                    end
                    `FWD_REQ_O : begin
                        word_mask_t ack_mask = 0;
                        if(tag_hit) begin
                            for (i = 0; i<WORDS_PER_LINE ; i++) begin 
                                if ((fwd_in.word_mask & (1 << i)) && state_buf[way_hit][i] == SPX_R) begin
                                    state_buff[way_hit][i] = SPX_I ;
                                    ack_mask |= 1 << i;
                                end
                            end
                            if(ack_mask) begin
                              l2_rsp_out_valid_int = 1'b1;
                              l2_rsp_out_o.coh_msg = `RSP_O; 
                              l2_rsp_out_o.req_id = fwd_in.req_id;
                              l2_rsp_out_o.to_req = 1'b1; 
                              l2_rsp_out_o.addr = l2_fwd_in.addr;
                              l2_rsp_out_o.line = 0;
                              l2_rsp_out_o.word_mask = ack_mask; 
                            end
                        end
                        if(!ready_bits[0]) begin
                            l2_inval_valid_int = 1'b1;
                            l2_inval_data = DATA;
                            l2_inval_o = l2_fwd_in_addr ; 
                        end
                    end
                    `FWD_REQ_Odata : begin 
                        word_mask_t ack_mask = 0;
                        if(tag_hit) begin
                            for(int i=0; i< WORDS_PER_LINE ; i++)begin
                                if ((fwd_in.word_mask & (1 << i)) && state_buf[way_hit][i] == SPX_R) begin
                                    state_buff[way_hit][i] = SPX_I;
                                    ack_mask |= 1 << i;
                                end
                            end
                            if(ack_mask) begin
                                l2_rsp_out_valid_int = 1'b1;
                                l2_rsp_out_o.coh_msg = `RSP_Odata; 
                                l2_rsp_out_o.req_id = fwd_in.req_id;
                                l2_rsp_out_o.to_req = 1'b1; 
                                l2_rsp_out_o.addr = l2_fwd_in.addr;
                                l2.line = line_buf[way_hit];
                                l2_rsp_out_o.word_mask = ack_mask; 
                            end
                        end
                        if(!ready_bits[0]) begin
                            l2_inval_valid_int = 1'b1;
                            l2_inval_data = DATA;
                            l2_inval_o = l2_fwd_in_addr ; 
                        end
                    end
                    `FWD_REQ_V : begin
                        word_mask_t nack_mask = 0; 
                        word_mask_t ack_mask = 0;
                        if(tag_hit) begin
                            for(int i=0; i < WORDS_PER_LINE ; i++) begin
                                if ((fwd_in.word_mask & (1 << i)) && state_buf[way_hit][i] == SPX_R) begin
                                    ack_mask |= 1 << i;
                                end
                                if ((fwd_in.word_mask & (1 << i)) && state_buf[way_hit][i] != SPX_R) begin
                                    nack_mask |= 1 << i;
                                end
                            end
                            if(ack_mask) begin
                                l2_rsp_out_valid_int = 1'b1;
                                l2_rsp_out_o.coh_msg = `RSP_V; 
                                l2_rsp_out_o.req_id = fwd_in.req_id;
                                l2_rsp_out_o.to_req = 1'b1; 
                                l2_rsp_out_o.addr = l2_fwd_in.addr;
                                l2.line = line_buf[way_hit];
                                l2_rsp_out_o.word_mask = ack_mask; 
                            end
                            if(nack_mask) begin 
                                if(!ready_bits[1]) begin
                                    l2_rsp_out_valid_int = 1'b1;
                                    l2_rsp_out_o.coh_msg = `RSP_NACK; 
                                    l2_rsp_out_o.req_id = fwd_in.req_id;
                                    l2_rsp_out_o.to_req = 1'b1; 
                                    l2_rsp_out_o.addr = l2_fwd_in.addr;
                                    l2_rsp_out_o.line = 0;
                                    l2_rsp_out_o.word_mask = ack_mask;
                                end
                            end
                        end
                        else begin 
                            l2_rsp_out_valid_int = 1'b1;
                            l2_rsp_out_o.coh_msg = `RSP_NACK; 
                            l2_rsp_out_o.req_id = fwd_in.req_id;
                            l2_rsp_out_o.to_req = 1'b1; 
                            l2_rsp_out_o.addr = l2_fwd_in.addr;
                            l2_rsp_out_o.line = 0;
                            l2_rsp_out_o.word_mask = fwd_in.word_mask;
                        end
                    end
                    `FWD_RVK_O : begin
                        word_mask_t rsp_mask = 0;
                        if(tag_hit) begin 
                            for (int i = 0; i < WORDS_PER_LINE ; i++) begin
                                if((fwd_in.word_mask & (1 << i) && state_buf[way_hit][i] == SPX_R)) begin
                                    rsp_mask |= 1 << i;
                                    state_buf[way_hit][i] = SPX_I;
                                end
                            end
                            if(rsp_mask) begin 
                                l2_rsp_out_valid_int = 1'b1;
                                l2_rsp_out_o.coh_msg = `RSP_RVK_O; 
                                l2_rsp_out_o.req_id = fwd_in.req_id;
                                l2_rsp_out_o.to_req = 1'b0; 
                                l2_rsp_out_o.addr = l2_fwd_in.addr;
                                l2_rsp_out_o.line = line_buf[way_hit];
                                l2_rsp_out_o.word_mask = rsp_mask;
                            end
                        end
                        if(!ready_bits[0]) begin 
                            l2_inval_valid_int = 1'b1;
                            l2_inval_data = DATA;
                            l2_inval_o = l2_fwd_in_addr ;
                        end
                    end
                    `FWD_REQ_S : begin 
                        word_mask_t rsp_mask = 0;
                        if(tag_hit) begin 
                            for (int i = 0; i < WORDS_PER_LINE ; i++) begin
                                if ((fwd_in.word_mask & (1 << i)) && state_buf[way_hit][i] == SPX_R) begin 
                                    rsp_mask |= 1 << i ;
                                    state_buff[way_hit][i] = SPX_I;
                                end
                            end
                            if(rsp_mask) begin
                                l2_rsp_out_valid_int = 1'b1;
                                l2_rsp_out_o.coh_msg = `RSP_S; 
                                l2_rsp_out_o.req_id = fwd_in.req_id;
                                l2_rsp_out_o.to_req = 1'b1; 
                                l2_rsp_out_o.addr = l2_fwd_in.addr;
                                l2_rsp_out_o.line = line_buf[way_hit];
                                l2_rsp_out_o.word_mask = rsp_mask;
                                if(!ready_bit[1]) begin
                                    l2_rsp_out_valid_int = 1'b1;
                                    l2_rsp_out_o.coh_msg = `RSP_RVK_O; 
                                    l2_rsp_out_o.req_id = fwd_in.req_id;
                                    l2_rsp_out_o.to_req = 1'b0; 
                                    l2_rsp_out_o.addr = l2_fwd_in.addr;
                                    l2_rsp_out_o.line = line_buf[way_hit];
                                    l2_rsp_out_o.word_mask = rsp_mask;
                                end
                            end
                        end
                        if(!ready_bit[0]) begin
                            l2_inval_valid_int = 1'b1;
                            l2_inval_data = DATA;
                            l2_inval_o = l2_fwd_in_addr ;
                        end
                    end
                    `FWD_WTfwd : begin
                        word_mask_t nack_mask = 0;
                        word_mask_t ack_mask = 0;
                        if(tag_hit) begin 
                            for(int i=0; i < WORDS_PER_LINE ; i++) begin 
                                if(fwd_in.word_mask & (1 << i)) begin 
                                    if(state_buf[way_hit][i] == SPX_R) begin 
                                        line_buf[way_hit].range(BITS_PER_WORD*(i+1)-1,BITS_PER_WORD*i) = fwd_in.line.range(BITS_PER_WORD*(i+1)-1,BITS_PER_WORD*i);
                                        ack_mask |= 1 << i;
                                    end
                                    else begin
                                        nack_mask |= 1 << i;
                                    end
                                end
                            end
                            lines.port1[0][base + way_hit] = line_buf[way_hit];
                            if(ack_mask) begin 
                                l2_rsp_out_valid_int = 1'b1;
                                l2_rsp_out_o.coh_msg = `RSP_O; 
                                l2_rsp_out_o.req_id = fwd_in.req_id;
                                l2_rsp_out_o.to_req = 1'b1; 
                                l2_rsp_out_o.addr = l2_fwd_in.addr;
                                l2_rsp_out_o.line = 0;
                                l2_rsp_out_o.word_mask = ack_mask;
                            end
                            if(nack_mask) begin
                                l2_rsp_out_valid_int = 1'b1;
                                l2_rsp_out_o.coh_msg = `RSP_NACK; 
                                l2_rsp_out_o.req_id = fwd_in.req_id;
                                l2_rsp_out_o.to_req = 1'b1; 
                                l2_rsp_out_o.addr = l2_fwd_in.addr;
                                l2_rsp_out_o.line = 0;
                                l2_rsp_out_o.word_mask = nack_mask;
                            end
                        end
                        else begin
                            l2_rsp_out_valid_int = 1'b1;
                            l2_rsp_out_o.coh_msg = `RSP_NACK; 
                            l2_rsp_out_o.req_id = fwd_in.req_id;
                            l2_rsp_out_o.to_req = 1'b1; 
                            l2_rsp_out_o.addr = l2_fwd_in.addr;
                            l2_rsp_out_o.line = 0;
                            l2_rsp_out_o.word_mask = nack_mask;
                        end
                    end
                endcase
            end
        endcase
    end
            






    `
    