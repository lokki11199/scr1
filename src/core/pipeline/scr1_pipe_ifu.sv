 /// Copyright by Syntacore LLC © 2016-2021. See LICENSE for details
/// @file       <scr1_pipe_ifu.sv>
/// @brief      Instruction Fetch Unit (IFU)
///

//------------------------------------------------------------------------------
 //
 // Functionality:
 // - Controls instruction fetching process:
 //   - Fetches instructions either from IMEM or from Program Buffer, supporting
 //     pending IMEM instructions handling
 //   - Handles new PC misalignment and constructs the correct instruction (supports
 //     RVI and RVC instructions)
 //   - Either stores instructions in the instruction queue or bypasses to the
 //     IDU if the corresponding option is used
 //   - Flushes instruction queue if requested
 //
 // Structure:
 // - Instruction queue
 // - IFU FSM
 // - IFU <-> IMEM i/f
 // - IFU <-> IDU i/f
 // - IFU <-> HDU i/f
 //
//------------------------------------------------------------------------------

`include "scr1_memif.svh"
`include "scr1_arch_description.svh"
`ifdef SCR1_DBG_EN
`include "scr1_hdu.svh"
`endif // SCR1_DBG_EN
module scr1_pipe_ifu
(
    // Control signals
    input   logic                                   rst_n,                      // IFU reset
    input   logic                                   clk,                        // IFU clock
    input   logic                                   pipe2ifu_stop_fetch_i,      // Stop instruction fetch

    // IFU <-> IMEM interface
    input   logic                                   imem2ifu_req_ack_i,         // Instruction memory request acknowledgement
    output  logic                                   ifu2imem_req_o,             // Instruction memory request
    output  type_scr1_mem_cmd_e                     ifu2imem_cmd_o,             // Instruction memory command (READ/WRITE)
    output  logic [`SCR1_IMEM_AWIDTH-1:0]           ifu2imem_addr_o,            // Instruction memory address
    input   logic [`SCR1_IMEM_DWIDTH-1:0]           imem2ifu_rdata_i,           // Instruction memory read data
    input   type_scr1_mem_resp_e                    imem2ifu_resp_i,            // Instruction memory response

    // IFU <-> EXU New PC interface
    input   logic                                   exu2ifu_pc_new_req_i,       // New PC request (jumps, branches, traps etc)
    input   logic [`SCR1_XLEN-1:0]                  exu2ifu_pc_new_i,           // New PC

`ifdef BPU
    input   logic                                   exu2ifu_b_type_i,           // B-type instr flag
    input   logic                                   exu2ifu_prev_prediction_i,  // Previous prediction
    input   logic [`SCR1_XLEN-1:0]                  exu2ifu_pc_prev_i,          // Previous PC
    input   logic                                   exu2ifu_btb_miss_i,         // BTB miss flag
    `ifdef SCR1_RVC_EXT
    input   logic                                   exu2ifu_rvi_flag_i,
    `endif
`endif

`ifdef SCR1_DBG_EN
    // IFU <-> HDU Program Buffer interface
    input   logic                                   hdu2ifu_pbuf_fetch_i,       // Fetch instructions provided by Program Buffer
    output  logic                                   ifu2hdu_pbuf_rdy_o,         // Program Buffer Instruction i/f ready
    input   logic                                   hdu2ifu_pbuf_vd_i,          // Program Buffer Instruction valid
    input   logic                                   hdu2ifu_pbuf_err_i,         // Program Buffer Instruction i/f error
    input   logic [SCR1_HDU_CORE_INSTR_WIDTH-1:0]   hdu2ifu_pbuf_instr_i,       // Program Buffer Instruction itself
`endif // SCR1_DBG_EN

`ifdef SCR1_CLKCTRL_EN
    output  logic                                   ifu2pipe_imem_txns_pnd_o,   // There are pending imem transactions
`endif // SCR1_CLKCTRL_EN

    // IFU <-> IDU interface
`ifdef BPU
    output  logic                                   ifu2idu_prediction_o,
    output  logic [`SCR1_XLEN-1:0]                  ifu2idu_predicted_pc_o,
`endif
    input   logic                                   idu2ifu_rdy_i,              // IDU ready for new data
    output  logic [`SCR1_IMEM_DWIDTH-1:0]           ifu2idu_instr_o,            // IFU instruction
    output  logic                                   ifu2idu_imem_err_o,         // Instruction access fault exception
    output  logic                                   ifu2idu_err_rvi_hi_o,       // 1 - imem fault when trying to fetch second half of an unaligned RVI instruction
    output  logic                                   ifu2idu_vd_o                // IFU request
);

//------------------------------------------------------------------------------
// Local parameters declaration
//------------------------------------------------------------------------------

localparam SCR1_IFU_Q_SIZE_WORD     = 2; //2 ����� ������
localparam SCR1_IFU_Q_SIZE_HALF     = SCR1_IFU_Q_SIZE_WORD * 2; //4 ���������
localparam SCR1_TXN_CNT_W           = 3;

localparam SCR1_IFU_QUEUE_ADR_W     = $clog2(SCR1_IFU_Q_SIZE_HALF); //2 ���� �� �������� ������
localparam SCR1_IFU_QUEUE_PTR_W     = SCR1_IFU_QUEUE_ADR_W + 1; //3 ���� �� �������� ���������

localparam SCR1_IFU_Q_FREE_H_W      = $clog2(SCR1_IFU_Q_SIZE_HALF + 1); //3
localparam SCR1_IFU_Q_FREE_W_W      = $clog2(SCR1_IFU_Q_SIZE_WORD + 1); //2

//------------------------------------------------------------------------------
// Local types declaration
//------------------------------------------------------------------------------

typedef enum logic {
    SCR1_IFU_FSM_IDLE,
    SCR1_IFU_FSM_FETCH
} type_scr1_ifu_fsm_e;

typedef enum logic[1:0] {
    SCR1_IFU_QUEUE_WR_NONE,      // No write to queue
`ifdef BPU
    SCR1_IFU_QUEUE_WR_LOW,
`endif    
    SCR1_IFU_QUEUE_WR_FULL,      // Write 32 rdata bits to queue
    SCR1_IFU_QUEUE_WR_HI         // Write 16 upper rdata bits to queue
} type_scr1_ifu_queue_wr_e;

typedef enum logic[1:0] {
    SCR1_IFU_QUEUE_RD_NONE,      // No queue read
    SCR1_IFU_QUEUE_RD_HWORD,     // Read halfword
    SCR1_IFU_QUEUE_RD_WORD       // Read word
} type_scr1_ifu_queue_rd_e;

`ifdef SCR1_NO_DEC_STAGE
typedef enum logic[1:0] {
    SCR1_BYPASS_NONE,               // No bypass
    SCR1_BYPASS_RVC,                // Bypass RVC
    SCR1_BYPASS_RVI_RDATA_QUEUE,    // Bypass RVI, rdata+queue
    SCR1_BYPASS_RVI_RDATA           // Bypass RVI, rdata only
} type_scr1_bypass_e;
`endif // SCR1_NO_DEC_STAGE

typedef enum logic [2:0] {
    // SCR1_IFU_INSTR_<UPPER_16_BITS>_<LOWER_16_BITS>
    SCR1_IFU_INSTR_NONE,                // No valid instruction
    SCR1_IFU_INSTR_RVI_HI_RVI_LO,       // Full RV32I instruction
    SCR1_IFU_INSTR_RVC_RVC,
    SCR1_IFU_INSTR_RVI_LO_RVC,
    SCR1_IFU_INSTR_RVC_RVI_HI,
    SCR1_IFU_INSTR_RVI_LO_RVI_HI,
    SCR1_IFU_INSTR_RVC_NV,              // Instruction after unaligned new_pc
    SCR1_IFU_INSTR_RVI_LO_NV            // Instruction after unaligned new_pc
} type_scr1_ifu_instr_e;

//------------------------------------------------------------------------------
// Local signals declaration
//------------------------------------------------------------------------------

// Instruction queue signals
//------------------------------------------------------------------------------

// New PC unaligned flag register
logic                               new_pc_unaligned_ff;
logic                               new_pc_unaligned_next;
logic                               new_pc_unaligned_upd;

// IMEM instruction type decoder
logic                               instr_hi_is_rvi;
logic                               instr_lo_is_rvi;
type_scr1_ifu_instr_e               instr_type;

// Register to store if the previous IMEM instruction had low part of RVI instruction
// in its high part
logic                               instr_hi_rvi_lo_ff;
logic                               instr_hi_rvi_lo_next;

// Queue read/write size decoders
type_scr1_ifu_queue_rd_e            q_rd_size;
logic                               q_rd_vd;
logic                               q_rd_none;
logic                               q_rd_hword;
type_scr1_ifu_queue_wr_e            q_wr_size;
logic                               q_wr_none;
logic                               q_wr_full;

// Write/read pointer registers
logic [SCR1_IFU_QUEUE_PTR_W-1:0]    q_rptr; //3 ����
logic [SCR1_IFU_QUEUE_PTR_W-1:0]    q_rptr_next; //3 ����
logic                               q_rptr_upd; 
logic [SCR1_IFU_QUEUE_PTR_W-1:0]    q_wptr; //3 ����
logic [SCR1_IFU_QUEUE_PTR_W-1:0]    q_wptr_next;//3 ����
logic                               q_wptr_upd;

// Instruction queue control signals
logic                               q_wr_en;
logic                               q_flush_req;

// Queue data registers
logic [`SCR1_IMEM_DWIDTH/2-1:0]     q_data  [SCR1_IFU_Q_SIZE_HALF];
logic [`SCR1_IMEM_DWIDTH/2-1:0]     q_data_head;
logic [`SCR1_IMEM_DWIDTH/2-1:0]     q_data_next;

`ifdef BPU
//Queue prediction registers
logic                               q_prediction [SCR1_IFU_Q_SIZE_HALF];
logic [`SCR1_XLEN-1:0]              q_predicted_pc [SCR1_IFU_Q_SIZE_HALF];
logic                               q_prediction_head;
logic [`SCR1_XLEN-1:0]              q_predicted_pc_head;
`endif

// Queue error flags registers
logic                               q_err   [SCR1_IFU_Q_SIZE_HALF];
logic                               q_err_head;
logic                               q_err_next;

// Instruction queue status signals
logic                               q_is_empty;
logic                               q_has_free_slots;
logic                               q_has_1_ocpd_hw;
logic                               q_head_is_rvc;
logic                               q_head_is_rvi;
logic [SCR1_IFU_Q_FREE_H_W-1:0]     q_ocpd_h;
logic [SCR1_IFU_Q_FREE_H_W-1:0]     q_free_h_next;
logic [SCR1_IFU_Q_FREE_W_W-1:0]     q_free_w_next;

// IFU FSM signals
//------------------------------------------------------------------------------

// IFU FSM control signals
logic                               ifu_fetch_req;
logic                               ifu_stop_req;

type_scr1_ifu_fsm_e                 ifu_fsm_curr;
type_scr1_ifu_fsm_e                 ifu_fsm_next;
logic                               ifu_fsm_fetch;

// IMEM signals
//------------------------------------------------------------------------------

// IMEM response signals
logic                               imem_resp_ok; //������ - ��
logic                               imem_resp_er; // ������ - ������
logic                               imem_resp_er_discard_pnd;
logic                               imem_resp_discard_req;
logic                               imem_resp_received;
logic                               imem_resp_vd;
logic                               imem_handshake_done;

logic [15:0]                        imem_rdata_lo;
logic [31:16]                       imem_rdata_hi;

// IMEM address signals
logic                               imem_addr_upd;
logic [`SCR1_XLEN-1:2]              imem_addr_ff;
logic [`SCR1_XLEN-1:2]              imem_addr_next;

// IMEM pending transactions counter
logic                               imem_pnd_txns_cnt_upd;
logic [SCR1_TXN_CNT_W-1:0]          imem_pnd_txns_cnt;
logic [SCR1_TXN_CNT_W-1:0]          imem_pnd_txns_cnt_next;
logic [SCR1_TXN_CNT_W-1:0]          imem_vd_pnd_txns_cnt;
logic                               imem_pnd_txns_q_full;

// IMEM responses discard counter
logic                               imem_resp_discard_cnt_upd;
logic [SCR1_TXN_CNT_W-1:0]          imem_resp_discard_cnt;
logic [SCR1_TXN_CNT_W-1:0]          imem_resp_discard_cnt_next;

`ifdef SCR1_NEW_PC_REG
logic                               new_pc_req_ff;
`endif // SCR1_NEW_PC_REG

// Instruction bypass signals
`ifdef SCR1_NO_DEC_STAGE
type_scr1_bypass_e                  instr_bypass_type;
logic                               instr_bypass_vd;
`endif // SCR1_NO_DEC_STAGE

`ifdef BPU
//------------------------------------------------------------------------------
// Branch Prediction
//------------------------------------------------------------------------------
// IFU <-> BPU interface
logic [`SCR1_XLEN-1:0]                   ifu2bpu_pc_o;            // BPU PC input
logic                                    bpu2ifu_prediction_lo_i; //Prediction from BPU for low part of addr
logic [`SCR1_XLEN-1:0]                   bpu2ifu_new_pc_lo_i;     //New PC from BPU for low part of addr
`ifdef SCR1_RVC_EXT
logic                                    bpu2ifu_prediction_hi_i; //Prediction from BPU for high part of addr
logic [`SCR1_XLEN-1:0]                   bpu2ifu_new_pc_hi_i;     //New PC from BPU for low part of addr
`endif //SCR1_RVC_EXT
logic                                    ifu2bpu_req_o;
parameter                                p_queue_depth = (2**SCR1_TXN_CNT_W);//Если 1 отложенная транзакция, то в очередь не записываем.
parameter                                p_queue_ptr_w = $clog2(p_queue_depth);
parameter                                p_queue_cnt_w = $clog2(p_queue_depth+1);
logic [p_queue_ptr_w-1:0]                p_queue_rptr;
logic [p_queue_ptr_w-1:0]                p_queue_rptr_next;
logic [p_queue_ptr_w-1:0]                p_queue_wptr;
logic [p_queue_ptr_w-1:0]                p_queue_wptr_next;
typedef struct packed {
    logic                  p_queue_prediction_lo;
    logic [`SCR1_XLEN-1:0] p_queue_predicted_pc_lo;
    `ifdef SCR1_RVC_EXT
    logic                  p_queue_rvi_flag_hi;
    logic                  p_queue_prediction_hi;
    logic [`SCR1_XLEN-1:0] p_queue_predicted_pc_hi;
    `endif //SCR1_RVC_EXT
} p_queue_struct;
p_queue_struct                           p_queue_data [0:p_queue_depth];
logic                                    p_queue_full;
logic                                    p_queue_empty;
logic                                    p_queue_flush_req;
logic                                    p_queue_write;
logic                                    p_queue_read;
logic                                    p_queue_prediction_head_lo;
logic [`SCR1_XLEN-1:0]                   p_queue_predicted_pc_head_lo;
`ifdef SCR1_RVC_EXT
logic                                    p_queue_rvi_flag_head_hi;
logic                                    p_queue_prediction_head_hi;
logic [`SCR1_XLEN-1:0]                   p_queue_predicted_pc_head_hi;
`endif //SCR1_RVC_EXT
logic [p_queue_cnt_w-1:0]                p_queue_cnt;
logic [p_queue_cnt_w-1:0]                p_queue_cnt_next;
logic                                    bpu_cur_prediction_lo;
logic [`SCR1_XLEN-1:0]                   bpu_cur_predicted_pc_lo;
`ifdef SCR1_RVC_EXT
logic                                    bpu_cur_rvi_flag_hi;
logic                                    bpu_cur_prediction_hi;
logic [`SCR1_XLEN-1:0]                   bpu_cur_predicted_pc_hi;
logic                                    bpu_prev_prediction_hi;
logic [`SCR1_XLEN-1:0]                   bpu_prev_predicted_pc_hi;
logic                                    bpu2ifu_rvi_flag_hi_i;
logic                                    prev_prediction_hi_ff;
logic                                    prev_prediction_hi_next;
logic [`SCR1_XLEN-1:0]                   prev_predicted_pc_hi;
`endif //SCR1_RVC_EXT
logic                                    imem_addr_unaligned_ff;
logic                                    imem_addr_unaligned_upd;
logic                                    imem_addr_unaligned_next;
logic                                    bpu_cur_update;

// BPU
//------------------------------------------------------------------------------
assign ifu2bpu_req_o = ifu2imem_req_o;
assign ifu2bpu_pc_o  = ifu2imem_addr_o;

BPU bp(.clk(clk),
       .rst_n(rst_n),
       .ifu2bpu_req_i(ifu2bpu_req_o),
       .ifu2bpu_pc_i(ifu2bpu_pc_o),
       .ifu2bpu_imem_handshake_done(imem_handshake_done),
       .ifu2bpu_pc_new_req_i(exu2ifu_pc_new_req_i),
       .ifu2bpu_pc_new_i(exu2ifu_pc_new_i),
       .ifu2bpu_b_type_i(exu2ifu_b_type_i),
       .ifu2bpu_prev_prediction_i(exu2ifu_prev_prediction_i),
       .ifu2bpu_pc_prev_i(exu2ifu_pc_prev_i),
       .ifu2bpu_btb_miss_i(exu2ifu_btb_miss_i),
       `ifdef SCR1_RVC_EXT
       .bpu2ifu_prediction_hi_o(bpu2ifu_prediction_hi_i),
       .bpu2ifu_new_pc_hi_o(bpu2ifu_new_pc_hi_i),
       .ifu2bpu_rvi_flag_i(exu2ifu_rvi_flag_i),
       .bpu2ifu_rvi_flag_hi_o(bpu2ifu_rvi_flag_hi_i),
       `endif //SCR1_RVC_EXT
       .bpu2ifu_prediction_lo_o(bpu2ifu_prediction_lo_i),
       .bpu2ifu_new_pc_lo_o(bpu2ifu_new_pc_lo_i));

//------------------------------------------------------------------------------
// BPU queue
//------------------------------------------------------------------------------
assign p_queue_flush_req = exu2ifu_pc_new_req_i | pipe2ifu_stop_fetch_i;
// BPU queue write pointer
//------------------------------------------------------------------------------
assign p_queue_write = (((imem_handshake_done & (|imem_pnd_txns_cnt[2:1] | !p_queue_empty)) & !p_queue_full) | p_queue_flush_req);
//Записываем в очередь, если в следующем такте обновится адрес и больше одной отложенной транзакции, или если очередь не пуста
//не записываем, если инструкции нужно отбросить
always_ff @(posedge clk or negedge rst_n)
    if(~rst_n) begin
        p_queue_wptr <= p_queue_ptr_w'(0);
    end
    else if (p_queue_write) begin
        p_queue_wptr <= p_queue_wptr_next;
    end

assign p_queue_wptr_next = p_queue_flush_req ? p_queue_ptr_w'(0) : p_queue_wptr + `ifdef SCR1_RVC_EXT 2 `else 1 `endif;

// BPU queue read pointer
//------------------------------------------------------------------------------
assign p_queue_read = (!p_queue_empty & imem_resp_received) | p_queue_flush_req; //Для ошибочной транзакции тоже считываем, потому что она будет передана дальше
always_ff @(posedge clk or negedge rst_n)
    if(~rst_n) begin
        p_queue_rptr <= p_queue_ptr_w'(0);
    end
    else if (p_queue_read) begin
        p_queue_rptr <= p_queue_rptr_next;
    end

assign p_queue_rptr_next = p_queue_flush_req ? p_queue_ptr_w'(0) : p_queue_rptr + `ifdef SCR1_RVC_EXT 2 `else 1 `endif;


// BPU queue read/write logic
//------------------------------------------------------------------------------
//Write logic
always_ff @(posedge clk, negedge rst_n)
    if(~rst_n) begin
        for(int i = 0; i < p_queue_depth; i++)
            p_queue_data[i] <= '{default: 0};
    end
    else if(p_queue_write) begin
        p_queue_data[p_queue_wptr].p_queue_prediction_lo   <= bpu2ifu_prediction_lo_i;
        p_queue_data[p_queue_wptr].p_queue_predicted_pc_lo <= bpu2ifu_new_pc_lo_i;
        `ifdef SCR1_RVC_EXT
        p_queue_data[p_queue_wptr].p_queue_rvi_flag_hi     <= bpu2ifu_rvi_flag_hi_i;
        p_queue_data[p_queue_wptr].p_queue_prediction_hi   <= bpu2ifu_prediction_hi_i;
        p_queue_data[p_queue_wptr].p_queue_predicted_pc_hi <= bpu2ifu_new_pc_hi_i;
        `endif
    end

//Read logic
assign p_queue_prediction_head_lo = p_queue_data[p_queue_rptr].p_queue_prediction_lo;
assign p_queue_predicted_pc_head_lo = p_queue_data[p_queue_rptr].p_queue_predicted_pc_lo;
`ifdef SCR1_RVC_EXT
assign p_queue_rvi_flag_head_hi = p_queue_data[p_queue_rptr].p_queue_rvi_flag_hi;
assign p_queue_prediction_head_hi = p_queue_data[p_queue_rptr].p_queue_prediction_hi;
assign p_queue_predicted_pc_head_hi = p_queue_data[p_queue_rptr].p_queue_predicted_pc_hi;
`endif
// BPU queue status logic
//------------------------------------------------------------------------------
always_ff @(posedge clk, negedge rst_n)
    if(~rst_n)
        p_queue_cnt <= '0;
    else
        p_queue_cnt <= p_queue_cnt_next;

assign p_queue_cnt_next = p_queue_flush_req ? '0 : p_queue_cnt + p_queue_write - p_queue_read;
assign p_queue_empty    = !p_queue_cnt;
assign p_queue_full     = p_queue_cnt == p_queue_depth;

// BPU current prediction multiplexer
//------------------------------------------------------------------------------
always_comb
    if(p_queue_empty) begin
        bpu_cur_prediction_lo   = bpu2ifu_prediction_lo_i;
        bpu_cur_predicted_pc_lo = bpu2ifu_new_pc_lo_i;
        `ifdef SCR1_RVC_EXT
        bpu_cur_rvi_flag_hi     = bpu2ifu_rvi_flag_hi_i;
        bpu_cur_prediction_hi   = bpu2ifu_prediction_hi_i;
        bpu_cur_predicted_pc_hi = bpu2ifu_new_pc_hi_i;
        `endif
    end
    else begin
        bpu_cur_prediction_lo   = p_queue_prediction_head_lo;
        bpu_cur_predicted_pc_lo = p_queue_predicted_pc_head_lo;
        `ifdef SCR1_RVC_EXT
        bpu_cur_rvi_flag_hi     = p_queue_rvi_flag_head_hi;
        bpu_cur_prediction_hi   = p_queue_prediction_head_hi;
        bpu_cur_predicted_pc_hi = p_queue_predicted_pc_head_hi;
        `endif
    end
assign bpu_cur_update = (p_queue_empty && imem_handshake_done) | (!p_queue_empty && p_queue_read);
// BPU previous prediction register
//------------------------------------------------------------------------------
`ifdef SCR1_RVC_EXT
always_ff @(posedge clk, negedge rst_n)
    if(~rst_n) begin
        bpu_prev_prediction_hi <= '0;
        bpu_prev_predicted_pc_hi <= '0;
    end
    else if (exu2ifu_pc_new_req_i | pipe2ifu_stop_fetch_i | (bpu_cur_prediction_lo && !bpu_prev_prediction_hi)) begin
        bpu_prev_prediction_hi <= '0;
        bpu_prev_predicted_pc_hi <= '0;
    end
    else begin
        if(!p_queue_empty) begin
            bpu_prev_prediction_hi <= p_queue_read 
                                    ? (bpu_prev_prediction_hi
                                    ? '0 //Чтобы не записать прдесказание по старшему адресу при дочитывании RVI инструкции. Сбрасываем флаг после перехода по новому PC
                                    : bpu_cur_prediction_hi && bpu_cur_rvi_flag_hi) 
                                    : bpu_prev_prediction_hi;
            //Если считываем предсказание из очереди предсказаний, то записываем предсказание перед его изменением, иначе - сохраняем
            bpu_prev_predicted_pc_hi <= p_queue_read 
                                      ? (bpu_prev_prediction_hi
                                      ? '0
                                      : bpu_cur_predicted_pc_hi) 
                                      : bpu_prev_predicted_pc_hi;       
        end 
        else begin
            bpu_prev_prediction_hi <= imem_handshake_done 
                                    ? (bpu_prev_prediction_hi
                                    ? '0 //Чтобы не записать прдесказание по старшему адресу при дочитывании RVI инструкции. Сбрасываем флаг после перехода по новому PC
                                    : bpu_cur_prediction_hi && bpu_cur_rvi_flag_hi) 
                                    : bpu_prev_prediction_hi;
            //Если считываем предсказание напрямую, то записываем предсказание перед его изменением, иначе - сохраняем
            bpu_prev_predicted_pc_hi <= imem_handshake_done 
                                      ? (bpu_prev_prediction_hi
                                      ? '0
                                      : bpu_cur_predicted_pc_hi) 
                                      : bpu_prev_predicted_pc_hi;    
        end
    end

always_ff @(posedge clk, negedge rst_n)
    if(~rst_n) begin
        prev_prediction_hi_ff <= 1'b0;
        prev_predicted_pc_hi  <= '0;
    end
    else if(imem_handshake_done) begin
        prev_prediction_hi_ff <= prev_prediction_hi_next;
        prev_predicted_pc_hi  <= bpu2ifu_new_pc_hi_i;
    end

assign prev_prediction_hi_next = (exu2ifu_pc_new_req_i | pipe2ifu_stop_fetch_i //| !imem_handshake_done
                                 | bpu2ifu_prediction_lo_i | (imem_handshake_done && prev_prediction_hi_ff)) //Если предыдущее предсказание было 1, то при отправке запроса флаг нужно снять
                                 ? 1'b0
                                 : prev_prediction_hi_ff //Если не нужен сброс, то проверяем флаг, если он выставлен, то сохраняем значение
                                 ? prev_prediction_hi_ff
                                 : (bpu2ifu_rvi_flag_hi_i && bpu2ifu_prediction_hi_i);
`endif //SCR1_RVC_EXT

// Imem addr unaligned flag register
//------------------------------------------------------------------------------

assign imem_addr_unaligned_upd = exu2ifu_pc_new_req_i | imem_handshake_done;
always_ff @(posedge clk, negedge rst_n) begin
    if (~rst_n) begin
        imem_addr_unaligned_ff <= 1'b0;
    end else if (imem_addr_unaligned_upd) begin
        imem_addr_unaligned_ff <= imem_addr_unaligned_next;
    end
end

always_comb begin
    priority case(1'b1)
    exu2ifu_pc_new_req_i    : imem_addr_unaligned_next = exu2ifu_pc_new_i[1];
    `ifdef SCR1_RVC_EXT
    prev_prediction_hi_ff   : imem_addr_unaligned_next = prev_predicted_pc_hi[1];
    `endif //SCR1_RVC_EXT
    bpu2ifu_prediction_lo_i : imem_addr_unaligned_next = imem_addr_unaligned_ff
                                                       ? 1'b0
                                                       : bpu2ifu_new_pc_lo_i[1];
    `ifdef SCR1_RVC_EXT
    bpu2ifu_prediction_hi_i : imem_addr_unaligned_next = bpu2ifu_rvi_flag_hi_i ? 1'b0 : bpu2ifu_new_pc_hi_i[1];
    `endif //SCR1_RVC_EXT
    ~imem_handshake_done    : imem_addr_unaligned_next = imem_addr_unaligned_ff;
    default                 : imem_addr_unaligned_next = 1'b0;
endcase
end


`endif
//------------------------------------------------------------------------------
// Instruction queue
//------------------------------------------------------------------------------
//
 // Instruction queue consists of the following functional units:
 // - New PC unaligned flag register
 // - Instruction type decoder, including register to store if the previous
 //   IMEM instruction had low part of RVI instruction in its high part
 // - Read/write size decoders
 // - Read/write pointer registers
 // - Data and error flag registers
 // - Status logic
//

// New PC unaligned flag register
//------------------------------------------------------------------------------

assign new_pc_unaligned_upd = exu2ifu_pc_new_req_i | imem_resp_vd
                `ifdef BPU | bpu_cur_prediction_lo | 
       `ifdef SCR1_RVC_EXT | (bpu_cur_prediction_hi & !bpu_cur_rvi_flag_hi)
                           | bpu_prev_prediction_hi `endif `endif;
always_ff @(posedge clk, negedge rst_n) begin
    if (~rst_n) begin
        new_pc_unaligned_ff <= 1'b0;
    end else if (new_pc_unaligned_upd) begin
        new_pc_unaligned_ff <= new_pc_unaligned_next;
    end
end

`ifndef BPU
assign new_pc_unaligned_next = exu2ifu_pc_new_req_i ? exu2ifu_pc_new_i[1]  
                             : ~imem_resp_vd        ? new_pc_unaligned_ff 
                                                    : 1'b0;

`else //BPU
always_comb begin
    priority case(1'b1)
    exu2ifu_pc_new_req_i                    : new_pc_unaligned_next = exu2ifu_pc_new_i[1];
    `ifdef SCR1_RVC_EXT
    bpu_prev_prediction_hi && bpu_cur_update: new_pc_unaligned_next = bpu_prev_predicted_pc_hi[1];
    `endif //SCR1_RVC_EXT
    bpu_cur_prediction_lo && bpu_cur_update : new_pc_unaligned_next = bpu_cur_predicted_pc_lo[1] && !new_pc_unaligned_ff;
    `ifdef SCR1_RVC_EXT
    bpu_cur_prediction_hi && bpu_cur_update : new_pc_unaligned_next = bpu_cur_rvi_flag_hi ? 1'b0 : bpu_cur_predicted_pc_hi[1];
    `endif //SCR1_RVC_EXT
    ~imem_resp_vd                           : new_pc_unaligned_next = new_pc_unaligned_ff;
    default                                 : new_pc_unaligned_next = 1'b0;
endcase
end
`endif //BPU

// Instruction type decoder
//------------------------------------------------------------------------------

assign instr_hi_is_rvi = &imem2ifu_rdata_i[17:16]; //1 ���� RVI ���������� �� �������� �����
assign instr_lo_is_rvi = &imem2ifu_rdata_i[1:0]; //1 ���� RVI ����������

always_comb begin
    instr_type = SCR1_IFU_INSTR_NONE;

    if (imem_resp_ok & ~imem_resp_discard_req) begin
        if (new_pc_unaligned_ff) begin
            instr_type = instr_hi_is_rvi ? SCR1_IFU_INSTR_RVI_LO_NV
                                         : SCR1_IFU_INSTR_RVC_NV;
        end else begin // ~new_pc_unaligned_ff
            if (instr_hi_rvi_lo_ff) begin
                instr_type = instr_hi_is_rvi ? SCR1_IFU_INSTR_RVI_LO_RVI_HI
                                             : SCR1_IFU_INSTR_RVC_RVI_HI;
            end else begin // SCR1_OTHER
                case ({instr_hi_is_rvi, instr_lo_is_rvi})
                    2'b00   : instr_type   = SCR1_IFU_INSTR_RVC_RVC;
                    2'b10   : instr_type   = SCR1_IFU_INSTR_RVI_LO_RVC;
                    default : instr_type   = SCR1_IFU_INSTR_RVI_HI_RVI_LO;
                endcase
            end
        end
    end
end

// Register to store if the previous IMEM instruction had low part of RVI
// instruction in its high part
//------------------------------------------------------------------------------

always_ff @(posedge clk, negedge rst_n) begin
    if (~rst_n) begin
        instr_hi_rvi_lo_ff <= 1'b0;
    end else begin
        if (exu2ifu_pc_new_req_i `ifdef BPU | bpu_cur_prediction_lo `endif) begin 
            //Если считали, например RVI_RVC, и для RVC будет предсказание ветвления, то нам уже без разницы,
            //что в старших байтах
            instr_hi_rvi_lo_ff <= 1'b0;
        end else if (imem_resp_vd) begin
            instr_hi_rvi_lo_ff <= instr_hi_rvi_lo_next;
        end
    end
end

assign instr_hi_rvi_lo_next = (instr_type == SCR1_IFU_INSTR_RVI_LO_NV)
                            | (instr_type == SCR1_IFU_INSTR_RVI_LO_RVI_HI)
                            | (instr_type == SCR1_IFU_INSTR_RVI_LO_RVC);

// Queue write/read size decoders
//------------------------------------------------------------------------------

// Queue read size decoder
assign q_rd_vd    = ~q_is_empty & ifu2idu_vd_o & idu2ifu_rdy_i;                                                               
assign q_rd_hword = q_head_is_rvc | q_err_head
`ifdef SCR1_NO_DEC_STAGE
                  | (q_head_is_rvi & instr_bypass_vd)
`endif // SCR1_NO_DEC_STAGE
                  ;
assign q_rd_size  = ~q_rd_vd   ? SCR1_IFU_QUEUE_RD_NONE
                  : q_rd_hword ? SCR1_IFU_QUEUE_RD_HWORD
                               : SCR1_IFU_QUEUE_RD_WORD;
assign q_rd_none  = (q_rd_size == SCR1_IFU_QUEUE_RD_NONE);

// Queue write size decoder
always_comb begin
    q_wr_size = SCR1_IFU_QUEUE_WR_NONE;
    if (~imem_resp_discard_req) begin
        if (imem_resp_ok) begin
`ifdef SCR1_NO_DEC_STAGE
            case (instr_type)
                SCR1_IFU_INSTR_NONE         : q_wr_size = SCR1_IFU_QUEUE_WR_NONE;
                SCR1_IFU_INSTR_RVI_LO_NV    : q_wr_size = SCR1_IFU_QUEUE_WR_HI;
                SCR1_IFU_INSTR_RVC_NV       : q_wr_size = (instr_bypass_vd & idu2ifu_rdy_i)
                                                        ? SCR1_IFU_QUEUE_WR_NONE
                                                        : SCR1_IFU_QUEUE_WR_HI;
                SCR1_IFU_INSTR_RVI_HI_RVI_LO: q_wr_size = (instr_bypass_vd & idu2ifu_rdy_i)
                                                        ? SCR1_IFU_QUEUE_WR_NONE
                                                        : SCR1_IFU_QUEUE_WR_FULL;
                `ifndef BPU
                SCR1_IFU_INSTR_RVC_RVC,
                SCR1_IFU_INSTR_RVI_LO_RVC,
                SCR1_IFU_INSTR_RVC_RVI_HI,
                SCR1_IFU_INSTR_RVI_LO_RVI_HI: q_wr_size = (instr_bypass_vd & idu2ifu_rdy_i)
                                                        ? SCR1_IFU_QUEUE_WR_HI
                                                        : SCR1_IFU_QUEUE_WR_FULL;
                `else
                SCR1_IFU_INSTR_RVC_RVC,
                SCR1_IFU_INSTR_RVI_LO_RVC:    q_wr_size = bpu_cur_prediction_lo 
                                                        ? ((instr_bypass_vd & idu2ifu_rdy_i)
                                                        ? SCR1_IFU_QUEUE_WR_NONE
                                                        : SCR1_IFU_QUEUE_WR_LOW)
                                                        : ((instr_bypass_vd & idu2ifu_rdy_i)
                                                        ? SCR1_IFU_QUEUE_WR_HI
                                                        : SCR1_IFU_QUEUE_WR_FULL);

                SCR1_IFU_INSTR_RVC_RVI_HI,
                SCR1_IFU_INSTR_RVI_LO_RVI_HI: q_wr_size = bpu_prev_prediction_hi  
                                                        ? ((instr_bypass_vd & idu2ifu_rdy_i)
                                                        ? SCR1_IFU_QUEUE_WR_NONE
                                                        : SCR1_IFU_QUEUE_WR_LOW)  
                                                        : ((instr_bypass_vd & idu2ifu_rdy_i)
                                                        ? SCR1_IFU_QUEUE_WR_HI
                                                        : SCR1_IFU_QUEUE_WR_FULL);                                     
                `endif
            endcase // instr_type
`else // SCR1_NO_DEC_STAGE
            case (instr_type)
            `ifndef BPU
                SCR1_IFU_INSTR_NONE         : q_wr_size = SCR1_IFU_QUEUE_WR_NONE;
                SCR1_IFU_INSTR_RVC_NV,
                SCR1_IFU_INSTR_RVI_LO_NV    : q_wr_size = SCR1_IFU_QUEUE_WR_HI;
                default                     : q_wr_size = SCR1_IFU_QUEUE_WR_FULL;
            `else
                SCR1_IFU_INSTR_NONE         : q_wr_size = SCR1_IFU_QUEUE_WR_NONE;
                SCR1_IFU_INSTR_RVC_NV,
                SCR1_IFU_INSTR_RVI_LO_NV    : q_wr_size = SCR1_IFU_QUEUE_WR_HI;
                SCR1_IFU_INSTR_RVI_HI_RVI_LO: q_wr_size = SCR1_IFU_QUEUE_WR_FULL;
                SCR1_IFU_INSTR_RVC_RVC,
                SCR1_IFU_INSTR_RVI_LO_RVC   : q_wr_size = bpu_cur_prediction_lo 
                                                        ? SCR1_IFU_QUEUE_WR_LOW
                                                        : SCR1_IFU_QUEUE_WR_FULL;
                SCR1_IFU_INSTR_RVC_RVI_HI,
                SCR1_IFU_INSTR_RVI_LO_RVI_HI: q_wr_size = bpu_prev_prediction_hi
                                                        ? SCR1_IFU_QUEUE_WR_LOW
                                                        : SCR1_IFU_QUEUE_WR_FULL;                                       

            `endif
            endcase // instr_type
`endif // SCR1_NO_DEC_STAGE
        end else if (imem_resp_er) begin
            q_wr_size = SCR1_IFU_QUEUE_WR_FULL;
        end // imem_resp_er
    end // ~imem_resp_discard_req
end

assign q_wr_none   = (q_wr_size == SCR1_IFU_QUEUE_WR_NONE);
assign q_wr_full   = (q_wr_size == SCR1_IFU_QUEUE_WR_FULL);

// Write/read pointer registers
//------------------------------------------------------------------------------

assign q_flush_req = exu2ifu_pc_new_req_i | pipe2ifu_stop_fetch_i; 

// Queue write pointer register
assign q_wptr_upd  = q_flush_req | ~q_wr_none;

always_ff @(posedge clk, negedge rst_n) begin
    if (~rst_n) begin
        q_wptr <= '0;
    end else if (q_wptr_upd) begin
        q_wptr <= q_wptr_next;
    end
end

assign q_wptr_next = q_flush_req ? '0
                   : ~q_wr_none  ? q_wptr + (q_wr_full ? SCR1_IFU_QUEUE_PTR_W'('b010) : SCR1_IFU_QUEUE_PTR_W'('b001))
                                 : q_wptr; 

// Queue read pointer register
assign q_rptr_upd  = q_flush_req | ~q_rd_none;

always_ff @(posedge clk, negedge rst_n) begin
    if (~rst_n) begin
        q_rptr <= '0;
    end else if (q_rptr_upd) begin
        q_rptr <= q_rptr_next;
    end
end

assign q_rptr_next = q_flush_req ? '0
                   : ~q_rd_none  ? q_rptr + (q_rd_hword ? SCR1_IFU_QUEUE_PTR_W'('b001) : SCR1_IFU_QUEUE_PTR_W'('b010))
                                 : q_rptr;

// Queue data and error flag registers
//------------------------------------------------------------------------------

assign imem_rdata_hi = imem2ifu_rdata_i[31:16];
assign imem_rdata_lo = imem2ifu_rdata_i[15:0];

assign q_wr_en = imem_resp_vd & ~q_flush_req;

always_ff @(posedge clk, negedge rst_n) begin
    if (~rst_n) begin
        q_data  <= '{SCR1_IFU_Q_SIZE_HALF{'0}};
        q_err   <= '{SCR1_IFU_Q_SIZE_HALF{1'b0}};
    end else if (q_wr_en) begin
        case (q_wr_size)
            SCR1_IFU_QUEUE_WR_HI    : begin
                q_data[SCR1_IFU_QUEUE_ADR_W'(q_wptr)]                <= imem_rdata_hi;
                q_err [SCR1_IFU_QUEUE_ADR_W'(q_wptr)]                <= imem_resp_er;
            `ifdef BPU
                q_prediction[SCR1_IFU_QUEUE_ADR_W'(q_wptr)]          <= bpu_cur_prediction_hi;
                q_predicted_pc[SCR1_IFU_QUEUE_ADR_W'(q_wptr)]        <= bpu_cur_predicted_pc_hi;
            `endif
            end
            `ifdef BPU
            SCR1_IFU_QUEUE_WR_LOW: begin
                q_data[SCR1_IFU_QUEUE_ADR_W'(q_wptr)]                <= imem_rdata_lo;
                q_err [SCR1_IFU_QUEUE_ADR_W'(q_wptr)]                <= imem_resp_er;
                q_prediction[SCR1_IFU_QUEUE_ADR_W'(q_wptr)]          <= bpu_cur_prediction_lo;
                q_predicted_pc[SCR1_IFU_QUEUE_ADR_W'(q_wptr)]        <= bpu_cur_predicted_pc_lo;
            end
            `endif
            SCR1_IFU_QUEUE_WR_FULL  : begin
                q_data[SCR1_IFU_QUEUE_ADR_W'(q_wptr)]                <= imem_rdata_lo;
                q_err [SCR1_IFU_QUEUE_ADR_W'(q_wptr)]                <= imem_resp_er;
                q_data[SCR1_IFU_QUEUE_ADR_W'(q_wptr + 1'b1)]         <= imem_rdata_hi;
                q_err [SCR1_IFU_QUEUE_ADR_W'(q_wptr + 1'b1)]         <= imem_resp_er;
            `ifdef BPU
                q_prediction[SCR1_IFU_QUEUE_ADR_W'(q_wptr)]          <= bpu_cur_prediction_lo;
                q_predicted_pc[SCR1_IFU_QUEUE_ADR_W'(q_wptr)]        <= bpu_cur_predicted_pc_lo;
                q_prediction[SCR1_IFU_QUEUE_ADR_W'(q_wptr + 1'b1)]   <= bpu_cur_prediction_hi;
                q_predicted_pc[SCR1_IFU_QUEUE_ADR_W'(q_wptr + 1'b1)] <= bpu_cur_predicted_pc_hi;
            `endif
            end
        endcase
    end
end

assign q_data_head         = q_data [SCR1_IFU_QUEUE_ADR_W'(q_rptr)];
assign q_data_next         = q_data [SCR1_IFU_QUEUE_ADR_W'(q_rptr + 1'b1)];
assign q_err_head          = q_err  [SCR1_IFU_QUEUE_ADR_W'(q_rptr)];
assign q_err_next          = q_err  [SCR1_IFU_QUEUE_ADR_W'(q_rptr + 1'b1)];
`ifdef BPU
assign q_prediction_head   = q_prediction[SCR1_IFU_QUEUE_ADR_W'(q_rptr)];
assign q_predicted_pc_head = q_predicted_pc[SCR1_IFU_QUEUE_ADR_W'(q_rptr)];
`endif

// Queue status logic ?????
//------------------------------------------------------------------------------

assign q_ocpd_h         = SCR1_IFU_Q_FREE_H_W'(q_wptr - q_rptr);
assign q_free_h_next    = SCR1_IFU_Q_FREE_H_W'(SCR1_IFU_Q_SIZE_HALF - (q_wptr - q_rptr_next));
assign q_free_w_next    = SCR1_IFU_Q_FREE_W_W'(q_free_h_next >> 1'b1);

assign q_is_empty       = (q_rptr == q_wptr);
assign q_has_free_slots = (SCR1_TXN_CNT_W'(q_free_w_next) > imem_vd_pnd_txns_cnt);
assign q_has_1_ocpd_hw  = (q_ocpd_h == SCR1_IFU_Q_FREE_H_W'(1));

assign q_head_is_rvi    = &(q_data_head[1:0]);
assign q_head_is_rvc    = ~q_head_is_rvi;

//------------------------------------------------------------------------------
// IFU FSM
//------------------------------------------------------------------------------

// IFU FSM control signals
assign ifu_fetch_req = exu2ifu_pc_new_req_i & ~pipe2ifu_stop_fetch_i;
assign ifu_stop_req  = pipe2ifu_stop_fetch_i
                     | (imem_resp_er_discard_pnd & ~exu2ifu_pc_new_req_i);

always_ff @(posedge clk, negedge rst_n) begin
    if (~rst_n) begin
        ifu_fsm_curr <= SCR1_IFU_FSM_IDLE;
    end else begin
        ifu_fsm_curr <= ifu_fsm_next;
    end
end

always_comb begin
    case (ifu_fsm_curr)
        SCR1_IFU_FSM_IDLE   : begin
            ifu_fsm_next = ifu_fetch_req ? SCR1_IFU_FSM_FETCH
                                         : SCR1_IFU_FSM_IDLE;
        end
        SCR1_IFU_FSM_FETCH  : begin
            ifu_fsm_next = ifu_stop_req  ? SCR1_IFU_FSM_IDLE
                                         : SCR1_IFU_FSM_FETCH;
        end
    endcase
end

assign ifu_fsm_fetch = (ifu_fsm_curr == SCR1_IFU_FSM_FETCH);

//------------------------------------------------------------------------------
// IFU <-> IMEM interface
//------------------------------------------------------------------------------
//
 // IFU <-> IMEM interface consists of the following functional units:
 // - IMEM response logic
 // - IMEM address register
 // - Pending IMEM transactions counter
 // - IMEM discard responses counter
 // - IFU <-> IMEM interface output signals
//

// IMEM response logic
//------------------------------------------------------------------------------

assign imem_resp_er             = (imem2ifu_resp_i == SCR1_MEM_RESP_RDY_ER); 
assign imem_resp_ok             = (imem2ifu_resp_i == SCR1_MEM_RESP_RDY_OK);
assign imem_resp_received       = imem_resp_ok | imem_resp_er; 
assign imem_resp_vd             = imem_resp_received & ~imem_resp_discard_req;
assign imem_resp_er_discard_pnd = imem_resp_er & ~imem_resp_discard_req;

assign imem_handshake_done = ifu2imem_req_o & imem2ifu_req_ack_i;

// IMEM address register
//------------------------------------------------------------------------------

assign imem_addr_upd = imem_handshake_done | exu2ifu_pc_new_req_i `ifdef BPU 
                     | (imem_handshake_done & (bpu2ifu_prediction_lo_i | bpu2ifu_prediction_hi_i)) `endif;

always_ff @(posedge clk, negedge rst_n) begin
    if (~rst_n) begin
        imem_addr_ff <= '0;
    end else if (imem_addr_upd) begin
        imem_addr_ff <= imem_addr_next;
    end
end

`ifndef SCR1_NEW_PC_REG 

`ifdef BPU
always_comb begin
    priority case(1'b1)
    exu2ifu_pc_new_req_i   : imem_addr_next = exu2ifu_pc_new_i[`SCR1_XLEN-1:2]  + imem_handshake_done;
    bpu2ifu_prediction_lo_i && !imem_addr_unaligned_ff: imem_addr_next = bpu2ifu_new_pc_lo_i[`SCR1_XLEN-1:2]  + imem_handshake_done;
    //Случай с bpu2ifu_prediction_hi_i и RVI инструкцией
    `ifdef SCR1_RVC_EXT
    prev_prediction_hi_ff  : imem_addr_next = prev_predicted_pc_hi[`SCR1_XLEN-1:2]  + imem_handshake_done;
    bpu2ifu_prediction_hi_i: imem_addr_next = bpu2ifu_rvi_flag_hi_i
                                            ? (&imem_addr_ff[5:2]   //Branch predicted for unaligned RVI
                                            ? imem_addr_ff + imem_handshake_done //Особого смысла не имеет, так как все-равно использоваться не будет
                                            : {imem_addr_ff[`SCR1_XLEN-1:6], imem_addr_ff[5:2] + imem_handshake_done})
                                            : bpu2ifu_new_pc_hi_i[`SCR1_XLEN-1:2]  + imem_handshake_done; //Branch predicted for unaligned RVC
    `endif //SCR1_RVC_EXT
    default:                 imem_addr_next = &imem_addr_ff[5:2]   ? imem_addr_ff                                     + imem_handshake_done
                                            : {imem_addr_ff[`SCR1_XLEN-1:6], imem_addr_ff[5:2] + imem_handshake_done};                                  
        endcase
end
`else  // BPU
assign imem_addr_next = exu2ifu_pc_new_req_i ? exu2ifu_pc_new_i[`SCR1_XLEN-1:2]                 + imem_handshake_done
                      : &imem_addr_ff[5:2]   ? imem_addr_ff                                     + imem_handshake_done
                                             : {imem_addr_ff[`SCR1_XLEN-1:6], imem_addr_ff[5:2] + imem_handshake_done};
`endif // BPU
                        
`else // SCR1_NEW_PC_REG
`ifdef BPU
always_comb begin
    priority case (1'b1)
    exu2ifu_pc_new_req_i   : imem_addr_next = exu2ifu_pc_new_i[`SCR1_XLEN-1:2];
    bpu2ifu_prediction_lo_i && !imem_addr_unaligned_ff: imem_addr_next = bpu2ifu_new_pc_lo_i[`SCR1_XLEN-1:2];
    `ifdef SCR1_RVC_EXT
    prev_prediction_hi_ff  : imem_addr_next = prev_predicted_pc_hi[`SCR1_XLEN-1:2];
    bpu2ifu_prediction_hi_i: imem_addr_next = bpu2ifu_rvi_flag_hi_i
                                            ? (&imem_addr_ff[5:2]   //Branch predicted for unaligned RVI
                                            ? imem_addr_ff + imem_handshake_done
                                            : {imem_addr_ff[`SCR1_XLEN-1:6], imem_addr_ff[5:2] + imem_handshake_done})
                                            : bpu2ifu_new_pc_hi_i[`SCR1_XLEN-1:2]; //Branch predicted for unaligned RVC
    `endif //SCR1_RVC_EXT
    default:                 imem_addr_next = &imem_addr_ff[5:2]   ? imem_addr_ff                                     + imem_handshake_done
                                            : {imem_addr_ff[`SCR1_XLEN-1:6], imem_addr_ff[5:2] + imem_handshake_done};
    endcase
end
`else // BPU
assign imem_addr_next = exu2ifu_pc_new_req_i ? exu2ifu_pc_new_i[`SCR1_XLEN-1:2]
                      : &imem_addr_ff[5:2]   ? imem_addr_ff                                     + imem_handshake_done
                                             : {imem_addr_ff[`SCR1_XLEN-1:6], imem_addr_ff[5:2] + imem_handshake_done};
`endif // BPU
`endif // SCR1_NEW_PC_REG

// Pending IMEM transactions counter
//------------------------------------------------------------------------------
// Pending IMEM transactions occur if IFU request has been acknowledged, but
// response comes in the next cycle or later
assign imem_pnd_txns_cnt_upd  = imem_handshake_done ^ imem_resp_received;

always_ff @(posedge clk, negedge rst_n) begin
    if (~rst_n) begin
        imem_pnd_txns_cnt <= '0;
    end else if (imem_pnd_txns_cnt_upd) begin
        imem_pnd_txns_cnt <= imem_pnd_txns_cnt_next;
    end
end

assign imem_pnd_txns_cnt_next = imem_pnd_txns_cnt + (imem_handshake_done - imem_resp_received);                        
assign imem_pnd_txns_q_full   = &imem_pnd_txns_cnt;

// IMEM discard responses counter
//------------------------------------------------------------------------------
// IMEM instructions should be discarded in the following 2 cases:
// 1. New PC is requested by jump, branch, mret or other instruction
// 2. IMEM response was erroneous and not discarded
//
// In both cases the number of instructions to be discarded equals to the number
// of pending instructions.
// In the 1st case we don't need all the instructions that haven't been fetched
// yet, since the PC has changed.
// In the 2nd case, since the IMEM responce was erroneous there is no guarantee
// that subsequent IMEM instructions would be valid.

assign imem_resp_discard_cnt_upd = exu2ifu_pc_new_req_i | imem_resp_er 
                                 | (imem_resp_ok & imem_resp_discard_req);

always_ff @(posedge clk, negedge rst_n) begin
    if (~rst_n) begin
        imem_resp_discard_cnt <= '0;
    end else if (imem_resp_discard_cnt_upd) begin
        imem_resp_discard_cnt <= imem_resp_discard_cnt_next;
    end
end

`ifndef SCR1_NEW_PC_REG
assign imem_resp_discard_cnt_next = exu2ifu_pc_new_req_i     ? imem_pnd_txns_cnt_next - imem_handshake_done                                                                       
                                  : imem_resp_er_discard_pnd ? imem_pnd_txns_cnt_next
                                                             : imem_resp_discard_cnt - 1'b1;
`else // SCR1_NEW_PC_REG
assign imem_resp_discard_cnt_next = exu2ifu_pc_new_req_i | imem_resp_er_discard_pnd
                                  ? imem_pnd_txns_cnt_next
                                  : imem_resp_discard_cnt - 1'b1;
`endif // SCR1_NEW_PC_REG

assign imem_vd_pnd_txns_cnt  = imem_pnd_txns_cnt - imem_resp_discard_cnt;
assign imem_resp_discard_req = |imem_resp_discard_cnt;

// IFU <-> IMEM interface output signals
//------------------------------------------------------------------------------

`ifndef SCR1_NEW_PC_REG
assign ifu2imem_req_o  = (exu2ifu_pc_new_req_i & ~imem_pnd_txns_q_full & ~pipe2ifu_stop_fetch_i)
                       | (ifu_fsm_fetch        & ~imem_pnd_txns_q_full & q_has_free_slots);
`ifndef BPU
assign ifu2imem_addr_o = exu2ifu_pc_new_req_i
                       ? {exu2ifu_pc_new_i[`SCR1_XLEN-1:2], 2'b00}
                       : {imem_addr_ff, 2'b00};
`else //BPU
always_comb begin
    priority case(1'b1)
       exu2ifu_pc_new_req_i   : ifu2imem_addr_o = {exu2ifu_pc_new_i[`SCR1_XLEN-1:2], 2'b00};
       bpu2ifu_prediction_lo_i && !imem_addr_unaligned_ff: ifu2imem_addr_o = {bpu2ifu_new_pc_lo_i[`SCR1_XLEN-1:2], 2'b00};
       `ifdef SCR1_RVC_EXT
       prev_prediction_hi_ff  : ifu2imem_addr_o = {prev_predicted_pc_hi[`SCR1_XLEN-1:2],2'b00};
       bpu2ifu_prediction_hi_i: ifu2imem_addr_o = bpu2ifu_rvi_flag_hi_i
                                                ? {imem_addr_ff, 2'b00}
                                                : {bpu2ifu_new_pc_hi_i[`SCR1_XLEN-1:2], 2'b00};
       `endif // SCR1_RVC_EXT
       default                : ifu2imem_addr_o = {imem_addr_ff, 2'b00};
    endcase
end
`endif //BPU
`else // SCR1_NEW_PC_REG
assign ifu2imem_req_o  = ifu_fsm_fetch & ~imem_pnd_txns_q_full & q_has_free_slots;
assign ifu2imem_addr_o = {imem_addr_ff, 2'b00};
`endif // SCR1_NEW_PC_REG

assign ifu2imem_cmd_o  = SCR1_MEM_CMD_RD;

`ifdef SCR1_CLKCTRL_EN
assign ifu2pipe_imem_txns_pnd_o = |imem_pnd_txns_cnt;
`endif // SCR1_CLKCTRL_EN

//------------------------------------------------------------------------------
// IFU <-> IDU interface
//------------------------------------------------------------------------------
//
 // IFU <-> IDU interface consists of the following functional units:
 // - Instruction bypass type decoder
 // - IFU <-> IDU status signals
 // - Output instruction multiplexer
//

`ifdef SCR1_NO_DEC_STAGE

// Instruction bypass type decoder
//------------------------------------------------------------------------------

assign instr_bypass_vd  = (instr_bypass_type != SCR1_BYPASS_NONE);

always_comb begin
    instr_bypass_type    = SCR1_BYPASS_NONE;

    if (imem_resp_vd) begin
        if (q_is_empty) begin
            case (instr_type)
                SCR1_IFU_INSTR_RVC_NV,
                SCR1_IFU_INSTR_RVC_RVC,
                SCR1_IFU_INSTR_RVI_LO_RVC       : begin
                    instr_bypass_type = SCR1_BYPASS_RVC;
                end
                SCR1_IFU_INSTR_RVI_HI_RVI_LO    : begin
                    instr_bypass_type = SCR1_BYPASS_RVI_RDATA;
                end
                default : begin end
            endcase // instr_type
        end else if (q_has_1_ocpd_hw & q_head_is_rvi) begin
            if (instr_hi_rvi_lo_ff) begin
                instr_bypass_type = SCR1_BYPASS_RVI_RDATA_QUEUE;
            end
        end
    end // imem_resp_vd
end

// IFU <-> IDU interface status signals
//------------------------------------------------------------------------------

always_comb begin
    ifu2idu_vd_o         = 1'b0;
    ifu2idu_imem_err_o   = 1'b0;
    ifu2idu_err_rvi_hi_o = 1'b0;

    if (ifu_fsm_fetch | ~q_is_empty) begin
        if (instr_bypass_vd) begin
            ifu2idu_vd_o          = 1'b1;
            ifu2idu_imem_err_o    = (instr_bypass_type == SCR1_BYPASS_RVI_RDATA_QUEUE)
                                  ? (imem_resp_er | q_err_head)
                                  : imem_resp_er;
            ifu2idu_err_rvi_hi_o  = (instr_bypass_type == SCR1_BYPASS_RVI_RDATA_QUEUE) & imem_resp_er;
        end else if (~q_is_empty) begin
            if (q_has_1_ocpd_hw) begin
                ifu2idu_vd_o         = q_head_is_rvc | q_err_head;
                ifu2idu_imem_err_o   = q_err_head;
                ifu2idu_err_rvi_hi_o = ~q_err_head & q_head_is_rvi & q_err_next;
            end else begin
                ifu2idu_vd_o         = 1'b1;
                ifu2idu_imem_err_o   = q_err_head ? 1'b1 : (q_head_is_rvi & q_err_next);
            end
        end // ~q_is_empty
    end
`ifdef SCR1_DBG_EN
    if (hdu2ifu_pbuf_fetch_i) begin
        ifu2idu_vd_o          = hdu2ifu_pbuf_vd_i;
        ifu2idu_imem_err_o    = hdu2ifu_pbuf_err_i;
    end
`endif // SCR1_DBG_EN
end

// Output instruction multiplexer
//------------------------------------------------------------------------------

always_comb begin
    case (instr_bypass_type)
        SCR1_BYPASS_RVC            : begin
            ifu2idu_instr_o = `SCR1_IMEM_DWIDTH'(new_pc_unaligned_ff ? imem_rdata_hi
                                                                     : imem_rdata_lo);
            `ifdef BPU
            ifu2idu_prediction_o = (instr_type == SCR1_IFU_INSTR_RVC_NV)
                                 ? bpu2ifu_prediction_hi_i
                                 : bpu2ifu_prediction_lo_i;
            ifu2idu_predicted_pc_o = (instr_type == SCR1_IFU_INSTR_RVC_NV)
                                 ? bpu2ifu_new_pc_hi_i
                                 : bpu2ifu_new_pc_lo_i;
            `endif // BPU
        end
        SCR1_BYPASS_RVI_RDATA      : begin
            ifu2idu_instr_o = imem2ifu_rdata_i;
            `ifdef BPU
            ifu2idu_prediction_o = bpu2ifu_prediction_lo_i;
            ifu2idu_predicted_pc_o = bpu2ifu_new_pc_lo_i;
            `endif // BPU
        end
        SCR1_BYPASS_RVI_RDATA_QUEUE: begin
            ifu2idu_instr_o = {imem_rdata_lo, q_data_head};
            `ifdef BPU
            ifu2idu_prediction_o = q_prediction_head;
            ifu2idu_predicted_pc_o = q_predicted_pc_head;
            `endif // BPU
        end
        default                    : begin
            ifu2idu_instr_o = `SCR1_IMEM_DWIDTH'(q_head_is_rvc ? q_data_head
                                                               : {q_data_next, q_data_head});
            `ifdef BPU
            ifu2idu_prediction_o = q_prediction_head;
            ifu2idu_predicted_pc_o = q_predicted_pc_head;
            `endif // BPU
        end
    endcase // instr_bypass_type
`ifdef SCR1_DBG_EN
    if (hdu2ifu_pbuf_fetch_i) begin
        ifu2idu_instr_o = `SCR1_IMEM_DWIDTH'({'0, hdu2ifu_pbuf_instr_i});
    end
`endif // SCR1_DBG_EN
end

`else   // SCR1_NO_DEC_STAGE

// IFU <-> IDU interface status signals
//------------------------------------------------------------------------------

always_comb begin
    ifu2idu_vd_o          = 1'b0;
    ifu2idu_imem_err_o    = 1'b0;
    ifu2idu_err_rvi_hi_o  = 1'b0;
    if (~q_is_empty) begin
        if (q_has_1_ocpd_hw) begin
            ifu2idu_vd_o          = q_head_is_rvc | q_err_head;
            ifu2idu_imem_err_o    = q_err_head;
        end else begin
            ifu2idu_vd_o          = 1'b1;
            ifu2idu_imem_err_o    = q_err_head ? 1'b1 : (q_head_is_rvi & q_err_next);
            ifu2idu_err_rvi_hi_o  = ~q_err_head & q_head_is_rvi & q_err_next;
        end
    end // ~q_is_empty
`ifdef SCR1_DBG_EN
    if (hdu2ifu_pbuf_fetch_i) begin
        ifu2idu_vd_o          = hdu2ifu_pbuf_vd_i;
        ifu2idu_imem_err_o    = hdu2ifu_pbuf_err_i;
    end
`endif // SCR1_DBG_EN
end

// Output instruction multiplexer
//------------------------------------------------------------------------------

always_comb begin
    ifu2idu_instr_o = q_head_is_rvc ? `SCR1_IMEM_DWIDTH'(q_data_head)
                                    : {q_data_next, q_data_head};
    `ifdef BPU
    ifu2idu_prediction_o = q_prediction_head;
    ifu2idu_predicted_pc_o = q_predicted_pc_head;
    `endif //BPU
`ifdef SCR1_DBG_EN
    if (hdu2ifu_pbuf_fetch_i) begin
        ifu2idu_instr_o = `SCR1_IMEM_DWIDTH'({'0, hdu2ifu_pbuf_instr_i});
    end
`endif // SCR1_DBG_EN
end

`endif  // SCR1_NO_DEC_STAGE

`ifdef SCR1_DBG_EN
assign ifu2hdu_pbuf_rdy_o = idu2ifu_rdy_i;
`endif // SCR1_DBG_EN

`ifdef SCR1_TRGT_SIMULATION

//------------------------------------------------------------------------------
// Assertions
//------------------------------------------------------------------------------

// X checks

SCR1_SVA_IFU_XCHECK : assert property (
    @(negedge clk) disable iff (~rst_n)
    !$isunknown({imem2ifu_req_ack_i, idu2ifu_rdy_i, exu2ifu_pc_new_req_i})
    ) else $error("IFU Error: unknown values");

SCR1_SVA_IFU_XCHECK_REQ : assert property (
    @(negedge clk) disable iff (~rst_n)
    ifu2imem_req_o |-> !$isunknown({ifu2imem_addr_o, ifu2imem_cmd_o})
    ) else $error("IFU Error: unknown {ifu2imem_addr_o, ifu2imem_cmd_o}");

// Behavior checks

SCR1_SVA_IFU_DRC_UNDERFLOW : assert property (
    @(negedge clk) disable iff (~rst_n)
    ~imem_resp_discard_req |=> ~(imem_resp_discard_cnt == SCR1_TXN_CNT_W'('1))
    ) else $error("IFU Error: imem_resp_discard_cnt underflow");

SCR1_SVA_IFU_DRC_RANGE : assert property (
    @(negedge clk) disable iff (~rst_n)
    (imem_resp_discard_cnt >= 0) & (imem_resp_discard_cnt <= imem_pnd_txns_cnt)
    ) else $error("IFU Error: imem_resp_discard_cnt out of range");

SCR1_SVA_IFU_QUEUE_OVF : assert property (
    @(negedge clk) disable iff (~rst_n)
    (q_ocpd_h >= SCR1_IFU_Q_FREE_H_W'(SCR1_IFU_Q_SIZE_HALF-1)) |->
    ((q_ocpd_h == SCR1_IFU_Q_FREE_H_W'(SCR1_IFU_Q_SIZE_HALF-1)) ? (q_wr_size != SCR1_IFU_QUEUE_WR_FULL)
                                                                : (q_wr_size == SCR1_IFU_QUEUE_WR_NONE))
    ) else $error("IFU Error: queue overflow");

SCR1_SVA_IFU_IMEM_ERR_BEH : assert property (
    @(negedge clk) disable iff (~rst_n)
    (imem_resp_er & ~imem_resp_discard_req & ~exu2ifu_pc_new_req_i) |=>
    (ifu_fsm_curr == SCR1_IFU_FSM_IDLE) & (imem_resp_discard_cnt == imem_pnd_txns_cnt)
    ) else $error("IFU Error: incorrect behavior after memory error");

SCR1_SVA_IFU_NEW_PC_REQ_BEH : assert property (
    @(negedge clk) disable iff (~rst_n)
    exu2ifu_pc_new_req_i |=> q_is_empty
    ) else $error("IFU Error: incorrect behavior after exu2ifu_pc_new_req_i");

SCR1_SVA_IFU_IMEM_ADDR_ALIGNED : assert property (
    @(negedge clk) disable iff (~rst_n)
    ifu2imem_req_o |-> ~|ifu2imem_addr_o[1:0]
    ) else $error("IFU Error: unaligned IMEM access");

SCR1_SVA_IFU_STOP_FETCH : assert property (
    @(negedge clk) disable iff (~rst_n)
    pipe2ifu_stop_fetch_i |=> (ifu_fsm_curr == SCR1_IFU_FSM_IDLE)
    ) else $error("IFU Error: fetch not stopped");

SCR1_SVA_IFU_IMEM_FAULT_RVI_HI : assert property (
    @(negedge clk) disable iff (~rst_n)
    ifu2idu_err_rvi_hi_o |-> ifu2idu_imem_err_o
    ) else $error("IFU Error: ifu2idu_imem_err_o == 0");

`endif // SCR1_TRGT_SIMULATION

endmodule : scr1_pipe_ifu
