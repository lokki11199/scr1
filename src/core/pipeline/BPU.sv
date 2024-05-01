`include "scr1_arch_description.svh"
`define SAT_CNT_2
module BPU #(parameter DEPTH = 2048,// Must be a power of two
             parameter TAG_UNUSED = 0)
            (
                input  logic                  clk ,rst_n,
                input  logic [`SCR1_XLEN-1:0] ifu2bpu_pc_i, //imem_addr_o
                input  logic                  ifu2bpu_pc_new_req_i,
                input  logic [`SCR1_XLEN-1:0] ifu2bpu_pc_new_i,
                input  logic                  ifu2bpu_b_type_i, //Add to pipeline
                input  logic                  ifu2bpu_prev_prediction_i,//Add to pipeline
                input  logic [`SCR1_XLEN-1:0] ifu2bpu_pc_prev_i,//Add to pipeline
                input  logic                  ifu2bpu_btb_miss_i, //Add to pipeline
                output logic                  bpu2ifu_prediction_o,//Add to pipeline
                output logic [`SCR1_XLEN-1:0] bpu2ifu_new_pc_o//Add to pipeline
            );
localparam pc_bit_used = $clog2(DEPTH) `ifdef SCR1_RVC_EXT +1 `else +2 `endif;//Если RVC, то +1?
logic prediction;
logic bpu2stc_b_type_i[0:DEPTH-1];
logic bpu2stc_new_pc_req_i[0:DEPTH-1];
logic stc2bpu_prediction_o[0:DEPTH-1];
logic bpu2sct_prev_prediction_i[0:DEPTH-1];
//Branch target buffer logic
typedef struct packed {
    logic [`SCR1_XLEN-pc_bit_used-1-TAG_UNUSED:0] tag;
    logic [`SCR1_XLEN-1:0]             pc;
} branch_buffer;

branch_buffer btb[0:DEPTH-1];

always_ff @(posedge clk, negedge rst_n)
    if(~rst_n) begin
        for(int i = 0; i < DEPTH; i++)
            btb[i] <= '{default: 0};
    end
    else if (ifu2bpu_b_type_i && ifu2bpu_pc_new_req_i && (!ifu2bpu_prev_prediction_i | ifu2bpu_btb_miss_i))
    begin//В каком случае обновляем btb? 1. Запрос на новый пк и предыдущее предсказание=0 или miss btb
    btb[ifu2bpu_pc_prev_i[pc_bit_used-1: `ifdef SCR1_RVC_EXT 1 `else 2 `endif ]].pc <= ifu2bpu_pc_new_i;
    btb[ifu2bpu_pc_prev_i[pc_bit_used-1: `ifdef SCR1_RVC_EXT 1 `else 2 `endif ]].tag <= ifu2bpu_pc_prev_i[`SCR1_XLEN-1-TAG_UNUSED:pc_bit_used];
    //btb[ifu2bpu_pc_prev_i[pc_bit_used-1:0]] <= '{ifu2bpu_pc_new_i, ifu2bpu_pc_prev_i[`SCR1_XLEN-1-TAG_UNUSED:pc_bit_used]};
    end

branch_buffer curr_data_btb;
logic [`SCR1_XLEN-1:0] curr_pc;
always_ff @(posedge clk)
    begin
    curr_data_btb <= btb[ifu2bpu_pc_i[pc_bit_used-1: `ifdef SCR1_RVC_EXT 1 `else 2 `endif ]];
    curr_pc <= ifu2bpu_pc_i;
    end

always_comb
    begin
            if(prediction && (curr_data_btb.tag==curr_pc[`SCR1_XLEN-1:pc_bit_used]))
            begin
                bpu2ifu_new_pc_o = curr_data_btb.pc;
                bpu2ifu_prediction_o = 1'b1;
            end
            else
                begin
                    bpu2ifu_new_pc_o = '0;
                    bpu2ifu_prediction_o = 1'b0;
                end
    end

//Branch prediction logic

// 2 bit saturation counter
`ifdef SAT_CNT_2
genvar i;
generate
    for(i = 0; i < DEPTH; i++)
        begin
            satcount_2bit stc(clk, rst_n, bpu2stc_b_type_i[i], bpu2stc_new_pc_req_i[i], bpu2sct_prev_prediction_i[i], stc2bpu_prediction_o[i]);
        end
endgenerate
`endif

always_comb
    begin
        prediction = stc2bpu_prediction_o[curr_pc[pc_bit_used-1: `ifdef SCR1_RVC_EXT 1 `else 2 `endif ]];
        for(int i = 0; i < DEPTH; i++) begin
            if(i == ifu2bpu_pc_prev_i[pc_bit_used-1: `ifdef SCR1_RVC_EXT 1 `else 2 `endif ])
                begin
                bpu2stc_b_type_i[i] = ifu2bpu_b_type_i;
                bpu2stc_new_pc_req_i[i] = ifu2bpu_pc_new_req_i;
                bpu2sct_prev_prediction_i[i] = ifu2bpu_prev_prediction_i;
                end
            else
            begin
                bpu2stc_b_type_i[i] = 1'b0;
                bpu2stc_new_pc_req_i[i] = 1'b0;
                bpu2sct_prev_prediction_i[i] = 1'b0;
            end
        end
    end

endmodule