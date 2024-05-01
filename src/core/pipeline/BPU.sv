`include "scr1_arch_description.svh"
`define SAT_CNT_2
module BPU #(parameter DEPTH = 2048,// Must be a power of two
             parameter TAG_UNUSED = 0)
            (
                input  logic                  clk ,rst_n,
                input  logic                  ifu2bpu_req_i,//New
                input  logic [`SCR1_XLEN-1:0] ifu2bpu_pc_i,
                input  logic                  ifu2bpu_imem_handshake_done,
                input  logic                  ifu2bpu_pc_new_req_i,
                input  logic [`SCR1_XLEN-1:0] ifu2bpu_pc_new_i,
                input  logic                  ifu2bpu_b_type_i,
                input  logic                  ifu2bpu_prev_prediction_i,
                input  logic [`SCR1_XLEN-1:0] ifu2bpu_pc_prev_i,
                input  logic                  ifu2bpu_btb_miss_i,
                `ifdef SCR1_RVC_EXT
                input  logic                  ifu2bpu_rvi_flag_i,
                output logic                  bpu2ifu_rvi_flag_hi_o,
                output logic                  bpu2ifu_prediction_hi_o,
                output logic [`SCR1_XLEN-1:0] bpu2ifu_new_pc_hi_o,
                `endif
                output logic                  bpu2ifu_prediction_lo_o,
                output logic [`SCR1_XLEN-1:0] bpu2ifu_new_pc_lo_o
            );
localparam pc_bit_used = $clog2(DEPTH) `ifdef SCR1_RVC_EXT +1 `else +2 `endif;//Если RVC, то +1?
logic prediction_lo;
`ifdef SCR1_RVC_EXT
logic prediction_hi;
`endif
logic bpu2stc_b_type_i[0:DEPTH-1];
logic bpu2stc_new_pc_req_i[0:DEPTH-1];
logic stc2bpu_prediction_o[0:DEPTH-1];
logic bpu2sct_prev_prediction_i[0:DEPTH-1];
//Branch target buffer logic
typedef struct packed {
    `ifdef SCR1_RVC_EXT
    logic                                         rvi_flag;
    `endif
    logic [`SCR1_XLEN-pc_bit_used-1-TAG_UNUSED:0] tag;
    logic [`SCR1_XLEN-1:0]                        pc;
} branch_buffer;

branch_buffer btb[0:DEPTH-1];

always_ff @(posedge clk, negedge rst_n)
    if(~rst_n) begin
        for(int i = 0; i < DEPTH; i++)
            btb[i] <= '{default: 0};
    end
    else if (ifu2bpu_b_type_i && ifu2bpu_pc_new_req_i && (!ifu2bpu_prev_prediction_i | ifu2bpu_btb_miss_i))
    begin
    btb[ifu2bpu_pc_prev_i[pc_bit_used-1: `ifdef SCR1_RVC_EXT 1 `else 2 `endif ]].pc <= ifu2bpu_pc_new_i;
    btb[ifu2bpu_pc_prev_i[pc_bit_used-1: `ifdef SCR1_RVC_EXT 1 `else 2 `endif ]].tag <= ifu2bpu_pc_prev_i[`SCR1_XLEN-1-TAG_UNUSED:pc_bit_used];
    `ifdef SCR1_RVC_EXT
    btb[ifu2bpu_pc_prev_i[pc_bit_used-1: `ifdef SCR1_RVC_EXT 1 `else 2 `endif ]].rvi_flag <= ifu2bpu_rvi_flag_i;
    `endif
    end

branch_buffer curr_data_btb_lo;
`ifdef SCR1_RVC_EXT
branch_buffer curr_data_btb_hi;
`endif
logic [`SCR1_XLEN-1:0] curr_pc;
always_ff @(posedge clk)
    begin
    if(ifu2bpu_imem_handshake_done)
    begin
    `ifndef SCR1_RVC_EXT
    curr_data_btb_lo <= btb[ifu2bpu_pc_i[pc_bit_used-1:2]];
    `else
    curr_data_btb_lo <= btb[{ifu2bpu_pc_i[pc_bit_used-1:2], 1'b0}];
    curr_data_btb_hi <= btb[{ifu2bpu_pc_i[pc_bit_used-1:2], 1'b1}];
    `endif
    curr_pc <= ifu2bpu_pc_i;
    end
    end

always_comb
    begin
            if(prediction_lo && (curr_data_btb_lo.tag==curr_pc[`SCR1_XLEN-1:pc_bit_used]))
            begin
                bpu2ifu_new_pc_lo_o = curr_data_btb_lo.pc;
                bpu2ifu_prediction_lo_o = 1'b1;
            end
            else
                begin
                    bpu2ifu_new_pc_lo_o = '0;
                    bpu2ifu_prediction_lo_o = 1'b0;
                end
            `ifdef SCR1_RVC_EXT
            if(prediction_hi && (curr_data_btb_hi.tag==curr_pc[`SCR1_XLEN-1:pc_bit_used]))
            begin
                bpu2ifu_new_pc_hi_o = curr_data_btb_hi.pc;
                bpu2ifu_prediction_hi_o = 1'b1;
                bpu2ifu_rvi_flag_hi_o = curr_data_btb_hi.rvi_flag;
            end
            else
                begin
                    bpu2ifu_new_pc_hi_o = '0;
                    bpu2ifu_prediction_hi_o = 1'b0;
                    bpu2ifu_rvi_flag_hi_o = 1'b0;
                end
            `endif
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
        `ifndef SCR1_RVC_EXT
        prediction_lo = stc2bpu_prediction_o[curr_pc[pc_bit_used-1:2]];
        `else
        prediction_lo = stc2bpu_prediction_o[curr_pc[pc_bit_used-1:1]];
        prediction_hi = stc2bpu_prediction_o[curr_pc[pc_bit_used-1:1] + 1'b1];
        `endif
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