`include "scr1_arch_description.svh"
`define SAT_CNT_2
module BPU #(parameter DEPTH = 64,// Must be a power of two
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

localparam pc_bit_used = $clog2(DEPTH/2); //two less significant bits is unused

//-------------------------------------------------------------------------------
// LOCAL TYPES DEFINITION
//-------------------------------------------------------------------------------
typedef struct packed {
    `ifdef SCR1_RVC_EXT
    logic                                         rvi_flag;
    `endif
    logic [`SCR1_XLEN-1-TAG_UNUSED:pc_bit_used+2] tag;
    logic [`SCR1_XLEN-1:0]                        pc;
} branch_buffer;

//-------------------------------------------------------------------------------
// LOCAL VARIABLES DECLARATION
//-------------------------------------------------------------------------------
logic prediction_lo;
`ifdef SCR1_RVC_EXT
logic prediction_hi;
`endif
logic bpu2stc_b_type_i[0:DEPTH-1];
logic bpu2stc_new_pc_req_i[0:DEPTH-1];
logic stc2bpu_prediction_o[0:DEPTH-1];
logic bpu2sct_prev_prediction_i[0:DEPTH-1];
//branch_buffer btb[0:DEPTH-1];
branch_buffer curr_data_btb_lo;
`ifdef SCR1_RVC_EXT
branch_buffer curr_data_btb_hi;
`endif
logic [`SCR1_XLEN-1:0] curr_pc;
logic prev_r_en;

logic                                         w_en;
logic                                         r_en;
logic                                         w_en_lo;
logic                                         w_en_hi;

logic                                         rvi_flag_w;
logic [`SCR1_XLEN-1-TAG_UNUSED:pc_bit_used+2] tag_w;
logic [`SCR1_XLEN-1:0]                        pc_w;

logic [$bits(branch_buffer)-1:0] read_data_ram_lo;
`ifdef SCR1_RVC_EXT
logic [$bits(branch_buffer)-1:0] read_data_ram_hi;
`endif

logic [$bits(branch_buffer)-1:0] read_data_ram_lo_saved;
`ifdef SCR1_RVC_EXT
logic [$bits(branch_buffer)-1:0] read_data_ram_hi_saved;
`endif

//-------------------------------------------------------------------------------
// BTB WRITE/READ OPERATION
//-------------------------------------------------------------------------------

//`ifdef SCR1_TRGT_FPGA_XILINX


assign rvi_flag_w = ifu2bpu_rvi_flag_i; //RVI flag write
assign tag_w      = ifu2bpu_pc_prev_i[`SCR1_XLEN-1-TAG_UNUSED:pc_bit_used+2]; //Tag write
assign pc_w       = ifu2bpu_pc_new_i; //PC write

assign w_en    = ifu2bpu_b_type_i && ifu2bpu_pc_new_req_i && (!ifu2bpu_prev_prediction_i | ifu2bpu_btb_miss_i);
assign r_en    = ifu2bpu_imem_handshake_done;
assign w_en_lo = w_en & ~ifu2bpu_pc_prev_i[1];
assign w_en_hi = w_en & ifu2bpu_pc_prev_i[1];
//-------------------------------------------------------------------------------
// Memory instantiation
//-------------------------------------------------------------------------------
//For aligned words
scr1_dp_ram #(
    .RAM_WIDTH ( $bits(branch_buffer) ),
    .RAM_DEPTH  ( DEPTH/2 )
) i_dp_ram_lo (
    .clk  ( clk ),
    .addra( ifu2bpu_pc_prev_i[2+:pc_bit_used] ),
    .addrb( ifu2bpu_pc_i[2+:pc_bit_used] ),
    .dina ( $bits(branch_buffer)'({rvi_flag_w, tag_w, pc_w}) ),
    .wena ( w_en_lo ),
    .renb ( r_en ),
    .doutb( read_data_ram_lo )                
);

`ifdef SCR1_RVC_EXT
//For unaligned words
scr1_dp_ram #(
    .RAM_WIDTH ( $bits(branch_buffer) ),
    .RAM_DEPTH  ( DEPTH/2 )
) i_dp_ram_hi (
    .clk  ( clk ),
    .addra( ifu2bpu_pc_prev_i[2+:pc_bit_used] ),
    .addrb( ifu2bpu_pc_i[2+:pc_bit_used] ),
    .dina ( $bits(branch_buffer)'({rvi_flag_w, tag_w, pc_w}) ),
    .wena ( w_en_hi ),
    .renb ( r_en ),
    .doutb( read_data_ram_hi )          
);
`endif
//-------------------------------------------------------------------------------
// Branch prediction logic
//-------------------------------------------------------------------------------
always_ff @(posedge clk, negedge rst_n)
    if(~rst_n)
        curr_pc <= SCR1_ARCH_RST_VECTOR;
    else if (r_en)
        curr_pc <= ifu2bpu_pc_i;

always_ff @(posedge clk, negedge rst_n)
    if(~rst_n)
        prev_r_en <= 1'b0;
    else
        prev_r_en <= r_en;

always_ff @(posedge clk, negedge rst_n)
    if(~rst_n) begin
        read_data_ram_lo_saved <= '0;
`ifdef SCR1_RVC_EXT
        read_data_ram_lo_saved <= '0;
`endif
    end
    else if(prev_r_en && !(r_en)) begin
        read_data_ram_lo_saved <= read_data_ram_lo;
`ifdef SCR1_RVC_EXT
        read_data_ram_hi_saved <= read_data_ram_hi;
`endif
    end

assign curr_data_btb_lo = branch_buffer'(prev_r_en ? read_data_ram_lo : read_data_ram_lo_saved);
`ifdef SCR1_RVC_EXT
assign curr_data_btb_hi = branch_buffer'(prev_r_en ? read_data_ram_hi : read_data_ram_hi_saved);
`endif

always_comb
    begin
            if((curr_data_btb_lo.tag==curr_pc[`SCR1_XLEN-1:pc_bit_used+2]))
            begin
                bpu2ifu_new_pc_lo_o = curr_data_btb_lo.pc;
                bpu2ifu_prediction_lo_o = prediction_lo;
            end
            else
                begin
                    bpu2ifu_new_pc_lo_o = '0;
                    bpu2ifu_prediction_lo_o = 1'b0;
                end
            `ifdef SCR1_RVC_EXT
            if((curr_data_btb_hi.tag==curr_pc[`SCR1_XLEN-1:pc_bit_used+2]))
            begin
                bpu2ifu_new_pc_hi_o = curr_data_btb_hi.pc;
                bpu2ifu_prediction_hi_o = prediction_hi;
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
        prediction_lo = stc2bpu_prediction_o[curr_pc[1+:(pc_bit_used+1)]];
        `else
        prediction_lo = stc2bpu_prediction_o[curr_pc[1+:(pc_bit_used+1)]];
        prediction_hi = stc2bpu_prediction_o[curr_pc[1+:(pc_bit_used+1)] + 1'b1];
        `endif
        for(int i = 0; i < DEPTH; i++) begin
            if(i == ifu2bpu_pc_prev_i[1+:(pc_bit_used+1)])
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