module satcount_2bit(input  logic clk, rst_n,
                     input  logic bpu2stc_b_type_i,
                     input  logic bpu2stc_new_pc_req_i,
                     input  logic bpu2sct_prev_prediction_i,
                     output logic stc2bpu_prediction_o);

enum logic [1:0] {snt, wnt, wt, st} state, nextstate, prevstate;

always_ff @(posedge clk, negedge rst_n)
    if(~rst_n)
        state <= snt;
    else if(bpu2stc_b_type_i)
        if(bpu2stc_new_pc_req_i)
            state <= bpu2sct_prev_prediction_i ? prevstate : nextstate;
        else
            state <= bpu2sct_prev_prediction_i ? nextstate : prevstate;


always_comb
    begin
        case(state)
        snt: begin nextstate = wnt; prevstate = snt; end
        wnt: begin nextstate = wt; prevstate = snt; end
        wt:  begin nextstate = st; prevstate = wnt; end
        st:  begin nextstate = st; prevstate = wt; end
        endcase
    end

assign stc2bpu_prediction_o = state[1];

endmodule