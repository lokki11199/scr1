module scr1_dp_ram #(parameter RAM_WIDTH = 32,
                     parameter RAM_DEPTH = 32)
                    (input  logic                         clk,     //clock
                     input  logic [$clog2(RAM_DEPTH)-1:0] addra,   //BTB write address
                     input  logic [$clog2(RAM_DEPTH)-1:0] addrb,   //BTB read address
                     input  logic [RAM_WIDTH-1:0]         dina,    //BTB write data
                     input  logic                         wena,     //BTB write enable
                     input  logic                         renb,     //BTB read enable
                     output logic [RAM_WIDTH-1:0]         doutb    //BTB read data
                     );

`ifdef SCR1_TRGT_FPGA_XILINX
(* ram_style = "block" *)  logic  [RAM_WIDTH-1:0]  ram_block  [RAM_DEPTH-1:0];
`else  // ASIC or SIMULATION
logic  [RAM_WIDTH-1:0]  ram_block  [RAM_DEPTH-1:0];
`endif

//-------------------------------------------------------------------------------
// BTB write logic
//-------------------------------------------------------------------------------
always_ff @(posedge clk)
begin
    if(wena)
        ram_block[addra] <= dina;
end

//-------------------------------------------------------------------------------
// BTB read logic
//-------------------------------------------------------------------------------
always_ff @(posedge clk) 
begin
    if (renb) begin
        doutb <= ram_block[addrb];
    end
end

endmodule: scr1_dp_ram