// Code showcasing the use of 8 FIFOs, 4 FIFOs and 2 FIFOs. Comments explain the modifications required for each scenario.
// This code has all the submodules included with the topmodule and requires few mentioned modifications in the testbench too.

module top_module (
    input wire clk,
    input wire rst,
    input wire [143:0] mem_req, //8 bits of data, 1 bit of wr_en, 1 bit of rd_en, 8 bits of address (used for handling 8 unique memory requests handled by 8 FIFOs)
    output wire [71:0] mem_resp  // 8 bits of data and 1 bit of status

);
    wire [143:0] xbar_inputs;
    wire [143:0] xbar_outputs;
    wire [15:0] fifo_full, fifo_empty;
    wire [15:0] write_enables, read_enables;
    
    wire [7:0] req_array;
    reg [23:0] select;
//  wire [7:0] select_fin;
    



    // Memory control signals
    wire [7:0] mem_write_en, mem_read_en;
    wire [5:0] mem_addrs [7:0];
    wire [143:0] mem_data_in;
  //wire [31:0] mem_data_out;

    // Instantiate crossbar switch
    crossbar_switch #(.N(8), .WIDTH(18)) c_b (
        .inputs(xbar_inputs),
        .select(select),
       // .select(8'b11100100),
        .outputs(xbar_outputs)
    );
	 
/* Logic for 2x2
	 
always @(*) begin

if((xbar_inputs[7:6] == 2'b00) || (xbar_inputs[25:24] == 2'b00) ) begin
  
  select[0] = 1'b0;
  
end

if((xbar_inputs[7:6] == 2'b01) || (xbar_inputs[25:24] == 2'b01) ) begin
  
  select[1] = 1'b1;
  
end
end
  */  
/* 4x4

always @(*) begin

if((xbar_inputs[7:6] == 2'b00) || (xbar_inputs[25:24] == 2'b00) || (xbar_inputs[43:42] == 2'b00) || (xbar_inputs[61:60] == 2'b00)) begin
  
  select[1:0] = 2'b00;
  
end

if((xbar_inputs[7:6] == 2'b01) || (xbar_inputs[25:24] == 2'b01) || (xbar_inputs[43:42] == 2'b01) || (xbar_inputs[61:60] == 2'b01)) begin
  
  select[3:2] = 2'b01;
  
end

if((xbar_inputs[7:6] == 2'b10) || (xbar_inputs[25:24] == 2'b10) || (xbar_inputs[43:42] == 2'b10) || (xbar_inputs[61:60] == 2'b10)) begin
  
  select[5:4] = 2'b10;
  
end

if((xbar_inputs[7:6] == 2'b11) || (xbar_inputs[25:24] == 2'b11) || (xbar_inputs[43:42] == 2'b11) || (xbar_inputs[61:60] == 2'b11)) begin
  
  select[7:6] = 2'b11;
  
end
  

end   
*/

//Logic for 8x8
always @(*) begin

if((xbar_inputs[7:5] == 3'b000) || (xbar_inputs[25:23] == 3'b000) || (xbar_inputs[43:41] == 3'b000) || (xbar_inputs[61:59] == 3'b000) || (xbar_inputs[79:77] == 3'b000) || (xbar_inputs[97:95] == 3'b000) || (xbar_inputs[115:113] == 3'b000) || (xbar_inputs[133:131] == 3'b000)) begin
  
  select[2:0] = 3'b000;
  
end

if((xbar_inputs[7:5] == 3'b001) || (xbar_inputs[25:23] == 3'b001) || (xbar_inputs[43:41] == 3'b001) || (xbar_inputs[61:59] == 3'b001) || (xbar_inputs[79:77] == 3'b001) || (xbar_inputs[97:95] == 3'b001) || (xbar_inputs[115:113] == 3'b001) || (xbar_inputs[133:131] == 3'b001)) begin  
  select[5:3] = 3'b001;
  
end

if((xbar_inputs[7:5] == 3'b010) || (xbar_inputs[25:23] == 3'b010) || (xbar_inputs[43:41] == 3'b010) || (xbar_inputs[61:59] == 3'b010) || (xbar_inputs[79:77] == 3'b010) || (xbar_inputs[97:95] == 3'b010) || (xbar_inputs[115:113] == 3'b010) || (xbar_inputs[133:131] == 3'b010)) begin  
  select[8:6] = 3'b010;
  
end

if((xbar_inputs[7:5] == 3'b011) || (xbar_inputs[25:23] == 3'b011) || (xbar_inputs[43:41] == 3'b011) || (xbar_inputs[61:59] == 3'b011) || (xbar_inputs[79:77] == 3'b011) || (xbar_inputs[97:95] == 3'b011) || (xbar_inputs[115:113] == 3'b011) || (xbar_inputs[133:131] == 3'b011)) begin  
  select[11:9] = 3'b011;
  
end


if((xbar_inputs[7:5] == 3'b100) || (xbar_inputs[25:23] == 3'b100) || (xbar_inputs[43:41] == 3'b100) || (xbar_inputs[61:59] == 3'b100) || (xbar_inputs[79:77] == 3'b100) || (xbar_inputs[97:95] == 3'b100) || (xbar_inputs[115:113] == 3'b100) || (xbar_inputs[133:131] == 3'b100)) begin
  
  select[14:12] = 3'b100;
  
end

if((xbar_inputs[7:5] == 3'b101) || (xbar_inputs[25:23] == 3'b101) || (xbar_inputs[43:41] == 3'b101) || (xbar_inputs[61:59] == 3'b101) || (xbar_inputs[79:77] == 3'b101) || (xbar_inputs[97:95] == 3'b101) || (xbar_inputs[115:113] == 3'b101) || (xbar_inputs[133:131] == 3'b101)) begin  
  select[17:15] = 3'b101;
  
end

if((xbar_inputs[7:5] == 3'b110) || (xbar_inputs[25:23] == 3'b110) || (xbar_inputs[43:41] == 3'b110) || (xbar_inputs[61:59] == 3'b110) || (xbar_inputs[79:77] == 3'b110) || (xbar_inputs[97:95] == 3'b110) || (xbar_inputs[115:113] == 3'b110) || (xbar_inputs[133:131] == 3'b110)) begin  
  select[20:18] = 3'b110;
  
end

if((xbar_inputs[7:5] == 3'b111) || (xbar_inputs[25:23] == 3'b111) || (xbar_inputs[43:41] == 3'b111) || (xbar_inputs[61:59] == 3'b111) || (xbar_inputs[79:77] == 3'b111) || (xbar_inputs[97:95] == 3'b111) || (xbar_inputs[115:113] == 3'b111) || (xbar_inputs[133:131] == 3'b111)) begin  
  select[23:21] = 3'b111;
  
end

  

end   


  genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin : left_fifos
            fifo #(.DATA_WIDTH(18), .ADDR_WIDTH(4)) left_fifo (
                .clk(clk),
                .rst(rst),
                .write_req(write_enables[i]),
                .read_req(read_enables[i]),
                .data_in(mem_req[i*18+:18]),
                .data_out(xbar_inputs[18*(i+1)-1:18*i]), //input to crossbar
                .full(fifo_full[i]),
                .empty(fifo_empty[i])
            );

            fifo #(.DATA_WIDTH(18), .ADDR_WIDTH(4)) right_fifo (
                .clk(clk),
                .rst(rst),
                .write_req(write_enables[i]),
                .read_req(read_enables[i]),
                .data_in(xbar_outputs[18*(i+1)-1:18*i]),
               // .data_out(data_out[i*8+:8]),
                .data_out(mem_data_in[i*18+:18]),
                .full(fifo_full[i+8]),
                .empty(fifo_empty[i+8])
            );
            
            assign write_enables[i]=1;
            assign read_enables[i]=1;


            // Memory Banks
            memory_bank #(.DATA_WIDTH(8), .ADDR_WIDTH(6)) mem_bank (
                .clk(clk),
                .rst(rst),
                .write_en(mem_write_en[i]),
                .read_en(mem_read_en[i]),
                .addr(mem_addrs[i]),
                .data_in(mem_data_in[(10+i*18)+:8]),
                .data_out(mem_resp[i*8+:8])
            );

            // Simple control logic for demo purposes
            assign mem_write_en[i] = mem_data_in[9 + 18*i];  // Enable write 
            assign mem_read_en[i] =  mem_data_in[8 + 18*i];   // Enable read 
            assign mem_addrs[i] = mem_data_in[(i*18)+:6]; // Address to write to and read from 
     
      
        end
    endgenerate
    
    

endmodule

module crossbar_switch #(
    parameter N = 3, // Number of inputs/outputs
    parameter WIDTH = 8 // Data width
)(
    input wire [N*WIDTH-1:0] inputs, // Concatenated input data
    input wire [N*$clog2(N)-1:0] select, // Control signals for each output
    output wire [N*WIDTH-1:0] outputs // Concatenated output data
);

// Generate block to instantiate the multiplexers
genvar i;
generate
    for (i = 0; i < N; i = i + 1) begin : gen_mux
        // Select lines for each multiplexer
        wire [$clog2(N)-1:0] sel = select[i*$clog2(N)+:$clog2(N)]; // 
        // Multiplexer for connecting any input to the ith output
        mux #( .WIDTH(WIDTH), .N(N) ) mux_i (
            .data_in(inputs),
            .select(sel),
            .data_out(outputs[i*WIDTH+:WIDTH])
        );
    end
endgenerate

endmodule

// Multiplexer module
module mux #(
    parameter WIDTH = 8, // Data width
    parameter N = 3 // Number of inputs
)(
    input wire [N*WIDTH-1:0] data_in, // Concatenated input data
    input wire [$clog2(N)-1:0] select, // Selection line 
    output wire [WIDTH-1:0] data_out // Output data
);

assign data_out = data_in[select*WIDTH+:WIDTH];

endmodule





module fifo #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 4
) (
    input wire clk,
    input wire rst,
    input wire write_req,
    input wire read_req,
    input wire [DATA_WIDTH-1:0] data_in,
    output reg [DATA_WIDTH-1:0] data_out,
    output wire full,
    output wire empty
);

//wire write_enable, read_enable;

reg [DATA_WIDTH-1:0] memory [2**ADDR_WIDTH-1:0];
reg [ADDR_WIDTH-1:0] write_ptr, read_ptr;
reg [ADDR_WIDTH:0] count;

always @(posedge clk) begin
    if (rst) begin
        write_ptr <= 0;
        read_ptr <= 0;
        //count <= 0;
    end else begin
        if (write_req && !full) begin
            memory[write_ptr] <= data_in;
            write_ptr <= write_ptr + 1;
        end
        if (read_req && !empty) begin
            data_out <= memory[read_ptr];
            read_ptr <= read_ptr + 1;
        end
    end
end

always @(posedge clk) begin
    if (rst) begin
        count <= 0;
    end else begin
        if (write_req && !full && !(read_req && !empty))
            count <= count + 1;
        else if (read_req && !empty && !(write_req && !full))
            count <= count - 1;
    end
end

assign full = (count == (2**ADDR_WIDTH));
assign empty = (count == 0);



endmodule

module memory_bank #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 6  // For 64 addresses
)(
    input wire clk,
    input wire rst,
    input wire write_en,
    input wire read_en,
    input wire [ADDR_WIDTH-1:0] addr,
    input wire [DATA_WIDTH-1:0] data_in,
    output reg [DATA_WIDTH-1:0] data_out
);

    reg [DATA_WIDTH-1:0] mem [2**ADDR_WIDTH-1:0];
    reg [ADDR_WIDTH-1:0] r_addr;     // Register to hold the address for delayed read
    reg r_read_en;                   // Register to hold the read enable signal for one cycle delay

    always @(posedge clk) begin
        if (rst) begin
            data_out <= 0;
            r_addr <= 0;
            r_read_en <= 0;
        end else begin
            // Write operation happens immediately
            if (write_en) begin
                mem[addr] <= data_in;
            end
            
            // Latch the address and read enable signal for the next cycle
            r_addr <= addr;
            r_read_en <= read_en;
            
            // Read operation with a one-cycle delay
            if (r_read_en) begin
                data_out <= mem[r_addr];
            end
        end
    end

endmodule

