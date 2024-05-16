module top_module (
    input wire clk,
    input wire rst,
    input wire [71:0] mem_req, //8 bits of data, 1 bit of wr_en, 1 bit of rd_en, 8 bits of address
    output wire [35:0] mem_resp  // 8 bits of data and 1 bit of status

);
    wire [71:0] xbar_inputs;
    wire [71:0] xbar_outputs;
    wire [7:0] fifo_full, fifo_empty;
    wire [7:0] write_enables, read_enables;
    
    wire [3:0] req_array;
    reg [7:0] select;
//  wire [7:0] select_fin;
    



    // Memory control signals
    wire [3:0] mem_write_en, mem_read_en;
    wire [5:0] mem_addrs [3:0];
    wire [71:0] mem_data_in;
  //wire [31:0] mem_data_out;

    // Instantiate crossbar switch
    crossbar_switch #(.N(4), .WIDTH(18)) c_b (
        .inputs(xbar_inputs),
        .select(select),
     // .select(8'b11100100), //hardcoded for initial phase of testing
        .outputs(xbar_outputs)
    );
    

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

/*
Logic used for testing single memory request operation
// wr_en, rd_en = (8,9), (26,27), (44, 45), (62, 63)

    assign req_array[0] = mem_req[8] || mem_req[9];
    assign req_array[1] = mem_req[26] || mem_req[27];
    assign req_array[2] = mem_req[44] || mem_req[45];
    assign req_array[3] = mem_req[62] || mem_req[63];
    
// if req_array[0] --> first always block    
    
    always @(*) begin
      
      if(req_array[0] == 1) begin
        
      case(xbar_inputs[7:6]) 
        2'b00: select[1:0] = 2'b00;
        2'b01: select[3:2] = 2'b01;
        2'b10: select[5:4] = 2'b10;
        2'b11: select[7:6] = 2'b11;
        default: select = 0;
      endcase
        
      end
      
    if(req_array[1] == 1 && req_array[0] == 0 ) begin  
  
      case(xbar_inputs[25:24])
        2'b00: select[1:0] = 2'b00;
        2'b01: select[3:2] = 2'b01;
        2'b10: select[5:4] = 2'b10;
        2'b11: select[7:6] = 2'b11;
        default: select = 0;
      endcase
          
    end
    
    if(req_array[2] == 1 && req_array[1] == 0 && req_array[0] == 0) begin  
  
      case(xbar_inputs[43:42])
        2'b00: select[1:0] = 2'b00;
        2'b01: select[3:2] = 2'b01;
        2'b10: select[5:4] = 2'b10;
        2'b11: select[7:6] = 2'b11;
        default: select = 0;
      endcase
          
    end
    

    
    if(req_array[3] == 1 && req_array[2] == 0 && req_array[1] == 0 && req_array[0] == 0) begin  
      
      case(xbar_inputs[61:60])
        2'b00: select[1:0] = 2'b00;
        2'b01: select[3:2] = 2'b01;
        2'b10: select[5:4] = 2'b10;
        2'b11: select[7:6] = 2'b11;
        default: select = 0;
      endcase
          
    end 
  end   

assign select_fin = select;
*/

    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : left_fifos
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
                .full(fifo_full[i+4]),
                .empty(fifo_empty[i+4])
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

            //  Control logic for demo purposes
            assign mem_write_en[i] = mem_data_in[9 + 18*i];  // Enable write 
            assign mem_read_en[i] =  mem_data_in[8 + 18*i];   // Enable read 
            assign mem_addrs[i] = mem_data_in[(i*18)+:6]; // Address to write to and read from 
     
      
        end
    endgenerate
    
    

endmodule
