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
        count <= 0;
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

//count keeps track of read and write operations in the FIFO
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
