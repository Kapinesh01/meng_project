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

/*

For combinational read and write conditions
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

    always @(posedge clk) begin
        if (rst) begin
            data_out <= 0;
        end else begin
            if (write_en) begin
                mem[addr] <= data_in;
            end
            if (read_en) begin
                data_out <= mem[addr];
            end
        end
    end

endmodule
*/