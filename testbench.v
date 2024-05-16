`timescale 1ns / 1ps

module testbench;
    reg clk = 0;
    reg rst = 1;
    reg [71:0] mem_req = 0;
    wire [35:0] mem_resp;
    //wire [31:0] mem_data_out_modified; // Output from memory bank

    // Instantiate the top-level module
    top_module dut (
        .clk(clk),
        .rst(rst),
        .mem_req(mem_req),
        .mem_resp(mem_resp)
    //    .mem_data_out_modified(mem_data_out_modified)
    );

    // Clock generation
    always #5 clk = ~clk; // Generate a clock with a period of 10ns

    initial begin
        // Reset sequence
        #10 rst = 1;
        #10 rst = 0; // Release reset

        // Feed data into the FIFO system
        repeat (1) begin
            @(posedge clk) begin
                  mem_req = 72'b101101111100000011_101101111100000010_101101111100000001_101101111100000000; //testing  memory request operations simultaneously
              //  mem_req = 18'b101101111100000000; //testing 1 memory request operation
              //  data_in = $random; // Generate random memory data input
            end
        end
        
        // Check output from memory
        @(posedge clk);
        $display("Data read from Memory Bank 0: %h", mem_resp);

        // Stop simulation after enough time has passed to observe behavior
        #200;
        $finish;
    end
endmodule
