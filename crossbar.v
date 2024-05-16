module crossbar_switch #(
    parameter N = 3, // Number of inputs/outputs. It is parameterized to make the design scalable
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
