module Multiplier (input  signed [7:0] a, input  signed [7:0] b, output signed [15:0] prod);
	assign prod = a * b;
endmodule
