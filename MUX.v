module MUX(input [31:0] DMAport, output reg [24:0] out0, out1, out2, out3, out4, out5);
	always @ (*) begin
		out0 = 25'b0;
		out1 = 25'b0;
		out2 = 25'b0;
		out3 = 25'b0;
		out4 = 25'b0;
		out5 = 25'b0;
		case(DMAport[2:0])
			3'b000: out0 = DMAport[31:7];
			3'b001: out1 = DMAport[31:7];
			3'b010: out2 = DMAport[31:7];
			3'b011: out3 = DMAport[31:7];
			3'b100: out4 = DMAport[31:7];
			3'b101: out5 = DMAport[31:7];
			default:;
		endcase
	end
endmodule
