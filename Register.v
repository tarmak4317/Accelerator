module Register(input clk, input rst, input enable, input [23:0] datain, output reg [23:0] dataout);
	always @ (posedge clk or posedge rst) begin
		if (rst)
			dataout <= 24'b0;
		else if(enable)
			dataout <= datain;
	end
endmodule
