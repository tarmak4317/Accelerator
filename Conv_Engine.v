module ConvEngine (
    input         clk,
    input         rst,
    input  [31:0] DMAport,       // 32-bit input from DMA
    input         acc_enable,    // Enable accumulator update
    input         acc_clear,     // Clear accumulator
    output [31:0] result         // 32-bit accumulated result
);

    /* -----------------------------
       MUX Output Wires
       Format: [24:1] = 24-bit data, [0] = enable
    ----------------------------- */
    wire [24:0] mux_out0, mux_out1, mux_out2, mux_out3, mux_out4, mux_out5;

    MUX demux (
        .DMAport(DMAport),
        .out0(mux_out0),
        .out1(mux_out1),
        .out2(mux_out2),
        .out3(mux_out3),
        .out4(mux_out4),
        .out5(mux_out5)
    );

    /* -----------------------------
       Register Instantiation
       6 registers × 24 bits = 18 bytes total
       Registers 0-2: Pixel values (9 bytes)
       Registers 3-5: Kernel values (9 bytes)
       
       Each register stores 3 bytes:
       Reg0: pixel[0], pixel[1], pixel[2]
       Reg1: pixel[3], pixel[4], pixel[5]
       Reg2: pixel[6], pixel[7], pixel[8]
       Reg3: kernel[0], kernel[1], kernel[2]
       Reg4: kernel[3], kernel[4], kernel[5]
       Reg5: kernel[6], kernel[7], kernel[8]
    ----------------------------- */
    wire [23:0] reg_data [5:0];

    // MUX outputs 0-5 → Registers 0-5
    // mux_out[24:1] = data, mux_out[0] = enable
    
    Register reg0 (
        .clk(clk),
        .rst(rst),
        .enable(mux_out0[0]),
        .datain(mux_out0[24:1]),
        .dataout(reg_data[0])
    );

    Register reg1 (
        .clk(clk),
        .rst(rst),
        .enable(mux_out1[0]),
        .datain(mux_out1[24:1]),
        .dataout(reg_data[1])
    );

    Register reg2 (
        .clk(clk),
        .rst(rst),
        .enable(mux_out2[0]),
        .datain(mux_out2[24:1]),
        .dataout(reg_data[2])
    );

    Register reg3 (
        .clk(clk),
        .rst(rst),
        .enable(mux_out3[0]),
        .datain(mux_out3[24:1]),
        .dataout(reg_data[3])
    );

    Register reg4 (
        .clk(clk),
        .rst(rst),
        .enable(mux_out4[0]),
        .datain(mux_out4[24:1]),
        .dataout(reg_data[4])
    );

    Register reg5 (
        .clk(clk),
        .rst(rst),
        .enable(mux_out5[0]),
        .datain(mux_out5[24:1]),
        .dataout(reg_data[5])
    );

    /* -----------------------------
       Extract Individual Bytes
       Pixels: reg_data[0-2] → 9 pixel bytes
       Kernels: reg_data[3-5] → 9 kernel bytes
    ----------------------------- */
    wire signed [7:0] pixel [8:0];
    wire signed [7:0] kernel [8:0];

    // Pixels from registers 0-2
    assign pixel[0] = reg_data[0][23:16];
    assign pixel[1] = reg_data[0][15:8];
    assign pixel[2] = reg_data[0][7:0];
    assign pixel[3] = reg_data[1][23:16];
    assign pixel[4] = reg_data[1][15:8];
    assign pixel[5] = reg_data[1][7:0];
    assign pixel[6] = reg_data[2][23:16];
    assign pixel[7] = reg_data[2][15:8];
    assign pixel[8] = reg_data[2][7:0];

    // Kernels from registers 3-5
    assign kernel[0] = reg_data[3][23:16];
    assign kernel[1] = reg_data[3][15:8];
    assign kernel[2] = reg_data[3][7:0];
    assign kernel[3] = reg_data[4][23:16];
    assign kernel[4] = reg_data[4][15:8];
    assign kernel[5] = reg_data[4][7:0];
    assign kernel[6] = reg_data[5][23:16];
    assign kernel[7] = reg_data[5][15:8];
    assign kernel[8] = reg_data[5][7:0];

    /* -----------------------------
       Wallace Tree Multipliers
       9 multipliers: pixel[i] × kernel[i]
    ----------------------------- */
    wire signed [15:0] prod [8:0];

    Multiplier mult0 (.a(pixel[0]), .b(kernel[0]), .prod(prod[0]));
    Multiplier mult1 (.a(pixel[1]), .b(kernel[1]), .prod(prod[1]));
    Multiplier mult2 (.a(pixel[2]), .b(kernel[2]), .prod(prod[2]));
    Multiplier mult3 (.a(pixel[3]), .b(kernel[3]), .prod(prod[3]));
    Multiplier mult4 (.a(pixel[4]), .b(kernel[4]), .prod(prod[4]));
    Multiplier mult5 (.a(pixel[5]), .b(kernel[5]), .prod(prod[5]));
    Multiplier mult6 (.a(pixel[6]), .b(kernel[6]), .prod(prod[6]));
    Multiplier mult7 (.a(pixel[7]), .b(kernel[7]), .prod(prod[7]));
    Multiplier mult8 (.a(pixel[8]), .b(kernel[8]), .prod(prod[8]));

    /* -----------------------------
       Sum of Products (Adder Tree)
       Sign-extend 16-bit products to 32-bit
    ----------------------------- */
    wire signed [31:0] prod_ext [8:0];
    
    assign prod_ext[0] = {{16{prod[0][15]}}, prod[0]};
    assign prod_ext[1] = {{16{prod[1][15]}}, prod[1]};
    assign prod_ext[2] = {{16{prod[2][15]}}, prod[2]};
    assign prod_ext[3] = {{16{prod[3][15]}}, prod[3]};
    assign prod_ext[4] = {{16{prod[4][15]}}, prod[4]};
    assign prod_ext[5] = {{16{prod[5][15]}}, prod[5]};
    assign prod_ext[6] = {{16{prod[6][15]}}, prod[6]};
    assign prod_ext[7] = {{16{prod[7][15]}}, prod[7]};
    assign prod_ext[8] = {{16{prod[8][15]}}, prod[8]};

    // Combinational sum of all 9 products
    wire signed [31:0] sum_products;
    assign sum_products = prod_ext[0] + prod_ext[1] + prod_ext[2] +
                          prod_ext[3] + prod_ext[4] + prod_ext[5] +
                          prod_ext[6] + prod_ext[7] + prod_ext[8];

    /* -----------------------------
       32-bit Accumulator
    ----------------------------- */
    reg signed [31:0] accumulator;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            accumulator <= 32'b0;
        end else if (acc_clear) begin
            accumulator <= 32'b0;
        end else if (acc_enable) begin
            accumulator <= accumulator + sum_products;
        end
    end

    assign result = accumulator;

endmodule
