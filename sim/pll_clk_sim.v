// Behavioral PLL model. SIM_CAMERA_PLL selects cam_pclk -> /2 and *2.5.
// Otherwise the DDR3 test configuration uses clkin1 -> 1x and 5x.
// SIM_FAST may freeze only the DDR3 serial clock; TMDS is tested separately.
`timescale 1ns/1ps
module PLL(input wire rst, clkin1,
           output wire clkout0, clkout1, lock);
    realtime previous_edge = 0, input_period = 20.0;
    always @(posedge clkin1) begin
        if (previous_edge != 0) input_period = $realtime - previous_edge;
        previous_edge = $realtime;
    end
    reg pixel_div = 0;
    always @(posedge clkin1 or posedge rst)
        if (rst) pixel_div <= 0;
        else pixel_div <= ~pixel_div;
    reg serial_clk = 0;
`ifdef SIM_CAMERA_PLL
    assign clkout0 = pixel_div;
    always begin
        #(input_period / 5.0);
        serial_clk = rst ? 1'b0 : ~serial_clk;
    end
`else
    assign clkout0 = clkin1;
`ifndef SIM_FAST
    always begin
        #(input_period / 10.0);
        serial_clk = rst ? 1'b0 : ~serial_clk;
    end
`endif
`endif
    assign clkout1 = serial_clk;
    reg [5:0] lock_count = 0;
    always @(posedge clkin1 or posedge rst)
        if (rst) lock_count <= 0;
        else if (lock_count != 63) lock_count <= lock_count + 1'b1;
    assign lock = !rst && (lock_count == 63);
endmodule
