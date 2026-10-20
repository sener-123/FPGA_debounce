// Simulation-only Logos2 differential output buffer (legacy filename).
`timescale 1ns/1ps
module GTP_OUTBUFDS #(parameter IOSTANDARD = "TMDS") (
    input wire I, output wire O, output wire OB
);
    assign O = I;
    assign OB = ~I;
endmodule
