// Simulation-only GTP_ODDR_E1 SAME_EDGE / SYNC_RESET model.
// Matches PDS output pipeline: a captured pair is emitted next clock period.
// Historical filename retained for existing simulation scripts.
`timescale 1ns/1ps
module GTP_ODDR_E1 #(
    parameter GRS_EN = "TRUE",
    parameter ODDR_MODE = "SAME_EDGE",
    parameter RS_TYPE = "SYNC_RESET"
)(output wire Q, input wire D0, D1, CLK, RS, CE);
    reg d0_hold = 0, d1_hold = 0, rise_bit = 0, fall_bit = 0;
    always @(posedge CLK)
        if (RS) begin d0_hold <= 0; d1_hold <= 0; fall_bit <= 0; end
        else if (CE) begin
            d0_hold <= D0; d1_hold <= D1; fall_bit <= d1_hold;
        end
    always @(negedge CLK)
        if (RS) rise_bit <= 0;
        else if (CE) rise_bit <= d0_hold;
    assign Q = (CLK && CE) ? rise_bit : fall_bit;
endmodule
