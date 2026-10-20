`timescale 1ns/1ps
// A rightward image motion must be reduced by the smooth->shift pipeline.
module tb_compensation_direction;
    reg clk=0, rst_n=0, de=0, sof=0, motion_valid=0;
    reg [15:0] data=0;
    always #5 clk=~clk;
    wire comp_valid;
    wire [5:0] comp_x,comp_y;
    wire out_de,out_sof,out_eol,out_eof;
    wire [15:0] out_data;
    trajectory_smooth smooth(
        .px_clk(clk),.rst_n(rst_n),.motion_valid(motion_valid),
        .mv_x(6'd1),.mv_y(6'd0),.comp_valid(comp_valid),.comp_x(comp_x),.comp_y(comp_y));
    shift_compensate #(.H_ACTIVE(96),.V_ACTIVE(48),.RING_DEPTH(4096)) dut(
        .px_clk(clk),.rst_n(rst_n),.px_de(de),.px_data(data),.px_sof(sof),
        .comp_valid(comp_valid),.comp_x(comp_x),.comp_y(comp_y),
        .crop_col0(12'd0),.crop_row0(12'd0),.crop_w(12'd96),.crop_h(12'd48),
        .px_de_o(out_de),.px_data_o(out_data),.px_sof_o(out_sof),
        .px_eol_o(out_eol),.px_eof_o(out_eof));
    integer f,x,y,n=0,markers=0;
    always @(negedge clk) if(out_de)begin
        if(out_sof)n=0;
        if(out_data!=0)begin
            // Baseline offset is 9. The input moves 16, corrected output moves 8.
            if((f==1 && n%96!=49) || (f==2 && n%96!=57))
                $fatal(1,"compensation amplified motion");
            markers=markers+1;
        end
        n=n+1;
    end
    initial begin
        repeat(3)@(negedge clk);rst_n=1;
        for(f=1;f<=2;f=f+1)begin
            if(f==2)begin
                motion_valid=1;@(negedge clk);motion_valid=0;
                repeat(4)@(negedge clk);
            end
            sof=1;@(negedge clk);sof=0;repeat(3)@(negedge clk);
            for(y=0;y<48;y=y+1)begin
                for(x=0;x<96;x=x+1)begin
                    de=1;data=(x==40+(f-1)*16 && y==20)?16'hffff:0;
                    @(negedge clk);de=0;@(negedge clk);
                end
                repeat(10)@(negedge clk);
            end
            repeat(10)@(negedge clk);
        end
        if(markers!=2)$fatal(1,"missing marker coverage");
        $display("=== tb_compensation_direction PASS ===");$stop;
    end
endmodule
