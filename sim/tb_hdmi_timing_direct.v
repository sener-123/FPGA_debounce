`timescale 1ns/1ps
module tb_hdmi_timing_direct;
    localparam HT=24, W=8, H=3, VB=5, VS=3;
    reg clk=0,rst_n=0,valid=0,le=0,fe=0;
    always #5 clk=~clk;
    wire hs,vs,de;wire [23:0] rgb;
    hdmi_timing_direct #(.H_TOTAL(HT),.H_SYNC(4),.V_SYNC(VS)) dut(
        .pix_clk(clk),.rst_n(rst_n),.pix_data(16'hffff),.pix_valid(valid),
        .line_end(le),.frame_end(fe),.video_hs(hs),.video_vs(vs),.video_de(de),.video_rgb(rgb));
    integer f,y,x,vs_width=0,vs_pulses=0,blank_hs=0,pixels=0;
    reg vs_d=0,hs_d=0;
    always @(negedge clk) if(rst_n)begin
        if(vs && de)$fatal(1,"VS overlaps active video");
        if(vs)vs_width=vs_width+1;
        if(vs_d && !vs)begin
            if(vs_width!=VS*HT)$fatal(1,"VS width=%0d expected %0d",vs_width,VS*HT);
            vs_width=0;vs_pulses=vs_pulses+1;
        end
        if(hs && !hs_d && vs)blank_hs=blank_hs+1;
        if(de)pixels=pixels+1;
        vs_d=vs;hs_d=hs;
    end
    initial begin
        repeat(3)@(posedge clk);#1;rst_n=1;
        for(f=0;f<3;f=f+1)
        for(y=0;y<H+VB;y=y+1)
        for(x=0;x<HT;x=x+1)begin
            @(posedge clk);#1;
            valid=(y<H && x<W);
            le=(y<H && x==W-1);fe=(y==H-1 && x==W-1);
        end
        @(posedge clk);#1;valid=0;le=0;fe=0;
        repeat(5)@(posedge clk);
        if(pixels!=3*W*H || vs_pulses!=3 || blank_hs<6)
            $fatal(1,"coverage pixels=%0d VS pulses=%0d blank HS=%0d",pixels,vs_pulses,blank_hs);
        $display("=== tb_hdmi_timing_direct PASS ===");$stop;
    end
endmodule
