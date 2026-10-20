`timescale 1ns/1ps
// Static then moving textured frames; VSYNC immediately follows the final pixel.
// This exercises the last tile-row RAM drain and uninitialized first-frame bank.
module tb_stab_core;
    localparam W=128,H=128;
    reg clk=0,rst_n=0,de=0,sof=0,eol=0;
    reg [15:0] data=0;
    always #5 clk=~clk;
    wire mv_valid,cv;wire[5:0] mx,my,cx,cy;
    stab_core #(.H_ACTIVE(W),.V_ACTIVE(H),.TILE_RAM_DEPTH(128)) dut(
        .px_clk(clk),.rst_n(rst_n),.px_de(de),.px_data(data),.px_sof(sof),.px_eol(eol),
        .motion_valid(mv_valid),.mv_x(mx),.mv_y(my),.comp_valid(cv),.comp_x(cx),.comp_y(cy));
    integer f,x,y,motions=0,comps=0;
    always @(negedge clk)begin
        if(mv_valid)begin
            if(mx !== (f<4 ? 6'd0 : 6'd1) || my !== 6'd0)
                $fatal(1,"frame %0d motion (%h,%h), wrong ping-pong direction",f,mx,my);
            motions=motions+1;
        end
        if(cv)begin
            if(cx !== (f<4 ? 6'd0 : -6'sd8) || cy !== 6'd0)
                $fatal(1,"frame %0d correction (%h,%h)",f,cx,cy);
            comps=comps+1;
        end
    end
    initial begin
        repeat(4)@(posedge clk);#1;rst_n=1;
        for(f=0;f<8;f=f+1)begin
            for(y=0;y<H;y=y+1)begin
                for(x=0;x<W;x=x+1)begin
                    @(posedge clk);#1;
                    de=1;eol=(x==W-1);
                    data=(((x/16-(f<4?0:f-3)+3)*1733)^((y/16+7)*7919));
                    @(posedge clk);#1;de=0;eol=0;
                end
                if(y==H-1)begin
                    sof=1;@(posedge clk);#1;sof=0;
                end
                repeat(20)@(posedge clk);#1;
            end
            repeat(6000)@(posedge clk);#1;
            if(motions!=f || comps!=f)
                $fatal(1,"frame %0d expected %0d estimates, got motion=%0d comp=%0d",f,f,motions,comps);
        end
        $display("=== tb_stab_core PASS (startup, early VSYNC and motion direction) ===");$stop;
    end
endmodule
