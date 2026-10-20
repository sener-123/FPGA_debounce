`timescale 1ns/1ps
// Every search candidate, both ping-pong directions, using a textured map.
module tb_motion_search;
    localparam TX=12, TY=10, DEP=256, R=3;
    reg clk=0, rst_n=0, start=0, sel=0;
    always #5 clk=~clk;
    wire [13:0] a0,a1;
    reg [15:0] q0,q1;
    reg [15:0] m0[0:DEP-1],m1[0:DEP-1];
    wire valid; wire [5:0] mx,my;
    motion_estimate #(.H_ACTIVE(TX*16),.V_ACTIVE(TY*16),.RAM_DEPTH(DEP)) dut(
        .px_clk(clk),.rst_n(rst_n),.start(start),.src_sel(sel),
        .ram0_raddr(a0),.ram1_raddr(a1),.ram0_rdata(q0),.ram1_rdata(q1),
        .motion_valid(valid),.mv_x(mx),.mv_y(my));
    always @(posedge clk) begin q0<=m0[a0[7:0]];q1<=m1[a1[7:0]];end
    function [15:0] texture;
        input integer x,y;
        reg [31:0] h;
        begin
            h=(x+17)*32'h45d9f3b ^ (y+29)*32'h119de1f3;
            texture=(h ^ (h>>16)) & 16'h7fff;
        end
    endfunction
    integer dx,dy,x,y,bank,cycles,cases=0;
    reg [15:0] target;
    initial begin
        repeat(4)@(negedge clk);rst_n=1;
        for(bank=0;bank<2;bank=bank+1)
        for(dy=-R;dy<=R;dy=dy+1)
        for(dx=-R;dx<=R;dx=dx+1)begin
            sel=bank;
            for(y=0;y<TY;y=y+1)for(x=0;x<TX;x=x+1)begin
                target=(x-dx>=0 && x-dx<TX && y-dy>=0 && y-dy<TY)
                       ? texture(x-dx,y-dy) : 16'hffff;
                m0[y*TX+x]=bank?target:texture(x,y);
                m1[y*TX+x]=bank?texture(x,y):target;
            end
            @(negedge clk);start=1;@(negedge clk);start=0;
            cycles=0;
            while(!valid)begin
                @(negedge clk);cycles=cycles+1;
                if(cycles>10000)$fatal(1,"motion search timeout");
            end
            if($signed(mx)!=dx || $signed(my)!=dy)
                $fatal(1,"bank=%0d expected (%0d,%0d), got (%0d,%0d)",bank,dx,dy,$signed(mx),$signed(my));
            cases=cases+1;
        end
        $display("=== tb_motion_search PASS (%0d cases) ===",cases);$stop;
    end
endmodule
