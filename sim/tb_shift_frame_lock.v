`timescale 1ns/1ps
// Pending correction may arrive after SOF; the active mask and address must agree.
module tb_shift_frame_lock;
    reg clk=0,rst_n=0,de=0,sof=0,cv=0;
    reg [5:0] cx=0,cy=0;
    reg [15:0] data=0;
    always #5 clk=~clk;
    wire ov,os,ol,of;wire[15:0] od;
    shift_compensate #(.H_ACTIVE(64),.V_ACTIVE(64),.RING_DEPTH(2048)) dut(
        .px_clk(clk),.rst_n(rst_n),.px_de(de),.px_data(data),.px_sof(sof),
        .comp_valid(cv),.comp_x(cx),.comp_y(cy),
        .crop_col0(12'd0),.crop_row0(12'd0),.crop_w(12'd64),.crop_h(12'd64),
        .px_de_o(ov),.px_data_o(od),.px_sof_o(os),.px_eol_o(ol),.px_eof_o(of));
    integer f,x,y,n=0,markers=0;
    always @(negedge clk)if(ov)begin
        if(os)n=0;
        if(od!=0)begin
            if(n%64 != (f==0?11:19) || n/64!=29)
                $fatal(1,"active frame offset changed at (%0d,%0d)",n%64,n/64);
            markers=markers+1;
        end
        n=n+1;
    end
    initial begin
        repeat(4)@(posedge clk);#1;rst_n=1;
        for(f=0;f<2;f=f+1)begin
            sof=1;@(posedge clk);#1;sof=0;
            for(y=0;y<64;y=y+1)begin
                for(x=0;x<64;x=x+1)begin
                    de=1;data=(x==2 && y==20)?16'hffff:0;
                    if(f==0 && y==10 && x==0)begin cv=1;cx=8;end
                    @(posedge clk);#1;de=0;cv=0;@(posedge clk);#1;
                end
                repeat(4)@(posedge clk);#1;
            end
            repeat(8)@(posedge clk);#1;
        end
        if(markers!=2)$fatal(1,"marker was clipped by a pending correction");
        $display("=== tb_shift_frame_lock PASS ===");$stop;
    end
endmodule
