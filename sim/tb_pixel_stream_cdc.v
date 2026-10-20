`timescale 1ns/1ps
// Unaligned clock phase, odd blanking lengths, pointer wraps and reset recovery.
module tb_pixel_stream_cdc;
    reg wc=0,rc=0,rst=0,valid=0,eol=0,eof=0;
    reg [15:0] data=0;
    always #5 wc=~wc;
    initial begin #3.5;forever #10 rc=~rc;end
    wire ov,ol,of;wire[15:0] od;
    pixel_stream_cdc #(.AW(4)) dut(
        .wr_clk(wc),.wr_rst_n(rst),.wr_valid(valid),.wr_data(data),.wr_eol(eol),.wr_eof(eof),
        .rd_clk(rc),.rd_rst_n(rst),.rd_valid(ov),.rd_data(od),.rd_eol(ol),.rd_eof(of));
    reg [17:0] expected[0:4095];
    integer wp=0,rp=0,epoch,f,y,x;
    always @(posedge wc) if(rst && valid)begin
        if(dut.full)$fatal(1,"unexpected pixel FIFO overflow");
        expected[wp]={eof,eol,data};wp=wp+1;
    end
    always @(negedge rc) if(rst && ov)begin
        if(rp>=wp || {of,ol,od} !== expected[rp])
            $fatal(1,"CDC sequence mismatch at pixel %0d",rp);
        rp=rp+1;
    end
    initial begin
        for(epoch=0;epoch<2;epoch=epoch+1)begin
            @(negedge wc);rst=0;valid=0;wp=0;rp=0;
            repeat(5)@(negedge wc);rst=1;
            for(f=0;f<3;f=f+1)begin
                for(y=0;y<4;y=y+1)begin
                    for(x=0;x<32;x=x+1)begin
                        valid=1;data=f*128+y*32+x;eol=(x==31);eof=(y==3 && x==31);
                        @(negedge wc);valid=0;eol=0;eof=0;@(negedge wc);
                    end
                    repeat(7)@(negedge wc);
                end
                repeat(101)@(negedge wc);
            end
            repeat(30)@(negedge rc);
            if(rp!=384 || wp!=384)$fatal(1,"CDC missing pixels wr=%0d rd=%0d",wp,rp);
        end
        $display("=== tb_pixel_stream_cdc PASS ===");$stop;
    end
endmodule
