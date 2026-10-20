`timescale 1ns/1ps
module tb_disp_reader;
    reg clk=0,rst_n=0,load=0,afull=0;
    always #5 clk=~clk;
    wire ren,rload,wen,flush; wire [15:0] wd;
    reg [15:0] data=16'hdead;
    integer next_pixel=0,reads=0,writes=0,f,k;
    disp_reader #(.H_ACTIVE(4),.V_ACTIVE(1),.T_FLUSH(2),.T_START(4)) dut(
        .rd_clk(clk),.rst_n(rst_n),.ddr3_rd_data(data),.ddr3_rd_en(ren),
        .rd_load(rload),.load_hdmi(load),.load_lcd(1'b0),.mode(1'b0),
        .lcd_id(16'd0),.fifo_afull(afull),.fifo_wen(wen),.fifo_wdata(wd),.fifo_flush(flush));
    // A registered FIFO read: the first return is zero, not the reset value.
    always @(posedge clk)begin
        if(rload)begin next_pixel=0; data<=16'hdead;end
        else if(ren)begin
            data<=next_pixel;next_pixel=next_pixel+1;reads=reads+1;
        end
    end
    always @(negedge clk) if(wen)begin
        if(wd !== writes[15:0])$fatal(1,"frame %0d pixel %0d got %h",f,writes,wd);
        writes=writes+1;
    end
    initial begin
        repeat(4)@(negedge clk);rst_n=1;repeat(4)@(negedge clk);
        for(f=0;f<6;f=f+1)begin
            @(negedge clk);reads=0;writes=0;load=1;@(negedge clk);load=0;
            for(k=0;k<50;k=k+1)begin
                @(posedge clk);#1;afull=(f[0] && k%5<2);
            end
            @(posedge clk);#1;afull=0;
            if(reads!=4 || writes!=4)$fatal(1,"frame %0d reads=%0d writes=%0d",f,reads,writes);
        end
        $display("=== tb_disp_reader PASS (6 frames, with backpressure) ===");$stop;
    end
endmodule
