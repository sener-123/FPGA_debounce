// Verify the generated no-DDR PLL with the PDS GTP_GPLL model.
`timescale 1ns/1ps
module tb_pll_ip;
    reg clkin=0, rst=1;
    always #(1000.0/84.0/2.0) clkin=~clkin;
    wire c0,c1,locked;
    PLL dut(.clkin1(clkin),.rst(rst),.clkout0(c0),.clkout1(c1),.lock(locked));
    realtime t0,t1,p0,p1;
    initial begin
        #100; rst=0;
        wait(locked===1'b1);
        repeat(20) @(posedge c0);
        @(posedge c0); t0=$realtime;
        repeat(100) @(posedge c0);
        p0=($realtime-t0)/100.0;
        @(posedge c1); t1=$realtime;
        repeat(500) @(posedge c1);
        p1=($realtime-t1)/500.0;
        if(p0<23.80 || p0>23.82 || p1<4.75 || p1>4.78)
            $fatal(1,"PLL frequency mismatch: periods %f %f ns",p0,p1);
        $display("PLL_IP PASS: pixel period=%f ns, serial period=%f ns",p0,p1);
        $finish;
    end
    initial begin #10000; $fatal(1,"PLL lock timeout"); end
endmodule
