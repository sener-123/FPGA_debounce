//=====================================================================
// 测试台  : tb_motion_estimate.v
// 目的    : 验证全网格 SAD 全局平移估计
// 方法    : 块图用互质线性纹理 p(tx,ty) = tx*37 + ty*101 (低 16 位),
//           目标块图 = 源块图平移 (2,-1) 块, 期望估计出 mv = (-2,+1)。
// 运行    : 在 ModelSim 中编译 rtl/motion_estimate.v 与本文件后
//           vsim tb_motion_estimate -do "run -all"
//=====================================================================
`timescale 1ns/1ps
module tb_motion_estimate;

    localparam W = 640, V = 480, T = 16, R = 3, DEP = 4096;
    localparam TX = W/T;          // 40
    localparam TY = V/T;          // 30
    localparam TILES = TX*TY;     // 1200

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    reg        start;
    reg        src_sel;
    wire [13:0] me_ra0, me_ra1;
    wire        motion_valid;
    wire [5:0]  mv_x, mv_y;
    reg  [15:0] ram0_q_r, ram1_q_r;   // 先声明, 供端口连接使用

    motion_estimate #(
        .H_ACTIVE (W), .V_ACTIVE (V), .TILE (T), .SEARCH_R (R),
        .RAM_DEPTH(DEP)
    ) dut (
        .px_clk(clk), .rst_n(rst_n), .start(start), .src_sel(src_sel),
        .ram0_raddr(me_ra0), .ram1_raddr(me_ra1),
        .ram0_rdata(ram0_q_r), .ram1_rdata(ram1_q_r),
        .motion_valid(motion_valid), .mv_x(mv_x), .mv_y(mv_y)
    );

    //-----------------------------------------------------------------
    // 模拟两块图 RAM(由综合器映射 BRAM 的模式)
    //-----------------------------------------------------------------
    reg [15:0] ram0 [0:DEP-1];
    reg [15:0] ram1 [0:DEP-1];
    always @(posedge clk) begin
        ram0_q_r <= ram0[me_ra0[11:0]];
        ram1_q_r <= ram1[me_ra1[11:0]];
    end

    integer tx, ty, i;
    reg [15:0] pval;

    function [15:0] pat(input integer a, input integer b);
        begin
            pat = (a*37 + b*101) & 16'hFFFF;
        end
    endfunction

    integer errs = 0;

    initial begin
        start = 0; src_sel = 0;
        // 初始化块图
        for (ty = 0; ty < TY; ty = ty + 1)
            for (tx = 0; tx < TX; tx = tx + 1)
                ram0[ty*TX+tx] = pat(tx, ty);
        for (ty = 0; ty < TY; ty = ty + 1)
            for (tx = 0; tx < TX; tx = tx + 1) begin
                // ram1 = ram0 平移 (dx0=2, dy0=-1): ram1(tx,ty)=ram0(tx+2,ty-1)
                if ((tx+2 < TX) && (ty-1 >= 0))
                    ram1[ty*TX+tx] = pat(tx+2, ty-1);
                else
                    ram1[ty*TX+tx] = pat(tx, ty);
            end
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        // 启动运动估计
        @(negedge clk); start <= 1;
        @(negedge clk); start <= 0;

        // 等待结果
        i = 0;
        while (!motion_valid) begin
            @(posedge clk);
            i = i + 1;
            if (i > 100000) begin
                $display("ERROR: timeout waiting motion_valid");
                errs = errs + 1;
                $stop;
            end
        end
        $display("motion_valid after %0d cycles, mv = (%0d, %0d)", i, mv_x, mv_y);
        if (mv_x !== -6'sd2 || mv_y !== 6'sd1) begin
            $display("ERROR: expect mv = (-2, +1)");
            errs = errs + 1;
        end

        if (errs == 0)
            $display("=== tb_motion_estimate PASS ===");
        else
            $display("=== tb_motion_estimate FAIL: %0d errors ===", errs);
        $stop;
    end

endmodule
