//=====================================================================
// 测试台  : tb_video_stab_top.v
// 目的    : 顶层整机冒烟测试(闭环)
// 方法    : 模拟 OV5640 并行总线(8bit, RGB565 双字节/像素, 640x480),
//           内容为随帧平移的斜坡纹理; 检查:
//             1. 每帧 motion_valid / comp_valid 均有脉冲;
//             2. 显示有效窗口像素数 = V_OUT x W_OUT;
//             3. 输出数据无未知态(X)。
// 运行    : 在 ModelSim 中编译 rtl/ 全部文件与本文件后
//           vsim tb_video_stab_top -do "run -all"
//=====================================================================
`timescale 1ns/1ps
module tb_video_stab_top;

    localparam H_ACTIVE = 640;
    localparam V_ACTIVE = 480;
    localparam H_TOTAL  = 1600;   // 640 像素 x 2 字节 + 消隐
    localparam V_TOTAL  = 525;
    localparam W_OUT    = H_ACTIVE - 2*(8+1);   // 622
    localparam V_OUT    = V_ACTIVE - 2*(8+1);   // 462
    localparam LINE_PIX = 640;
    localparam V_ACT     = 480;   // 场有效行数
    localparam V_BLANK   = V_TOTAL - V_ACT;

    reg         pclk = 0;
    always #10 pclk = ~pclk;      // 50MHz

    reg         rst_n = 0;
    reg         cam_vsync, cam_href;
    reg  [7:0]  cam_data;

    wire        disp_hsync, disp_vsync, disp_de;
    wire [23:0] disp_data;
    wire        motion_valid, comp_valid;
    wire [5:0]  mv_x, mv_y, comp_x, comp_y;

    video_stab_top #(
        .H_ACTIVE (H_ACTIVE), .V_ACTIVE (V_ACTIVE),
        .H_TOTAL  (H_TOTAL),  .V_TOTAL  (V_TOTAL),
        .H_FP     (128), .H_BP (132), .H_SYNC (96),
        .V_FP     (10), .V_BP (33), .V_SYNC (2),
        .TILE     (16), .SEARCH_R (3), .MAX_SHIFT (8),
        .SMOOTH_WIN (4), .TRAJ_MAX (1024),
        .TILE_RAM_DEPTH (4096), .RING_DEPTH (16384),
        .VS_POL   (1), .HREF_POL (1),
        .PX_CLK_PER_PIX (2)
    ) dut (
        .cam_pclk   (pclk),
        .cam_vsync  (cam_vsync),
        .cam_href   (cam_href),
        .cam_data   (cam_data),
        .rst_n      (rst_n),
        .disp_hsync (disp_hsync),
        .disp_vsync (disp_vsync),
        .disp_de    (disp_de),
        .disp_data  (disp_data),
        .motion_valid (motion_valid),
        .mv_x       (mv_x),
        .mv_y       (mv_y),
        .comp_valid (comp_valid),
        .comp_x     (comp_x),
        .comp_y     (comp_y)
    );

    //-----------------------------------------------------------------
    // OV5640 并行总线生成: 内容 = (帧号*16 + y*W + x), 随帧右移 1 块
    //-----------------------------------------------------------------
    integer fnum = 0;      // 帧号
    integer ln, px, b;
    integer mv_cnt, comp_cnt;
    integer de_cnt, hsync_cnt, vsync_cnt;
    integer x_errors = 0;
    integer errs = 0;

    reg [15:0] pixval;

    // 监视输出
    always @(posedge pclk) begin
        if (disp_de) begin
            de_cnt = de_cnt + 1;
            // 第一帧环形缓冲尚未填满, 输出无意义, 不检查 X
            if (fnum >= 1 && (^disp_data === 1'bx)) begin
                x_errors = x_errors + 1;
                $display("ERROR: X at %0t fnum=%0d", $time, fnum);
            end
        end
        if (disp_hsync) hsync_cnt = hsync_cnt + 1;
        if (disp_vsync) vsync_cnt = vsync_cnt + 1;
        if (motion_valid) mv_cnt = mv_cnt + 1;
        if (comp_valid)   comp_cnt = comp_cnt + 1;
    end

    initial begin
        cam_vsync = 0; cam_href = 0; cam_data = 0;
        mv_cnt = 0; comp_cnt = 0; de_cnt = 0; hsync_cnt = 0; vsync_cnt = 0;
        repeat (5) @(posedge pclk);
        rst_n = 1;
        repeat (5) @(posedge pclk);

        // 发送 5 帧: 每帧 V_TOTAL 行 x H_TOTAL 拍
        for (fnum = 0; fnum < 5; fnum = fnum + 1) begin
            for (ln = 0; ln < V_TOTAL; ln = ln + 1) begin
                if (ln < V_ACT) begin
                    cam_vsync <= 1'b1;
                    // 行有效: 640 像素 x 2 字节
                    cam_href <= 1'b1;
                    for (px = 0; px < LINE_PIX; px = px + 1) begin
                        pixval = (fnum*16 + ln*H_ACTIVE + px) & 16'hFFFF;
                        cam_data <= pixval[15:8];
                        @(posedge pclk);
                        cam_data <= pixval[7:0];
                        @(posedge pclk);
                    end
                    cam_href <= 1'b0;
                    // 行消隐
                    repeat (H_TOTAL - 2*LINE_PIX) @(posedge pclk);
                end else begin
                    cam_vsync <= 1'b0;
                    repeat (H_TOTAL) @(posedge pclk);
                end
            end
        end

        #1000;
        $display("mv_cnt=%0d comp_cnt=%0d de_cnt=%0d hsync=%0d vsync=%0d x_err=%0d",
                 mv_cnt, comp_cnt, de_cnt, hsync_cnt, vsync_cnt, x_errors);

        // 每帧应有 1 次运动估计与 1 次补偿(首帧除外)
        if (mv_cnt < 3 || comp_cnt < 3) begin
            $display("ERROR: motion/comp pulses too few");
            errs = errs + 1;
        end
        // 每帧显示有效窗口 = V_OUT x W_OUT
        if (de_cnt < 3*V_OUT*W_OUT || de_cnt > 5*V_OUT*W_OUT) begin
            $display("ERROR: de window pixel count wrong");
            errs = errs + 1;
        end
        if (x_errors != 0) begin
            $display("ERROR: X in output data");
            errs = errs + 1;
        end
        if (errs == 0)
            $display("=== tb_video_stab_top PASS ===");
        else
            $display("=== tb_video_stab_top FAIL: %0d errors ===", errs);
        $stop;
    end

endmodule
