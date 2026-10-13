//=====================================================================
// 测试台  : tb_video_stab_hdmi.v
// 目的    : 验证 video_stab_hdmi(无 DDR3, HDMI 直出)数据通路
// 方法    : sim_cam_source 产生 1280x720 测试图(帧 0~7 静止, 帧 8 起
//           每帧 +16 像素水平漂移), 检查 HDMI 直出:
//              - 每帧 video_de 数 = 1280*720;
//              - 静止帧: X' == (x-9) mod 32(平移补偿基准延迟 9);
//              - 漂移帧: X' == (x-17+dx) mod 32(comp 饱和 +8)。
// 运行    : vsim -do tb_video_stab_hdmi.do
//           (需定义 SIM_SOURCE)
//=====================================================================
`timescale 1ns/1ps
module tb_video_stab_hdmi;

    reg sys_clk  = 1'b0;
    always #10 sys_clk = ~sys_clk;           // 50MHz
    reg cam_pclk = 1'b0;
    always #5  cam_pclk = ~cam_pclk;         // 100MHz(仿真简化)

    reg sys_rst_n = 1'b0;

    wire cam_rst_n, cam_pwdn, cam_scl;
    wire cam_sda;
    wire tmds_clk_p, tmds_clk_n;
    wire [2:0] tmds_data_p, tmds_data_n;
    wire dbg_motion_valid, dbg_comp_valid;
    wire [5:0] dbg_mv_x, dbg_mv_y, dbg_comp_x, dbg_comp_y;

    video_stab_hdmi dut (
        .sys_clk         (sys_clk),
        .sys_rst_n       (sys_rst_n),
        .cam_pclk        (cam_pclk),
        .cam_vsync       (1'b0),
        .cam_href        (1'b0),
        .cam_data        (8'd0),
        .cam_rst_n       (cam_rst_n),
        .cam_pwdn        (cam_pwdn),
        .cam_scl         (cam_scl),
        .cam_sda         (cam_sda),
        .tmds_clk_p      (tmds_clk_p),
        .tmds_clk_n      (tmds_clk_n),
        .tmds_data_p     (tmds_data_p),
        .tmds_data_n     (tmds_data_n),
        .dbg_motion_valid(dbg_motion_valid),
        .dbg_mv_x        (dbg_mv_x),
        .dbg_mv_y        (dbg_mv_y),
        .dbg_comp_valid  (dbg_comp_valid),
        .dbg_comp_x      (dbg_comp_x),
        .dbg_comp_y      (dbg_comp_y)
    );

    //=================================================================
    // HDMI 监控(negedge pixel_clk 采样避免竞争)
    //=================================================================
    integer hdmi_de_cnt;
    integer hdmi_frame;
    reg     hdmi_vs_d;
    integer hdmi_errs = 0;
    integer hdmi_col;
    integer F_obs, Xp, exp_x, dxf;

    initial begin
        hdmi_de_cnt = 0; hdmi_frame = 0; hdmi_vs_d = 0; hdmi_col = 0;
    end

    always @(negedge dut.pixel_clk) begin
        hdmi_vs_d <= dut.u_hdmi_tg.video_vs;
        if (dut.u_hdmi_tg.video_vs && !hdmi_vs_d) begin
            if (hdmi_frame > 3) begin
                if (hdmi_de_cnt != 1280*720) begin
                    $display("ERROR: hdmi frame %0d de count %0d expect %0d",
                             hdmi_frame, hdmi_de_cnt, 1280*720);
                    hdmi_errs = hdmi_errs + 1;
                end
            end
            hdmi_frame = hdmi_frame + 1;
            hdmi_de_cnt = 0;
        end else if (dut.u_hdmi_tg.video_de) begin
            hdmi_col = hdmi_de_cnt - 1;
            hdmi_de_cnt = hdmi_de_cnt + 1;
            if (hdmi_col >= 0 && hdmi_frame > 3 &&
                (hdmi_col/1280 == 100 || hdmi_col/1280 == 300 ||
                 hdmi_col/1280 == 500)) begin
                F_obs = dut.u_hdmi_tg.video_rgb[23:19];
                Xp    = dut.u_hdmi_tg.video_rgb[7:3];
                if ((^dut.u_hdmi_tg.video_rgb) === 1'bx) begin
                    hdmi_errs = hdmi_errs + 1;
                end else if (F_obs < 7) begin
                    exp_x = (hdmi_col - 9) & 31;
                    if (Xp !== exp_x[4:0]) begin
                        $display("ERROR[static] hdmi (%0d,%0d) X'=%0d expect %0d",
                                 hdmi_col%1280, hdmi_col/1280, Xp, exp_x[4:0]);
                        hdmi_errs = hdmi_errs + 1;
                    end
                end else if (F_obs >= 12) begin
                    dxf = (F_obs - 7) << 4;
                    exp_x = (hdmi_col - 17 + dxf) & 31;
                    if (Xp !== exp_x[4:0]) begin
                        $display("ERROR[ramp] hdmi (%0d,%0d) X'=%0d expect %0d (F=%0d)",
                                 hdmi_col%1280, hdmi_col/1280, Xp, exp_x[4:0], F_obs);
                        hdmi_errs = hdmi_errs + 1;
                    end
                end
            end
        end
    end

    //=================================================================
    // 主流程
    //=================================================================
    initial begin
        repeat (20) @(posedge sys_clk);
        sys_rst_n = 1'b1;
        // 覆盖静止帧(F<7)与漂移帧(F>=12), 约 400ms
        #400_000_000;
        if (hdmi_errs == 0)
            $display("=== tb_video_stab_hdmi PASS (hdmi_errs=%0d) ===", hdmi_errs);
        else
            $display("=== tb_video_stab_hdmi FAIL: hdmi=%0d ===", hdmi_errs);
        $stop;
    end

endmodule
