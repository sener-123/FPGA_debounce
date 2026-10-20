//=====================================================================
// 测试台  : tb_video_stab_sys.v
// 目的    : 验证 video_stab_sys 全系统数据通路
// 方法    : 1. HDMI 阶段(默认): sim_cam_source 产生 1280x720 测试图
//              (像素 = {帧号[5:0], y'[4:0], x'[4:0]}, 帧 0~1 静止,
//              帧 2 起每帧水平漂移 +16 像素), 检查 HDMI 输出:
//              - 每帧 video_de 数 = 1280*720;
//              - 使用 RGB565 的完整6位帧号, 按生效补偿核对各帧;
//              - 漂移从帧2开始, 按每帧实际生效的补偿量核对内容;
//           2. 按键切换到 LCD 阶段: 检查 lcd_rgb_top 的 data_in:
//              每帧 800*480 个像素, 内容为 720p 帧居中裁剪窗口
//              (col0=240, row0=120) 的平移结果。
// 运行    : ModelSim 编译 rtl 全部模块 + sim 模型后
//           vsim -L work tb_video_stab_sys -do "run -all"
//           (需定义 SIM_SOURCE / SIM_DDR3)
//=====================================================================
`timescale 1ns/1ps
module tb_video_stab_sys;

    reg sys_clk  = 1'b0;
    always #10 sys_clk = ~sys_clk;           // 50MHz
    reg cam_pclk = 1'b0;
    always #5 cam_pclk = ~cam_pclk;          // 100MHz

    reg sys_rst_n = 1'b0;
    reg key_mode  = 1'b1;                    // KEY_POL=0: 松开为高

    wire led_mode;
    wire cam_rst_n, cam_pwdn, cam_scl;
    wire cam_sda;
    wire mem_rst_n, mem_ck, mem_ck_n, mem_cke, mem_ras_n, mem_cas_n;
    wire mem_we_n, mem_odt, mem_cs_n;
    wire [14:0] mem_a;
    wire [2:0]  mem_ba;
    wire [3:0]  mem_dqs, mem_dqs_n, mem_dm;
    wire [31:0] mem_dq;
    wire lcd_hs, lcd_vs, lcd_de, lcd_bl, lcd_rst, lcd_pclk;
    wire [23:0] lcd_rgb;
    wire tmds_clk_p, tmds_clk_n;
    wire [2:0] tmds_data_p, tmds_data_n;
    wire dbg_motion_valid, dbg_comp_valid;
    wire [5:0] dbg_mv_x, dbg_mv_y, dbg_comp_x, dbg_comp_y;

    // LCD ID 上拉: {R7,G7,B7} = 3'b001 -> 7084(7寸 800x480)
    reg id_pull = 1'b1;
    assign lcd_rgb = id_pull ? 24'h800000 : 24'hz;

    video_stab_sys dut (
        .sys_clk        (sys_clk),
        .sys_rst_n      (sys_rst_n),
        .key_mode       (key_mode),
        .led_mode       (led_mode),
        .cam_pclk       (cam_pclk),
        .cam_vsync      (1'b0),
        .cam_href       (1'b0),
        .cam_data       (8'd0),
        .cam_rst_n      (cam_rst_n),
        .cam_pwdn       (cam_pwdn),
        .cam_scl        (cam_scl),
        .cam_sda        (cam_sda),
        .mem_rst_n      (mem_rst_n),
        .mem_ck         (mem_ck),
        .mem_ck_n       (mem_ck_n),
        .mem_cke        (mem_cke),
        .mem_ras_n      (mem_ras_n),
        .mem_cas_n      (mem_cas_n),
        .mem_we_n       (mem_we_n),
        .mem_odt        (mem_odt),
        .mem_cs_n       (mem_cs_n),
        .mem_a          (mem_a),
        .mem_ba         (mem_ba),
        .mem_dqs        (mem_dqs),
        .mem_dqs_n      (mem_dqs_n),
        .mem_dq         (mem_dq),
        .mem_dm         (mem_dm),
        .lcd_hs         (lcd_hs),
        .lcd_vs         (lcd_vs),
        .lcd_de         (lcd_de),
        .lcd_rgb        (lcd_rgb),
        .lcd_bl         (lcd_bl),
        .lcd_rst        (lcd_rst),
        .lcd_pclk       (lcd_pclk),
        .tmds_clk_p     (tmds_clk_p),
        .tmds_clk_n     (tmds_clk_n),
        .tmds_data_p    (tmds_data_p),
        .tmds_data_n    (tmds_data_n),
        .dbg_motion_valid(dbg_motion_valid),
        .dbg_mv_x       (dbg_mv_x),
        .dbg_mv_y       (dbg_mv_y),
        .dbg_comp_valid (dbg_comp_valid),
        .dbg_comp_x     (dbg_comp_x),
        .dbg_comp_y     (dbg_comp_y)
    );

    // Record the signed correction applied to each source frame. This
    // checks display transport independently of the separately tested estimator.
    integer frame_comp[0:63];
    integer ci;
    initial for(ci=0;ci<64;ci=ci+1)frame_comp[ci]=0;
    always @(posedge cam_pclk) if(dut.px_sof)
        frame_comp[(dut.u_sim_cam.frame_cnt+1)&63]=$signed(dut.u_shift.comp_x_r);

    defparam dut.u_sim_cam.DRIFT_START_FRAME = 2;

    //=================================================================
    // HDMI 监控(negedge 采样避免竞争): 帧号/行列/像素校验
    //=================================================================
    integer hdmi_de_cnt;      // 本帧 de 计数(第 j 个 de 显示列 j-1)
    integer hdmi_frame;       // 显示帧号(HDMI 60Hz 计数)
    reg     hdmi_vs_d;
    integer hdmi_errs = 0;
    integer hdmi_x_errs = 0;
    integer hdmi_col;
    integer F_obs, Xp, exp_x, dxf;

    initial begin
        hdmi_de_cnt = 0; hdmi_frame = 0; hdmi_vs_d = 0; hdmi_col = 0;
    end

    always @(negedge dut.clk_hdmi_pix) begin
        hdmi_vs_d <= dut.u_hdmi_tg.video_vs;
        if (dut.u_hdmi_tg.video_vs && !hdmi_vs_d) begin
            // 帧结束统计
            if (hdmi_frame > 2 && dut.mode == 1'b0) begin
                if (hdmi_de_cnt != 1280*720) begin
                    $display("ERROR: hdmi frame %0d de count %0d expect %0d",
                             hdmi_frame, hdmi_de_cnt, 1280*720);
                    hdmi_errs = hdmi_errs + 1;
                end
            end
            hdmi_frame = hdmi_frame + 1;
            hdmi_de_cnt = 0;
        end else if (dut.u_hdmi_tg.video_de && dut.mode == 1'b0) begin
            hdmi_col = hdmi_de_cnt;      // 本拍显示的是列 hdmi_col
            hdmi_de_cnt = hdmi_de_cnt + 1;
            // 只校验部分行(行 = de_cnt/1280)
            if ((hdmi_col%1280==0 || hdmi_col%1280>=17) && hdmi_frame > 2 &&
                (hdmi_col/1280 == 100 || hdmi_col/1280 == 300 ||
                 hdmi_col/1280 == 500)) begin
                F_obs = {dut.u_hdmi_tg.video_rgb[23:19],dut.u_hdmi_tg.video_rgb[15]};
                Xp    = dut.u_hdmi_tg.video_rgb[7:3];   // B 场在 [7:3]
                if ((^dut.u_hdmi_tg.video_rgb) === 1'bx) begin
                    hdmi_x_errs = hdmi_x_errs + 1;
                end else begin
                    dxf = (F_obs < 2) ? 0 : (F_obs-1)*16;
                    exp_x = (hdmi_col%1280 < 9+frame_comp[F_obs]) ? 0
                          : (hdmi_col - 9-frame_comp[F_obs] + dxf) & 31;
                    if (Xp !== exp_x[4:0]) begin
                        $display("ERROR[hdmi] F=%0d (%0d,%0d) X=%0d expect=%0d",F_obs,hdmi_col%1280,hdmi_col/1280,Xp,exp_x);
                        hdmi_errs=hdmi_errs+1;
                        if(hdmi_errs>20)$fatal(1,"HDMI system content mismatch");
                    end
                end
            end
        end
    end

    //=================================================================
    // LCD 监控: 检查 lcd_rgb_top 的 data_in(negedge lcd_clk 采样)
    //=================================================================
    integer lcd_de_cnt;       // 本帧 de 计数
    integer lcd_row, lcd_col;
    reg     lcd_vs_d;
    integer lcd_errs = 0;
    integer lcd_frame = 0;
    integer lcd_F, lcd_Xp, lcd_exp, lcd_dxf;

    initial begin
        lcd_de_cnt = 0; lcd_row = 0; lcd_col = 0; lcd_vs_d = 0;
    end

    always @(negedge dut.u_lcd.lcd_clk or posedge dut.switch_rst) begin
        if (dut.switch_rst || !dut.mode) begin
            lcd_frame=0;lcd_de_cnt=0;lcd_row=0;lcd_col=0;lcd_vs_d=0;
        end else begin
        lcd_vs_d <= dut.u_lcd.out_vsync;
        if (dut.u_lcd.out_vsync && !lcd_vs_d) begin
            if (lcd_frame > 2 && dut.mode == 1'b1) begin
                if (lcd_de_cnt != 800*480) begin
                    $display("ERROR: lcd frame %0d de count %0d expect %0d",
                             lcd_frame, lcd_de_cnt, 800*480);
                    lcd_errs = lcd_errs + 1;
                end
            end
            lcd_frame = lcd_frame + 1;
            lcd_de_cnt = 0;
            lcd_row = 0;
            lcd_col = 0;
        end else if (dut.lcd_de && dut.mode == 1'b1) begin
            // 本拍 data_in = 窗口列 lcd_col 的像素
            if (lcd_frame > 2 && lcd_row < 480) begin
                if (lcd_row == 100 || lcd_row == 300 || lcd_row == 450) begin
                    if (lcd_col == 0 || lcd_col == 400 || lcd_col == 799) begin
                        lcd_F  = dut.u_lcd.data_in[15:10];
                        lcd_Xp = dut.u_lcd.data_in[4:0];
                        if ((^dut.u_lcd.data_in) === 1'bx) begin
                            lcd_errs = lcd_errs + 1;
                        end else begin
                            // 居中裁剪(col0=240, row0=120), 按源帧实际补偿核对
                            lcd_dxf = (lcd_F < 2) ? 0 : (lcd_F-1)*16;
                            lcd_exp = (240 + lcd_col - 9 - frame_comp[lcd_F] + lcd_dxf) & 31;
                            if (lcd_Xp !== lcd_exp[4:0]) begin
                                $display("ERROR[lcd] (%0d,%0d) X'=%0d expect %0d (F=%0d)",
                                         lcd_col, lcd_row, lcd_Xp, lcd_exp[4:0], lcd_F);
                                lcd_errs = lcd_errs + 1;
                            end
                        end
                    end
                end
            end
            lcd_col = lcd_col + 1;
            lcd_de_cnt = lcd_de_cnt + 1;
        end else if (lcd_col != 0) begin
            lcd_row = lcd_row + 1;
            lcd_col = 0;
        end
    end

    end

    //=================================================================
    // 主流程
    //=================================================================
    initial begin
        // 上电
        repeat (20) @(posedge sys_clk);
        sys_rst_n = 1'b1;

        // 阶段 1: HDMI, 图像从帧2开始漂移
        // 摄像头帧周期约 26.3ms, 覆盖多帧运动与重复显示
        #160_000_000;   // 160ms

        // 阶段 2: 按键切换到 LCD
        key_mode = 1'b0;   // 按下
        #40_000_000;
        key_mode = 1'b1;   // 松开
        #170_000_000;      // 等待切换复位 + LCD 显示稳定

        // Both output modes must actually have been exercised.
        if(hdmi_frame<4 || lcd_frame<4)$fatal(1,"insufficient system frame coverage");
        // 汇总
        if (hdmi_errs == 0 && hdmi_x_errs == 0 && lcd_errs == 0)
            $display("=== tb_video_stab_sys PASS (hdmi_errs=%0d x=%0d lcd_errs=%0d) ===",
                     hdmi_errs, hdmi_x_errs, lcd_errs);
        else
            $display("=== tb_video_stab_sys FAIL: hdmi=%0d x=%0d lcd=%0d ===",
                     hdmi_errs, hdmi_x_errs, lcd_errs);
        $stop;
    end

    // LCD ID 上拉窗口: 上电初期 + 按键切换后的重读窗口
    initial begin
        id_pull = 1'b1;
        #60_000_000;
        id_pull = 1'b0;
        #98_000_000;    // t = 158ms
        id_pull = 1'b1;
        #62_000_000;    // t = 620ms
        id_pull = 1'b0;
    end

endmodule
