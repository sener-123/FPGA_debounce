//=====================================================================
// 文件    : video_stab_sys.v
// 功能    : FPGA 相机视频防抖系统工程顶层(HDMI 优先, LCD 可切换)
// 描述    : 整合摄像头驱动、防抖算法、DDR3 帧缓存与双显示输出:
//
//   OV5640(SCCB 配置) ---> cmos2px ---> stab_core(运动估计+轨迹平滑)
//                          |                  | comp(dx,dy)
//                          +--> shift_compensate(平移+裁剪+黑边)
//                                   | 裁剪后帧流(px_de_o)
//                              ddr3_ctrl_top(DDR3 乒乓帧缓存)
//                                   | rd_data(74.25MHz)
//                     +-------------+-------------+
//                disp_reader(帧同步/节流/补黑)   async_fifo_16
//                                   | 显示 FIFO
//                   +---------------+---------------+
//                HDMI 720p60                     LCD(RGB 屏)
//            hdmi_timing_gen+dvi_tx_top        lcd_rgb_top(正点原子)
//
//   HDMI 模式(默认): 摄像头 1280x720@30, DDR3 存整帧, HDMI 按
//   720p60 输出(每帧显示两次); LCD 模式: 按屏分辨率居中裁剪后
//   存 DDR3, LCD 1:1 显示。
//   按 KEY 在 HDMI/LCD 间切换, LED 指示当前模式。
//
// 平台    : 紫光同创 Logos-2 PGL22G6CMBG324 (PGL22G 开发板)
// 时钟    : sys_clk 50MHz(晶振 V9); PLL 输出 74.25MHz/371.25MHz
// 资源    : 约 45 个 18Kb 块 RAM + 少量逻辑, 0 DSP(详见 使用说明.md)
// 注意    : 1. 需用户例化 PLL IP(名 pll, 端口 pll_rst/clkin1/
//              pll_lock/clkout0/clkout1): 输入 50MHz, 输出
//              clkout0=74.25MHz, clkout1=371.25MHz;
//           2. 需 DDR3 IP(名 ddr3_ip)与 wr_fifo/rd_fifo IP,
//              与正点原子 ov5640_lcd 例程相同;
//           3. 板上摄像头 XCLK 由底板提供, 本工程不输出;
//           4. 摄像头默认配置为 1280x720 RGB565, 帧率约 30fps
//              (HTS=1650, VTS=845, 由 i2c_ov5640_rgb565_cfg 配置)。
//=====================================================================
module video_stab_sys #(
    // ---- 摄像头/算法分辨率 ----
    parameter H_ACTIVE  = 1280,
    parameter V_ACTIVE  = 720,
    // ---- 算法参数(与仿真验证值一致) ----
    parameter TILE      = 16,      // 下采样块尺寸
    parameter SEARCH_R  = 3,       // 运动搜索半径(块)
    parameter MAX_SHIFT = 8,       // 补偿幅度限幅(±像素)
    parameter SMOOTH_WIN= 4,       // 轨迹平滑窗口(2 的幂)
    parameter TRAJ_MAX  = 1024,    // 轨迹饱和值(±像素)
    parameter TILE_RAM_DEPTH = 4096, // 单块图 RAM 深度
    parameter RING_DEPTH = 32768,  // 平移补偿环形缓冲深度(2 的幂)
    // ---- 按键极性 ----
    parameter KEY_POL   = 1'b0     // 按键按下电平(0=低有效)
)(
    // ---- 系统 ----
    input  wire        sys_clk,     // 50MHz, 引脚 V9
    input  wire        sys_rst_n,   // 系统复位, 引脚 C4, 低有效
    input  wire        key_mode,    // 模式切换按键(低有效)
    output wire        led_mode,    // 模式指示(0=HDMI, 1=LCD)

    // ---- 摄像头(OV5640) ----
    input  wire        cam_pclk,    // 引脚 L1
    input  wire        cam_vsync,   // 引脚 P4
    input  wire        cam_href,    // 引脚 M5
    input  wire [7:0]  cam_data,    // 引脚 N1/P2/T2/P1/T1/U2/M3/U1
    output wire        cam_rst_n,   // 引脚 M1
    output wire        cam_pwdn,    // 引脚 L2
    output wire        cam_scl,     // 引脚 P3
    inout  wire        cam_sda,     // 引脚 L6

    // ---- DDR3 ----
    output wire        mem_rst_n,
    output wire        mem_ck,
    output wire        mem_ck_n,
    output wire        mem_cke,
    output wire        mem_ras_n,
    output wire        mem_cas_n,
    output wire        mem_we_n,
    output wire        mem_odt,
    output wire        mem_cs_n,
    output wire [14:0] mem_a,
    output wire [2:0]  mem_ba,
    inout  wire [3:0]  mem_dqs,
    inout  wire [3:0]  mem_dqs_n,
    inout  wire [31:0] mem_dq,
    output wire [3:0]  mem_dm,

    // ---- LCD(RGB 屏) ----
    output wire        lcd_hs,
    output wire        lcd_vs,
    output wire        lcd_de,
    inout  wire [23:0] lcd_rgb,
    output wire        lcd_bl,
    output wire        lcd_rst,
    output wire        lcd_pclk,

    // ---- HDMI(TMDS, 4 对 LVDS) ----
    output wire        tmds_clk_p,   // HDMI_CLK_P B16
    output wire        tmds_clk_n,   // HDMI_CLK_N A16
    output wire [2:0]  tmds_data_p,  // D2 B10, D1 B11, D0 B14
    output wire [2:0]  tmds_data_n,  // D2 A10, D1 A11, D0 A14

    // ---- 调试观测(可选, 不接即可) ----
    output wire        dbg_motion_valid, // 每帧运动估计完成脉冲
    output wire [5:0]  dbg_mv_x,         // 帧间运动矢量(块单位)
    output wire [5:0]  dbg_mv_y,
    output wire        dbg_comp_valid,   // 每帧补偿量更新脉冲
    output wire [5:0]  dbg_comp_x,       // 抖动补偿量(像素单位)
    output wire [5:0]  dbg_comp_y
);

    //=================================================================
    // 内部信号声明(全部集中于此, 便于 vlog 顺序解析)
    //=================================================================
    // PLL
    wire clk_hdmi_pix;     // 74.25MHz: HDMI 像素时钟 + DDR3 读时钟
    wire clk_hdmi_5x;      // 371.25MHz: TMDS 5x 串行时钟
    wire pll_lock;

    // 模式切换
    wire key_pulse;

    // 摄像头采集输出
    wire        cmos_frame_vsync;
    wire        cmos_frame_valid;
    wire [15:0] cmos_frame_data;

    // 统一内部像素流
    wire        px_clk, px_de, px_sof, px_eol, px_eof;
    wire [15:0] px_data;

    // 防抖补偿量
    wire       comp_valid;
    wire [5:0] comp_x, comp_y;

    // 平移补偿输出(写入 DDR3 的帧流)
    wire        px_de_o, px_sof_o, px_eol_o, px_eof_o;
    wire [15:0] px_data_o;

    // DDR3
    wire        ddr3_init_done;
    wire        disp_rd_en;
    wire [15:0] ddr3_rd_data;
    wire        disp_rd_load;

    // LCD
    wire [15:0] lcd_id;
    wire        lcd_clk;
    wire        lcd_data_req;
    wire        lcd_out_vsync;

    // HDMI
    wire        hdmi_data_req;
    wire        hdmi_load_pulse;
    wire        hdmi_hs, hdmi_vs, hdmi_de;
    wire [23:0] hdmi_rgb;

    // 显示 FIFO
    wire        fifo_wen;
    wire [15:0] fifo_wdata;
    wire        fifo_flush;
    wire        fifo_afull;
    wire [15:0] fifo_dout;
    wire        fifo_rd_clk;
    wire        fifo_rd_en;
    reg         flush_d0, flush_d1;
    wire        fifo_wr_rst_n;
    wire        fifo_rd_rst_n;

    // 显示数据选择
    wire [15:0] hdmi_data_in;
    wire [15:0] lcd_data_in;

    // 裁剪窗口与 DDR3 地址(系统时钟域, 准静态)
    reg [11:0] crop_col0, crop_row0, crop_w, crop_h;
    reg [27:0] ddr3_addr_max;       // 一帧像素总数(1280x720=921600 等)
    reg [9:0]  burst_len;           // 每行 16 像素字个数(帧宽[12:4])

    // 显示模式与切换复位
    reg        mode;                // 0=HDMI(默认), 1=LCD
    reg [21:0] sw_cnt;
    wire       switch_rst;

    //=================================================================
    // 1. PLL(用户 IP: 输入 50MHz, 输出 74.25MHz + 371.25MHz)
    //=================================================================
    pll u_pll (
        .pll_rst  (~sys_rst_n),
        .clkin1   (sys_clk),
        .pll_lock (pll_lock),
        .clkout0  (clk_hdmi_pix),  // DDR3 版: 74.25MHz(需重配 PLL)
        .clkout1  (clk_hdmi_5x)    // DDR3 版: 371.25MHz(需重配 PLL)
    );

    //=================================================================
    // 2. 显示模式切换(KEY 消抖, 切换后复位显示侧约 50ms)
    //=================================================================
    key_debounce #(.KEY_POL(KEY_POL)) u_key_deb (
        .clk       (sys_clk),
        .rst_n     (sys_rst_n),
        .key_in    (key_mode),
        .key_pulse (key_pulse)
    );

    always @(posedge sys_clk or negedge sys_rst_n)
        if (!sys_rst_n) mode <= 1'b0;
        else if (key_pulse) mode <= ~mode;

    always @(posedge sys_clk or negedge sys_rst_n)
        if (!sys_rst_n) sw_cnt <= 22'd0;
        else if (key_pulse) sw_cnt <= 22'd0;
`ifdef SIM_FAST
        else if (sw_cnt < 22'd2500) sw_cnt <= sw_cnt + 22'd1;
    assign switch_rst = (sw_cnt < 22'd2500);     // 仿真加速: 50us
`else
        else if (sw_cnt < 22'd2_500_000) sw_cnt <= sw_cnt + 22'd1;
    assign switch_rst = (sw_cnt < 22'd2_500_000);   // 高有效复位(50ms)
`endif

    assign led_mode = mode;

    //=================================================================
    // 3. 摄像头驱动(正点原子 ov5640_dri: SCCB 配置 + 采集)
    //    配置为 1280x720 RGB565, HTS=1650, VTS=845(约 30fps)
    //=================================================================
`ifdef SIM_SOURCE
    // 仿真: 用测试图像源代替摄像头(见 sim/sim_cam_source.v)
    sim_cam_source u_sim_cam (
        .cam_pclk          (cam_pclk),
        .rst_n             (sys_rst_n),
        .cmos_frame_vsync  (cmos_frame_vsync),
        .cmos_frame_valid  (cmos_frame_valid),
        .cmos_frame_data   (cmos_frame_data)
    );
`else
    ov5640_dri u_ov5640_dri (
        .clk               (sys_clk),
        .rst_n             (sys_rst_n),
        .cam_pclk          (cam_pclk),
        .cam_vsync         (cam_vsync),
        .cam_href          (cam_href),
        .cam_data          (cam_data),
        .cam_rst_n         (cam_rst_n),
        .cam_pwdn          (cam_pwdn),
        .cam_scl           (cam_scl),
        .cam_sda           (cam_sda),
        .cmos_h_pixel      (13'd1280),
        .cmos_v_pixel      (13'd720),
        .total_h_pixel     (13'd1650),
        .total_v_pixel     (13'd845),
        .y_addr_st         (13'd234),
        .y_addr_end        (13'd1709),
        .capture_start     (ddr3_init_done),
        .cam_init_done     (),
        .cmos_frame_vsync  (cmos_frame_vsync),
        .cmos_frame_href   (),
        .cmos_frame_valid  (cmos_frame_valid),
        .cmos_frame_data   (cmos_frame_data)
    );
`endif

    //=================================================================
    // 4. 统一内部像素流适配
    //=================================================================
    cmos2px #(
        .H_ACTIVE (H_ACTIVE),
        .V_ACTIVE (V_ACTIVE)
    ) u_c2p (
        .cam_pclk         (cam_pclk),
        .rst_n            (sys_rst_n),
        .cmos_frame_vsync (cmos_frame_vsync),
        .cmos_frame_valid (cmos_frame_valid),
        .cmos_frame_data  (cmos_frame_data),
        .px_clk           (px_clk),
        .px_de            (px_de),
        .px_data          (px_data),
        .px_sof           (px_sof),
        .px_eol           (px_eol),
        .px_eof           (px_eof)
    );

    //=================================================================
    // 5. 防抖算法核心(只监视像素流, 输出每帧补偿量)
    //=================================================================
    stab_core #(
        .H_ACTIVE   (H_ACTIVE),
        .V_ACTIVE   (V_ACTIVE),
        .TILE       (TILE),
        .SEARCH_R   (SEARCH_R),
        .MAX_SHIFT  (MAX_SHIFT),
        .SMOOTH_WIN (SMOOTH_WIN),
        .TRAJ_MAX   (TRAJ_MAX),
        .TILE_RAM_DEPTH (TILE_RAM_DEPTH)
    ) u_stab_core (
        .px_clk       (px_clk),
        .rst_n        (sys_rst_n),
        .px_de        (px_de),
        .px_data      (px_data),
        .px_sof       (px_sof),
        .px_eol       (px_eol),
        .comp_valid   (comp_valid),
        .comp_x       (comp_x),
        .comp_y       (comp_y),
        .motion_valid (dbg_motion_valid),
        .mv_x         (dbg_mv_x),
        .mv_y         (dbg_mv_y)
    );

    //=================================================================
    // 6. 裁剪窗口与 DDR3 地址表(按模式 + LCD ID 选择)
    //=================================================================
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            crop_col0     <= 12'd0;
            crop_row0     <= 12'd0;
            crop_w        <= 12'd1280;
            crop_h        <= 12'd720;
            ddr3_addr_max <= 28'd921600;
            burst_len     <= 10'd80;
        end else begin
            if (mode == 1'b0) begin
                // ---- HDMI: 存整帧 ----
                crop_col0     <= 12'd0;
                crop_row0     <= 12'd0;
                crop_w        <= 12'd1280;
                crop_h        <= 12'd720;
                ddr3_addr_max <= 28'd921600;
                burst_len     <= 10'd80;
            end else begin
                case (lcd_id)
                    16'h4342 : begin   // 4.3寸 480x272
                        crop_col0 <= 12'd400;  crop_row0 <= 12'd224;
                        crop_w    <= 12'd480;  crop_h    <= 12'd272;
                        ddr3_addr_max <= 28'd130560; burst_len <= 10'd30;
                    end
                    16'h7084 : begin   // 7寸 800x480
                        crop_col0 <= 12'd240;  crop_row0 <= 12'd120;
                        crop_w    <= 12'd800;  crop_h    <= 12'd480;
                        ddr3_addr_max <= 28'd384000; burst_len <= 10'd50;
                    end
                    16'h7016 : begin   // 7寸 1024x600
                        crop_col0 <= 12'd128;  crop_row0 <= 12'd60;
                        crop_w    <= 12'd1024; crop_h    <= 12'd600;
                        ddr3_addr_max <= 28'd614400; burst_len <= 10'd64;
                    end
                    16'h4384 : begin   // 4.3寸 800x480
                        crop_col0 <= 12'd240;  crop_row0 <= 12'd120;
                        crop_w    <= 12'd800;  crop_h    <= 12'd480;
                        ddr3_addr_max <= 28'd384000; burst_len <= 10'd50;
                    end
                    16'h1018 : begin   // 10.1寸 1280x800(存 1280x720, 下方补黑)
                        crop_col0 <= 12'd0;    crop_row0 <= 12'd0;
                        crop_w    <= 12'd1280; crop_h    <= 12'd720;
                        ddr3_addr_max <= 28'd921600; burst_len <= 10'd80;
                    end
                    default : begin    // 未识别/未接屏: 按 800x480
                        crop_col0 <= 12'd240;  crop_row0 <= 12'd120;
                        crop_w    <= 12'd800;  crop_h    <= 12'd480;
                        ddr3_addr_max <= 28'd384000; burst_len <= 10'd50;
                    end
                endcase
            end
        end
    end

    //=================================================================
    // 7. 平移补偿 + 裁剪(HDMI 整帧 / LCD 按屏分辨率居中)
    //=================================================================
    shift_compensate #(
        .H_ACTIVE   (H_ACTIVE),
        .V_ACTIVE   (V_ACTIVE),
        .MAX_SHIFT  (MAX_SHIFT),
        .RING_DEPTH (RING_DEPTH)
    ) u_shift (
        .px_clk     (px_clk),
        .rst_n      (sys_rst_n),
        .px_de      (px_de),
        .px_data    (px_data),
        .px_sof     (px_sof),
        .comp_valid (comp_valid),
        .comp_x     (comp_x),
        .comp_y     (comp_y),
        .crop_col0  (crop_col0),
        .crop_row0  (crop_row0),
        .crop_w     (crop_w),
        .crop_h     (crop_h),
        .px_de_o    (px_de_o),
        .px_data_o  (px_data_o),
        .px_sof_o   (px_sof_o),
        .px_eol_o   (px_eol_o),
        .px_eof_o   (px_eof_o)
    );

    //=================================================================
    // 8. DDR3 帧缓存(乒乓, 正点原子 ddr3_ctrl_top + Pango DDR3 IP)
    //=================================================================
`ifdef SIM_DDR3
    // 仿真: 行为级 DDR3 帧缓存模型(见 sim/ddr3_frame_buf_model.v)
    ddr3_frame_buf_model u_ddr3 (
        .clk              (sys_clk),
        .wd_clk           (cam_pclk),
        .rd_clk           (clk_hdmi_pix),
        .rst_n            (sys_rst_n),
        .wd_en            (px_de_o),
        .wd_data          (px_data_o),
        .rd_en            (disp_rd_en),
        .rd_data          (ddr3_rd_data),
        .wr_load          (px_sof),
        .rd_load          (disp_rd_load),
        .ddr3_pingpang_en (1'b1),
        .ddr3_read_valid  (1'b1),
        .addr_rd_min      (27'd0),
        .addr_rd_max      (ddr3_addr_max[26:1]),
        .rd_burst_len     (burst_len),
        .addr_wd_min      (27'd0),
        .addr_wd_max      (ddr3_addr_max[26:1]),
        .wd_burst_len     (burst_len),
        .ddr3_init_done   (ddr3_init_done),
        .mem_rst_n        (mem_rst_n),
        .mem_ck           (mem_ck),
        .mem_ck_n         (mem_ck_n),
        .mem_cke          (mem_cke),
        .mem_ras_n        (mem_ras_n),
        .mem_cas_n        (mem_cas_n),
        .mem_we_n         (mem_we_n),
        .mem_odt          (mem_odt),
        .mem_cs_n         (mem_cs_n),
        .mem_a            (mem_a),
        .mem_ba           (mem_ba),
        .mem_dqs          (mem_dqs),
        .mem_dqs_n        (mem_dqs_n),
        .mem_dq           (mem_dq),
        .mem_dm           (mem_dm)
    );
`else
    ddr3_ctrl_top u_ddr3 (
        .clk              (sys_clk),
        .rst_n            (sys_rst_n),
        .ddr3_init_done   (ddr3_init_done),
        // 用户接口
        .wd_clk           (cam_pclk),
        .rd_clk           (clk_hdmi_pix),
        .wd_en            (px_de_o),
        .wd_data          (px_data_o),
        .rd_en            (disp_rd_en),
        .rd_data          (ddr3_rd_data),
        .addr_rd_min      (27'd0),
        .addr_rd_max      (ddr3_addr_max[26:1]),
        .rd_burst_len     (burst_len),
        .addr_wd_min      (27'd0),
        .addr_wd_max      (ddr3_addr_max[26:1]),
        .wd_burst_len     (burst_len),
        .wr_load          (px_sof),
        .rd_load          (disp_rd_load),
        .ddr3_pingpang_en (1'b1),
        .ddr3_read_valid  (1'b1),
        // DDR3 IO 接口
        .mem_rst_n        (mem_rst_n),
        .mem_ck           (mem_ck),
        .mem_ck_n         (mem_ck_n),
        .mem_cke          (mem_cke),
        .mem_ras_n        (mem_ras_n),
        .mem_cas_n        (mem_cas_n),
        .mem_we_n         (mem_we_n),
        .mem_odt          (mem_odt),
        .mem_cs_n         (mem_cs_n),
        .mem_a            (mem_a),
        .mem_ba           (mem_ba),
        .mem_dqs          (mem_dqs),
        .mem_dqs_n        (mem_dqs_n),
        .mem_dq           (mem_dq),
        .mem_dm           (mem_dm)
    );
`endif

    //=================================================================
    // 9. LCD 显示(正点原子 lcd_rgb_top, 自动识别屏 ID 与分频)
    //=================================================================
    lcd_rgb_top u_lcd (
        .sys_clk      (sys_clk),
        .sys_rst_n    (sys_rst_n & ~switch_rst),
        .sys_init_done(ddr3_init_done),
        .lcd_id       (lcd_id),
        .lcd_hs       (lcd_hs),
        .lcd_vs       (lcd_vs),
        .lcd_de       (lcd_de),
        .lcd_rgb      (lcd_rgb),
        .lcd_bl       (lcd_bl),
        .lcd_rst      (lcd_rst),
        .lcd_pclk     (lcd_pclk),
        .lcd_clk      (lcd_clk),
        .out_vsync    (lcd_out_vsync),
        .h_disp       (),
        .v_disp       (),
        .pixel_xpos   (),
        .pixel_ypos   (),
        .data_in      (lcd_data_in),
        .data_req     (lcd_data_req)
    );

    //=================================================================
    // 10. HDMI 显示(720p60 时序 + TMDS 发送)
    //=================================================================
    hdmi_timing_gen u_hdmi_tg (
        .pix_clk    (clk_hdmi_pix),
        .rst_n      (sys_rst_n & ~switch_rst & pll_lock),
        .pix_data   (hdmi_data_in),
        .data_req   (hdmi_data_req),
        .video_hs   (hdmi_hs),
        .video_vs   (hdmi_vs),
        .video_de   (hdmi_de),
        .video_rgb  (hdmi_rgb),
        .load_pulse (hdmi_load_pulse)
    );

    dvi_tx_top u_dvi (
        .pclk        (clk_hdmi_pix),
        .pclk_x5     (clk_hdmi_5x),
        .reset_n     (sys_rst_n & pll_lock),
        .video_din   (hdmi_rgb),
        .video_hsync (hdmi_hs),
        .video_vsync (hdmi_vs),
        .video_de    (hdmi_de),
        .tmds_clk_p  (tmds_clk_p),
        .tmds_clk_n  (tmds_clk_n),
        .tmds_data_p (tmds_data_p),
        .tmds_data_n (tmds_data_n)
    );

    //=================================================================
    // 11. DDR3 读侧主控(帧同步/节流/补黑) + 显示 FIFO
    //=================================================================
    disp_reader #(
        .H_ACTIVE (H_ACTIVE),
        .V_ACTIVE (V_ACTIVE)
    ) u_disp_rd (
        .rd_clk      (clk_hdmi_pix),
        .rst_n       (sys_rst_n & ~switch_rst),
        .ddr3_rd_data(ddr3_rd_data),
        .ddr3_rd_en  (disp_rd_en),
        .rd_load     (disp_rd_load),
        .load_hdmi   (hdmi_load_pulse),
        .load_lcd    (lcd_out_vsync),
        .mode        (mode),
        .lcd_id      (lcd_id),
        .fifo_afull  (fifo_afull),
        .fifo_wen    (fifo_wen),
        .fifo_wdata  (fifo_wdata),
        .fifo_flush  (fifo_flush)
    );

    // 显示 FIFO 读时钟/读请求按模式选择(切换瞬间有毛刺, 由
    // switch_rst 复位保护; 见 使用说明.md)
    assign fifo_rd_clk = (mode == 1'b1) ? lcd_clk : clk_hdmi_pix;
    assign fifo_rd_en  = (mode == 1'b1) ? lcd_data_req : hdmi_data_req;

    // 帧 flush 同步到显示时钟域(长脉冲, 2 级同步即可)
    always @(posedge fifo_rd_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            flush_d0 <= 1'b0;
            flush_d1 <= 1'b0;
        end else begin
            flush_d0 <= fifo_flush;
            flush_d1 <= flush_d0;
        end
    end

    assign fifo_wr_rst_n = sys_rst_n & ~switch_rst & ~fifo_flush;
    assign fifo_rd_rst_n = sys_rst_n & ~switch_rst & ~flush_d1;

    async_fifo_16 u_fifo (
        .wr_clk   (clk_hdmi_pix),
        .wr_rst_n (fifo_wr_rst_n),
        .wr_en    (fifo_wen),
        .wr_data  (fifo_wdata),
        .wr_afull (fifo_afull),
        .rd_clk   (fifo_rd_clk),
        .rd_rst_n (fifo_rd_rst_n),
        .rd_en    (fifo_rd_en),
        .rd_data  (fifo_dout),
        .rd_empty ()
    );

    // 显示数据: 未选中路径固定输出黑(选中路径接 FIFO)
    assign hdmi_data_in = (mode == 1'b1) ? 16'd0 : fifo_dout;
    assign lcd_data_in  = (mode == 1'b1) ? fifo_dout : 16'd0;

    //=================================================================
    // 12. 调试观测
    //=================================================================
    assign dbg_comp_valid = comp_valid;
    assign dbg_comp_x     = comp_x;
    assign dbg_comp_y     = comp_y;

endmodule
