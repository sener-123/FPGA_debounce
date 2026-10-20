//=====================================================================
// 文件    : video_stab_hdmi.v
// 功能    : FPGA 相机视频防抖工程顶层 —— 无 DDR3, HDMI 直出版
// 描述    : 整合摄像头驱动、防抖算法与 HDMI 直出(与摄像头锁相):
//
//   OV5640(720p30, pclk≈84MHz) ---> cmos2px ---> stab_core(运动估计+轨迹平滑)
//          |                                      |  comp(dx,dy)
//          +--> shift_compensate(平移补偿, BRAM 环形缓冲)
//                   | px_de_o/px_data_o(cam_pclk 域)
//              (cam_pclk/2 像素时钟, 帧同步锁相)
//                   | pix_data(1 像素/拍)
//              hdmi_timing_direct(720p30 时序) + dvi_tx_top(TMDS)
//                   | 4 对 LVDS
//                 HDMI 显示器
//
//   相比 video_stab_sys(DDR3 版): 去掉 DDR3 帧缓存 / disp_reader /
//   显示 FIFO / LCD 通路, 时钟域大幅简化。HDMI 刷新率 = 摄像头帧率
//   (30Hz), 不再做 30->60 帧率转换, 也无法做不同分辨率缩放。
//
// 平台    : 正点原子达芬奇 Pro Logos2 (PG2L200H-6IFBB484)
// 时钟    : sys_clk 50MHz(晶振); 像素时钟 = cam_pclk/2(约 42MHz,
//           由以摄像头为输入的 PLL 产生, 与串行时钟同源锁相);
//           TMDS 5x 串行时钟 clk_hdmi_5x = 5*像素时钟(约 210MHz)。
// 资源    : 约 33 个 18Kb 块 RAM(环形缓冲 + 块图 RAM), 0 DSP,
//           无 DDR3 控制器。
// 注意    : PLL IP 名 PLL, 输入 cam_pclk, clkout0=cam_pclk/2,
//           clkout1=cam_pclk*2.5, 两输出须同源且满足串行器时序约束。
//           典型输入84MHz时, 输出42MHz/210MHz。禁止用独立sys_clk
//           生成串行时钟。像素经小型异步FIFO跨入PLL像素时钟域。
//           H_TOTAL 为摄像头完整行周期, 单位为RGB565像素周期。
//=====================================================================
module video_stab_hdmi #(
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
    // ---- HDMI 同步脉宽(数据派生) ----
    parameter H_TOTAL   = 1650,    // Complete camera line in pixel clocks
    parameter H_SYNC    = 40,      // 行同步脉宽(像素时钟拍)
    parameter V_SYNC    = 5        // 场同步脉宽(行)
)(
    // ---- 系统 ----
    input  wire        sys_clk,     // 50MHz 晶振
    input  wire        sys_rst_n,   // 系统复位, 低有效

    // ---- 摄像头(OV5640) ----
    input  wire        cam_pclk,    // 摄像头像素时钟(约 84MHz)
    input  wire        cam_vsync,
    input  wire        cam_href,
    input  wire [7:0]  cam_data,
    output wire        cam_rst_n,
    output wire        cam_pwdn,
    output wire        cam_scl,
    inout  wire        cam_sda,

    // ---- HDMI(TMDS, 4 对 LVDS) ----
    output wire        tmds_clk_p,
    output wire        tmds_clk_n,
    output wire [2:0]  tmds_data_p,
    output wire [2:0]  tmds_data_n,

    // ---- 调试观测(可选, 不接即可) ----
    output wire        dbg_motion_valid, // 每帧运动估计完成脉冲
    output wire [5:0]  dbg_mv_x,         // 帧间运动矢量(块单位)
    output wire [5:0]  dbg_mv_y,
    output wire        dbg_comp_valid,   // 每帧补偿量更新脉冲
    output wire [5:0]  dbg_comp_x,       // 抖动补偿量(像素单位)
    output wire [5:0]  dbg_comp_y
);

    //=================================================================
    // 内部信号
    //=================================================================
    // PLL
    wire clk_hdmi_5x;      // TMDS 5x 串行时钟(约 210MHz)
    wire pll_lock;

    // 摄像头采集输出
    wire        cmos_frame_vsync;
    wire        cmos_frame_valid;
    wire [15:0] cmos_frame_data;

    // 统一内部像素流(cam_pclk 域)
    wire        px_clk, px_de, px_sof, px_eol, px_eof;
    wire [15:0] px_data;

    // 防抖补偿量
    wire       comp_valid;
    wire [5:0] comp_x, comp_y;

    // 平移补偿输出(cam_pclk 域)
    wire        px_de_o, px_sof_o, px_eol_o, px_eof_o;
    wire [15:0] px_data_o;

    // 像素时钟与跨域
    wire        pixel_clk;     // PLL output = cam_pclk/2 (typically 42MHz)
    wire [15:0] pix_data;      // pixel_clk 域像素(RGB565)
    wire        pix_valid;     // pixel_clk 域像素有效
    wire        line_end;      // pixel_clk 域行末
    wire        frame_end;     // pixel_clk 域帧末

    //=================================================================
    // 1. PLL: cam_pclk input; coherent 0.5x pixel and 2.5x serial outputs
    //=================================================================
    PLL u_pll (
        .rst      (~sys_rst_n),
        .clkin1   (cam_pclk),
        .lock     (pll_lock),
        .clkout0  (pixel_clk),     // cam_pclk / 2
        .clkout1  (clk_hdmi_5x)    // cam_pclk * 2.5
    );

    //=================================================================
    // 2. 摄像头驱动(正点原子 ov5640_dri: SCCB 配置 + 采集)
    //    配置为 1280x720 RGB565, HTS=1650, VTS=845(约 30fps)
    //=================================================================
`ifdef SIM_SOURCE
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
        .capture_start     (pll_lock),       // PLL 锁定即开始采集
        .cam_init_done     (),
        .cmos_frame_vsync  (cmos_frame_vsync),
        .cmos_frame_href   (),
        .cmos_frame_valid  (cmos_frame_valid),
        .cmos_frame_data   (cmos_frame_data)
    );
`endif

    //=================================================================
    // 3. 统一内部像素流适配
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
    // 4. 防抖算法核心(只监视像素流, 输出每帧补偿量)
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
    // 5. 平移补偿(环形 BRAM; HDMI 直出取整帧窗口 0,0,1280,720)
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
        .crop_col0  (12'd0),
        .crop_row0  (12'd0),
        .crop_w     (H_ACTIVE[11:0]),
        .crop_h     (V_ACTIVE[11:0]),
        .px_de_o    (px_de_o),
        .px_data_o  (px_data_o),
        .px_sof_o   (px_sof_o),
        .px_eol_o   (px_eol_o),
        .px_eof_o   (px_eof_o)
    );

    assign dbg_comp_valid = comp_valid;
    assign dbg_comp_x     = comp_x;
    assign dbg_comp_y     = comp_y;

    // Reset assertion is asynchronous; release is synchronized per domain.
    reg [1:0] cam_ready, pix_ready;
    wire stream_reset_n = sys_rst_n & pll_lock;
    always @(posedge cam_pclk or negedge stream_reset_n)
        if (!stream_reset_n) cam_ready <= 2'b00;
        else cam_ready <= {cam_ready[0], 1'b1};
    always @(posedge pixel_clk or negedge stream_reset_n)
        if (!stream_reset_n) pix_ready <= 2'b00;
        else pix_ready <= {pix_ready[0], 1'b1};

    // A complete pixel and its line/frame flags cross together. The FIFO
    // absorbs PLL phase and pointer synchronizer latency without a fabric clock.
    pixel_stream_cdc u_pixel_cdc (
        .wr_clk(cam_pclk), .wr_rst_n(cam_ready[1]),
        .wr_valid(px_de_o), .wr_data(px_data_o),
        .wr_eol(px_eol_o), .wr_eof(px_eof_o),
        .rd_clk(pixel_clk), .rd_rst_n(pix_ready[1]),
        .rd_valid(pix_valid), .rd_data(pix_data),
        .rd_eol(line_end), .rd_eof(frame_end)
    );

    //=================================================================
    // 8. HDMI 直出时序发生器(数据派生, 与摄像头锁相)
    //=================================================================
    wire        video_hs, video_vs, video_de;
    wire [23:0] video_rgb;

    hdmi_timing_direct #(
        .H_TOTAL(H_TOTAL),
        .H_SYNC (H_SYNC),
        .V_SYNC (V_SYNC)
    ) u_hdmi_tg (
        .pix_clk    (pixel_clk),
        .rst_n      (pix_ready[1]),
        .pix_data   (pix_data),
        .pix_valid  (pix_valid),
        .line_end   (line_end),
        .frame_end  (frame_end),
        .video_hs   (video_hs),
        .video_vs   (video_vs),
        .video_de   (video_de),
        .video_rgb  (video_rgb)
    );

    //=================================================================
    // 9. DVI/HDMI 发送器(TMDS 编码 + 10:1 串行化)
    //=================================================================
    dvi_tx_top u_dvi_tx (
        .pclk        (pixel_clk),
        .pclk_x5     (clk_hdmi_5x),
        .reset_n     (sys_rst_n & pll_lock),
        .video_din   (video_rgb),
        .video_hsync (video_hs),
        .video_vsync (video_vs),
        .video_de    (video_de),
        .tmds_clk_p  (tmds_clk_p),
        .tmds_clk_n  (tmds_clk_n),
        .tmds_data_p (tmds_data_p),
        .tmds_data_n (tmds_data_n)
    );

endmodule
