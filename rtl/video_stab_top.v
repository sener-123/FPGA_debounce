//=====================================================================
// 文件    : video_stab_top.v
// 功能    : FPGA 相机视频防抖顶层
// 描述    : 数据通路与参考实现(lengkujiaai/video_stabilization)对应:
//            摄像头(OV5640)
//               |  原始并行视频总线
//         video_in_if   视频输入接口适配(解耦层)
//               |  统一内部像素流 px_*
//        +------+------+
//        |             |
//      stab_core  shift_compensate
//     (运动估计/   (环形 BRAM 平移补偿,
//      轨迹平滑)    裁剪输出窗口)
//        |             |
//        +----comp----+|
//               |  统一内部像素流 px_*o
//         video_out_if  视频输出接口适配(解耦层)
//               |  标准显示时序 hsync/vsync/de/RGB888
//           显示器驱动(HDMI 等)
//
// 平台    : 紫光同创 Logos-2 PGL22G6CMBG324 (PGL22G 开发板)
// 时钟    : 全部逻辑工作于摄像头像素时钟 cam_pclk, 不使用板上 50MHz
// 资源    : 约 24 个 18Kb 块 RAM + 少量逻辑, 不使用 DSP(详见 使用说明.md)
// 注意    : 1. OV5640 的 SCCB/I2C 寄存器配置由外部摄像头驱动完成
//              (本工程不包含), 摄像头需配置为 RGB565 输出;
//           2. 板上摄像头 RESET/PWDN 引脚未接出, 无需控制;
//           3. 摄像头输出帧率必须与 video_out_if 的显示时序总周期一致。
//=====================================================================
module video_stab_top #(
    // ---- 视频源/分辨率 ----
    parameter H_ACTIVE  = 1280,    // 行有效像素数
    parameter V_ACTIVE  = 720,     // 场有效行数
    // ---- 显示时序(默认 1280x720@60) ----
    parameter H_TOTAL   = 1650,
    parameter V_TOTAL   = 750,
    parameter H_FP      = 110,
    parameter H_BP      = 220,
    parameter H_SYNC    = 40,
    parameter V_FP      = 5,
    parameter V_BP      = 20,
    parameter V_SYNC    = 5,
    // ---- 算法参数 ----
    parameter TILE      = 16,      // 下采样块尺寸
    parameter SEARCH_R  = 3,       // 运动搜索半径(块)
    parameter MAX_SHIFT = 8,       // 补偿幅度限幅(±像素)
    parameter SMOOTH_WIN= 4,       // 轨迹平滑窗口(2 的幂)
    parameter TRAJ_MAX  = 1024,    // 轨迹饱和值(±像素)
    parameter TILE_RAM_DEPTH = 4096, // 单块图 RAM 深度
    parameter RING_DEPTH = 32768,  // 平移补偿环形缓冲深度(2 的幂)
    // ---- 摄像头极性 ----
    parameter VS_POL    = 1,
    parameter HREF_POL  = 1,
    // 每像素占用的 px_clk 周期数: OV5640 RGB565 为双字节输出(2 拍/像素)
    parameter PX_CLK_PER_PIX = 2
)(
    // ---- 摄像头并行视频总线(PGL22G 开发板 OV5640 引脚) ----
    input  wire        cam_pclk,     // CMOS_PCLK  J17
    input  wire        cam_vsync,    // CMOS_VSYNC J18
    input  wire        cam_href,     // CMOS_HREF  H14
    input  wire [7:0]  cam_data,     // CMOS_D[7:0] E18/D18/F14/F13/E17/D17/G14/G13
    input  wire        rst_n,        // 系统复位(可接按键, 低有效)

    // ---- 显示时序输出(接 HDMI/LCD 驱动) ----
    output wire        disp_hsync,
    output wire        disp_vsync,
    output wire        disp_de,
    output wire [23:0] disp_data,    // RGB888

    // ---- 调试观测 ----
    output wire        motion_valid, // 每帧运动估计完成脉冲
    output wire [5:0]  mv_x,         // 帧间运动矢量(块单位)
    output wire [5:0]  mv_y,
    output wire        comp_valid,   // 每帧补偿量更新脉冲
    output wire [5:0]  comp_x,       // 抖动补偿量(像素单位)
    output wire [5:0]  comp_y
);

    //=================================================================
    // 1. 视频输入接口适配(解耦层)
    //=================================================================
    wire        px_clk;
    wire        px_de;
    wire [15:0] px_data;
    wire        px_sof;
    wire        px_eol;
    wire        px_eof;

    video_in_if #(
        .H_ACTIVE (H_ACTIVE),
        .V_ACTIVE (V_ACTIVE),
        .VS_POL   (VS_POL),
        .HREF_POL (HREF_POL)
    ) u_video_in (
        .cam_pclk  (cam_pclk),
        .cam_vsync (cam_vsync),
        .cam_href  (cam_href),
        .cam_data  (cam_data),
        .rst_n     (rst_n),
        .px_clk    (px_clk),
        .px_de     (px_de),
        .px_data   (px_data),
        .px_sof    (px_sof),
        .px_eol    (px_eol),
        .px_eof    (px_eof)
    );

    //=================================================================
    // 2. 防抖算法核心(只监视像素流, 输出补偿量)
    //=================================================================
    wire [5:0] comp_x_w, comp_y_w;
    wire       comp_valid_w;

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
        .rst_n        (rst_n),
        .px_de        (px_de),
        .px_data      (px_data),
        .px_sof       (px_sof),
        .px_eol       (px_eol),
        .comp_valid   (comp_valid_w),
        .comp_x       (comp_x_w),
        .comp_y       (comp_y_w),
        .motion_valid (motion_valid),
        .mv_x         (mv_x),
        .mv_y         (mv_y)
    );

    //=================================================================
    // 3. 平移补偿(视频通路: 平移 + 裁剪)
    //=================================================================
    wire        px_de_o;
    wire [15:0] px_data_o;
    wire        px_sof_o;
    wire        px_eol_o;
    wire        px_eof_o;

    shift_compensate #(
        .H_ACTIVE   (H_ACTIVE),
        .V_ACTIVE   (V_ACTIVE),
        .MAX_SHIFT  (MAX_SHIFT),
        .RING_DEPTH (RING_DEPTH)
    ) u_shift (
        .px_clk     (px_clk),
        .rst_n      (rst_n),
        .px_de      (px_de),
        .px_data    (px_data),
        .px_sof     (px_sof),
        .comp_valid (comp_valid_w),
        .comp_x     (comp_x_w),
        .comp_y     (comp_y_w),
        .px_de_o    (px_de_o),
        .px_data_o  (px_data_o),
        .px_sof_o   (px_sof_o),
        .px_eol_o   (px_eol_o),
        .px_eof_o   (px_eof_o)
    );

    //=================================================================
    // 4. 视频输出接口适配(解耦层): 生成显示时序 + 黑边
    //=================================================================
    video_out_if #(
        .H_ACTIVE (H_ACTIVE),
        .V_ACTIVE (V_ACTIVE),
        .H_TOTAL  (H_TOTAL),
        .V_TOTAL  (V_TOTAL),
        .H_FP     (H_FP),
        .H_BP     (H_BP),
        .H_SYNC   (H_SYNC),
        .V_FP     (V_FP),
        .V_BP     (V_BP),
        .V_SYNC   (V_SYNC),
        .BORDER   (MAX_SHIFT + 1),
        .PX_CLK_PER_PIX (PX_CLK_PER_PIX),
        .HS_POL   (1),
        .VS_POL   (1)
    ) u_video_out (
        .px_clk     (px_clk),
        .rst_n      (rst_n),
        .px_de      (px_de_o),
        .px_data    (px_data_o),
        .px_sof     (px_sof_o),
        .px_eol     (px_eol_o),
        .px_eof     (px_eof_o),
        .disp_hsync (disp_hsync),
        .disp_vsync (disp_vsync),
        .disp_de    (disp_de),
        .disp_data  (disp_data)
    );

    // 调试观测输出
    assign comp_valid = comp_valid_w;
    assign comp_x     = comp_x_w;
    assign comp_y     = comp_y_w;

endmodule
