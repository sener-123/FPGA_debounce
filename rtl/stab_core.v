//=====================================================================
// 文件    : stab_core.v
// 功能    : 视频防抖算法核心(与视频接口完全解耦)
// 描述    : 内部由三个环节组成, 与参考实现
//           (lengkujiaai/video_stabilization)的软件流程一一对应:
//             参考流程                     本工程硬件实现
//             ------------------------     ---------------------------
//             特征点检测 + 光流跟踪   -->   downsample_tile 分块下采样
//                                          + motion_estimate 全网格
//                                          SAD 全局平移估计
//             运动轨迹 cumsum         -->   trajectory_smooth 轨迹累积
//             轨迹移动平均平滑        -->   trajectory_smooth 尾随滑窗
//             warpAffine 帧变换       -->   shift_compensate 平移补偿
//             相机抖动分量为平移, 忽略旋转/缩放, 换取极小资源占用。
// 资源    : 2 块图 RAM(各 4096x16) + 少量逻辑, 无乘法器/DSP
// 接口    : 只与统一内部像素流(px_*)交互, 不接触任何摄像头/显示器引脚
//=====================================================================
module stab_core #(
    parameter H_ACTIVE  = 1280,    // 行有效像素数
    parameter V_ACTIVE  = 720,     // 场有效行数
    parameter TILE      = 16,      // 下采样块尺寸
    parameter SEARCH_R  = 3,       // 运动搜索半径(块)
    parameter MAX_SHIFT = 8,       // 补偿幅度限幅(±像素)
    parameter SMOOTH_WIN= 4,       // 轨迹平滑窗口(2 的幂)
    parameter TRAJ_MAX  = 1024,    // 轨迹饱和值(±像素)
    parameter TILE_RAM_DEPTH = 4096 // 单块图 RAM 深度(>= TILES_X*TILES_Y)
)(
    input  wire        px_clk,
    input  wire        rst_n,      // 异步复位, 低有效
    // ---- 统一内部像素流(监视端口, 不修改视频通路) ----
    input  wire        px_de,
    input  wire [15:0] px_data,    // RGB565
    input  wire        px_sof,
    input  wire        px_eol,

    // ---- 补偿量输出(每帧 1 次) ----
    output wire        comp_valid,
    output wire [5:0]  comp_x,     // 有符号, ±MAX_SHIFT 像素
    output wire [5:0]  comp_y,

    // ---- 调试观测输出 ----
    output wire        motion_valid,
    output wire [5:0]  mv_x,
    output wire [5:0]  mv_y
);

    localparam TX    = H_ACTIVE / TILE;
    localparam TY    = V_ACTIVE / TILE;
    localparam TILES = TX * TY;

    function integer clog2;
        input integer v;
        integer t;
        begin
            t = v - 1;
            clog2 = 0;
            while (t > 0) begin
                t = t >> 1;
                clog2 = clog2 + 1;
            end
        end
    endfunction
    localparam AW = clog2(TILE_RAM_DEPTH);   // RAM 地址宽度

    //-----------------------------------------------------------------
    // Switch only after the final block write has committed. Camera VSYNC
    // can arrive while the last tile row is still being drained to RAM.
    //-----------------------------------------------------------------
    reg frame_parity;
    reg have_previous_frame;

    //-----------------------------------------------------------------
    // 分块下采样(写侧)
    //-----------------------------------------------------------------
    wire        ds_ram_wen;
    wire [12:0] ds_ram_waddr;
    wire [15:0] ds_ram_wdata;
    wire        ds_frame_done;

    always @(posedge px_clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_parity <= 1'b0;
            have_previous_frame <= 1'b0;
        end else if (ds_frame_done) begin
            frame_parity <= ~frame_parity;
            have_previous_frame <= 1'b1;
        end
    end

    downsample_tile #(
        .H_ACTIVE  (H_ACTIVE),
        .V_ACTIVE  (V_ACTIVE),
        .TILE      (TILE),
        .RAM_DEPTH (2*TILE_RAM_DEPTH)
    ) u_ds (
        .px_clk        (px_clk),
        .rst_n         (rst_n),
        .px_de         (px_de),
        .px_data       (px_data),
        .px_sof        (px_sof),
        .px_eol        (px_eol),
        .frame_parity  (frame_parity),
        .ram_wen       (ds_ram_wen),
        .ram_waddr     (ds_ram_waddr),
        .ram_wdata     (ds_ram_wdata),
        .frame_done    (ds_frame_done)
    );

    //-----------------------------------------------------------------
    // 两块图 RAM(乒乓): 各 TILE_RAM_DEPTH x 16bit
    // 写口接 downsampler(由 parity 选择), 读口接 motion_estimate
    //-----------------------------------------------------------------
    reg [15:0] ram0 [0:TILE_RAM_DEPTH-1];
    reg [15:0] ram1 [0:TILE_RAM_DEPTH-1];
    reg [15:0] ram0_q, ram1_q;

    wire [13:0] me_ram0_raddr;
    wire [13:0] me_ram1_raddr;

    wire we0 = ds_ram_wen & ~frame_parity;
    wire we1 = ds_ram_wen &  frame_parity;
    // 下采样器输出地址 = BASE + 块序号(BASE = TILE_RAM_DEPTH),
    // 与 (TILE_RAM_DEPTH-1) 相与即得到被选中 RAM 的局部块地址
    wire [AW-1:0] wa0 = ds_ram_waddr & (TILE_RAM_DEPTH-1);
    wire [AW-1:0] wa1 = ds_ram_waddr & (TILE_RAM_DEPTH-1);

    always @(posedge px_clk) begin
        if (we0) ram0[wa0] <= ds_ram_wdata;
        ram0_q <= ram0[me_ram0_raddr[AW-1:0]];
    end
    always @(posedge px_clk) begin
        if (we1) ram1[wa1] <= ds_ram_wdata;
        ram1_q <= ram1[me_ram1_raddr[AW-1:0]];
    end

    //-----------------------------------------------------------------
    // 运动估计(读侧)
    //-----------------------------------------------------------------
    wire        me_motion_valid;
    wire [5:0]  me_mv_x, me_mv_y;

    motion_estimate #(
        .H_ACTIVE (H_ACTIVE),
        .V_ACTIVE (V_ACTIVE),
        .TILE     (TILE),
        .SEARCH_R (SEARCH_R),
        .RAM_DEPTH(TILE_RAM_DEPTH)
    ) u_me (
        .px_clk       (px_clk),
        .rst_n        (rst_n),
        .start        (ds_frame_done & have_previous_frame),
        .src_sel      (~frame_parity),   // 上一帧块图所在 RAM
        .ram0_raddr   (me_ram0_raddr),
        .ram1_raddr   (me_ram1_raddr),
        .ram0_rdata   (ram0_q),
        .ram1_rdata   (ram1_q),
        .motion_valid (me_motion_valid),
        .mv_x         (me_mv_x),
        .mv_y         (me_mv_y)
    );

    //-----------------------------------------------------------------
    // 轨迹平滑 -> 补偿量
    //-----------------------------------------------------------------
    trajectory_smooth #(
        .MAX_SHIFT  (MAX_SHIFT),
        .SMOOTH_WIN (SMOOTH_WIN),
        .TRAJ_MAX   (TRAJ_MAX),
        .TILE       (TILE)
    ) u_smooth (
        .px_clk       (px_clk),
        .rst_n        (rst_n),
        .motion_valid (me_motion_valid),
        .mv_x         (me_mv_x),
        .mv_y         (me_mv_y),
        .comp_valid   (comp_valid),
        .comp_x      (comp_x),
        .comp_y      (comp_y)
    );

    // 调试观测
    assign motion_valid = me_motion_valid;
    assign mv_x         = me_mv_x;
    assign mv_y         = me_mv_y;

endmodule
