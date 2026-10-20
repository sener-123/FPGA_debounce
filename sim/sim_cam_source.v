//=====================================================================
// 文件    : sim_cam_source.v (仿真测试图像源)
// 功能    : 代替 ov5640_dri, 产生 cmos_frame_* 测试视频流
// 描述    : 1280x720 RGB565, 每像素 2 拍(模拟 OV5640 双字节输出);
//           像素值 = {frame_cnt[5:0], y'[4:0], x'[4:0]}, 其中
//           x' = (x + dx_inj) mod 32, dx_inj 为注入的水平抖动:
//             第 0~7 帧 dx=0(静止);
//             第 8 帧起每帧 +1(线性漂移, 用于验证防抖跟踪)。
//           帧结构: 720 行 x (2560+300) 拍 + 200 行场消隐。
// 使用    : 测试台编译本文件并定义 SIM_SOURCE。
//=====================================================================
`timescale 1ns/1ps
module sim_cam_source #(
    parameter H = 1280,
    parameter V = 720,
    parameter ROW_BLANK = 300,     // 行消隐(cam_pclk 拍)
    parameter DRIFT_START_FRAME = 8, // First frame with a 16-pixel injected shift
    parameter V_BLANK   = 200      // 场消隐行数
)(
    input  wire        cam_pclk,
    input  wire        rst_n,
    output reg         cmos_frame_vsync,  // 帧起始脉冲(场消隐期)
    output reg         cmos_frame_valid,  // 像素有效(每像素 2 拍中第 2 拍)
    output reg  [15:0] cmos_frame_data
);

    localparam ROW_TOTAL = 2*H + ROW_BLANK;
    localparam V_TOTAL   = V + V_BLANK;

    reg [11:0] h_cnt;       // 行内计数 0..ROW_TOTAL-1
    reg [11:0] y_cnt;       // 行计数 0..V_TOTAL-1
    reg [10:0] x_cnt;       // 当前像素列
    reg        byte_phase;  // 每像素第 1/2 拍
    reg [5:0]  frame_cnt;   // 帧号(mod 64)
    reg [9:0]  dx_inj;      // 注入水平抖动(帧 8 起每帧 +16 像素=1 块)

    // 当前像素值(组合): x' = (x + dx) mod 32
    wire [4:0] xm = (x_cnt[4:0] + dx_inj[4:0]);
    wire [4:0] ym = y_cnt[4:0];
    wire [15:0] px_w = {frame_cnt, ym, xm};

    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) begin
            h_cnt             <= 12'd0;
            y_cnt             <= 12'd0;
            x_cnt             <= 11'd0;
            byte_phase        <= 1'b0;
            frame_cnt         <= 6'd0;
            dx_inj            <= 10'd0;
            cmos_frame_vsync  <= 1'b0;
            cmos_frame_valid  <= 1'b0;
            cmos_frame_data   <= 16'd0;
        end else begin
            // ---- 行列计数 ----
            if (h_cnt == ROW_TOTAL-1) begin
                h_cnt <= 12'd0;
                if (y_cnt == V_TOTAL-1) begin
                    y_cnt <= 12'd0;
                    frame_cnt <= frame_cnt + 6'd1;
                    // 帧 0~7 静止(dx=0); 帧 8 起每帧水平漂移 +16 像素
                    // (=1 块, 运动估计可见; 此处 frame_cnt 为旧值,
                    //  进入帧 8 时旧值=7 -> dx=16)
                    dx_inj <= (frame_cnt < DRIFT_START_FRAME-1) ? 10'd0
                             : ({4'd0, frame_cnt} - (DRIFT_START_FRAME-2)) << 4;
                end else begin
                    y_cnt <= y_cnt + 12'd1;
                end
            end else begin
                h_cnt <= h_cnt + 12'd1;
            end

            // ---- 像素输出(有效区内) ----
            if (y_cnt < V && h_cnt < 2*H) begin
                byte_phase <= ~byte_phase;
                if (byte_phase == 1'b0) begin
                    // 第 1 拍: 收"高字节", 无有效(与 cmos_capture_data 一致)
                    x_cnt <= h_cnt[11:1];
                    cmos_frame_valid <= 1'b0;
                end else begin
                    cmos_frame_valid <= 1'b1;
                    cmos_frame_data  <= px_w;
                end
            end else begin
                byte_phase       <= 1'b0;
                cmos_frame_valid <= 1'b0;
            end

            // ---- 帧起始脉冲(场消隐起始 8 拍) ----
            cmos_frame_vsync <= (y_cnt == V) && (h_cnt < 12'd8);
        end
    end

endmodule
