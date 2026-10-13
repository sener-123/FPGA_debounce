//=====================================================================
// 文件    : hdmi_timing_direct.v
// 功能    : HDMI 直出时序发生器(无 DDR3/无 FIFO 版本)
// 描述    : 与摄像头锁相(genlock)的 1280x720@30 显示时序发生器,
//           在像素时钟 pix_clk(= cam_pclk/2)上直接从防抖平移补偿模块
//           的像素流派生同步信号(数据派生, 无需自由计数器, 因此不
//           依赖摄像头消隐的具体结构):
//             video_de   = pix_valid(像素有效, 即摄像头有效区)
//             video_hs   = 行末(line_end)后 H_SYNC 拍的同步脉冲
//             video_vs   = 帧末(frame_end)后 V_SYNC 行的同步脉冲
//           RGB565 -> RGB888 输出。
// 平台    : 紫光同创 Logos-2 PGL22G6CMBG324 (PGL22G 开发板)
// 时钟    : pix_clk = cam_pclk / 2(约 42MHz, 摄像头像素速率)
// 资源    : 少量计数器, 不使用块 RAM/DSP
// 注意    : 1. 输入像素流须为 1 像素/拍(与 pix_clk 同域), 且 line_end/
//              frame_end 与像素有效对齐;
//           2. 视频正极性同步(符合 DVI 控制编码)。
//=====================================================================
module hdmi_timing_direct #(
    parameter H_SYNC = 40,   // 行同步脉宽(像素时钟拍)
    parameter V_SYNC = 5     // 场同步脉宽(行数)
)(
    input  wire        pix_clk,
    input  wire        rst_n,         // 异步复位, 低有效

    // ---- 上游像素流(RGB565, 1 像素/拍, 与 pix_clk 同域) ----
    input  wire [15:0] pix_data,
    input  wire        pix_valid,     // 像素有效
    input  wire        line_end,      // 行末 1 拍脉冲
    input  wire        frame_end,     // 帧末 1 拍脉冲

    // ---- HDMI/DVI 视频输出 ----
    output reg         video_hs,
    output reg         video_vs,
    output reg         video_de,
    output reg  [23:0] video_rgb       // RGB888
);

    //-----------------------------------------------------------------
    // 行同步: line_end 后保持 H_SYNC 拍
    //-----------------------------------------------------------------
    reg [6:0] hs_cnt;
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n)
            hs_cnt <= 7'd0;
        else if (line_end)
            hs_cnt <= H_SYNC[6:0];
        else if (hs_cnt != 7'd0)
            hs_cnt <= hs_cnt - 7'd1;
    end
    wire hs_w = (hs_cnt != 7'd0);

    //-----------------------------------------------------------------
    // 场同步: frame_end 后保持 V_SYNC 行(按 line_end 计行)
    //-----------------------------------------------------------------
    reg [3:0] vs_lines;
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n)
            vs_lines <= 4'd0;
        else if (frame_end)
            vs_lines <= V_SYNC[3:0];
        else if (line_end && vs_lines != 4'd0)
            vs_lines <= vs_lines - 4'd1;
    end
    wire vs_w = (vs_lines != 4'd0);

    //-----------------------------------------------------------------
    // 输出寄存
    //-----------------------------------------------------------------
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            video_hs  <= 1'b0;
            video_vs  <= 1'b0;
            video_de  <= 1'b0;
            video_rgb <= 24'd0;
        end else begin
            video_hs  <= hs_w;
            video_vs  <= vs_w;
            video_de  <= pix_valid;
            // RGB565 -> RGB888 (低位补 0)
            video_rgb <= pix_valid ? {pix_data[15:11], 3'b000,
                                      pix_data[10:5],  2'b00,
                                      pix_data[4:0],   3'b000}
                                   : 24'h000000;
        end
    end

endmodule
