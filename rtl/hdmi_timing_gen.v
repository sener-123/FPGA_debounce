//=====================================================================
// 文件    : hdmi_timing_gen.v
// 功能    : HDMI(720p60)显示时序发生器 + 显示 FIFO 读侧
// 描述    : 按 CEA-861 720p60 时序(74.25MHz)产生 hsync/vsync/de,
//           从显示 FIFO(FWFT)取像素并转为 RGB888 输出;
//           数据时序: data_req 当拍请求 -> FIFO 数据下一拍到达,
//           video_de 相对 data_req 延迟 1 拍, 与 FIFO 读延迟对齐。
//           同时输出 load_pulse(场起始宽脉冲), 供 DDR3 读侧帧同步。
// 平台    : 紫光同创 Logos-2 PGL22G6CMBG324 (PGL22G 开发板)
// 时钟    : pix_clk = 74.25MHz(PLL)
// 资源    : 计数器与比较器, 不使用块 RAM
//=====================================================================
module hdmi_timing_gen #(
    parameter H_ACTIVE = 1280,   // 行有效像素
    parameter V_ACTIVE = 720,    // 场有效行
    parameter H_FP     = 110,    // 行前肩
    parameter H_SYNC   = 40,     // 行同步宽
    parameter H_BP     = 220,    // 行后肩
    parameter H_TOTAL  = 1650,
    parameter V_FP     = 5,      // 场前肩
    parameter V_SYNC   = 5,      // 场同步宽
    parameter V_BP     = 20,     // 场后肩
    parameter V_TOTAL  = 750
)(
    input  wire        pix_clk,
    input  wire        rst_n,         // 异步复位, 低有效

    // ---- 显示 FIFO 读口(FWFT) ----
    input  wire [15:0] pix_data,      // RGB565(队首数据)
    output wire        data_req,      // FIFO 读请求(有效窗口内)

    // ---- HDMI/DVI 视频输出 ----
    output reg         video_hs,
    output reg         video_vs,
    output reg         video_de,
    output reg  [23:0] video_rgb,     // RGB888

    // ---- DDR3 读侧帧同步(宽脉冲, 本时钟域) ----
    output reg         load_pulse
);

    //-----------------------------------------------------------------
    // 时序计数器
    //-----------------------------------------------------------------
    reg [11:0] h_cnt;
    reg [11:0] v_cnt;

    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            h_cnt <= 12'd0;
            v_cnt <= 12'd0;
        end else begin
            if (h_cnt == H_TOTAL-1) begin
                h_cnt <= 12'd0;
                if (v_cnt == V_TOTAL-1)
                    v_cnt <= 12'd0;
                else
                    v_cnt <= v_cnt + 12'd1;
            end else begin
                h_cnt <= h_cnt + 12'd1;
            end
        end
    end

    //-----------------------------------------------------------------
    // 有效窗口与同步
    //-----------------------------------------------------------------
    wire h_active = (h_cnt >= H_SYNC + H_BP) && (h_cnt < H_SYNC + H_BP + H_ACTIVE);
    wire v_active = (v_cnt >= V_SYNC + V_BP) && (v_cnt < V_SYNC + V_BP + V_ACTIVE);
    wire active   = h_active && v_active;

    // FIFO 读请求 = 有效窗口(FWFT: 数据下一拍到达)
    assign data_req = active;

    // 行/场同步(正极性, 符合 DVI 控制编码)
    wire hs_w = (h_cnt < H_SYNC);
    wire vs_w = (v_cnt < V_SYNC);

    // 帧起始宽脉冲(第 0 行前 64 拍, 供 DDR3 读侧同步)
    wire load_w = (v_cnt == 12'd0) && (h_cnt < 12'd64);

    //-----------------------------------------------------------------
    // 输出寄存(data_req 延迟 1 拍对齐 FIFO 读延迟)
    //   data_req(k) 当拍请求 -> FIFO dout 在 k+1 拍给出像素(k)
    //   de_d1(k+1) = data_req(k), 故 k+1 拍直接取 pix_data 即对齐
    //-----------------------------------------------------------------
    reg de_d1;
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            video_hs   <= 1'b0;
            video_vs   <= 1'b0;
            video_de   <= 1'b0;
            video_rgb  <= 24'd0;
            de_d1      <= 1'b0;
            load_pulse <= 1'b0;
        end else begin
            video_hs   <= hs_w;
            video_vs   <= vs_w;
            de_d1      <= data_req;
            video_de   <= de_d1;
            // RGB565 -> RGB888 (低位补 0), 与 video_de 同拍对齐
            video_rgb  <= de_d1 ? {pix_data[15:11], 3'b000,
                                   pix_data[10:5],  2'b00,
                                   pix_data[4:0],   3'b000} : 24'h000000;
            load_pulse <= load_w;
        end
    end

endmodule
