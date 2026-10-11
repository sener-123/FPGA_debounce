//=====================================================================
// 文件    : video_out_if.v
// 功能    : 视频输出接口适配层(解耦层)
// 描述    : 将统一内部像素流(px_*)转换为标准显示时序
//           (hsync/vsync/de + RGB888 数据), 供 HDMI/LCD 等显示驱动使用。
//           防抖输出的有效窗口比输入小(四周各裁剪 MAX_SHIFT+1 像素),
//           本模块把有效窗口居中放到显示帧内, 窗口外输出黑边。
//           算法核心与具体显示器件完全解耦: 更换显示器件时
//           只需修改本模块的时序参数。
// 平台    : 紫光同创 Logos-2 PGL22G6CMBG324 (PGL22G 开发板)
// 时钟    : 像素时钟 px_clk(与输入像素流同源)
// 资源    : 计数器与比较器, 不使用块 RAM
// 注意    : 1. 输入像素流的总行周期必须等于 H_TOTAL 个 px_clk,
//              输入帧周期必须等于 H_TOTAL*V_TOTAL 个 px_clk
//              (本设计为"同频直通", 摄像头输出帧率 = 显示帧率);
//           2. 帧起始 px_sof 到来时对计数器强制同步, 消除漂移。
//=====================================================================
module video_out_if #(
    parameter H_ACTIVE = 1280,   // 输入(原始)行有效像素数
    parameter V_ACTIVE = 720,    // 输入(原始)场有效行数
    parameter H_TOTAL  = 1650,   // 显示行总周期(720p60 = 1650)
    parameter V_TOTAL  = 750,    // 显示场总周期(720p60 = 750)
    parameter H_FP     = 110,    // 行前肩
    parameter H_BP     = 220,    // 行后肩
    parameter H_SYNC   = 40,     // 行同步宽
    parameter V_FP     = 5,      // 场前肩
    parameter V_BP     = 20,     // 场后肩
    parameter V_SYNC   = 5,      // 场同步宽
    parameter BORDER   = 9,      // 四周黑边宽 = MAX_SHIFT + 1
    parameter PX_CLK_PER_PIX = 1, // 每像素占用的 px_clk 周期数
                                  // (OV5640 RGB565 双字节输出时为 2)
    parameter HS_POL   = 1,      // 行同步极性(1=高有效)
    parameter VS_POL   = 1       // 场同步极性(1=高有效)
)(
    // ---- 统一内部像素流输入(来自 shift_compensate) ----
    input  wire        px_clk,
    input  wire        rst_n,        // 异步复位, 低有效
    input  wire        px_de,        // 像素有效
    input  wire [15:0] px_data,      // RGB565
    input  wire        px_sof,       // 帧起始
    input  wire        px_eol,       // 行结束
    input  wire        px_eof,       // 帧结束

    // ---- 标准显示时序输出 ----
    output reg         disp_hsync,
    output reg         disp_vsync,
    output reg         disp_de,
    output reg  [23:0] disp_data     // RGB888
);

    // 防抖后的有效窗口尺寸(行方向按 px_clk 周期折算)
    localparam H_OUT = H_ACTIVE - 2*BORDER;
    localparam V_OUT = V_ACTIVE - 2*BORDER;
    localparam H_OUT_PX = H_OUT * PX_CLK_PER_PIX;

    //-----------------------------------------------------------------
    // 显示时序计数器(自由运行, 在 px_sof 处强制同步)
    // px_sof 与首像素同拍, 复位要到下一拍才生效: 用组合逻辑取"生效值",
    // 保证首像素即落在显示有效窗口左上角, 不丢首像素。
    //-----------------------------------------------------------------
    reg [11:0] h_cnt;
    reg [11:0] v_cnt;

    wire [11:0] h_cnt_eff = px_sof ? H_FP : h_cnt;
    wire [11:0] v_cnt_eff = px_sof ? V_FP : v_cnt;

    always @(posedge px_clk or negedge rst_n) begin
        if (!rst_n) begin
            h_cnt <= 12'd0;
            v_cnt <= 12'd0;
        end else begin
            if (px_sof) begin
                // 帧首像素强制对齐到显示有效窗口左上角。
                // 强制值 +1: 计数器在 sof 拍之后的下一拍才生效,
                // 而首像素已由组合逻辑取 H_FP, +1 补偿这一拍延迟,
                // 保证后续各行首像素都落在 H_FP。
                h_cnt <= H_FP + 1;
                v_cnt <= V_FP;
            end else if (h_cnt == H_TOTAL-1) begin
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
    // 有效窗口与同步信号
    //-----------------------------------------------------------------
    wire in_active = (v_cnt_eff >= V_FP) && (v_cnt_eff < V_FP + V_OUT) &&
                     (h_cnt_eff >= H_FP) && (h_cnt_eff < H_FP + H_OUT_PX);

    wire hsync_w = (h_cnt_eff < H_SYNC);
    wire vsync_w = (v_cnt_eff < V_SYNC);

    always @(posedge px_clk or negedge rst_n) begin
        if (!rst_n) begin
            disp_hsync <= 1'b0;
            disp_vsync <= 1'b0;
            disp_de    <= 1'b0;
            disp_data  <= 24'd0;
        end else begin
            disp_hsync <= (HS_POL == 1) ? hsync_w : ~hsync_w;
            disp_vsync <= (VS_POL == 1) ? vsync_w : ~vsync_w;
            disp_de    <= in_active & px_de;
            if (in_active & px_de) begin
                // RGB565 -> RGB888 位扩展(低位复制, 视觉无差异)
                disp_data <= {px_data[15:11], px_data[15:13],
                              px_data[10:5],  px_data[10:9],
                              px_data[4:0],   px_data[4:2]};
            end else begin
                disp_data <= 24'd0;   // 窗口外输出黑边
            end
        end
    end

endmodule
