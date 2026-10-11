//=====================================================================
// 文件    : video_in_if.v
// 功能    : 视频输入接口适配层(解耦层)
// 描述    : 将 OV5640 摄像头(8bit DVP 总线, RGB565 每像素 2 字节)的并行
//           视频信号转换为工程内部统一的像素流接口(px_* 信号组)。
//           防抖算法核心只与统一像素流交互, 与具体传感器完全解耦:
//           更换摄像头/视频源时只需修改(或替换)本模块。
// 平台    : 紫光同创 Logos-2 PGL22G6CMBG324 (PGL22G 开发板)
// 时钟    : 全部逻辑工作于摄像头像素时钟 cam_pclk(= 内部 px_clk)
// 资源    : 仅少量寄存器, 不使用块 RAM
// 注意    : 1. OV5640 的寄存器初始化(SCCB/I2C 配置)由外部摄像头驱动
//              模块完成, 本模块只负责并行视频总线的时序适配;
//           2. H_ACTIVE/V_ACTIVE 必须与摄像头实际输出分辨率一致
//              (推荐 1280x720@60 RGB565)。
//=====================================================================
module video_in_if #(
    parameter H_ACTIVE  = 1280,   // 行有效像素数(须与摄像头输出一致)
    parameter V_ACTIVE  = 720,    // 场有效行数(须与摄像头输出一致)
    parameter VS_POL    = 1,      // 场同步有效电平(1=高有效, 0=低有效)
    parameter HREF_POL  = 1       // 行参考有效电平(1=高有效, 0=低有效)
)(
    // ---- 摄像头并行视频总线(原始信号) ----
    input  wire        cam_pclk,    // 摄像头像素时钟
    input  wire        cam_vsync,   // 场同步
    input  wire        cam_href,    // 行参考
    input  wire [7:0]  cam_data,    // 8bit 数据(RGB565 高/低字节交替)
    input  wire        rst_n,       // 异步复位, 低有效

    // ---- 统一内部像素流(解耦后的视频接口, 见 接口说明.md) ----
    output wire        px_clk,      // 像素时钟(= cam_pclk)
    output wire        px_de,       // 像素有效(1 个像素/拍)
    output wire [15:0] px_data,     // RGB565 像素数据
    output wire        px_sof,      // 帧起始标志(场消隐期脉冲, 早于首像素)
    output wire        px_eol,      // 行结束标志(末像素同拍脉冲)
    output wire        px_eof       // 帧结束标志(末像素同拍脉冲)
);

    //-----------------------------------------------------------------
    // 同步极性处理
    //-----------------------------------------------------------------
    wire vsync_pol = (VS_POL   == 1) ? cam_vsync : ~cam_vsync;
    wire href_pol  = (HREF_POL == 1) ? cam_href  : ~cam_href;

    reg vsync_d1;
    always @(posedge cam_pclk or negedge rst_n)
        if (!rst_n) vsync_d1 <= 1'b0;
        else        vsync_d1 <= vsync_pol;

    wire vs_rise =  vsync_pol & ~vsync_d1;   // 场同步上升沿
    wire vs_fall = ~vsync_pol &  vsync_d1;   // 场同步下降沿(仅调试用)

    //-----------------------------------------------------------------
    // RGB565 字节拼接: OV5640 每像素 2 字节
    //   第 1 字节 = {R[4:0], G[5:3]} -> px_data[15:8]
    //   第 2 字节 = {G[2:0], B[4:0]} -> px_data[7:0]
    //-----------------------------------------------------------------
    reg        byte_sel;         // 0=等待高字节, 1=等待低字节
    reg [7:0]  byte_hi;          // 高字节暂存
    reg        de_r;             // 像素有效(输出前寄存)
    reg [15:0] data_r;           // 像素数据(输出前寄存)

    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) begin
            byte_sel <= 1'b0;
            byte_hi  <= 8'd0;
            de_r     <= 1'b0;
            data_r   <= 16'd0;
        end else begin
            if (href_pol) begin
                if (!byte_sel) begin
                    byte_hi  <= cam_data;      // 收高字节
                    byte_sel <= 1'b1;
                    de_r     <= 1'b0;
                end else begin
                    data_r   <= {byte_hi, cam_data};  // 拼出完整 RGB565
                    byte_sel <= 1'b0;
                    de_r     <= 1'b1;
                end
            end else begin
                byte_sel <= 1'b0;   // 行消隐期间复位拼接状态, 防止跨行错位
                de_r     <= 1'b0;
            end
        end
    end

    //-----------------------------------------------------------------
    // 像素行列计数器(用于生成 eol/eof, 不依赖 vsync/href 下降沿时序)
    // 注意: px_sof 在场消隐期(首个像素之前)给出 1 拍脉冲,
    //       使所有下游计数器的复位先于首像素生效, 避免首像素错位。
    //-----------------------------------------------------------------
    reg [11:0] col_cnt;
    reg [11:0] row_cnt;

    wire sof_w  = vs_rise;                    // 帧起始脉冲(场消隐期)
    wire eol_w  = (col_cnt == H_ACTIVE-1) && de_r;   // 行末像素
    wire eof_w  = (row_cnt == V_ACTIVE-1) && eol_w;  // 帧末像素

    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) begin
            col_cnt  <= 12'd0;
            row_cnt  <= 12'd0;
        end else begin
            if (sof_w) begin
                col_cnt <= 12'd0;   // 场消隐期复位, 首像素时列=0
                row_cnt <= 12'd0;
            end else if (de_r) begin
                if (eol_w) begin
                    col_cnt <= 12'd0;    // 行末: 下一拍回 0
                    row_cnt <= row_cnt + 12'd1;
                end else begin
                    col_cnt <= col_cnt + 12'd1;
                end
            end else if (!href_pol) begin
                col_cnt <= 12'd0;        // 行消隐期间强制回 0(抗扰自愈)
            end
        end
    end

    //-----------------------------------------------------------------
    // 统一像素流输出
    //-----------------------------------------------------------------
    assign px_clk  = cam_pclk;
    assign px_de   = de_r;
    assign px_data = data_r;
    assign px_sof  = sof_w;    // 帧起始(场消隐期, 首像素之前 1 拍脉冲)
    assign px_eol  = eol_w;
    assign px_eof  = eof_w;

endmodule
