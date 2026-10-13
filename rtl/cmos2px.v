//=====================================================================
// 文件    : cmos2px.v
// 功能    : 摄像头采集输出 -> 统一内部像素流(px_*)适配层
// 描述    : 将 ov5640_dri 中 cmos_capture_data 输出的
//             cmos_frame_vsync / cmos_frame_valid / cmos_frame_data
//            (已拼好的 RGB565, 每像素 1 拍有效) 转换为本工程的统一
//            内部像素流协议 px_*(见 接口说明.md)。
//            px_sof 约定: 帧起始 1 拍脉冲, 位于场消隐期(首像素之前),
//            由 cmos_frame_vsync 的上升沿产生。
// 平台    : 紫光同创 Logos-2 PGL22G6CMBG324 (PGL22G 开发板)
// 时钟    : 全部逻辑工作于摄像头像素时钟 cam_pclk(= px_clk)
// 资源    : 少量计数器与寄存器, 不使用块 RAM/DSP
// 注意    : 1. H_ACTIVE/V_ACTIVE 必须与 OV5640 寄存器配置的输出
//              分辨率一致(本工程默认 1280x720);
//           2. cmos_frame_vsync 为帧起始脉冲(高有效), 其上升沿
//              必须位于场消隐期, 早于本帧首像素。
//=====================================================================
module cmos2px #(
    parameter H_ACTIVE = 1280,   // 行有效像素数(须与摄像头输出一致)
    parameter V_ACTIVE = 720     // 场有效行数(须与摄像头输出一致)
)(
    // ---- 摄像头采集模块输出(正点原子 ov5640_dri 用户接口) ----
    input  wire        cam_pclk,          // 摄像头像素时钟(= px_clk)
    input  wire        rst_n,             // 异步复位, 低有效
    input  wire        cmos_frame_vsync,  // 帧起始脉冲(高有效)
    input  wire        cmos_frame_valid,  // 像素数据有效(1 像素/拍)
    input  wire [15:0] cmos_frame_data,   // RGB565 像素

    // ---- 统一内部像素流(见 接口说明.md) ----
    output wire        px_clk,      // 像素时钟(= cam_pclk)
    output wire        px_de,       // 像素有效
    output wire [15:0] px_data,     // RGB565
    output wire        px_sof,      // 帧起始(场消隐期 1 拍脉冲)
    output wire        px_eol,      // 行末像素同拍脉冲
    output wire        px_eof       // 帧末像素同拍脉冲
);

    //-----------------------------------------------------------------
    // 帧起始脉冲: cmos_frame_vsync 上升沿(场消隐期)
    //-----------------------------------------------------------------
    reg vsync_d1;
    always @(posedge cam_pclk or negedge rst_n)
        if (!rst_n) vsync_d1 <= 1'b0;
        else        vsync_d1 <= cmos_frame_vsync;

    wire vs_rise = cmos_frame_vsync & ~vsync_d1;

    //-----------------------------------------------------------------
    // 像素行列计数器(用于生成 eol/eof, 依赖像素有效而不是行信号,
    // 抗干扰自愈)
    //-----------------------------------------------------------------
    reg [11:0] col_cnt;
    reg [11:0] row_cnt;

    wire eol_w = (col_cnt == H_ACTIVE-1) && cmos_frame_valid;
    wire eof_w = (row_cnt == V_ACTIVE-1) && eol_w;

    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) begin
            col_cnt <= 12'd0;
            row_cnt <= 12'd0;
        end else begin
            if (vs_rise) begin
                col_cnt <= 12'd0;    // 帧起始: 计数复位(首像素之前)
                row_cnt <= 12'd0;
            end else if (cmos_frame_valid) begin
                if (eol_w) begin
                    col_cnt <= 12'd0;
                    row_cnt <= row_cnt + 12'd1;
                end else begin
                    col_cnt <= col_cnt + 12'd1;
                end
            end
        end
    end

    //-----------------------------------------------------------------
    // 统一像素流输出
    //-----------------------------------------------------------------
    assign px_clk  = cam_pclk;
    assign px_de   = cmos_frame_valid;
    assign px_data = cmos_frame_data;
    assign px_sof  = vs_rise;     // 帧起始(场消隐期, 首像素之前)
    assign px_eol  = eol_w;
    assign px_eof  = eof_w;

endmodule
