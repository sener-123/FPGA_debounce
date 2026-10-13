//=====================================================================
// 文件    : shift_compensate.v
// 功能    : 像素平移补偿(参考算法"warpAffine 变换"环节的硬件实现)
// 描述    : 用 1 块环形 BRAM 把视频流整体平移 (DX, DY) 像素:
//             out(ox,oy) = in(fx - DX, fy - DY),
//             fx = crop_col0 + ox, fy = crop_row0 + oy,
//             DX = SHIFT_BASE + comp_x, DY = SHIFT_BASE + comp_y,
//             comp 为 trajectory_smooth 输出的抖动补偿量(±MAX_SHIFT)。
//            SHIFT_BASE(MAX_SHIFT+1) 是基准延迟: 保证环形 BRAM 的读
//            地址永远落后写地址, 同时对应参考实现的固定边框偏移。
//            裁剪窗口 (crop_col0, crop_row0, crop_w, crop_h) 可配置:
//              - HDMI 模式: 全帧 0,0,1280,720
//              - LCD 模式 : 与屏幕分辨率相同的居中/对齐窗口
//            源坐标越出本帧(fx-DX<0 或 fy-DY<0)的像素输出黑色,
//            对应参考实现的 fixBorder(去黑边)语义。
// 资源    : 1 块环形 BRAM(RING_DEPTH x 16bit) + 少量计数器
// 注意    : 1. RING_DEPTH 为 2 的幂, 且 >= (2*MAX_SHIFT+1)*(H_ACTIVE+1);
//           2. 上电后第一帧输出无效(环形缓冲尚未填满), 第二帧起正常;
//           3. crop_* 建议在帧起始前更新, 帧内保持不变。
//=====================================================================
module shift_compensate #(
    parameter H_ACTIVE   = 1280,   // 输入行有效像素数
    parameter V_ACTIVE   = 720,    // 输入场有效行数
    parameter MAX_SHIFT  = 8,      // 最大补偿幅度(±像素)
    parameter RING_DEPTH = 32768   // 环形缓冲深度(2 的幂)
)(
    input  wire        px_clk,
    input  wire        rst_n,      // 异步复位, 低有效
    // ---- 统一内部像素流输入 ----
    input  wire        px_de,
    input  wire [15:0] px_data,    // RGB565
    input  wire        px_sof,     // 帧起始(场消隐期)
    // ---- 补偿量(来自 trajectory_smooth) ----
    input  wire        comp_valid,
    input  wire [5:0]  comp_x,     // 有符号, ±MAX_SHIFT
    input  wire [5:0]  comp_y,
    // ---- 输出裁剪窗口(帧坐标, 帧起始时锁存) ----
    input  wire [11:0] crop_col0,  // 窗口左上角列
    input  wire [11:0] crop_row0,  // 窗口左上角行
    input  wire [11:0] crop_w,     // 窗口宽(像素)
    input  wire [11:0] crop_h,     // 窗口高(行)
    // ---- 统一内部像素流输出(已平移并裁剪) ----
    output reg         px_de_o,
    output reg  [15:0] px_data_o,
    output reg         px_sof_o,
    output reg         px_eol_o,
    output reg         px_eof_o
);

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

    localparam SHIFT_BASE = MAX_SHIFT + 1;   // 基准延迟(避免读写同地址冲突)
    localparam AW         = clog2(RING_DEPTH);   // 环形地址位宽
    localparam MASK       = RING_DEPTH - 1;

    //-----------------------------------------------------------------
    // 环形缓冲: 写入地址 = 绝对像素序号(低位为地址), 读取地址 = 写入
    // 位置回退 (DY*W + DX) 个像素, 从而得到平移后的内容。
    //-----------------------------------------------------------------
    reg [31:0] wr_cnt;                // 绝对写计数(低位即环形地址)
    reg [31:0] frame_base;            // 帧首像素的绝对写计数(px_sof 锁存)
    reg [31:0] rd_frame_base;         // 本帧读基址 = frame_base - DY*W - DX

    reg [5:0] comp_x_r, comp_y_r;     // 补偿量锁存
    reg [11:0] c0_r, r0_r, cw_r, ch_r; // 裁剪窗口锁存(帧起始生效)

    // 当前帧补偿: DX/DY ∈ [1, 2*MAX_SHIFT+1]
    wire [5:0] dx_w = SHIFT_BASE + $signed(comp_x_r);
    wire [5:0] dy_w = SHIFT_BASE + $signed(comp_y_r);

    //-----------------------------------------------------------------
    // 环形 BRAM(由综合器映射为块 RAM)
    //-----------------------------------------------------------------
    reg [15:0] ring [0:RING_DEPTH-1];
    reg [15:0] ring_q;

    //-----------------------------------------------------------------
    // 输入行列计数(第 k 个像素: x_in = k-1)
    //-----------------------------------------------------------------
    reg [11:0] x_in;
    reg [11:0] y_in;

    // 帧起始周期内计数器/读基址尚未更新, 用组合逻辑取"生效值"
    wire [11:0] x_eff   = px_sof ? 12'd0 : x_in;
    wire [11:0] y_eff   = px_sof ? 12'd0 : y_in;
    wire [31:0] rd_base_eff = px_sof ? (wr_cnt - dy_w*H_ACTIVE - dx_w)
                                     : rd_frame_base;

    // 裁剪窗口内(窗口坐标 = 帧坐标 - crop 原点)
    wire emit = px_de
              & (y_eff >= r0_r) & (y_eff < (r0_r + ch_r))
              & (x_eff >= c0_r) & (x_eff < (c0_r + cw_r));

    // 源坐标在本帧内(否则输出黑色, 对应 fixBorder)
    // 源行 = y_eff - DY, 源列 = x_eff - DX; 越界只在左上(负方向),
    // 因 y_eff/x_eff 自身不超过帧尺寸。
    wire in_frame = (y_eff >= {6'd0, dy_w}) & (x_eff >= {6'd0, dx_w});

    // 读地址 = rd_base_eff + y_eff*W + x_eff (两补码相加, 取低位)
    wire [31:0] rd_sum  = rd_base_eff + (y_eff * H_ACTIVE) + x_eff;
    wire [AW-1:0] rd_addr = rd_sum[AW-1:0] & MASK[AW-1:0];

    always @(posedge px_clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_cnt        <= 32'd0;
            frame_base    <= 32'd0;
            rd_frame_base <= 32'd0;
            comp_x_r      <= 6'd0;
            comp_y_r      <= 6'd0;
            c0_r          <= 12'd0;
            r0_r          <= 12'd0;
            cw_r          <= 12'd0;
            ch_r          <= 12'd0;
            x_in          <= 12'd0;
            y_in          <= 12'd0;
        end else begin
            // 补偿量锁存(帧内多次有效也仅覆盖, 帧起始时生效)
            if (comp_valid) begin
                comp_x_r <= comp_x;
                comp_y_r <= comp_y;
            end
            if (px_sof) begin
                frame_base    <= wr_cnt;
                rd_frame_base <= wr_cnt - dy_w*H_ACTIVE - dx_w;
                c0_r          <= crop_col0;
                r0_r          <= crop_row0;
                cw_r          <= crop_w;
                ch_r          <= crop_h;
                x_in          <= 12'd0;
                y_in          <= 12'd0;
            end else if (px_de) begin
                if (x_in == H_ACTIVE-1) begin
                    x_in <= 12'd0;
                    y_in <= (y_in == V_ACTIVE-1) ? 12'd0 : y_in + 12'd1;
                end else begin
                    x_in <= x_in + 12'd1;
                end
            end

            // 写计数(像素有效时推进)
            if (px_de)
                wr_cnt <= wr_cnt + 32'd1;
        end
    end

    // 环形缓冲写与读
    always @(posedge px_clk) begin
        if (px_de)
            ring[wr_cnt & MASK[31:0]] <= px_data;
        ring_q <= ring[rd_addr];
    end

    //-----------------------------------------------------------------
    // 输出侧(读数据延迟 1 拍与 BRAM 读延迟对齐)
    //-----------------------------------------------------------------
    reg emit_d;
    reg in_frame_d;
    reg sof_f, eol_f, eof_f;

    always @(posedge px_clk or negedge rst_n) begin
        if (!rst_n) begin
            emit_d     <= 1'b0;
            in_frame_d <= 1'b0;
            px_de_o    <= 1'b0;
            px_data_o  <= 16'd0;
            px_sof_o   <= 1'b0;
            px_eol_o   <= 1'b0;
            px_eof_o   <= 1'b0;
            sof_f      <= 1'b0;
            eol_f      <= 1'b0;
            eof_f      <= 1'b0;
        end else begin
            emit_d     <= emit;
            in_frame_d <= in_frame;
            sof_f      <= emit & (y_eff == r0_r) & (x_eff == c0_r);
            eol_f      <= emit & (x_eff == c0_r + cw_r - 12'd1);
            eof_f      <= emit & (y_eff == r0_r + ch_r - 12'd1)
                              & (x_eff == c0_r + cw_r - 12'd1);
            px_de_o    <= emit_d;
            px_data_o  <= in_frame_d ? ring_q : 16'h0000;   // 越界填黑
            px_sof_o   <= sof_f;
            px_eol_o   <= eol_f;
            px_eof_o   <= eof_f;
        end
    end

endmodule
