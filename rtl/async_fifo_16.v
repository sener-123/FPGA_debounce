//=====================================================================
// 文件    : async_fifo_16.v
// 功能    : 16bit 位宽异步 FIFO(首字直通 FWFT)
// 描述    : 写口工作于 DDR3 读侧时钟(74.25MHz), 读口工作于当前显示
//           时钟(HDMI: 74.25MHz / LCD: lcd_clk)。
//           FWFT: rd_data 始终呈现队首数据, rd_en 一个周期后更新为
//           下一个数据(读延迟 1 拍), 与 lcd_driver 的 data_req/lcd_de
//           时序正好对齐(见 使用说明.md 数据流章节)。
// 特性    : 格雷码指针跨时钟域同步; wr_afull 提供写侧流量控制;
//           rd_empty 供显示侧等待(上电初期)。
// 资源    : 1 块 BRAM(DEPTH x 16bit)
// 注意    : 1. DEPTH 必须为 2 的幂;
//           2. 帧同步时由 disp_reader 同时复位两侧指针(通过
//              wr_rst_n/rd_rst_n), 复位需保持 >= 3 个读时钟周期。
//=====================================================================
module async_fifo_16 #(
    parameter DEPTH        = 1024,   // 深度(2 的幂)
    parameter AFULL_MARGIN = 16      // 几乎满裕量(覆盖指针同步延迟)
)(
    // ---- 写口(DDR3 读侧时钟域) ----
    input  wire        wr_clk,
    input  wire        wr_rst_n,     // 异步复位, 低有效
    input  wire        wr_en,
    input  wire [15:0] wr_data,
    output wire        wr_afull,     // 几乎满(>= DEPTH-AFULL_MARGIN)

    // ---- 读口(显示时钟域) ----
    input  wire        rd_clk,
    input  wire        rd_rst_n,     // 异步复位, 低有效
    input  wire        rd_en,
    output wire [15:0] rd_data,      // FWFT: 队首数据
    output wire        rd_empty
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
    localparam AW    = clog2(DEPTH);
    localparam PTR_W = AW + 1;

    function [PTR_W-1:0] bin2gray;
        input [PTR_W-1:0] b;
        bin2gray = b ^ {1'b0, b[PTR_W-1:1]};
    endfunction

    function [PTR_W-1:0] gray2bin;
        input [PTR_W-1:0] g;
        integer i;
        reg [PTR_W-1:0] b;
        begin
            b[PTR_W-1] = g[PTR_W-1];
            for (i = PTR_W-2; i >= 0; i = i - 1)
                b[i] = b[i+1] ^ g[i];
            gray2bin = b;
        end
    endfunction

    //-----------------------------------------------------------------
    // 存储阵列与指针(声明集中在模块顶部)
    //-----------------------------------------------------------------
    reg [15:0] mem [0:DEPTH-1];

    // 写侧
    reg [PTR_W-1:0] wr_ptr_b;
    reg [PTR_W-1:0] wr_ptr_g;
    reg [PTR_W-1:0] rd_ptr_g_s , rd_ptr_g_ss;   // 读指针(格雷码)同步到写域
    reg [PTR_W-1:0] rd_ptr_b_s;                 // 同步后转二进制

    // 读侧
    reg [PTR_W-1:0] rd_ptr_b;
    reg [PTR_W-1:0] rd_ptr_g;
    reg [PTR_W-1:0] wr_ptr_g_s , wr_ptr_g_ss;   // 写指针(格雷码)同步到读域
    reg [PTR_W-1:0] wr_ptr_b_s;
    reg [15:0]      rd_data_r;

    wire [PTR_W-1:0] wr_level = wr_ptr_b - rd_ptr_b_s;
    wire [PTR_W-1:0] rd_level = wr_ptr_b_s - rd_ptr_b;
    wire             rd_empty_w = (rd_level == {PTR_W{1'b0}});

    //-----------------------------------------------------------------
    // 写侧
    //-----------------------------------------------------------------
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_ptr_b    <= {PTR_W{1'b0}};
            wr_ptr_g    <= {PTR_W{1'b0}};
            rd_ptr_g_s  <= {PTR_W{1'b0}};
            rd_ptr_g_ss <= {PTR_W{1'b0}};
            rd_ptr_b_s  <= {PTR_W{1'b0}};
        end else begin
            if (wr_en) begin
                mem[wr_ptr_b[AW-1:0]] <= wr_data;
                wr_ptr_b <= wr_ptr_b + {{(PTR_W-1){1'b0}}, 1'b1};
                wr_ptr_g <= bin2gray(wr_ptr_b + {{(PTR_W-1){1'b0}}, 1'b1});
            end
            rd_ptr_g_s  <= rd_ptr_g;
            rd_ptr_g_ss <= rd_ptr_g_s;
            rd_ptr_b_s  <= gray2bin(rd_ptr_g_ss);
        end
    end

    assign wr_afull = (wr_level >= (DEPTH - AFULL_MARGIN));

    //-----------------------------------------------------------------
    // 读侧(FWFT)
    //-----------------------------------------------------------------
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_ptr_b    <= {PTR_W{1'b0}};
            rd_ptr_g    <= {PTR_W{1'b0}};
            wr_ptr_g_s  <= {PTR_W{1'b0}};
            wr_ptr_g_ss <= {PTR_W{1'b0}};
            wr_ptr_b_s  <= {PTR_W{1'b0}};
            rd_data_r   <= 16'd0;
        end else begin
            if (rd_en && !rd_empty_w) begin
                rd_ptr_b <= rd_ptr_b + {{(PTR_W-1){1'b0}}, 1'b1};
                rd_ptr_g <= bin2gray(rd_ptr_b + {{(PTR_W-1){1'b0}}, 1'b1});
            end
            wr_ptr_g_s  <= wr_ptr_g;
            wr_ptr_g_ss <= wr_ptr_g_s;
            wr_ptr_b_s  <= gray2bin(wr_ptr_g_ss);
            rd_data_r   <= mem[rd_ptr_b[AW-1:0]];
        end
    end

    assign rd_data  = rd_data_r;
    assign rd_empty = rd_empty_w;

endmodule
