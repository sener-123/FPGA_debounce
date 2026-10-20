//=====================================================================
// 文件    : dvi_tx_top.v
// 功能    : DVI/HDMI(TMDS)发送器顶层
// 描述    : 3 通道 TMDS 8b/10b 编码 + 10:1 串行化(5x 像素时钟 DDR)+
//           时钟通道, 直接驱动 HDMI_B 接口的 4 对 TMDS 引脚。
//           通道分配(HDMI 标准): 通道 2 = 红 + DE, 通道 1 = 绿,
//           通道 0 = 蓝 + 行/场同步。
// 平台    : 正点原子达芬奇 Pro Logos2 (PG2L200H-6IFBB484)
// 时钟    : pclk = 74.25MHz, pclk_x5 = 371.25MHz(均由 PLL 产生,
//           两时钟相位对齐, 路由后需检查半周期时序,
//           在5x下降沿检测pclk, 准备下一上升沿使用的数据对)
// 资源    : 编码逻辑 + GTP_ODDR_E1/GTP_OUTBUFDS 输出原语
// 注意    : 1. GTP_ODDR_E1/GTP_OUTBUFDS 为紫光 Logos-2 的 DDR 输出寄存器
//              与 TMDS 3.3V 差分输出缓冲原语, 由 PDS 综合时识别;
//              见达芬奇手册第669页、第1524页。D0/D1在同一上升沿采样。
//           2. TMDS 位序为低位先发(LSB first)。
//=====================================================================
module dvi_tx_top (
    input  wire        pclk,          // 像素时钟 74.25MHz
    input  wire        pclk_x5,       // 5 倍像素时钟 371.25MHz
    input  wire        reset_n,       // 异步复位, 低有效

    // ---- 视频输入(与 pclk 同步) ----
    input  wire [23:0] video_din,     // RGB888
    input  wire        video_hsync,
    input  wire        video_vsync,
    input  wire        video_de,

    // ---- TMDS 差分输出 ----
    output wire        tmds_clk_p,
    output wire        tmds_clk_n,
    output wire [2:0]  tmds_data_p,
    output wire [2:0]  tmds_data_n
);

    //-----------------------------------------------------------------
    // 3 通道 TMDS 编码
    // 通道 2: R + {1'b0, de}
    // 通道 1: G + {1'b0, 1'b0}
    // 通道 0: B + {vsync, hsync}
    //-----------------------------------------------------------------
    wire [9:0] char2, char1, char0;

    dvi_encoder u_enc2 (
        .clkin (pclk), .rstin (~reset_n),
        .din   (video_din[23:16]),
        .c0    (video_de),   .c1    (1'b0),
        .de    (video_de),
        .dout  (char2)
    );
    dvi_encoder u_enc1 (
        .clkin (pclk), .rstin (~reset_n),
        .din   (video_din[15:8]),
        .c0    (1'b0),      .c1    (1'b0),
        .de    (video_de),
        .dout  (char1)
    );
    dvi_encoder u_enc0 (
        .clkin (pclk), .rstin (~reset_n),
        .din   (video_din[7:0]),
        .c0    (video_hsync), .c1 (video_vsync),
        .de    (video_de),
        .dout  (char0)
    );

    //-----------------------------------------------------------------
    // 10:1 串行化(每通道): pclk 域锁存 10bit 字符, pclk_x5 域按
    // 模 5 计数, 每拍输出 2 bit(D0=偶位, D1=奇位)给 GTP_ODDR_E1,
    // 低位先发。
    //-----------------------------------------------------------------
    wire ser2, ser1, ser0;
    wire ser_clk;

    dvi_serializer u_ser2 (
        .pclk    (pclk),
        .pclk_x5 (pclk_x5),
        .reset_n (reset_n),
        .char    (char2),
        .ser     (ser2)
    );
    dvi_serializer u_ser1 (
        .pclk    (pclk),
        .pclk_x5 (pclk_x5),
        .reset_n (reset_n),
        .char    (char1),
        .ser     (ser1)
    );
    dvi_serializer u_ser0 (
        .pclk    (pclk),
        .pclk_x5 (pclk_x5),
        .reset_n (reset_n),
        .char    (char0),
        .ser     (ser0)
    );

    // TMDS clock channel forwards one clock period per 10-bit character.
    GTP_ODDR_E1 #(.GRS_EN("TRUE"), .ODDR_MODE("SAME_EDGE"), .RS_TYPE("SYNC_RESET")) u_ck_oddr (
        .Q   (ser_clk),
        .D0  (1'b0),
        .D1  (1'b1),
        .CLK (pclk),
        .RS  (~reset_n),
        .CE  (1'b1)
    );

    //-----------------------------------------------------------------
    // TMDS 3.3V 差分输出
    //-----------------------------------------------------------------
    GTP_OUTBUFDS #(.IOSTANDARD("TMDS")) u_ck_lvds (.I(ser_clk), .O(tmds_clk_p), .OB(tmds_clk_n));

    GTP_OUTBUFDS #(.IOSTANDARD("TMDS")) u_d2_lvds (.I(ser2), .O(tmds_data_p[2]), .OB(tmds_data_n[2]));
    GTP_OUTBUFDS #(.IOSTANDARD("TMDS")) u_d1_lvds (.I(ser1), .O(tmds_data_p[1]), .OB(tmds_data_n[1]));
    GTP_OUTBUFDS #(.IOSTANDARD("TMDS")) u_d0_lvds (.I(ser0), .O(tmds_data_p[0]), .OB(tmds_data_n[0]));

endmodule


//=====================================================================
// dvi_serializer: 10:1 串行化器(5x 时钟, GTP_ODDR_E1 双沿输出)
//=====================================================================
module dvi_serializer (
    input  wire        pclk,
    input  wire        pclk_x5,
    input  wire        reset_n,
    input  wire [9:0]  char,
    output wire        ser
);

    // pclk 域: 锁存本像素字符
    reg [9:0] char_s;
    always @(posedge pclk or negedge reset_n) begin
        if (!reset_n)
            char_s <= 10'd0;
        else
            char_s <= char;
    end

    // Prepare pairs on the falling edge, stable before the ODDR rising edge.
    // Both bits belong to one held character; coherent PLL clocks are required.
    // pclk_x5 域: 检测 pclk 上升沿 -> 对齐模 5 计数器
    // At the first serial falling edge after pclk rises, capture the entire
    // character and prepare pair 0. The next four falling edges prepare
    // 第 1~4 对, 保证每像素周期恰好输出 10 位且无重复/遗漏。
    reg pclk_d;
    always @(negedge pclk_x5 or negedge reset_n) begin
        if (!reset_n)
            pclk_d <= 1'b0;
        else
            pclk_d <= pclk;
    end
    wire pclk_rise = pclk & ~pclk_d;

    reg [2:0] cnt5;
    reg [9:0] char_hold;
    reg       d0, d1;

    always @(negedge pclk_x5 or negedge reset_n) begin
        if (!reset_n) begin
            cnt5 <= 3'd0;
            char_hold <= 10'd0;
            d0   <= 1'b0;
            d1   <= 1'b0;
        end else begin
            if (pclk_rise) begin
                char_hold <= char_s;
                cnt5 <= 3'd1;
                d0   <= char_s[0];
                d1   <= char_s[1];
            end else begin
                if (cnt5 == 3'd4)
                    cnt5 <= 3'd0;
                else
                    cnt5 <= cnt5 + 3'd1;
                case (cnt5)
                    3'd0 : begin d0 <= char_hold[0]; d1 <= char_hold[1]; end
                    3'd1 : begin d0 <= char_hold[2]; d1 <= char_hold[3]; end
                    3'd2 : begin d0 <= char_hold[4]; d1 <= char_hold[5]; end
                    3'd3 : begin d0 <= char_hold[6]; d1 <= char_hold[7]; end
                    3'd4 : begin d0 <= char_hold[8]; d1 <= char_hold[9]; end
                    default : begin d0 <= 1'b0; d1 <= 1'b0; end
                endcase
            end
        end
    end

    GTP_ODDR_E1 #(.GRS_EN("TRUE"), .ODDR_MODE("SAME_EDGE"), .RS_TYPE("SYNC_RESET")) u_oddr (
        .Q   (ser),
        .D0  (d0),
        .D1  (d1),
        .CLK (pclk_x5),
        .RS  (~reset_n),
        .CE  (1'b1)
    );

endmodule
