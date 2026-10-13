//=====================================================================
// 文件    : dvi_encoder.v
// 功能    : DVI/HDMI TMDS 8b/10b 编码器(单通道)
// 描述    : 按 DVI 1.0 规范把 8bit 像素数据编码为 10bit TMDS 字符,
//           并实现直流平衡(disparity)计数; de=0 时输出控制字符
//           (c1,c0 编码为 4 种固定 10bit 字符, 用于行/场同步)。
//           输出相对输入固定延迟 3 个时钟周期(数据/控制一致)。
// 平台    : 纯逻辑, 与器件无关(紫光同创 Logos-2 可直接综合)
// 资源    : 少量查找表与寄存器, 无 DSP/BRAM
//=====================================================================
module dvi_encoder (
    input  wire        clkin,     // 像素时钟
    input  wire        rstin,     // 异步复位(高有效)
    input  wire [7:0]  din,       // 像素数据(蓝/绿/红任一通道)
    input  wire        c0,        // 控制信号 0(通道 0: hsync)
    input  wire        c1,        // 控制信号 1(通道 0: vsync)
    input  wire        de,        // 数据使能(1=像素数据期)
    output reg  [9:0]  dout       // 10bit TMDS 字符
);

    //-----------------------------------------------------------------
    // 第 1 级: 输入寄存 + "1" 的个数统计(延迟 1 拍)
    //-----------------------------------------------------------------
    reg [7:0] din_q;
    reg [3:0] n1d;
    reg       c0_q, c1_q, de_q;

    always @(posedge clkin or posedge rstin) begin
        if (rstin) begin
            din_q <= 8'd0;
            n1d   <= 4'd0;
            c0_q  <= 1'b0; c1_q <= 1'b0; de_q <= 1'b0;
        end else begin
            din_q <= din;
            n1d   <= din[0] + din[1] + din[2] + din[3]
                   + din[4] + din[5] + din[6] + din[7];
            c0_q  <= c0;  c1_q  <= c1;  de_q <= de;
        end
    end

    //-----------------------------------------------------------------
    // 第 2 级(组合): 转移数最小化 q_m_next
    //-----------------------------------------------------------------
    wire       decision1 = (n1d > 4'd4) || ((n1d == 4'd4) && (din_q[0] == 1'b0));
    reg  [8:0] q_m_next;
    integer i;
    always @(*) begin
        q_m_next[0] = din_q[0];
        if (decision1) begin
            for (i = 1; i <= 7; i = i + 1)
                q_m_next[i] = q_m_next[i-1] ~^ din_q[i];
            q_m_next[8] = 1'b0;
        end else begin
            for (i = 1; i <= 7; i = i + 1)
                q_m_next[i] = q_m_next[i-1] ^ din_q[i];
            q_m_next[8] = 1'b1;
        end
    end

    wire [3:0] n1q_m = q_m_next[0] + q_m_next[1] + q_m_next[2] + q_m_next[3]
                     + q_m_next[4] + q_m_next[5] + q_m_next[6] + q_m_next[7];
    wire [3:0] n0q_m = 4'd8 - n1q_m;

    //-----------------------------------------------------------------
    // 第 2 级(时序): 直流平衡 + 输出字符(相对输入延迟 2 拍)
    //-----------------------------------------------------------------
    reg [4:0] cnt;          // 直流不平衡计数(带符号, -16..+16)
    reg [9:0] q_out;

    always @(posedge clkin or posedge rstin) begin
        if (rstin) begin
            q_out <= 10'd0;
            cnt   <= 5'd0;
        end else begin
            if (de_q) begin
                if ((cnt == 5'd0) || (n1q_m == n0q_m)) begin
                    q_out[9]   <= ~q_m_next[8];
                    q_out[8]   <= q_m_next[8];
                    q_out[7:0] <= q_m_next[8] ? q_m_next[7:0] : ~q_m_next[7:0];
                    cnt <= q_m_next[8]
                           ? cnt + {1'b0, n1q_m} - {1'b0, n0q_m}
                           : cnt + {1'b0, n0q_m} - {1'b0, n1q_m};
                end else begin
                    if ((~cnt[4] && (n1q_m > n0q_m)) ||
                        ( cnt[4] && (n1q_m < n0q_m))) begin
                        q_out[9]   <= 1'b1;
                        q_out[8]   <= q_m_next[8];
                        q_out[7:0] <= ~q_m_next[7:0];
                        cnt <= cnt + {q_m_next[8], 1'b0} + {1'b0, n0q_m} - {1'b0, n1q_m};
                    end else begin
                        q_out[9]   <= 1'b0;
                        q_out[8]   <= q_m_next[8];
                        q_out[7:0] <= q_m_next[7:0];
                        cnt <= cnt - {~q_m_next[8], 1'b0} + {1'b0, n1q_m} - {1'b0, n0q_m};
                    end
                end
            end else begin
                case ({c1_q, c0_q})
                    2'b00   : q_out <= 10'b1101010100;
                    2'b01   : q_out <= 10'b0010101011;
                    2'b10   : q_out <= 10'b0101010100;
                    default : q_out <= 10'b1010101011;
                endcase
                cnt <= 5'd0;
            end
        end
    end

    // 输出寄存(相对输入总延迟 3 拍)
    always @(posedge clkin or posedge rstin) begin
        if (rstin)
            dout <= 10'd0;
        else
            dout <= q_out;
    end

endmodule
