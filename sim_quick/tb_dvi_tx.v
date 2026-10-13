//=====================================================================
// 测试台  : tb_dvi_tx.v
// 目的    : 验证 TMDS 编码器与 5x 串行化器
// 方法    : 1. 参照 DVI 1.0 规范的方程实现独立参考编码器, 与
//              dvi_encoder 逐拍比对 3 个通道的 10bit 字符;
//           2. 在 pclk_x5 双沿采样 TMDS 数据位, 每像素周期重建
//              10bit 串行位流, 与对应通道字符的低位先发序列
//              比对(以消隐期固定字符锁定旋转相位, 数据期锁定
//              字符滞后, 随后逐窗口严格比对)。
// 运行    : ModelSim 编译 rtl/dvi_encoder.v rtl/dvi_tx_top.v
//           sim/oddrx1_model.v sim/elvds_obuf_model.v + 本文件
//           vsim tb_dvi_tx -do "run -all"
//=====================================================================
`timescale 1ns/1ps
module tb_dvi_tx;

    //-----------------------------------------------------------------
    // 时钟: pclk = pclk_x5 / 5 (沿对齐, pclk 高 3 拍低 2 拍)
    //-----------------------------------------------------------------
    reg clk5 = 1'b0;
    always #4 clk5 = ~clk5;                // 125MHz
    reg [2:0] pcnt = 0;
    reg pclk = 1'b1;
    always @(posedge clk5) begin
        pcnt <= (pcnt == 3'd4) ? 3'd0 : pcnt + 3'd1;
        if (pcnt == 3'd0)      pclk <= 1'b1;
        else if (pcnt == 3'd3) pclk <= 1'b0;
    end

    reg rst_n = 1'b0;

    //-----------------------------------------------------------------
    // 视频激励(简化时序: 每行 20 拍, 行内 10 拍数据, 10 行/场)
    //-----------------------------------------------------------------
    reg [11:0] hcnt = 0;
    reg [11:0] vcnt = 0;
    reg [7:0]  din_r, din_g, din_b;
    reg        hs, vs, de;

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            hcnt <= 12'd0; vcnt <= 12'd0;
            din_r <= 8'd0; din_g <= 8'd0; din_b <= 8'd0;
            hs <= 1'b0; vs <= 1'b0; de <= 1'b0;
        end else begin
            if (hcnt == 12'd19) begin
                hcnt <= 12'd0;
                if (vcnt == 12'd9) vcnt <= 12'd0;
                else               vcnt <= vcnt + 12'd1;
            end else
                hcnt <= hcnt + 12'd1;
            de <= (hcnt >= 12'd4) && (hcnt < 12'd14) && (vcnt < 12'd8);
            hs <= (hcnt < 12'd2);
            vs <= (vcnt == 12'd8) && (hcnt < 12'd4);   // 场同步(含 {1,1} 控制组合)
            din_r <= din_r + 8'd7;
            din_g <= din_g - 8'd3;
            din_b <= din_b + 8'd13;
        end
    end

    //-----------------------------------------------------------------
    // 被测模块
    //-----------------------------------------------------------------
    wire tmds_clk_p, tmds_clk_n;
    wire [2:0] tmds_data_p, tmds_data_n;

    dvi_tx_top dut (
        .pclk        (pclk),
        .pclk_x5     (clk5),
        .reset_n     (rst_n),
        .video_din   ({din_r, din_g, din_b}),
        .video_hsync (hs),
        .video_vsync (vs),
        .video_de    (de),
        .tmds_clk_p  (tmds_clk_p),
        .tmds_clk_n  (tmds_clk_n),
        .tmds_data_p (tmds_data_p),
        .tmds_data_n (tmds_data_n)
    );

    //-----------------------------------------------------------------
    // 参考编码器(独立实现, 3 级流水与 dvi_encoder 一致)
    //-----------------------------------------------------------------
    wire [9:0] ref0, ref1, ref2;
    ref_dvi_enc u_ref0 (.clk(pclk), .rst(~rst_n), .din(din_b), .c0(hs), .c1(vs), .de(de), .dout(ref0));
    ref_dvi_enc u_ref1 (.clk(pclk), .rst(~rst_n), .din(din_g), .c0(1'b0), .c1(1'b0), .de(de), .dout(ref1));
    ref_dvi_enc u_ref2 (.clk(pclk), .rst(~rst_n), .din(din_r), .c0(de), .c1(1'b0), .de(de), .dout(ref2));

    //-----------------------------------------------------------------
    // 检查 1: 编码字符比对(3 通道)
    //-----------------------------------------------------------------
    integer errs = 0;
    integer cyc_cnt = 0;
    always @(posedge pclk) begin
        if (rst_n && cyc_cnt > 20) begin
            if (dut.u_enc0.dout !== ref0) begin
                $display("ERROR[%0d] ch0: dut=%b ref=%b (din=%h c0=%b c1=%b de=%b)",
                         cyc_cnt, dut.u_enc0.dout, ref0, din_b, hs, vs, de);
                errs = errs + 1;
            end
            if (dut.u_enc1.dout !== ref1) begin
                $display("ERROR[%0d] ch1: dut=%b ref=%b", cyc_cnt, dut.u_enc1.dout, ref1);
                errs = errs + 1;
            end
            if (dut.u_enc2.dout !== ref2) begin
                $display("ERROR[%0d] ch2: dut=%b ref=%b", cyc_cnt, dut.u_enc2.dout, ref2);
                errs = errs + 1;
            end
        end
        cyc_cnt = cyc_cnt + 1;
        if (errs > 20) begin
            $display("too many errors, aborting");
            $stop;
        end
    end

    //-----------------------------------------------------------------
    // 检查 2: 串行化器位序验证(通道 0)
    // 方法: 每个 pclk_x5 下降沿采样 (d0,d1) 对, 连续 2000 对(4000 位);
    //           按物理时间顺序存入 pair_bits(索引 0 为最早一位, 每对内
    //           d0 在前 d1 在后, 与 ODDRX1 实际输出一致); 同时记录
    //           char_s 全历史; 结束仿真后寻找相位偏移与滞后, 使位流
    //           与 "char_s 低位先发" 完全一致。串行器逻辑(cnt5 选择 +
    //           d0/d1 寄存器)由此得到完整验证; ODDRX1 差分输出由器件
    //           原语保证(按数据手册行为)。
    //-----------------------------------------------------------------
    reg [3999:0] pair_bits = 4000'd0;
    integer      pair_cnt = 0;
    reg [9:0]    ch_all [0:1023];    // char_s 全历史(1024 个像素周期)
    integer      ch_wr = 0;

    always @(negedge clk5) begin
        if (pair_cnt < 2000) begin
            // 右移 2 位, 新 (d0,d1) 对从高位进入; 索引 0 恒为最早位。
            pair_bits = {dut.u_ser0.d1, dut.u_ser0.d0, pair_bits[3999:2]};
            pair_cnt  = pair_cnt + 1;
        end
    end

    always @(posedge pclk) begin
        ch_all[ch_wr % 1024] <= dut.u_ser0.char_s;
        ch_wr <= ch_wr + 1;
    end

    integer ser_errs = 0;
    task check_serial;
        integer o, l, i, ok;
        integer bit_idx, ch_idx;
        reg [9:0] c;
        begin
            ok = 0;
            for (o = 0; o < 10 && !ok; o = o + 1) begin
                for (l = 0; l < 4 && !ok; l = l + 1) begin
                    ok = 1;
                    for (i = 200; i < 4000; i = i + 1) begin
                        bit_idx = (i + o) % 10;
                        ch_idx  = (i + o) / 10 - l;
                        c = ch_all[ch_idx % 1024];
                        if (pair_bits[i] !== c[bit_idx]) begin
                            ok = 0;
                        end
                    end
                end
            end
            if (!ok) begin
                $display("SERIAL FAIL: no (offset,lag) matches the char stream");
                for (i = 0; i < 30; i = i + 1)
                    $display("DBG ch_all[%0d]=%b", i, ch_all[i]);
                for (i = 0; i < 300; i = i + 10)
                    $display("DBG pb[%0d..%0d]=%b", i, i+9, pair_bits[i +: 10]);
                ser_errs = 1;
            end else begin
                $display("SERIAL OK: serializer pairs match char_s (10b LSB-first)");
            end
        end
    endtask

    //-----------------------------------------------------------------
    // 结束
    //-----------------------------------------------------------------
    initial begin
        repeat (10) @(posedge pclk);
        rst_n = 1'b1;
        #40000;                              // 400 个像素周期
        check_serial;
        if (errs == 0 && ser_errs == 0)
            $display("=== tb_dvi_tx PASS (encoder compare + serial bits) ===");
        else begin
            if (errs != 0)
                $display("=== tb_dvi_tx FAIL: %0d encoder errors ===", errs);
            if (ser_errs != 0)
                $display("=== tb_dvi_tx FAIL: serial check ===");
        end
        $stop;
    end

endmodule


//=====================================================================
// 参考 TMDS 编码器(按 DVI 1.0 规范方程独立实现, 3 级流水)
//=====================================================================
module ref_dvi_enc (
    input            clk,
    input            rst,          // 高有效
    input  [7:0]     din,
    input            c0, c1, de,
    output reg [9:0] dout
);

    reg [7:0] d1;
    reg [3:0] n1d;
    reg       c01, c11, de1;
    integer  i;
    reg [8:0] qm;
    reg [3:0] n1q, n0q;
    reg [4:0] cnt;
    reg [9:0] qo;

    always @(posedge clk) begin
        if (rst) begin
            d1 <= 0; n1d <= 0; c01 <= 0; c11 <= 0; de1 <= 0;
            qo <= 0; cnt <= 0; dout <= 0;
        end else begin
            d1  <= din;
            n1d <= din[0] + din[1] + din[2] + din[3]
                 + din[4] + din[5] + din[6] + din[7];
            c01 <= c0; c11 <= c1; de1 <= de;

            // 转移数最小化(与 dvi_encoder 同一拍的数据: d1/n1d)
            qm[0] = d1[0];
            if ((n1d > 4) || ((n1d == 4) && (d1[0] == 0))) begin
                for (i = 1; i <= 7; i = i + 1)
                    qm[i] = qm[i-1] ~^ d1[i];
                qm[8] = 0;
            end else begin
                for (i = 1; i <= 7; i = i + 1)
                    qm[i] = qm[i-1] ^ d1[i];
                qm[8] = 1;
            end
            n1q = qm[0]+qm[1]+qm[2]+qm[3]+qm[4]+qm[5]+qm[6]+qm[7];
            n0q = 8 - n1q;

            if (de1) begin
                if ((cnt == 0) || (n1q == n0q)) begin
                    qo[9] <= ~qm[8]; qo[8] <= qm[8];
                    qo[7:0] <= qm[8] ? qm[7:0] : ~qm[7:0];
                    cnt <= qm[8] ? cnt + n1q - n0q : cnt + n0q - n1q;
                end else begin
                    if ((~cnt[4] && n1q > n0q) || (cnt[4] && n1q < n0q)) begin
                        qo[9] <= 1; qo[8] <= qm[8];
                        qo[7:0] <= ~qm[7:0];
                        cnt <= cnt + {qm[8], 1'b0} + n0q - n1q;
                    end else begin
                        qo[9] <= 0; qo[8] <= qm[8];
                        qo[7:0] <= qm[7:0];
                        cnt <= cnt - {~qm[8], 1'b0} + n1q - n0q;
                    end
                end
            end else begin
                case ({c11, c01})
                    2'b00   : qo <= 10'b1101010100;
                    2'b01   : qo <= 10'b0010101011;
                    2'b10   : qo <= 10'b0101010100;
                    default : qo <= 10'b1010101011;
                endcase
                cnt <= 0;
            end
            dout <= qo;
        end
    end

endmodule
