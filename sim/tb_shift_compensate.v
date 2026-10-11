//=====================================================================
// 测试台  : tb_shift_compensate.v
// 目的    : 验证环形 BRAM 平移补偿的读写地址运算
// 方法    : 以绝对像素序号作为图像内容(ramp), 逐帧给定补偿量
//           (comp_x, comp_y), 在帧生成的同时逐像素比对输出
//           out(x,y) 是否等于 in(x-DX, y-DY)。
// 运行    : 在 ModelSim 中编译 rtl/shift_compensate.v 与本文件后
//           vsim tb_shift_compensate -do "run -all"
//=====================================================================
`timescale 1ns/1ps
module tb_shift_compensate;

    localparam W  = 640;
    localparam V  = 480;
    localparam MS = 8;
    localparam RD = 16384;
    localparam SB    = MS + 1;         // 9
    localparam W_OUT = W - 2*SB;       // 622
    localparam V_OUT = V - 2*SB;       // 462

    reg         clk = 0;
    reg         rst_n = 0;
    always #5 clk = ~clk;              // 100MHz 仿真时钟

    reg         px_de;
    reg  [15:0] px_data;
    reg         px_sof;
    reg         comp_valid;
    reg  [5:0]  comp_x;
    reg  [5:0]  comp_y;

    wire        px_de_o;
    wire [15:0] px_data_o;
    wire        px_sof_o, px_eol_o, px_eof_o;

    shift_compensate #(
        .H_ACTIVE   (W),
        .V_ACTIVE   (V),
        .MAX_SHIFT  (MS),
        .RING_DEPTH (RD)
    ) dut (
        .px_clk     (clk),
        .rst_n      (rst_n),
        .px_de      (px_de),
        .px_data    (px_data),
        .px_sof     (px_sof),
        .comp_valid (comp_valid),
        .comp_x     (comp_x),
        .comp_y     (comp_y),
        .px_de_o    (px_de_o),
        .px_data_o  (px_data_o),
        .px_sof_o   (px_sof_o),
        .px_eol_o   (px_eol_o),
        .px_eof_o   (px_eof_o)
    );

    //-----------------------------------------------------------------
    // 输入帧生成: 每帧 W*V 像素 + 消隐, 内容 = 绝对像素序号(低 16 位)
    //-----------------------------------------------------------------
    integer     in_abs = 0;
    integer     x, y, f;
    reg  [31:0] frame_cnt = 0;
    integer     fbase [0:7];
    integer     comp_sched [0:7];

    task gen_frame(input integer idx);
        begin
            fbase[idx] = in_abs;
            repeat (48) @(posedge clk);          // 场消隐
            // 场消隐期给出 px_sof 脉冲(与 video_in_if 的约定一致)
            px_sof <= 1'b1;
            @(posedge clk);
            px_sof <= 1'b0;
            @(posedge clk);
            for (y = 0; y < V; y = y + 1) begin
                for (x = 0; x < W; x = x + 1) begin
                    px_data <= in_abs[15:0];
                    px_de   <= 1'b1;
                    @(posedge clk);
                    in_abs = in_abs + 1;
                end
                px_de <= 1'b0;                   // 行消隐
                repeat (8) @(posedge clk);
            end
            repeat (48) @(posedge clk);          // 帧消隐
        end
    endtask

    //-----------------------------------------------------------------
    // 并发检查器: 等待输出帧首, 逐像素比对(与帧生成并行)
    //-----------------------------------------------------------------
    integer errs = 0;
    integer out_x, out_y, exp_abs, DX, DY, guard;

    task check_frame(input integer fb, input integer cdx, input integer cdy);
        begin
            DX = SB + cdx;
            DY = SB + cdy;
            // 等待输出帧首(带超时保护)
            guard = 0;
            while (px_sof_o !== 1'b1) begin
                @(posedge clk);
                guard = guard + 1;
                if (guard > 20000) begin
                    $display("ERROR: timeout waiting sof_o");
                    errs = errs + 1;
                    disable check_frame;
                end
            end
            if (px_de_o !== 1'b1) begin
                $display("ERROR: px_de_o not asserted at sof_o");
                errs = errs + 1;
            end
            for (out_y = 0; out_y < V_OUT; out_y = out_y + 1) begin
                for (out_x = 0; out_x < W_OUT; out_x = out_x + 1) begin
                    guard = 0;
                    while (px_de_o !== 1'b1) begin
                        @(posedge clk);
                        guard = guard + 1;
                        if (guard > 2000) begin
                            $display("ERROR: timeout waiting pixel (%0d,%0d)",
                                     out_x, out_y);
                            errs = errs + 1;
                            disable check_frame;
                        end
                    end
                    exp_abs = fb + out_y*W + out_x - DY*W - DX;
                    if (px_data_o !== exp_abs[15:0]) begin
                        $display("ERROR: out(%0d,%0d)=%h expect %h (frame %0d)",
                                 out_x, out_y, px_data_o, exp_abs[15:0], fb);
                        errs = errs + 1;
                    end
                    if (out_y == 0 && out_x == 0 && px_sof_o !== 1'b1) begin
                        $display("ERROR: sof_o not aligned with first pixel");
                        errs = errs + 1;
                    end
                    if (out_x == W_OUT-1 && px_eol_o !== 1'b1) begin
                        $display("ERROR: eol_o not aligned with last pixel");
                        errs = errs + 1;
                    end
                    if (out_y == V_OUT-1 && out_x == W_OUT-1 && px_eof_o !== 1'b1) begin
                        $display("ERROR: eof_o not aligned with last pixel");
                        errs = errs + 1;
                    end
                    if (errs > 50) begin
                        $display("too many errors, aborting check");
                        disable check_frame;
                    end
                    @(posedge clk);
                end
            end
        end
    endtask

    initial begin
        px_de = 0; px_data = 0; px_sof = 0; comp_valid = 0; comp_x = 0; comp_y = 0;
        comp_sched[0] = +2; comp_sched[1] = -3;
        comp_sched[2] = +8; comp_sched[3] = -8;
        comp_sched[4] = 0;  comp_sched[5] = +8;
        comp_sched[6] = -3; comp_sched[7] = +2;
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        for (f = 0; f < 8; f = f + 1) begin
            @(negedge clk);
            comp_valid <= 1'b1;
            comp_x <= comp_sched[f][5:0];
            comp_y <= comp_sched[(1^f)][5:0];   // y 轴用错位序列
            @(negedge clk);
            comp_valid <= 1'b0;
            fork
                gen_frame(f);
                begin
                    if (f >= 1)   // 第一帧环形缓冲未满, 不检查
                        // 注意: Verilog 位选为无符号, 需 $signed 恢复符号
                        check_frame(fbase[f],
                                    $signed(comp_sched[f][5:0]),
                                    $signed(comp_sched[(1^f)][5:0]));
                end
            join
            #100;
        end

        if (errs == 0)
            $display("=== tb_shift_compensate PASS ===");
        else
            $display("=== tb_shift_compensate FAIL: %0d errors ===", errs);
        $stop;
    end

endmodule
