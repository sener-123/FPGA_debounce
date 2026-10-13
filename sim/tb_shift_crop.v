//=====================================================================
// 测试台  : tb_shift_crop.v
// 目的    : 验证 shift_compensate 的可配置裁剪窗口(居中裁剪)与
//           黑色填充行为(video_stab_sys 中 LCD 模式使用该功能)
// 方法    : 640x480 帧, 裁剪窗口 (col0=80, row0=60, 480x360);
//           帧内容为绝对像素序号 ramp; 逐帧给定补偿量,
//           期望 out(ox,oy) = in(80+ox-DX, 60+oy-DY),
//           源坐标越出本帧时为 0x0000(黑)。
// 运行    : ModelSim 编译 rtl/shift_compensate.v + 本文件
//           vsim tb_shift_crop -do "run -all"
//=====================================================================
`timescale 1ns/1ps
module tb_shift_crop;

    localparam W  = 640;
    localparam V  = 480;
    localparam MS = 8;
    localparam RD = 16384;
    localparam SB = MS + 1;          // 9

    localparam C0 = 80;              // 裁剪窗口左上角
    localparam R0 = 60;
    localparam CW = 480;             // 窗口宽
    localparam CH = 360;             // 窗口高

    reg         clk = 0;
    reg         rst_n = 0;
    always #5 clk = ~clk;            // 100MHz

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
        .crop_col0  (C0),
        .crop_row0  (R0),
        .crop_w     (CW),
        .crop_h     (CH),
        .px_de_o    (px_de_o),
        .px_data_o  (px_data_o),
        .px_sof_o   (px_sof_o),
        .px_eol_o   (px_eol_o),
        .px_eof_o   (px_eof_o)
    );

    //-----------------------------------------------------------------
    // 输入帧生成(ramp)
    //-----------------------------------------------------------------
    integer     in_abs = 0;
    integer     x, y, f;
    integer     fbase [0:7];
    integer     comp_sched [0:7];

    task gen_frame(input integer idx);
        begin
            fbase[idx] = in_abs;
            repeat (48) @(posedge clk);
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
                px_de <= 1'b0;
                repeat (8) @(posedge clk);
            end
            repeat (48) @(posedge clk);
        end
    endtask

    //-----------------------------------------------------------------
    // 并发检查器
    //-----------------------------------------------------------------
    integer errs = 0;
    integer out_x, out_y, exp_abs, DX, DY, guard;
    integer src_fx, src_fy, de_cnt;

    task check_frame(input integer fb, input integer cdx, input integer cdy);
        begin
            DX = SB + cdx;
            DY = SB + cdy;
            guard = 0;
            while (px_sof_o !== 1'b1) begin
                @(posedge clk);
                guard = guard + 1;
                // 窗口首像素位于帧内 (row0*W+col0) 之后, 超时需覆盖
                if (guard > 200000) begin
                    $display("ERROR: timeout waiting sof_o");
                    errs = errs + 1;
                    disable check_frame;
                end
            end
            de_cnt = 0;
            for (out_y = 0; out_y < CH; out_y = out_y + 1) begin
                for (out_x = 0; out_x < CW; out_x = out_x + 1) begin
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
                    de_cnt = de_cnt + 1;
                    // 源坐标(帧坐标)
                    src_fx = C0 + out_x - DX;
                    src_fy = R0 + out_y - DY;
                    if (src_fx < 0 || src_fy < 0)
                        exp_abs = 0;
                    else
                        exp_abs = fb + src_fy*W + src_fx;
                    if (px_data_o !== exp_abs[15:0]) begin
                        $display("ERROR: out(%0d,%0d)=%h expect %h (frame %0d)",
                                 out_x, out_y, px_data_o, exp_abs[15:0], fb);
                        errs = errs + 1;
                    end
                    if (out_y == 0 && out_x == 0 && px_sof_o !== 1'b1) begin
                        $display("ERROR: sof_o not aligned with first window pixel");
                        errs = errs + 1;
                    end
                    if (out_x == CW-1 && px_eol_o !== 1'b1) begin
                        $display("ERROR: eol_o not aligned with last window pixel");
                        errs = errs + 1;
                    end
                    if (out_y == CH-1 && out_x == CW-1 && px_eof_o !== 1'b1) begin
                        $display("ERROR: eof_o not aligned with last window pixel");
                        errs = errs + 1;
                    end
                    if (errs > 50) begin
                        $display("too many errors, aborting check");
                        disable check_frame;
                    end
                    @(posedge clk);
                end
            end
            if (de_cnt != CW*CH) begin
                $display("ERROR: frame %0d de count %0d expect %0d",
                         fb, de_cnt, CW*CH);
                errs = errs + 1;
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
            comp_y <= comp_sched[(1^f)][5:0];
            @(negedge clk);
            comp_valid <= 1'b0;
            fork
                gen_frame(f);
                begin
                    if (f >= 1)   // 第一帧环形缓冲未满, 不检查
                        check_frame(fbase[f],
                                    $signed(comp_sched[f][5:0]),
                                    $signed(comp_sched[(1^f)][5:0]));
                end
            join
            #100;
        end

        if (errs == 0)
            $display("=== tb_shift_crop PASS ===");
        else
            $display("=== tb_shift_crop FAIL: %0d errors ===", errs);
        $stop;
    end

endmodule
