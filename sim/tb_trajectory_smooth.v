//=====================================================================
// 测试台  : tb_trajectory_smooth.v
// 目的    : 验证轨迹累积 + 尾随滑动平均平滑 + 补偿限幅
// 方法    : 运动矢量序列 mv = {4,4,4,4,0,0,0,0}(块), TILE=16,
//           期望补偿序列 comp = {8,8,8,8,8,8,0,0}。
// 运行    : 在 ModelSim 中编译 rtl/trajectory_smooth.v 与本文件后
//           vsim tb_trajectory_smooth -do "run -all"
//=====================================================================
`timescale 1ns/1ps
module tb_trajectory_smooth;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    reg        motion_valid;
    reg  [5:0] mv_x, mv_y;
    wire       comp_valid;
    wire [5:0] comp_x, comp_y;

    trajectory_smooth #(
        .MAX_SHIFT  (8),
        .SMOOTH_WIN (4),
        .TRAJ_MAX   (1024),
        .TILE       (16)
    ) dut (
        .px_clk(clk), .rst_n(rst_n),
        .motion_valid(motion_valid), .mv_x(mv_x), .mv_y(mv_y),
        .comp_valid(comp_valid), .comp_x(comp_x), .comp_y(comp_y)
    );

    integer mv_seq [0:7];
    integer exp_seq [0:7];
    integer i, errs = 0;

    initial begin
        motion_valid = 0; mv_x = 0; mv_y = 0;
        mv_seq[0] = 4;  mv_seq[1] = 4;  mv_seq[2] = 4;  mv_seq[3] = 4;
        mv_seq[4] = 0;  mv_seq[5] = 0;  mv_seq[6] = 0;  mv_seq[7] = 0;
        exp_seq[0] = 8; exp_seq[1] = 8; exp_seq[2] = 8; exp_seq[3] = 8;
        exp_seq[4] = 8; exp_seq[5] = 8; exp_seq[6] = 0; exp_seq[7] = 0;

        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        for (i = 0; i < 8; i = i + 1) begin
            @(negedge clk);
            motion_valid <= 1'b1;
            mv_x <= mv_seq[i][5:0];
            mv_y <= 0;
            @(negedge clk);
            motion_valid <= 1'b0;
            @(posedge clk);   // comp 在 motion_valid 后 1 拍输出
            if (comp_valid !== 1'b1) begin
                $display("ERROR: comp_valid missing at pulse %0d", i);
                errs = errs + 1;
            end
            if (comp_x !== exp_seq[i][5:0]) begin
                $display("ERROR: pulse %0d comp_x=%0d expect %0d",
                         i, $signed(comp_x), exp_seq[i]);
                errs = errs + 1;
            end
            repeat (4) @(posedge clk);
        end

        if (errs == 0)
            $display("=== tb_trajectory_smooth PASS ===");
        else
            $display("=== tb_trajectory_smooth FAIL: %0d errors ===", errs);
        $stop;
    end

endmodule
