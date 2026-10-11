//=====================================================================
// 文件    : trajectory_smooth.v
// 功能    : 轨迹累积与平滑(参考算法"轨迹计算 + 平滑"环节的硬件实现)
// 描述    : 1. 运动矢量换算为像素并累积得到相机运动轨迹;
//           2. 用滑动平均(SMOOTH_WIN 窗口, 尾随因果实现)平滑轨迹;
//           3. 补偿量 comp = 轨迹 - 平滑轨迹, 即待消除的高频抖动
//              (低频的"有意识"运镜被保留), 幅度限幅在 ±MAX_SHIFT;
//           4. 输出 comp_valid + comp_x/comp_y, 供 shift_compensate
//              在下一帧起始时套用。
//           参考实现为离线"中心滑动平均"(前后各看 RADIUS 帧), 本模块
//           为实时流式硬件, 采用因果尾随滑动平均, 延迟仅 1 帧。
// 资源    : 少量寄存器与加减法器, 不使用块 RAM
//=====================================================================
module trajectory_smooth #(
    parameter MAX_SHIFT  = 8,     // 补偿幅度限幅(±像素)
    parameter SMOOTH_WIN = 4,     // 平滑窗口(必须为 2 的幂)
    parameter TRAJ_MAX   = 1024,  // 轨迹累积饱和值(±像素)
    parameter TILE       = 16     // 运动矢量单位换算(块->像素)
)(
    input  wire       px_clk,
    input  wire       rst_n,       // 异步复位, 低有效
    input  wire       motion_valid,// 运动矢量有效脉冲(每帧 1 次)
    input  wire [5:0] mv_x,        // 有符号帧间运动(块单位)
    input  wire [5:0] mv_y,

    // ---- 补偿量输出(每帧 1 次, 像素单位) ----
    output reg        comp_valid,
    output reg  [5:0] comp_x,      // 有符号, 已限幅 ±MAX_SHIFT
    output reg  [5:0] comp_y
);

    // SMOOTH_WIN 必须为 2 的幂, 用移位代替除法
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
    localparam LOG2_WIN = clog2(SMOOTH_WIN);

    //-----------------------------------------------------------------
    // 轨迹与窗口
    //-----------------------------------------------------------------
    reg signed [11:0] traj_x, traj_y;              // 累积轨迹(±1024 饱和)
    reg signed [11:0] win_x [0:SMOOTH_WIN-1];      // 轨迹窗口(尾随)
    reg signed [11:0] win_y [0:SMOOTH_WIN-1];
    reg signed [13:0] sum_x, sum_y;                // 窗口和(±4096)
    integer i;

    // 运动矢量换算到像素: mv * TILE(TILE=16 即左移 4 位)
    wire signed [9:0] mx_pix = {{4{mv_x[5]}}, mv_x} <<< 4;
    wire signed [9:0] my_pix = {{4{mv_y[5]}}, mv_y} <<< 4;

    // 新轨迹(饱和)
    reg signed [12:0] traj_tmp_x, traj_tmp_y;
    reg signed [11:0] traj_new_x, traj_new_y;
    // 新窗口和
    reg signed [13:0] sum_new_x, sum_new_y;
    // 平滑轨迹与补偿
    reg signed [12:0] comp_tmp_x, comp_tmp_y;

    always @* begin
        // 1. 轨迹累积(饱和)
        traj_tmp_x = traj_x + mx_pix;
        traj_tmp_y = traj_y + my_pix;
        traj_new_x = (traj_tmp_x > TRAJ_MAX) ? TRAJ_MAX :
                     (traj_tmp_x < -TRAJ_MAX) ? -TRAJ_MAX : traj_tmp_x[11:0];
        traj_new_y = (traj_tmp_y > TRAJ_MAX) ? TRAJ_MAX :
                     (traj_tmp_y < -TRAJ_MAX) ? -TRAJ_MAX : traj_tmp_y[11:0];
        // 2. 窗口滑动和: 和 - 最旧 + 最新
        sum_new_x = sum_x - win_x[SMOOTH_WIN-1] + traj_new_x;
        sum_new_y = sum_y - win_y[SMOOTH_WIN-1] + traj_new_y;
        // 3. 补偿 = 轨迹 - 平滑轨迹(高频抖动), 限幅 ±MAX_SHIFT
        comp_tmp_x = traj_new_x - (sum_new_x >>> LOG2_WIN);
        comp_tmp_y = traj_new_y - (sum_new_y >>> LOG2_WIN);
    end

    always @(posedge px_clk or negedge rst_n) begin
        if (!rst_n) begin
            traj_x     <= 12'sd0;
            traj_y     <= 12'sd0;
            sum_x      <= 14'sd0;
            sum_y      <= 14'sd0;
            for (i = 0; i < SMOOTH_WIN; i = i + 1) begin
                win_x[i] <= 12'sd0;
                win_y[i] <= 12'sd0;
            end
            comp_valid <= 1'b0;
            comp_x     <= 6'd0;
            comp_y     <= 6'd0;
        end else begin
            comp_valid <= motion_valid;
            if (motion_valid) begin
                // 轨迹与窗口更新
                traj_x <= traj_new_x;
                traj_y <= traj_new_y;
                sum_x  <= sum_new_x;
                sum_y  <= sum_new_y;
                for (i = SMOOTH_WIN-1; i > 0; i = i - 1) begin
                    win_x[i] <= win_x[i-1];
                    win_y[i] <= win_y[i-1];
                end
                win_x[0] <= traj_new_x;
                win_y[0] <= traj_new_y;
                // 补偿输出(限幅)
                comp_x <= (comp_tmp_x > MAX_SHIFT)  ? MAX_SHIFT :
                          (comp_tmp_x < -MAX_SHIFT) ? -MAX_SHIFT : comp_tmp_x[5:0];
                comp_y <= (comp_tmp_y > MAX_SHIFT)  ? MAX_SHIFT :
                          (comp_tmp_y < -MAX_SHIFT) ? -MAX_SHIFT : comp_tmp_y[5:0];
            end
        end
    end

endmodule
