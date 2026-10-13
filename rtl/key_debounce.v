//=====================================================================
// 文件    : key_debounce.v
// 功能    : 按键消抖(20ms), 输出一次按键事件脉冲
// 描述    : 按键稳定按下(电平等于 KEY_POL)超过消抖时间后, 输出
//           1 拍 key_pulse; 松开并再次按下才会再次触发。
// 平台    : 与器件无关
// 时钟    : clk(本工程接 50MHz 系统时钟)
// 资源    : 计数器与寄存器
//=====================================================================
module key_debounce #(
    parameter CNT_MAX = 1_000_000,  // 消抖时间(时钟周期数, 20ms@50MHz)
    parameter KEY_POL = 1'b0        // 按键按下电平(0=低有效)
)(
    input  wire clk,
    input  wire rst_n,     // 异步复位, 低有效
    input  wire key_in,
    output reg  key_pulse  // 按键事件脉冲(1 拍)
);

    reg        key_d0, key_d1;   // 同步打拍
    reg        stable;           // 消抖后的按下状态(1=按下)
    reg        stable_d1;
    reg [19:0] cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_d0    <= 1'b1;
            key_d1    <= 1'b1;
            stable    <= 1'b0;
            stable_d1 <= 1'b0;
            cnt       <= 20'd0;
            key_pulse <= 1'b0;
        end else begin
            key_d0 <= key_in;
            key_d1 <= key_d0;

            // 按下判定
            if (key_d1 == KEY_POL) begin
                if (cnt < CNT_MAX - 1)
                    cnt <= cnt + 20'd1;
            end else begin
                cnt <= 20'd0;
            end

            // 稳定状态: 按下保持满计数后置 1; 松开清零
            if (cnt == CNT_MAX - 1)
                stable <= 1'b1;
            else if (key_d1 != KEY_POL)
                stable <= 1'b0;

            stable_d1 <= stable;
            key_pulse <= stable & ~stable_d1;   // 稳定按下沿
        end
    end

endmodule
