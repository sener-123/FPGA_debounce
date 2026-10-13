//=====================================================================
// 文件    : pll_clk_sim.v (仿真模型)
// 功能    : PLL IP 的仿真替代模型(与用户 PDS 工程中的 pll_clk IP
//           端口一致)。实际板级频率: clkout0=74.25MHz, clkout1=
//           371.25MHz; 仿真中 clkout0 直通 clkin1(50MHz), clkout1
//           为 250MHz 自由振荡(仅用于保持串行器运转, 其输出不在
//           系统级测试台中被检查)。
// 使用    : 仿真编译本文件并定义 SIM_PLL(顶层已直接例化 pll_clk,
//           编译时以本文件替代 IP 即可)。
//=====================================================================
`timescale 1ns/1ps
module pll_clk (
    input  wire pll_rst,     // 高有效复位
    input  wire clkin1,      // 50MHz
    output wire clkout0,     // 仿真: 直通 50MHz
    output wire clkout1,     // 仿真: 250MHz
    output wire pll_lock
);

    assign clkout0 = clkin1;

    // 系统级测试台不检查 TMDS 串行位(由 tb_dvi_tx 单独验证),
    // 仿真中将 5x 时钟冻结以大幅加速仿真; 如需观察串行器行为
    // 可恢复为 250MHz 自由振荡: always #2 c5x = ~c5x;
    reg c5x = 1'b0;
    assign clkout1 = c5x;

    reg lock_r = 1'b0;
    initial begin
        lock_r = 1'b0;
        #2000;               // 2us
        lock_r = 1'b1;
    end
    assign pll_lock = lock_r;

endmodule
