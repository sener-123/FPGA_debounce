//=====================================================================
// 文件    : oddrx1_model.v (仿真模型)
// 功能    : 紫光 Logos-2 ODDRX1(DDR 输出寄存器)的仿真模型
// 描述    : 上升沿输出 D0, 下降沿输出 D1; RST 高有效异步复位。
// 使用    : 仅仿真编译; PDS 综合时使用器件原语。
//=====================================================================
`timescale 1ns/1ps
module ODDRX1 #(
    parameter INIT = 1'b0
)(
    output wire Q,
    input  wire D0,
    input  wire D1,
    input  wire CLK,
    input  wire RST
);

    reg q_r, q_f;
    always @(posedge CLK or posedge RST)
        if (RST) q_r <= INIT;
        else     q_r <= D0;
    always @(negedge CLK or posedge RST)
        if (RST) q_f <= INIT;
        else     q_f <= D1;

    assign Q = CLK ? q_f : q_r;

endmodule
