//=====================================================================
// 文件    : elvds_obuf_model.v (仿真模型)
// 功能    : 紫光 Logos-2 ELVDS_OBUF(LVDS 差分输出缓冲)的仿真模型
// 使用    : 仅仿真编译; PDS 综合时使用器件原语。
//=====================================================================
`timescale 1ns/1ps
module ELVDS_OBUF (
    input  wire I,
    output wire O,
    output wire OB
);

    assign O  = I;
    assign OB = ~I;

endmodule
