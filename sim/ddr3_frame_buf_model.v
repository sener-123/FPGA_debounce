//=====================================================================
// 文件    : ddr3_frame_buf_model.v (仿真模型)
// 功能    : ddr3_ctrl_top 的行为级仿真替代模型
// 描述    : 端口与 ddr3_ctrl_top 完全一致, 内部用大容量寄存器阵列
//           模拟 DDR3 乒乓帧缓存:
//             - wd_clk 域: wd_en 有效时写入, wr_load 边沿复位写指针
//               (帧起始), 写满一帧(addr_wd_max*2 像素)后回绕;
//             - rd_clk 域: rd_en 弹出, 数据 1 拍后有效(与真实 rfifo
//               一致), rd_load 边沿复位读指针到帧首;
//             - ddr3_init_done 上电约 20us 后拉高。
// 使用    : 仿真编译本文件并定义 SIM_DDR3。
// 注意    : 模型不模拟 DDR3 的读写延迟/仲裁与页面切换细节, 仅用于
//           验证数据通路与控制逻辑; 真实时序以 DDR3 IP 为准。
//=====================================================================
`timescale 1ns/1ps
module ddr3_frame_buf_model #(
    parameter MEM_AW = 21          // 存储深度 2^21 x 16bit = 4MB
)(
    input                   clk,            // 50MHz 参考时钟(未用)
    input                   wd_clk,
    input                   rd_clk,
    input                   rst_n,
    // 外部数据接口
    input                   wd_en,
    input       [15:0]      wd_data,
    input                   rd_en,
    output reg  [15:0]      rd_data,
    input                   wr_load,
    input                   rd_load,
    input                   ddr3_pingpang_en,
    input                   ddr3_read_valid,
    // 地址长度控制接口
    input       [26:0]      addr_rd_min,
    input       [26:0]      addr_rd_max,
    input       [9:0]       rd_burst_len,
    input       [26:0]      addr_wd_min,
    input       [26:0]      addr_wd_max,
    input       [9:0]       wd_burst_len,
    output reg              ddr3_init_done,
    // DDR3 IO(仿真中悬空)
    output wire             mem_rst_n,
    output wire             mem_ck,
    output wire             mem_ck_n,
    output wire             mem_cke,
    output wire             mem_ras_n,
    output wire             mem_cas_n,
    output wire             mem_we_n,
    output wire             mem_odt,
    output wire             mem_cs_n,
    output wire [14:0]      mem_a,
    output wire [2:0]       mem_ba,
    inout  wire [3:0]       mem_dqs,
    inout  wire [3:0]       mem_dqs_n,
    inout  wire [31:0]      mem_dq,
    output wire [3:0]       mem_dm
);

    //-----------------------------------------------------------------
    // 存储阵列与指针(一帧像素数 = addr_wd_max*2, 与正点原子例程一致)
    //-----------------------------------------------------------------
    reg [15:0] mem [0:(1<<MEM_AW)-1];
    reg [MEM_AW-1:0] wptr;
    reg [MEM_AW-1:0] rptr;

    wire [MEM_AW-1:0] frame_pixels = {addr_wd_max, 1'b0};

    // 写侧(wd_clk 域)
    reg wr_load_d1;
    always @(posedge wd_clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr <= {MEM_AW{1'b0}};
            wr_load_d1 <= 1'b0;
        end else begin
            wr_load_d1 <= wr_load;
            if (wr_load & ~wr_load_d1)
                wptr <= {MEM_AW{1'b0}};          // 帧起始复位写指针
            else if (wd_en) begin
                mem[wptr] <= wd_data;
                if (wptr == frame_pixels - 1)
                    wptr <= {MEM_AW{1'b0}};      // 回绕
                else
                    wptr <= wptr + 1'b1;
            end
        end
    end

    // 读侧(rd_clk 域): rd_en 后 1 拍给出数据
    reg rd_load_d1;
    always @(posedge rd_clk or negedge rst_n) begin
        if (!rst_n) begin
            rptr   <= {MEM_AW{1'b0}};
            rd_data <= 16'd0;
            rd_load_d1 <= 1'b0;
        end else begin
            rd_load_d1 <= rd_load;
            if (rd_load & ~rd_load_d1)
                rptr <= {MEM_AW{1'b0}};          // 帧起始复位读指针
            else if (rd_en) begin
                if (rptr == frame_pixels - 1)
                    rptr <= {MEM_AW{1'b0}};
                else
                    rptr <= rptr + 1'b1;
            end
            rd_data <= mem[rptr];
        end
    end

    // 初始化完成: 上电约 20us 后拉高
    reg [9:0] init_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            init_cnt <= 10'd0;
            ddr3_init_done <= 1'b0;
        end else begin
            if (init_cnt == 10'd1000)
                ddr3_init_done <= 1'b1;
            else
                init_cnt <= init_cnt + 10'd1;
        end
    end

    // DDR3 IO 悬空
    assign mem_rst_n = 1'b1;
    assign mem_ck    = 1'b0;
    assign mem_ck_n  = 1'b1;
    assign mem_cke   = 1'b0;
    assign mem_ras_n = 1'b1;
    assign mem_cas_n = 1'b1;
    assign mem_we_n  = 1'b1;
    assign mem_odt   = 1'b0;
    assign mem_cs_n  = 1'b1;
    assign mem_a     = 15'd0;
    assign mem_ba    = 3'd0;
    assign mem_dm    = 4'd0;
    assign mem_dqs   = 4'bz;
    assign mem_dqs_n = 4'bz;
    assign mem_dq    = 32'bz;

endmodule
