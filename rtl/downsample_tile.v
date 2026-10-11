//=====================================================================
// 文件    : downsample_tile.v
// 功能    : 分块灰度下采样(运动估计的预处理)
// 描述    : 把输入帧按 TILE x TILE(默认 16x16)分块, 逐块累加亮度值,
//           每个块得到 1 个 16bit 块和(块平均灰度的定点表示),
//           按行写入块存储 RAM, 供 motion_estimate 做全局平移估计。
//           仅使用亮度分量参与运动估计, 大幅降低存储与计算开销。
// 资源    : 每行 TILES_X 个 16bit 累加器 + 亮度计算逻辑, 不使用块 RAM
// 注意    : 1. 每行块和在一次"行消隐"内写完, 要求水平消隐周期数
//              >= TILES_X(720p: 消隐 370 > 80, 满足);
//           2. 每帧结束输出 frame_done 脉冲作为 motion_estimate 的启动;
//           3. 帧的写基地址按乒乓切换(由外部 frame_parity 选择)。
//=====================================================================
module downsample_tile #(
    parameter H_ACTIVE = 1280,   // 行有效像素数
    parameter V_ACTIVE = 720,    // 场有效行数
    parameter TILE     = 16,     // 下采样块尺寸(固定 16x16)
    parameter RAM_DEPTH= 8192    // 块存储 RAM 深度(>= 2*TILES_X*TILES_Y)
)(
    input  wire        px_clk,
    input  wire        rst_n,        // 异步复位, 低有效
    input  wire        px_de,        // 像素有效
    input  wire [15:0] px_data,      // RGB565
    input  wire        px_sof,       // 帧起始
    input  wire        px_eol,       // 行结束(与末像素同拍)
    input  wire        frame_parity, // 帧乒乓选择: 写 RAM 的基地址选择

    // ---- 块存储 RAM 写口(接 stab_core 中的 RAM) ----
    output reg         ram_wen,
    output reg  [12:0] ram_waddr,
    output reg  [15:0] ram_wdata,
    output reg         frame_done     // 一帧全部块写完脉冲
);

    localparam TILES_X = H_ACTIVE / TILE;   // 每行块数(720p: 80)
    localparam TILES_Y = V_ACTIVE / TILE;   // 每场块行数(720p: 45)
    localparam BASE    = RAM_DEPTH / 2;     // 乒乓各半(720p: 4096)

    //-----------------------------------------------------------------
    // 亮度计算: Y = (77R + 150G + 29B) >> 8  (R5/G6/B5 来自 RGB565)
    // 最大和 = 31*77 + 63*150 + 31*29 = 12736, 14bit 足够
    //-----------------------------------------------------------------
    wire [4:0] r5 = px_data[15:11];
    wire [5:0] g6 = px_data[10:5];
    wire [4:0] b5 = px_data[4:0];

    wire [13:0] r77  = ({9'd0,r5}<<6) + ({9'd0,r5}<<3) + ({9'd0,r5}<<2) + {9'd0,r5};
    wire [13:0] g150 = ({8'd0,g6}<<7) + ({8'd0,g6}<<4) + ({8'd0,g6}<<2) + ({8'd0,g6}<<1);
    wire [13:0] b29  = ({9'd0,b5}<<4) + ({9'd0,b5}<<3) + ({9'd0,b5}<<2) + {9'd0,b5};
    wire [13:0] y_sum= r77 + g150 + b29;
    wire [7:0]  luma = y_sum[13:6];   // >> 8

    //-----------------------------------------------------------------
    // 块列累加器: 一行内 TILES_X 个累加器, 逐行累加 16 行后写 RAM
    // (acc 的累加/清零/读出集中在同一时序块, 避免多驱动)
    //-----------------------------------------------------------------
    reg [15:0] acc [0:TILES_X-1];
    reg [11:0] col;          // 行内像素列计数(第 k 个像素时 col=k-1)
    reg [3:0]  line_in_tile; // 块内行计数(第 k 行时 = k-1)
    reg [5:0]  tile_row;     // 块行计数(0..TILES_Y-1)
    integer    i;

    wire [11:0] tile_col = col >> 4;   // 像素所属块列(TILE=16 固定)

    //-----------------------------------------------------------------
    // 写 RAM 状态机: 每写完一块行(16 行)后, 在行消隐期间连续写入 TILES_X 个块和
    //-----------------------------------------------------------------
    reg [1:0] wst;       // 0=IDLE, 1=BURST, 2=DONE
    reg [6:0] wcnt;      // 突发写计数(0..TILES_X-1)

    wire burst_start = px_eol & (line_in_tile == TILE-1);

    always @(posedge px_clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < TILES_X; i = i + 1)
                acc[i] <= 16'd0;
            wst        <= 2'd0;
            wcnt       <= 7'd0;
            tile_row   <= 6'd0;
            ram_wen    <= 1'b0;
            ram_waddr  <= 13'd0;
            ram_wdata  <= 16'd0;
            frame_done <= 1'b0;
        end else begin
            frame_done <= 1'b0;   // 默认清零

            // ---- acc: 累加(仅空闲态, 即像素有效期间) / 突发清零 ----
            if (px_de && wst == 2'd0) begin
                if (tile_col < TILES_X)   // 忽略不满一个整块的边缘像素
                    acc[tile_col] <= acc[tile_col] + {8'd0, luma};
            end else if (wst == 2'd1) begin
                acc[wcnt] <= 16'd0;
            end

            // ---- 写状态机 ----
            case (wst)
                2'd0: begin
                    ram_wen <= 1'b0;
                    if (burst_start) begin
                        wst  <= 2'd1;
                        wcnt <= 7'd0;
                    end
                end
                2'd1: begin
                    ram_wen   <= 1'b1;
                    ram_waddr <= (frame_parity ? BASE : 0) + tile_row*TILES_X + wcnt;
                    ram_wdata <= acc[wcnt];
                    if (wcnt == TILES_X-1)
                        wst <= 2'd2;
                    else
                        wcnt <= wcnt + 7'd1;
                end
                2'd2: begin
                    ram_wen <= 1'b0;
                    if (tile_row == TILES_Y-1)
                        frame_done <= 1'b1;   // 一帧块数据写完
                    tile_row <= (tile_row == TILES_Y-1) ? 6'd0 : tile_row + 6'd1;
                    wst <= 2'd0;
                end
                default: wst <= 2'd0;
            endcase
        end
    end

    //-----------------------------------------------------------------
    // 行列计数(第 k 个像素: col=k-1; 第 k 行: line_in_tile=k-1)
    //-----------------------------------------------------------------
    always @(posedge px_clk or negedge rst_n) begin
        if (!rst_n) begin
            col          <= 12'd0;
            line_in_tile <= 4'd0;
        end else begin
            if (px_sof)
                col <= 12'd0;
            else if (px_de) begin
                if (col == H_ACTIVE-1)
                    col <= 12'd0;
                else
                    col <= col + 12'd1;
            end

            if (px_sof)
                line_in_tile <= 4'd0;
            else if (px_eol) begin
                if (line_in_tile == TILE-1)
                    line_in_tile <= 4'd0;
                else
                    line_in_tile <= line_in_tile + 4'd1;
            end
        end
    end

endmodule
