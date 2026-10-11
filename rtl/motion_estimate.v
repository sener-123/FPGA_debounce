//=====================================================================
// 文件    : motion_estimate.v
// 功能    : 全局平移运动估计(参考算法"运动估计"环节的硬件实现)
// 描述    : 在相邻两帧的下采样块图上做全网格 SAD 块匹配, 搜索范围
//           ±SEARCH_R 个块(即 ±SEARCH_R*TILE 像素), 估计帧间全局
//           平移矢量 (dx, dy)。特征点匹配+光流在硬件上代价过高,
//           本模块用"整帧下采样块 + SAD 全搜索"达到同样目的,
//           资源占用极小: 仅加法器/比较器与一个状态机。
// 时序    : 每次启动耗时 CAND*(TILES+4) 拍(720p: 49*3604 ≈ 17.7 万拍),
//           必须在帧消隐期内完成(720p60 消隐约 31.6 万拍, 满足)。
// 注意    : 1. 由 downsample_tile 的 frame_done 脉冲启动;
//           2. src = 上一帧块图, dst = 当前帧块图(由 src_sel 选择);
//           3. 若分辨率更高导致消隐期不足, 可减小 SEARCH_R 或
//              把候选拆分到多帧计算(见 使用说明.md)。
//=====================================================================
module motion_estimate #(
    parameter H_ACTIVE  = 1280,   // 行有效像素数
    parameter V_ACTIVE  = 720,    // 场有效行数
    parameter TILE      = 16,     // 下采样块尺寸(与 downsample_tile 一致)
    parameter SEARCH_R  = 3,      // 搜索半径(块), 像素范围 ±SEARCH_R*TILE
    parameter RAM_DEPTH = 4096    // 单块图 RAM 深度(>= TILES_X*TILES_Y, 地址宽度参考)
)(
    input  wire        px_clk,
    input  wire        rst_n,      // 异步复位, 低有效
    input  wire        start,      // 启动脉冲(frame_done)
    input  wire        src_sel,    // 乒乓选择(1: 块图1为源; 与写侧帧奇偶对应)

    // ---- 两块图 RAM 读口(接 stab_core 中的 RAM0/RAM1) ----
    output reg  [13:0] ram0_raddr,
    output reg  [13:0] ram1_raddr,
    input  wire [15:0] ram0_rdata,
    input  wire [15:0] ram1_rdata,

    // ---- 运动矢量输出(每帧 1 次) ----
    output reg         motion_valid,
    output reg  [5:0]  mv_x,       // 有符号, 帧间内容平移(块单位)
    output reg  [5:0]  mv_y        // 有符号
);

    localparam TX    = H_ACTIVE / TILE;    // 每行块数(720p: 80)
    localparam TY    = V_ACTIVE / TILE;    // 每场块行数(720p: 45)
    localparam TILES = TX * TY;            // 每帧总块数(720p: 3600)
    localparam SIDE  = 2*SEARCH_R + 1;     // 单方向候选数(7)
    localparam CAND  = SIDE * SIDE;        // 候选总数(49)

    //-----------------------------------------------------------------
    // 状态机: 0=IDLE, 1=CSTART(锁定参数), 2=CRUN(全图 SAD), 3=CEND(判优)
    //-----------------------------------------------------------------
    reg [1:0]  fsm;
    reg        busy;
    reg        src_sel_r;    // 启动时锁存乒乓选择
    reg [2:0]  dx_cnt;       // 候选 dx 计数(0..SIDE-1)
    reg [2:0]  dy_cnt;       // 候选 dy 计数(0..SIDE-1)
    reg [3:0]  dx_r, dy_r;   // 当前候选有符号偏移(块单位)
    reg [9:0]  offset;       // dy*TX + dx(有符号)
    reg [13:0] t_addr;       // 已发出的块地址计数(0..TILES)
    reg [13:0] run_cnt;      // CRUN 内节拍计数(0..TILES+2, 需覆盖 TILES+2)
    reg [6:0]  tx_cnt;       // 当前块列
    reg [5:0]  ty_cnt;       // 当前块行
    reg        q_valid;      // 上一拍发出的读地址是否有效(流水 1 级)
    reg        q_valid2;     // 流水 2 级(与 RAM 读数据 2 拍延迟对齐)
    reg        valid_q;      // 延迟 1 拍的合法性标志
    reg        valid_q2;     // 延迟 2 拍(与读回数据对齐)
    reg [27:0] sad;          // 当前候选 SAD 累加
    reg [27:0] best_sad;
    reg [3:0]  best_x, best_y;

    //-----------------------------------------------------------------
    // 地址合法性: 目标块(tx+dx, ty+dy)必须在块图内
    // 注意: 与 0 比较必须按有符号处理(负偏移应判为非法)
    //-----------------------------------------------------------------
    wire [8:0] sx = $signed({1'b0, tx_cnt}) + $signed({{4{dx_r[3]}}, dx_r});
    wire [7:0] sy = $signed({1'b0, ty_cnt}) + $signed({{3{dy_r[3]}}, dy_r});
    wire valid_x = (sx[8] == 1'b0) && (sx < TX);   // 0 <= sx < TX
    wire valid_y = (sy[7] == 1'b0) && (sy < TY);   // 0 <= sy < TY
    wire tile_valid = valid_x & valid_y;

    wire [13:0] src_addr = {1'b0, t_addr};   // 源块图内局部地址(0..TILES-1)
    wire [13:0] dst_addr = tile_valid ? (t_addr + $signed({{4{offset[9]}},offset}))
                                      : 14'd0;   // 非法时读 0(数据丢弃)

    // RAM 读地址分配: ram0 读"源"或"目标"由 src_sel 决定(局部寻址)
    wire [13:0] ram0_addr = src_sel_r ? dst_addr : src_addr;
    wire [13:0] ram1_addr = src_sel_r ? src_addr : dst_addr;

    // 数据选择(与地址分配镜像对应)
    wire [15:0] q_src = src_sel_r ? ram1_rdata : ram0_rdata;
    wire [15:0] q_dst = src_sel_r ? ram0_rdata : ram1_rdata;
    wire [15:0] diff_w = (q_src >= q_dst) ? (q_src - q_dst) : (q_dst - q_src);

    always @(posedge px_clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm          <= 2'd0;
            busy         <= 1'b0;
            motion_valid <= 1'b0;
            mv_x         <= 6'd0;
            mv_y         <= 6'd0;
            ram0_raddr   <= 14'd0;
            ram1_raddr   <= 14'd0;
        end else begin
            motion_valid <= 1'b0;   // 默认清零

            case (fsm)
                //----------------------------------------- IDLE
                2'd0: begin
                    ram0_raddr <= 14'd0;
                    ram1_raddr <= 14'd0;
                    if (start && !busy) begin
                        busy      <= 1'b1;
                        src_sel_r <= src_sel;
                        dx_cnt    <= 3'd0;
                        dy_cnt    <= 3'd0;
                        best_sad  <= 28'hFFFFFFF;
                        best_x    <= 4'd0;
                        best_y    <= 4'd0;
                        fsm       <= 2'd1;
                    end
                end
                //----------------------------------------- CSTART
                2'd1: begin
                    dx_r    <= $signed({1'b0,dx_cnt}) - SEARCH_R;
                    dy_r    <= $signed({1'b0,dy_cnt}) - SEARCH_R;
                    offset  <= ($signed({1'b0,dy_cnt}) - SEARCH_R) * TX
                             + ($signed({1'b0,dx_cnt}) - SEARCH_R);
                    t_addr  <= 14'd0;
                    tx_cnt  <= 7'd0;
                    ty_cnt  <= 6'd0;
                    run_cnt <= 10'd0;
                    q_valid  <= 1'b0;
                    q_valid2 <= 1'b0;
                    valid_q  <= 1'b0;
                    valid_q2 <= 1'b0;
                    sad     <= 28'd0;
                    fsm     <= 2'd2;
                end
                //----------------------------------------- CRUN
                2'd2: begin
                    // 发出块 t_addr 的读地址
                    if (t_addr < TILES) begin
                        ram0_raddr <= ram0_addr;
                        ram1_raddr <= ram1_addr;
                        // 块行列计数推进
                        if (tx_cnt == TX-1) begin
                            tx_cnt <= 7'd0;
                            ty_cnt <= (ty_cnt == TY-1) ? 6'd0 : ty_cnt + 6'd1;
                        end else begin
                            tx_cnt <= tx_cnt + 7'd1;
                        end
                        t_addr <= t_addr + 14'd1;
                    end else begin
                        ram0_raddr <= 14'd0;
                        ram1_raddr <= 14'd0;
                    end
                    q_valid <= (t_addr < TILES);
                    q_valid2 <= q_valid;
                    valid_q <= tile_valid;
                    valid_q2 <= valid_q;
                    // 累加两拍前读回的数据(与 RAM 读延迟对齐)
                    if (q_valid2 && valid_q2)
                        sad <= sad + {12'd0, diff_w};
                    // 结束判定
                    if (run_cnt == TILES+2) begin
                        fsm     <= 2'd3;
                        run_cnt <= 10'd0;
                    end else begin
                        run_cnt <= run_cnt + 10'd1;
                    end
                end
                //----------------------------------------- CEND
                2'd3: begin
                    if (sad < best_sad) begin
                        best_sad <= sad;
                        best_x   <= dx_r;
                        best_y   <= dy_r;
                    end
                    // 候选推进(先列后行)
                    if (dx_cnt == SIDE-1) begin
                        dx_cnt <= 3'd0;
                        if (dy_cnt == SIDE-1) begin
                            // 全部候选完成 -> 输出结果
                            dy_cnt       <= 3'd0;
                            busy         <= 1'b0;
                            motion_valid <= 1'b1;
                            mv_x         <= {{2{best_x[3]}}, best_x};
                            mv_y         <= {{2{best_y[3]}}, best_y};
                            fsm          <= 2'd0;
                        end else begin
                            dy_cnt <= dy_cnt + 3'd1;
                            fsm    <= 2'd1;
                        end
                    end else begin
                        dx_cnt <= dx_cnt + 3'd1;
                        fsm    <= 2'd1;
                    end
                end
                default: fsm <= 2'd0;
            endcase
        end
    end

endmodule
