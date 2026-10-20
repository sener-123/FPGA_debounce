//=====================================================================
// 文件    : disp_reader.v
// 功能    : DDR3 读侧数据搬运与显示同步(读侧主控)
// 描述    : 工作于 clk_hdmi_pix(74.25MHz, PLL)时钟域, 负责:
//            1. 帧同步: 检测显示侧帧起始脉冲(HDMI 场同步/LCD 场同步)
//               -> 输出 rd_load 脉冲复位 DDR3 读地址, 并复位本模块
//                  计数器、复位显示 FIFO 两侧指针(flush);
//            2. 读节流: 以 FIFO 几乎满信号(afull)节流 ddr3_rd_en,
//               防止 FIFO 溢出; 每帧搬运 win_w*win_h 个像素;
//            3. 黑行扩展: 当显示行数 > 已存帧行数(如 1280x800 屏显示
//               1280x720 帧)时, 在帧尾补黑像素行;
//            4. 上电/切换保护: 帧同步后等待 T_START 个时钟再开始取数,
//               覆盖 DDR3 控制器复位读 FIFO 及重新突发的时间, 保证
//               取出的第一个像素就是本帧第 0 号像素。
// 资源    : 少量计数器, 不使用块 RAM
// 注意    : 1. load_hdmi/load_lcd 须为宽脉冲(>= 16 个源时钟周期);
//           2. mode/lcd_id 为准静态信号(仅按键切换时变化);
//           3. 帧内取数必须完成(win_w*win_h 个像素)且早于下一帧
//              起始脉冲: 720p 帧 @74.25MHz 约 12.4ms, 满足要求。
//=====================================================================
module disp_reader #(
    parameter H_ACTIVE = 1280,   // 摄像头帧宽(HDMI 模式存整帧)
    parameter V_ACTIVE = 720,    // 摄像头帧高
    parameter T_FLUSH  = 64,     // flush 保持周期(复位显示 FIFO)
    parameter T_START  = 2048    // 帧同步后取数等待周期(约 27.6us)
)(
    input  wire        rd_clk,        // 74.25MHz(PLL)
    input  wire        rst_n,         // 异步复位, 低有效

    // ---- DDR3 控制器读侧接口 ----
    input  wire [15:0] ddr3_rd_data,  // 读数据(rd_en 后 1 拍有效)
    output reg         ddr3_rd_en,    // 读请求/弹出 rfifo
    output reg         rd_load,       // 帧起始脉冲(1 拍, 复位 DDR3 读地址)

    // ---- 显示侧帧起始(宽脉冲) ----
    input  wire        load_hdmi,     // HDMI 场同步脉冲(rd_clk 域)
    input  wire        load_lcd,      // LCD 场同步脉冲(lcd_clk 域)

    // ---- 显示模式(准静态) ----
    input  wire        mode,          // 0=HDMI, 1=LCD
    input  wire [15:0] lcd_id,        // LCD 屏 ID(16'h7084 等)

    // ---- 显示 FIFO 写口(与 rd_clk 同域) ----
    input  wire        fifo_afull,    // FIFO 几乎满
    output reg         fifo_wen,
    output reg  [15:0] fifo_wdata,
    output reg         fifo_flush     // 高有效: 保持 FIFO 复位(两侧)
);

    //-----------------------------------------------------------------
    // 输入同步(mode/lcd_id 为跨域准静态信号, 打两拍)
    //-----------------------------------------------------------------
    reg         mode_d0, mode_d1;
    reg [15:0]  lcd_id_d0, lcd_id_d1;
    always @(posedge rd_clk) begin
        mode_d0   <= mode;
        mode_d1   <= mode_d0;
        lcd_id_d0 <= lcd_id;
        lcd_id_d1 <= lcd_id_d0;
    end
    wire        mode_s   = mode_d1;
    wire [15:0] lcd_id_s = lcd_id_d1;

    //-----------------------------------------------------------------
    // LCD 分辨率表(与 正点原子 lcd_driver 的屏 ID 一致)
    //-----------------------------------------------------------------
    reg [11:0] lcd_w, lcd_h;
    always @(*) begin
        case (lcd_id_s)
            16'h4342 : begin lcd_w = 12'd480;  lcd_h = 12'd272; end
            16'h7084 : begin lcd_w = 12'd800;  lcd_h = 12'd480; end
            16'h7016 : begin lcd_w = 12'd1024; lcd_h = 12'd600; end
            16'h4384 : begin lcd_w = 12'd800;  lcd_h = 12'd480; end
            16'h1018 : begin lcd_w = 12'd1280; lcd_h = 12'd800; end
            default  : begin lcd_w = 12'd800;  lcd_h = 12'd480; end
        endcase
    end

    // 本帧窗口(HDMI: 整帧; LCD: 屏分辨率, 高截断到帧高)
    wire [11:0] win_w_w = mode_s ? lcd_w                : H_ACTIVE[11:0];
    wire [11:0] win_h_w = mode_s ? (lcd_h > V_ACTIVE[11:0] ? V_ACTIVE[11:0] : lcd_h)
                                 : V_ACTIVE[11:0];
    // 显示行数(用于黑行扩展判断)
    wire [11:0] disp_rows_w = mode_s ? lcd_h : V_ACTIVE[11:0];

    //-----------------------------------------------------------------
    // 帧起始检测: 选择当前模式对应的场同步, 打两拍后取上升沿
    //-----------------------------------------------------------------
    reg load_lcd_d0, load_lcd_d1;
    always @(posedge rd_clk) begin
        load_lcd_d0 <= load_lcd;
        load_lcd_d1 <= load_lcd_d0;
    end
    wire load_sel = mode_s ? load_lcd_d1 : load_hdmi;

    reg load_sel_d1;
    always @(posedge rd_clk) load_sel_d1 <= load_sel;
    wire load_edge = load_sel & ~load_sel_d1;

    //-----------------------------------------------------------------
    // 帧参数锁存(帧起始时生效)
    //-----------------------------------------------------------------
    reg [11:0] win_w, win_h, disp_rows;
    reg [19:0] total;               // 本帧像素总数(win_w*win_h <= 2^20)
    reg [19:0] black_left;          // 帧尾补黑像素数

    //-----------------------------------------------------------------
    // 状态机: IDLE -> FLUSH -> WAIT -> RUN -> (BLACK) -> DONE
    //-----------------------------------------------------------------
    localparam [2:0] S_IDLE  = 3'd0;
    localparam [2:0] S_FLUSH = 3'd1;
    localparam [2:0] S_WAIT  = 3'd2;
    localparam [2:0] S_RUN   = 3'd3;
    localparam [2:0] S_BLACK = 3'd4;
    localparam [2:0] S_DONE  = 3'd5;

    reg [2:0]  state;
    reg [10:0] flush_cnt;
    reg [11:0] wait_cnt;
    reg [19:0] pop_cnt;             // 本帧已取像素数
    reg [19:0] req_cnt;             // Requests issued, including in-flight reads.
    reg        pop_valid;           // rd_en 后 1 拍(读数据有效)

    always @(posedge rd_clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            flush_cnt  <= 11'd0;
            wait_cnt   <= 12'd0;
            pop_cnt    <= 20'd0;
            req_cnt    <= 20'd0;
            pop_valid  <= 1'b0;
            black_left <= 20'd0;
            win_w      <= H_ACTIVE[11:0];
            win_h      <= V_ACTIVE[11:0];
            disp_rows  <= V_ACTIVE[11:0];
            total      <= H_ACTIVE[11:0] * V_ACTIVE[11:0];
            ddr3_rd_en <= 1'b0;
            rd_load    <= 1'b0;
            fifo_wen   <= 1'b0;
            fifo_wdata <= 16'd0;
            fifo_flush <= 1'b0;
        end else begin
            // ---- 输出缺省值 ----
            ddr3_rd_en <= 1'b0;
            rd_load    <= 1'b0;
            fifo_wen   <= 1'b0;
            fifo_wdata <= 16'd0;
            fifo_flush <= 1'b0;

            case (state)
                S_IDLE : begin
                    if (load_edge) begin
                        win_w      <= win_w_w;
                        win_h      <= win_h_w;
                        disp_rows  <= disp_rows_w;
                        total      <= win_w_w * win_h_w;
                        black_left <= (disp_rows_w > win_h_w)
                                      ? (disp_rows_w - win_h_w) * win_w_w : 20'd0;
                        flush_cnt  <= 11'd0;
                        state      <= S_FLUSH;
                    end
                end

                S_FLUSH : begin
                    pop_valid <= 1'b0;
                    req_cnt <= 20'd0;
                    fifo_flush <= 1'b1;          // 复位显示 FIFO 两侧
                    if (flush_cnt == T_FLUSH-1) begin
                        flush_cnt <= 11'd0;
                        wait_cnt  <= 12'd0;
                        state     <= S_WAIT;
                    end else begin
                        flush_cnt <= flush_cnt + 11'd1;
                    end
                end

                S_WAIT : begin
                    pop_valid <= 1'b0;
                    // 等待 DDR3 控制器复位读 FIFO 并重新突发
                    if (wait_cnt == T_START-1) begin
                        pop_cnt <= 20'd0;
                        state   <= S_RUN;
                    end else begin
                        wait_cnt <= wait_cnt + 12'd1;
                    end
                end

                S_RUN : begin
                    if (load_edge) begin
                        pop_valid <= 1'b0;
                        // 帧同步提前到来(异常/切换): 重新同步
                        win_w      <= win_w_w;
                        win_h      <= win_h_w;
                        disp_rows  <= disp_rows_w;
                        total      <= win_w_w * win_h_w;
                        black_left <= (disp_rows_w > win_h_w)
                                      ? (disp_rows_w - win_h_w) * win_w_w : 20'd0;
                        flush_cnt  <= 11'd0;
                        state      <= S_FLUSH;
                    end else begin
                        // 以 FIFO 空间节流读请求
                        ddr3_rd_en <= ~fifo_afull && (req_cnt < total);
                        if (!fifo_afull && req_cnt < total)
                            req_cnt <= req_cnt + 20'd1;
                        pop_valid  <= ddr3_rd_en;
                        if (pop_valid) begin
                            fifo_wen   <= 1'b1;
                            fifo_wdata <= ddr3_rd_data;
                            if (pop_cnt == total-1) begin
                                // 本帧取完: 若显示行数大于帧高则补黑
                                pop_cnt <= 20'd0;
                                pop_valid <= 1'b0;
                                ddr3_rd_en <= 1'b0;
                                if (black_left != 20'd0)
                                    state <= S_BLACK;
                                else
                                    state <= S_DONE;
                            end else begin
                                pop_cnt <= pop_cnt + 20'd1;
                            end
                        end
                    end
                end

                S_BLACK : begin
                    // 帧尾补黑像素(如 1280x800 屏的最下面 80 行)
                    if (black_left != 20'd0) begin
                        if (!fifo_afull) begin
                            fifo_wen   <= 1'b1;
                            fifo_wdata <= 16'h0000;
                            black_left <= black_left - 20'd1;
                        end
                    end else begin
                        state <= S_DONE;
                    end
                end

                S_DONE : begin
                    // 本帧结束, 等待下一帧起始脉冲
                    if (load_edge) begin
                        win_w      <= win_w_w;
                        win_h      <= win_h_w;
                        disp_rows  <= disp_rows_w;
                        total      <= win_w_w * win_h_w;
                        black_left <= (disp_rows_w > win_h_w)
                                      ? (disp_rows_w - win_h_w) * win_w_w : 20'd0;
                        flush_cnt  <= 11'd0;
                        state      <= S_FLUSH;
                    end
                end

                default : state <= S_IDLE;
            endcase

            // 帧起始脉冲输出(1 拍, 供 DDR3 控制器复位读地址)
            if (load_edge)
                rd_load <= 1'b1;
        end
    end

endmodule
