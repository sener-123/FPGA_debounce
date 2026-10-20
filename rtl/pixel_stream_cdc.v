// RGB565 pixel + EOL/EOF asynchronous FIFO. DEPTH must be a power of two
// and >= 4. Camera writes at most once per two wr_clk cycles; rd_clk is
// produced by the camera PLL at wr_clk/2. Both sides reset on PLL unlock.
module pixel_stream_cdc #(
    parameter AW = 6
)(
    input wire wr_clk, wr_rst_n,
    input wire wr_valid,
    input wire [15:0] wr_data,
    input wire wr_eol, wr_eof,
    input wire rd_clk, rd_rst_n,
    output reg rd_valid,
    output reg [15:0] rd_data,
    output reg rd_eol, rd_eof
);
    reg [17:0] mem [0:(1<<AW)-1];
    reg [AW:0] wp, wg, rp, rg;
    (* ASYNC_REG = "TRUE" *) reg [AW:0] rg_s0, rg_s1, wg_s0, wg_s1;
    wire empty = (rg == wg_s1);
    wire full = (wg == {~rg_s1[AW:AW-1], rg_s1[AW-2:0]});
    wire [AW:0] wn = wp + 1'b1;
    wire [AW:0] rn = rp + 1'b1;

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wp <= 0; wg <= 0; rg_s0 <= 0; rg_s1 <= 0;
        end else begin
            rg_s0 <= rg;
            rg_s1 <= rg_s0;
            if (wr_valid && !full) begin
                wp <= wn;
                wg <= (wn >> 1) ^ wn;
            end
        end
    end
    always @(posedge wr_clk)
        if (wr_valid && !full && wr_rst_n)
            mem[wp[AW-1:0]] <= {wr_eof, wr_eol, wr_data};

    // Registered RAM output is valid exactly when the matching pointer pops.
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rp <= 0; rg <= 0; wg_s0 <= 0; wg_s1 <= 0;
            rd_valid <= 0;
            rd_data <= 0; rd_eol <= 0; rd_eof <= 0;
        end else begin
            wg_s0 <= wg;
            wg_s1 <= wg_s0;
            rd_valid <= !empty;
            rd_eol <= 0;
            rd_eof <= 0;
            if (!empty) begin
                {rd_eof, rd_eol, rd_data} <= mem[rp[AW-1:0]];
                rp <= rn;
                rg <= (rn >> 1) ^ rn;
            end
        end
    end
endmodule
