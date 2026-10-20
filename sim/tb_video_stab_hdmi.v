// Compact full-top regression: real camera/PLL rates, exact pixel scoreboard,
// complete frames, blanking VS width and board-facing forwarded clock.
`timescale 1ns/1ps
module tb_video_stab_hdmi;
    localparam W=160,H=96,HT=200;
    reg sys_clk=0,cam_pclk=0,sys_rst_n=0;
    always #10 sys_clk=~sys_clk;
    always #5 cam_pclk=~cam_pclk;
    wire tp,tn;wire[2:0] dp,dn;
    video_stab_hdmi #(.H_ACTIVE(W),.V_ACTIVE(H),.H_TOTAL(HT),
        .H_SYNC(8),.V_SYNC(3),.TILE_RAM_DEPTH(128),.RING_DEPTH(4096)) dut(
        .sys_clk(sys_clk),.sys_rst_n(sys_rst_n),.cam_pclk(cam_pclk),
        .cam_vsync(1'b0),.cam_href(1'b0),.cam_data(8'd0),
        .cam_rst_n(),.cam_pwdn(),.cam_scl(),.cam_sda(),
        .dbg_motion_valid(),.dbg_mv_x(),.dbg_mv_y(),
        .dbg_comp_valid(),.dbg_comp_x(),.dbg_comp_y(),
        .tmds_clk_p(tp),.tmds_clk_n(tn),.tmds_data_p(dp),.tmds_data_n(dn));
    defparam dut.u_sim_cam.H=W, dut.u_sim_cam.V=H,
             dut.u_sim_cam.ROW_BLANK=2*(HT-W),dut.u_sim_cam.V_BLANK=50;
    reg [17:0] expected[0:127];
    integer written=0,read_count=0,frame_pixels=0,line_pixels=0,frames=0;
    integer vs_clocks=0,vs_pulses=0,motions=0,comps=0;
    reg vs_d=0;
    reg [17:0] exp_pixel;
    reg [23:0] exp_rgb;
    always @(posedge cam_pclk)begin
        if(dut.px_de_o && dut.cam_ready[1])begin
            if ((^dut.px_data_o) === 1'bx)
                $fatal(1,"unknown pixel entering display CDC");
            expected[written%128]={dut.px_eof_o,dut.px_eol_o,dut.px_data_o};
            written=written+1;
            if(written-read_count>120)$fatal(1,"system FIFO backlog");
        end
        if(dut.dbg_motion_valid)begin
            if ((^{dut.dbg_mv_x,dut.dbg_mv_y}) === 1'bx)
                $fatal(1,"unknown motion estimate");
            motions=motions+1;
        end
        if(dut.dbg_comp_valid)begin
            if ((^{dut.dbg_comp_x,dut.dbg_comp_y}) === 1'bx)
                $fatal(1,"unknown correction");
            comps=comps+1;
        end
    end
    always @(negedge dut.pixel_clk) if(dut.pix_ready[1])begin
        if(dut.video_de)begin
            if(read_count>=written)$fatal(1,"unexpected HDMI pixel");
            exp_pixel=expected[read_count%128];
            exp_rgb={exp_pixel[15:11],3'b0,exp_pixel[10:5],2'b0,exp_pixel[4:0],3'b0};
            if(dut.video_rgb !== exp_rgb)$fatal(1,"pixel %0d got %h expected %h",read_count,dut.video_rgb,exp_rgb);
            read_count=read_count+1;frame_pixels=frame_pixels+1;line_pixels=line_pixels+1;
            if(exp_pixel[16])begin
                if(line_pixels!=W)$fatal(1,"line pixel count %0d",line_pixels);
                line_pixels=0;
            end
            if(exp_pixel[17])begin
                if(frame_pixels!=W*H)$fatal(1,"frame pixel count %0d",frame_pixels);
                frame_pixels=0;frames=frames+1;
            end
        end
        if(dut.video_vs && dut.video_de)$fatal(1,"VS overlaps DE");
        if(dut.video_vs)vs_clocks=vs_clocks+1;
        if(vs_d && !dut.video_vs)begin
            if(vs_clocks!=3*HT)$fatal(1,"system VS width=%0d",vs_clocks);
            vs_pulses=vs_pulses+1;vs_clocks=0;
        end
        vs_d=dut.video_vs;
    end
    // The synchronous ODDR reset and output pipeline need two pixel clocks.
    integer tmds_ready_cycles=0;
    always @(posedge dut.pixel_clk) begin
        if (!dut.pix_ready[1]) tmds_ready_cycles=0;
        else if (tmds_ready_cycles<3) tmds_ready_cycles=tmds_ready_cycles+1;
    end
    always @(negedge dut.clk_hdmi_5x)begin
        #0.001;
        if(tmds_ready_cycles>=3 && (tp !== ~dut.pixel_clk || tn !== dut.pixel_clk))
            $fatal(1,"system TMDS clock ratio/polarity");
    end
    initial begin
        repeat(20)@(negedge sys_clk);sys_rst_n=1;
        wait(frames==14);
        repeat(1000)@(negedge dut.pixel_clk);
        if(written!=read_count || vs_pulses<14 || motions<13 || comps<13)
            $fatal(1,"coverage wr=%0d rd=%0d VS=%0d motion=%0d comp=%0d",written,read_count,vs_pulses,motions,comps);
        $display("=== tb_video_stab_hdmi PASS (%0d frames, %0d pixels) ===",frames,read_count);$stop;
    end
    initial begin #20_000_000;$fatal(1,"HDMI system timeout");end
endmodule
