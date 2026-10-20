// Davinci Pro Logos2 PG2L200H-6IFBB484 board top; physical ports only.
// Pin assignments: constraints/davinci_hdmi.fdc
module davinci_hdmi_top #(
    // 0: traditional OV5640 PWDN low; 1: SGM module selects local oscillator.
    parameter CAMERA_SGM = 1'b0
) (
    input wire         sys_clk,
    input wire         sys_rst_n,
    input wire         cam_pclk,
    input wire         cam_vsync,
    input wire         cam_href,
    input wire [7:0]   cam_data,
    output wire         cam_rst_n,
    output wire         cam_pwdn,
    output wire         cam_scl,
    inout wire         cam_sda,
    output wire         tmds_clk_p,
    output wire         tmds_clk_n,
    output wire [2:0]   tmds_data_p,
    output wire [2:0]   tmds_data_n
);
    wire camera_control;
    assign cam_pwdn = CAMERA_SGM ? 1'b1 : camera_control;
    video_stab_hdmi u_core (
        .sys_clk(sys_clk),
        .sys_rst_n(sys_rst_n),
        .cam_pclk(cam_pclk),
        .cam_vsync(cam_vsync),
        .cam_href(cam_href),
        .cam_data(cam_data),
        .cam_rst_n(cam_rst_n),
        .cam_pwdn(camera_control),
        .cam_scl(cam_scl),
        .cam_sda(cam_sda),
        .tmds_clk_p(tmds_clk_p),
        .tmds_clk_n(tmds_clk_n),
        .tmds_data_p(tmds_data_p),
        .tmds_data_n(tmds_data_n),
        .dbg_motion_valid(), .dbg_mv_x(), .dbg_mv_y(),
        .dbg_comp_valid(), .dbg_comp_x(), .dbg_comp_y()
    );
endmodule
