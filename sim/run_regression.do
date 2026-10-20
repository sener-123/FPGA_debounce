# Invoked by run_regression.ps1 in an isolated temporary ModelSim work library.
onerror {quit -f -code 1}
vlib work
foreach name {cmos2px downsample_tile motion_estimate trajectory_smooth stab_core shift_compensate hdmi_timing_direct dvi_encoder dvi_tx_top disp_reader async_fifo_16 pixel_stream_cdc} {
    vlog -work work "$SRC/rtl/$name.v"
}
vlog -work work "$SRC/sim/oddrx1_model.v" "$SRC/sim/elvds_obuf_model.v"
set tests {tb_motion_estimate tb_motion_search tb_stab_core tb_trajectory_smooth tb_shift_compensate tb_shift_crop tb_shift_frame_lock tb_compensation_direction tb_disp_reader tb_hdmi_timing_direct tb_pixel_stream_cdc tb_dvi_tx}
foreach name $tests {vlog -work work "$SRC/sim/$name.v"}
onbreak {resume}
foreach name $tests {
    vsim -voptargs="+acc" work.$name
    run -all
}
vlog +define+SIM_SOURCE -work work "$SRC/rtl/video_stab_hdmi.v"
vlog +define+SIM_CAMERA_PLL -work work "$SRC/sim/pll_clk_sim.v"
vlog -work work "$SRC/sim/sim_cam_source.v" "$SRC/sim/tb_video_stab_hdmi.v"
vsim -voptargs="+acc" work.tb_video_stab_hdmi
run -all
quit -f
