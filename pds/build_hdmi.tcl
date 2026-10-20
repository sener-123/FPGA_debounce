# PDS 2022.2-SP4.2: Davinci Pro Logos2 PG2L200H-6IFBB484, no DDR3.
set root [file normalize [file join [file dirname [info script]] ..]]
set_arch -family Logos2 -device PG2L200H -speedgrade -6 -package FBB484
foreach name {davinci_hdmi_top video_stab_hdmi cmos2px stab_core downsample_tile motion_estimate trajectory_smooth shift_compensate pixel_stream_cdc hdmi_timing_direct dvi_encoder dvi_tx_top} {
    add_design "$root/rtl/$name.v"
}
foreach name {cmos_capture_data i2c_dri i2c_ov5640_rgb565_cfg ov5640_dri} {
    add_design "$root/rtl/ov5640_dri/$name.v"
}
add_design "$root/pds/ip/PLL/PLL.v"
add_constraint "$root/constraints/davinci_hdmi.fdc"
add_constraint "$root/constraints/davinci_clocks.fdc"
compile -top_module davinci_hdmi_top
synthesize -ads -selected_syn_tool_opt 2
dev_map
pnr
report_timing
gen_bit_stream
