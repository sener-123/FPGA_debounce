# Run in an isolated work directory:
# vsim -c -do "set SRC {C:/path/FPGA_debounce}; do $SRC/sim/check_davinci_board.do"
# Interface elaboration only; PLL/DDR3 are behavioral models.
onerror {quit -f -code 1}
vlib work
foreach path [glob "$SRC/rtl/*.v"] {
    vlog +define+SIM_SOURCE+SIM_DDR3 -work work $path
}
foreach path [glob "$SRC/rtl/lcd_rgb_top/*.v"] {vlog -work work $path}
foreach name {pll_clk_sim ddr3_frame_buf_model oddrx1_model elvds_obuf_model sim_cam_source} {
    vlog -work work "$SRC/sim/$name.v"
}
foreach top {davinci_hdmi_top davinci_sys_top} {
    vsim work.$top
    run 0
    quit -sim
    vsim -gCAMERA_SGM=1 work.$top
    run 0
    if {[examine -radix binary /$top/cam_pwdn] ne "1"} {error "SGM clock select must be high"}
    quit -sim
}
puts "DAVINCI BOARD ELABORATION PASS"
quit -f
