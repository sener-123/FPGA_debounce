#====================================================================
# tb_video_stab_sys 仿真脚本(ModelSim SE)
# 用法: vsim -do tb_video_stab_sys.do
#====================================================================
vlib work
vmap work work

# ---- 防抖算法模块 ----
vlog -work work ../rtl/cmos2px.v
vlog -work work ../rtl/downsample_tile.v
vlog -work work ../rtl/motion_estimate.v
vlog -work work ../rtl/trajectory_smooth.v
vlog -work work ../rtl/stab_core.v
vlog -work work ../rtl/shift_compensate.v

# ---- 系统数据通路(本工程新增) ----
vlog -work work ../rtl/async_fifo_16.v
vlog -work work ../rtl/disp_reader.v
vlog -work work ../rtl/hdmi_timing_gen.v
vlog -work work ../rtl/dvi_encoder.v
vlog -work work ../rtl/dvi_tx_top.v
vlog -work work ../rtl/key_debounce.v
# ---- 系统顶层(定义 SIM_SOURCE/SIM_DDR3 使用仿真替代, SIM_FAST 缩短切换复位) ----
vlog +define+SIM_SOURCE +define+SIM_DDR3 +define+SIM_FAST -work work ../rtl/video_stab_sys.v

# ---- 正点原子 LCD 驱动 ----
vlog -work work ../rtl/lcd_rgb_top/rd_id.v
vlog -work work ../rtl/lcd_rgb_top/clk_div.v
vlog -work work ../rtl/lcd_rgb_top/lcd_driver.v
vlog -work work ../rtl/lcd_rgb_top/lcd_rgb_top.v

# ---- 仿真模型 ----
vlog -work work pll_clk_sim.v
vlog -work work ddr3_frame_buf_model.v
vlog -work work oddrx1_model.v
vlog -work work elvds_obuf_model.v
vlog -work work sim_cam_source.v

# ---- 测试台 ----
vlog -work work tb_video_stab_sys.v

vsim -voptargs="+acc" work.tb_video_stab_sys
run -all
