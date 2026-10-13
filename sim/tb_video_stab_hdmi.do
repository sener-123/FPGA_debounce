#====================================================================
# tb_video_stab_hdmi 仿真脚本(无 DDR3, HDMI 直出)
# 用法: vsim -do tb_video_stab_hdmi.do
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

# ---- HDMI 直出数据通路(本工程新增, 无 DDR3) ----
vlog -work work ../rtl/hdmi_timing_direct.v
vlog -work work ../rtl/dvi_encoder.v
vlog -work work ../rtl/dvi_tx_top.v

# ---- 系统顶层(定义 SIM_SOURCE 使用仿真摄像头) ----
vlog +define+SIM_SOURCE -work work ../rtl/video_stab_hdmi.v

# ---- 仿真模型 ----
vlog -work work pll_clk_sim.v
vlog -work work oddrx1_model.v
vlog -work work elvds_obuf_model.v
vlog -work work sim_cam_source.v

# ---- 测试台 ----
vlog -work work tb_video_stab_hdmi.v

vsim -voptargs="+acc" work.tb_video_stab_hdmi
run -all
