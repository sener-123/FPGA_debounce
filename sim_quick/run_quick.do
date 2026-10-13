#====================================================================
# 快速测试台脚本(tb_shift_compensate / tb_shift_crop / tb_dvi_tx)
# 在独立目录中编译, 避免与系统级测试的 work 库冲突
#====================================================================
vlib work
vmap work work

vlog -work work ../rtl/shift_compensate.v
vlog -work work ../rtl/dvi_encoder.v
vlog -work work ../rtl/dvi_tx_top.v
vlog -work work oddrx1_model.v
vlog -work work elvds_obuf_model.v

vlog -work work tb_shift_compensate.v
vlog -work work tb_shift_crop.v
vlog -work work tb_dvi_tx.v

vsim -voptargs="+acc" work.tb_shift_compensate
run -all

vsim -voptargs="+acc" work.tb_shift_crop
run -all

vsim -voptargs="+acc" work.tb_dvi_tx
run -all
