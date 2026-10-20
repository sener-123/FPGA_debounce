# Run from ModelSim: do sim/check_pll_ip.do
# Override PDS_HOME before sourcing if PDS is installed elsewhere.
onerror {quit -f -code 1}
set root [file normalize [file join [file dirname [info script]] ..]]
if {![info exists PDS_HOME]} {set PDS_HOME D:/PDS_2022.2-SP4.2-ads}
set lib [file join $root prj pll_sim_work]
file mkdir [file dirname $lib]
vlib $lib
vmap pll_check $lib
vlog -work pll_check $PDS_HOME/arch/vendor/pango/verilog/simulation/GTP_GPLL.v
vlog -work pll_check $root/pds/ip/PLL/PLL.v $root/sim/tb_pll_ip.v
# Required for reliable initialization of real-valued vendor PLL model state.
vsim -voptargs="+acc" pll_check.tb_pll_ip
run -all
