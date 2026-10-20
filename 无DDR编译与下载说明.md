# 达芬奇 PG2L200H 无DDR3编译结果与下载说明

本次已使用本机 **Pango Design Suite 2022.2-SP4.2-ads（build 132111）** 完成编译。目标为 **正点原子达芬奇 Pro Logos2，PG2L200H-6IFBB484**，顶层 `davinci_hdmi_top`，不使用DDR3和LCD。

## 直接使用的文件

- PDS工程：[davinci_hdmi.pds](prj/davinci_hdmi/davinci_hdmi.pds)。在PDS中打开此工程，旧工程已备份至 `backup/debounce_legacy/`，原入口已改名为 `debounce.pds.disabled`，不再作为编译入口。
- 下载文件：[davinci_hdmi_top.sbit](output/bitstream/davinci_hdmi_top.sbit)。PDS生成的配置位流使用 `.sbit` 扩展名。
- 配套掩码：[davinci_hdmi_top.smsk](output/bitstream/davinci_hdmi_top.smsk)。按下载工具要求使用。
- 原始生成目录：`prj/davinci_hdmi/generate_bitstream/`。
- 结果清单与SHA256：[build_manifest.json](output/bitstream/build_manifest.json)。
- 时序报告：[davinci_hdmi_top.rtr](prj/davinci_hdmi/report_timing/davinci_hdmi_top.rtr)。
- 全流程日志：[build_console.log](prj/davinci_hdmi/build_console.log)。

## 已完成的配置

1. 独立工程固定使用 Logos2 / PG2L200H / -6 / FBB484，25个物理引脚全部按达芬奇板连接；没有使用DDR3 IP。
2. 官方IP Generator已重新生成 `pds/ip/PLL/PLL.v`。输入为摄像头 `cam_pclk`，典型84MHz；输出42MHz和210MHz，同源0°、50%占空比。VCO为840MHz，反馈倍率10、输出分频20和4。模块名 `PLL`，接口 `rst/lock/clkin1/clkout0/clkout1`。
3. 使用 `constraints/davinci_hdmi.fdc` 和 `constraints/davinci_clocks.fdc`。PDS的ADS流程不能直接把本项目SDC作为该类约束源导入，因此新增等效FDC时钟文件；二者不要重复加载。
4. 板上摄像头时钟U18触发Place-0084。无DDR版FDC对 `cam_pclk` 设置 `PAP_CLOCK_DEDICATED_ROUTE=FALSE`，允许IO至时钟网络使用普通布线，后续仍使用全局时钟网络。引脚未改动，也未用时序假路径掩盖该路径。
5. 默认 `CAMERA_SGM=0`，适用于传统OV5640模块，AB18输出低。带SGM时钟选择的模块应改为1后重新编译；此位流不自动识别摄像头模块版本。

## 编译结果

PDS依次完成 Compile、Synthesize、Device Map、Place & Route、Report Timing、Generate Bitstream，日志末尾为 `Executing : gen_bit_stream successfully`。

- 位流大小：9,188,552字节。
- 当前时序报告：`Design Summary : All Constraints Met`。
- 慢角关键建立裕量：84MHz域2.362ns、210MHz域0.846ns、像素到串行域0.693ns。
- 综合资源：2,563个LUT、2,123个寄存器、20个36Kb RAM、1个APM、1个PLL、25个IO。
- 生成的PLL已通过PDS官方 `GTP_GPLL` 模型仿真：输出周期23.808ns和4.7616ns，0错误、0警告。ModelSim使用 `+acc` 保留模型初始化行为；可运行 `do sim/check_pll_ip.do` 复现检查。

这些结果表明当前约束下能够成功完成实现并生成配置文件。板外摄像头输入及输出接口的完整延迟约束尚未签核；U18使用非专用入口，摄像头派生HDMI时序也需显示器实测。**尚未下载到实板验证，不将生成位流等同于实板视频功能通过。**

## 如何重新编译

在项目根目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\pdsuild_hdmi.ps1
```

脚本调用本机PDS官方IP生成器和命令行编译器，重新生成PLL并完成位流流程。PDS安装在其他目录时传入 `-PdsHome`。重新编译的文件位于 `prj/davinci_hdmi/generate_bitstream/`；`output/bitstream/` 是本次交付快照，不会被重编译脚本自动更新。

也可在PDS图形界面打开上述 `.pds`，检查顶层和约束后运行 **Generate Bitstream**。`prj/` 被Git忽略，工程可通过已保存的 `pds/build_hdmi.tcl` 与PowerShell入口重新生成。

## 实板连接

摄像头接CMOS接口，显示器接 **HDMI_B OUT**；此配置不使用HDMI_A、DDR3或LCD。使用PDS配套下载工具连接开发板JTAG，加载上面的 `.sbit`，执行FPGA配置下载。本次仅生成文件，没有操作JTAG或烧写Flash。

## 从旧工程切换

如果PDS标题栏仍显示 `prj/debounce/debounce.pds`，先关闭此工程，不保存旧配置，再通过 File → Open Project 打开 `prj/davinci_hdmi/davinci_hdmi.pds`。旧工程目录被运行中的PDS占用，暂时保留，其入口已改名停用。不要通过“另存为”重新创建旧入口。
