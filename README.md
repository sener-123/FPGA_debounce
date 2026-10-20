# FPGA 相机视频防抖（FPGA Video Stabilization）

> 无DDR3默认方案已在PDS 2022.2-SP4.2完成Generate Bitstream。可打开工程、位流和重编译入口见 [无DDR编译与下载说明.md](无DDR编译与下载说明.md)。当前唯一使用的工程为 `prj/davinci_hdmi/davinci_hdmi.pds`，顶层 `davinci_hdmi_top`，无需DDR3/FIFO IP。旧工程已备份至 `backup/debounce_legacy/`，原入口已改名为 `debounce.pds.disabled`，DDR相关内容仅作后续扩展参考。ADS时钟约束请使用 `constraints/davinci_clocks.fdc`。


基于 FPGA 实现的实时数字视频防抖（Digital Video Stabilization）工程，参考 [lengkujiaai/video_stabilization](https://github.com/lengkujiaai/video_stabilization) 的软件算法流程，用纯 Verilog 硬件实现。目标平台为**正点原子达芬奇 Pro Logos2** 开发板（PG2L200H-6IFBB484 核心板），外接 OV5640 摄像头、板载 DDR3（32 位、512MB）、RGB LCD 接口。

> **达芬奇上板入口**：使用 `davinci_hdmi_top` / `davinci_sys_top` 及 `constraints/` 对应约束。器件、摄像头版本及 IP 配置见 [达芬奇板级适配.md](达芬奇板级适配.md)。

> **两个内部功能顶层**：
> - **`video_stab_hdmi.v`（推荐，无 DDR3）**：HDMI 直出 720p30，与摄像头锁相，资源最少、时钟域最简单，**防抖算法本身不依赖 DDR3**；
> - **`video_stab_sys.v`（DDR3 版）**：DDR3 帧缓存，HDMI 720p60 + RGB LCD 按键切换（LCD 需要帧缓存，故保留 DDR3）。

## 系统架构（无 DDR3 版）

```
OV5640(720p30, pclk≈84MHz) → ov5640_dri(SCCB配置+采集) → cmos2px → px_* 统一像素流
        px_* ──┬──▶ stab_core(运动估计+轨迹平滑) ── comp_x/comp_y
               └──▶ shift_compensate(环形BRAM平移补偿) ── px_*_o(每像素2拍)
                        ──pixel_stream_cdc──▶ PLL 像素时钟域(1像素/拍)
                    hdmi_timing_direct(数据派生同步) + dvi_tx_top(TMDS)
                        └─▶ HDMI 720p30
```

- **HDMI 直出 720p30**：HDMI 刷新率 = 摄像头帧率（约 30Hz），与摄像头锁相，无 DDR3、无整帧缓存；使用 64 深度像素异步 FIFO 跨时钟域；
- **防抖算法全 BRAM**：运动估计 + 轨迹平滑 + 平移补偿（环形缓冲）全部用片上块 RAM，**0 个 DSP**；
- **视频接口解耦**：算法核心只与统一内部像素流 `px_*` 交互（见 [接口说明.md](接口说明.md)）。

DDR3 版（`video_stab_sys.v`）额外提供：HDMI 720p60（30→60 帧率转换）与 RGB LCD 按键切换（LCD 不同分辨率裁剪 + 跨时钟域），DDR3 仅作显示帧缓存。

## 算法对应关系

| 参考软件流程（OpenCV） | 本工程硬件实现 | 资源代价 |
|------------------------|----------------|----------|
| 特征点检测 + Lucas-Kanade 光流 | `downsample_tile` 16×16 分块亮度下采样 + `motion_estimate` 全网格 SAD 全局平移估计 | 两块图 RAM + 状态机 |
| 运动轨迹 cumsum | `trajectory_smooth` 轨迹累积 | 少量寄存器 |
| 轨迹移动平均平滑 | `trajectory_smooth` 尾随滑窗平滑（因果实时化） | 少量寄存器 |
| `warpAffine` 仿射变换 + `fixBorder` 去黑边 | `shift_compensate` 环形缓冲平移补偿 + 窗口裁剪 + 越界填黑 | 1 块环形 BRAM |

手持抖动主要为平移分量，本工程只做平移补偿，换取极小资源占用：**约 45 个 18Kb 块 RAM（其中 DDR3 前的环形缓冲 32 个）、0 个 DSP**，端到端延迟约 2 帧。

## 特性

- ✅ 纯 Verilog-2001，算法核心无 IP 依赖
- ✅ **视频接口完全解耦**：`px_*` 统一像素流（见 [接口说明.md](接口说明.md)）
- ✅ **HDMI / LCD 双显示，按键一键切换**（[切换方法](#hdmi--lcd-切换)）
- ✅ 集成正点原子 `ov5640_dri`（SCCB 初始化 + 采集）、`ddr3_ctrl_top`（DDR3 乒乓帧缓存）、`lcd_rgb_top`（RGB 屏驱动）
- ✅ HDMI TMDS 编码 + 5× 串行化（`dvi_encoder` / `dvi_tx_top`，Pango `GTP_ODDR_E1`/`GTP_OUTBUFDS` 原语）
- ✅ 资源占用小，无乘法器/DSP
- ✅ 全参数化（分辨率/搜索范围/补偿幅度/平滑窗口/裁剪窗口）
- ✅ 自动回归覆盖运动搜索全候选、补偿方向、连续帧读取、跨时钟域、同步时序和 TMDS 引脚位流

## 目录结构

```
FPGA_debounce/
├── rtl/                    # RTL 源码(Verilog-2001)
│   ├── video_stab_hdmi.v   #   无 DDR3 顶层(HDMI 直出 720p30, 推荐)
│   ├── video_stab_sys.v    #   DDR3 版顶层(HDMI 720p60 + LCD 可切换)
│   ├── video_stab_top.v    #   直通演示顶层(无 DDR3, 保留兼容)
│   ├── video_in_if.v       #   原始 DVP 输入适配(直通顶层用)
│   ├── video_out_if.v      #   显示时序适配(直通顶层用)
│   ├── cmos2px.v           #   摄像头采集输出 -> px_* 适配
│   ├── stab_core.v         #   防抖算法核心(运动估计+轨迹平滑)
│   ├── downsample_tile.v   #   16x16 分块亮度下采样
│   ├── motion_estimate.v   #   全网格 SAD 全局平移估计
│   ├── trajectory_smooth.v #   轨迹累积+滑动平均平滑
│   ├── shift_compensate.v  #   环形缓冲平移补偿(可配置裁剪+黑边)
│   ├── disp_reader.v       #   DDR3 读侧主控(帧同步/节流/补黑)
│   ├── async_fifo_16.v     #   显示 FIFO(FWFT 异步 FIFO)
│   ├── hdmi_timing_gen.v   #   720p60 显示时序(DDR3 版)
│   ├── pixel_stream_cdc.v #   64×18bit 像素/EOL/EOF 异步 FIFO
│   ├── hdmi_timing_direct.v#   HDMI 直出时序(无 DDR3 版, 数据派生)
│   ├── dvi_encoder.v       #   TMDS 8b/10b 编码器
│   ├── dvi_tx_top.v        #   TMDS 发送器(3 通道+串行化)
│   ├── key_debounce.v      #   按键消抖
│   ├── ov5640_dri/         #   正点原子摄像头驱动(SCCB 配置+采集)
│   ├── ddr3_ctrl_top/      #   正点原子 DDR3 控制器封装(AXI 主控+读写控制)
│   ├── lcd_rgb_top/        #   正点原子 RGB LCD 驱动(自动识别屏 ID)
│   ├── ov5640_lcd.v        #   正点原子原始例程顶层(参考)
│   └── picture_size.v      #   正点原子分辨率选择(参考, 本工程未用)
├── sim/                    # ModelSim 测试台、模型与自动回归入口
├── 接口说明.md             # 模块端口/统一接口/信号-引脚映射
├── 使用说明.md             # 搭建/仿真/上板/PLL/参数/资源指南
└── README.md
```

## HDMI / LCD 切换

| 方式 | 说明 |
|------|------|
| **按键切换** | 上电默认 **HDMI**；按一下 `key_mode` 按键（约 20ms 消抖）切换到 **LCD**，再按一下切回。每次切换后显示侧自动复位约 50ms 重新同步。 |
| **LED 指示** | `led_mode` 输出当前模式：0 = HDMI，1 = LCD（LED 接线极性见《接口说明.md》）。 |
| 注意 | 切换瞬间显示会短暂黑屏/花屏约 1 帧（DDR3 帧尺寸改变 + 显示重新同步），属正常现象。 |

摄像头始终输出 1280×720@约30fps，不随模式改变；LCD 模式时按屏分辨率（如 800×480）居中裁剪后显示。

## 需要用户准备的 IP 核

| IP | 用途 | 说明 |
|----|------|------|
| **PLL**（名 `PLL`） | 时钟 | **HDMI 直出版**：输入 `cam_pclk`，`clkout0=cam_pclk/2`、`clkout1=cam_pclk×2.5`（典型 84MHz→42MHz/210MHz）。两输出必须同源并满足时序约束。**DDR3 版**：输入 `sys_clk` 50MHz，输出 74.25MHz/371.25MHz。端口 `rst/clkin1/lock/clkout0/clkout1`。 |
| **DDR3 IP**（名 `ddr3_ip`） | 内存 | 仅 DDR3 版（`video_stab_sys.v`）需要，参考正点原子摄像头例程，按200H器件重新生成（32 位，含内部 PLL/PHY）。 |
| FIFO IP ×2 | DDR3 用户接口 | 仅 DDR3 版需要，`wr_fifo`、`rd_fifo`（正点原子例程自带）。 |

> 用户提供的 `rtl/ddr3_ctrl_top/`、`rtl/ov5640_dri/`、`rtl/lcd_rgb_top/` 直接使用，未修改；本工程新增的防抖算法、HDMI 发送逻辑均与其用户接口对接（详见 [接口说明.md](接口说明.md)）。
> **无 DDR3 版（`video_stab_hdmi.v`）只需 PLL 一个 IP**，无需 DDR3/FIFO IP；小型像素 FIFO 为 RTL 实现。

## 快速开始

1. 阅读 [使用说明.md](使用说明.md) 搭建 PDS 工程（器件 `PG2L200H-6IFBB484`，详见板级适配说明）；
2. 例化 PLL IP（无 DDR3 版）；DDR3 版再例化 DDR3/FIFO IP；
3. 引脚约束见 [接口说明.md](接口说明.md) 第 2–8 节（逐位列出方向、FPGA 引脚、电平及用途）；
4. 综合 → 布局布线 → 下载；上电即 HDMI 输出（无 DDR3 版 720p30，DDR3 版 720p60 + 按 KEY 切 LCD）；
5. 自动回归：在项目根目录运行 `powershell -ExecutionPolicy Bypass -File sim/run_regression.ps1`（需 ModelSim 在 PATH；也可通过 `-Vsim` 指定路径）。DDR3 整机测试在 `sim/` 目录执行 `vsim -do tb_video_stab_sys.do`。

## 文档

- **[使用说明.md](使用说明.md)** — 硬件平台、工程搭建、IP 核生成（PLL/DDR3/FIFO）、参数配置、时序预算、仿真、上板调试、资源优化、已知限制
- **[接口说明.md](接口说明.md)** — 统一内部像素流约定、各模块端口、**代码信号名 ↔ FPGA 引脚名对照表**、时钟与复位

## 参考与致谢

- 算法原型：<https://github.com/lengkujiaai/video_stabilization>
- 摄像头/LCD/DDR3 驱动：正点原子（ALIENTEK）OV5640 例程（`rtl/ov5640_dri`、`rtl/lcd_rgb_top`、`rtl/ddr3_ctrl_top`）
- 硬件平台：《达芬奇Pro Logos2之FPGA开发指南 V1.0》（正点原子）
- 本文仅供学习与参考使用。
