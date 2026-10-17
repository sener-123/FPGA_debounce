# FPGA 相机视频防抖（FPGA Video Stabilization）

基于 FPGA 实现的实时数字视频防抖（Digital Video Stabilization）工程，参考 [lengkujiaai/video_stabilization](https://github.com/lengkujiaai/video_stabilization) 的软件算法流程，用纯 Verilog 硬件实现。目标平台为紫光同创 Logos-2 系列 **PGL22G6CMBG324** 开发板（正点原子领航者），板载 OV5640 摄像头、DDR3（32 位）、RGB LCD。

> **两个顶层**：
> - **`video_stab_hdmi.v`（推荐，无 DDR3）**：HDMI 直出 720p30，与摄像头锁相，资源最少、时钟域最简单，**防抖算法本身不依赖 DDR3**；
> - **`video_stab_sys.v`（DDR3 版）**：DDR3 帧缓存，HDMI 720p60 + RGB LCD 按键切换（LCD 需要帧缓存，故保留 DDR3）。

## 系统架构（无 DDR3 版）

```
OV5640(720p30, pclk≈84MHz) → ov5640_dri(SCCB配置+采集) → cmos2px → px_* 统一像素流
        px_* ──┬──▶ stab_core(运动估计+轨迹平滑) ── comp_x/comp_y
               └──▶ shift_compensate(环形BRAM平移补偿) ── px_*_o(每像素2拍)
                        ──(cam_pclk/2 像素时钟)──▶ 1像素/拍
                    hdmi_timing_direct(数据派生同步) + dvi_tx_top(TMDS)
                        └─▶ HDMI 720p30
```

- **HDMI 直出 720p30**：HDMI 刷新率 = 摄像头帧率（约 30Hz），与摄像头锁相，无 DDR3、无帧缓存、无 FIFO；
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
- ✅ HDMI TMDS 编码 + 5× 串行化（`dvi_encoder` / `dvi_tx_top`，Pango `ODDRX1`/`ELVDS_OBUF` 原语）
- ✅ 资源占用小，无乘法器/DSP
- ✅ 全参数化（分辨率/搜索范围/补偿幅度/平滑窗口/裁剪窗口）
- ✅ 6 个 ModelSim 测试台全部通过（含全系统 HDMI/LCD 双模式闭环验证）

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
│   ├── hdmi_timing_direct.v#   HDMI 直出时序(无 DDR3 版, 数据派生)
│   ├── dvi_encoder.v       #   TMDS 8b/10b 编码器
│   ├── dvi_tx_top.v        #   TMDS 发送器(3 通道+串行化)
│   ├── key_debounce.v      #   按键消抖
│   ├── ov5640_dri/         #   正点原子摄像头驱动(SCCB 配置+采集)
│   ├── ddr3_ctrl_top/      #   正点原子 DDR3 控制器封装(AXI 主控+读写控制)
│   ├── lcd_rgb_top/        #   正点原子 RGB LCD 驱动(自动识别屏 ID)
│   ├── ov5640_lcd.v        #   正点原子原始例程顶层(参考)
│   └── picture_size.v      #   正点原子分辨率选择(参考, 本工程未用)
├── sim/                    # ModelSim 测试台与模型(全部 PASS)
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
| **PLL**（名 `pll`） | 时钟 | 输入 50MHz 晶振（R4），输出 **clkout0 = 210MHz**（=5× 像素时钟 5×cam_pclk/2，TMDS 5× 串行时钟）；`clkout1` 无 DDR3 版未使用。端口 `pll_rst/clkin1/pll_lock/clkout0/clkout1`。 |
| **DDR3 IP**（名 `ddr3_ip`） | 内存 | 仅 DDR3 版（`video_stab_sys.v`）需要，与正点原子 `38_ov5640_lcd` 例程完全相同（32 位，含内部 PLL/PHY）。 |
| FIFO IP ×2 | DDR3 用户接口 | 仅 DDR3 版需要，`wr_fifo`、`rd_fifo`（正点原子例程自带）。 |

> 用户提供的 `rtl/ddr3_ctrl_top/`、`rtl/ov5640_dri/`、`rtl/lcd_rgb_top/` 直接使用，未修改；本工程新增的防抖算法、HDMI 发送逻辑均与其用户接口对接（详见 [接口说明.md](接口说明.md)）。
> **无 DDR3 版（`video_stab_hdmi.v`）只需 PLL 一个 IP**，无需 DDR3/FIFO。

## 快速开始

1. 阅读 [使用说明.md](使用说明.md) 搭建 PDS 工程（器件 `PGL22G-6CMBG324`）；
2. 例化 PLL IP（无 DDR3 版）；DDR3 版再例化 DDR3/FIFO IP；
3. 引脚约束见 [接口说明.md](接口说明.md) 第 4 节（含每个信号的 FPGA 引脚名）；
4. 综合 → 布局布线 → 下载；上电即 HDMI 输出（无 DDR3 版 720p30，DDR3 版 720p60 + 按 KEY 切 LCD）；
5. 仿真：无 DDR3 版 `vsim -do sim/tb_video_stab_hdmi.do`，DDR3 版 `vsim -do sim/tb_video_stab_sys.do`。

## 文档

- **[使用说明.md](使用说明.md)** — 硬件平台、工程搭建、IP 核生成（PLL/DDR3/FIFO）、参数配置、时序预算、仿真、上板调试、资源优化、已知限制
- **[接口说明.md](接口说明.md)** — 统一内部像素流约定、各模块端口、**代码信号名 ↔ FPGA 引脚名对照表**、时钟与复位

## 参考与致谢

- 算法原型：<https://github.com/lengkujiaai/video_stabilization>
- 摄像头/LCD/DDR3 驱动：正点原子（ALIENTEK）OV5640 例程（`rtl/ov5640_dri`、`rtl/lcd_rgb_top`、`rtl/ddr3_ctrl_top`）
- 硬件平台：《PGL22G 开发板用户手册 Rev1.1》（芯驿电子 ALINX，紫光同创 Logos-2）
- 本文仅供学习与参考使用。
