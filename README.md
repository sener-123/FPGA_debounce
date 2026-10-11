# FPGA 相机视频防抖（FPGA Video Stabilization）

基于 FPGA 实现的实时数字视频防抖（Digital Video Stabilization）工程，参考 [lengkujiaai/video_stabilization](https://github.com/lengkujiaai/video_stabilization) 的软件算法流程，用纯 Verilog 硬件实现。目标平台为紫光同创 Logos-2 系列 **PGL22G6CMBG324** 开发板（OV5640 摄像头输入 + HDMI 输出）。

## 算法对应关系

| 参考软件流程（OpenCV） | 本工程硬件实现 | 资源代价 |
|------------------------|----------------|----------|
| 特征点检测 + Lucas-Kanade 光流 | `downsample_tile` 16×16 分块亮度下采样 + `motion_estimate` 全网格 SAD 全局平移估计 | 两块图 RAM + 状态机 |
| 运动轨迹 cumsum | `trajectory_smooth` 轨迹累积 | 少量寄存器 |
| 轨迹移动平均平滑 | `trajectory_smooth` 尾随滑窗平滑（因果实时化） | 少量寄存器 |
| `warpAffine` 仿射变换 + `fixBorder` 去黑边 | `shift_compensate` 环形缓冲平移补偿 + 窗口裁剪（与 fixBorder 对应） | 1 块环形 BRAM |

手持抖动主要为平移分量，本工程只做平移补偿，换取极小资源占用：**约 24 个 18Kb 块 RAM、2~3K LUT、0 个 DSP**，帧间延迟约 1 帧。

## 特性

- ✅ 纯 Verilog-2001，无 IP 依赖，PDS 可直接综合
- ✅ **视频接口完全解耦**：算法核心只与统一内部像素流 `px_*` 交互，更换摄像头/显示器只需改接口适配层（见 [接口说明.md](接口说明.md)）
- ✅ 资源占用小，无乘法器/DSP
- ✅ 全参数化（分辨率/搜索范围/补偿幅度/平滑窗口）
- ✅ 4 个 ModelSim 测试台全部通过（移位补偿逐像素比对、SAD 运动估计、轨迹平滑、顶层闭环）

## 目录结构

```
FPGA_debounce/
├── rtl/                  # RTL 源码(Verilog-2001)
│   ├── video_stab_top.v  #   顶层
│   ├── video_in_if.v     #   视频输入接口适配(解耦层, OV5640)
│   ├── video_out_if.v    #   视频输出接口适配(解耦层, 显示时序)
│   ├── stab_core.v       #   防抖算法核心(运动估计+轨迹平滑)
│   ├── downsample_tile.v #   16x16 分块亮度下采样
│   ├── motion_estimate.v #   全网格 SAD 全局平移估计
│   ├── trajectory_smooth.v # 轨迹累积+滑动平均平滑
│   └── shift_compensate.v # 环形缓冲平移补偿
├── sim/                  # ModelSim 测试台(4 个, 全部 PASS)
├── prj/                  # PDS 工程目录(约束文件放这里)
├── 接口说明.md           # 模块端口/统一接口/引脚映射
├── 使用说明.md           # 搭建/仿真/上板/参数/优化指南
└── README.md
```

## 快速开始

1. 阅读 [使用说明.md](使用说明.md) 第 3 节搭建 PDS 工程；
2. 引脚约束见 [接口说明.md](接口说明.md) 第 4 节（PGL22G 开发板手册的摄像头/HDMI 引脚表）；
3. 默认配置 1280×720，`MAX_SHIFT=8`，输出有效窗口 1262×702；
4. 仿真：`vlog rtl/*.v sim/tb_video_stab_top.v && vsim work.tb_video_stab_top -do "run -all"`。

## 文档

- **[使用说明.md](使用说明.md)** — 硬件平台、工程搭建、参数配置、时序预算、仿真、上板调试、资源优化、已知限制
- **[接口说明.md](接口说明.md)** — 统一内部像素流约定、各模块端口、PGL22G 引脚映射、时钟与复位

## 参考与致谢

- 算法原型：<https://github.com/lengkujiaai/video_stabilization>
- 硬件平台：《PGL22G 开发板用户手册 Rev1.1》（芯驿电子 ALINX，紫光同创 Logos-2）
- 本文仅供学习与参考使用。
