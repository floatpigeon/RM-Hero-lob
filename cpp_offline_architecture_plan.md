# C++ 离线回放优先的霓光轨迹合成架构方案

## 1. 文档目标

本文档用于说明`稳像优先的霓光轨迹合成`在代码层面的首版实现架构。

目标不是描述算法原理本身，而是回答以下问题：

- 代码按什么层次拆分
- 哪些模块负责哪些职责
- 数据如何在各模块之间流动
- 首版工程如何兼顾验证效率和后续扩展

本文档是对`trajectory_overlay_plan.md`的实现补充，重点面向代码组织和工程落地。

## 2. 首版实现目标

首版实现采用以下默认策略：

- 语言：C++
- 图像库：OpenCV 4.x
- 运行形态：离线命令行工具 + 可复用算法库
- 输入形态：文件回放优先
- 触发方式：命令行传入触发帧号或时间戳
- 缓存方式：单窗口原始帧驻留内存
- 输出方式：每次运行输出一个结果目录

首版不优先解决的问题：

- 实时工业相机接入
- 多线程流水线调度
- 网络接口或服务化
- 高位深输入支持
- GPU 加速

## 3. 架构原则

### 3.1 分层明确

算法核心不直接依赖视频文件或相机来源。

输入适配、窗口组装、算法处理、调试输出应分层，避免后续接入实时相机时重写算法主链。

### 3.2 以窗口为处理单位

单次处理围绕一次触发窗口展开。

- 先收集完整窗口数据
- 再统一执行背景构建、稳像、亮迹提取和合成

这样更符合当前“结果质量优先于实时性”的目标。

### 3.3 内存克制

虽然首版选择“窗口帧驻留内存”，但必须避免重复保留整窗全分辨率中间结果。

原则如下：

- 原始 BGR 帧只保留一份所有权
- 中间灰度图、掩膜和 warp 结果尽量按帧即时生成、即时释放
- 调试模式也不默认保存每一帧全图

### 3.4 先离线验证，再扩展实时接入

首版交付优先服务联调和算法验证，因此优先做：

- 可复现的文件回放
- 可批量比对的结果目录输出
- 足够细的调试指标

## 4. 顶层模块划分

建议工程拆成两层：

1. `lob_hero_core`
2. `lob_hero_cli`

其中：

- `lob_hero_core`负责算法和数据结构
- `lob_hero_cli`负责参数解析、文件回放、结果落盘

### 4.1 Core 层模块

`core`建议拆成以下模块：

- `types`
  - 定义核心数据结构和配置结构
- `replay`
  - 定义回放输入接口
- `windowing`
  - 负责根据触发信息组装单个处理窗口
- `background`
  - 负责构建参考背景
- `registration`
  - 负责逐帧稳像
- `motion`
  - 负责亮迹掩膜提取
- `compositor`
  - 负责轨迹层更新和最终图合成
- `debug`
  - 负责调试信息采集与导出
- `pipeline`
  - 负责串联完整处理流程

### 4.2 CLI 层模块

`cli`建议只负责：

- 读取命令行参数
- 打开输入视频文件
- 解析触发帧号或时间戳
- 调用核心处理入口
- 创建输出目录并落盘

CLI 不应承载算法逻辑。

## 5. 推荐目录结构

建议目录结构如下：

```text
.
├── CMakeLists.txt
├── include/
│   └── lob_hero/
│       ├── types.hpp
│       ├── replay_source.hpp
│       ├── window_capture.hpp
│       ├── reference_builder.hpp
│       ├── registration_engine.hpp
│       ├── motion_extractor.hpp
│       ├── trail_compositor.hpp
│       ├── offline_processor.hpp
│       └── debug_recorder.hpp
├── src/
│   ├── replay/
│   ├── windowing/
│   ├── background/
│   ├── registration/
│   ├── motion/
│   ├── compositor/
│   ├── debug/
│   ├── pipeline/
│   └── cli/
└── tests/
```

如果首版规模较小，也可以先把 `src` 下按模块拆目录，等功能稳定后再细分为更多子文件。

## 6. 核心数据结构

### 6.1 配置结构

建议用一个总配置结构统一承载算法参数：

`CompositeConfig`

至少包含以下字段：

- `crop_rect`
- `pretrigger_frame_count`
- `capture_duration_ms`
- `registration_downsample_ratio`
- `auto_tile_count`
- `ecc_iterations`
- `registration_score_threshold`
- `translation_fallback_threshold`
- `motion_sigma_multiplier`
- `motion_floor_threshold`
- `min_visible_value`
- `min_blob_area`
- `debug_enabled`

说明：

- 首版默认输入为 8-bit BGR 图像
- 如需支持 `1440x1080` 裁切到 `960x960`，优先通过 `crop_rect` 实现

### 6.2 帧数据

`FramePacket`

建议包含：

- `frame_index`
- `timestamp_ms`
- `cv::Mat bgr`

原则：

- `bgr` 由窗口对象统一持有
- 不在下游模块中复制整帧所有权

### 6.3 触发描述

`TriggerSpec`

建议包含：

- `mode`
  - `frame_index`
  - `timestamp_ms`
- `value`

### 6.4 窗口对象

`WindowCapture`

建议包含：

- `pretrigger_frames`
- `posttrigger_frames`
- `source_fps`
- `source_frame_size`

说明：

- `posttrigger_frames`覆盖触发后固定 3 秒窗口
- `pretrigger_frames`仅用于构建参考背景

### 6.5 配准结果

`RegistrationResult`

建议包含：

- `accepted`
- `used_translation_fallback`
- `cv::Mat warp_2x3`
- `score`
- `reject_reason`

### 6.6 帧级调试信息

`FrameDebug`

建议包含：

- `frame_index`
- `timestamp_ms`
- `registration_score`
- `accepted`
- `used_fallback`
- `motion_pixel_count`
- `reject_reason`

### 6.7 最终输出对象

`CompositeResult`

建议包含：

- `cv::Mat final_bgr`
- `cv::Mat reference_background_bgr`
- `cv::Mat trail_layer_bgr`
- `accepted_frame_count`
- `dropped_frame_count`
- `std::vector<FrameDebug> frame_debug_list`

## 7. 核心模块职责

### 7.1 ReplaySource

定义统一输入接口，避免算法层直接依赖 `cv::VideoCapture`。

建议接口职责：

- 打开输入源
- 逐帧读取 `FramePacket`
- 提供帧率和分辨率信息

首版只实现：

- `VideoFileSource`

后续可扩展：

- `ImageSequenceSource`
- `LiveCameraSource`

### 7.2 WindowAssembler

负责根据 `TriggerSpec` 从回放源中组装出一个完整的 `WindowCapture`。

职责包括：

- 维护预触发帧缓存
- 根据触发帧号或时间戳确定窗口起点
- 读取触发后固定 3 秒数据
- 在读入时执行可选裁切

首版建议让窗口组装逻辑只服务“单次触发、单窗口”场景，不提前引入多窗口调度。

### 7.3 ReferenceBuilder

负责从预触发帧生成参考背景。

职责包括：

- 选择背景锚点帧
- 将其余预触发帧对齐到锚点
- 在对齐后做时域中值融合
- 输出背景的 BGR 图和灰度图

说明：

- 该模块不关心触发后运动目标
- 重点是给后续稳像提供统一坐标系

### 7.4 RegistrationEngine

负责将触发后每一帧对齐到参考背景坐标系。

建议分两级：

1. 粗配准
2. 精配准

粗配准职责：

- 在降采样亮度图上做相位相关
- 估计平移初值

精配准职责：

- 基于掩膜的 Euclidean ECC
- 估计 `tx + ty + theta`

失败处理：

- 若 ECC 失败但平移可信，则退化为平移结果
- 若平移也不可信，则该帧标记为拒绝

### 7.5 MotionExtractor

负责从已配准帧中提取运动亮迹掩膜。

职责包括：

- 转换到亮度通道
- 与参考背景做正向差分
- 按背景噪声做自适应阈值
- 执行最小面积筛除和轻量去噪

该模块输出的是：

- 二值亮迹掩膜
- 像素计数等统计信息

不输出：

- 目标 ID
- 轨迹线段
- 目标中心点

### 7.6 TrailCompositor

负责维护轨迹层并输出最终图。

职责包括：

- 初始化空白轨迹层
- 对接纳帧仅在掩膜区域更新轨迹层
- 轨迹层按 RGB 通道逐像素取极大值
- 最终将轨迹层与参考背景合成

设计要求：

- 不覆盖背景静态区域
- 不做亮度累积
- 不做人为平滑或插值

### 7.7 DebugRecorder

负责把处理过程中的结构化信息和关键图像输出到结果目录。

建议输出：

- `final.png`
- `background.png`
- `trail_layer.png`
- `metrics.json`
- 若干关键帧的 `mask` 或 `overlay`

默认不要保存每一帧完整图像，除非后续联调明确需要。

### 7.8 OfflineProcessor

这是核心编排模块，负责串联全部处理阶段。

处理顺序建议固定为：

1. 获取窗口
2. 构建参考背景
3. 初始化轨迹层
4. 对每个触发后帧执行配准
5. 对接纳帧执行亮迹提取
6. 更新轨迹层
7. 生成最终结果
8. 输出调试信息

`OfflineProcessor`负责流程控制，但不应承载具体图像算法细节。

## 8. 数据流

建议采用如下数据流：

```text
VideoFileSource
  -> WindowAssembler
  -> WindowCapture
  -> ReferenceBuilder
  -> RegistrationEngine (per frame)
  -> MotionExtractor (accepted frames only)
  -> TrailCompositor
  -> CompositeResult
  -> DebugRecorder / CLI output
```

几个关键点：

- 参考背景只构建一次
- 每帧只相对于参考背景配准，不做逐帧累计配准
- 轨迹层是窗口级状态对象
- 调试信息按帧累计，但整图中间结果不长期持有

## 9. 内存与性能策略

### 9.1 原始帧持有策略

在 `1440x1080`、`8-bit BGR`、`165 Hz`、`3 秒` 条件下，单窗口原始图像量级较大。

因此首版实现必须遵守：

- 原始帧只保留一份
- 不缓存“全窗口对齐后帧数组”
- 不缓存“全窗口灰度帧数组”
- 中间临时图尽量复用工作缓冲

### 9.2 裁切优先级

若现场允许，建议优先使用 `960x960` 工作裁切区。

这样可以显著降低：

- 内存占用
- ECC 计算量
- 调试导出负担

### 9.3 并发策略

首版不建议先做复杂多线程。

原因：

- 当前重点是算法稳定性和可解释性
- 多线程会提高调试难度
- 单窗口后处理模型本身不要求极低延迟

后续如需优化性能，可优先考虑：

- 预分配缓冲
- ROI 裁切
- 金字塔和降采样策略
- 再考虑多线程

## 10. CLI 交付形态

建议首版命令行为：

```bash
lob_hero_cli \
  --input sample.mp4 \
  --trigger-frame 120 \
  --output-dir out/run_001 \
  --crop 240,60,960,960
```

CLI 至少支持：

- `--input`
- `--trigger-frame` 或 `--trigger-time-ms`
- `--output-dir`
- `--crop`
- `--config`
- `--debug`

输出目录建议包含：

- 最终结果图
- 背景图
- 轨迹层图
- 结构化指标文件

## 11. 测试与验证策略

### 11.1 单元测试

优先覆盖：

- 背景构建
- 配准退化逻辑
- 正向差分阈值逻辑
- 轨迹层取极大值逻辑

### 11.2 集成测试

至少覆盖以下样例：

- 单目标运动
- 多发光物体运动
- 静态灯条背景
- 小幅抖动
- 局部过曝
- 个别帧配准失败

### 11.3 资源测试

需要额外统计：

- 单窗口峰值内存
- 总处理时长
- 每阶段耗时
- 被拒绝帧比例

## 12. 分阶段实现建议

建议按以下顺序推进：

### 阶段 1：工程骨架

- 搭建 CMake 工程
- 建立 core 和 cli 两层
- 跑通视频读取、触发定位和结果目录输出

### 阶段 2：最小算法闭环

- 构建参考背景
- 完成单帧到背景的配准
- 完成亮迹掩膜和轨迹层取极大值
- 输出一张完整结果图

### 阶段 3：异常与调试

- 增加配准退化逻辑
- 增加帧级调试信息
- 增加关键中间图导出

### 阶段 4：稳定性增强

- 优化自动选块
- 优化阈值与掩膜策略
- 对不同裁切区和不同场景做联调

## 13. 默认结论

首版代码架构建议明确遵守以下默认结论：

- 首版用 C++ 和 OpenCV 实现
- 首版做成库加 CLI，而不是只做单体脚本
- 首版先服务离线文件回放，不先做实时相机接入
- 首版按完整窗口后处理，不先做实时流水线
- 首版以内存持有单窗口原始帧，但严格控制中间副本
- 首版默认输出结果目录和必要调试信息
- 首版优先保证模块边界清晰，为后续实时接入保留扩展空间
