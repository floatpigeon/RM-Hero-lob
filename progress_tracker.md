# image_trans 当前进度

## 概述

当前仓库已经具备一个可编译的 `C++23 + OpenCV` 离线轨迹叠加工程骨架。

当前状态如下：

- 工程目录结构已经建立。
- 核心接口和模块边界已经定义完成。
- CLI 到输出目录的端到端占位流程已经跑通。
- 部分算法模块仍然是占位实现，尚未完成目标图像处理逻辑。

## 已实现内容

### 构建系统与工程结构

- 根目录 `CMakeLists.txt` 已建立，并配置了以下目标：
  - `image_trans_core`
  - `image_trans_cli`
  - `image_trans_tests`
- C++ 标准已统一设置为 `C++23`。
- 已通过 `find_package(OpenCV REQUIRED)` 接入 OpenCV。
- 公共头文件已组织到 `include/image_trans/`。
- 源码已按子模块拆分到 `src/`。
- `tests/` 下已建立最小测试目标。

### 公共接口与数据模型

- 核心数据类型已定义在 `include/image_trans/common_types.hpp`。
- 配置结构已定义在 `include/image_trans/config_types.hpp`。
- 输入抽象接口 `ReplaySource` 已定义。
- 离线主流程入口 `OfflineProcessor` 已定义。
- CLI 参数模型和解析入口已定义。

### 已有真实功能实现

- `VideoFileSource`
  - 可以打开视频文件。
  - 可以顺序读取视频帧。
  - 会填充 `frame_index`、`timestamp_ms` 和 `bgr`。
  - 支持 `reset()`。

- `WindowAssembler`
  - 支持按帧号或时间戳解析触发点。
  - 支持维护滚动预触发缓存。
  - 支持采集固定时长的触发后窗口。
  - 支持在组装窗口时执行可选裁切。

- `TrailCompositor`
  - 可以初始化窗口尺寸的轨迹层。
  - 可以在掩膜区域内按通道逐像素取最大值累积轨迹。
  - 可以将轨迹层与参考背景进行合成。

- `DebugRecorder`
  - 可以创建输出目录。
  - 可以输出以下文件：
    - `final.png`
    - `background.png`
    - `trail_layer.png`
    - `metrics.json`

- `OfflineProcessor`
  - 已串联输入源、窗口组装、背景构建、配准、亮迹提取、轨迹合成和调试输出模块。
  - 可以产出 `CompositeResult`。
  - 可以完成从输入视频到输出文件目录的端到端占位流程。

- `image_trans_cli`
  - 已支持解析：
    - `--input`
    - `--output-dir`
    - `--trigger-frame`
    - `--trigger-time-ms`
    - `--crop`
    - `--config`
    - `--debug`
  - 可以根据参数构造配置并启动 `OfflineProcessor`。

## 占位实现模块

以下模块已经存在并且可以编译，但当前实现仍然是占位逻辑，不是目标算法版本。

### `ReferenceBuilder`

当前行为：

- 要求预触发帧缓存非空。
- 默认选择预触发帧中间帧作为锚点。
- 直接使用锚点帧作为 `background_bgr`。
- 仅对锚点帧做灰度转换。

尚未实现的目标行为：

- 锚点质量选择策略。
- 其他预触发帧向锚点对齐。
- 基于时域中值的稳定背景融合。

### `RegistrationEngine`

当前行为：

- 仅校验参考图和输入帧是否存在。
- 始终返回单位变换矩阵。
- 始终将当前帧标记为接纳，分数固定为 `1.0`。
- `warp_bgr()` 已是真实实现，内部使用 `cv::warpAffine`。

尚未实现的目标行为：

- 基于降采样相位相关的粗平移估计。
- 基于 ECC 的 Euclidean 精配准。
- 配准掩膜构建。
- 基于分数的接纳/拒绝逻辑。
- 平移退化逻辑。
- 有意义的 `RejectReason` 判定。

### `MotionExtractor`

当前行为：

- 仅校验参考背景和对齐后图像是否存在。
- 始终返回全零二值掩膜。
- 始终返回 `motion_pixel_count = 0`。

尚未实现的目标行为：

- 对齐图像灰度化。
- 相对参考背景的正向残差计算。
- 基于背景噪声的自适应阈值。
- 最低可见亮度约束。
- 小连通域过滤或噪点清理。

## 尚未实现内容

### 算法能力

- 多帧预触发图像的稳定背景融合。
- 真实的图像配准主流程。
- 真实的运动亮迹掩膜提取。
- 饱和区掩膜和动态区域掩膜。
- 基于真实质量指标的帧拒绝逻辑。
- 关键掩膜导出或更丰富的调试中间结果。

### 配置与运行能力

- 从 `--config` 加载算法配置。
- 外部配置文件格式校验和结构定义。
- 更细粒度的运行时指标，例如分阶段耗时。
- 可选保存关键掩膜或叠加图。

### 测试缺口

- 目前还没有为以下模块补测试：
  - `VideoFileSource`
  - `ReferenceBuilder`
  - `RegistrationEngine`
  - `MotionExtractor`
  - `DebugRecorder`
  - `OfflineProcessor` 基于真实样例视频的端到端行为
- 目前还没有建立回归测试资源或基于样例数据的集成测试。

## 已验证可用

以下内容已在本地验证通过：

- `cmake -S . -B build`
- `cmake --build build`
- `ctest --test-dir build --output-on-failure`

当前自动化测试已覆盖：

- `TrailCompositor` 的逐像素最大值累积行为
- `WindowAssembler` 的窗口长度计算行为
- `CliOptionsParser` 的触发参数、裁切参数和调试开关解析

## 建议下一步

1. 将 `ReferenceBuilder` 从占位逻辑替换为锚点选择加预触发帧对齐后的中值背景融合。
2. 实现 `RegistrationEngine` 的粗到细配准和拒绝逻辑。
3. 实现 `MotionExtractor` 的正向差分阈值和噪点清理逻辑。
4. 为上述三个算法模块补充针对性测试。
5. 为完整离线流程补一个基于样例视频的集成测试。
