# C++ 类与接口设计草案

## 1. 文档目的

本文档用于把首版实现中需要的具体类、结构体、接口和调用关系整理清楚。

目标是让后续开始写代码时，不再临时决定：

- 类该放在哪个头文件
- 哪些类是抽象接口，哪些类是首版具体实现
- 每个类暴露哪些方法
- 数据所有权放在哪一层
- 模块之间通过什么结构体交互

本文档与以下文档配套：

- `trajectory_overlay_plan.md`
- `cpp_offline_architecture_plan.md`

## 2. 统一约定

### 2.1 命名空间

统一使用：

```cpp
namespace lob_hero {
}
```

### 2.2 C++ 标准

首版按 `C++20` 设计。

原因：

- 足够现代
- 可以稳定使用 `std::span`
- 不必为了 `std::expected` 额外引入自定义错误容器

### 2.3 错误处理原则

分两类处理：

- `配置错误 / 文件错误 / 环境错误`
  - 直接抛 `std::runtime_error`
- `算法过程中的正常失败`
  - 不抛异常
  - 通过结果结构体中的状态字段表达

例如：

- 视频打不开：异常
- 单帧配准失败：`RegistrationResult::accepted = false`

### 2.4 图像所有权原则

- 原始 `cv::Mat` 由 `WindowCapture` 持有
- 逐帧处理阶段只借用引用，不复制整窗全分辨率图像
- 中间掩膜、灰度图、warp 缓冲由具体算法类内部临时管理

## 3. 建议头文件清单

```text
include/lob_hero/
├── common_types.hpp
├── config_types.hpp
├── replay_source.hpp
├── video_file_source.hpp
├── window_assembler.hpp
├── reference_builder.hpp
├── registration_engine.hpp
├── motion_extractor.hpp
├── trail_compositor.hpp
├── debug_recorder.hpp
├── offline_processor.hpp
└── cli_options.hpp
```

说明：

- `common_types.hpp` 放通用结构体
- `config_types.hpp` 放配置结构
- 抽象接口和具体实现分开头文件
- 首版不额外引入 `factory` 目录

## 4. 通用结构体

### 4.1 common_types.hpp

建议放以下结构：

```cpp
namespace lob_hero {

enum class TriggerMode {
  kFrameIndex,
  kTimestampMs,
};

enum class RejectReason {
  kNone,
  kRegistrationFailed,
  kLowRegistrationScore,
  kInvalidMotionMask,
};

struct TriggerSpec {
  TriggerMode mode;
  std::int64_t value = 0;
};

struct FramePacket {
  std::int64_t frame_index = 0;
  std::int64_t timestamp_ms = 0;
  cv::Mat bgr;
};

struct WindowCapture {
  std::vector<FramePacket> pretrigger_frames;
  std::vector<FramePacket> posttrigger_frames;
  double source_fps = 0.0;
  cv::Size source_frame_size;
};

struct ReferenceFrameSet {
  cv::Mat background_bgr;
  cv::Mat background_gray;
  std::int64_t anchor_frame_index = -1;
};

struct RegistrationResult {
  bool accepted = false;
  bool used_translation_fallback = false;
  double score = 0.0;
  RejectReason reject_reason = RejectReason::kNone;
  cv::Mat warp_2x3;
};

struct MotionMaskResult {
  cv::Mat binary_mask;
  int motion_pixel_count = 0;
};

struct FrameDebug {
  std::int64_t frame_index = 0;
  std::int64_t timestamp_ms = 0;
  bool accepted = false;
  bool used_translation_fallback = false;
  double registration_score = 0.0;
  int motion_pixel_count = 0;
  RejectReason reject_reason = RejectReason::kNone;
};

struct CompositeResult {
  cv::Mat final_bgr;
  cv::Mat reference_background_bgr;
  cv::Mat trail_layer_bgr;
  int accepted_frame_count = 0;
  int dropped_frame_count = 0;
  std::vector<FrameDebug> frame_debug_list;
};

}  // namespace lob_hero
```

设计决定：

- `RejectReason` 用枚举，不用裸字符串
- `FrameDebug` 只保留结构化指标，不持有整帧图像
- `ReferenceFrameSet` 单独抽出来，避免背景构建模块只返回一张图而丢失灰度底图

## 5. 配置结构

### 5.1 config_types.hpp

建议拆成四个配置结构，而不是把所有参数塞进一个巨大结构体。

```cpp
namespace lob_hero {

struct CropConfig {
  bool enabled = false;
  cv::Rect crop_rect;
};

struct RegistrationConfig {
  double downsample_ratio = 0.5;
  int auto_tile_count = 6;
  int ecc_iterations = 50;
  double registration_score_threshold = 0.90;
  double translation_fallback_threshold = 0.60;
  int saturation_threshold = 245;
  int min_tile_gradient = 20;
};

struct MotionConfig {
  double sigma_multiplier = 3.0;
  int floor_threshold = 18;
  int min_visible_value = 32;
  int min_blob_area = 6;
};

struct DebugConfig {
  bool enabled = false;
  bool export_key_masks = true;
  int max_key_masks = 12;
};

struct CompositeConfig {
  CropConfig crop;
  RegistrationConfig registration;
  MotionConfig motion;
  DebugConfig debug;
  int pretrigger_frame_count = 10;
  int capture_duration_ms = 3000;
};

}  // namespace lob_hero
```

设计决定：

- `CompositeConfig` 仍然是统一入口配置
- 但内部按功能分组，避免接口方法参数表爆炸
- 首版不做配置热更新

## 6. 输入层接口

### 6.1 replay_source.hpp

`ReplaySource` 是抽象基类。

```cpp
namespace lob_hero {

class ReplaySource {
 public:
  virtual ~ReplaySource() = default;

  virtual double fps() const = 0;
  virtual cv::Size frame_size() const = 0;
  virtual bool read(FramePacket& out_frame) = 0;
  virtual void reset() = 0;
};

}  // namespace lob_hero
```

设计决定：

- `read()` 返回 `bool`，EOF 时返回 `false`
- 首版不引入异步读取接口
- `reset()` 允许离线回放重新跑同一视频

### 6.2 video_file_source.hpp

首版唯一具体输入实现：

```cpp
namespace lob_hero {

class VideoFileSource final : public ReplaySource {
 public:
  explicit VideoFileSource(const std::filesystem::path& input_path);

  double fps() const override;
  cv::Size frame_size() const override;
  bool read(FramePacket& out_frame) override;
  void reset() override;

 private:
  std::filesystem::path input_path_;
  cv::VideoCapture capture_;
  std::int64_t next_frame_index_ = 0;
};

}  // namespace lob_hero
```

设计决定：

- 构造函数完成打开动作，失败直接抛异常
- `timestamp_ms` 由 `frame_index / fps` 计算，不依赖容器内嵌时间戳

## 7. 窗口组装层

### 7.1 window_assembler.hpp

```cpp
namespace lob_hero {

class WindowAssembler {
 public:
  explicit WindowAssembler(CompositeConfig config);

  WindowCapture build_window(ReplaySource& source, const TriggerSpec& trigger) const;

 private:
  CompositeConfig config_;

  cv::Mat apply_crop_if_needed(const cv::Mat& input) const;
  std::int64_t resolve_trigger_frame_index(
      ReplaySource& source, const TriggerSpec& trigger) const;
};

}  // namespace lob_hero
```

职责边界：

- 负责把完整窗口从回放源里读出来
- 负责裁切
- 不负责任何配准和算法判断

设计决定：

- `build_window()` 返回完整 `WindowCapture`
- 首版以“单次运行一个触发窗口”为默认约束
- `resolve_trigger_frame_index()` 内部如需重置视频流，可调用 `source.reset()`

## 8. 背景构建层

### 8.1 reference_builder.hpp

```cpp
namespace lob_hero {

class ReferenceBuilder {
 public:
  explicit ReferenceBuilder(CompositeConfig config);

  ReferenceFrameSet build(const WindowCapture& window) const;

 private:
  CompositeConfig config_;

  int select_anchor_index(std::span<const FramePacket> frames) const;
  cv::Mat to_gray(const cv::Mat& bgr) const;
  cv::Mat align_to_anchor(
      const cv::Mat& anchor_gray,
      const cv::Mat& frame_bgr) const;
};

}  // namespace lob_hero
```

设计决定：

- `build()` 只使用 `pretrigger_frames`
- 锚点选择逻辑封装在内部，不暴露给上层
- `align_to_anchor()` 首版可只做平移对齐，用于构建较稳定的背景

说明：

- 这里故意不依赖 `RegistrationEngine`
- 预触发背景对齐可以先用更轻量的内部逻辑，避免两层相互耦合

## 9. 配准层

### 9.1 registration_engine.hpp

```cpp
namespace lob_hero {

struct RegistrationContext {
  cv::Mat reference_gray;
  cv::Mat reference_bgr;
};

class RegistrationEngine {
 public:
  explicit RegistrationEngine(CompositeConfig config);

  RegistrationResult estimate(
      const RegistrationContext& context,
      const FramePacket& frame) const;

  cv::Mat warp_bgr(
      const cv::Mat& input_bgr,
      const cv::Mat& warp_2x3,
      const cv::Size& output_size) const;

 private:
  CompositeConfig config_;

  cv::Mat build_registration_mask(
      const cv::Mat& reference_gray,
      const cv::Mat& current_gray) const;

  cv::Point2d estimate_translation_phase_correlation(
      const cv::Mat& reference_gray,
      const cv::Mat& current_gray,
      double* response) const;

  cv::Mat estimate_euclidean_warp(
      const cv::Mat& reference_gray,
      const cv::Mat& current_gray,
      const cv::Mat& mask,
      const cv::Point2d& initial_translation,
      double* ecc_score) const;
};

}  // namespace lob_hero
```

设计决定：

- `estimate()` 只负责给出 warp 和状态，不直接返回对齐图像
- `warp_bgr()` 单独暴露，避免上层重复写 `warpAffine`
- `RegistrationContext` 显式传入参考背景，而不是在构造函数中长期持有窗口级状态

说明：

- 这样 `RegistrationEngine` 可以无状态复用
- 后续若要支持不同配准策略，也容易扩展成同接口的多个实现

## 10. 亮迹提取层

### 10.1 motion_extractor.hpp

```cpp
namespace lob_hero {

class MotionExtractor {
 public:
  explicit MotionExtractor(CompositeConfig config);

  MotionMaskResult extract(
      const ReferenceFrameSet& reference,
      const cv::Mat& aligned_bgr) const;

 private:
  CompositeConfig config_;

  cv::Mat compute_positive_residual(
      const cv::Mat& reference_gray,
      const cv::Mat& current_gray) const;

  int estimate_noise_floor(const cv::Mat& reference_gray) const;
  cv::Mat threshold_motion(const cv::Mat& positive_residual) const;
  cv::Mat remove_small_components(const cv::Mat& binary_mask) const;
};

}  // namespace lob_hero
```

设计决定：

- 输入是已经配准后的 BGR 图像
- 输出只有掩膜和统计，不输出任何轨迹对象
- 噪声估计逻辑内聚在模块内部，上层不参与阈值细节

## 11. 合成层

### 11.1 trail_compositor.hpp

`TrailCompositor` 是少数需要持有窗口级内部状态的类。

```cpp
namespace lob_hero {

class TrailCompositor {
 public:
  explicit TrailCompositor(cv::Size canvas_size);

  void reset();

  void accumulate(
      const cv::Mat& aligned_bgr,
      const cv::Mat& motion_mask);

  cv::Mat trail_layer() const;

  cv::Mat compose_with_background(const cv::Mat& reference_bgr) const;

 private:
  cv::Mat trail_layer_bgr_;
};

}  // namespace lob_hero
```

设计决定：

- `TrailCompositor` 构造时只依赖画布尺寸，不依赖大配置对象
- `accumulate()` 内部做逐像素取极大值
- `trail_layer()` 返回当前轨迹层拷贝；若后续确认性能敏感，可改为 `const cv::Mat&`

## 12. 调试输出层

### 12.1 debug_recorder.hpp

```cpp
namespace lob_hero {

struct DebugArtifacts {
  cv::Mat final_bgr;
  cv::Mat background_bgr;
  cv::Mat trail_layer_bgr;
  std::vector<FrameDebug> frame_debug_list;
  std::vector<std::pair<std::int64_t, cv::Mat>> key_masks;
};

class DebugRecorder {
 public:
  explicit DebugRecorder(CompositeConfig config);

  void write(
      const std::filesystem::path& output_dir,
      const DebugArtifacts& artifacts) const;

 private:
  CompositeConfig config_;

  void write_metrics_json(
      const std::filesystem::path& output_dir,
      const DebugArtifacts& artifacts) const;
};

}  // namespace lob_hero
```

设计决定：

- `DebugRecorder` 只做落盘，不参与算法主流程
- `DebugArtifacts` 由上层组装，避免 `DebugRecorder` 反向依赖多个算法模块

## 13. 编排层

### 13.1 offline_processor.hpp

`OfflineProcessor` 是首版主入口。

```cpp
namespace lob_hero {

struct ReplayRequest {
  std::filesystem::path input_path;
  TriggerSpec trigger;
  std::filesystem::path output_dir;
};

class OfflineProcessor {
 public:
  explicit OfflineProcessor(CompositeConfig config);

  CompositeResult run(const ReplayRequest& request) const;

 private:
  CompositeConfig config_;

  CompositeResult process_window(const WindowCapture& window) const;
  FrameDebug make_rejected_debug(
      const FramePacket& frame,
      const RegistrationResult& reg) const;
};

}  // namespace lob_hero
```

调用职责：

- 创建 `VideoFileSource`
- 调用 `WindowAssembler`
- 调用 `ReferenceBuilder`
- 逐帧调用 `RegistrationEngine`
- 对接纳帧调用 `MotionExtractor`
- 使用 `TrailCompositor` 累加结果
- 组装 `CompositeResult`
- 如开启调试，则调用 `DebugRecorder`

设计决定：

- `OfflineProcessor` 是 orchestration 层，不实现任何底层图像算法
- `run()` 内部自己创建具体依赖，不在首版引入复杂 DI 容器

## 14. CLI 层

### 14.1 cli_options.hpp

```cpp
namespace lob_hero {

struct CliOptions {
  std::filesystem::path input_path;
  std::filesystem::path output_dir;
  TriggerSpec trigger;
  std::optional<cv::Rect> crop_rect;
  bool debug_enabled = false;
  std::optional<std::filesystem::path> config_path;
};

CliOptions parse_cli_options(int argc, char** argv);

CompositeConfig make_config_from_options(const CliOptions& options);

}  // namespace lob_hero
```

设计决定：

- CLI 解析与算法配置转换分开
- `main.cpp` 只做三件事：
  - 解析 CLI
  - 构造 `OfflineProcessor`
  - 执行并处理异常

## 15. 类关系图

```text
main
  -> parse_cli_options
  -> make_config_from_options
  -> OfflineProcessor::run
       -> VideoFileSource
       -> WindowAssembler
       -> ReferenceBuilder
       -> RegistrationEngine
       -> MotionExtractor
       -> TrailCompositor
       -> DebugRecorder
```

依赖方向要求：

- `ReplaySource` 不依赖任何算法模块
- `ReferenceBuilder`、`RegistrationEngine`、`MotionExtractor`、`TrailCompositor` 彼此不直接持有对方
- `OfflineProcessor` 是唯一允许同时了解全部模块的类

## 16. 首版不引入的类

以下类首版明确不做：

- `LivePipelineController`
- `CameraTriggerListener`
- `ThreadPool`
- `RegistrationStrategyFactory`
- `TrackManager`
- `TrajectoryLinker`
- `GpuRegistrationEngine`

原因：

- 当前目标不是实时系统
- 当前目标不是目标跟踪
- 当前目标是先把稳像和长曝光式合成闭环跑通

## 17. 推荐落地顺序

按代码实现顺序，建议先建这些头文件和类：

1. `common_types.hpp`
2. `config_types.hpp`
3. `replay_source.hpp`
4. `video_file_source.hpp`
5. `window_assembler.hpp`
6. `reference_builder.hpp`
7. `registration_engine.hpp`
8. `motion_extractor.hpp`
9. `trail_compositor.hpp`
10. `offline_processor.hpp`
11. `debug_recorder.hpp`
12. `cli_options.hpp`

原因：

- 先把数据结构和输入层立住
- 再写算法模块
- 最后补调试和 CLI，避免一开始被输出格式牵着走

## 18. 默认结论

为了避免后续实现时反复摇摆，首版按以下结论执行：

- 只保留一个抽象输入接口：`ReplaySource`
- 只做一个首版具体输入实现：`VideoFileSource`
- 算法模块尽量做成无状态或弱状态类
- `TrailCompositor` 是唯一明确持有窗口级内部图层状态的算法类
- `OfflineProcessor` 负责总编排，是首版唯一核心入口
- 调试输出与算法计算分离，统一由 `DebugRecorder` 落盘
- CLI 只负责参数和启动，不承担算法逻辑
