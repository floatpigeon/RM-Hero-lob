# Hero-Lob

基于 C++ 和 OpenCV 的视频长曝光合成管线。对固定视角视频进行配准、前景提取和轨迹累积，输出类似星轨效果的长曝光图像。

## 算法流程

```
输入视频 → [ReferenceFrameSelector] → [ImageRegistratorOrb] → [BackgroundRemover] → [TrackerProcessorFast] → [ImageSynthesis] → 输出长曝光图
```

### 1. ReferenceFrameSelector — 参考帧选择

将第一帧设为参考帧并冻结，后续帧复用此参考帧进行配准。

### 2. ImageRegistratorOrb — ORB 图像配准

对当前帧与参考帧进行特征匹配和透视变换，输出半尺寸配准结果。

**子流程**:

| 阶段 | 操作 |
|------|------|
| Input Validation | 检查参考帧和当前帧有效性 |
| Ref Frame Preprocess | 灰度转换 → 降采样(0.5x) → ORB 特征提取（仅参考帧变化时执行） |
| Cur Frame Preprocess | 灰度转换 → 降采样(0.5x) → ORB 特征提取 |
| Feature Matching | BFMatcher knnMatch(k=2) + Lowe's ratio test(0.75) |
| Homography Estimation | RANSAC 求单应矩阵(重投影阈值 3.0) |
| Image Transform | 缩放 H 矩阵 → warpPerspective 输出半尺寸 |
| Result Output | BGR→HSV 转换 |

**关键参数**:
- `max_features=200`: ORB 特征点数
- `downscale_factor=0.5`: 特征检测降采样比
- `out_scale=0.5`: 输出图像缩放比
- `exclude_top_left={100,100}`: 排除左上角区域

### 3. BackgroundRemover — 前景提取

差分法提取运动前景：
1. 参考帧与配准帧灰度差分
2. 亮度阈值(128) + 差分阈值(24) → 候选掩码
3. 形态学开运算(kernel=3) + 闭运算(kernel=5) 去噪

### 4. TrackerProcessorFast — 轨迹累积

连通域分析 + 时间连续性筛选，累积轨迹层：
1. `connectedComponentsWithStats` 提取连通域
2. 过滤面积 < 5px 的连通域
3. 与上一帧连通域匹配（距离 < 120px）
4. 计算速度方向，排除垂直运动（角度 > 40°）
5. 符合条件的连通域 BGR 累加到 `trajectory_layer_`

### 5. ImageSynthesis — 图像合成

1. 除以曝光计数 → 平均亮度
2. P99 归一化 → 映射到 [0, 255]
3. BGR→Lab → CLAHE 亮度增强(a=2.0) → Lab→BGR
4. 参考帧 + 增强轨迹层叠加

### 6. Pipeline 输出

- `resize` 到目标尺寸(288x216)
- `imwrite` 输出 JPG

## 数据流尺寸

```
原图 WxH
  ↓ ImageRegistratorOrb
半尺寸 W/2 x H/2
  ↓ BackgroundRemover / TrackerProcessorFast / ImageSynthesis
半尺寸
  ↓ Pipeline 输出
resize 到 288x216
```

## 构建

需要本机已安装：
- CMake 3.20+
- 支持 C++17 的编译器
- OpenCV 4

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
```

## 运行

```bash
./build/hero_lob <input_video> <output_image>
```

示例：

```bash
./build/hero_lob ./data/new-test/shooting.avi ./data/new-test/output/result.jpg
```

## 性能 (240帧测试)

| 模块 | 每帧耗时 | 占比 |
|------|---------|------|
| ImageRegistratorOrb | 6.26ms | 80% |
| BackgroundRemover | 0.67ms | 8.5% |
| TrackerProcessorFast | 0.38ms | 4.9% |
| ImageSynthesis | 0.51ms | 6.5% |
| **总计** | **7.82ms** | **100%** |
| **FPS** | **128 fps** | |

## 测试工具

### 色彩参数获取器

```bash
./build/hero_lob_color_picker /path/to/input.png
```

鼠标左键点击输出原图像素的 BGR 和 HSV 值，按 `q` 或 `Esc` 退出。

### 图片提亮工具

```bash
./build/hero_lob_brighten /path/to/input.png /tmp/output.png
./build/hero_lob_brighten /path/to/input.png /tmp/output_auto.png --auto
```

默认模式对 HSV 的 V 通道应用固定增强，`--auto` 模式对 Lab 的 L 通道应用 CLAHE 自动增强。

### 识别器调试工具

```bash
./build/hero_lob_identifier_debug image /path/to/input.png /tmp/identifier_debug
./build/hero_lob_identifier_debug video /path/to/input.mp4 /tmp/identifier_debug --max-frames 120
```

导出各类 mask 和 overlay 图像，摘要文件输出检测结果。

### 轨迹调试工具

```bash
./build/hero_lob_trajectory_debug /path/to/input.mp4 /tmp/trajectory_debug
./build/hero_lob_trajectory_debug /path/to/input.mp4 /tmp/trajectory_debug --max-frames 180 --gui
```

导出轨迹曝光图、热力图和叠加图，`--gui` 模式支持交互查看。

### 图像压缩工具

```bash
./build/hero_lob_compress /path/to/input.png /tmp/output.jpg --size 55x55 --gray
./build/hero_lob_compress /path/to/input.png /tmp/output.jpg --max-bytes 300 --gray
```

支持 `--size WxH`、`--width W`、`--height H`、`--gray`、`--quality Q`、`--max-bytes N`。

### 中值背景提取工具

```bash
./build/hero_lob_median_background /path/to/input.mp4 /tmp/background.png
./build/hero_lob_median_background /path/to/input.mp4 /tmp/background.png --max-frames 50 --step 3
```

使用中值法从视频中提取静态背景。

## 当前状态

- 已完成：C++ 工程框架、主管线、参考帧选择、ORB 图像配准、前景提取、轨迹累积、图像合成、各项工具。
- 待优化：当前帧预处理(ORB 特征检测)占总耗时 70%，为主要性能瓶颈。
