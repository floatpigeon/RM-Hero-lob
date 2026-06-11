# Hero-Lob

这是一个基于 C++ 和 OpenCV 的图像处理管线项目，目标是对固定视角视频中的目标进行识别、跟踪、配准、前景提取和轨迹合成，最终输出一张类似星轨效果的结果图。

当前仓库已经搭好了完整的模块化骨架：
- `capture`
- `identifier`
- `tracker`
- `reference_frame_selector`
- `image_registrator`
- `background_remover`
- `tracker_processor`
- `image_synthesis`

其中 `capture` 已经实现了视频逐帧读取、HSV 转换和 0.5 秒滑动窗口缓存；`identifier` 已经实现了首版亮度优先识别；其余模块目前为可贯通的占位实现，用于跑通整条管线。

## 算法流程

整体流程如下：

1. `capture`
   持续获取图像，转换到 HSV 色域，并缓存最近 0.5 秒的图像序列。

2. `identifier`
   在当前帧中先用亮度筛出所有发光结构，稳健检测引导灯，再只在引导灯下方局部 ROI 中寻找一对稳定竖向短灯条作为主锚，并用边缘未过曝像素后判定红蓝或 `unknown`。

3. `tracker`
   基于上一时刻状态持续跟踪目标；当目标丢失超过 200ms 时，进入重识别。

4. `reference_frame_selector`
   在最近 0.5 秒内判断画面是否稳定；稳定时建立参考帧，触发后冻结参考帧 3 秒。

5. `image_registrator`
   根据当前帧锚点信息，估计相对参考帧的平移和旋转变换，完成画面配准。

6. `background_remover`
   在参考坐标系下生成 ROI 和静态灯排除区域，并提取候选运动前景。

7. `tracker_processor`
   对候选前景做连通域和时间连续性筛选，累积得到轨迹层。

8. `image_synthesis`
   将轨迹层增强后叠加回参考背景帧，输出最终显示图像。

## 构建

需要本机已安装：
- CMake 3.20+
- 支持 C++17 的编译器
- OpenCV 4

构建命令：

```bash
cmake -S . -B build
cmake --build build
```

构建完成后，可执行文件位于：

```bash
./build/hero_lob
```

同时会生成以下工具和测试程序：

```bash
./build/hero_lob_color_picker
./build/hero_lob_brighten
./build/hero_lob_identifier_debug
./build/identifier_geometry_test
./build/identifier_synthetic_test
```

## 运行

程序当前的命令行接口为：

```bash
./build/hero_lob <input_video> <output_image>
```

示例：

```bash
./build/hero_lob /path/to/input.mp4 /tmp/output.png
```

如果输入视频可以正常解码，程序会逐帧跑完整条管线，并在结束后输出一张结果图。

## 测试工具

### 色彩参数获取器

命令：

```bash
./build/hero_lob_color_picker /path/to/input.png
```

行为：
- 打开图片窗口，鼠标左键点击后输出原图像素的 `BGR` 和 `HSV`。
- 大图会按比例缩放显示，但输出的仍是原图坐标和值。
- 画面上只保留最近一次点击的标记和文本。
- 按 `q` 或 `Esc` 退出。

该工具依赖 OpenCV 的图形界面能力，需在有桌面显示的环境下运行。

### 图片提亮工具

命令：

```bash
./build/hero_lob_brighten /path/to/input.png /tmp/output.png
./build/hero_lob_brighten /path/to/input.png /tmp/output_auto.png --auto
```

行为：
- 默认模式对 `HSV` 的 `V` 通道应用固定增强参数。
- `--auto` 模式对 `Lab` 的 `L` 通道应用 `CLAHE` 自动增强。
- 输出图像保持原始分辨率，格式由输出文件扩展名决定。

### 识别器调试工具

命令：

```bash
./build/hero_lob_identifier_debug image /path/to/input.png /tmp/identifier_debug
./build/hero_lob_identifier_debug video /path/to/input.mp4 /tmp/identifier_debug --max-frames 120
```

行为：
- 导出 `raw_brightness_mask`、`brightness_mask`、`guide_candidate_mask`、`light_candidate_mask`、`stable_pair_roi`、`edge_red_mask`、`edge_blue_mask`、`candidate_overlay`、`stable_pair_overlay`、`result_overlay`。
- 摘要文件会输出是否检测成功、最终颜色类别，以及 guide 和稳定竖向短灯条对的 anchors。
- `--gui` 模式可在原图、各类 mask 和 overlay 之间切换查看。

## 当前状态

- 已完成：C++ 工程框架、模块划分、主管线、图像获取模块、亮度优先识别器、三项工具入口、识别器几何与合成图测试。
- 未完成：跟踪、参考帧选择、配准、背景剔除、轨迹增强等后续算法细节仍为占位实现。
