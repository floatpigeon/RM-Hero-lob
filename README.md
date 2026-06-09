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

其中 `capture` 已经实现了视频逐帧读取、HSV 转换和 0.5 秒滑动窗口缓存；其余模块目前为可贯通的占位实现，用于跑通整条管线。

## 算法流程

整体流程如下：

1. `capture`
   持续获取图像，转换到 HSV 色域，并缓存最近 0.5 秒的图像序列。

2. `identifier`
   在当前帧中识别目标，包括绿色引导灯、左右灯条、方向信息和颜色类别。

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

同时会生成两个测试工具：

```bash
./build/hero_lob_color_picker
./build/hero_lob_brighten
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

## 当前状态

- 已完成：C++ 工程框架、模块划分、主管线、图像获取模块、两项测试工具入口。
- 未完成：真实的目标识别、跟踪、参考帧选择、配准、背景剔除、轨迹增强等算法细节。
