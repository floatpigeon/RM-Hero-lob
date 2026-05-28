# image_trans

离线图传传输模拟器。

它会读取一个输入视频，按以下约束模拟传输链路，并输出处理后的视频：

- 输出固定为 `128x128`
- 输出帧率固定为 `12.5 fps`
- 输出像素格式为 `16-bit grayscale`
- 单包总长度不超过 `300B`
- 最大发包频率为 `50Hz`

当前实现面向离线转换，已包含：

- 发送端按 `50Hz` 节拍发包
- 接收端按到达顺序收包并重组帧
- 基于 `1s` 滑动窗口的实时包率和线速监测

当前仍不模拟丢包、乱序、抖动或重传。

## 依赖

- CMake `>= 3.18`
- C++17 编译器
- OpenCV 4
- FFmpeg

在当前实现里：

- OpenCV 负责读视频、缩放、灰度化、JPEG 编码与解码
- FFmpeg 负责把中间 `16-bit PNG` 序列封装成最终 `gray16le` 视频

## 构建

```bash
cmake -S . -B build
cmake --build build
```

运行测试：

```bash
ctest --test-dir build --output-on-failure
```

## 使用方法

基础命令：

```bash
./build/image_trans --input <input_video> --output <output_video.mkv>
```

示例：

```bash
./build/image_trans \
  --input /path/to/input.mp4 \
  --output /path/to/output.mkv
```

执行成功后，程序会输出处理统计，例如：

- 总输出帧数
- 编码帧数
- 重复帧数
- 强制刷新帧数
- 总包数
- 平均每帧包数
- 峰值包率
- 峰值实时线速
- 最大发送排队时延
- 最大单帧传输时延

## 参数说明

- `--input <path>`
  输入视频路径，必填。
- `--output <path>`
  输出视频路径，必填。当前建议使用 `.mkv`。
- `--size <int>`
  输出正方形分辨率，默认 `128`。
  当前方案按 `128x128` 设计和验证，虽然参数可传，但不建议偏离需求值。
- `--fps <double>`
  输出帧率，默认 `12.5`。
- `--packet-bytes <int>`
  单包总字节数上限，默认 `300`。
- `--packet-hz <int>`
  最大发包频率，默认 `50`。
- `--header-bytes <int>`
  包头字节数，当前实现固定为 `12`。
- `--refresh-interval <int>`
  每隔多少输出帧强制发送一次 JPEG 刷新帧，默认 `12`。
- `--temp-dir <path>`
  指定中间 PNG 帧目录。若指定该参数，中间文件会保留。
- `--keep-temp`
  保留中间 PNG 帧。
- `--help`
  查看帮助。

## 处理流程

程序当前按下面的方式工作：

1. 用 OpenCV 读取输入视频。
2. 按 `12.5 fps` 对输入视频做重采样。
3. 将画面中心裁成正方形，再缩放为 `128x128`。
4. 转成 `8-bit` 灰度工作图。
5. 若当前帧与上一重建帧差异很小，则发送“重复上一帧”控制包。
6. 否则尝试用 JPEG 编码，并确保单帧总有效载荷不超过链路预算。
7. 若仍超预算，则先降采样再回放大并轻微模糊，再重试编码。
8. 发送端把分包结果按 `50Hz` 节拍调度发送。
9. 发送时持续监测最近 `1s` 内的包数和总线速，若下一包会超限，则自动顺延发送时刻。
10. 接收端按包顺序校验 CRC、重组帧，并在完整帧到达后输出重建结果。
11. 最终把 `8-bit` 灰度扩展为 `16-bit` 灰度，写成 PNG 序列，再由 FFmpeg 封装为输出视频。

## 输出规格验证

可以用 `ffprobe` 验证输出是否满足要求：

```bash
ffprobe -v error \
  -select_streams v:0 \
  -show_entries stream=codec_name,pix_fmt,width,height,avg_frame_rate \
  -of default=noprint_wrappers=1 \
  /path/to/output.mkv
```

期望看到类似结果：

```text
codec_name=ffv1
width=128
height=128
pix_fmt=gray16le
avg_frame_rate=25/2
```

其中 `25/2` 即 `12.5 fps`。

## 实现约束

- 包头固定为 `12B`
- 单包有效载荷上限为 `288B`
- 在默认配置下，每帧最多允许 `4` 个包
- 发送端按 `20ms` 一个发包时隙进行调度
- 实时速率监测窗口为最近 `1s`
- 链路内部工作图为 `8-bit grayscale`
- 最终输出文件为 `16-bit grayscale`

## 已知限制

- 当前是离线转换工具，不是实时图传程序
- 不模拟丢包、乱序、网络时延或重传
- 输出封装依赖系统里的 `ffmpeg` 命令可执行
- 若输入视频无法被 OpenCV 打开，程序会直接失败
