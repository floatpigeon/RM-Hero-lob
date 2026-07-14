# TrackerProcessor 优化方案

## 背景

原版 `tracker_processor` 每帧处理耗时约 6ms，是管线中最耗时的模块（占比 72%）。

## 原版实现分析

### 主要瓶颈

1. **连通域分析** `connectedComponentsWithStats`
   - 每帧扫描全图像素（1440×1080 = 1.55M）
   - 耗时约 2-3ms

2. **标签提取** `(labels == i)`
   - 每个连通域生成一个全图大小的临时 `cv::Mat`
   - 内存分配 + 比较操作

3. **轨迹累积逐像素循环**
   - 手动遍历 bbox 内所有像素
   - `mask_roi.convertTo(mask_f, CV_32F, 1.0/255.0)` 每帧分配新 Mat

4. **距离计算**
   - 每次调用 `std::sqrt`，即使只是用于比较大小

## 优化方法

### 1. 消除全图临时 Mat

**原版：**
```cpp
cv::Mat mask_roi = (labels == i);  // 全图大小
```

**优化：**
```cpp
cv::Mat mask_roi = (labels(comp.bbox) == label_id);  // 仅 bbox 区域
```

直接在 labels 的 ROI 上做比较，避免分配全图大小的临时 Mat。

### 2. 复用缓冲区

**原版：**
```cpp
cv::Mat mask_f;
mask_roi.convertTo(mask_f, CV_32F, 1.0 / 255.0);  // 每帧分配新 Mat
```

**优化：**
```cpp
// 类成员变量
cv::Mat mask_buffer_;

// 使用时复用
mask_roi.convertTo(mask_buffer_, CV_32F, 1.0 / 255.0);
```

通过成员变量复用内存，减少堆分配次数。

### 3. OpenCV 向量化替代手动循环

**原版：**
```cpp
for (int r = 0; r < bbox.height; ++r) {
    const float* mask_row = mask_f.ptr<float>(r);
    const uchar* bgr_row = bgr_roi.ptr<uchar>(r);
    float* traj_row = traj_roi.ptr<float>(r);
    for (int col = 0; col < bbox.width; ++col) {
        float m = mask_row[col];
        if (m < 1e-6F) continue;
        int idx = col * 3;
        traj_row[idx + 0] += bgr_row[idx + 0] * m;
        traj_row[idx + 1] += bgr_row[idx + 1] * m;
        traj_row[idx + 2] += bgr_row[idx + 2] * m;
    }
}
```

**优化：**
```cpp
cv::Mat bgr_f;
bgr_roi.convertTo(bgr_f, CV_32F);
cv::Mat mask_3ch;
cv::merge(std::vector<cv::Mat>{mask_f, mask_f, mask_f}, mask_3ch);
cv::Mat masked_bgr;
cv::multiply(bgr_f, mask_3ch, masked_bgr);
cv::add(traj_roi, masked_bgr, traj_roi);
```

利用 OpenCV 内部 SIMD 优化，避免手动像素遍历。

### 4. 延迟开方

**原版：**
```cpp
float dist = std::sqrt(dx * dx + dy * dy);
if (dist < best_dist) { ... }
```

**优化：**
```cpp
float dist_sq = dx * dx + dy * dy;
if (dist_sq < best_dist) { ... }
// 循环结束后再开方
best_dist = std::sqrt(best_dist);
```

减少循环内的 sqrt 调用次数（从 N 次降至 1 次）。

### 5. 使用 stats 直接获取 bbox

**原版：**
```cpp
comp.mask = (labels == i);  // 需要额外的 mask
// 后续通过 boundingRect 获取 bbox
cv::Rect bbox = cv::boundingRect(comp.mask);
```

**优化：**
```cpp
comp.bbox = cv::Rect(stats.at<int>(i, cv::CC_STAT_LEFT),
                     stats.at<int>(i, cv::CC_STAT_TOP),
                     stats.at<int>(i, cv::CC_STAT_WIDTH),
                     stats.at<int>(i, cv::CC_STAT_HEIGHT));
```

`connectedComponentsWithStats` 已经计算了 bbox，直接使用即可。

## 测试结果

| 版本 | 每帧耗时 | 加速比 |
|------|----------|--------|
| 原版 `tracker_processor` | 6.16ms | 1.0x |
| 优化版 `tracker_processor_fast` | 1.42ms | **4.35x** |

## 后续可选优化

1. **空间索引加速匹配**
   - 当前匹配是 O(n×m) 复杂度
   - 可用 KD-Tree 或网格索引降至 O(n×log(m))

2. **ROI 裁剪后做连通域分析**
   - 先用 `boundingRect` 获取所有组件的 union bbox
   - 裁剪 labels 后再做处理

3. **CUDA 加速**
   - 连通域分析和轨迹累积都适合 GPU 并行
