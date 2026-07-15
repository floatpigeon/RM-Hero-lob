# 图像配准模块方案对比与优化思路

## 背景

Hero-Lob pipeline 中新增图像配准模块，用于将每帧对齐到参考帧，消除相机抖动对背景差分的影响。

当前实现了两种方案：
- **方案一：相位相关（Phase Correlation）** — 纯平移配准
- **方案二：ORB + Homography** — 特征点配准

---

## 方案对比

### 算法原理

| | 相位相关 | ORB + Homography |
|---|---|---|
| 核心思路 | FFT 互功率谱算平移量 | 局部特征点匹配 + 单应性估计 |
| 处理步骤 | 灰度→FFT→互功率谱→IFFT→平移 | 灰度→ORB提取→BFMatcher→ratio test→RANSAC→Homography |
| 支持变换 | 仅纯平移 (dx, dy) | 平移 + 旋转 + 缩放 + 透视 |
| 精度 | 亚像素级 | 像素级 |

### 算法复杂度

设图像尺寸 W×H，N 为特征点数量。

| | 相位相关 | ORB + Homography |
|---|---|---|
| 时间复杂度 | O(W·H·log(W·H)) | O(W·H + N·M + K·N) |
| 空间复杂度 | O(W·H) | O(W·H + N) |
| 主导项 | FFT | 特征提取 + 匹配 |

### 实测耗时（640×480）

| 步骤 | 相位相关 | ORB + Homography |
|------|----------|------------------|
| 灰度转换 | ~0.2ms | ~0.2ms |
| 降采样 | ~0.2ms | ~0.2ms |
| 特征提取 | — | ~3.1ms（46.7%） |
| FFT/IFFT | ~0.3ms | — |
| 匹配 | — | ~0.1ms（1.5%） |
| RANSAC | — | ~0.4ms（6.5%） |
| warp | ~0.2ms (Affine) | ~1.7ms (Perspective) |
| HSV 转换 | — | ~1.3ms（20.1%） |
| **总计** | **~0.7ms** | **~6.6ms** |
| **加速比** | **~9x 快** | 基准 |

### ORB 版本耗时分布

| 步骤 | 平均耗时 | 占比 |
|------|----------|------|
| Cur Frame Preprocess | 3.10ms | 46.7% |
| Image Transform | 1.65ms | 24.8% |
| Result Output (HSV) | 1.33ms | 20.1% |
| Homography Estimation | 0.43ms | 6.5% |
| Feature Matching | 0.10ms | 1.5% |

### 适用场景

| 场景 | 推荐方案 | 原因 |
|------|----------|------|
| 固定机位 + 轻微晃动 | 相位相关 | 快速、够用 |
| 手持拍摄 + 明显抖动 | ORB | 需处理旋转 |
| 运动模糊严重 | ORB | 特征描述子抗模糊 |
| 光照剧烈变化 | ORB | 描述子不变性 |
| 实时性要求高 | 相位相关 | < 1ms |
| 纹理稀疏场景 | ORB | 局部特征 |
| 高对比度亮点 + 大面积暗区 | ORB | 局部特征更鲁棒 |

### 本项目结论

本场景为**固定机位 + 轻微晃动 + 高对比度亮点 + 大面积暗区**。

实测发现：
- 相位相关理论快 10-20 倍，但实测差距缩小（OpenCV 内部优化 + 窗函数开销）
- 相位相关对暗区为主的场景效果差（FFT 信噪比低）
- ORB 局部特征对亮点匹配更鲁棒

**当前选择：相位相关作为默认，ORB 保留作为备选。**

---

## 优化思路

### 已尝试的优化（均回退）

| 优化 | 预期 | 实际结果 | 失败原因 |
|------|------|----------|----------|
| 跳过近似单位变换 | 省整帧 ~13ms | 效果不达预期 | 连续帧跳过导致配准不连续 |
| downscale_factor 0.5→0.25 | 省 ~1ms | 质量严重下降 | 特征点太少，匹配不可靠 |
| max_features 500→200 | 省 ~0.5ms | 质量下降 | 特征点不足 |
| cross-check 替代 knnMatch | 省 ~0.05ms | 匹配质量差 | 无 ratio test 过滤误匹配 |
| 移除 HSV 转换 | 省 ~1.3ms | 下游报错 | BackgroundRemover 依赖 HSV |
| warpAffine 替代 warpPerspective | 省 ~0.3ms | 一起回退 | 与上述优化一起测试 |

### 未尝试的安全优化

| 优化 | 预计节省 | 风险 | 说明 |
|------|----------|------|------|
| downscale_factor 0.5→0.4 | ~0.5ms | 低 | 仅降 20%，特征点质量影响小 |
| max_features 500→400 | ~0.3ms | 低 | 保留足够特征点 |
| 限制 RANSAC 迭代次数（2000→500） | ~0.2ms | 低 | 通常 200-500 次足够收敛 |
| BFMatcher 复用为成员变量 | ~0.1ms | 无 | 避免每帧构造 |
| **总计** | **~1.1ms** | | 从 ~6.6ms 降至 ~5.5ms |

### 左上角遮罩（已实现）

- 配置 `exclude_top_left_width` 和 `exclude_top_left_height` 排除数字区域
- 避免变化的 OSD 数字干扰特征匹配
- 配合 `line_marker` 工具可视化确认遮罩范围

### 瓶颈分析

ORB 版本的真正瓶颈在 **Cur Frame Preprocess（46.7%）**，即 ORB 特征提取。这是算法固有开销，难以大幅优化。

如果需要更快，需要换算法思路（如 ECC、稀疏光流），而非继续压榨 ORB。

---

## 文件清单

| 文件 | 说明 |
|------|------|
| `core/include/image_registrator.hpp` | 相位相关版本头文件 |
| `core/src/image_registrator.cpp` | 相位相关版本实现 |
| `core/include/image_registrator_orb.hpp` | ORB 版本头文件 |
| `core/src/image_registrator_orb.cpp` | ORB 版本实现（含耗时统计） |
| `core/tools/line_marker_main.cpp` | 辅助工具：画十字线确定遮罩范围 |

## 配置参数

### 相位相关版本

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_shift_pixels` | 3.0 | 平移量超过此阈值时跳过配准 |
| `downscale_factor` | 0.5 | 降采样比例 |

### ORB 版本

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_features` | 500 | ORB 最大特征点数 |
| `match_ratio_threshold` | 0.75 | Lowe 比例测试阈值 |
| `ransac_reproj_threshold` | 3.0 | RANSAC 重投影误差阈值 |
| `min_matches` | 10 | 最少匹配点数 |
| `downscale_factor` | 0.5 | 降采样比例 |
| `exclude_top_left_width` | 0 | 左上角遮罩宽度 |
| `exclude_top_left_height` | 0 | 左上角遮罩高度 |
