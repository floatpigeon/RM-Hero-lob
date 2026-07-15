# ImageRegistratorOrb 模块分析

## 模块功能
基于ORB特征点的图像配准模块，用于将当前帧与参考帧进行空间对齐。

## 核心流程

| 步骤 | 操作 | 输入 | 输出 | 关键配置 | 备注 |
|------|------|------|------|----------|------|
| 1 | 输入验证 | `reference`, `frame` | `result.valid` | 无 | 若参考帧无效或当前帧为空，返回无效结果 |
| 2 | 参考帧缓存判断 | `reference.reference_frame.frame_index` | `cached_ref_frame_index_` | 无 | 仅当参考帧变化时重新提取特征，避免重复计算 |
| 3 | 参考帧预处理 | `reference.reference_frame.bgr` | `ref_kp_`, `ref_desc_` | `downscale_factor` | 可选缩放后提取ORB特征，关键点坐标反向缩放 |
| 4 | 当前帧预处理 | `frame.bgr` | `cur_kp`, `cur_desc` | `downscale_factor` | 同上 |
| 5 | 特征匹配 | `cur_desc`, `ref_desc_` | `good_matches` | `match_ratio_threshold` | BFMatcher(KNN=2) + Lowe比例测试 |
| 6 | 匹配验证 | `good_matches.size()` | `result.valid` | `min_matches` | 若匹配点过少，返回单位变换 |
| 7 | 单应性估计 | `pts_cur`, `pts_ref` | `H` (3x3矩阵) | `ransac_reproj_threshold` | RANSAC算法计算透视变换 |
| 8 | 图像变换 | `frame.bgr`, `H` | `registered_bgr` | 无 | `warpPerspective` 透视变换 |
| 9 | 结果输出 | `H`, `registered_bgr` | `result` | 无 | 包含变换矩阵、BGR/HSV图像 |

## 关键配置参数 (`PipelineConfig::image_registrator_orb`)

| 参数 | 类型 | 作用 |
|------|------|------|
| `max_features` | int | ORB检测器最大特征点数 |
| `downscale_factor` | float | 图像缩放比例 (<1.0时启用) |
| `match_ratio_threshold` | float | Lowe比例测试阈值 |
| `min_matches` | int | 最少匹配点数量 |
| `ransac_reproj_threshold` | float | RANSAC重投影误差阈值 |

## 特殊处理
- **参考帧缓存**：仅当参考帧索引变化时重新计算特征，提升性能
- **降采样处理**：缩小图像以加速特征提取，关键点坐标等比还原
- **降级处理**：特征不足时返回单位矩阵，保证流程继续