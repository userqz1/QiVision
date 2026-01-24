# QiVision 开发进度追踪

> 最后更新: 2026-01-24 (NCCModel 框架实现)
>
> 状态图例:
> - ⬜ 未开始
> - 🟡 进行中
> - ✅ 完成
> - ⏸️ 暂停
> - ❌ 废弃

---

## 总体进度

```
Platform █████████████████░░░ 86%
Core     ████████████████████ 100%
Internal ████████████████████ 100%
Feature  ██████░░░░░░░░░░░░░░ 30%
Tests    █████████████████░░░ 87%
```

---

## Phase 0: Platform 层

| 模块 | 设计 | 实现 | 单测 | 审查 | 备注 |
|------|:----:|:----:|:----:|:----:|------|
| Memory.h | ✅ | ✅ | ✅ | ⬜ | 对齐内存分配 (64字节对齐) |
| SIMD.h | ✅ | ✅ | ✅ | ⬜ | SSE4/AVX2/AVX512/NEON 检测 |
| Thread.h | ✅ | ✅ | ✅ | ⬜ | 线程池、ParallelFor |
| Timer.h | ✅ | ✅ | ✅ | ⬜ | 高精度计时 |
| FileIO.h | ✅ | ✅ | ✅ | ⬜ | 文件操作抽象、UTF-8 支持 |
| Random.h | ✅ | ✅ | ✅ | ⬜ | 随机数（RANSAC用） |
| GPU.h | ⬜ | ⬜ | ⬜ | ⬜ | GPU 抽象（预留） |

---

## Phase 1: Core 层

| 模块 | 设计 | 实现 | 单测 | 审查 | 备注 |
|------|:----:|:----:|:----:|:----:|------|
| Types.h | ✅ | ✅ | ✅ | ⬜ | Point, Rect, Line, Circle, Segment, Ellipse, Arc, RotatedRect |
| Constants.h | ✅ | ✅ | ✅ | ⬜ | 数学常量、精度常量、工具函数 |
| Exception.h | ✅ | ✅ | ⬜ | ⬜ | 异常类层次 (未编写专门单测) |
| QImage.h | ✅ | ✅ | ✅ | ⬜ | 图像类（Domain + 元数据 + stb_image I/O） |
| QRegion.h | ✅ | ✅ | ✅ | ⬜ | RLE 区域 (int32_t 游程) |
| QContour.h | ✅ | ✅ | ✅ | ⬜ | XLD 轮廓（含层次结构、属性、变换） |
| QContourArray.h | ✅ | ✅ | ✅ | ⬜ | 轮廓数组（层次管理） |
| QMatrix.h | ✅ | ✅ | ✅ | ⬜ | 2D 仿射变换矩阵 (QHomMat2d) |

---

## Phase 2: Internal 层 - 基础数学

| 模块 | 设计 | 实现 | 单测 | 精度测试 | SIMD | 审查 | 备注 |
|------|:----:|:----:|:----:|:--------:|:----:|:----:|------|
| Gaussian.h | ✅ | ✅ | ✅ | ⬜ | - | ⬜ | 高斯核、导数核 |
| Matrix.h | ✅ | ✅ | ✅ | ⬜ | - | ✅ | 小矩阵运算 (Vec/Mat固定+动态) |
| Solver.h | ✅ | ✅ | ✅ | ⬜ | - | ⬜ | 线性方程组 LU/QR/SVD/Cholesky |
| Eigen.h | ✅ | ✅ | ✅ | ⬜ | - | ⬜ | 特征值分解 (Jacobi/QR/Power/2x2/3x3) |

---

## Phase 3: Internal 层 - 图像处理

| 模块 | 设计 | 实现 | 单测 | 精度测试 | SIMD | 审查 | 备注 |
|------|:----:|:----:|:----:|:--------:|:----:|:----:|------|
| Interpolate.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | ⬜ | 双线性/双三次插值 |
| Convolution.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | ⬜ | 可分离卷积、Domain感知 |
| Gradient.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | ⬜ | Sobel/Scharr 梯度 |
| Pyramid.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | ⬜ | 高斯/拉普拉斯/梯度金字塔 |
| Histogram.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | ⬜ | 直方图、均衡化、CLAHE |
| Threshold.h | ✅ | ✅ | ✅ | ⬜ | - | ⬜ | 全局/自适应/多级阈值 |

---

## Phase 4: Internal 层 - 边缘检测

| 模块 | 设计 | 实现 | 单测 | 精度测试 | 审查 | 备注 |
|------|:----:|:----:|:----:|:--------:|:----:|------|
| Profiler.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 1D 投影采样 |
| Edge1D.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 1D 边缘检测（Caliper核心） |
| NonMaxSuppression.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 1D/2D 非极大值抑制 |
| Hessian.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | Hessian 矩阵计算、特征值分解 |
| Steger.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | Steger 亚像素边缘 |
| EdgeLinking.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 边缘点连接 |
| Canny.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | Canny 边缘检测（含亚像素精化、自动阈值） |

---

## Phase 5: Internal 层 - 几何运算

| 模块 | 设计 | 实现 | 单测 | 精度测试 | 审查 | 备注 |
|------|:----:|:----:|:----:|:--------:|:----:|------|
| Geometry2d.h | ✅ | ✅ | ✅ | - | ✅ | 几何基元操作 (规范化/变换/属性/采样/构造) |
| Distance.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 距离计算 (Point-Line/Circle/Ellipse/Arc/Segment/Contour) |
| Intersection.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 交点计算 (Line-Line/Segment/Circle/Ellipse/Arc/RotatedRect) |
| GeomRelation.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 几何关系 (包含/相交/平行/垂直/共线) |
| GeomConstruct.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 几何构造 (垂线/切线/外接圆/内切圆/凸包/最小包围圆) |
| SubPixel.h | ✅ | ✅ | ✅ | ✅ | ✅ | 亚像素精化 (1D/2D/Edge/Match/Angle) - 精度待优化 |
| Fitting.h | ✅ | ✅ | ✅ | ✅ | ✅ | 直线/圆/椭圆/RANSAC (已知问题: 旋转椭圆拟合) |
| AffineTransform.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 仿射变换 |
| Homography.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 单应性变换 (DLT+RANSAC, WarpPerspective, LM精化) |
| Hough.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 霍夫变换（直线/圆） |

---

## Phase 5.5: Internal 层 - 轮廓操作

| 模块 | 设计 | 实现 | 单测 | 精度测试 | 审查 | 备注 |
|------|:----:|:----:|:----:|:--------:|:----:|------|
| ContourProcess.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 平滑/简化/重采样 |
| ContourAnalysis.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 长度/面积/曲率/矩/形状描述符/凸性 |
| ContourConvert.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 轮廓↔区域转换 |
| ContourSelect.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 按属性筛选轮廓 |
| ContourSegment.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 轮廓分割为线段/圆弧 |

---

## Phase 6: Internal 层 - 区域处理与形态学

| 模块 | 设计 | 实现 | 单测 | 精度测试 | 审查 | 备注 |
|------|:----:|:----:|:----:|:--------:|:----:|------|
| RLEOps.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | RLE 编解码、集合运算、阈值、边界、填充、连通域 |
| StructElement.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 结构元素 (矩形/椭圆/十字/菱形/线/八边形/自定义) |
| MorphBinary.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 二值形态学 (膨胀/腐蚀/开/闭/梯度/TopHat/Hit-or-Miss/Thin/Skeleton/Geodesic) |
| MorphGray.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 灰度形态学 (膨胀/腐蚀/开/闭/梯度/TopHat/BlackHat/重构/背景校正) |
| ConnectedComponent.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 连通域标记 (图像+RLE两种实现, 统计/过滤/合并/孔洞检测) |
| DistanceTransform.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 距离变换 (L1/L2/LInf/Chamfer, 区域签名距离, Voronoi, 骨架) |
| RegionFeatures.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 区域特征 (面积/周长/圆度/矩/椭圆/凸包/最小包围圆) |

---

## Phase 7: Feature 层 - Measure

| 模块 | 设计 | 实现 | 单测 | 精度测试 | 审查 | 备注 |
|------|:----:|:----:|:----:|:--------:|:----:|------|
| MeasureTypes.h | ✅ | ✅ | - | - | ✅ | 参数和结果结构体 |
| MeasureHandle.h | ✅ | ✅ | ✅ | - | ✅ | 矩形/弧形/同心圆句柄 |
| Caliper.h | ✅ | ✅ | ✅ | ✅ | ✅ | 卡尺测量 |
| CaliperArray.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 多卡尺阵列 (沿线/弧/圆/轮廓) |

---

## Phase 8: Feature 层 - Matching

> 详细设计见: docs/design/Matching_Module_Design.md

| 模块 | 设计 | 实现 | 单测 | 精度测试 | 审查 | 备注 |
|------|:----:|:----:|:----:|:--------:|:----:|------|
| MatchTypes.h | ✅ | ✅ | - | - | ⬜ | 参数和结果结构体 |
| ShapeModel.h | ✅ | ✅ | ⬜ | ⬜ | ⬜ | 形状匹配（P0，梯度方向特征） |
| NCCModel.h | ✅ | 🟡 | ⬜ | ⬜ | ⬜ | NCC 匹配（P1，归一化互相关）- 框架已完成 |
| ComponentModel.h | ✅ | ⬜ | ⬜ | ⬜ | ⬜ | 组件匹配（P1，多部件关系约束） |
| DeformableModel.h | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | 变形匹配（P2） |
| Internal/AnglePyramid.h | ✅ | ✅ | ⬜ | ⬜ | ⬜ | 角度预计算模型（新增依赖） |
| Internal/IntegralImage.h | ✅ | ✅ | ⬜ | ⬜ | ⬜ | 积分图（NCCModel依赖） |

---

## Phase 9: Feature 层 - Metrology

| 模块 | 设计 | 实现 | 单测 | 精度测试 | 审查 | 备注 |
|------|:----:|:----:|:----:|:--------:|:----:|------|
| Metrology.h | ✅ | ✅ | ✅ | ⬜ | ⬜ | 计量模型框架 (合并为单文件) |

**说明**: Metrology 模块已整合到单个头文件，包含:
- MetrologyMeasureParams: 测量参数
- MetrologyLineResult/CircleResult/EllipseResult/Rectangle2Result: 结果结构体
- MetrologyObjectLine/Circle/Ellipse/Rectangle2: 测量对象类
- MetrologyModel: 组合测量模型

---

## Phase 10+: Feature 层 - 其他模块

| 模块 | 设计 | 实现 | 单测 | 精度测试 | 审查 | 优先级 | 备注 |
|------|:----:|:----:|:----:|:--------:|:----:|:------:|------|
| **IO/ImageIO.h** | ✅ | ✅ | ⬜ | - | ⬜ | **P0** | 图像读写 (PNG/JPEG/BMP/RAW) |
| **Color/ColorConvert.h** | ✅ | ✅ | ⬜ | ⬜ | ⬜ | **P1** | 颜色转换 (RGB/HSV/Lab/YCrCb) |
| **Filter/Filter.h** | ✅ | ✅ | ⬜ | ⬜ | ⬜ | **P1** | 滤波+增强 (Gauss/Median/Sobel/CLAHE/HistogramEq) |
| **Segment/Segment.h** | ✅ | ✅ | ⬜ | ⬜ | ⬜ | **P1** | 图像分割 (Threshold/Otsu/Adaptive/DynThreshold) |
| **Display/Display.h** | ✅ | ✅ | ⬜ | - | ⬜ | **P0** | 图像显示与绘制 (Halcon 风格 API) |
| **GUI/Window.h** | ✅ | ✅ | ⬜ | - | ⬜ | **P0** | 窗口调试 (Win32/X11, macOS/Android stub, AutoResize) |
| **Blob/Blob.h** | ✅ | ✅ | ⬜ | ⬜ | ⬜ | **P0** | Blob 分析 (Connection, SelectShape, InnerCircle, FillUp, CountHoles等) |
| Edge/* | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | P1 | 2D 边缘检测 |
| Transform/* | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | P1 | 几何变换 |
| Morphology/* | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | P1 | 形态学 |
| **OCR/*** | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | **P1** | 字符识别/验证 |
| **Barcode/*** | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | **P1** | 一维码/二维码 |
| **Defect/*** | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | **P1** | 缺陷检测 |
| **Texture/*** | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | **P2** | 纹理分析 |
| **Calib/*** | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | **P2** | 相机标定 |

---

## Phase 11: Feature 层 - Calib 标定与坐标转换

> 详细设计规范见: `.claude/docs/Calibration_CoordinateSystem_Rules.md`

### 核心数据结构

| 模块 | 设计 | 实现 | 单测 | 审查 | 备注 |
|------|:----:|:----:|:----:|:----:|------|
| QPose.h | ⬜ | ⬜ | ⬜ | ⬜ | 6DOF 位姿，欧拉角 ZYX |
| QHomMat2d.h | ⬜ | ⬜ | ⬜ | ⬜ | 2D 齐次变换矩阵 |
| QHomMat3d.h | ⬜ | ⬜ | ⬜ | ⬜ | 3D 齐次变换矩阵 |
| CameraModel.h | ⬜ | ⬜ | ⬜ | ⬜ | 相机内外参 + 畸变 |

### 标定功能

| 模块 | 设计 | 实现 | 单测 | 精度测试 | 审查 | 备注 |
|------|:----:|:----:|:----:|:--------:|:----:|------|
| CalibBoard.h | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | 标定板检测 (棋盘格/圆点) |
| CameraCalib.h | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | 相机内参标定 (张正友法) |
| Undistort.h | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | 畸变校正 |
| HandEyeCalib.h | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | 手眼标定 |
| StereoCalib.h | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | 双目标定 |

### 坐标系转换

| 模块 | 设计 | 实现 | 单测 | 精度测试 | 审查 | 备注 |
|------|:----:|:----:|:----:|:--------:|:----:|------|
| CoordTransform2d.h | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | 2D 坐标转换 (图像↔世界) |
| CoordTransform3d.h | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | 3D 坐标转换 |
| MatchTransform.h | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | 模板匹配结果→世界坐标 |
| RobotTransform.h | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | 机器人坐标系转换 |

---

## 基础设施

| 项目 | 状态 | 备注 |
|------|:----:|------|
| CMakeLists.txt (根) | ✅ | 主构建配置 (C++17, SIMD选项, GoogleTest) |
| CMakeLists.txt (src) | ✅ | 源码构建 (QiVision库) |
| CMakeLists.txt (tests) | ✅ | 测试构建 (FetchContent GoogleTest) |
| third_party/stb | ✅ | stb_image + stb_image_write 集成 |
| .clang-format | ✅ | 代码格式化配置 |
| QiVision.h | ✅ | 总头文件 |
| accuracy_config.json | ⬜ | 精度测试配置 |
| benchmark_config.json | ⬜ | 性能基准配置 |

---

## 变更日志

### 2026-01-24 (NCCModel 框架实现)

- **NCCModel 模块**
  - 新增 `include/QiVision/Matching/NCCModel.h`: 公开 API 头文件
  - 新增 `src/Matching/NCCModelImpl.h`: 内部实现结构体
  - 新增 `src/Matching/NCCModel.cpp`: 公开 API 实现
  - 新增 `src/Matching/NCCModelCreate.cpp`: 模型创建实现
  - 新增 `src/Matching/NCCModelSearch.cpp`: 多级金字塔搜索
  - 新增 `src/Matching/NCCModelScore.cpp`: NCC 分数计算（使用积分图加速）

- **NCCModel API (Halcon 风格)**
  - `CreateNCCModel`: 3个重载（无ROI、Rect2i ROI、QRegion ROI）
  - `CreateScaledNCCModel`: 带缩放搜索
  - `FindNCCModel` / `FindScaledNCCModel`: 匹配搜索
  - `GetNCCModelParams` / `GetNCCModelOrigin` / `SetNCCModelOrigin` / `GetNCCModelSize`
  - `WriteNCCModel` / `ReadNCCModel` / `ClearNCCModel`
  - `DetermineNCCModelParams`: 自动参数推荐

- **实现特性**
  - 预计算旋转模板（离散角度）
  - 积分图加速区域统计
  - 多级金字塔粗到精搜索
  - 抛物线插值亚像素精化
  - 支持 use_polarity / ignore_global_polarity 模式

### 2026-01-24 (API 文档重写为 OpenCV 风格)

- **API_Reference.md 全面重写**
  - 格式改为 OpenCV 官方文档风格
  - 每个函数独立小节: 简短描述 + 函数签名 + Parameters 表格 + Returns 表格
  - 新增 Segment 模块完整文档 (之前未记录)
  - 删除冗余示例代码，保持简洁
  - 版本号更新为 0.5.0

### 2026-01-23 (API 风格统一：直接参数取代结构体)

- **API 风格重构**
  - 所有公开 API 新增直接参数版本（Halcon/OpenCV 风格）
  - 结构体版本保留用于向后兼容
  - 可选参数使用 `std::vector<int>` 键值对（参考 OpenCV imwrite）

- **ImageIO 模块**
  - `ReadImageRaw`: 新增 (filename, image, width, height, pixelType, ...) 版本
  - `WriteImage`: 新增 (image, filename, format, vector<int> params) 版本
  - 新增 `ImageWriteFlag` 枚举: QIWRITE_JPEG_QUALITY, QIWRITE_PNG_COMPRESSION, QIWRITE_TIFF_COMPRESSION

- **Metrology 模块**
  - `Add*Measure` 方法新增直接参数版本
  - 新增 `MetrologyParamFlag` 枚举用于 vector<int> 参数
  - 示例: `AddCircleMeasure(row, col, r, len1, len2, "all", "all", {METROLOGY_NUM_MEASURES, 20})`

- **CaliperArray 模块**
  - `CreateAlong*` 方法新增直接参数版本
  - 示例: `CreateAlongCircle(center, radius, caliperCount, profileLength, handleWidth)`

- **文档更新**
  - `docs/API_Reference.md`: 更新 IO、Metrology 章节，添加新 API 示例

### 2026-01-23 (Draw Region API 和 Blob 示例)

- **Draw 模块新增 Region 绘制 API**
  - `Draw::Region`: 填充绘制 QRegion
  - `Draw::RegionContour`: 绘制区域轮廓（边界像素）
  - `Draw::RegionAlpha`: 半透明填充区域
  - 支持 RGB 和灰度图像

- **新增 Blob 分析示例程序**
  - `samples/blob/blob_analysis.cpp`: Blob 分析演示
  - 功能: 阈值分割、连通组件、形状特征、区域筛选、排序
  - 可视化: 半透明填充、轮廓绘制、圆形检测、孔洞检测
  - 键盘交互: Q/A/W/S 调整阈值, P 打印特征, ESC 退出

- **文档更新**
  - `docs/API_Reference.md`: 添加 7.9 Region 绘制小节

### 2026-01-22 (缩放匹配功能)

- **FindScaledShapeModel 实现** ✅
  - 支持 [scaleMin, scaleMax] 范围搜索
  - 自动计算 scale step（约 10 个等级）
  - 跨 scale 进行 NMS 抑制重复匹配
  - 返回最佳匹配的 scale 值

- **SearchPyramid 优化**
  - 支持 params.scaleMin 参数传递（默认 1.0，向后兼容）
  - 添加 SearchPyramidScaled 包装函数

- **测试程序**
  - 新增 test_scaled_match.cpp 验证缩放匹配功能
  - 测试结果: scale=1.0 时与 FindShapeModel 结果一致

### 2026-01-21 (架构审查与修复)

- **架构问题修复**
  - **Draw 模块迁移**: Core/Draw.h → Display/Draw.h
    - Core/Draw.h 改为兼容性头文件，自动重定向到 Display/Draw.h
    - 修复层级依赖违规 (Display 现在可以依赖 Matching)
    - Color 结构体已重命名为 Scalar (避免与 Color namespace 冲突)
  - **坐标顺序统一**: 全部使用 (x, y) OpenCV 风格
    - Display.h/cpp 所有函数参数从 (row, col) 改为 (x, y)
    - Draw.h 已经是 (x, y) 风格，无需修改
  - **Agent 规则重构**: 精简为 4 个 Agent
    - algorithm-expert: 策略分析、架构设计、复杂算法、精度诊断
    - dev: 编码实现（Core, Internal, Feature, Platform）
    - code-reviewer: 代码审查、精度验证
    - git-sync: Git 同步

- **新增 Segment 模块** (Feature 层)
  - 从 Internal/Threshold.h 提升阈值功能到公开 API
  - 全局阈值: Threshold, ThresholdRange
  - 自动阈值: ThresholdOtsu, ThresholdTriangle, ThresholdAuto
  - 自适应阈值: ThresholdAdaptive (Mean/Gaussian/Sauvola/Niblack)
  - 动态阈值: DynThreshold, VarThreshold, CharThreshold
  - 阈值转区域: ThresholdToRegion, ThresholdAutoToRegion
  - 二值操作: BinaryAnd/Or/Xor/Diff/Invert

- **扩展 Filter 模块** (直方图增强)
  - 从 Internal/Histogram.h 提升增强功能到公开 API
  - HistogramEqualize - 直方图均衡化
  - ApplyCLAHE - 自适应直方图均衡
  - ContrastStretch - 对比度拉伸
  - AutoContrast - 自动对比度
  - NormalizeImage - 图像归一化
  - HistogramMatch - 直方图匹配

- **更新 QiVision.h**
  - 添加所有 Feature 层主要头文件的 include
  - 启用 QContour, QContourArray, QMatrix 的 include

### 2026-01-20 (Draw 模块 Metrology 可视化)

- **Core/Draw 模块增强**
  - **新增 MeasureRect/MeasureRects**: Halcon 风格卡尺矩形绘制
    - 修复 Phi 参数理解：Phi 是边缘方向，投影方向 = Phi + π/2
    - Length1 沿投影方向（径向），Length2 沿边缘方向（切向）
    - MeasureRects 自动连接各卡尺中心形成测量轮廓线
  - **新增 EdgePointsWeighted**: 根据权重自动着色边缘点
    - 自动检测权重类型（二值 vs 连续）
    - RANSAC/Tukey（二值）：绿色（内点）、红色（离群点）
    - Huber（连续）：绿色（≥0.8）、黄色（0.3~0.8）、红色（<0.3）
  - **改进 Line 绘制**: 粗线使用平行 Bresenham 线实现，边缘更锐利
  - **改进 Circle/Ellipse 绘制**: 参数化方法 + 线段连接，曲线更平滑
  - **新增 MetrologyLine/Circle/Ellipse/Rectangle**: 绘制测量结果
  - **新增 MetrologyModelResult**: 一键绘制完整测量模型

- **Measure/Metrology 模块**
  - 启用 `computeInlierMask = true`，所有拟合方法返回内点掩码
  - 支持离群点可视化

- **示例更新**
  - `samples/measure/circle_measure.cpp`: 使用 Draw 模块绘制卡尺和边缘点

### 2026-01-20 (Ellipse/Rectangle2 鲁棒拟合)

- **Internal/Fitting 模块扩展**
  - **新增 FitEllipseHuber/FitEllipseTukey**: 椭圆鲁棒拟合 (IRLS)
    - 使用加权 Fitzgibbon 算法
    - Huber 权重函数适合中等离群点
    - Tukey 权重函数完全拒绝极端离群点
  - **新增 FitRectangle/FitRectangleIterative**: 矩形鲁棒拟合
    - 边缘点按矩形边分割 (SegmentPointsByRectangleSide)
    - 4条线独立拟合 (Huber/Tukey)
    - 从4条线计算矩形参数 (RectangleFromLines)
    - 迭代精化直至收敛
  - **新增 RectangleFitResult 结构体**: 包含4条边的 LineFitResult

- **Measure/Metrology 模块完善**
  - **Ellipse 测量**: 使用 FitEllipseHuber 替代 FitEllipseFitzgibbon
  - **Rectangle2 测量**: 完整实现 (之前仅占位符)
    - 需要至少8个边缘点（每边2个）
    - 使用 FitRectangleIterative 迭代拟合
    - 输出包含 RMS 误差和拟合质量分数

### 2026-01-20 (Metrology 自动阈值增强)

- **Measure/Metrology 模块增强**
  - **新增自动阈值功能**:
    - 新增 `ThresholdMode` 枚举 (`Manual`, `Auto`)
    - 新增 `SetThreshold("auto")` API 支持 Halcon 风格字符串参数
    - 自动阈值算法：`threshold = max(5.0, contrast×0.2, noise×4.0)`
    - 使用 MAD (Median Absolute Deviation) 估计噪声，比标准差更鲁棒
    - 每个投影区域（profile）独立计算阈值
  - **API 变更**:
    - `MetrologyMeasureParams::SetThreshold(double)` - 手动模式
    - `MetrologyMeasureParams::SetThreshold(const std::string&)` - 支持 "auto"
    - `SetMeasureThreshold()` 标记为 deprecated
  - **移除不合适的功能**:
    - 移除 `autoDetect` 参数（Metrology 是精确测量工具，不适合做自动检测）
    - 自动检测圆应使用专门的 `HoughCircles` 等工具
  - **亚像素支持确认**:
    - `RefineEdgeSubpixel`: 三点抛物线拟合，精度 < 0.02 px
    - `RefineEdgeZeroCrossing`: 二阶导数过零点

- **示例更新**
  - `samples/measure/circle_measure.cpp`: 演示自动阈值模式
  - 新增权重可视化（绿色=内点，黄色=中等，红色=离群点）

### 2026-01-19 (ToFloat+Copy 融合优化)
- **Internal/AnglePyramid.cpp 性能优化**:
  - 融合 ToFloat + Copy 阶段为一步操作
  - 原流程: uint8 → float QImage (有 stride) → 连续 float vector
  - 新流程: uint8 → 连续 float vector (直接)
  - 消除中间 float QImage 分配（大图像约 32MB）
  - **性能提升**:
    - Small Images (640x512): 6.8ms → 5.8ms (-14.7%)
    - Large Images (2048x4001): 162.8ms → 133.0ms (-18.3%)
    - Copy 阶段: 3-18% → 0% (完全消除)
  - **精度保持**: 所有测试 100% 通过
- **文档更新**: TROUBLESHOOTING.md 记录成功优化和失败的内存对齐尝试

### 2026-01-19 (rcp+NR 快速除法优化)
- **Internal/AnglePyramid.cpp 性能优化**:
  - 新增 `fast_rcp_avx2()`: rcp_ps + Newton-Raphson 迭代，精度 ~23 位
  - 新增 `fast_div_avx2()`: 快速除法 a * rcp(b)
  - 替换 `fast_quantize_bin_avx2` 和 `atan2_avx2` 中的 `_mm256_div_ps`
  - **性能提升**:
    - Small Images (640x512): 7.2ms → 6.3ms (-12.5%)
    - Large Images (2048x4001): ~147ms → 144.4ms (-1.8%)
  - **精度保持**: 所有测试 100% 通过
- **文档更新**: TROUBLESHOOTING.md 记录成功优化

### 2026-01-17 (GUI 交互功能)
- **GUI/Window.h 交互增强**:
  - **鼠标事件类型**: `MouseButton`, `MouseEventType`, `KeyModifier`, `MouseEvent`
  - **事件回调**: `SetMouseCallback()`, `SetKeyCallback()`
  - **鼠标位置查询**: `GetMousePosition()`, `GetMouseImagePosition()`
  - **缩放平移**: `EnableZoomPan()`, `GetZoomLevel()`, `SetZoomLevel()`, `GetPanOffset()`, `SetPanOffset()`, `ResetZoom()`, `ZoomToRegion()`
  - **坐标转换**: `WindowToImage()`, `ImageToWindow()`
  - **交互式 ROI 绘制**: `DrawRectangle()`, `DrawCircle()`, `DrawLine()`, `DrawPolygon()`, `DrawPoint()`, `DrawROI()`
  - **交互方式**:
    - 滚轮缩放（以光标为中心）
    - 左键拖拽平移
    - 右键重置为 1:1
    - 'F' 键重置为适应窗口
  - X11/Win32 双平台完整实现

### 2026-01-17 (Blob 模块增强)
- **Blob/Blob.h 新增函数**:
  - `InnerCircle`: 最大内接圆（基于距离变换）
  - `ContourLength`: 区域轮廓长度（周长）
  - `CountHoles` / `EulerNumber`: 孔洞分析
  - `FillUp`: 填充孔洞
  - `GetHoles`: 获取孔洞区域列表
  - `SelectShapeStd`: 按标准差选择（剔除异常值）
  - `SelectShapeMulti`: 多特征同时选择
  - `SelectShapeConvexity` / `SelectShapeElongation`: 按凸度/延伸度选择
  - `SelectShapeProto`: 选择 N 个最大/最小区域
- **GUI/Window.h 增强**:
  - `SetAutoResize(bool, maxW, maxH)`: 自适应窗口大小
  - 修复 X11 大图像显示时细线消失问题（使用区域平均而非最近邻）
- **文档更新**:
  - `docs/API_Reference.md`: 添加 Blob 新函数文档 (6.11-6.14)
  - `PROGRESS.md`: 更新 Blob 模块状态

### 2026-01-17 (GUI 多平台支持)
- **GUI/Window.cpp 平台扩展**
  - 添加平台检测: Windows, macOS, iOS, Android, Linux
  - Windows: Win32 GDI 完整实现
  - Linux: X11 完整实现
  - macOS/iOS/Android: Stub 实现 (Cocoa/Swift/Java 层需要单独集成)
  - CMakeLists.txt 更新: 平台条件编译和消息输出

---

### 2026-01-16 及更早 (历史存档)

> 详细历史记录已存档。主要完成内容摘要：
>
> - **2026-01-15~16**: GUI/Window 模块, Display 模块, Metrology 模块, API 文档
> - **2026-01-12**: LINEMOD 算法实现与性能优化 (245ms → 60ms, 75%提升)
> - **2026-01-08~09**: ShapeModel 模块实现, AnglePyramid, OpenMP/SIMD 优化
> - **2026-01-07~08**: Hough 变换, Eigen 分解, 几何关系模块
> - **2026-01-03~06**: SubPixel, Fitting, 轮廓分析/处理, RLE 形态学
> - **2026-01-01~02**: 基础架构, Core 层数据结构

---

## 如何更新此文件

当完成某个模块的某个阶段时，更新对应的状态：

```markdown
# 示例：完成了 Gaussian.h 的设计和实现
| Gaussian.h | ✅ | ✅ | ⬜ | ⬜ | - | ⬜ | 高斯核、导数核 |

# 示例：正在实现 Steger.h
| Steger.h | ✅ | 🟡 | ⬜ | ⬜ | ⬜ | Steger 亚像素边缘 |
```

每次更新后，同时更新"最后更新"日期和"变更日志"。
