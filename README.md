<p align="center">
  <h1 align="center">QiVision</h1>
  <p align="center">
    <strong>工业级机器视觉算法库 - 零依赖、亚像素精度、Halcon 兼容</strong>
  </p>
</p>

<p align="center">
    <a href="./README_EN.md">English</a> | 简体中文
</p>

<p align="center">
    <img src="https://img.shields.io/badge/C++-17-blue.svg" alt="C++17">
    <img src="https://img.shields.io/badge/License-MIT-green.svg" alt="MIT License">
    <img src="https://img.shields.io/badge/Platform-Windows%20|%20Linux-lightgrey.svg" alt="Platform">
    <img src="https://img.shields.io/badge/SIMD-AVX2%20|%20SSE4-orange.svg" alt="SIMD">
    <img src="https://img.shields.io/badge/Dependencies-stb__image%20only-brightgreen.svg" alt="Dependencies">
</p>

---

## 项目简介

**QiVision** 是一个从零实现的工业机器视觉算法库，目标是达到 Halcon 的核心功能和精度水平。

### 核心特性

| 特性 | 描述 |
|------|------|
| **零依赖** | 仅使用 stb_image 进行图像读写，无 OpenCV 依赖 |
| **亚像素精度** | 边缘检测 < 0.02px，形状匹配 < 0.05px |
| **Halcon 兼容** | Domain 概念、XLD 轮廓、RLE 区域编码、Halcon 风格 API |
| **SIMD 优化** | 支持 AVX2/SSE4 指令集加速 |
| **现代 C++17** | RAII 设计、干净的 API 接口 |

---

## 文档

| 文档 | 描述 |
|------|------|
| [API 参考手册](docs/API_Reference.md) | 完整的公开 API 文档，包含 ~120 个函数 |
| [开发进度](PROGRESS.md) | 详细的模块开发状态 |
| [示例程序](samples/) | 各功能模块的使用示例 |

---

## 开发进度

```
Platform ████████████████░░░░ 86%   (Memory, SIMD, Thread, Timer, FileIO, Random)
Core     ████████████████████ 100%  (QImage, QRegion, QContour, QMatrix)
Internal ████████████████████ 100%  (Gradient, Pyramid, Fitting, Steger, Hessian...)
Feature  █████░░░░░░░░░░░░░░░ 25%   (Matching, Measure, IO, Color, Filter)
Tests    █████████████████░░░ 87%   (2616/2626 通过)
```

### 已完成模块

| 模块 | 状态 | 说明 |
|------|:----:|------|
| **Matching** | ✅ | 形状模板匹配，支持旋转 0-360°，缩放 |
| **Measure** | ✅ | 卡尺测量，矩形/弧形句柄，< 0.03px 精度 |
| **IO** | ✅ | 图像读写，PNG/JPEG/BMP/RAW，16位支持 |
| **Color** | ✅ | 颜色转换（HSV/Lab/YCrCb）、Bayer 去马赛克、PCA |
| **Filter** | ✅ | 高斯/中值/双边滤波、边缘检测、图像增强 |

### 计划中

| 模块 | 优先级 | 状态 |
|------|:------:|:----:|
| NCCModel | P1 | 设计中 |
| ComponentModel | P1 | 设计中 |
| Blob 分析 | P1 | 计划中 |
| OCR | P1 | 计划中 |
| Barcode | P1 | 计划中 |
| 相机标定 | P2 | 计划中 |

---

## 快速开始

### 环境要求

- **编译器**: GCC 9+, Clang 10+, MSVC 2019+
- **构建工具**: CMake 3.16+
- **C++ 标准**: C++17

### 编译

```bash
git clone https://github.com/userqz1/QiVision.git
cd QiVision
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

### 运行测试

```bash
# 单元测试
./build/bin/unit_test

# 形状匹配示例
./build/bin/samples/08_shape_match_large
```

---

## 使用示例

### 形状模板匹配

```cpp
#include <QiVision/Matching/ShapeModel.h>
#include <QiVision/IO/ImageIO.h>

using namespace Qi::Vision;
using namespace Qi::Vision::Matching;

int main() {
    // 加载图像
    QImage templ = IO::ReadImage("template.png");
    QImage search = IO::ReadImage("search.png");

    // 创建模型 (Halcon 风格 API)
    ShapeModel model = CreateShapeModel(
        templ,
        4,                      // 金字塔层数
        0.0, RAD(360), 0.0,     // 角度范围
        "auto",                 // 优化
        "use_polarity",         // 度量
        "auto", 10.0            // 对比度
    );

    // 搜索匹配
    std::vector<double> rows, cols, angles, scores;
    FindShapeModel(
        search, model,
        0.0, RAD(360),          // 搜索角度
        0.7, 0, 0.5,            // minScore, numMatches, maxOverlap
        "least_squares", 0, 0.9,
        rows, cols, angles, scores
    );

    for (size_t i = 0; i < rows.size(); ++i) {
        printf("Match %zu: (%.1f, %.1f) Angle=%.1f° Score=%.2f\n",
               i, cols[i], rows[i], DEG(angles[i]), scores[i]);
    }
    return 0;
}
```

### 卡尺测量

```cpp
#include <QiVision/Measure/Caliper.h>
#include <QiVision/IO/ImageIO.h>

using namespace Qi::Vision;
using namespace Qi::Vision::Measure;

int main() {
    QImage image = IO::ReadImage("edge_image.png");

    // 创建矩形测量句柄
    MeasureRectangle2 rect;
    rect.row = 240; rect.col = 320;
    rect.phi = 0.0;
    rect.length1 = 50; rect.length2 = 5;

    // 测量边缘
    MeasureParams params;
    params.sigma = 1.0;
    params.threshold = 30.0;
    params.transition = EdgeTransition::All;

    auto edges = MeasurePos(image, rect, params);
    for (const auto& e : edges) {
        printf("Edge: (%.3f, %.3f) amp=%.1f\n", e.col, e.row, e.amplitude);
    }
    return 0;
}
```

### 颜色空间转换

```cpp
#include <QiVision/Color/ColorConvert.h>
#include <QiVision/IO/ImageIO.h>

using namespace Qi::Vision;

int main() {
    QImage rgb = IO::ReadImage("photo.png");

    // RGB -> HSV
    QImage hsv = Color::TransFromRgb(rgb, "hsv");

    // 分解通道
    QImage h, s, v;
    Color::Decompose3(hsv, h, s, v);

    // 调整饱和度
    QImage saturated = Color::AdjustSaturation(rgb, 1.5);

    // Bayer 去马赛克
    QImage raw = IO::ReadImage("camera_raw.pgm");
    QImage debayered = Color::CfaToRgb(raw, "bayer_rg");

    return 0;
}
```

### 图像滤波

```cpp
#include <QiVision/Filter/Filter.h>
#include <QiVision/IO/ImageIO.h>

using namespace Qi::Vision;

int main() {
    QImage image = IO::ReadImage("noisy.png");

    // 高斯平滑
    QImage smooth = Filter::GaussFilter(image, 1.5);

    // 边缘检测
    QImage edges = Filter::SobelAmp(image, "sum_abs", 3);

    // 双边滤波（边缘保持）
    QImage bilateral = Filter::BilateralFilter(image, 5.0, 30.0);

    // 图像锐化
    QImage sharp = Filter::UnsharpMask(image, 2.0, 1.5);

    return 0;
}
```

---

## 集成到你的项目

**CMake FetchContent:**

```cmake
include(FetchContent)
FetchContent_Declare(QiVision
    GIT_REPOSITORY https://github.com/userqz1/QiVision.git
    GIT_TAG main)
FetchContent_MakeAvailable(QiVision)
target_link_libraries(your_app PRIVATE QiVision)
```

**作为子目录:**

```cmake
add_subdirectory(QiVision)
target_link_libraries(your_app PRIVATE QiVision)
```

---

## 架构设计

```
┌─────────────────────────────────────────────────────────────────┐
│ API Layer                                                        │
│   QImage (Domain), QRegion (RLE), QContour (XLD), QMatrix       │
├─────────────────────────────────────────────────────────────────┤
│ Feature Layer                                                    │
│   Matching: ShapeModel ✓, NCCModel, ComponentModel              │
│   Measure:  Caliper ✓, CaliperArray ✓, Metrology                │
│   IO:       ReadImage ✓, WriteImage ✓, Sequence ✓               │
│   Color:    ColorConvert ✓, Decompose ✓, CfaToRgb ✓             │
│   Filter:   Gauss ✓, Median ✓, Bilateral ✓, Sobel ✓             │
│   Analysis: Blob, OCR, Barcode, Defect                          │
├─────────────────────────────────────────────────────────────────┤
│ Internal Layer (不导出)                                          │
│   Math:     Gaussian, Matrix, Solver, Eigen                     │
│   Image:    Interpolate, Convolution, Gradient, Pyramid         │
│   Edge:     Edge1D, Steger, Hessian, NMS, Canny                 │
│   Geometry: SubPixel, Fitting, Homography, Hough                │
│   Region:   RLEOps, Morphology, ConnectedComponent              │
├─────────────────────────────────────────────────────────────────┤
│ Platform Layer                                                   │
│   Memory (64B对齐), SIMD, Thread, Timer, FileIO, Random         │
└─────────────────────────────────────────────────────────────────┘
```

---

## 性能指标

### 形状匹配性能 (ShapeModel)

| 测试集 | 图像尺寸 | 图像数 | 平均耗时 | 匹配率 |
|--------|----------|--------|----------|--------|
| Small | 640x512 | 5 | **9.5 ms** | 100% |
| Large | 2048x4001 | 11 | **205 ms** | 100% |
| Medium | 1280x1024 | 66 | **49 ms** | 100% |
| Rotated | 888x702 | 20 | **34 ms** | 100% |

### 精度目标

| 模块 | 指标 | 目标精度 |
|------|------|----------|
| Edge1D | 边缘位置 | < 0.02 px |
| Caliper | 位置/宽度 | < 0.03 px / < 0.05 px |
| ShapeModel | 位置/角度 | < 0.05 px / < 0.05° |
| CircleFit | 圆心/半径 | < 0.02 px |
| LineFit | 角度 | < 0.005° |

---

## 许可证

[MIT License](LICENSE)

---

<p align="center">
  <i>QiVision - 让机器视觉更简单</i>
</p>
