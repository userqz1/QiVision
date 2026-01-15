<p align="center">
  <h1 align="center">🔬 QiVision</h1>
  <p align="center">
    <strong>Industrial Machine Vision Library - Zero Dependencies, Sub-pixel Precision, Halcon Compatible</strong>
  </p>
</p>

<p align="center">
    English | <a href="./README.md">简体中文</a>
</p>

<p align="center">
    <img src="https://img.shields.io/badge/C++-17-blue.svg" alt="C++17">
    <img src="https://img.shields.io/badge/License-MIT-green.svg" alt="MIT License">
    <img src="https://img.shields.io/badge/Platform-Windows%20|%20Linux-lightgrey.svg" alt="Platform">
    <img src="https://img.shields.io/badge/SIMD-AVX2%20|%20SSE4-orange.svg" alt="SIMD">
    <img src="https://img.shields.io/badge/Dependencies-stb__image%20only-brightgreen.svg" alt="Dependencies">
</p>

---

## 📋 Introduction

**QiVision** is an industrial machine vision library built from scratch in C++17, designed to match Halcon's core functionality and precision.

### ✨ Key Features

| Feature | Description |
|---------|-------------|
| 🚀 **Zero Dependencies** | Only uses stb_image for file I/O, no OpenCV required |
| 🎯 **Sub-pixel Precision** | Edge detection < 0.02px, Shape matching < 0.05px |
| 🔧 **Halcon Compatible** | Domain concept, XLD contours, RLE region encoding |
| ⚡ **SIMD Optimized** | AVX2/SSE4 instruction set acceleration |
| 📐 **Modern C++17** | RAII design, clean API interface |

---

## 📊 Development Progress

```
Platform ████████████████░░░░ 86%   (Memory, SIMD, Thread, Timer, FileIO, Random)
Core     ████████████████████ 100%  (QImage, QRegion, QContour, QMatrix)
Internal ████████████████████ 100%  (Gradient, Pyramid, Fitting, Steger, Hessian...)
Feature  ██░░░░░░░░░░░░░░░░░░ 10%   (ShapeModel ✓, Caliper ✓)
Tests    █████████████████░░░ 87%   (2616/2626 passed)
```

### 🎯 Completed Modules

| Module | Status | Performance | Description |
|--------|:------:|-------------|-------------|
| **ShapeModel** | ✅ Done | 640x512: **9.5ms**<br>2048x4001: **205ms** | Shape template matching, 0-360° rotation |
| **Caliper** | ✅ Done | < 0.03px precision | Caliper measurement with rectangle/arc handles |
| **CaliperArray** | ✅ Done | - | Multi-caliper array measurement |

### 📋 In Development / Planned

| Module | Priority | Status |
|--------|:--------:|:------:|
| NCCModel | P1 | 🟡 Designing |
| ComponentModel | P1 | 🟡 Designing |
| Blob Analysis | P1 | ⬜ Planned |
| OCR | P1 | ⬜ Planned |
| Barcode | P1 | ⬜ Planned |
| Camera Calibration | P2 | ⬜ Planned |

> 📄 **Detailed Progress**: See [PROGRESS.md](PROGRESS.md) for full module status

---

## 🚀 Quick Start

### Requirements

- **Compiler**: GCC 9+, Clang 10+, MSVC 2019+
- **Build Tool**: CMake 3.16+
- **C++ Standard**: C++17

### Build

```bash
git clone https://github.com/userqz1/QiVision.git
cd QiVision
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

### Run Tests

```bash
# Unit tests
./build/bin/unit_test

# Shape matching sample
./build/bin/samples/08_shape_match_large
```

---

## 💡 Usage Examples

### Shape Template Matching

```cpp
#include <QiVision/QiVision.h>
#include <QiVision/Matching/ShapeModel.h>

using namespace Qi::Vision;
using namespace Qi::Vision::Matching;

int main() {
    // Load template and search images
    QImage templateImg = QImage::FromFile("template.png");
    QImage searchImg = QImage::FromFile("search.png");

    // Create model
    ModelParams params;
    params.angleStart = 0;
    params.angleExtent = RAD(360);  // Full rotation support
    params.numLevels = 4;

    ShapeModel model;
    Rect2i roi(100, 100, 50, 50);  // Template region
    model.Create(templateImg, roi, params);

    // Search for matches
    SearchParams searchParams;
    searchParams.minScore = 0.8;
    searchParams.maxMatches = 10;

    auto results = model.Find(searchImg, searchParams);

    for (const auto& match : results) {
        printf("Match: (%.1f, %.1f) Score=%.3f Angle=%.1f°\n",
               match.x, match.y, match.score, DEG(match.angle));
    }

    return 0;
}
```

### Caliper Measurement

```cpp
#include <QiVision/QiVision.h>
#include <QiVision/Measure/Caliper.h>

using namespace Qi::Vision;
using namespace Qi::Vision::Measure;

int main() {
    QImage image = QImage::FromFile("edge_image.png");

    // Create rectangle caliper handle
    MeasureHandle handle = MeasureHandle::Rectangle(
        Point2d{320, 240},  // Center
        100,                // Length
        30,                 // Width
        0                   // Angle
    );

    // Measurement parameters
    CaliperParams params;
    params.transition = EdgeTransition::Positive;
    params.selectType = EdgeSelect::First;

    // Execute measurement
    auto result = MeasurePos(image, handle, params);

    if (result.edgeFound) {
        printf("Edge at: (%.3f, %.3f)\n", result.edgeX, result.edgeY);
    }

    return 0;
}
```

---

## 📦 Integration

**CMake FetchContent:**

```cmake
include(FetchContent)
FetchContent_Declare(QiVision
    GIT_REPOSITORY https://github.com/userqz1/QiVision.git
    GIT_TAG main)
FetchContent_MakeAvailable(QiVision)
target_link_libraries(your_app PRIVATE QiVision)
```

**As Subdirectory:**

```cmake
add_subdirectory(QiVision)
target_link_libraries(your_app PRIVATE QiVision)
```

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│ API Layer                                                        │
│   QImage (Domain), QRegion (RLE), QContour (XLD), QMatrix       │
├─────────────────────────────────────────────────────────────────┤
│ Feature Layer                                                    │
│   Matching: ShapeModel ✓, NCCModel, ComponentModel              │
│   Measure:  Caliper ✓, CaliperArray ✓, Metrology                │
│   Analysis: Blob, OCR, Barcode, Defect                          │
├─────────────────────────────────────────────────────────────────┤
│ Internal Layer (not exported)                                    │
│   Math:     Gaussian, Matrix, Solver, Eigen                     │
│   Image:    Interpolate, Convolution, Gradient, Pyramid         │
│   Edge:     Edge1D, Steger, Hessian, NMS, Canny                 │
│   Geometry: SubPixel, Fitting, Homography, Hough                │
│   Region:   RLEOps, Morphology, ConnectedComponent              │
├─────────────────────────────────────────────────────────────────┤
│ Platform Layer                                                   │
│   Memory (64B aligned), SIMD, Thread, Timer, FileIO, Random     │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📈 Performance

### Shape Matching Performance (ShapeModel)

| Test Set | Image Size | Images | Avg Time | Match Rate |
|----------|------------|--------|----------|------------|
| Small | 640x512 | 5 | **9.5 ms** | 100% |
| Large | 2048x4001 | 11 | **204.8 ms** | 100% |
| Medium | 1280x1024 | 66 | **49.0 ms** | 100% |
| Rotated | 888x702 | 20 | **34.4 ms** | 100% |

### Precision Targets

| Module | Metric | Target |
|--------|--------|--------|
| Edge1D | Edge position | < 0.02 px |
| Caliper | Position/Width | < 0.03 px / < 0.05 px |
| ShapeModel | Position/Angle | < 0.05 px / < 0.05° |
| CircleFit | Center/Radius | < 0.02 px |
| LineFit | Angle | < 0.005° |

---

## 📚 Documentation

- [PROGRESS.md](PROGRESS.md) - Development progress
- [samples/](samples/) - Example programs
- [CLAUDE.md](.claude/CLAUDE.md) - Development guidelines

---

## 📄 License

[MIT License](LICENSE)

---

<p align="center">
  <i>QiVision - Making Machine Vision Simpler</i>
</p>
