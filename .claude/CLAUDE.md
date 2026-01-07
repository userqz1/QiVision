# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

QiVision is an industrial machine vision algorithm library, fully self-implemented without OpenCV dependencies (only stb_image for I/O). Target: match Halcon's core functionality and accuracy. Namespace: `Qi::Vision`.

**Progress tracking: See /PROGRESS.md (项目根目录)**

## ⚠️ 进度更新规则 (强制)

**每次完成任何模块工作后，必须立即更新 PROGRESS.md：**

1. **查看状态** - 开始工作前先读 PROGRESS.md 了解当前进度
2. **更新状态** - 完成后立即更新对应模块的状态标记
3. **添加日志** - 在"变更日志"部分记录本次工作内容
4. **禁止** - 禁止通过阅读代码来判断完成状态，必须以 PROGRESS.md 为准

```
状态图例:
⬜ 未开始 | 🟡 进行中 | ✅ 完成 | ⏸️ 暂停 | ❌ 废弃
```

**示例更新流程：**
```markdown
# 完成 Types.h 的实现后
| Types.h | ✅ | ✅ | ⬜ | ⬜ | Point2d, Line2d... |

# 在变更日志添加
### 2025-XX-XX
- Types.h: 完成设计和实现
```

## Build Commands

```bash
# Configure (when CMakeLists.txt exists)
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build --parallel

# Run tests
./build/tests/unit_test
./build/tests/accuracy_test

# Run single test
./build/tests/unit_test --gtest_filter=*TestName*
```

## Technical Requirements

- **C++ Standard**: C++17
- **Compilers**: MSVC 2019+, GCC 9+, Clang 10+
- **Dependencies**: stb_image only (header-only I/O)
- **Build System**: CMake 3.16+

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│ API Layer: QImage (Domain), QRegion (RLE), QContour (XLD), QMatrix │
├─────────────────────────────────────────────────────────────┤
│ Feature Layer: Matching, Measure, Edge, Blob, Filter, Transform, │
│                OCR, Barcode, Defect, Calib, Metrology              │
├─────────────────────────────────────────────────────────────┤
│ Internal Layer (not exported):                              │
│   Math: Gaussian, Matrix, Solver, Eigen                     │
│   Image: Interpolate, Convolution, Gradient, Pyramid        │
│   Edge: Edge1D, Steger, Hessian, NMS                        │
│   Geometry: SubPixel, Fitting, Homography, Hough            │
│   Region: RLEOps, MorphKernel, ConnectedComponent           │
├─────────────────────────────────────────────────────────────┤
│ Platform Layer: Memory, Thread, SIMD, Timer, FileIO, Random │
└─────────────────────────────────────────────────────────────┘
```

**Layer Rules**:
1. Upper layers call lower layers only (never reverse)
2. No cross-layer dependencies (don't skip layers)
3. Internal layer not exported, only for internal use
4. Feature modules must reuse Internal (no duplicate implementations)

## Critical Design Rules

### Coordinate Types
| Context | Type | Reason |
|---------|------|--------|
| Pixel coordinates | `int32_t` | Support >32K resolution (line scan) |
| Subpixel coordinates | `double` | Precision |
| RLE runs | `int32_t` | High resolution compatibility |

### Core Data Structures
- **QImage**: Must support Domain + metadata, 64-byte row alignment
- **QRegion**: Must use `int32_t` for Run struct (not int16_t), runs sorted by (row, colBegin)
- **QContour**: Must support hierarchy (parent/children) for holes

### Domain Rules
- All algorithms must support Halcon-style Domain concept
- Empty domain → return empty result (don't throw)
- Full domain → optimize for fast path via `IsFullDomain()`
- Domain propagation: point ops preserve, morphology expands/contracts

### Result Handling
- No results found → return empty vector (don't throw)
- Invalid input → throw exception
- Multi-results sorted by score descending by default

### Border Handling Defaults
| Algorithm | Default |
|-----------|---------|
| Filters (Gauss/Mean/Median) | Reflect101 |
| Gradients (Sobel/Scharr) | Reflect101 |
| Morphology | Replicate |
| Affine transforms | Constant(0) |

### Interpolation for Subpixel Accuracy
| Use Case | Recommended |
|----------|-------------|
| Subpixel edge/measurement | Bicubic |
| Affine/rotation | Bilinear |
| Pyramid downsampling | Area or Bilinear |
| Mask transforms | Nearest |

### Precision Specs (Standard Conditions: contrast≥50, noise σ≤5)
| Module | Metric | Requirement |
|--------|--------|-------------|
| Edge1D | Position | <0.02px (1σ) |
| Caliper | Position/Width | <0.03px / <0.05px (1σ) |
| ShapeModel | Position/Angle | <0.05px / <0.05° (1σ) |
| CircleFit | Center/Radius | <0.02px (1σ) |
| LineFit | Angle | <0.005° (1σ) |

## Naming Conventions

| Type | Convention | Example |
|------|------------|---------|
| Namespace | PascalCase | `Qi::Vision::Matching` |
| Class | PascalCase, Q prefix for core | `QImage`, `ShapeModel` |
| Function/Method | PascalCase | `FindShapes()`, `CreateModel()` |
| Variable | camelCase | `imageWidth`, `modelPoints` |
| Private member | trailing underscore | `data_`, `threshold_` |
| Constant | UPPER_SNAKE_CASE | `MAX_PYRAMID_LEVELS` |
| Enum value | PascalCase | `EdgePolarity::Positive` |
| File name | PascalCase | `QRegion.h`, `ShapeModel.cpp` |

### Header Include Order
```cpp
#pragma once

// 1. Project headers
#include <QiVision/Core/Types.h>

// 2. Third-party libraries

// 3. C++ standard library
#include <memory>
#include <vector>

// 4. C standard library
#include <cmath>
```

## Namespaces

```cpp
namespace Qi::Vision { }              // Top-level API
namespace Qi::Vision::Internal { }    // Internal (not exported)
namespace Qi::Vision::Platform { }    // Platform abstraction
namespace Qi::Vision::Matching { }    // Template matching
namespace Qi::Vision::Measure { }     // Measurement
namespace Qi::Vision::Calib { }       // Calibration
// ... and others per Feature module
```

## Git Workflow

### Branch Naming
```
feature/<module>-<brief>   # Feature development
fix/<issue>-<brief>        # Bug fixes
internal/<module>          # Internal module development
refactor/<scope>           # Refactoring
```

### Commit Format
```
<type>(<scope>): <subject>

type: feat, fix, refactor, test, docs, perf, chore
scope: Core, Internal, Measure, Matching, Platform, etc.

Examples:
feat(Measure): implement arc caliper
fix(ShapeModel): correct angle interpolation
perf(Internal): add AVX2 for bilinear interpolation
```

## Agent Usage

Use specialized agents via Task tool:

| Agent | Use For |
|-------|---------|
| vision-architect | Design modules, plan implementations |
| core-dev | Implement QImage, QRegion, QContour, QMatrix |
| internal-dev | Implement Steger, Hessian, Edge1D, Interpolate, Gradient |
| feature-dev | Implement Measure, Matching, Blob, OCR modules |
| platform-dev | Implement Memory, SIMD, Thread |
| test-generator | Generate synthetic test images with ground truth |
| accuracy-tester | Write precision tests, compare with Halcon |
| unit-tester | Write functional unit tests |
| code-reviewer | Verify code follows CLAUDE.md rules |
| test-runner | Run tests, collect results |
| **algorithm-expert** | **[Opus]** Complex algorithm design, precision issues, math derivation |
| **git-sync** | Commit and push to GitHub after completing modules |

**Special Agents**:
- `algorithm-expert`: Uses **Opus model**. Only invoke for:
  - Complex algorithm design (Steger, shape matching, camera calibration)
  - Precision issues that are hard to diagnose
  - Mathematical derivation and numerical stability analysis
  - This agent is **read-only** (provides analysis and recommendations, does not modify code)

- `git-sync`: Auto-sync to GitHub. Invoke when:
  - A module implementation is complete (header + source + tests passing)
  - Design document is created
  - Significant work is done (≥3 files or ≥200 lines changed)
  - Work session ends

**Module Implementation Command**: `/project:implement-module <Path>` (e.g., `Internal/Steger`)

## Key Internal Dependencies

| Feature Module | Required Internal Modules |
|----------------|---------------------------|
| Matching/ShapeModel | Gradient, Pyramid, SubPixel, Interpolate, NMS |
| Measure/Caliper | Edge1D, Profiler, SubPixel, Fitting, Interpolate |
| Edge/SubPixelEdge | Hessian, Steger, EdgeLinking, Gradient |
| Blob | RLEOps, ConnectedComponent, MorphKernel |
| Calib | Fitting, Homography, SubPixel, Matrix, Solver, Eigen |

## Key Files

- **Progress tracking**: `PROGRESS.md`
- **Calibration & coordinates**: `docs/Calibration_CoordinateSystem_Rules.md`
- **Troubleshooting**: `TROUBLESHOOTING.md`
- **Accuracy test config**: `accuracy_config.json`

## Core Algorithm Papers

1. Steger - "An Unbiased Detector of Curvilinear Structures" (1998)
2. Ulrich, Steger - "Robust Template Matching" (2003)
3. Zhang - "A Flexible New Technique for Camera Calibration" (2000)
4. Fitzgibbon - "Direct Least Square Fitting of Ellipses" (1999)
