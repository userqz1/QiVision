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

## 仓库地址

| 平台 | 地址 |
|------|------|
| **GitHub** | https://github.com/userqz1/QiVision |
| **Gitee** | https://gitee.com/flyingtoad/QiVision |

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
| **Matching** | ✅ | ShapeModel (形状匹配) + NCCModel (灰度匹配)，支持旋转、缩放 |
| **Measure** | ✅ | 卡尺测量，矩形/弧形句柄，< 0.03px 精度 |
| **IO** | ✅ | 图像读写，PNG/JPEG/BMP/RAW，16位支持 |
| **Color** | ✅ | 颜色转换（HSV/Lab/YCrCb）、Bayer 去马赛克、PCA |
| **Filter** | ✅ | 高斯/中值/双边滤波、边缘检测、图像增强 |

### 计划中

| 模块 | 优先级 | 状态 |
|------|:------:|:----:|
| ComponentModel | P1 | 设计中 |
| Blob 分析 | P1 | 进行中 |
| OCR | P1 | 计划中 |
| Barcode | P1 | 计划中 |
| 相机标定 | P2 | 计划中 |

---

## 快速开始

### 环境要求

| 平台 | 编译器 | CMake | 备注 |
|------|--------|-------|------|
| **Windows** | Visual Studio 2019+ | 3.16+ | 推荐 VS2022 |
| **Linux** | GCC 9+ / Clang 10+ | 3.16+ | Ubuntu 20.04+ |
| **macOS** | Clang 10+ | 3.16+ | Xcode 12+ |

### Windows 构建指南

#### 1. 安装工具

**方法 A: 使用 Visual Studio (推荐新手)**

1. 下载 [Visual Studio 2022 Community](https://visualstudio.microsoft.com/downloads/) (免费)
2. 安装时勾选 **"使用 C++ 的桌面开发"** 工作负载
3. CMake 已内置，无需单独安装

**方法 B: 使用 CMake + 命令行**

1. 下载 [CMake](https://cmake.org/download/) (选择 Windows x64 Installer)
2. 安装时勾选 **"Add CMake to system PATH"**
3. 确保已安装 Visual Studio 或 [Build Tools for Visual Studio](https://visualstudio.microsoft.com/downloads/#build-tools-for-visual-studio-2022)

#### 2. 克隆并编译

**使用 Visual Studio:**
```powershell
# 在 PowerShell 或 CMD 中
git clone https://github.com/userqz1/QiVision.git
cd QiVision

# 打开 Visual Studio，选择 "打开本地文件夹"，选择 QiVision 目录
# VS 会自动检测 CMakeLists.txt 并配置项目
# 在工具栏选择 Release x64，然后 生成 -> 全部生成
```

**使用命令行:**
```powershell
git clone https://github.com/userqz1/QiVision.git
cd QiVision

# 配置 (生成 Visual Studio 解决方案)
cmake -B build -G "Visual Studio 17 2022" -A x64

# 编译 Release 版本
cmake --build build --config Release --parallel

# 或者使用 Ninja (更快，需要在 VS Developer Command Prompt 中运行)
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

#### 3. 运行测试

```powershell
# 单元测试
.\build\bin\Release\unit_test.exe

# 形状匹配示例 (带 GUI 显示)
.\build\bin\Release\samples\matching_shape_match.exe
```

---

### Linux 构建指南

```bash
# 安装依赖 (Ubuntu/Debian)
sudo apt update
sudo apt install build-essential cmake git

# 克隆并编译
git clone https://github.com/userqz1/QiVision.git
cd QiVision
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel

# 运行测试
./build/bin/unit_test
./build/bin/samples/matching_shape_match
```

---

### macOS 构建指南

```bash
# 安装 Xcode 命令行工具
xcode-select --install

# 安装 CMake (使用 Homebrew)
brew install cmake

# 克隆并编译
git clone https://github.com/userqz1/QiVision.git
cd QiVision
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel

# 运行测试
./build/bin/unit_test
```

---

### CMake 选项

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `QIVISION_ENABLE_SIMD` | ON | 启用 AVX2/SSE4 加速 |
| `QIVISION_ENABLE_OPENMP` | ON | 启用多线程并行 |
| `QIVISION_BUILD_TESTS` | ON | 构建单元测试 |
| `QIVISION_BUILD_SAMPLES` | ON | 构建示例程序 |

```bash
# 示例：禁用 OpenMP
cmake -B build -DCMAKE_BUILD_TYPE=Release -DQIVISION_ENABLE_OPENMP=OFF
```

---

### 常见问题

<details>
<summary><b>Q: Windows 上编译报错 "找不到编译器"</b></summary>

确保在 **"Developer Command Prompt for VS"** 或 **"Developer PowerShell for VS"** 中运行命令，而不是普通的 CMD/PowerShell。

或者指定编译器：
```powershell
cmake -B build -G "Visual Studio 17 2022" -A x64
```
</details>

<details>
<summary><b>Q: Linux 上报错 "CMake version too old"</b></summary>

```bash
# 安装最新 CMake
sudo apt remove cmake
pip3 install cmake --upgrade
# 或从官网下载: https://cmake.org/download/
```
</details>

<details>
<summary><b>Q: 运行时报错 "找不到 DLL"</b></summary>

确保运行的是 Release 目录下的可执行文件：
```powershell
.\build\bin\Release\unit_test.exe  # ✓ 正确
.\build\bin\unit_test.exe          # ✗ 错误
```
</details>

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
│   Matching: ShapeModel ✓, NCCModel ✓, ComponentModel            │
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
| Small | 640x512 | 5 | **7.6 ms** | 100% |
| Large | 2048x4001 | 11 | **147 ms** | 100% |
| Medium | 1280x1024 | 66 | **29 ms** | 100% |
| Rotated | 888x702 | 20 | **35 ms** | 100% |

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
