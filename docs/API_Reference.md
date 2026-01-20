# QiVision 公开 API 参考手册

> 版本: 0.3.0
> 最后更新: 2026-01-20
> 命名空间: `Qi::Vision`

---

## 目录

1. [Matching 模块](#1-matching-模块) - 模板匹配
2. [Measure 模块](#2-measure-模块) - 测量工具
3. [IO 模块](#3-io-模块) - 图像读写
4. [Color 模块](#4-color-模块) - 颜色转换
5. [Filter 模块](#5-filter-模块) - 图像滤波
6. [Blob 模块](#6-blob-模块) - 区域分析
7. [Display 模块](#7-display-模块) - 绘图原语
8. [GUI 模块](#8-gui-模块) - 窗口显示

---

## 1. Matching 模块

**命名空间**: `Qi::Vision::Matching`
**头文件**: `<QiVision/Matching/ShapeModel.h>`

基于形状的模板匹配，使用梯度方向特征，兼容 Halcon 的 shape model 算子。

### 1.1 ShapeModel 类

模型句柄类，存储模板数据。

```cpp
class ShapeModel {
public:
    ShapeModel();
    bool IsValid() const;  // 检查模型是否有效
};
```

---

### 1.2 CreateShapeModel

创建形状模型（仅旋转）。

```cpp
ShapeModel CreateShapeModel(
    const QImage& templateImage,    // 模板图像（灰度）
    int32_t numLevels,              // 金字塔层数（0=自动）
    double angleStart,              // 起始角度 [rad]
    double angleExtent,             // 角度范围 [rad]（0=全方向）
    double angleStep,               // 角度步长 [rad]（0=自动，约1度）
    const std::string& optimization,// 点优化："none", "auto", "point_reduction_low/medium/high"
    const std::string& metric,      // 匹配度量："use_polarity", "ignore_global_polarity"
    const std::string& contrast,    // 对比度阈值："auto" 或数值字符串
    double minContrast              // 搜索时最小对比度
);
```

**重载版本**:
```cpp
// 使用矩形 ROI
ShapeModel CreateShapeModel(
    const QImage& templateImage,
    const Rect2i& roi,              // 矩形区域
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast
);

// 使用任意形状 QRegion
ShapeModel CreateShapeModel(
    const QImage& templateImage,
    const QRegion& region,          // 任意形状区域
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast
);
```

**示例**:
```cpp
#include <QiVision/Matching/ShapeModel.h>
using namespace Qi::Vision::Matching;

QImage templ = IO::ReadImage("template.png");

// 创建模型：4层金字塔，0-360度旋转
ShapeModel model = CreateShapeModel(
    templ,
    4,                      // numLevels
    0.0,                    // angleStart
    RAD(360),               // angleExtent (使用 RAD 宏转换角度)
    0.0,                    // angleStep (自动)
    "auto",                 // optimization
    "use_polarity",         // metric
    "auto",                 // contrast
    10.0                    // minContrast
);
```

---

### 1.3 CreateScaledShapeModel

创建带缩放的形状模型。

```cpp
ShapeModel CreateScaledShapeModel(
    const QImage& templateImage,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    double scaleMin,                // 最小缩放比例
    double scaleMax,                // 最大缩放比例
    double scaleStep,               // 缩放步长（0=自动）
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast
);
```

**示例**:
```cpp
// 支持 0.8x - 1.2x 缩放
ShapeModel model = CreateScaledShapeModel(
    templ, 4,
    0.0, RAD(360), 0.0,     // 角度
    0.8, 1.2, 0.0,          // 缩放范围
    "auto", "use_polarity", "auto", 10.0
);
```

---

### 1.4 FindShapeModel

在图像中查找模板（仅旋转）。

```cpp
void FindShapeModel(
    const QImage& image,            // 搜索图像
    const ShapeModel& model,        // 模型句柄
    double angleStart,              // 搜索起始角度 [rad]
    double angleExtent,             // 搜索角度范围 [rad]
    double minScore,                // 最小匹配分数 (0-1)
    int32_t numMatches,             // 最大匹配数（0=全部）
    double maxOverlap,              // 最大重叠率 (0-1)
    const std::string& subPixel,    // 亚像素精度："none", "interpolation", "least_squares"
    int32_t numLevels,              // 使用的金字塔层数（0=全部）
    double greediness,              // 贪婪度 (0-1)，越高越快但可能漏检
    std::vector<double>& rows,      // [out] 匹配位置 Y 坐标
    std::vector<double>& cols,      // [out] 匹配位置 X 坐标
    std::vector<double>& angles,    // [out] 匹配角度 [rad]
    std::vector<double>& scores     // [out] 匹配分数
);
```

**示例**:
```cpp
QImage searchImg = IO::ReadImage("search.png");
std::vector<double> rows, cols, angles, scores;

FindShapeModel(
    searchImg, model,
    0.0, RAD(360),          // 搜索全角度
    0.7,                    // 最小分数 70%
    0,                      // 返回所有匹配
    0.5,                    // 最大重叠 50%
    "least_squares",        // 亚像素精化
    0,                      // 使用全部金字塔层
    0.9,                    // 高贪婪度
    rows, cols, angles, scores
);

for (size_t i = 0; i < rows.size(); ++i) {
    printf("Match %zu: (%.2f, %.2f) angle=%.1f° score=%.2f\n",
           i, cols[i], rows[i], DEG(angles[i]), scores[i]);
}
```

---

### 1.5 FindScaledShapeModel

查找带缩放的模板。

```cpp
void FindScaledShapeModel(
    const QImage& image,
    const ShapeModel& model,
    double angleStart,
    double angleExtent,
    double scaleMin,                // 搜索最小缩放
    double scaleMax,                // 搜索最大缩放
    double minScore,
    int32_t numMatches,
    double maxOverlap,
    const std::string& subPixel,
    int32_t numLevels,
    double greediness,
    std::vector<double>& rows,
    std::vector<double>& cols,
    std::vector<double>& angles,
    std::vector<double>& scales,    // [out] 匹配缩放比例
    std::vector<double>& scores
);
```

---

### 1.6 模型操作函数

```cpp
// 获取模型轮廓（用于可视化）
void GetShapeModelContours(
    const ShapeModel& model,
    double angle,                   // 旋转角度 [rad]
    std::vector<QContour>& contours // [out] 轮廓数组
);

// 获取模型参数
void GetShapeModelParams(
    const ShapeModel& model,
    int32_t& numLevels,             // [out] 金字塔层数
    double& angleStart,             // [out] 角度范围
    double& angleExtent,
    double& angleStep,
    double& scaleMin,               // [out] 缩放范围（仅 Scaled 模型）
    double& scaleMax,
    double& scaleStep,
    std::string& metric             // [out] 度量类型
);

// 获取/设置模型原点
void GetShapeModelOrigin(const ShapeModel& model, double& row, double& col);
void SetShapeModelOrigin(ShapeModel& model, double row, double col);

// 读写模型文件
void WriteShapeModel(const ShapeModel& model, const std::string& filename);
ShapeModel ReadShapeModel(const std::string& filename);

// 释放模型
void ClearShapeModel(ShapeModel& model);

// 自动确定参数
void DetermineShapeModelParams(
    const QImage& templateImage,
    int32_t& numLevels,             // [out] 推荐金字塔层数
    double& contrast                // [out] 推荐对比度
);

// 检查模型（调试用）
void InspectShapeModel(
    const ShapeModel& model,
    std::vector<QImage>& pyramidImages  // [out] 各层金字塔图像
);
```

---

## 2. Measure 模块

**命名空间**: `Qi::Vision::Measure`
**头文件**: `<QiVision/Measure/Caliper.h>`, `<QiVision/Measure/CaliperArray.h>`

亚像素边缘测量工具，用于精确测量边缘位置和宽度。

### 2.1 测量句柄类型

```cpp
// 矩形测量区域
struct MeasureRectangle2 {
    double row;         // 中心 Y
    double col;         // 中心 X
    double phi;         // 旋转角度 [rad]
    double length1;     // 半宽（沿法线方向）
    double length2;     // 半长（沿边缘方向）
};

// 弧形测量区域
struct MeasureArc {
    double centerRow;   // 圆心 Y
    double centerCol;   // 圆心 X
    double radius;      // 半径
    double angleStart;  // 起始角度 [rad]
    double angleExtent; // 角度范围 [rad]
    double annulusWidth;// 环宽
};

// 同心圆测量区域
struct MeasureConcentricCircles {
    double centerRow;
    double centerCol;
    double radiusInner;
    double radiusOuter;
    int32_t numCircles;
};
```

### 2.2 测量参数

```cpp
struct MeasureParams {
    // 边缘检测参数
    double sigma = 1.0;             // 高斯平滑 sigma
    double threshold = 30.0;        // 边缘阈值
    EdgeTransition transition = EdgeTransition::All;  // 边缘极性
    EdgeSelect select = EdgeSelect::All;              // 边缘选择

    // 配对参数（用于 MeasurePairs）
    double minPairDistance = 1.0;   // 最小配对距离
    double maxPairDistance = 1000.0;// 最大配对距离
};

// 边缘极性
enum class EdgeTransition {
    All,        // 所有边缘
    Positive,   // 亮到暗
    Negative    // 暗到亮
};

// 边缘选择
enum class EdgeSelect {
    All,        // 所有边缘
    First,      // 第一个
    Last        // 最后一个
};
```

### 2.3 MeasurePos

测量边缘位置。

```cpp
std::vector<EdgeResult> MeasurePos(
    const QImage& image,
    const MeasureRectangle2& handle,
    const MeasureParams& params = MeasureParams()
);

std::vector<EdgeResult> MeasurePos(
    const QImage& image,
    const MeasureArc& handle,
    const MeasureParams& params = MeasureParams()
);
```

**返回值**:
```cpp
struct EdgeResult {
    double row;         // 边缘位置 Y（亚像素）
    double col;         // 边缘位置 X（亚像素）
    double amplitude;   // 边缘强度
    double distance;    // 沿投影线的距离
};
```

**示例**:
```cpp
#include <QiVision/Measure/Caliper.h>
using namespace Qi::Vision::Measure;

// 创建矩形测量区域
MeasureRectangle2 rect;
rect.row = 100.0;
rect.col = 200.0;
rect.phi = 0.0;         // 水平
rect.length1 = 50.0;    // 半宽
rect.length2 = 5.0;     // 半长

// 测量参数
MeasureParams params;
params.sigma = 1.0;
params.threshold = 30.0;
params.transition = EdgeTransition::All;

// 执行测量
auto edges = MeasurePos(image, rect, params);

for (const auto& e : edges) {
    printf("Edge at (%.3f, %.3f), amplitude=%.1f\n", e.col, e.row, e.amplitude);
}
```

---

### 2.4 MeasurePairs

测量边缘对（宽度测量）。

```cpp
std::vector<PairResult> MeasurePairs(
    const QImage& image,
    const MeasureRectangle2& handle,
    const MeasureParams& params = MeasureParams()
);
```

**返回值**:
```cpp
struct PairResult {
    EdgeResult firstEdge;   // 第一条边缘
    EdgeResult secondEdge;  // 第二条边缘
    double width;           // 边缘对宽度
};
```

**示例**:
```cpp
// 测量宽度
MeasureParams params;
params.transition = EdgeTransition::All;
params.minPairDistance = 5.0;
params.maxPairDistance = 100.0;

auto pairs = MeasurePairs(image, rect, params);

for (const auto& p : pairs) {
    printf("Width: %.3f pixels\n", p.width);
}
```

---

### 2.5 FuzzyMeasurePos / FuzzyMeasurePairs

模糊测量，对噪声更鲁棒。

```cpp
std::vector<EdgeResult> FuzzyMeasurePos(
    const QImage& image,
    const MeasureRectangle2& handle,
    const MeasureParams& params = MeasureParams()
);

std::vector<PairResult> FuzzyMeasurePairs(
    const QImage& image,
    const MeasureRectangle2& handle,
    const MeasureParams& params = MeasureParams()
);
```

---

### 2.6 辅助函数

```cpp
// 提取灰度剖面
std::vector<double> ExtractMeasureProfile(
    const QImage& image,
    const MeasureRectangle2& handle
);

// 获取采样点数
int32_t GetNumSamples(const MeasureRectangle2& handle);
int32_t GetNumSamples(const MeasureArc& handle);

// 筛选边缘
std::vector<EdgeResult> SelectEdges(
    const std::vector<EdgeResult>& edges,
    EdgeSelect select,
    int32_t maxCount = 0
);

// 筛选配对
std::vector<PairResult> SelectPairs(
    const std::vector<PairResult>& pairs,
    PairSelect select,
    int32_t maxCount = 0
);

// 排序边缘
void SortEdges(
    std::vector<EdgeResult>& edges,
    EdgeSortBy criterion,           // ByPosition, ByAmplitude
    bool ascending = true
);

// 排序配对
void SortPairs(
    std::vector<PairResult>& pairs,
    PairSortBy criterion,           // ByWidth, ByPosition
    bool ascending = true
);
```

---

### 2.7 Metrology 模块

**头文件**: `<QiVision/Measure/Metrology.h>`

计量模型框架，用于组合测量多个几何对象。

#### 2.7.1 ThresholdMode

```cpp
enum class ThresholdMode {
    Manual,    // 使用用户指定的阈值
    Auto       // 每个投影区域独立计算自动阈值
};
```

#### 2.7.2 MetrologyMeasureParams

```cpp
struct MetrologyMeasureParams {
    int32_t numInstances = 1;           // 实例数量
    double measureLength1 = 20.0;       // 卡尺半长（沿投影方向）
    double measureLength2 = 5.0;        // 卡尺半宽（垂直方向）
    double measureSigma = 1.0;          // 高斯平滑 sigma
    double measureThreshold = 30.0;     // 边缘阈值（Manual 模式）
    ThresholdMode thresholdMode = ThresholdMode::Manual;
    EdgeTransition measureTransition = EdgeTransition::All;
    int32_t numMeasures = 10;           // 卡尺数量
    double minScore = 0.5;              // 最小分数

    // 设置阈值（数值 = Manual 模式）
    MetrologyMeasureParams& SetThreshold(double t);

    // 设置阈值（"auto" = Auto 模式）
    MetrologyMeasureParams& SetThreshold(const std::string& mode);
};
```

#### 2.7.3 MetrologyModel

```cpp
class MetrologyModel {
public:
    // 添加测量对象
    int32_t AddLineMeasure(double row1, double col1, double row2, double col2,
                           const MetrologyMeasureParams& params = {});
    int32_t AddCircleMeasure(double row, double col, double radius,
                             const MetrologyMeasureParams& params = {});
    int32_t AddArcMeasure(double row, double col, double radius,
                          double angleStart, double angleEnd,
                          const MetrologyMeasureParams& params = {});
    int32_t AddEllipseMeasure(double row, double col, double phi,
                              double ra, double rb,
                              const MetrologyMeasureParams& params = {});

    // 执行测量
    bool Apply(const QImage& image);

    // 获取结果
    MetrologyLineResult GetLineResult(int32_t index) const;
    MetrologyCircleResult GetCircleResult(int32_t index) const;
    MetrologyEllipseResult GetEllipseResult(int32_t index) const;

    // 获取测量点
    std::vector<Point2d> GetMeasuredPoints(int32_t index) const;
    std::vector<double> GetPointWeights(int32_t index) const;

    // 对齐
    void Align(double rowOffset, double colOffset, double phi);
    void ResetAlignment();
};
```

**示例**:
```cpp
#include <QiVision/Measure/Metrology.h>
using namespace Qi::Vision::Measure;

MetrologyMeasureParams params;
params.SetMeasureLength(30.0, 10.0)
      .SetNumMeasures(36)
      .SetMeasureSigma(1.5)
      .SetThreshold("auto");  // 自动阈值模式

MetrologyModel model;
int idx = model.AddCircleMeasure(500.0, 650.0, 220.0, params);

if (model.Apply(image)) {
    auto result = model.GetCircleResult(idx);
    if (result.IsValid()) {
        std::cout << "Center: (" << result.column << ", " << result.row << ")\n";
        std::cout << "Radius: " << result.radius << "\n";
        std::cout << "RMS Error: " << result.rmsError << "\n";
    }
}
```

---

## 3. IO 模块

**命名空间**: `Qi::Vision::IO`
**头文件**: `<QiVision/IO/ImageIO.h>`

图像读写，支持多种格式和位深。

### 3.1 ReadImage

读取图像文件。

```cpp
QImage ReadImage(const std::string& filename);

QImage ReadImage(
    const std::string& filename,
    ImageFormat format              // 强制指定格式
);
```

**支持格式**:
| 格式 | 扩展名 | 8位 | 16位 | Alpha |
|------|--------|:---:|:----:|:-----:|
| PNG | .png | ✓ | ✓ | ✓ |
| JPEG | .jpg/.jpeg | ✓ | - | - |
| BMP | .bmp | ✓ | - | - |
| TIFF | .tif/.tiff | ✓ | ✓ | ✓ |
| PGM/PPM | .pgm/.ppm | ✓ | ✓ | - |
| RAW | .raw/.bin | ✓ | ✓ | ✓ |

**示例**:
```cpp
#include <QiVision/IO/ImageIO.h>
using namespace Qi::Vision::IO;

QImage img = ReadImage("photo.png");
printf("Size: %d x %d, Channels: %d\n",
       img.Width(), img.Height(), img.Channels());
```

---

### 3.2 ReadImageRaw

读取 RAW 格式图像（无头信息）。

```cpp
struct RawReadParams {
    int32_t width = 0;              // 图像宽度（必填）
    int32_t height = 0;             // 图像高度（必填）
    PixelType pixelType = PixelType::UInt8;   // 像素类型
    ChannelType channelType = ChannelType::Gray; // 通道类型
    int32_t headerBytes = 0;        // 跳过的头部字节数
    bool bigEndian = false;         // 大端序（16位图像）
};

QImage ReadImageRaw(
    const std::string& filename,
    const RawReadParams& params
);
```

**示例**:
```cpp
RawReadParams params;
params.width = 1920;
params.height = 1080;
params.pixelType = PixelType::UInt16;
params.channelType = ChannelType::Gray;
params.bigEndian = true;

QImage raw = ReadImageRaw("camera.raw", params);
```

---

### 3.3 ReadImageAs / ReadImageGray

读取并转换类型。

```cpp
// 读取并转换为指定像素类型
QImage ReadImageAs(
    const std::string& filename,
    PixelType targetType            // UInt8, UInt16, Float32
);

// 读取为灰度图
QImage ReadImageGray(const std::string& filename);
```

---

### 3.4 ReadImageMetadata

读取图像元数据（不加载像素数据）。

```cpp
struct ImageMetadata {
    int32_t width = 0;
    int32_t height = 0;
    int32_t channels = 0;
    int32_t bitsPerChannel = 8;
    PixelType pixelType = PixelType::UInt8;
    ChannelType channelType = ChannelType::Gray;
};

bool ReadImageMetadata(
    const std::string& filename,
    ImageMetadata& metadata         // [out] 元数据
);
```

---

### 3.5 WriteImage

写入图像文件。

```cpp
bool WriteImage(
    const QImage& image,
    const std::string& filename
);

struct CompressionParams {
    int32_t jpegQuality = 95;       // JPEG 质量 (0-100)
    int32_t pngCompression = 6;     // PNG 压缩级别 (0-9)
};

bool WriteImage(
    const QImage& image,
    const std::string& filename,
    ImageFormat format,             // 指定格式
    const CompressionParams& params // 压缩参数
);
```

**示例**:
```cpp
// 默认保存
WriteImage(img, "output.png");

// 指定 JPEG 质量
CompressionParams params;
params.jpegQuality = 85;
WriteImage(img, "output.jpg", ImageFormat::JPEG, params);
```

---

### 3.6 WriteImageRaw

写入 RAW 格式。

```cpp
bool WriteImageRaw(
    const QImage& image,
    const std::string& filename,
    bool bigEndian = false          // 大端序（16位图像）
);
```

---

### 3.7 序列读写

```cpp
// 批量读取
std::vector<QImage> ReadSequence(
    const std::string& pattern,     // 文件名模式，如 "img_%04d.png"
    int32_t startIndex,             // 起始索引
    int32_t endIndex,               // 结束索引
    int32_t step = 1                // 步长
);

// 读取目录
std::vector<QImage> ReadDirectory(
    const std::string& directory,
    const std::vector<std::string>& extensions = {}  // 过滤扩展名
);

// 批量写入
int32_t WriteSequence(
    const std::vector<QImage>& images,
    const std::string& pattern,     // 输出文件名模式
    int32_t startIndex = 0,
    const CompressionParams& params = CompressionParams()
);
```

**示例**:
```cpp
// 读取序列 frame_0001.png ~ frame_0100.png
auto frames = ReadSequence("frames/frame_%04d.png", 1, 100);

// 读取目录中所有 PNG 和 BMP
auto images = ReadDirectory("images/", {".png", ".bmp"});

// 写入序列
WriteSequence(frames, "output/out_%03d.png", 0);
```

---

### 3.8 格式工具函数

```cpp
// 从文件名获取格式
ImageFormat GetFormatFromFilename(const std::string& filename);

// 获取格式对应的扩展名
std::string GetExtensionForFormat(ImageFormat format);

// 格式能力查询
bool FormatSupports16Bit(ImageFormat format);
bool FormatSupportsAlpha(ImageFormat format);
bool FormatIsLossless(ImageFormat format);

// 获取支持的扩展名列表
std::vector<std::string> GetSupportedExtensions();

// 检查文件是否是支持的图像格式
bool IsSupportedImageFile(const std::string& filename);
```

---

## 4. Color 模块

**命名空间**: `Qi::Vision::Color`
**头文件**: `<QiVision/Color/ColorConvert.h>`

颜色空间转换、通道操作、颜色调整。

### 4.1 颜色空间

```cpp
enum class ColorSpace {
    Gray,       // 灰度 (1 通道)
    RGB,        // 红、绿、蓝
    BGR,        // 蓝、绿、红 (OpenCV 顺序)
    RGBA,       // RGB + Alpha
    BGRA,       // BGR + Alpha
    HSV,        // 色相 [0-360]、饱和度 [0-1]、明度 [0-1]
    HSL,        // 色相、饱和度、亮度
    Lab,        // CIE L*a*b* (感知均匀)
    Luv,        // CIE L*u*v*
    XYZ,        // CIE XYZ
    YCrCb,      // 亮度 + 色度 (JPEG/MPEG)
    YUV         // 亮度 + 色度 (模拟视频)
};
```

---

### 4.2 TransFromRgb / TransToRgb

颜色空间转换（Halcon 风格）。

```cpp
// RGB 转其他颜色空间
QImage TransFromRgb(
    const QImage& image,
    ColorSpace toSpace              // 目标颜色空间
);

QImage TransFromRgb(
    const QImage& image,
    const std::string& colorSpace   // "hsv", "hsl", "lab", "yuv", "ycrcb"
);

// 其他颜色空间转 RGB
QImage TransToRgb(
    const QImage& image,
    ColorSpace fromSpace
);

QImage TransToRgb(
    const QImage& image,
    const std::string& colorSpace
);
```

**示例**:
```cpp
#include <QiVision/Color/ColorConvert.h>
using namespace Qi::Vision::Color;

QImage rgb = IO::ReadImage("photo.png");

// RGB -> HSV
QImage hsv = TransFromRgb(rgb, ColorSpace::HSV);
// 或使用字符串
QImage hsv2 = TransFromRgb(rgb, "hsv");

// HSV -> RGB
QImage back = TransToRgb(hsv, ColorSpace::HSV);
```

---

### 4.3 Rgb1ToGray / Rgb3ToGray

灰度转换。

```cpp
// RGB 图像转灰度
QImage Rgb1ToGray(
    const QImage& image,
    const std::string& method = "luminosity"
    // 方法:
    // - "luminosity": 0.299*R + 0.587*G + 0.114*B (默认)
    // - "average": (R + G + B) / 3
    // - "lightness": (max + min) / 2
    // - "bt709": 0.2126*R + 0.7152*G + 0.0722*B
    // - "max": max(R, G, B)
    // - "min": min(R, G, B)
);

// 三个单通道图像合成灰度
QImage Rgb3ToGray(
    const QImage& red,
    const QImage& green,
    const QImage& blue,
    const std::string& method = "luminosity"
);

// 灰度转 RGB（复制到三通道）
QImage GrayToRgb(const QImage& gray);
```

---

### 4.4 Decompose3 / Decompose4

通道分解。

```cpp
// 分解 3 通道图像
void Decompose3(
    const QImage& image,
    QImage& ch1,                    // [out] 第1通道 (R/H/L...)
    QImage& ch2,                    // [out] 第2通道 (G/S/a...)
    QImage& ch3                     // [out] 第3通道 (B/V/b...)
);

// 分解 4 通道图像
void Decompose4(
    const QImage& image,
    QImage& ch1,
    QImage& ch2,
    QImage& ch3,
    QImage& ch4                     // [out] Alpha 通道
);
```

**示例**:
```cpp
QImage r, g, b;
Decompose3(rgbImage, r, g, b);

// 处理单个通道
QImage processedG = Filter::GaussFilter(g, 2.0);
```

---

### 4.5 Compose3 / Compose4

通道合成。

```cpp
QImage Compose3(
    const QImage& ch1,
    const QImage& ch2,
    const QImage& ch3,
    ChannelType channelType = ChannelType::RGB
);

QImage Compose4(
    const QImage& ch1,
    const QImage& ch2,
    const QImage& ch3,
    const QImage& ch4,
    ChannelType channelType = ChannelType::RGBA
);
```

---

### 4.6 AccessChannel / SplitChannels / MergeChannels

```cpp
// 访问单个通道（返回副本）
QImage AccessChannel(
    const QImage& image,
    int32_t channelIndex            // 0-based 索引
);

// 获取通道数
int32_t CountChannels(const QImage& image);

// 分离所有通道
std::vector<QImage> SplitChannels(const QImage& image);

// 合并通道
QImage MergeChannels(
    const std::vector<QImage>& channels,
    ChannelType channelType = ChannelType::RGB
);
```

---

### 4.7 通道交换

```cpp
// RGB <-> BGR
QImage RgbToBgr(const QImage& image);
QImage BgrToRgb(const QImage& image);

// 交换两个通道
QImage SwapChannels(
    const QImage& image,
    int32_t ch1,                    // 第一个通道索引
    int32_t ch2                     // 第二个通道索引
);

// 重排通道
QImage ReorderChannels(
    const QImage& image,
    const std::vector<int32_t>& order  // 新顺序，如 {2, 1, 0} = BGR
);
```

---

### 4.8 颜色调整

```cpp
// 亮度调整
QImage AdjustBrightness(
    const QImage& image,
    double brightness               // [-255, 255]，正值增亮
);

// 对比度调整
QImage AdjustContrast(
    const QImage& image,
    double contrast                 // 1.0=不变，>1增强，<1降低
);

// 饱和度调整（彩色图像）
QImage AdjustSaturation(
    const QImage& image,
    double saturation               // 0=灰度，1.0=不变，>1更饱和
);

// 色相偏移
QImage AdjustHue(
    const QImage& image,
    double hueShift                 // [-180, 180] 度
);

// Gamma 校正
QImage AdjustGamma(
    const QImage& image,
    double gamma                    // 1.0=不变，<1增亮，>1变暗
);

// 颜色反转
QImage InvertColors(const QImage& image);
```

**示例**:
```cpp
// 增加亮度和对比度
QImage bright = AdjustBrightness(image, 30);
QImage contrast = AdjustContrast(image, 1.2);

// Gamma 校正
QImage corrected = AdjustGamma(image, 2.2);
```

---

### 4.9 白平衡

```cpp
// 自动白平衡
QImage AutoWhiteBalance(
    const QImage& image,
    const std::string& method = "gray_world"
    // 方法:
    // - "gray_world": 假设平均颜色为灰色
    // - "white_patch": 假设最亮像素为白色
);

// 手动白平衡
QImage ApplyWhiteBalance(
    const QImage& image,
    double whiteR,                  // R 通道缩放系数
    double whiteG,                  // G 通道缩放系数
    double whiteB                   // B 通道缩放系数
);
```

---

### 4.10 CreateColorTransLut / ApplyColorTransLut

LUT 加速颜色转换（批量处理时快 10x+）。

```cpp
// 创建 LUT（内存占用 ~48MB）
ColorTransLut CreateColorTransLut(
    const std::string& colorSpace,  // "hsv", "hsl", "lab", "ycrcb", "yuv"
    const std::string& transDirection = "from_rgb",  // "from_rgb" 或 "to_rgb"
    int32_t numBits = 8             // 仅支持 8 位
);

// 应用 LUT（三通道分离版本）
void ApplyColorTransLut(
    const QImage& image1,           // 输入通道 1
    const QImage& image2,           // 输入通道 2
    const QImage& image3,           // 输入通道 3
    QImage& result1,                // [out] 输出通道 1
    QImage& result2,                // [out] 输出通道 2
    QImage& result3,                // [out] 输出通道 3
    const ColorTransLut& lut
);

// 应用 LUT（多通道图像版本）
QImage ApplyColorTransLut(
    const QImage& image,            // 3 通道图像
    const ColorTransLut& lut
);

// 释放 LUT 内存
void ClearColorTransLut(ColorTransLut& lut);
```

**示例**:
```cpp
// 创建 RGB -> HSV 的 LUT
ColorTransLut lut = CreateColorTransLut("hsv", "from_rgb", 8);

// 批量处理多张图像
for (auto& img : images) {
    QImage hsv = ApplyColorTransLut(img, lut);
    // 处理 HSV 图像...
}

// 释放 LUT
ClearColorTransLut(lut);
```

---

### 4.11 CfaToRgb

Bayer 去马赛克（工业相机原始图像）。

```cpp
QImage CfaToRgb(
    const QImage& cfaImage,         // 单通道 Bayer 图像
    const std::string& cfaType = "bayer_gb",
    // Bayer 模式:
    // - "bayer_rg" / "rggb": R G / G B
    // - "bayer_gr" / "grbg": G R / B G
    // - "bayer_gb" / "gbrg": G B / R G
    // - "bayer_bg" / "bggr": B G / G R
    const std::string& interpolation = "bilinear"
    // 插值方法:
    // - "bilinear": 标准双线性插值
    // - "bilinear_dir": 方向感知双线性（减少锯齿）
);
```

**示例**:
```cpp
// 读取相机原始图像
QImage raw = IO::ReadImage("camera_raw.pgm");

// 转换为 RGB（相机使用 RGGB 模式）
QImage rgb = CfaToRgb(raw, "bayer_rg", "bilinear");
```

---

### 4.12 LinearTransColor / ApplyColorMatrix

颜色仿射变换。

```cpp
// 通用颜色变换 (m x (n+1) 矩阵)
QImage LinearTransColor(
    const QImage& image,
    const std::vector<double>& transMat,  // 行优先存储
    // 矩阵大小: numOutputChannels × (numInputChannels + 1)
    // 最后一列为偏移量
    int32_t numOutputChannels
);

// 3x3 颜色矩阵（无偏移）
QImage ApplyColorMatrix(
    const QImage& image,
    const std::vector<double>& matrix     // 9 个元素
);
```

**示例**:
```cpp
// RGB 转灰度（自定义权重）
std::vector<double> grayMat = {
    0.299, 0.587, 0.114, 0.0  // Y = 0.299*R + 0.587*G + 0.114*B
};
QImage gray = LinearTransColor(rgbImage, grayMat, 1);

// 色彩校正矩阵
std::vector<double> colorMat = {
    1.2, -0.1, -0.1,    // R' = 1.2*R - 0.1*G - 0.1*B
    -0.1, 1.2, -0.1,    // G' = ...
    -0.1, -0.1, 1.2     // B' = ...
};
QImage corrected = ApplyColorMatrix(image, colorMat);
```

---

### 4.13 PrincipalComp / GenPrincipalCompTrans

主成分分析（PCA）。

```cpp
// 计算主成分
QImage PrincipalComp(
    const QImage& image,
    int32_t numComponents = 0       // 保留的主成分数（0=全部）
);

// 生成 PCA 变换矩阵
void GenPrincipalCompTrans(
    const QImage& image,
    std::vector<double>& transMat,  // [out] 变换矩阵
    std::vector<double>& mean,      // [out] 各通道均值
    std::vector<double>& eigenvalues // [out] 特征值（方差）
);
```

**示例**:
```cpp
// 对 RGB 图像做 PCA，保留前 2 个主成分
QImage pca = PrincipalComp(rgbImage, 2);

// 获取变换矩阵
std::vector<double> mat, mean, eigenvalues;
GenPrincipalCompTrans(rgbImage, mat, mean, eigenvalues);

printf("Variance explained: %.2f%%, %.2f%%, %.2f%%\n",
       eigenvalues[0] / (eigenvalues[0] + eigenvalues[1] + eigenvalues[2]) * 100,
       eigenvalues[1] / (eigenvalues[0] + eigenvalues[1] + eigenvalues[2]) * 100,
       eigenvalues[2] / (eigenvalues[0] + eigenvalues[1] + eigenvalues[2]) * 100);
```

---

### 4.14 工具函数

```cpp
// 颜色空间名称
std::string GetColorSpaceName(ColorSpace space);
ColorSpace ParseColorSpace(const std::string& name);

// 通道数
int32_t GetChannelCount(ColorSpace space);

// 是否有 Alpha 通道
bool HasAlphaChannel(ColorSpace space);
```

---

## 5. Filter 模块

**命名空间**: `Qi::Vision::Filter`
**头文件**: `<QiVision/Filter/Filter.h>`

图像滤波：平滑、边缘检测、增强。

### 5.1 GaussFilter

高斯平滑。

```cpp
// 各向同性高斯
QImage GaussFilter(
    const QImage& image,
    double sigma                    // 标准差
);

// 各向异性高斯
QImage GaussFilter(
    const QImage& image,
    double sigmaX,                  // X 方向 sigma
    double sigmaY,                  // Y 方向 sigma
    const std::string& borderMode = "reflect"
    // 边界模式:
    // - "reflect" / "reflect101": 镜像
    // - "replicate": 复制边缘
    // - "constant": 常数（0）
    // - "wrap": 循环
);

// 固定尺寸高斯
QImage GaussImage(
    const QImage& image,
    const std::string& size         // "3x3", "5x5", "7x7", "9x9", "11x11"
);
```

**示例**:
```cpp
#include <QiVision/Filter/Filter.h>
using namespace Qi::Vision::Filter;

QImage smooth = GaussFilter(image, 1.5);
QImage smooth2 = GaussImage(image, "5x5");
```

---

### 5.2 MeanImage

均值平滑（盒式滤波）。

```cpp
QImage MeanImage(
    const QImage& image,
    int32_t width,                  // 核宽度
    int32_t height,                 // 核高度
    const std::string& borderMode = "reflect"
);

// 正方形核
QImage MeanImage(
    const QImage& image,
    int32_t size,
    const std::string& borderMode = "reflect"
);
```

---

### 5.3 MedianImage / MedianRect

中值滤波。

```cpp
QImage MedianImage(
    const QImage& image,
    const std::string& maskType,    // "circle", "square", "rhombus"
    int32_t radius,
    const std::string& marginMode = "mirrored"
);

QImage MedianRect(
    const QImage& image,
    int32_t width,
    int32_t height
);
```

---

### 5.4 BilateralFilter

双边滤波（边缘保持平滑）。

```cpp
QImage BilateralFilter(
    const QImage& image,
    double sigmaSpatial,            // 空间 sigma（距离权重）
    double sigmaIntensity           // 强度 sigma（颜色相似权重）
);

QImage BilateralFilter(
    const QImage& image,
    int32_t size,                   // 核大小
    double sigmaSpatial,
    double sigmaIntensity
);
```

**示例**:
```cpp
// 保持边缘的平滑
QImage smooth = BilateralFilter(image, 5.0, 30.0);
```

---

### 5.5 BinomialFilter

二项式滤波（高斯近似，更快）。

```cpp
QImage BinomialFilter(
    const QImage& image,
    int32_t width,
    int32_t height,
    const std::string& borderMode = "reflect"
);
```

---

### 5.6 SobelAmp / SobelDir

Sobel 梯度。

```cpp
// 梯度幅值
QImage SobelAmp(
    const QImage& image,
    const std::string& filterType = "sum_abs",
    // 类型:
    // - "sum_abs": |Gx| + |Gy|（快速）
    // - "sum_sqrt": sqrt(Gx² + Gy²)（精确）
    int32_t size = 3                // 核大小: 3, 5, 7
);

// 梯度方向
QImage SobelDir(
    const QImage& image,
    const std::string& dirType = "gradient",
    // 类型:
    // - "gradient": 边缘法线方向
    // - "tangent": 边缘切线方向
    int32_t size = 3
);
```

**示例**:
```cpp
QImage edges = SobelAmp(image, "sum_abs", 3);
QImage direction = SobelDir(image, "gradient", 3);
```

---

### 5.7 DerivateGauss

高斯导数。

```cpp
QImage DerivateGauss(
    const QImage& image,
    double sigma,
    const std::string& component    // 导数分量
    // 一阶导数:
    // - "x": dG/dx
    // - "y": dG/dy
    // 二阶导数:
    // - "xx": d²G/dx²
    // - "yy": d²G/dy²
    // - "xy": d²G/dxdy
    // 梯度幅值:
    // - "gradient": sqrt((dG/dx)² + (dG/dy)²)
);
```

**示例**:
```cpp
QImage gx = DerivateGauss(image, 1.5, "x");
QImage gyy = DerivateGauss(image, 1.5, "yy");
QImage gradient = DerivateGauss(image, 1.5, "gradient");
```

---

### 5.8 GradientMagnitude / GradientDirection

梯度计算（使用高斯导数）。

```cpp
QImage GradientMagnitude(
    const QImage& image,
    double sigma
);

QImage GradientDirection(
    const QImage& image,
    double sigma
);
```

---

### 5.9 Laplace / LaplacianOfGaussian

拉普拉斯算子。

```cpp
QImage Laplace(
    const QImage& image,
    const std::string& filterType = "3x3"
    // 类型: "3x3", "5x5", "n4", "n8"
);

QImage LaplacianOfGaussian(
    const QImage& image,
    double sigma
);
```

---

### 5.10 HighpassImage / LowpassImage

频域滤波（空间实现）。

```cpp
QImage HighpassImage(
    const QImage& image,
    int32_t width,
    int32_t height
);

QImage LowpassImage(
    const QImage& image,
    int32_t width,
    int32_t height
);
```

---

### 5.11 EmphasizeImage / UnsharpMask

图像增强/锐化。

```cpp
// 增强（锐化 + 对比度）
QImage EmphasizeImage(
    const QImage& image,
    int32_t width,
    int32_t height,
    double factor                   // 增强因子（1.0=不变，>1=更强）
);

// 非锐化掩模
QImage UnsharpMask(
    const QImage& image,
    double sigma,                   // 模糊 sigma
    double amount = 1.0,            // 锐化量
    double threshold = 0.0          // 细节阈值（0=锐化所有）
);
```

**示例**:
```cpp
QImage sharp = EmphasizeImage(image, 7, 7, 1.5);
QImage sharp2 = UnsharpMask(image, 2.0, 1.5, 5.0);
```

---

### 5.12 ShockFilter

冲击滤波（边缘增强扩散）。

```cpp
QImage ShockFilter(
    const QImage& image,
    int32_t iterations,
    double dt = 0.1                 // 时间步长
);
```

---

### 5.13 AnisoDiff

各向异性扩散（Perona-Malik）。

```cpp
QImage AnisoDiff(
    const QImage& image,
    const std::string& mode,        // "pm1" 或 "pm2"
    // - "pm1": g(x) = exp(-(x/K)²)，保留高对比度边缘
    // - "pm2": g(x) = 1/(1+(x/K)²)，保留宽区域
    double contrast,                // K 参数（边缘阈值）
    double theta,                   // 扩散系数 (0-0.25)
    int32_t iterations
);
```

**示例**:
```cpp
// 边缘保持平滑
QImage smooth = AnisoDiff(image, "pm1", 30.0, 0.25, 10);
```

---

### 5.14 ConvolImage / ConvolSeparable

自定义卷积。

```cpp
// 2D 卷积
QImage ConvolImage(
    const QImage& image,
    const std::vector<double>& kernel,  // 行优先
    int32_t kernelWidth,
    int32_t kernelHeight,
    bool normalize = false,
    const std::string& borderMode = "reflect"
);

// 可分离卷积（更快）
QImage ConvolSeparable(
    const QImage& image,
    const std::vector<double>& kernelX,
    const std::vector<double>& kernelY,
    const std::string& borderMode = "reflect"
);
```

**示例**:
```cpp
// 自定义 3x3 锐化核
std::vector<double> kernel = {
    0, -1, 0,
    -1, 5, -1,
    0, -1, 0
};
QImage sharp = ConvolImage(image, kernel, 3, 3, false);

// 可分离高斯（更快）
std::vector<double> gauss = {0.25, 0.5, 0.25};
QImage smooth = ConvolSeparable(image, gauss, gauss);
```

---

### 5.15 RankImage / MinImage / MaxImage

秩滤波。

```cpp
QImage RankImage(
    const QImage& image,
    int32_t width,
    int32_t height,
    int32_t rank                    // 0=最小，width*height-1=最大
);

QImage MinImage(const QImage& image, int32_t width, int32_t height);
QImage MaxImage(const QImage& image, int32_t width, int32_t height);
```

---

### 5.16 StdDevImage / VarianceImage / EntropyImage

纹理特征。

```cpp
// 局部标准差
QImage StdDevImage(
    const QImage& image,
    int32_t width,
    int32_t height
);

// 局部方差
QImage VarianceImage(
    const QImage& image,
    int32_t width,
    int32_t height
);

// 局部熵
QImage EntropyImage(
    const QImage& image,
    int32_t width,
    int32_t height,
    int32_t numBins = 256           // 直方图 bins
);
```

---

### 5.17 核生成工具

```cpp
// 生成高斯核
std::vector<double> GenGaussKernel(
    double sigma,
    int32_t size = 0                // 0=自动计算
);

// 生成高斯导数核
std::vector<double> GenGaussDerivKernel(
    double sigma,
    int32_t order,                  // 1=一阶导数，2=二阶导数
    int32_t size = 0
);

// 计算最优核大小
int32_t OptimalKernelSize(double sigma);
```

---

## 6. Blob 模块

**命名空间**: `Qi::Vision::Blob`
**头文件**: `<QiVision/Blob/Blob.h>`

区域分析和形状选择，兼容 Halcon 的 blob analysis 算子。

### 6.1 类型定义

```cpp
// 形状特征
enum class ShapeFeature {
    Area,               // 区域面积（像素）
    Row,                // 质心行坐标
    Column,             // 质心列坐标
    Width,              // 边界框宽度
    Height,             // 边界框高度
    Circularity,        // 圆形度: 4*pi*area/perimeter²
    Compactness,        // 紧凑度: perimeter²/area
    Convexity,          // 凸度: perimeter_convex/perimeter
    Rectangularity,     // 矩形度: area/bbox_area
    Elongation,         // 延伸度: major_axis/minor_axis
    Orientation,        // 主轴角度 [rad]
    Ra,                 // 等效椭圆长轴半径
    Rb,                 // 等效椭圆短轴半径
    Phi,                // 等效椭圆方向角
    Anisometry,         // 各向异性: Ra/Rb
    Bulkiness,          // 膨胀度: pi*Ra*Rb/area
    StructureFactor,    // 结构因子: Anisometry*Bulkiness-1
    OuterRadius,        // 最小外接圆半径
    InnerRadius,        // 最大内接圆半径
    Holes               // 孔洞数量
};

// 选择操作
enum class SelectOperation {
    And,    // 所有特征都在范围内
    Or      // 任一特征在范围内
};

// 排序模式
enum class SortMode {
    None,               // 不排序
    Area,               // 按面积排序
    Row,                // 按质心行排序
    Column,             // 按质心列排序
    FirstPoint,         // 按首点排序
    LastPoint           // 按末点排序
};
```

---

### 6.2 Connection

连通域分析（提取连通组件）。

```cpp
// 从区域提取连通组件
std::vector<QRegion> Connection(const QRegion& region);

// 从二值图像提取连通组件
std::vector<QRegion> Connection(
    const QImage& binaryImage,
    Connectivity connectivity = Connectivity::Eight  // Four 或 Eight
);
```

**示例**:
```cpp
#include <QiVision/Blob/Blob.h>
using namespace Qi::Vision::Blob;

// 阈值分割
QRegion binaryRegion = ThresholdToRegion(image, 128, 255);

// 提取连通域
auto blobs = Connection(binaryRegion);
std::cout << "Found " << blobs.size() << " blobs\n";
```

---

### 6.3 CountObj / SelectObj

对象计数和选择。

```cpp
// 获取区域数量
inline int32_t CountObj(const std::vector<QRegion>& regions);

// 按索引选择区域（1-based，兼容 Halcon）
QRegion SelectObj(const std::vector<QRegion>& regions, int32_t index);
```

**示例**:
```cpp
auto blobs = Connection(region);
int32_t count = CountObj(blobs);       // 获取数量

QRegion first = SelectObj(blobs, 1);   // 获取第 1 个（1-based）
QRegion last = SelectObj(blobs, count); // 获取最后一个
```

---

### 6.4 AreaCenter

计算区域面积和质心。

```cpp
// 单个区域
void AreaCenter(
    const QRegion& region,
    int64_t& area,      // [out] 面积（像素）
    double& row,        // [out] 质心行
    double& column      // [out] 质心列
);

// 多个区域
void AreaCenter(
    const std::vector<QRegion>& regions,
    std::vector<int64_t>& areas,
    std::vector<double>& rows,
    std::vector<double>& columns
);
```

**示例**:
```cpp
int64_t area;
double row, col;
AreaCenter(blob, area, row, col);

printf("Area: %lld, Center: (%.2f, %.2f)\n", area, col, row);
```

---

### 6.5 SmallestRectangle1 / SmallestRectangle2

获取边界框。

```cpp
// 轴对齐边界框
void SmallestRectangle1(
    const QRegion& region,
    int32_t& row1, int32_t& column1,    // [out] 左上角
    int32_t& row2, int32_t& column2     // [out] 右下角
);

// 最小面积旋转边界框
void SmallestRectangle2(
    const QRegion& region,
    double& row, double& column,        // [out] 中心
    double& phi,                        // [out] 旋转角度 [rad]
    double& length1, double& length2    // [out] 半轴长度
);
```

---

### 6.6 SmallestCircle

获取最小外接圆。

```cpp
void SmallestCircle(
    const QRegion& region,
    double& row, double& column,        // [out] 圆心
    double& radius                      // [out] 半径
);
```

---

### 6.7 形状特征函数

```cpp
// 圆形度 (0-1，1=完美圆形)
double Circularity(const QRegion& region);

// 紧凑度 (>=4*pi，圆形最小)
double Compactness(const QRegion& region);

// 凸度 (0-1)
double Convexity(const QRegion& region);

// 矩形度 (0-1，1=完美矩形)
double Rectangularity(const QRegion& region);

// 等效椭圆参数
void EllipticAxis(
    const QRegion& region,
    double& ra,         // [out] 长轴半径
    double& rb,         // [out] 短轴半径
    double& phi         // [out] 方向角 [rad]
);

// 区域方向 [-pi/2, pi/2]
double OrientationRegion(const QRegion& region);

// 二阶中心矩
void MomentsRegion2nd(
    const QRegion& region,
    double& m11, double& m20, double& m02,  // [out] 中心矩
    double& ia, double& ib                  // [out] 惯性矩
);

// 偏心特征
void Eccentricity(
    const QRegion& region,
    double& anisometry,     // [out] Ra/Rb
    double& bulkiness,      // [out] pi*Ra*Rb/Area
    double& structureFactor // [out] Anisometry*Bulkiness-1
);
```

**示例**:
```cpp
double circ = Circularity(blob);
if (circ > 0.9) {
    printf("This is a circular blob!\n");
}

double ra, rb, phi;
EllipticAxis(blob, ra, rb, phi);
printf("Ellipse: Ra=%.2f, Rb=%.2f, Phi=%.2f°\n", ra, rb, DEG(phi));
```

---

### 6.8 SelectShape

按形状特征选择区域。

```cpp
// 使用枚举
std::vector<QRegion> SelectShape(
    const std::vector<QRegion>& regions,
    ShapeFeature feature,
    SelectOperation operation,
    double minValue,
    double maxValue
);

// 使用字符串（Halcon 兼容）
std::vector<QRegion> SelectShape(
    const std::vector<QRegion>& regions,
    const std::string& feature,     // "area", "circularity", ...
    const std::string& operation,   // "and" 或 "or"
    double minValue,
    double maxValue
);

// 快捷函数
std::vector<QRegion> SelectShapeArea(
    const std::vector<QRegion>& regions,
    int64_t minArea, int64_t maxArea
);

std::vector<QRegion> SelectShapeCircularity(
    const std::vector<QRegion>& regions,
    double minCirc, double maxCirc
);

std::vector<QRegion> SelectShapeRectangularity(
    const std::vector<QRegion>& regions,
    double minRect, double maxRect
);
```

**示例**:
```cpp
// 选择面积 > 100 的 blob
auto large = SelectShape(blobs, ShapeFeature::Area,
                         SelectOperation::And, 100, 999999);

// 选择圆形 blob
auto circular = SelectShape(large, ShapeFeature::Circularity,
                           SelectOperation::And, 0.8, 1.0);

// 或使用字符串
auto selected = SelectShape(blobs, "area", "and", 100, 999999);
```

---

### 6.9 SortRegion

区域排序。

```cpp
// 使用枚举
std::vector<QRegion> SortRegion(
    const std::vector<QRegion>& regions,
    SortMode mode,
    bool ascending = true
);

// 使用字符串（Halcon 兼容）
std::vector<QRegion> SortRegion(
    const std::vector<QRegion>& regions,
    const std::string& sortMode,    // "character", "first_point", ...
    const std::string& order,       // "true" 或 "false"
    const std::string& rowOrCol     // "row" 或 "column"
);
```

---

### 6.10 工具函数

```cpp
// 获取单个特征值
double GetRegionFeature(const QRegion& region, ShapeFeature feature);

// 获取多个区域的特征值
std::vector<double> GetRegionFeatures(
    const std::vector<QRegion>& regions,
    ShapeFeature feature
);

// 特征名称转换
ShapeFeature ParseShapeFeature(const std::string& name);
std::string GetShapeFeatureName(ShapeFeature feature);
```

---

### 6.11 InnerCircle

获取最大内接圆（使用距离变换）。

```cpp
void InnerCircle(
    const QRegion& region,
    double& row, double& column,        // [out] 圆心
    double& radius                      // [out] 半径
);
```

**示例**:
```cpp
double row, col, radius;
InnerCircle(blob, row, col, radius);
printf("Largest inscribed circle: center=(%.2f, %.2f), radius=%.2f\n", col, row, radius);
```

---

### 6.12 ContourLength

计算区域轮廓长度（周长）。

```cpp
double ContourLength(const QRegion& region);
```

**示例**:
```cpp
double perimeter = ContourLength(blob);
double circularity = 4 * M_PI * area / (perimeter * perimeter);
```

---

### 6.13 孔洞分析

```cpp
// 计算孔洞数量
int32_t CountHoles(const QRegion& region);

// 计算欧拉数 (连通域数量 - 孔洞数量)
int32_t EulerNumber(const QRegion& region);

// 填充所有孔洞
QRegion FillUp(const QRegion& region);

// 获取所有孔洞区域
std::vector<QRegion> GetHoles(const QRegion& region);
```

**示例**:
```cpp
// 检测有孔洞的对象
int32_t holes = CountHoles(blob);
if (holes > 0) {
    printf("Blob has %d holes\n", holes);

    // 获取孔洞区域
    auto holeRegions = GetHoles(blob);
    for (const auto& hole : holeRegions) {
        int64_t holeArea;
        double r, c;
        AreaCenter(hole, holeArea, r, c);
        printf("  Hole at (%.2f, %.2f), area=%lld\n", c, r, holeArea);
    }

    // 填充孔洞
    QRegion filled = FillUp(blob);
}
```

---

### 6.14 高级选择函数

```cpp
// 按标准差选择（选择特征值在均值±N个标准差范围内的区域）
std::vector<QRegion> SelectShapeStd(
    const std::vector<QRegion>& regions,
    ShapeFeature feature,
    double deviationFactor          // 标准差倍数，如 1.0, 2.0
);

// 多特征选择
std::vector<QRegion> SelectShapeMulti(
    const std::vector<QRegion>& regions,
    const std::vector<ShapeFeature>& features,
    SelectOperation operation,      // And: 全部满足, Or: 任一满足
    const std::vector<double>& minValues,
    const std::vector<double>& maxValues
);

// 按凸度选择
std::vector<QRegion> SelectShapeConvexity(
    const std::vector<QRegion>& regions,
    double minConvex, double maxConvex
);

// 按延伸度选择
std::vector<QRegion> SelectShapeElongation(
    const std::vector<QRegion>& regions,
    double minElong, double maxElong
);

// 选择 N 个最大/最小区域
std::vector<QRegion> SelectShapeProto(
    const std::vector<QRegion>& regions,
    int32_t n,                      // 选择数量
    bool largest = true             // true=最大, false=最小
);
```

**示例**:
```cpp
// 剔除异常大小的区域（面积在均值±2个标准差内）
auto normal = SelectShapeStd(blobs, ShapeFeature::Area, 2.0);

// 选择同时满足多个条件的区域
auto selected = SelectShapeMulti(blobs,
    {ShapeFeature::Area, ShapeFeature::Circularity},
    SelectOperation::And,
    {100, 0.7},     // 最小值
    {10000, 1.0}    // 最大值
);

// 选择 5 个最大的区域
auto top5 = SelectShapeProto(blobs, 5, true);

// 选择凸度高的区域（接近凸形状）
auto convex = SelectShapeConvexity(blobs, 0.9, 1.0);
```

---

## 7. Display 模块

**命名空间**: `Qi::Vision`
**头文件**: `<QiVision/Display/Display.h>`

图像显示和绘图原语，用于调试和可视化。

### 7.1 DrawColor

绘图颜色类型。

```cpp
struct DrawColor {
    uint8_t r = 255;
    uint8_t g = 255;
    uint8_t b = 255;

    DrawColor();
    DrawColor(uint8_t r, uint8_t g, uint8_t b);
    DrawColor(uint8_t gray);  // 灰度

    // 预定义颜色
    static DrawColor Red();
    static DrawColor Green();
    static DrawColor Blue();
    static DrawColor Yellow();
    static DrawColor Cyan();
    static DrawColor Magenta();
    static DrawColor White();
    static DrawColor Black();
    static DrawColor Orange();
};
```

---

### 7.2 DispImage

显示图像。

```cpp
// 显示图像（保存到临时文件并用系统查看器打开）
bool DispImage(
    const QImage& image,
    const std::string& title = "image"
);

// 设置输出目录
void SetDispOutputDir(const std::string& path);

// 清理临时显示图像
void CleanDispImages();
```

---

### 7.3 DispLine

绘制线段。

```cpp
// 坐标绘制
void DispLine(
    QImage& image,
    double row1, double col1,       // 起点
    double row2, double col2,       // 终点
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);

// 从 Line2d 绘制
void DispLine(
    QImage& image,
    const Line2d& line,
    double length,
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);
```

---

### 7.4 DispCircle / DispEllipse

绘制圆和椭圆。

```cpp
// 圆形
void DispCircle(
    QImage& image,
    double row, double column,      // 圆心
    double radius,
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);

void DispCircle(
    QImage& image,
    const Circle2d& circle,
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);

// 椭圆
void DispEllipse(
    QImage& image,
    double row, double column,      // 中心
    double phi,                     // 方向角 [rad]
    double ra, double rb,           // 半轴长度
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);

void DispEllipse(
    QImage& image,
    const Ellipse2d& ellipse,
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);
```

---

### 7.5 DispRectangle1 / DispRectangle2

绘制矩形。

```cpp
// 轴对齐矩形
void DispRectangle1(
    QImage& image,
    double row1, double col1,       // 左上角
    double row2, double col2,       // 右下角
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);

void DispRectangle1(
    QImage& image,
    const Rect2i& rect,
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);

// 旋转矩形
void DispRectangle2(
    QImage& image,
    double row, double column,      // 中心
    double phi,                     // 旋转角度 [rad]
    double length1, double length2, // 半边长
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);
```

---

### 7.6 DispCross / DispArrow

绘制标记。

```cpp
// 十字标记
void DispCross(
    QImage& image,
    double row, double column,
    int32_t size,                   // 十字臂长
    double angle = 0.0,             // 旋转角度 [rad]
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);

// 箭头
void DispArrow(
    QImage& image,
    double row1, double col1,       // 起点
    double row2, double col2,       // 终点（箭头尖端）
    double headSize = 10.0,         // 箭头头部大小
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);
```

---

### 7.7 DispPolygon / DispContour

绘制多边形和轮廓。

```cpp
// 多边形
void DispPolygon(
    QImage& image,
    const std::vector<double>& rows,
    const std::vector<double>& cols,
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);

// 单个轮廓
void DispContour(
    QImage& image,
    const QContour& contour,
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);

// 多个轮廓
void DispContours(
    QImage& image,
    const QContourArray& contours,
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);
```

---

### 7.8 DispPoint / DispText

绘制点和文字。

```cpp
// 单点
void DispPoint(
    QImage& image,
    double row, double column,
    const DrawColor& color = DrawColor::Green()
);

// 多点
void DispPoints(
    QImage& image,
    const std::vector<double>& rows,
    const std::vector<double>& cols,
    const DrawColor& color = DrawColor::Green()
);

// 文字
void DispText(
    QImage& image,
    double row, double column,
    const std::string& text,
    const DrawColor& color = DrawColor::Green(),
    int32_t scale = 1
);
```

---

### 7.9 高级绘图函数

```cpp
// 绘制匹配结果（十字 + 角度指示器）
void DispMatchResult(
    QImage& image,
    double row, double column,
    double angle,                   // 匹配角度 [rad]
    double score = 1.0,
    const DrawColor& color = DrawColor::Green(),
    int32_t markerSize = 20
);

// 绘制边缘测量结果
void DispEdgeResult(
    QImage& image,
    double row, double column,
    const DrawColor& color = DrawColor::Green(),
    int32_t markerSize = 5
);
```

---

### 7.10 图像转换工具

```cpp
// 灰度转 RGB（用于彩色绘图）
QImage GrayToRgb(const QImage& gray);

// 准备绘图（灰度自动转 RGB）
QImage PrepareForDrawing(const QImage& image);
```

**示例**:
```cpp
#include <QiVision/Display/Display.h>
using namespace Qi::Vision;

// 准备图像
QImage display = PrepareForDrawing(grayImage);

// 绘制匹配结果
for (size_t i = 0; i < rows.size(); ++i) {
    DispMatchResult(display, rows[i], cols[i], angles[i], scores[i],
                   DrawColor::Green(), 30);
}

// 绘制测量区域
DispRectangle2(display, rect.row, rect.col, rect.phi,
              rect.length1, rect.length2, DrawColor::Yellow(), 2);

// 保存结果
IO::WriteImage(display, "result.png");
```

---

## 8. GUI 模块

**命名空间**: `Qi::Vision::GUI`
**头文件**: `<QiVision/GUI/Window.h>`

轻量级 GUI 窗口，用于图像显示和调试。

**平台支持**:
- Linux: X11 (Xlib)
- Windows: Win32 GDI

### 8.1 ScaleMode

图像缩放模式。

```cpp
enum class ScaleMode {
    None,       // 不缩放 (1:1 像素)
    Fit,        // 适应窗口（保持宽高比）
    Fill,       // 填充窗口（可能裁剪）
    Stretch     // 拉伸至窗口大小（忽略宽高比）
};
```

---

### 8.2 Window 类

窗口类。

```cpp
class Window {
public:
    // 构造函数
    Window(
        const std::string& title = "QiVision",
        int32_t width = 0,      // 0 = 自动（从首张图像获取）
        int32_t height = 0
    );

    ~Window();

    // 禁止拷贝，允许移动
    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;
    Window(Window&& other) noexcept;
    Window& operator=(Window&& other) noexcept;

    // 显示图像
    void DispImage(
        const QImage& image,
        ScaleMode scaleMode = ScaleMode::Fit
    );

    // 等待按键
    int32_t WaitKey(
        int32_t timeoutMs = 0   // 0 = 永久等待，-1 = 不等待
    );  // 返回按键码，超时/关闭返回 -1

    // 窗口状态
    bool IsOpen() const;
    void Close();

    // 窗口属性
    void SetTitle(const std::string& title);
    void Resize(int32_t width, int32_t height);
    void GetSize(int32_t& width, int32_t& height) const;
    void Move(int32_t x, int32_t y);

    // 静态便捷函数
    static int32_t ShowImage(
        const QImage& image,
        const std::string& title = "QiVision"
    );

    static int32_t ShowImage(
        const QImage& image,
        const std::string& title,
        int32_t timeoutMs
    );
};
```

**示例**:
```cpp
#include <QiVision/GUI/Window.h>
using namespace Qi::Vision::GUI;

// 创建窗口
Window win("Debug", 800, 600);

// 显示图像
win.DispImage(image);

// 等待任意键
win.WaitKey();

// 带超时的循环显示
while (win.WaitKey(30) != 'q') {
    win.DispImage(processedImage);
}

// 快速显示（静态方法）
Window::ShowImage(image, "Preview");
```

---

### 8.3 便捷全局函数

Halcon 风格的全局函数。

```cpp
// 显示图像（自动创建/复用窗口）
void DispImage(
    const QImage& image,
    const std::string& windowName = "QiVision"
);

// 等待按键（任意窗口）
int32_t WaitKey(int32_t timeoutMs = 0);

// 关闭指定窗口
void CloseWindow(const std::string& windowName);

// 关闭所有窗口
void CloseAllWindows();
```

**示例**:
```cpp
#include <QiVision/GUI/Window.h>
using namespace Qi::Vision::GUI;

// 使用全局函数（简化用法）
DispImage(image1, "Window1");
DispImage(image2, "Window2");

// 等待按键
int key = WaitKey();

// 关闭窗口
CloseWindow("Window1");
CloseAllWindows();
```

---

## 附录

### A. 像素类型

```cpp
enum class PixelType {
    UInt8,      // 8 位无符号 (0-255)
    UInt16,     // 16 位无符号 (0-65535)
    Int16,      // 16 位有符号
    Float32     // 32 位浮点
};
```

### B. 通道类型

```cpp
enum class ChannelType {
    Gray,       // 灰度 (1 通道)
    RGB,        // RGB (3 通道)
    BGR,        // BGR (3 通道)
    RGBA,       // RGBA (4 通道)
    BGRA        // BGRA (4 通道)
};
```

### C. 角度转换宏

```cpp
#define RAD(deg) ((deg) * 0.017453292519943295)  // 度 -> 弧度
#define DEG(rad) ((rad) * 57.29577951308232)     // 弧度 -> 度
```

### D. 边界处理模式

| 模式 | 说明 | 示例 (边界外访问 x=-1) |
|------|------|------------------------|
| `reflect` | 镜像 | `image[1]` |
| `replicate` | 复制边缘 | `image[0]` |
| `constant` | 常数填充 | `0` |
| `wrap` | 循环 | `image[width-1]` |

---

## 版本历史

| 版本 | 日期 | 变更 |
|------|------|------|
| 0.3.0 | 2026-01-20 | 添加 Metrology 模块文档，新增自动阈值 API |
| 0.2.0 | 2026-01-17 | 添加 Blob, Display, GUI 模块文档 |
| 0.1.0 | 2026-01-15 | 初始版本：Matching, Measure, IO, Color, Filter |

