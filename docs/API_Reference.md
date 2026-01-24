# QiVision API Reference

> Version: 0.5.0
> Last Updated: 2026-01-24
> Namespace: `Qi::Vision`

This document provides a comprehensive reference for all public APIs in QiVision. All functions follow the Halcon-style signature: `void Func(inputs..., outputs..., params...)`.

---

## Table of Contents

1. [Matching](#1-matching) - Shape-based template matching
2. [Measure](#2-measure) - Edge measurement and metrology
3. [IO](#3-io) - Image read/write
4. [Color](#4-color) - Color space conversion
5. [Filter](#5-filter) - Image filtering
6. [Segment](#6-segment) - Image segmentation
7. [Blob](#7-blob) - Region analysis
8. [Display](#8-display) - Drawing primitives
9. [GUI](#9-gui) - Window display
10. [Appendix](#appendix) - Types and constants

---

## 1. Matching

**Namespace**: `Qi::Vision::Matching`
**Header**: `<QiVision/Matching/ShapeModel.h>`

Shape-based template matching using gradient orientation features. Compatible with Halcon's shape model operators.

---

### ShapeModel

Handle class for shape model data.

```cpp
class ShapeModel {
public:
    ShapeModel();
    bool IsValid() const;
};
```

---

### CreateShapeModel

Creates a shape model for rotation-only matching.

```cpp
void CreateShapeModel(
    const QImage& templateImage,
    ShapeModel& model,
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

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| templateImage | const QImage& | Template image (grayscale) |
| model | ShapeModel& | [out] Output model handle |
| numLevels | int32_t | Pyramid levels (0=auto) |
| angleStart | double | Start angle [rad] |
| angleExtent | double | Angle range [rad] (0=full rotation) |
| angleStep | double | Angle step [rad] (0=auto, ~1 degree) |
| optimization | const std::string& | Point optimization: "none", "auto", "point_reduction_low/medium/high" |
| metric | const std::string& | Match metric: "use_polarity", "ignore_global_polarity" |
| contrast | const std::string& | Contrast threshold: "auto" or numeric string |
| minContrast | double | Minimum contrast for search |

---

### CreateShapeModel (with ROI)

Creates a shape model using a rectangular or region-based ROI.

```cpp
void CreateShapeModel(
    const QImage& templateImage,
    const Rect2i& roi,
    ShapeModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast
);

void CreateShapeModel(
    const QImage& templateImage,
    const QRegion& region,
    ShapeModel& model,
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

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| roi | const Rect2i& | Rectangular region of interest |
| region | const QRegion& | Arbitrary shape region |

---

### CreateScaledShapeModel

Creates a shape model with scale support.

```cpp
void CreateScaledShapeModel(
    const QImage& templateImage,
    ShapeModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    double scaleMin,
    double scaleMax,
    double scaleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| scaleMin | double | Minimum scale ratio |
| scaleMax | double | Maximum scale ratio |
| scaleStep | double | Scale step (0=auto) |

---

### FindShapeModel

Finds template matches in an image (rotation only).

```cpp
void FindShapeModel(
    const QImage& image,
    const ShapeModel& model,
    double angleStart,
    double angleExtent,
    double minScore,
    int32_t numMatches,
    double maxOverlap,
    const std::string& subPixel,
    int32_t numLevels,
    double greediness,
    std::vector<double>& rows,
    std::vector<double>& cols,
    std::vector<double>& angles,
    std::vector<double>& scores
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| image | const QImage& | Search image |
| model | const ShapeModel& | Model handle |
| angleStart | double | Search start angle [rad] |
| angleExtent | double | Search angle range [rad] |
| minScore | double | Minimum match score (0-1) |
| numMatches | int32_t | Maximum matches (0=all) |
| maxOverlap | double | Maximum overlap ratio (0-1) |
| subPixel | const std::string& | Subpixel mode: "none", "interpolation", "least_squares" |
| numLevels | int32_t | Pyramid levels to use (0=all) |
| greediness | double | Greediness (0-1), higher is faster but may miss matches |
| rows | std::vector<double>& | [out] Match Y coordinates |
| cols | std::vector<double>& | [out] Match X coordinates |
| angles | std::vector<double>& | [out] Match angles [rad] |
| scores | std::vector<double>& | [out] Match scores |

---

### FindScaledShapeModel

Finds template matches with scale variation.

```cpp
void FindScaledShapeModel(
    const QImage& image,
    const ShapeModel& model,
    double angleStart,
    double angleExtent,
    double scaleMin,
    double scaleMax,
    double minScore,
    int32_t numMatches,
    double maxOverlap,
    const std::string& subPixel,
    int32_t numLevels,
    double greediness,
    std::vector<double>& rows,
    std::vector<double>& cols,
    std::vector<double>& angles,
    std::vector<double>& scales,
    std::vector<double>& scores
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| scaleMin | double | Search minimum scale |
| scaleMax | double | Search maximum scale |
| scales | std::vector<double>& | [out] Match scale values |

---

### GetShapeModelXLD

Gets model contours for visualization.

```cpp
void GetShapeModelXLD(
    const ShapeModel& model,
    int32_t level,
    QContourArray& contours
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| level | int32_t | Pyramid level |
| contours | QContourArray& | [out] Model contours |

---

### GetShapeModelParams

Gets model parameters.

```cpp
void GetShapeModelParams(
    const ShapeModel& model,
    int32_t& numLevels,
    double& angleStart,
    double& angleExtent,
    double& angleStep,
    double& scaleMin,
    double& scaleMax,
    double& scaleStep,
    std::string& metric
);
```

---

### GetShapeModelOrigin / SetShapeModelOrigin

Gets or sets the model origin point.

```cpp
void GetShapeModelOrigin(const ShapeModel& model, double& row, double& col);
void SetShapeModelOrigin(ShapeModel& model, double row, double col);
```

---

### WriteShapeModel / ReadShapeModel

Saves or loads a model file.

```cpp
void WriteShapeModel(const ShapeModel& model, const std::string& filename);
void ReadShapeModel(const std::string& filename, ShapeModel& model);
```

---

### ClearShapeModel

Releases model resources.

```cpp
void ClearShapeModel(ShapeModel& model);
```

---

### DetermineShapeModelParams

Automatically determines recommended parameters.

```cpp
void DetermineShapeModelParams(
    const QImage& templateImage,
    int32_t& numLevels,
    double& contrast
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| numLevels | int32_t& | [out] Recommended pyramid levels |
| contrast | double& | [out] Recommended contrast threshold |

---

## 2. Measure

**Namespace**: `Qi::Vision::Measure`
**Headers**: `<QiVision/Measure/Caliper.h>`, `<QiVision/Measure/CaliperArray.h>`, `<QiVision/Measure/Metrology.h>`

Subpixel edge measurement tools for precise edge position and width measurement.

---

### MeasureRectangle2

Rectangular measurement region structure.

```cpp
struct MeasureRectangle2 {
    double row;         // Center Y
    double col;         // Center X
    double phi;         // Rotation angle [rad]
    double length1;     // Half-width (along normal direction)
    double length2;     // Half-length (along edge direction)
};
```

---

### MeasureArc

Arc-shaped measurement region structure.

```cpp
struct MeasureArc {
    double centerRow;   // Center Y
    double centerCol;   // Center X
    double radius;      // Radius
    double angleStart;  // Start angle [rad]
    double angleExtent; // Angle extent [rad]
    double annulusWidth;// Annulus width
};
```

---

### EdgeResult

Edge detection result structure.

```cpp
struct EdgeResult {
    double row;         // Edge Y position (subpixel)
    double column;      // Edge X position (subpixel)
    double amplitude;   // Edge strength
    double distance;    // Distance along profile
};
```

---

### PairResult

Edge pair result structure.

```cpp
struct PairResult {
    EdgeResult first;       // First edge
    EdgeResult second;      // Second edge
    double width;           // Edge pair width
    double centerRow;       // Center Y
    double centerColumn;    // Center X
};
```

---

### MeasurePos

Measures edge positions.

```cpp
std::vector<EdgeResult> MeasurePos(
    const QImage& image,
    const MeasureRectangle2& handle,
    double sigma,
    double threshold,
    const std::string& transition,
    const std::string& select
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| image | const QImage& | Input grayscale image |
| handle | const MeasureRectangle2& | Measurement handle |
| sigma | double | Gaussian smoothing sigma |
| threshold | double | Edge amplitude threshold |
| transition | const std::string& | Edge type: "positive", "negative", "all" |
| select | const std::string& | Selection: "first", "last", "all" |

**Returns**
| Type | Description |
|------|-------------|
| std::vector<EdgeResult> | Detected edge results |

---

### MeasurePairs

Measures edge pairs (width measurement).

```cpp
std::vector<PairResult> MeasurePairs(
    const QImage& image,
    const MeasureRectangle2& handle,
    double sigma,
    double threshold,
    const std::string& transition,
    const std::string& select
);
```

**Returns**
| Type | Description |
|------|-------------|
| std::vector<PairResult> | Detected edge pair results |

---

### FuzzyMeasurePos / FuzzyMeasurePairs

Fuzzy measurement with noise robustness.

```cpp
std::vector<EdgeResult> FuzzyMeasurePos(
    const QImage& image,
    const MeasureRectangle2& handle,
    double sigma,
    double threshold,
    const std::string& transition,
    const std::string& select,
    double fuzzyThresh = 0.5
);
```

---

### MetrologyModel

Combined measurement model framework.

```cpp
class MetrologyModel {
public:
    int32_t AddLineMeasure(double row1, double col1, double row2, double col2,
                           double measureLength1, double measureLength2,
                           const std::string& transition = "all",
                           const std::string& select = "all",
                           const std::vector<int>& params = {});

    int32_t AddCircleMeasure(double row, double col, double radius,
                              double measureLength1, double measureLength2,
                              const std::string& transition = "all",
                              const std::string& select = "all",
                              const std::vector<int>& params = {});

    int32_t AddEllipseMeasure(double row, double col, double phi,
                               double ra, double rb,
                               double measureLength1, double measureLength2,
                               const std::string& transition = "all",
                               const std::string& select = "all",
                               const std::vector<int>& params = {});

    int32_t AddRectangle2Measure(double row, double col, double phi,
                                  double length1, double length2,
                                  double measureLength1, double measureLength2,
                                  const std::string& transition = "all",
                                  const std::string& select = "all",
                                  const std::vector<int>& params = {});

    bool Apply(const QImage& image);

    MetrologyLineResult GetLineResult(int32_t index) const;
    MetrologyCircleResult GetCircleResult(int32_t index) const;
    MetrologyEllipseResult GetEllipseResult(int32_t index) const;
    MetrologyRectangle2Result GetRectangle2Result(int32_t index) const;

    std::vector<Point2d> GetMeasuredPoints(int32_t index) const;
    std::vector<double> GetPointWeights(int32_t index) const;

    void Align(double rowOffset, double colOffset, double phi);
    void ResetAlignment();
};
```

---

### MetrologyParamFlag

OpenCV-style parameter flags for vector<int> params.

```cpp
enum MetrologyParamFlag {
    METROLOGY_NUM_INSTANCES = 1,
    METROLOGY_MEASURE_SIGMA = 2,      // sigma * 100
    METROLOGY_MEASURE_THRESHOLD = 3,
    METROLOGY_NUM_MEASURES = 4,
    METROLOGY_MIN_SCORE = 5,          // score * 100
    METROLOGY_FIT_METHOD = 6,         // 0=RANSAC, 1=Huber, 2=Tukey
    METROLOGY_DISTANCE_THRESHOLD = 7, // threshold * 100
    METROLOGY_MAX_ITERATIONS = 8,
    METROLOGY_RAND_SEED = 9,
    METROLOGY_THRESHOLD_MODE = 10     // 0=Manual, 1=Auto
};
```

---

## 3. IO

**Namespace**: `Qi::Vision::IO`
**Header**: `<QiVision/IO/ImageIO.h>`

Image reading and writing with multi-format support.

---

### ReadImage

Reads an image file.

```cpp
void ReadImage(const std::string& filename, QImage& image);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| filename | const std::string& | File path |
| image | QImage& | [out] Output image |

**Supported Formats**: PNG, JPEG, BMP, TIFF, PGM/PPM, RAW

---

### ReadImageRaw

Reads a RAW format image (headerless).

```cpp
void ReadImageRaw(
    const std::string& filename,
    QImage& image,
    int32_t width,
    int32_t height,
    PixelType pixelType = PixelType::UInt8,
    ChannelType channelType = ChannelType::Gray,
    int32_t headerBytes = 0,
    bool bigEndian = false
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| width | int32_t | Image width |
| height | int32_t | Image height |
| pixelType | PixelType | Pixel type (UInt8, UInt16, Float32) |
| channelType | ChannelType | Channel type (Gray, RGB, etc.) |
| headerBytes | int32_t | Header bytes to skip |
| bigEndian | bool | Big-endian byte order (for 16-bit) |

---

### ReadImageGray

Reads and converts to grayscale.

```cpp
void ReadImageGray(const std::string& filename, QImage& image);
```

---

### WriteImage

Writes an image file.

```cpp
bool WriteImage(const QImage& image, const std::string& filename);

bool WriteImage(
    const QImage& image,
    const std::string& filename,
    ImageFormat format,
    const std::vector<int>& params
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| format | ImageFormat | Output format (Auto, PNG, JPEG, etc.) |
| params | const std::vector<int>& | Key-value parameter pairs |

**Returns**
| Type | Description |
|------|-------------|
| bool | True on success |

---

### ImageWriteFlag

Write parameter flags.

```cpp
enum ImageWriteFlag {
    QIWRITE_JPEG_QUALITY = 1,       // 0-100, default 95
    QIWRITE_PNG_COMPRESSION = 2,    // 0-9, default 6
    QIWRITE_TIFF_COMPRESSION = 3    // 0=off, 1=LZW
};
```

---

### ReadSequence / WriteSequence

Batch read/write image sequences.

```cpp
void ReadSequence(
    const std::string& pattern,
    std::vector<QImage>& images,
    int32_t startIndex,
    int32_t endIndex,
    int32_t step = 1
);

int32_t WriteSequence(
    const std::vector<QImage>& images,
    const std::string& pattern,
    int32_t startIndex = 0
);
```

---

## 4. Color

**Namespace**: `Qi::Vision::Color`
**Header**: `<QiVision/Color/ColorConvert.h>`

Color space conversion and channel operations.

---

### ColorSpace

Color space enumeration.

```cpp
enum class ColorSpace {
    Gray, RGB, BGR, RGBA, BGRA,
    HSV, HSL, Lab, Luv, XYZ, YCrCb, YUV
};
```

---

### TransFromRgb

Converts from RGB to another color space.

```cpp
void TransFromRgb(const QImage& image, QImage& output, ColorSpace toSpace);
void TransFromRgb(const QImage& image, QImage& output, const std::string& colorSpace);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| image | const QImage& | Input RGB image |
| output | QImage& | [out] Output image |
| toSpace | ColorSpace | Target color space |
| colorSpace | const std::string& | Target: "hsv", "hsl", "lab", "yuv", "ycrcb" |

---

### TransToRgb

Converts from another color space to RGB.

```cpp
void TransToRgb(const QImage& image, QImage& output, ColorSpace fromSpace);
void TransToRgb(const QImage& image, QImage& output, const std::string& colorSpace);
```

---

### Rgb1ToGray

Converts RGB image to grayscale.

```cpp
void Rgb1ToGray(
    const QImage& image,
    QImage& output,
    const std::string& method = "luminosity"
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| method | const std::string& | Method: "luminosity", "average", "lightness", "bt709", "max", "min" |

---

### GrayToRgb

Converts grayscale to RGB (copies to 3 channels).

```cpp
void GrayToRgb(const QImage& gray, QImage& output);
```

---

### Decompose3 / Compose3

Splits or merges 3-channel images.

```cpp
void Decompose3(const QImage& image, QImage& ch1, QImage& ch2, QImage& ch3);
void Compose3(const QImage& ch1, const QImage& ch2, const QImage& ch3, QImage& output);
```

---

### SplitChannels / MergeChannels

Splits or merges arbitrary channel images.

```cpp
void SplitChannels(const QImage& image, std::vector<QImage>& channels);
void MergeChannels(const std::vector<QImage>& channels, QImage& output);
```

---

### AdjustBrightness / AdjustContrast / AdjustGamma

Color adjustment functions.

```cpp
void AdjustBrightness(const QImage& image, QImage& output, double brightness);
void AdjustContrast(const QImage& image, QImage& output, double contrast);
void AdjustGamma(const QImage& image, QImage& output, double gamma);
void AdjustSaturation(const QImage& image, QImage& output, double saturation);
void AdjustHue(const QImage& image, QImage& output, double hueShift);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| brightness | double | Brightness offset [-255, 255] |
| contrast | double | Contrast factor (1.0=unchanged) |
| gamma | double | Gamma value (1.0=unchanged) |
| saturation | double | Saturation (0=gray, 1.0=unchanged) |
| hueShift | double | Hue shift in degrees [-180, 180] |

---

### CfaToRgb

Bayer demosaicing for industrial cameras.

```cpp
QImage CfaToRgb(
    const QImage& cfaImage,
    const std::string& cfaType = "bayer_gb",
    const std::string& interpolation = "bilinear"
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| cfaType | const std::string& | Bayer pattern: "bayer_rg", "bayer_gr", "bayer_gb", "bayer_bg" |
| interpolation | const std::string& | Method: "bilinear", "bilinear_dir" |

**Returns**
| Type | Description |
|------|-------------|
| QImage | RGB output image |

---

## 5. Filter

**Namespace**: `Qi::Vision::Filter`
**Header**: `<QiVision/Filter/Filter.h>`

Image filtering: smoothing, edge detection, and enhancement.

---

### GaussFilter

Gaussian smoothing.

```cpp
void GaussFilter(const QImage& image, QImage& output, double sigma);

void GaussFilter(
    const QImage& image,
    QImage& output,
    double sigmaX,
    double sigmaY,
    const std::string& borderMode = "reflect"
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| sigma | double | Gaussian sigma |
| sigmaX | double | X-direction sigma |
| sigmaY | double | Y-direction sigma |
| borderMode | const std::string& | Border mode: "reflect", "replicate", "constant", "wrap" |

---

### MeanImage

Box filter (mean smoothing).

```cpp
QImage MeanImage(const QImage& image, int32_t size);
QImage MeanImage(const QImage& image, int32_t width, int32_t height);
```

---

### MedianImage

Median filter.

```cpp
QImage MedianImage(
    const QImage& image,
    const std::string& maskType,
    int32_t radius
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| maskType | const std::string& | Mask shape: "circle", "square", "rhombus" |
| radius | int32_t | Mask radius |

---

### BilateralFilter

Edge-preserving bilateral filter.

```cpp
QImage BilateralFilter(
    const QImage& image,
    double sigmaSpatial,
    double sigmaIntensity
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| sigmaSpatial | double | Spatial sigma (distance weight) |
| sigmaIntensity | double | Intensity sigma (color similarity weight) |

---

### SobelAmp / SobelDir

Sobel gradient operators.

```cpp
QImage SobelAmp(
    const QImage& image,
    const std::string& filterType = "sum_abs",
    int32_t size = 3
);

QImage SobelDir(
    const QImage& image,
    const std::string& dirType = "gradient",
    int32_t size = 3
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| filterType | const std::string& | Amplitude type: "sum_abs", "sum_sqrt" |
| dirType | const std::string& | Direction type: "gradient", "tangent" |
| size | int32_t | Kernel size: 3, 5, or 7 |

---

### DerivateGauss

Gaussian derivative filter.

```cpp
QImage DerivateGauss(
    const QImage& image,
    double sigma,
    const std::string& component
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| component | const std::string& | Derivative: "x", "y", "xx", "yy", "xy", "gradient" |

---

### Laplace / LaplacianOfGaussian

Laplacian operators.

```cpp
QImage Laplace(const QImage& image, const std::string& filterType = "3x3");
QImage LaplacianOfGaussian(const QImage& image, double sigma);
```

---

### UnsharpMask

Sharpening via unsharp masking.

```cpp
QImage UnsharpMask(
    const QImage& image,
    double sigma,
    double amount = 1.0,
    double threshold = 0.0
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| sigma | double | Blur sigma |
| amount | double | Sharpening amount |
| threshold | double | Detail threshold (0=sharpen all) |

---

### AnisoDiff

Anisotropic diffusion (Perona-Malik).

```cpp
QImage AnisoDiff(
    const QImage& image,
    const std::string& mode,
    double contrast,
    double theta,
    int32_t iterations
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| mode | const std::string& | Mode: "pm1" or "pm2" |
| contrast | double | Edge threshold K |
| theta | double | Diffusion coefficient (0-0.25) |
| iterations | int32_t | Number of iterations |

---

### HistogramEqualize / ApplyCLAHE

Histogram enhancement.

```cpp
void HistogramEqualize(const QImage& image, QImage& output);
void ApplyCLAHE(const QImage& image, QImage& output, double clipLimit = 40.0, int32_t tileSize = 8);
```

---

## 6. Segment

**Namespace**: `Qi::Vision::Segment`
**Header**: `<QiVision/Segment/Segment.h>`

Image segmentation and thresholding operations.

---

### ThresholdType

Threshold operation types.

```cpp
enum class ThresholdType {
    Binary,         // dst = (src > thresh) ? maxVal : 0
    BinaryInv,      // dst = (src > thresh) ? 0 : maxVal
    Truncate,       // dst = (src > thresh) ? thresh : src
    ToZero,         // dst = (src > thresh) ? src : 0
    ToZeroInv       // dst = (src > thresh) ? 0 : src
};
```

---

### AutoMethod

Auto-threshold methods.

```cpp
enum class AutoMethod {
    Otsu,           // Otsu's method
    Triangle,       // Triangle algorithm
    MinError,       // Minimum error (Kittler-Illingworth)
    Isodata,        // Iterative isodata
    Median          // Median-based
};
```

---

### AdaptiveMethod

Adaptive threshold methods.

```cpp
enum class AdaptiveMethod {
    Mean,           // Local mean
    Gaussian,       // Gaussian-weighted mean
    Sauvola,        // Sauvola method
    Niblack,        // Niblack method
    Wolf            // Wolf method
};
```

---

### LightDark

Selection mode for dynamic threshold.

```cpp
enum class LightDark {
    Light,          // Pixels brighter than reference
    Dark,           // Pixels darker than reference
    Equal,          // Pixels equal to reference
    NotEqual        // Pixels different from reference
};
```

---

### Threshold

Applies global threshold.

```cpp
void Threshold(
    const QImage& src,
    QImage& dst,
    double threshold,
    double maxValue = 255.0,
    ThresholdType type = ThresholdType::Binary
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| src | const QImage& | Input grayscale image |
| dst | QImage& | [out] Output binary image |
| threshold | double | Threshold value |
| maxValue | double | Maximum output value |
| type | ThresholdType | Threshold operation type |

---

### ThresholdRange

Range thresholding (keeps pixels in [low, high]).

```cpp
void ThresholdRange(
    const QImage& src,
    QImage& dst,
    double low,
    double high,
    double maxValue = 255.0
);
```

---

### ThresholdAuto / ThresholdOtsu / ThresholdTriangle

Automatic threshold selection.

```cpp
void ThresholdAuto(
    const QImage& src,
    QImage& dst,
    AutoMethod method = AutoMethod::Otsu,
    double maxValue = 255.0,
    double* computedThreshold = nullptr
);

void ThresholdOtsu(const QImage& src, QImage& dst, double maxValue = 255.0);
void ThresholdTriangle(const QImage& src, QImage& dst, double maxValue = 255.0);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| method | AutoMethod | Auto-threshold method |
| computedThreshold | double* | [out] Computed threshold value (optional) |

---

### ThresholdAdaptive

Adaptive local thresholding.

```cpp
void ThresholdAdaptive(
    const QImage& src,
    QImage& dst,
    AdaptiveMethod method,
    int32_t blockSize,
    double C
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| method | AdaptiveMethod | Adaptive method |
| blockSize | int32_t | Block size (odd number) |
| C | double | Constant subtracted from mean |

---

### ThresholdToRegion

Creates a region from threshold.

```cpp
QRegion ThresholdToRegion(const QImage& src, double low, double high);
QRegion ThresholdToRegion(const QImage& src, double threshold, bool above = true);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| low | double | Lower threshold (inclusive) |
| high | double | Upper threshold (inclusive) |
| above | bool | If true, select pixels > threshold |

**Returns**
| Type | Description |
|------|-------------|
| QRegion | Region containing selected pixels |

---

### DynThreshold

Dynamic threshold comparing with reference image.

```cpp
QRegion DynThreshold(
    const QImage& image,
    const QImage& reference,
    double offset,
    LightDark lightDark = LightDark::Light
);

QRegion DynThreshold(
    const QImage& image,
    int32_t filterSize,
    double offset,
    LightDark lightDark = LightDark::Light
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| reference | const QImage& | Reference/smoothed image |
| filterSize | int32_t | Gaussian filter size for auto-reference |
| offset | double | Offset value for comparison |
| lightDark | LightDark | Selection mode |

**Returns**
| Type | Description |
|------|-------------|
| QRegion | Selected region |

---

### VarThreshold

Variance-based threshold.

```cpp
QRegion VarThreshold(
    const QImage& image,
    int32_t windowSize,
    double varianceThreshold,
    LightDark lightDark = LightDark::Light
);
```

---

### CharThreshold

Character/text threshold (optimized for documents).

```cpp
QRegion CharThreshold(
    const QImage& image,
    double sigma = 2.0,
    double percent = 95.0,
    LightDark lightDark = LightDark::Dark
);
```

---

### HysteresisThreshold

Dual threshold with connectivity constraint.

```cpp
QRegion HysteresisThreshold(
    const QImage& image,
    double lowThreshold,
    double highThreshold
);
```

---

### BinaryAnd / BinaryOr / BinaryXor / BinaryDiff / BinaryInvert

Binary image operations.

```cpp
void BinaryAnd(const QImage& src1, const QImage& src2, QImage& dst);
void BinaryOr(const QImage& src1, const QImage& src2, QImage& dst);
void BinaryXor(const QImage& src1, const QImage& src2, QImage& dst);
void BinaryDiff(const QImage& src1, const QImage& src2, QImage& dst);
void BinaryInvert(const QImage& src, QImage& dst);
```

---

### RegionToMask / MaskToRegion

Converts between regions and masks.

```cpp
void RegionToMask(const QRegion& region, QImage& mask);
QRegion MaskToRegion(const QImage& mask, double threshold = 0);
```

---

## 7. Blob

**Namespace**: `Qi::Vision::Blob`
**Header**: `<QiVision/Blob/Blob.h>`

Region analysis and shape selection.

---

### ShapeFeature

Shape feature enumeration.

```cpp
enum class ShapeFeature {
    Area,           // Pixel count
    Row, Column,    // Centroid coordinates
    Width, Height,  // Bounding box size
    Circularity,    // 4*pi*area/perimeter^2
    Compactness,    // perimeter^2/area
    Convexity,      // perimeter_convex/perimeter
    Rectangularity, // area/bbox_area
    Elongation,     // major_axis/minor_axis
    Orientation,    // Principal axis angle [rad]
    Ra, Rb, Phi,    // Ellipse parameters
    Anisometry,     // Ra/Rb
    OuterRadius,    // Minimum enclosing circle radius
    InnerRadius,    // Maximum inscribed circle radius
    Holes           // Number of holes
};
```

---

### Connection

Extracts connected components.

```cpp
void Connection(const QRegion& region, std::vector<QRegion>& regions);

void Connection(
    const QImage& binaryImage,
    std::vector<QRegion>& regions,
    Connectivity connectivity = Connectivity::Eight
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| region | const QRegion& | Input region |
| binaryImage | const QImage& | Binary image |
| regions | std::vector<QRegion>& | [out] Connected components |
| connectivity | Connectivity | Four or Eight connectivity |

---

### CountObj / SelectObj

Object count and selection.

```cpp
int32_t CountObj(const std::vector<QRegion>& regions);
QRegion SelectObj(const std::vector<QRegion>& regions, int32_t index);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| index | int32_t | 1-based index (Halcon compatible) |

---

### AreaCenter

Computes area and centroid.

```cpp
void AreaCenter(
    const QRegion& region,
    int64_t& area,
    double& row,
    double& column
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| area | int64_t& | [out] Area in pixels |
| row | double& | [out] Centroid Y |
| column | double& | [out] Centroid X |

---

### SmallestRectangle1 / SmallestRectangle2

Gets bounding boxes.

```cpp
void SmallestRectangle1(
    const QRegion& region,
    int32_t& row1, int32_t& column1,
    int32_t& row2, int32_t& column2
);

void SmallestRectangle2(
    const QRegion& region,
    double& row, double& column,
    double& phi,
    double& length1, double& length2
);
```

---

### SmallestCircle / InnerCircle

Gets enclosing/inscribed circles.

```cpp
void SmallestCircle(
    const QRegion& region,
    double& row, double& column,
    double& radius
);

void InnerCircle(
    const QRegion& region,
    double& row, double& column,
    double& radius
);
```

---

### Circularity / Compactness / Convexity / Rectangularity

Shape feature functions.

```cpp
double Circularity(const QRegion& region);
double Compactness(const QRegion& region);
double Convexity(const QRegion& region);
double Rectangularity(const QRegion& region);
```

**Returns**
| Type | Description |
|------|-------------|
| double | Feature value (0-1 for normalized features) |

---

### EllipticAxis

Gets equivalent ellipse parameters.

```cpp
void EllipticAxis(
    const QRegion& region,
    double& ra,
    double& rb,
    double& phi
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| ra | double& | [out] Semi-major axis |
| rb | double& | [out] Semi-minor axis |
| phi | double& | [out] Orientation angle [rad] |

---

### SelectShape

Selects regions by shape feature.

```cpp
void SelectShape(
    const std::vector<QRegion>& regions,
    std::vector<QRegion>& selected,
    ShapeFeature feature,
    SelectOperation operation,
    double minValue,
    double maxValue
);

void SelectShape(
    const std::vector<QRegion>& regions,
    std::vector<QRegion>& selected,
    const std::string& feature,
    const std::string& operation,
    double minValue,
    double maxValue
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| feature | ShapeFeature | Feature to filter by |
| operation | SelectOperation | And or Or |
| minValue | double | Minimum feature value |
| maxValue | double | Maximum feature value |

---

### SelectShapeArea / SelectShapeCircularity

Convenience selection functions.

```cpp
void SelectShapeArea(
    const std::vector<QRegion>& regions,
    std::vector<QRegion>& selected,
    int64_t minArea, int64_t maxArea
);

void SelectShapeCircularity(
    const std::vector<QRegion>& regions,
    std::vector<QRegion>& selected,
    double minCirc, double maxCirc
);
```

---

### SortRegion

Sorts regions by feature.

```cpp
void SortRegion(
    const std::vector<QRegion>& regions,
    std::vector<QRegion>& sorted,
    SortMode mode,
    bool ascending = true
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| mode | SortMode | Sort criterion: None, Area, Row, Column, FirstPoint, LastPoint |
| ascending | bool | Sort order |

---

### CountHoles / EulerNumber / FillUp / GetHoles

Hole analysis functions.

```cpp
int32_t CountHoles(const QRegion& region);
int32_t EulerNumber(const QRegion& region);
void FillUp(const QRegion& region, QRegion& filled);
void GetHoles(const QRegion& region, std::vector<QRegion>& holes);
```

---

### SelectShapeProto

Selects N largest/smallest regions.

```cpp
void SelectShapeProto(
    const std::vector<QRegion>& regions,
    std::vector<QRegion>& selected,
    int32_t n,
    bool largest = true
);
```

---

## 8. Display

**Namespace**: `Qi::Vision`
**Headers**: `<QiVision/Display/Display.h>`, `<QiVision/Display/Draw.h>`

Drawing primitives for visualization and debugging.

---

### DrawColor / Scalar

Drawing color types.

```cpp
struct DrawColor {
    uint8_t r, g, b;

    static DrawColor Red();
    static DrawColor Green();
    static DrawColor Blue();
    static DrawColor Yellow();
    static DrawColor Cyan();
    static DrawColor Magenta();
    static DrawColor White();
    static DrawColor Black();
};
```

---

### DispLine

Draws a line segment.

```cpp
void DispLine(
    QImage& image,
    double row1, double col1,
    double row2, double col2,
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);
```

---

### DispCircle / DispEllipse

Draws circle or ellipse.

```cpp
void DispCircle(
    QImage& image,
    double row, double column,
    double radius,
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);

void DispEllipse(
    QImage& image,
    double row, double column,
    double phi,
    double ra, double rb,
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);
```

---

### DispRectangle1 / DispRectangle2

Draws rectangles.

```cpp
void DispRectangle1(
    QImage& image,
    double row1, double col1,
    double row2, double col2,
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);

void DispRectangle2(
    QImage& image,
    double row, double column,
    double phi,
    double length1, double length2,
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);
```

---

### DispCross / DispArrow

Draws markers.

```cpp
void DispCross(
    QImage& image,
    double row, double column,
    int32_t size,
    double angle = 0.0,
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);

void DispArrow(
    QImage& image,
    double row1, double col1,
    double row2, double col2,
    double headSize = 10.0,
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);
```

---

### DispContour / DispContours

Draws contours.

```cpp
void DispContour(
    QImage& image,
    const QContour& contour,
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);

void DispContours(
    QImage& image,
    const QContourArray& contours,
    const DrawColor& color = DrawColor::Green(),
    int32_t thickness = 1
);
```

---

### Draw::Region / Draw::RegionContour / Draw::RegionAlpha

Region drawing functions (in Draw namespace).

```cpp
void Draw::Region(QImage& image, const QRegion& region, const Scalar& color);

void Draw::RegionContour(
    QImage& image,
    const QRegion& region,
    const Scalar& color,
    int32_t thickness = 1
);

void Draw::RegionAlpha(
    QImage& image,
    const QRegion& region,
    const Scalar& color,
    double alpha
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| alpha | double | Transparency [0.0-1.0] |

---

### Draw::MeasureRect / Draw::MeasureRects

Draws caliper rectangles.

```cpp
void Draw::MeasureRect(
    QImage& image,
    const MeasureRectangle2& handle,
    const Scalar& color = Scalar::Cyan(),
    int32_t thickness = 1
);

void Draw::MeasureRects(
    QImage& image,
    const std::vector<MeasureRectangle2>& handles,
    const Scalar& color = Scalar::Cyan(),
    int32_t thickness = 1
);
```

---

### Draw::EdgePointsWeighted

Draws edge points with weight-based coloring.

```cpp
void Draw::EdgePointsWeighted(
    QImage& image,
    const std::vector<Point2d>& points,
    const std::vector<double>& weights,
    int32_t markerSize = 3
);
```

---

### Draw::MetrologyModelResult

Draws complete metrology model visualization.

```cpp
void Draw::MetrologyModelResult(
    QImage& image,
    const MetrologyModel& model,
    const Scalar& objectColor = Scalar::Cyan(),
    const Scalar& resultColor = Scalar::Green(),
    const Scalar& pointColor = Scalar::Red(),
    bool drawCalipers = true,
    bool drawPoints = true
);
```

---

## 9. GUI

**Namespace**: `Qi::Vision::GUI`
**Header**: `<QiVision/GUI/Window.h>`

Lightweight GUI for image display and debugging.

**Platform Support**: Linux (X11), Windows (Win32 GDI)

---

### ScaleMode

Image scaling modes.

```cpp
enum class ScaleMode {
    None,       // 1:1 pixels
    Fit,        // Fit to window (keep aspect ratio)
    Fill,       // Fill window (may crop)
    Stretch     // Stretch to window (ignore aspect ratio)
};
```

---

### Window

Window class for image display.

```cpp
class Window {
public:
    Window(const std::string& title = "QiVision", int32_t width = 0, int32_t height = 0);
    ~Window();

    void DispImage(const QImage& image, ScaleMode scaleMode = ScaleMode::Fit);
    int32_t WaitKey(int32_t timeoutMs = 0);

    bool IsOpen() const;
    void Close();

    void SetTitle(const std::string& title);
    void Resize(int32_t width, int32_t height);
    void Move(int32_t x, int32_t y);

    static int32_t ShowImage(const QImage& image, const std::string& title = "QiVision");
};
```

---

### WaitKey

Waits for keyboard input.

```cpp
int32_t WaitKey(int32_t timeoutMs = 0);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| timeoutMs | int32_t | Timeout in ms (0=infinite, -1=no wait) |

**Returns**
| Type | Description |
|------|-------------|
| int32_t | Key code, or -1 on timeout/close |

---

### Global Functions

Halcon-style global display functions.

```cpp
void DispImage(const QImage& image, const std::string& windowName = "QiVision");
int32_t WaitKey(int32_t timeoutMs = 0);
void CloseWindow(const std::string& windowName);
void CloseAllWindows();
```

---

## Appendix

### A. PixelType

```cpp
enum class PixelType {
    UInt8,      // 8-bit unsigned (0-255)
    UInt16,     // 16-bit unsigned (0-65535)
    Int16,      // 16-bit signed
    Float32     // 32-bit float
};
```

### B. ChannelType

```cpp
enum class ChannelType {
    Gray,       // 1 channel
    RGB,        // 3 channels
    BGR,        // 3 channels (OpenCV order)
    RGBA,       // 4 channels
    BGRA        // 4 channels
};
```

### C. Angle Conversion Macros

```cpp
#define RAD(deg) ((deg) * 0.017453292519943295)  // Degrees to radians
#define DEG(rad) ((rad) * 57.29577951308232)     // Radians to degrees
```

### D. Border Handling Modes

| Mode | Description | Example (accessing x=-1) |
|------|-------------|--------------------------|
| `reflect` | Mirror reflection | `image[1]` |
| `replicate` | Replicate edge | `image[0]` |
| `constant` | Constant padding | `0` |
| `wrap` | Circular | `image[width-1]` |

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 0.5.0 | 2026-01-24 | Rewrite to OpenCV-style format, add Segment module documentation |
| 0.4.0 | 2026-01-21 | Unified Halcon-style API signatures |
| 0.3.0 | 2026-01-20 | Added Metrology module, auto-threshold API |
| 0.2.0 | 2026-01-17 | Added Blob, Display, GUI modules |
| 0.1.0 | 2026-01-15 | Initial: Matching, Measure, IO, Color, Filter |
