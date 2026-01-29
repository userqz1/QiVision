# QiVision API Reference

> Version: 0.11.0
> Last Updated: 2026-01-29
> Namespace: `Qi::Vision`

Professional industrial machine vision library.

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
10. [Morphology](#10-morphology) - Morphological operations
11. [Calib](#11-calib) - Camera calibration and distortion correction
12. [Transform](#12-transform) - Geometric transformations (Polar, Affine, Homography)
13. [Edge](#13-edge) - Edge detection (Canny, Steger)
14. [Appendix](#appendix) - Types and constants

---

## 1. Matching

**Namespace**: `Qi::Vision::Matching`
**Header**: `<QiVision/Matching/ShapeModel.h>`

Shape-based template matching using gradient orientation features.

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

**Methods**
| Method | Returns | Description |
|--------|---------|-------------|
| IsValid() | bool | True if model contains valid data |

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

**Returns**
| Type | Description |
|------|-------------|
| std::vector<EdgeResult> | Detected edge results with fuzzy scoring |

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
                           const MetrologyMeasureParams& params = {});

    int32_t AddCircleMeasure(double row, double col, double radius,
                              double measureLength1, double measureLength2,
                              const std::string& transition = "all",
                              const std::string& select = "all",
                              const MetrologyMeasureParams& params = {});

    int32_t AddArcMeasure(double row, double col, double radius,
                           double angleStart, double angleEnd,
                           double measureLength1, double measureLength2,
                           const std::string& transition = "all",
                           const std::string& select = "all",
                           const MetrologyMeasureParams& params = {});

    int32_t AddEllipseMeasure(double row, double col, double phi,
                               double ra, double rb,
                               double measureLength1, double measureLength2,
                               const std::string& transition = "all",
                               const std::string& select = "all",
                               const MetrologyMeasureParams& params = {});

    int32_t AddRectangle2Measure(double row, double col, double phi,
                                  double length1, double length2,
                                  double measureLength1, double measureLength2,
                                  const std::string& transition = "all",
                                  const std::string& select = "all",
                                  const MetrologyMeasureParams& params = {});

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

**Method Returns**
| Method | Returns | Description |
|--------|---------|-------------|
| AddLineMeasure | int32_t | Object index (0-based) for later retrieval |
| AddCircleMeasure | int32_t | Object index (0-based) for later retrieval |
| AddArcMeasure | int32_t | Object index (0-based) for later retrieval |
| AddEllipseMeasure | int32_t | Object index (0-based) for later retrieval |
| AddRectangle2Measure | int32_t | Object index (0-based) for later retrieval |
| Apply | bool | True if measurement succeeded |
| GetLineResult | MetrologyLineResult | Fitted line result for object at index |
| GetCircleResult | MetrologyCircleResult | Fitted circle result for object at index |
| GetEllipseResult | MetrologyEllipseResult | Fitted ellipse result for object at index |
| GetRectangle2Result | MetrologyRectangle2Result | Fitted rectangle result for object at index |
| GetMeasuredPoints | std::vector<Point2d> | Edge points used for fitting |
| GetPointWeights | std::vector<double> | Weight of each edge point (0-1) |

---

### MetrologyMeasureParams

Measurement parameters for metrology objects.

```cpp
struct MetrologyMeasureParams {
    int32_t numInstances = 1;           // Number of instances to find
    double measureLength1 = 20.0;       // Half-length of caliper along profile
    double measureLength2 = 5.0;        // Half-width of caliper perpendicular
    double measureSigma = 1.0;          // Gaussian smoothing sigma
    double measureThreshold = 30.0;     // Edge amplitude threshold
    ThresholdMode thresholdMode = ThresholdMode::Manual;  // Manual or Auto
    EdgeTransition measureTransition = EdgeTransition::All;
    EdgeSelectMode measureSelect = EdgeSelectMode::All;
    int32_t numMeasures = 10;           // Number of calipers per object
    double minScore = 0.7;              // Minimum score threshold
    MetrologyFitMethod fitMethod = MetrologyFitMethod::RANSAC;
    double distanceThreshold = 3.5;     // Outlier distance threshold (pixels)
    int32_t maxIterations = -1;         // Max RANSAC iterations (-1 = unlimited)
    int32_t randSeed = 42;              // Random seed for RANSAC

    // Builder pattern setters
    MetrologyMeasureParams& SetNumInstances(int32_t n);
    MetrologyMeasureParams& SetMeasureLength(double l1, double l2);
    MetrologyMeasureParams& SetMeasureSigma(double s);
    MetrologyMeasureParams& SetThreshold(double t);          // Manual mode
    MetrologyMeasureParams& SetThreshold(const std::string& mode);  // "auto" for Auto mode
    MetrologyMeasureParams& SetMeasureTransition(EdgeTransition t);
    MetrologyMeasureParams& SetMeasureSelect(EdgeSelectMode m);
    MetrologyMeasureParams& SetNumMeasures(int32_t n);
    MetrologyMeasureParams& SetMinScore(double s);
    MetrologyMeasureParams& SetFitMethod(MetrologyFitMethod m);
    MetrologyMeasureParams& SetFitMethod(const std::string& method);  // "ransac", "huber", "tukey"
    MetrologyMeasureParams& SetDistanceThreshold(double t);
    MetrologyMeasureParams& SetMaxIterations(int32_t n);
    MetrologyMeasureParams& SetRandSeed(int32_t s);
};
```

---

### MetrologyParamFlag (deprecated)

Parameter flags. **Deprecated: Use MetrologyMeasureParams instead.**

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

**Returns (WriteSequence)**
| Type | Description |
|------|-------------|
| int32_t | Number of images successfully written |

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

**Returns**
| Type | Description |
|------|-------------|
| QImage | Mean-filtered output image |

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

**Returns**
| Type | Description |
|------|-------------|
| QImage | Median-filtered output image |

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

**Returns**
| Type | Description |
|------|-------------|
| QImage | Edge-preserving filtered output image |

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

**Returns**
| Type | Description |
|------|-------------|
| QImage (SobelAmp) | Gradient amplitude image |
| QImage (SobelDir) | Gradient direction image (angle in radians) |

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

**Returns**
| Type | Description |
|------|-------------|
| QImage | Gaussian derivative response image |

---

### Laplace / LaplacianOfGaussian

Laplacian operators.

```cpp
QImage Laplace(const QImage& image, const std::string& filterType = "3x3");
QImage LaplacianOfGaussian(const QImage& image, double sigma);
```

**Returns**
| Type | Description |
|------|-------------|
| QImage (Laplace) | Laplacian response image (edge enhancement) |
| QImage (LaplacianOfGaussian) | LoG response image (blob detection) |

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

**Returns**
| Type | Description |
|------|-------------|
| QImage | Sharpened output image |

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

**Returns**
| Type | Description |
|------|-------------|
| QImage | Edge-preserving smoothed output image |

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

**Returns**
| Type | Description |
|------|-------------|
| QRegion | Region containing pixels with variance above threshold |

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

**Returns**
| Type | Description |
|------|-------------|
| QRegion | Region containing character/text pixels |

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

**Returns**
| Type | Description |
|------|-------------|
| QRegion | Region containing connected pixels meeting hysteresis criteria |

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

**Returns (MaskToRegion)**
| Type | Description |
|------|-------------|
| QRegion | Region containing pixels above threshold |

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

**Returns**
| Type | Description |
|------|-------------|
| int32_t (CountObj) | Number of regions in the array |
| QRegion (SelectObj) | Region at the specified index |

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

**Returns**
| Type | Description |
|------|-------------|
| int32_t (CountHoles) | Number of holes in the region |
| int32_t (EulerNumber) | Euler number (components - holes) |

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

Window class for image display and interaction.

```cpp
class Window {
public:
    Window(const std::string& title = "QiVision", int32_t width = 0, int32_t height = 0);
    ~Window();

    // Display
    void DispImage(const QImage& image, ScaleMode scaleMode = ScaleMode::Fit);
    int32_t WaitKey(int32_t timeoutMs = 0);

    // Window control
    bool IsOpen() const;
    void Close();
    void SetTitle(const std::string& title);
    void Resize(int32_t width, int32_t height);
    void Move(int32_t x, int32_t y);

    // Auto resize for large images
    void SetAutoResize(bool enable, int32_t maxWidth = 0, int32_t maxHeight = 0);
    bool IsAutoResize() const;

    // Mouse interaction
    void SetMouseCallback(MouseCallback callback);
    bool GetMousePosition(int32_t& x, int32_t& y) const;
    bool GetMouseImagePosition(double& imageX, double& imageY) const;

    // Interactive ROI drawing
    ROIResult DrawRectangle();   // Click and drag to draw
    ROIResult DrawCircle();      // Click center, drag radius
    ROIResult DrawLine();
    ROIResult DrawPolygon();     // Click points, double-click to close
    ROIResult DrawPoint();
    ROIResult DrawROI(ROIType type);

    // Static helpers
    static int32_t ShowImage(const QImage& image, const std::string& title = "QiVision");
};
```

**Method Returns**
| Method | Returns | Description |
|--------|---------|-------------|
| IsOpen() | bool | True if window is currently open |
| WaitKey() | int32_t | Key code pressed, or -1 on timeout/close |
| IsAutoResize() | bool | True if auto-resize is enabled |
| GetMousePosition() | bool | True if mouse position is valid |
| GetMouseImagePosition() | bool | True if mouse is over image area |
| DrawRectangle() | ROIResult | User-drawn rectangle region |
| DrawCircle() | ROIResult | User-drawn circle region |
| DrawLine() | ROIResult | User-drawn line |
| DrawPolygon() | ROIResult | User-drawn polygon region |
| DrawPoint() | ROIResult | User-selected point |
| DrawROI() | ROIResult | User-drawn ROI of specified type |
| ShowImage() | int32_t | Key code pressed after display |

**ROIResult Structure**
```cpp
struct ROIResult {
    ROIType type;
    bool valid;                      // True if completed (not cancelled)
    double row1, col1, row2, col2;   // Rectangle bounds
    double centerRow, centerCol;     // Circle/Ellipse center
    double radius;                   // Circle radius
    std::vector<Point2d> points;     // Polygon points
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

Global display functions.

```cpp
void DispImage(const QImage& image, const std::string& windowName = "QiVision");
int32_t WaitKey(int32_t timeoutMs = 0);
void CloseWindow(const std::string& windowName);
void CloseAllWindows();
```

---

## 10. Morphology

**Namespace**: `Qi::Vision::Morphology`
**Header**: `<QiVision/Morphology/Morphology.h>`

Binary and gray-scale morphological operations.

---

### StructuringElement

Structuring element for morphological operations.

```cpp
class StructuringElement {
public:
    // Factory methods
    static StructuringElement Rectangle(int32_t width, int32_t height);
    static StructuringElement Square(int32_t size);
    static StructuringElement Circle(int32_t radius);
    static StructuringElement Ellipse(int32_t radiusX, int32_t radiusY);
    static StructuringElement Cross(int32_t armLength, int32_t thickness = 1);
    static StructuringElement Diamond(int32_t radius);
    static StructuringElement Line(int32_t length, double angle);
    static StructuringElement FromMask(const QImage& mask, int32_t anchorX = -1, int32_t anchorY = -1);
    static StructuringElement FromRegion(const QRegion& region, int32_t anchorX = -1, int32_t anchorY = -1);

    // Properties
    bool Empty() const;
    int32_t Width() const;
    int32_t Height() const;
    size_t PixelCount() const;

    // Transformations
    StructuringElement Reflect() const;
    StructuringElement Rotate(double angle) const;
};
```

**Factory Methods**
| Method | Description |
|--------|-------------|
| Rectangle(w, h) | Rectangular SE of width×height |
| Square(s) | Square SE of size s |
| Circle(r) | Circular SE of radius r |
| Ellipse(rx, ry) | Elliptical SE |
| Cross(len, thick) | Cross-shaped SE |
| Diamond(r) | Diamond (rhombus) SE |
| Line(len, angle) | Line segment SE |
| FromMask(mask) | Custom SE from binary mask |
| FromRegion(region) | Custom SE from QRegion |

---

### Binary Morphology (Region)

Operations on QRegion objects.

#### Dilation

Dilates a region with a structuring element.

```cpp
QRegion Dilation(const QRegion& region, const StructuringElement& se);
QRegion DilationCircle(const QRegion& region, int32_t radius);
QRegion DilationRectangle(const QRegion& region, int32_t width, int32_t height);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| region | const QRegion& | Input region |
| se | const StructuringElement& | Structuring element |
| radius | int32_t | Circle radius |
| width, height | int32_t | Rectangle dimensions |

**Returns**
| Type | Description |
|------|-------------|
| QRegion | Dilated region |

---

#### Erosion

Erodes a region with a structuring element.

```cpp
QRegion Erosion(const QRegion& region, const StructuringElement& se);
QRegion ErosionCircle(const QRegion& region, int32_t radius);
QRegion ErosionRectangle(const QRegion& region, int32_t width, int32_t height);
```

---

#### Opening

Erosion followed by dilation. Removes small protrusions.

```cpp
QRegion Opening(const QRegion& region, const StructuringElement& se);
QRegion OpeningCircle(const QRegion& region, int32_t radius);
QRegion OpeningRectangle(const QRegion& region, int32_t width, int32_t height);
```

---

#### Closing

Dilation followed by erosion. Fills small holes.

```cpp
QRegion Closing(const QRegion& region, const StructuringElement& se);
QRegion ClosingCircle(const QRegion& region, int32_t radius);
QRegion ClosingRectangle(const QRegion& region, int32_t width, int32_t height);
```

---

#### Derived Operations

```cpp
QRegion Boundary(const QRegion& region, const std::string& type = "both");
QRegion Skeleton(const QRegion& region);
QRegion Thinning(const QRegion& region, int32_t maxIterations = 0);
QRegion PruneSkeleton(const QRegion& skeleton, int32_t iterations = 1);
QRegion FillUp(const QRegion& region);
QRegion ClearBorder(const QRegion& region, const Rect2i& bounds);
```

| Function | Description |
|----------|-------------|
| Boundary | Region boundary ("inner", "outer", or "both") |
| Skeleton | Medial axis extraction |
| Thinning | Iterative thinning |
| PruneSkeleton | Remove skeleton branches |
| FillUp | Fill holes in region |
| ClearBorder | Remove border-touching regions |

---

### Gray Morphology (Image)

Operations on QImage objects.

#### GrayDilation

Grayscale dilation (local maximum).

```cpp
void GrayDilation(const QImage& image, QImage& output, const StructuringElement& se);
void GrayDilationCircle(const QImage& image, QImage& output, int32_t radius);
void GrayDilationRectangle(const QImage& image, QImage& output, int32_t width, int32_t height);
```

---

#### GrayErosion

Grayscale erosion (local minimum).

```cpp
void GrayErosion(const QImage& image, QImage& output, const StructuringElement& se);
void GrayErosionCircle(const QImage& image, QImage& output, int32_t radius);
void GrayErosionRectangle(const QImage& image, QImage& output, int32_t width, int32_t height);
```

---

#### GrayOpening / GrayClosing

Compound operations.

```cpp
void GrayOpening(const QImage& image, QImage& output, const StructuringElement& se);
void GrayOpeningCircle(const QImage& image, QImage& output, int32_t radius);
void GrayClosing(const QImage& image, QImage& output, const StructuringElement& se);
void GrayClosingCircle(const QImage& image, QImage& output, int32_t radius);
```

---

#### Top-Hat Transforms

Extract small features.

```cpp
void GrayTopHat(const QImage& image, QImage& output, const StructuringElement& se);
void GrayTopHatCircle(const QImage& image, QImage& output, int32_t radius);
void GrayBlackHat(const QImage& image, QImage& output, const StructuringElement& se);
void GrayBlackHatCircle(const QImage& image, QImage& output, int32_t radius);
```

| Function | Formula | Description |
|----------|---------|-------------|
| GrayTopHat | image - opening | Extract bright features |
| GrayBlackHat | closing - image | Extract dark features |

---

#### Additional Operations

```cpp
void GrayGradient(const QImage& image, QImage& output, const StructuringElement& se);
void GrayRange(const QImage& image, QImage& output, int32_t width, int32_t height);
void RollingBall(const QImage& image, QImage& output, int32_t radius);
void GrayReconstructDilation(const QImage& marker, const QImage& mask, QImage& output);
void GrayReconstructErosion(const QImage& marker, const QImage& mask, QImage& output);
void GrayFillHoles(const QImage& image, QImage& output);
```

| Function | Description |
|----------|-------------|
| GrayGradient | Morphological gradient (dilation - erosion) |
| GrayRange | Local contrast (max - min) |
| RollingBall | Background subtraction |
| GrayReconstructDilation | Reconstruction by dilation |
| GrayReconstructErosion | Reconstruction by erosion |
| GrayFillHoles | Fill dark holes |

---

### Convenience Functions

```cpp
StructuringElement SE_Cross3();   // 3x3 cross (4-connected)
StructuringElement SE_Square3();  // 3x3 square (8-connected)
StructuringElement SE_Disk5();    // 5x5 disk
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

## 11. Calib

**Namespace**: `Qi::Vision::Calib`
**Headers**: `<QiVision/Calib/CameraModel.h>`, `<QiVision/Calib/Undistort.h>`, `<QiVision/Calib/CalibBoard.h>`

Camera calibration, distortion correction, and coordinate transformations.

---

### CameraIntrinsics

Camera intrinsic parameters.

```cpp
struct CameraIntrinsics {
    double fx = 1.0;    // Focal length X (pixels)
    double fy = 1.0;    // Focal length Y (pixels)
    double cx = 0.0;    // Principal point X (pixels)
    double cy = 0.0;    // Principal point Y (pixels)

    Internal::Mat33 ToMatrix() const;
    static CameraIntrinsics FromMatrix(const Internal::Mat33& K);
};
```

**Methods**
| Method | Returns | Description |
|--------|---------|-------------|
| ToMatrix() | Internal::Mat33 | 3x3 intrinsic matrix K |
| FromMatrix() | CameraIntrinsics | Create from 3x3 matrix |

---

### DistortionCoeffs

Lens distortion coefficients (Brown-Conrady model).

```cpp
struct DistortionCoeffs {
    double k1 = 0.0;    // Radial distortion k1
    double k2 = 0.0;    // Radial distortion k2
    double k3 = 0.0;    // Radial distortion k3
    double p1 = 0.0;    // Tangential distortion p1
    double p2 = 0.0;    // Tangential distortion p2

    bool IsZero() const;
};
```

---

### CameraModel

Complete camera model with intrinsics and distortion.

```cpp
class CameraModel {
public:
    CameraModel();
    CameraModel(const CameraIntrinsics& intr, const DistortionCoeffs& dist,
                const Size2i& imgSize = Size2i(0, 0));

    const CameraIntrinsics& Intrinsics() const;
    const DistortionCoeffs& Distortion() const;
    const Size2i& ImageSize() const;

    Point2d Distort(const Point2d& normalized) const;
    Point2d Undistort(const Point2d& distorted, int maxIterations = 10) const;
    Point2d ProjectPoint(const Point3d& p3d) const;
    Point3d UnprojectPixel(const Point2d& pixel) const;

    bool IsValid() const;
};
```

**Methods**
| Method | Returns | Description |
|--------|---------|-------------|
| Distort() | Point2d | Apply distortion to normalized point |
| Undistort() | Point2d | Remove distortion (iterative) |
| ProjectPoint() | Point2d | Project 3D point to 2D pixel |
| UnprojectPixel() | Point3d | Unproject pixel to 3D ray |

---

### UndistortMap

Pre-computed undistortion map for efficient batch processing.

```cpp
struct UndistortMap {
    std::vector<float> mapX;    // Source X coordinates
    std::vector<float> mapY;    // Source Y coordinates
    int32_t width = 0;
    int32_t height = 0;

    bool IsValid() const;
};
```

---

### Undistort

Removes lens distortion from image.

```cpp
void Undistort(
    const QImage& src,
    QImage& dst,
    const CameraModel& camera,
    Internal::InterpolationMethod method = Internal::InterpolationMethod::Bilinear
);

void Undistort(
    const QImage& src,
    QImage& dst,
    const CameraModel& camera,
    const CameraIntrinsics& newCameraMatrix,
    const Size2i& outputSize = Size2i(0, 0),
    Internal::InterpolationMethod method = Internal::InterpolationMethod::Bilinear
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| src | const QImage& | Source (distorted) image |
| dst | QImage& | [out] Output (undistorted) image |
| camera | const CameraModel& | Camera model with intrinsics and distortion |
| newCameraMatrix | const CameraIntrinsics& | New camera matrix (adjusts FOV) |
| outputSize | const Size2i& | Output size (0 = same as source) |
| method | InterpolationMethod | Bilinear (default), Nearest, or Bicubic |

---

### InitUndistortMap

Pre-computes undistortion map for efficient batch processing.

```cpp
UndistortMap InitUndistortMap(
    const CameraModel& camera,
    const Size2i& outputSize,
    const CameraIntrinsics* newCameraMatrix = nullptr
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| camera | const CameraModel& | Camera model |
| outputSize | const Size2i& | Output image size |
| newCameraMatrix | const CameraIntrinsics* | Optional new camera matrix |

**Returns**
| Type | Description |
|------|-------------|
| UndistortMap | Pre-computed mapping coordinates |

---

### Remap

Applies remapping using pre-computed map.

```cpp
void Remap(
    const QImage& src,
    QImage& dst,
    const UndistortMap& map,
    Internal::InterpolationMethod method = Internal::InterpolationMethod::Bilinear,
    Internal::BorderMode borderMode = Internal::BorderMode::Constant,
    double borderValue = 0.0
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| src | const QImage& | Source image |
| dst | QImage& | [out] Output image |
| map | const UndistortMap& | Pre-computed undistortion map |
| borderMode | BorderMode | Border handling |
| borderValue | double | Value for constant border mode |

---

### GetOptimalNewCameraMatrix

Computes optimal new camera matrix for undistortion.

```cpp
CameraIntrinsics GetOptimalNewCameraMatrix(
    const CameraModel& camera,
    double alpha = 1.0,
    const Size2i& newImageSize = Size2i(0, 0)
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| camera | const CameraModel& | Original camera model |
| alpha | double | Free scaling (0=all valid, 1=keep all source) |
| newImageSize | const Size2i& | New image size (0,0 = same as original) |

**Returns**
| Type | Description |
|------|-------------|
| CameraIntrinsics | Optimal camera matrix balancing FOV and valid pixels |

---

### UndistortPoint / UndistortPoints

Undistorts point(s) using camera model.

```cpp
Point2d UndistortPoint(const Point2d& point, const CameraModel& camera);

std::vector<Point2d> UndistortPoints(
    const std::vector<Point2d>& points,
    const CameraModel& camera
);
```

**Returns**
| Type | Description |
|------|-------------|
| Point2d | Undistorted pixel coordinate |
| std::vector<Point2d> | Undistorted pixel coordinates |

---

### DistortPoint

Applies distortion to a point (for simulation).

```cpp
Point2d DistortPoint(const Point2d& point, const CameraModel& camera);
```

**Returns**
| Type | Description |
|------|-------------|
| Point2d | Distorted pixel coordinate |

---

### CornerGrid

Result of chessboard corner detection.

```cpp
struct CornerGrid {
    std::vector<Point2d> corners;   // Detected corners in row-major order
    int32_t rows = 0;               // Pattern rows (inner corners)
    int32_t cols = 0;               // Pattern columns (inner corners)
    bool found = false;             // Whether pattern was fully detected

    Point2d At(int32_t row, int32_t col) const;
    size_t Count() const;
    bool IsValid() const;
};
```

**Methods**
| Method | Returns | Description |
|--------|---------|-------------|
| At(row, col) | Point2d | Get corner at grid position |
| Count() | size_t | Total number of corners |
| IsValid() | bool | Check if grid is valid |

---

### FindChessboardCorners

Detects chessboard inner corners in image.

```cpp
CornerGrid FindChessboardCorners(
    const QImage& image,
    int32_t patternCols,
    int32_t patternRows,
    ChessboardFlags flags = ChessboardFlags::AdaptiveThresh | ChessboardFlags::NormalizeImage
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| image | const QImage& | Input grayscale image |
| patternCols | int32_t | Number of inner corners per row |
| patternRows | int32_t | Number of inner corners per column |
| flags | ChessboardFlags | Detection flags |

**Returns**
| Type | Description |
|------|-------------|
| CornerGrid | Detected corners (empty if not found) |

**ChessboardFlags**
| Flag | Description |
|------|-------------|
| None | No special processing |
| AdaptiveThresh | Use adaptive thresholding |
| NormalizeImage | Normalize image before detection |
| FilterQuads | Filter out false quads |
| FastCheck | Quick check if chessboard present |

---

### CornerSubPix

Refines corner positions to subpixel accuracy.

```cpp
void CornerSubPix(
    const QImage& image,
    std::vector<Point2d>& corners,
    int32_t winSize = 5,
    int32_t maxIterations = 30,
    double epsilon = 0.001
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| image | const QImage& | Input grayscale image |
| corners | std::vector<Point2d>& | [in/out] Corner positions |
| winSize | int32_t | Half-size of search window |
| maxIterations | int32_t | Maximum refinement iterations |
| epsilon | double | Convergence threshold |

---

### GenerateChessboardPoints

Generates ideal chessboard corner positions in object space.

```cpp
std::vector<Point3d> GenerateChessboardPoints(
    int32_t patternCols,
    int32_t patternRows,
    double squareSize = 1.0
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| patternCols | int32_t | Number of inner corners per row |
| patternRows | int32_t | Number of inner corners per column |
| squareSize | double | Size of each square |

**Returns**
| Type | Description |
|------|-------------|
| std::vector<Point3d> | 3D corner positions at z=0 |

---

### DrawChessboardCorners

Draws detected corners on image.

```cpp
void DrawChessboardCorners(
    QImage& image,
    const CornerGrid& grid,
    bool drawOrder = true
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| image | QImage& | [in/out] Image to draw on |
| grid | const CornerGrid& | Detected corner grid |
| drawOrder | bool | Draw lines connecting corners |

---

## 12. Transform

**Namespace**: `Qi::Vision::Transform`
**Header**: `<QiVision/Transform/PolarTransform.h>`

Polar coordinate transformations for circular object inspection and rotation-invariant processing.

---

### PolarMode

Polar transformation mapping mode.

```cpp
enum class PolarMode {
    Linear,     // Linear radial mapping: output_y proportional to radius
    SemiLog     // Semi-log radial mapping: enhanced detail near center
};
```

---

### PolarInterpolation

Interpolation method for transformation.

```cpp
enum class PolarInterpolation {
    Nearest,    // Nearest neighbor (fast, pixelated)
    Bilinear,   // Bilinear interpolation (default, good quality)
    Bicubic     // Bicubic interpolation (best quality, slower)
};
```

---

### CartesianToPolar

Transforms image from Cartesian to Polar coordinates.

Maps a circular region centered at 'center' to a rectangular image where:
- X axis = angle [0, 2*pi)
- Y axis = radius [0, maxRadius]

```cpp
void CartesianToPolar(
    const QImage& src,
    QImage& dst,
    const Point2d& center,
    double maxRadius,
    int32_t dstWidth = 0,
    int32_t dstHeight = 0,
    PolarMode mode = PolarMode::Linear,
    PolarInterpolation interp = PolarInterpolation::Bilinear
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| src | const QImage& | Source image (grayscale) |
| dst | QImage& | [out] Output polar image |
| center | const Point2d& | Center point of polar transformation |
| maxRadius | double | Maximum radius to include |
| dstWidth | int32_t | Output width (angle resolution), 0 = auto |
| dstHeight | int32_t | Output height (radial resolution), 0 = auto |
| mode | PolarMode | Mapping mode (Linear or SemiLog) |
| interp | PolarInterpolation | Interpolation method |

**Use Cases**
- Inspecting circular objects (defects become horizontal lines)
- Analyzing ring-shaped patterns
- Rotation-invariant processing

---

### PolarToCartesian

Transforms image from Polar back to Cartesian coordinates.

Inverse of CartesianToPolar. Maps a polar image back to Cartesian space.

```cpp
void PolarToCartesian(
    const QImage& src,
    QImage& dst,
    const Point2d& center,
    double maxRadius,
    int32_t dstWidth = 0,
    int32_t dstHeight = 0,
    PolarMode mode = PolarMode::Linear,
    PolarInterpolation interp = PolarInterpolation::Bilinear
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| src | const QImage& | Source polar image |
| dst | QImage& | [out] Output Cartesian image |
| center | const Point2d& | Center point for output image |
| maxRadius | double | Maximum radius that was used in forward transform |
| dstWidth | int32_t | Output width, 0 = 2 * maxRadius |
| dstHeight | int32_t | Output height, 0 = 2 * maxRadius |
| mode | PolarMode | Mapping mode (must match forward transform) |
| interp | PolarInterpolation | Interpolation method |

---

### WarpPolar

General polar warp function combining forward and inverse transforms.

```cpp
void WarpPolar(
    const QImage& src,
    QImage& dst,
    const Point2d& center,
    double maxRadius,
    int32_t dstWidth = 0,
    int32_t dstHeight = 0,
    PolarMode mode = PolarMode::Linear,
    bool inverse = false,
    PolarInterpolation interp = PolarInterpolation::Bilinear
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| src | const QImage& | Source image |
| dst | QImage& | [out] Output image |
| center | const Point2d& | Center point for polar transformation |
| maxRadius | double | Maximum radius |
| dstWidth | int32_t | Output width (0 = auto) |
| dstHeight | int32_t | Output height (0 = auto) |
| mode | PolarMode | Mapping mode |
| inverse | bool | If true, Polar->Cartesian; if false, Cartesian->Polar |
| interp | PolarInterpolation | Interpolation method |

---

### PointCartesianToPolar

Converts a single point from Cartesian to Polar coordinates.

```cpp
Point2d PointCartesianToPolar(const Point2d& pt, const Point2d& center);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| pt | const Point2d& | Point in Cartesian coordinates |
| center | const Point2d& | Center of polar system |

**Returns**
| Type | Description |
|------|-------------|
| Point2d | Point where x = angle (radians, [0, 2*pi)), y = radius |

---

### PointPolarToCartesian

Converts a single point from Polar to Cartesian coordinates.

```cpp
Point2d PointPolarToCartesian(double angle, double radius, const Point2d& center);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| angle | double | Angle in radians |
| radius | double | Radius |
| center | const Point2d& | Center of polar system |

**Returns**
| Type | Description |
|------|-------------|
| Point2d | Point in Cartesian coordinates |

---

## 12.2 Affine Transforms

**Namespace**: `Qi::Vision::Transform`
**Header**: `<QiVision/Transform/AffineTransform.h>`

Affine transformation operations for images and points.

---

### AffineTransImage

Applies affine transformation to an image.

```cpp
void AffineTransImage(
    const QImage& src,
    QImage& dst,
    const QMatrix& matrix,
    const std::string& interpolation = "bilinear",
    const std::string& borderMode = "constant",
    double borderValue = 0.0
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| src | const QImage& | Source image |
| dst | QImage& | [out] Output transformed image |
| matrix | const QMatrix& | 2D affine transformation matrix |
| interpolation | const std::string& | "nearest", "bilinear" (default), "bicubic" |
| borderMode | const std::string& | "constant" (default), "replicate", "reflect", "wrap" |
| borderValue | double | Border value for constant mode |

---

### RotateImage

Rotates image around its center or specified point.

```cpp
void RotateImage(
    const QImage& src,
    QImage& dst,
    double angle,
    const std::string& interpolation = "bilinear"
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| angle | double | Rotation angle in radians (positive = counter-clockwise) |

---

### ScaleImage / ZoomImageSize

Scales image by factors or to specified size.

```cpp
void ScaleImage(const QImage& src, QImage& dst, double scaleX, double scaleY, const std::string& interpolation = "bilinear");
void ZoomImageSize(const QImage& src, QImage& dst, int32_t dstWidth, int32_t dstHeight, const std::string& interpolation = "bilinear");
```

---

### Matrix Creation Functions

Halcon-style matrix creation functions.

| Function | Description |
|----------|-------------|
| HomMat2dIdentity() | Creates identity matrix |
| HomMat2dRotate(phi, cy, cx) | Creates rotation matrix around (cx, cy) |
| HomMat2dScale(sy, sx, cy, cx) | Creates scaling matrix around (cx, cy) |
| HomMat2dTranslate(mat, ty, tx) | Adds translation to existing matrix |
| HomMat2dCompose(m1, m2) | Composes two matrices (m1 * m2) |
| HomMat2dInvert(mat) | Inverts transformation matrix |

---

### AffineTransPoint2d

Transforms points using affine matrix.

```cpp
Point2d AffineTransPoint2d(const QMatrix& homMat2d, const Point2d& point);
std::vector<Point2d> AffineTransPoint2d(const QMatrix& homMat2d, const std::vector<Point2d>& points);
```

---

### Transform Estimation

Estimates transformation from point correspondences.

| Function | Min Points | Description |
|----------|------------|-------------|
| VectorToHomMat2d | 3 | Full affine transform |
| VectorToRigid | 2 | Rotation + translation only |
| VectorToSimilarity | 2 | Rotation + translation + uniform scale |

---

## 12.3 Projective Transforms (Homography)

**Namespace**: `Qi::Vision::Transform`
**Header**: `<QiVision/Transform/Homography.h>`

Projective (perspective) transformation operations.

---

### HomMat3d

3x3 homography matrix class for projective transformations.

```cpp
class HomMat3d {
public:
    HomMat3d();  // Identity
    static HomMat3d Identity();
    static HomMat3d FromAffine(const QMatrix& affine);

    HomMat3d operator*(const HomMat3d& other) const;
    HomMat3d Inverse() const;

    Point2d Transform(const Point2d& p) const;
};
```

---

### ProjectiveTransImage

Applies projective (perspective) transformation to an image.

```cpp
void ProjectiveTransImage(
    const QImage& src,
    QImage& dst,
    const HomMat3d& homMat3d,
    const std::string& interpolation = "bilinear",
    const std::string& borderMode = "constant",
    double borderValue = 0.0
);
```

---

### Homography Estimation

```cpp
bool VectorToProjHomMat2d(const std::vector<Point2d>& srcPoints, const std::vector<Point2d>& dstPoints, HomMat3d& homMat3d);
bool ProjMatchPointsRansac(const std::vector<Point2d>& srcPoints, const std::vector<Point2d>& dstPoints, HomMat3d& homMat3d, double distanceThreshold = 3.0, double confidence = 0.99, int32_t maxIterations = 2000, std::vector<bool>* inlierMask = nullptr);
```

| Function | Min Points | Description |
|----------|------------|-------------|
| VectorToProjHomMat2d | 4 | DLT homography estimation |
| ProjMatchPointsRansac | 4 | RANSAC robust estimation |

---

### Quadrilateral Rectification

```cpp
bool RectifyQuadrilateral(const std::array<Point2d, 4>& quadPoints, double width, double height, HomMat3d& homMat3d);
bool RectangleToQuadrilateral(double width, double height, const std::array<Point2d, 4>& quadPoints, HomMat3d& homMat3d);
```

---

## 13. Edge

**Namespace**: `Qi::Vision::Edge`
**Header**: `<QiVision/Edge/Edge.h>`

Edge detection operations including Canny and Steger (subpixel line detection).

---

### EdgesImage

Detects edges using Canny algorithm (binary output).

```cpp
void EdgesImage(
    const QImage& image,
    QImage& edges,
    const std::string& filter = "canny",
    double sigma = 1.0,
    double lowThreshold = 20.0,
    double highThreshold = 40.0
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| image | const QImage& | Input grayscale image |
| edges | QImage& | [out] Binary edge image (255 = edge, 0 = non-edge) |
| filter | const std::string& | Filter type: "canny", "canny_sobel", "canny_scharr" |
| sigma | double | Gaussian smoothing sigma |
| lowThreshold | double | Low threshold for hysteresis |
| highThreshold | double | High threshold for hysteresis |

---

### EdgesSubPix

Detects edges with subpixel accuracy (contour output).

```cpp
void EdgesSubPix(
    const QImage& image,
    QContourArray& contours,
    const std::string& filter = "canny",
    double sigma = 1.0,
    double lowThreshold = 20.0,
    double highThreshold = 40.0
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| image | const QImage& | Input grayscale image |
| contours | QContourArray& | [out] Edge contours (XLD format) |
| filter | const std::string& | Filter type |
| sigma | double | Gaussian smoothing sigma |
| lowThreshold | double | Low threshold for hysteresis |
| highThreshold | double | High threshold for hysteresis |

---

### EdgesSubPixAuto

Detects edges with auto-computed thresholds.

```cpp
void EdgesSubPixAuto(
    const QImage& image,
    QContourArray& contours,
    const std::string& filter = "canny",
    double sigma = 1.0
);
```

---

### LinesSubPix

Detects lines/edges with subpixel accuracy using Steger algorithm.

Uses Hessian matrix eigenvalue analysis to detect curvilinear structures
(lines, ridges, valleys) with high subpixel accuracy.

```cpp
void LinesSubPix(
    const QImage& image,
    QContourArray& contours,
    double sigma = 1.5,
    double lowThreshold = 3.0,
    double highThreshold = 8.0,
    const std::string& lightDark = "light"
);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| image | const QImage& | Input grayscale image |
| contours | QContourArray& | [out] Line contours (XLD format) |
| sigma | double | Gaussian sigma (controls line width sensitivity) |
| lowThreshold | double | Low threshold for hysteresis |
| highThreshold | double | High threshold for hysteresis |
| lightDark | const std::string& | Line polarity: "light", "dark", "all" |

---

### LinesSubPixAuto

Detects lines with auto-computed thresholds.

```cpp
void LinesSubPixAuto(
    const QImage& image,
    QContourArray& contours,
    double sigma = 1.5,
    const std::string& lightDark = "light"
);
```

---

### CannyEdgeParams

Parameters for Canny edge detection.

```cpp
struct CannyEdgeParams {
    double sigma = 1.0;
    double lowThreshold = 20.0;
    double highThreshold = 40.0;
    bool autoThreshold = false;
    std::string gradientOp = "sobel";
    bool subPixelRefinement = true;
    double minContourLength = 5.0;
    int32_t minContourPoints = 3;

    static CannyEdgeParams Auto(double sigma = 1.0);
    static CannyEdgeParams WithThresholds(double low, double high, double sigma = 1.0);
};
```

---

### StegerLineParams

Parameters for Steger line detection.

```cpp
struct StegerLineParams {
    double sigma = 1.5;
    double lowThreshold = 3.0;
    double highThreshold = 8.0;
    std::string lineType = "light";
    double minLength = 5.0;
    double maxGap = 2.0;
    double maxAngleDiff = 0.5;
    bool subPixelRefinement = true;

    static StegerLineParams Light(double sigma, double lowThr, double highThr);
    static StegerLineParams Dark(double sigma, double lowThr, double highThr);
};
```

---

### DetectEdges

Detects edges with full parameter control.

```cpp
void DetectEdges(
    const QImage& image,
    QContourArray& contours,
    const CannyEdgeParams& params
);
```

---

### DetectLines

Detects lines with full parameter control.

```cpp
void DetectLines(
    const QImage& image,
    QContourArray& contours,
    const StegerLineParams& params
);
```

---

### ComputeSigmaForLineWidth

Computes recommended sigma based on line width.

```cpp
double ComputeSigmaForLineWidth(double lineWidthPixels);
```

**Parameters**
| Name | Type | Description |
|------|------|-------------|
| lineWidthPixels | double | Expected line width in pixels |

**Returns**
| Type | Description |
|------|-------------|
| double | Recommended sigma value |

---

### EstimateThresholds

Estimates thresholds from image gradient statistics.

```cpp
void EstimateThresholds(
    const QImage& image,
    double sigma,
    double& lowThreshold,
    double& highThreshold
);
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 0.11.0 | 2026-01-29 | Add Affine/Homography Transform modules |
| 0.10.0 | 2026-01-29 | Add Edge module (Canny, Steger subpixel edge detection) |
| 0.9.0 | 2026-01-29 | Add Transform module (polar coordinate transformations) |
| 0.8.0 | 2026-01-28 | Add CalibBoard module (chessboard detection, corner refinement) |
| 0.7.0 | 2026-01-28 | Add Calib module (CameraModel, Undistort) |
| 0.6.0 | 2026-01-27 | Add Morphology module (binary + gray-scale) |
| 0.5.0 | 2026-01-24 | API format update, add Segment module, NCCModel |
| 0.4.0 | 2026-01-21 | Unified API signatures |
| 0.3.0 | 2026-01-20 | Added Metrology module, auto-threshold API |
| 0.2.0 | 2026-01-17 | Added Blob, Display, GUI modules |
| 0.1.0 | 2026-01-15 | Initial: Matching, Measure, IO, Color, Filter |
