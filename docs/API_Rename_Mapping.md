# QiVision API 规范与重构映射表

本文档定义 QiVision 公开 API 的统一规范，并追踪重构进度。

## 核心规范：Halcon 风格

### 参数顺序规则

```
函数名(输入参数..., 输出参数..., 配置参数...)
```

| 类型 | 规则 | 示例 |
|------|------|------|
| **输入** | `const T&`，在前 | `const QImage& input` |
| **输出** | `T&`，在输入之后 | `QImage& output` |
| **配置** | 值类型或 `const&`，在最后 | `double sigma` |

### API 类型分类

| 类型 | 签名风格 | 示例 |
|------|---------|------|
| **图像处理** | `void Func(const QImage& in, QImage& out, params)` | `GaussFilter`, `SobelAmp` |
| **多输出** | `void Func(const T& in, T& out1, T& out2, ...)` | `Decompose3`, `FindShapeModel` |
| **绘图** | `void Func(QImage& image, params)` | `DispLine`, `DispCircle` |
| **查询** | `T Func(const T& obj)` 或 `T Obj::Method() const` | `Area()`, `Width()` |
| **类型转换** | `T2 Func(const T1& in)` | `RegionToImage`, `ImageToRegion` |
| **构造** | `T Func(params)` 或 `T::Create(params)` | `GenRectangle1`, `QImage::FromFile` |

---

## 1. Filter 模块 - ✅ 已完成 (31 个函数)

### 平滑滤波

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 1 | `QImage GaussFilter(const QImage& image, double sigma)` | `void GaussFilter(const QImage& in, QImage& out, double sigma)` | ✅ |
| 2 | `QImage GaussFilter(const QImage& image, double sigmaX, double sigmaY, const std::string& borderMode)` | `void GaussFilter(const QImage& in, QImage& out, double sigmaX, double sigmaY, const std::string& borderMode)` | ✅ |
| 3 | `QImage GaussImage(const QImage& image, const std::string& size)` | `void GaussImage(const QImage& in, QImage& out, const std::string& size)` | ✅ |
| 4 | `QImage MeanImage(const QImage& image, int32_t width, int32_t height, const std::string& borderMode)` | `void MeanImage(const QImage& in, QImage& out, int32_t width, int32_t height, const std::string& borderMode)` | ✅ |
| 5 | `QImage MeanImage(const QImage& image, int32_t size, const std::string& borderMode)` | `void MeanImage(const QImage& in, QImage& out, int32_t size, const std::string& borderMode)` | ✅ |
| 6 | `QImage MedianImage(const QImage& image, const std::string& maskType, int32_t radius, const std::string& marginMode)` | `void MedianImage(const QImage& in, QImage& out, const std::string& maskType, int32_t radius, const std::string& marginMode)` | ✅ |
| 7 | `QImage MedianRect(const QImage& image, int32_t width, int32_t height)` | `void MedianRect(const QImage& in, QImage& out, int32_t width, int32_t height)` | ✅ |
| 8 | `QImage BilateralFilter(const QImage& image, double sigmaSpatial, double sigmaIntensity)` | `void BilateralFilter(const QImage& in, QImage& out, double sigmaSpatial, double sigmaIntensity)` | ✅ |
| 9 | `QImage BilateralFilter(const QImage& image, int32_t size, double sigmaSpatial, double sigmaIntensity)` | `void BilateralFilter(const QImage& in, QImage& out, int32_t size, double sigmaSpatial, double sigmaIntensity)` | ✅ |
| 10 | `QImage BinomialFilter(const QImage& image, int32_t width, int32_t height, const std::string& borderMode)` | `void BinomialFilter(const QImage& in, QImage& out, int32_t width, int32_t height, const std::string& borderMode)` | ✅ |

### 边缘检测/梯度

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 11 | `QImage SobelAmp(const QImage& image, const std::string& filterType, int32_t size)` | `void SobelAmp(const QImage& in, QImage& out, const std::string& filterType, int32_t size)` | ✅ |
| 12 | `QImage SobelDir(const QImage& image, const std::string& dirType, int32_t size)` | `void SobelDir(const QImage& in, QImage& out, const std::string& dirType, int32_t size)` | ✅ |
| 13 | `QImage PrewittAmp(const QImage& image, const std::string& filterType)` | `void PrewittAmp(const QImage& in, QImage& out, const std::string& filterType)` | ✅ |
| 14 | `QImage RobertsAmp(const QImage& image, const std::string& filterType)` | `void RobertsAmp(const QImage& in, QImage& out, const std::string& filterType)` | ✅ |
| 15 | `QImage DerivateGauss(const QImage& image, double sigma, const std::string& component)` | `void DerivateGauss(const QImage& in, QImage& out, double sigma, const std::string& component)` | ✅ |
| 16 | `QImage GradientMagnitude(const QImage& image, double sigma)` | `void GradientMagnitude(const QImage& in, QImage& out, double sigma)` | ✅ |
| 17 | `QImage GradientDirection(const QImage& image, double sigma)` | `void GradientDirection(const QImage& in, QImage& out, double sigma)` | ✅ |
| 18 | `QImage Laplace(const QImage& image, const std::string& filterType)` | `void Laplace(const QImage& in, QImage& out, const std::string& filterType)` | ✅ |
| 19 | `QImage LaplacianOfGaussian(const QImage& image, double sigma)` | `void LaplacianOfGaussian(const QImage& in, QImage& out, double sigma)` | ✅ |

### 增强/锐化

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 20 | `QImage HighpassImage(const QImage& image, int32_t width, int32_t height)` | `void HighpassImage(const QImage& in, QImage& out, int32_t width, int32_t height)` | ✅ |
| 21 | `QImage LowpassImage(const QImage& image, int32_t width, int32_t height)` | `void LowpassImage(const QImage& in, QImage& out, int32_t width, int32_t height)` | ✅ |
| 22 | `QImage EmphasizeImage(const QImage& image, int32_t width, int32_t height, double factor)` | `void EmphasizeImage(const QImage& in, QImage& out, int32_t width, int32_t height, double factor)` | ✅ |
| 23 | `QImage UnsharpMask(const QImage& image, double sigma, double amount, double threshold)` | `void UnsharpMask(const QImage& in, QImage& out, double sigma, double amount, double threshold)` | ✅ |
| 24 | `QImage ShockFilter(const QImage& image, int32_t iterations, double dt)` | `void ShockFilter(const QImage& in, QImage& out, int32_t iterations, double dt)` | ✅ |
| 25 | `QImage AnisoDiff(const QImage& image, const std::string& mode, double contrast, double theta, int32_t iterations)` | `void AnisoDiff(const QImage& in, QImage& out, const std::string& mode, double contrast, double theta, int32_t iterations)` | ✅ |

### 卷积/秩滤波

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 26 | `QImage ConvolImage(const QImage& image, const std::vector<double>& kernel, int32_t kw, int32_t kh, bool normalize, const std::string& borderMode)` | `void ConvolImage(const QImage& in, QImage& out, const std::vector<double>& kernel, int32_t kw, int32_t kh, bool normalize, const std::string& borderMode)` | ✅ |
| 27 | `QImage ConvolSeparable(const QImage& image, const std::vector<double>& kernelX, const std::vector<double>& kernelY, const std::string& borderMode)` | `void ConvolSeparable(const QImage& in, QImage& out, const std::vector<double>& kernelX, const std::vector<double>& kernelY, const std::string& borderMode)` | ✅ |
| 28 | `QImage RankImage(const QImage& image, int32_t width, int32_t height, int32_t rank)` | `void RankImage(const QImage& in, QImage& out, int32_t width, int32_t height, int32_t rank)` | ✅ |
| 29 | `QImage MinImage(const QImage& image, int32_t width, int32_t height)` | `void MinImage(const QImage& in, QImage& out, int32_t width, int32_t height)` | ✅ |
| 30 | `QImage MaxImage(const QImage& image, int32_t width, int32_t height)` | `void MaxImage(const QImage& in, QImage& out, int32_t width, int32_t height)` | ✅ |

### 纹理特征

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 31 | `QImage StdDevImage(const QImage& image, int32_t width, int32_t height)` | `void StdDevImage(const QImage& in, QImage& out, int32_t width, int32_t height)` | ✅ |
| 32 | `QImage VarianceImage(const QImage& image, int32_t width, int32_t height)` | `void VarianceImage(const QImage& in, QImage& out, int32_t width, int32_t height)` | ✅ |
| 33 | `QImage EntropyImage(const QImage& image, int32_t width, int32_t height, int32_t numBins)` | `void EntropyImage(const QImage& in, QImage& out, int32_t width, int32_t height, int32_t numBins)` | ✅ |

---

## 2. Color 模块 - ✅ 已完成 (30 个函数)

### 颜色空间转换

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 1 | `QImage TransFromRgb(const QImage& image, ColorSpace toSpace)` | `void TransFromRgb(const QImage& in, QImage& out, ColorSpace toSpace)` | ✅ |
| 2 | `QImage TransFromRgb(const QImage& image, const std::string& colorSpace)` | `void TransFromRgb(const QImage& in, QImage& out, const std::string& colorSpace)` | ✅ |
| 3 | `QImage TransToRgb(const QImage& image, ColorSpace fromSpace)` | `void TransToRgb(const QImage& in, QImage& out, ColorSpace fromSpace)` | ✅ |
| 4 | `QImage TransToRgb(const QImage& image, const std::string& colorSpace)` | `void TransToRgb(const QImage& in, QImage& out, const std::string& colorSpace)` | ✅ |
| 5 | `QImage ConvertColorSpace(const QImage& image, ColorSpace from, ColorSpace to)` | `void ConvertColorSpace(const QImage& in, QImage& out, ColorSpace from, ColorSpace to)` | ✅ |

### 灰度转换

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 6 | `QImage Rgb1ToGray(const QImage& image, const std::string& method)` | `void Rgb1ToGray(const QImage& in, QImage& out, const std::string& method)` | ✅ |
| 7 | `QImage Rgb3ToGray(const QImage& r, const QImage& g, const QImage& b, const std::string& method)` | `void Rgb3ToGray(const QImage& r, const QImage& g, const QImage& b, QImage& out, const std::string& method)` | ✅ |
| 8 | `QImage GrayToRgb(const QImage& gray)` | `void GrayToRgb(const QImage& in, QImage& out)` | ✅ |

### 通道操作

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 9 | `void Decompose3(const QImage& image, QImage& ch1, QImage& ch2, QImage& ch3)` | - | ✅ 已符合 |
| 10 | `void Decompose4(const QImage& image, QImage& ch1, QImage& ch2, QImage& ch3, QImage& ch4)` | - | ✅ 已符合 |
| 11 | `QImage Compose3(const QImage& ch1, const QImage& ch2, const QImage& ch3, ChannelType type)` | `void Compose3(const QImage& ch1, const QImage& ch2, const QImage& ch3, QImage& out, ChannelType type)` | ✅ |
| 12 | `QImage Compose4(const QImage& ch1, const QImage& ch2, const QImage& ch3, const QImage& ch4, ChannelType type)` | `void Compose4(const QImage& ch1, const QImage& ch2, const QImage& ch3, const QImage& ch4, QImage& out, ChannelType type)` | ✅ |
| 13 | `QImage AccessChannel(const QImage& image, int32_t channelIndex)` | `void AccessChannel(const QImage& in, QImage& out, int32_t channelIndex)` | ✅ |
| 14 | `std::vector<QImage> SplitChannels(const QImage& image)` | `void SplitChannels(const QImage& in, std::vector<QImage>& out)` | ✅ |
| 15 | `QImage MergeChannels(const std::vector<QImage>& channels, ChannelType type)` | `void MergeChannels(const std::vector<QImage>& channels, QImage& out, ChannelType type)` | ✅ |

### 通道交换

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 16 | `QImage RgbToBgr(const QImage& image)` | `void RgbToBgr(const QImage& in, QImage& out)` | ✅ |
| 17 | `QImage BgrToRgb(const QImage& image)` | `void BgrToRgb(const QImage& in, QImage& out)` | ✅ |
| 18 | `QImage SwapChannels(const QImage& image, int32_t ch1, int32_t ch2)` | `void SwapChannels(const QImage& in, QImage& out, int32_t ch1, int32_t ch2)` | ✅ |
| 19 | `QImage ReorderChannels(const QImage& image, const std::vector<int32_t>& order)` | `void ReorderChannels(const QImage& in, QImage& out, const std::vector<int32_t>& order)` | ✅ |

### 颜色调整

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 20 | `QImage AdjustBrightness(const QImage& image, double brightness)` | `void AdjustBrightness(const QImage& in, QImage& out, double brightness)` | ✅ |
| 21 | `QImage AdjustContrast(const QImage& image, double contrast)` | `void AdjustContrast(const QImage& in, QImage& out, double contrast)` | ✅ |
| 22 | `QImage AdjustSaturation(const QImage& image, double saturation)` | `void AdjustSaturation(const QImage& in, QImage& out, double saturation)` | ✅ |
| 23 | `QImage AdjustHue(const QImage& image, double hueShift)` | `void AdjustHue(const QImage& in, QImage& out, double hueShift)` | ✅ |
| 24 | `QImage AdjustGamma(const QImage& image, double gamma)` | `void AdjustGamma(const QImage& in, QImage& out, double gamma)` | ✅ |
| 25 | `QImage InvertColors(const QImage& image)` | `void InvertColors(const QImage& in, QImage& out)` | ✅ |
| 26 | `QImage ScaleImage(const QImage& image, double mult, double add)` | `void ScaleImage(const QImage& in, QImage& out, double mult, double add)` | ✅ |
| 27 | `QImage ScaleImageMax(const QImage& image)` | `void ScaleImageMax(const QImage& in, QImage& out)` | ✅ |
| 28 | `QImage EquHistoImage(const QImage& image)` | `void EquHistoImage(const QImage& in, QImage& out)` | ✅ |

### 白平衡

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 29 | `QImage AutoWhiteBalance(const QImage& image, const std::string& method)` | `void AutoWhiteBalance(const QImage& in, QImage& out, const std::string& method)` | ✅ |
| 30 | `QImage ApplyWhiteBalance(const QImage& image, double whiteR, double whiteG, double whiteB)` | `void ApplyWhiteBalance(const QImage& in, QImage& out, double whiteR, double whiteG, double whiteB)` | ✅ |

### CFA/Bayer

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 31 | `QImage CfaToRgb(const QImage& cfaImage, const std::string& cfaType, const std::string& interpolation)` | `void CfaToRgb(const QImage& in, QImage& out, const std::string& cfaType, const std::string& interpolation)` | ✅ |

### 颜色矩阵

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 32 | `QImage LinearTransColor(const QImage& image, const std::vector<double>& transMat, int32_t numOutputChannels)` | `void LinearTransColor(const QImage& in, QImage& out, const std::vector<double>& transMat, int32_t numOutputChannels)` | ✅ |
| 33 | `QImage ApplyColorMatrix(const QImage& image, const std::vector<double>& matrix)` | `void ApplyColorMatrix(const QImage& in, QImage& out, const std::vector<double>& matrix)` | ✅ |
| 34 | `QImage PrincipalComp(const QImage& image, int32_t numComponents)` | `void PrincipalComp(const QImage& in, QImage& out, int32_t numComponents)` | ✅ |

### 已符合的查询函数

| # | 当前 API | 说明 | 状态 |
|---|----------|------|------|
| - | `int32_t CountChannels(const QImage& image)` | 查询函数 | ✅ |
| - | `std::vector<int64_t> GrayHistoAbs(const QImage& image)` | 查询函数 | ✅ |
| - | `double EntropyGray(const QImage& image)` | 查询函数 | ✅ |
| - | `double GrayHistoPercentile(const QImage& image, double percentile)` | 查询函数 | ✅ |
| - | `void GrayHisto(const QImage& image, std::vector<int64_t>& absHisto, std::vector<double>& relHisto)` | 多输出参数 | ✅ |
| - | `void MinMaxGray(const QImage& image, double& minGray, double& maxGray, double& range)` | 多输出参数 | ✅ |
| - | `void Intensity(const QImage& image, double& mean, double& deviation)` | 多输出参数 | ✅ |
| - | `void GenPrincipalCompTrans(...)` | 多输出参数 | ✅ |
| - | `ColorTransLut CreateColorTransLut(...)` | 构造函数 | ✅ |
| - | `std::string GetColorSpaceName(ColorSpace)` | 查询函数 | ✅ |
| - | `ColorSpace ParseColorSpace(const std::string&)` | 类型转换 | ✅ |
| - | `int32_t GetChannelCount(ColorSpace)` | 查询函数 | ✅ |
| - | `bool HasAlphaChannel(ColorSpace)` | 查询函数 | ✅ |

---

## 3. IO 模块 - ✅ 已完成 (7 个函数)

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 1 | `QImage ReadImage(const std::string& filename)` | `void ReadImage(const std::string& filename, QImage& image)` | ✅ |
| 2 | `QImage ReadImage(const std::string& filename, ImageFormat format)` | `void ReadImage(const std::string& filename, QImage& image, ImageFormat format)` | ✅ |
| 3 | `QImage ReadImageRaw(const std::string& filename, const RawReadParams& params)` | `void ReadImageRaw(const std::string& filename, QImage& image, const RawReadParams& params)` | ✅ |
| 4 | `QImage ReadImageAs(const std::string& filename, PixelType targetType)` | `void ReadImageAs(const std::string& filename, QImage& image, PixelType targetType)` | ✅ |
| 5 | `QImage ReadImageGray(const std::string& filename)` | `void ReadImageGray(const std::string& filename, QImage& image)` | ✅ |
| 6 | `bool WriteImage(const QImage& image, const std::string& filename)` | - | ✅ 已符合 |
| 7 | `std::vector<QImage> ReadSequence(...)` | `void ReadSequence(const std::string& pattern, std::vector<QImage>& images, ...)` | ✅ |
| 8 | `std::vector<QImage> ReadDirectory(...)` | `void ReadDirectory(const std::string& directory, std::vector<QImage>& images, ...)` | ✅ |

---

## 4. Matching 模块 - ✅ 已完成 (6 个函数)

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 1 | `ShapeModel CreateShapeModel(const QImage& templateImage, ...)` | `void CreateShapeModel(const QImage& templateImage, ShapeModel& model, ...)` | ✅ |
| 2 | `ShapeModel CreateShapeModel(const QImage& templateImage, const Rect2i& roi, ...)` | `void CreateShapeModel(const QImage& templateImage, const Rect2i& roi, ShapeModel& model, ...)` | ✅ |
| 3 | `ShapeModel CreateShapeModel(const QImage& templateImage, const QRegion& region, ...)` | `void CreateShapeModel(const QImage& templateImage, const QRegion& region, ShapeModel& model, ...)` | ✅ |
| 4 | `ShapeModel CreateScaledShapeModel(...)` | `void CreateScaledShapeModel(const QImage& templateImage, ShapeModel& model, ...)` | ✅ |
| 5 | `ShapeModel ReadShapeModel(const std::string& filename)` | `void ReadShapeModel(const std::string& filename, ShapeModel& model)` | ✅ |
| 6 | `QContourArray GetShapeModelXLD(const ShapeModel& model, int32_t level)` | `void GetShapeModelXLD(const ShapeModel& model, int32_t level, QContourArray& contours)` | ✅ |
| 7 | `void FindShapeModel(...)` | - | ✅ 已符合 |
| 8 | `void FindScaledShapeModel(...)` | - | ✅ 已符合 |
| 9 | `void WriteShapeModel(const ShapeModel& model, const std::string& filename)` | - | ✅ 已符合 |
| 10 | `void GetShapeModelOrigin(const ShapeModel& model, double& row, double& col)` | - | ✅ 已符合 |
| 11 | `void SetShapeModelOrigin(ShapeModel& model, double row, double col)` | - | ✅ 已符合 |

---

## 5. Blob 模块 - ✅ 已完成 (15 个函数)

### 连通域分析

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 1 | `std::vector<QRegion> Connection(const QRegion& region)` | `void Connection(const QRegion& in, std::vector<QRegion>& out)` | ✅ |
| 2 | `std::vector<QRegion> Connection(const QImage& binaryImage, Connectivity connectivity)` | `void Connection(const QImage& in, std::vector<QRegion>& out, Connectivity connectivity)` | ✅ |

### 形状选择

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 3 | `std::vector<QRegion> SelectShape(...)` | `void SelectShape(const std::vector<QRegion>& in, std::vector<QRegion>& out, ...)` | ✅ |
| 4 | `std::vector<QRegion> SelectShape(...string...)` | `void SelectShape(const std::vector<QRegion>& in, std::vector<QRegion>& out, ...)` | ✅ |
| 5 | `std::vector<QRegion> SelectShapeArea(...)` | `void SelectShapeArea(const std::vector<QRegion>& in, std::vector<QRegion>& out, ...)` | ✅ |
| 6 | `std::vector<QRegion> SelectShapeCircularity(...)` | `void SelectShapeCircularity(const std::vector<QRegion>& in, std::vector<QRegion>& out, ...)` | ✅ |
| 7 | `std::vector<QRegion> SelectShapeRectangularity(...)` | `void SelectShapeRectangularity(const std::vector<QRegion>& in, std::vector<QRegion>& out, ...)` | ✅ |
| 8 | `std::vector<QRegion> SelectShapeStd(...)` | `void SelectShapeStd(const std::vector<QRegion>& in, std::vector<QRegion>& out, ...)` | ✅ |
| 9 | `std::vector<QRegion> SelectShapeMulti(...)` | `void SelectShapeMulti(const std::vector<QRegion>& in, std::vector<QRegion>& out, ...)` | ✅ |
| 10 | `std::vector<QRegion> SelectShapeConvexity(...)` | `void SelectShapeConvexity(const std::vector<QRegion>& in, std::vector<QRegion>& out, ...)` | ✅ |
| 11 | `std::vector<QRegion> SelectShapeElongation(...)` | `void SelectShapeElongation(const std::vector<QRegion>& in, std::vector<QRegion>& out, ...)` | ✅ |
| 12 | `std::vector<QRegion> SelectShapeProto(...)` | `void SelectShapeProto(const std::vector<QRegion>& in, std::vector<QRegion>& out, ...)` | ✅ |

### 排序

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 13 | `std::vector<QRegion> SortRegion(...)` | `void SortRegion(const std::vector<QRegion>& in, std::vector<QRegion>& out, ...)` | ✅ |
| 14 | `std::vector<QRegion> SortRegion(...string...)` | `void SortRegion(const std::vector<QRegion>& in, std::vector<QRegion>& out, ...)` | ✅ |

### 孔洞分析

| # | 当前 API | 目标 API | 状态 |
|---|----------|----------|------|
| 15 | `std::vector<QRegion> GetHoles(const QRegion& region)` | `void GetHoles(const QRegion& in, std::vector<QRegion>& out)` | ✅ |
| - | `QRegion FillUp(const QRegion& region)` | `void FillUp(const QRegion& in, QRegion& out)` | ✅ |

### 已符合的函数

| # | 当前 API | 说明 | 状态 |
|---|----------|------|------|
| - | `void AreaCenter(const QRegion& region, int64_t& area, double& row, double& column)` | 多输出参数 | ✅ |
| - | `void SmallestRectangle1(const QRegion& region, int32_t& row1, int32_t& column1, int32_t& row2, int32_t& column2)` | 多输出参数 | ✅ |
| - | `void SmallestRectangle2(const QRegion& region, double& row, double& column, double& phi, double& length1, double& length2)` | 多输出参数 | ✅ |
| - | `void SmallestCircle(const QRegion& region, double& row, double& column, double& radius)` | 多输出参数 | ✅ |
| - | `void EllipticAxis(const QRegion& region, double& ra, double& rb, double& phi)` | 多输出参数 | ✅ |
| - | `double Circularity(const QRegion& region)` | 查询函数 | ✅ |
| - | `double Compactness(const QRegion& region)` | 查询函数 | ✅ |
| - | `double Convexity(const QRegion& region)` | 查询函数 | ✅ |
| - | `double Rectangularity(const QRegion& region)` | 查询函数 | ✅ |
| - | `double OrientationRegion(const QRegion& region)` | 查询函数 | ✅ |
| - | `int32_t CountObj(const std::vector<QRegion>& regions)` | 查询函数 | ✅ |
| - | `QRegion SelectObj(const std::vector<QRegion>& regions, int32_t index)` | 对象选择 | ✅ |

---

## 6. Measure 模块 - 已符合

所有 Measure 模块函数已符合 Halcon 风格，返回结果集合是合理设计。

| 当前 API | 说明 | 状态 |
|----------|------|------|
| `std::vector<EdgeResult> MeasurePos(const QImage& image, const MeasureRectangle2& handle, const MeasureParams& params)` | 返回值合理（结果集合） | ✅ |
| `std::vector<PairResult> MeasurePairs(const QImage& image, const MeasureRectangle2& handle, const MeasureParams& params)` | 返回值合理 | ✅ |
| `MetrologyModel::Apply(const QImage& image)` | in-place 更新模型状态 | ✅ |
| `MetrologyModel::GetCircleResult(int32_t idx)` | 查询方法 | ✅ |

---

## 7. Display/Draw 模块 - 已符合

绘图函数使用 in-place 修改风格（累积绘制），这是合理的设计。

| 当前 API | 说明 | 状态 |
|----------|------|------|
| `void DispLine(QImage& image, ...)` | in-place 绘制 | ✅ |
| `void DispCircle(QImage& image, ...)` | in-place 绘制 | ✅ |
| `void DispContour(QImage& image, ...)` | in-place 绘制 | ✅ |
| `void Draw::Rectangle(QImage& image, ...)` | in-place 绘制 | ✅ |
| `void Draw::Contour(QImage& image, ...)` | in-place 绘制 | ✅ |
| `void Draw::ToRGB(const QImage& gray, QImage& output)` | Halcon 风格 | ✅ |
| `void Draw::PrepareForDrawing(const QImage& image, QImage& output)` | Halcon 风格 | ✅ |

---

## 8. Core 模块 - 保持现有风格

类成员方法保持现有风格，不需要修改。

| 当前 API | 说明 | 状态 |
|----------|------|------|
| `QImage::FromFile(filename)` | 静态构造 | ✅ 保留 |
| `QImage::ToGray()` | 类型转换方法 | ✅ 保留 |
| `QImage::Clone()` | 深拷贝 | ✅ 保留 |
| `QRegion::Union(other)` | 返回新区域 | ✅ 保留 |
| `QContour::Length()` | 查询 | ✅ 保留 |

---

## 9. GUI 模块 - 保持现有风格

GUI 模块保持现有设计，窗口类使用对象方法风格。

---

## 10. Internal 模块 - 不导出

内部模块不遵循公开 API 规范，保持现有命名。

---

## 重构统计

| 模块 | 需重构 | 已符合 | 总计 | 状态 |
|------|--------|--------|------|--------|
| Filter | 0 | 33 | 33 | ✅ **完成** |
| Color | 0 | 34+ | 34+ | ✅ **完成** |
| IO | 0 | 8 | 8 | ✅ **完成** |
| Matching | 0 | 11 | 11 | ✅ **完成** |
| Blob | 0 | 25+ | 25+ | ✅ **完成** |
| Measure | 0 | 全部 | - | ✅ **完成** |
| Display/Draw | 0 | 全部 | - | ✅ **完成** |
| Core | 0 | 全部 | - | ✅ **完成** |
| **总计** | **0** | **100+** | - | ✅ **全部完成** |

---

## 重构示例

### Filter 模块重构示例

```cpp
// ============ 重构前 ============
QImage GaussFilter(const QImage& image, double sigma);

// 调用
QImage result = GaussFilter(input, 1.5);

// ============ 重构后 ============
void GaussFilter(const QImage& input, QImage& output, double sigma);

// 调用
QImage result;
GaussFilter(input, result, 1.5);

// 或者复用缓冲区
static QImage buffer;  // 预分配
GaussFilter(input, buffer, 1.5);
```

### Blob 模块重构示例

```cpp
// ============ 重构前 ============
std::vector<QRegion> Connection(const QRegion& region);

// 调用
auto blobs = Connection(region);

// ============ 重构后 ============
void Connection(const QRegion& input, std::vector<QRegion>& output);

// 调用
std::vector<QRegion> blobs;
Connection(region, blobs);
```

### Matching 模块重构示例

```cpp
// ============ 重构前 ============
ShapeModel CreateShapeModel(const QImage& image, ...);

// 调用
ShapeModel model = CreateShapeModel(templ, ...);

// ============ 重构后 ============
void CreateShapeModel(const QImage& templateImage, ShapeModel& model, ...);

// 调用
ShapeModel model;
CreateShapeModel(templ, model, ...);
```

---

## 变更日志

### 2026-01-21 (API 重构全部完成 🎉)

**最终统计**：共重构 **100+** 个公开 API 函数，全部符合 Halcon 风格规范。

#### IO 模块 (7 个函数)
- `ReadImage` (2个重载)、`ReadImageRaw`、`ReadImageAs`、`ReadImageGray` → 改为 `void Func(..., QImage& out, ...)`
- `ReadSequence`、`ReadDirectory` → 改为 `void Func(..., std::vector<QImage>& out, ...)`
- 无外部调用需更新，编译验证通过

#### Matching 模块 (6 个函数)
- `CreateShapeModel` (3个重载)、`CreateScaledShapeModel` → 改为 `void Func(..., ShapeModel& model, ...)`
- `ReadShapeModel` → 改为 `void ReadShapeModel(filename, ShapeModel& model)`
- `GetShapeModelXLD` → 改为 `void GetShapeModelXLD(model, level, QContourArray& contours)`
- 已更新调用：shape_match.cpp、test_create_model.cpp、Draw.cpp
- 编译验证通过

#### Blob 模块 (15 个函数)
- `Connection` (2个重载) → 改为 `void Connection(..., std::vector<QRegion>& out)`
- `SelectShape` 系列 (10个) → 改为 `void SelectShape(..., std::vector<QRegion>& out, ...)`
- `SortRegion` (2个重载) → 改为 `void SortRegion(..., std::vector<QRegion>& out, ...)`
- `GetHoles` → 改为 `void GetHoles(region, std::vector<QRegion>& holes)`
- `FillUp` → 改为 `void FillUp(region, QRegion& filled)`
- 无外部调用需更新，编译验证通过

**允许返回值的例外情况**（符合规范）：
- 查询函数：`Circularity()`, `Compactness()`, `SelectObj()` 等
- 类型转换：`GrayToRgb()`, `PrepareForDrawing()` 等
- 构造函数：`QImage::FromFile()`, `CreateColorTransLut()` 等
- Internal 模块：非公开 API，不强制遵循规范

### 2026-01-21 (Color 模块)
- **Color 模块重构完成**：所有 34 个 Color 函数已转换为 Halcon 风格 API
  - 修改 ColorConvert.h 头文件：所有函数签名改为 `void Func(const QImage& in, QImage& out, params)`
  - 修改 ColorConvert.cpp 源文件：所有实现改为输出参数风格
  - 编译验证通过

### 2026-01-21 (Filter 模块)
- **Filter 模块重构完成**：所有 33 个 Filter 函数已转换为 Halcon 风格 API
  - 修改 Filter.h 头文件：所有函数签名改为 `void Func(const QImage& in, QImage& out, params)`
  - 修改 Filter.cpp 源文件：所有实现改为输出参数风格
  - 编译验证通过

### 2026-01-21
- **重大更新**：完整列出所有需要重构的公开 API（85 个函数）
- 添加详细的函数签名对照表
- 添加重构统计表
- 确认 Measure、Display、Core 模块已符合规范
- Filter (31个)、Color (28个)、Blob (13个)、IO (7个)、Matching (6个) 需要重构

### 2025-01-12
- 创建 API 重命名映射文档
- 分析所有公共 API 并确定重命名策略
