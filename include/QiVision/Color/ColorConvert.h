#pragma once

/**
 * @file ColorConvert.h
 * @brief Color space conversion and channel operations (Halcon-style API)
 *
 * Halcon reference operators:
 * - trans_from_rgb, trans_to_rgb
 * - decompose3, decompose4, compose3, compose4
 * - rgb1_to_gray, rgb3_to_gray
 * - access_channel, count_channels
 *
 * Supported color spaces:
 * - RGB/BGR (standard)
 * - HSV/HSL (hue-saturation)
 * - Lab/Luv (perceptually uniform)
 * - YCrCb/YUV (video)
 * - XYZ (CIE standard)
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/Types.h>

#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

namespace Qi::Vision::Color {

// =============================================================================
// Color Space Enumeration
// =============================================================================

/**
 * @brief Supported color spaces
 */
enum class ColorSpace {
    Gray,       ///< Grayscale (1 channel)
    RGB,        ///< Red, Green, Blue
    BGR,        ///< Blue, Green, Red (OpenCV convention)
    RGBA,       ///< RGB + Alpha
    BGRA,       ///< BGR + Alpha
    HSV,        ///< Hue [0-360], Saturation [0-1], Value [0-1]
    HSL,        ///< Hue [0-360], Saturation [0-1], Lightness [0-1]
    Lab,        ///< CIE L*a*b* (perceptually uniform)
    Luv,        ///< CIE L*u*v*
    XYZ,        ///< CIE XYZ (standard observer)
    YCrCb,      ///< Luma + Chroma (JPEG/MPEG)
    YUV         ///< Luma + Chroma (analog video)
};

/**
 * @brief White point reference for Lab/XYZ conversions
 */
enum class WhitePoint {
    D50,        ///< Daylight 5003K (ICC profile default)
    D65,        ///< Daylight 6504K (sRGB default)
    A,          ///< Incandescent tungsten
    C,          ///< Average daylight
    E           ///< Equal energy
};

// =============================================================================
// Color Space Conversion
// =============================================================================

/**
 * @brief Convert image from one color space to another
 *
 * @param image Input image
 * @param fromSpace Source color space
 * @param toSpace Target color space
 * @return Converted image
 *
 * @code
 * QImage hsv = ConvertColorSpace(rgb, ColorSpace::RGB, ColorSpace::HSV);
 * QImage lab = ConvertColorSpace(rgb, ColorSpace::RGB, ColorSpace::Lab);
 * @endcode
 */
QImage ConvertColorSpace(const QImage& image,
                          ColorSpace fromSpace,
                          ColorSpace toSpace);

/**
 * @brief Convert RGB image to specified color space
 *
 * Equivalent to Halcon's trans_from_rgb operator.
 *
 * @param image Input RGB image
 * @param toSpace Target color space
 * @return Converted image
 */
QImage TransFromRgb(const QImage& image, ColorSpace toSpace);

/**
 * @brief Convert RGB image to specified color space (string version)
 *
 * @param image Input RGB image
 * @param colorSpace Color space name: "gray", "hsv", "hsl", "lab", "luv",
 *                   "xyz", "ycrcb", "yuv", "bgr"
 * @return Converted image
 */
QImage TransFromRgb(const QImage& image, const std::string& colorSpace);

/**
 * @brief Convert image to RGB from specified color space
 *
 * Equivalent to Halcon's trans_to_rgb operator.
 *
 * @param image Input image in source color space
 * @param fromSpace Source color space
 * @return RGB image
 */
QImage TransToRgb(const QImage& image, ColorSpace fromSpace);

/**
 * @brief Convert image to RGB (string version)
 */
QImage TransToRgb(const QImage& image, const std::string& colorSpace);

// =============================================================================
// Grayscale Conversion
// =============================================================================

/**
 * @brief Convert RGB image to grayscale
 *
 * Equivalent to Halcon's rgb1_to_gray operator.
 *
 * @param image Input RGB/RGBA image
 * @param method Conversion method:
 *        - "luminosity" (default): 0.299*R + 0.587*G + 0.114*B
 *        - "average": (R + G + B) / 3
 *        - "lightness": (max(R,G,B) + min(R,G,B)) / 2
 *        - "bt601": ITU-R BT.601 (same as luminosity)
 *        - "bt709": ITU-R BT.709 (0.2126*R + 0.7152*G + 0.0722*B)
 *        - "max": max(R, G, B)
 *        - "min": min(R, G, B)
 * @return Grayscale image
 */
QImage Rgb1ToGray(const QImage& image, const std::string& method = "luminosity");

/**
 * @brief Convert 3-channel RGB to grayscale
 *
 * Equivalent to Halcon's rgb3_to_gray operator.
 *
 * @param red Red channel image
 * @param green Green channel image
 * @param blue Blue channel image
 * @param method Conversion method
 * @return Grayscale image
 */
QImage Rgb3ToGray(const QImage& red, const QImage& green, const QImage& blue,
                   const std::string& method = "luminosity");

/**
 * @brief Convert grayscale to RGB (replicate to 3 channels)
 *
 * @param gray Grayscale image
 * @return RGB image with R=G=B=gray
 */
QImage GrayToRgb(const QImage& gray);

// =============================================================================
// Channel Decomposition
// =============================================================================

/**
 * @brief Decompose multi-channel image into separate channels
 *
 * Equivalent to Halcon's decompose3 operator.
 *
 * @param image Input multi-channel image
 * @param ch1 [out] First channel
 * @param ch2 [out] Second channel
 * @param ch3 [out] Third channel
 *
 * @code
 * QImage r, g, b;
 * Decompose3(rgbImage, r, g, b);
 *
 * QImage h, s, v;
 * Decompose3(hsvImage, h, s, v);
 * @endcode
 */
void Decompose3(const QImage& image,
                 QImage& ch1, QImage& ch2, QImage& ch3);

/**
 * @brief Decompose 4-channel image
 *
 * Equivalent to Halcon's decompose4 operator.
 */
void Decompose4(const QImage& image,
                 QImage& ch1, QImage& ch2, QImage& ch3, QImage& ch4);

/**
 * @brief Compose 3 channels into multi-channel image
 *
 * Equivalent to Halcon's compose3 operator.
 *
 * @param ch1 First channel
 * @param ch2 Second channel
 * @param ch3 Third channel
 * @param channelType Output channel type (RGB, HSV, Lab, etc.)
 * @return Composed multi-channel image
 */
QImage Compose3(const QImage& ch1, const QImage& ch2, const QImage& ch3,
                 ChannelType channelType = ChannelType::RGB);

/**
 * @brief Compose 4 channels into multi-channel image
 *
 * Equivalent to Halcon's compose4 operator.
 */
QImage Compose4(const QImage& ch1, const QImage& ch2,
                 const QImage& ch3, const QImage& ch4,
                 ChannelType channelType = ChannelType::RGBA);

// =============================================================================
// Channel Access
// =============================================================================

/**
 * @brief Access single channel from multi-channel image
 *
 * Equivalent to Halcon's access_channel operator.
 *
 * @param image Input image
 * @param channelIndex Channel index (0-based)
 * @return Single channel image (copy)
 *
 * @code
 * QImage blue = AccessChannel(rgbImage, 2);  // RGB: 0=R, 1=G, 2=B
 * QImage hue = AccessChannel(hsvImage, 0);   // HSV: 0=H, 1=S, 2=V
 * @endcode
 */
QImage AccessChannel(const QImage& image, int32_t channelIndex);

/**
 * @brief Get number of channels in image
 *
 * Equivalent to Halcon's count_channels operator.
 *
 * @param image Input image
 * @return Number of channels (1, 3, or 4)
 */
int32_t CountChannels(const QImage& image);

/**
 * @brief Extract all channels as vector of images
 *
 * @param image Input image
 * @return Vector of single-channel images
 */
std::vector<QImage> SplitChannels(const QImage& image);

/**
 * @brief Merge vector of channels into multi-channel image
 *
 * @param channels Vector of single-channel images
 * @param channelType Output channel type
 * @return Merged multi-channel image
 */
QImage MergeChannels(const std::vector<QImage>& channels,
                      ChannelType channelType = ChannelType::RGB);

// =============================================================================
// Channel Swapping and Reordering
// =============================================================================

/**
 * @brief Convert between RGB and BGR
 *
 * @param image Input image
 * @return Image with swapped R and B channels
 */
QImage RgbToBgr(const QImage& image);
QImage BgrToRgb(const QImage& image);

/**
 * @brief Swap two channels
 *
 * @param image Input image
 * @param ch1 First channel index
 * @param ch2 Second channel index
 * @return Image with swapped channels
 */
QImage SwapChannels(const QImage& image, int32_t ch1, int32_t ch2);

/**
 * @brief Reorder channels
 *
 * @param image Input image
 * @param order Channel order (e.g., {2, 1, 0} for RGB->BGR)
 * @return Image with reordered channels
 */
QImage ReorderChannels(const QImage& image, const std::vector<int32_t>& order);

// =============================================================================
// Color Adjustment
// =============================================================================

/**
 * @brief Adjust image brightness
 *
 * @param image Input image
 * @param brightness Brightness adjustment [-255, 255]
 * @return Adjusted image
 */
QImage AdjustBrightness(const QImage& image, double brightness);

/**
 * @brief Adjust image contrast
 *
 * @param image Input image
 * @param contrast Contrast factor (1.0 = no change, >1 = more contrast)
 * @return Adjusted image
 */
QImage AdjustContrast(const QImage& image, double contrast);

/**
 * @brief Adjust saturation (for color images)
 *
 * @param image Input image (RGB)
 * @param saturation Saturation factor (1.0 = no change, 0 = grayscale, >1 = more saturated)
 * @return Adjusted image
 */
QImage AdjustSaturation(const QImage& image, double saturation);

/**
 * @brief Adjust hue (for color images)
 *
 * @param image Input image (RGB)
 * @param hueShift Hue shift in degrees [-180, 180]
 * @return Adjusted image
 */
QImage AdjustHue(const QImage& image, double hueShift);

/**
 * @brief Apply gamma correction
 *
 * @param image Input image
 * @param gamma Gamma value (1.0 = no change, <1 = brighter, >1 = darker)
 * @return Gamma-corrected image
 */
QImage AdjustGamma(const QImage& image, double gamma);

/**
 * @brief Invert image colors
 *
 * @param image Input image
 * @return Inverted image (255 - pixel for UInt8)
 */
QImage InvertColors(const QImage& image);

/**
 * @brief Scale image intensity values linearly
 *
 * Equivalent to Halcon's scale_image operator.
 * Output = Input * mult + add
 *
 * @param image Input image
 * @param mult Multiplication factor
 * @param add Addition value
 * @return Scaled image
 *
 * @code
 * QImage bright = ScaleImage(image, 1.2, 20);  // Increase brightness
 * QImage contrast = ScaleImage(image, 1.5, -64);  // Increase contrast
 * @endcode
 */
QImage ScaleImage(const QImage& image, double mult, double add);

/**
 * @brief Scale image to full dynamic range
 *
 * Equivalent to Halcon's scale_image_max operator.
 * Linearly maps [min, max] to [0, 255].
 *
 * @param image Input image
 * @return Scaled image with full dynamic range
 *
 * @code
 * QImage enhanced = ScaleImageMax(image);
 * @endcode
 */
QImage ScaleImageMax(const QImage& image);

/**
 * @brief Apply histogram equalization
 *
 * Equivalent to Halcon's equ_histo_image operator.
 * Enhances contrast by redistributing intensity values.
 *
 * @param image Input grayscale image
 * @return Equalized image
 *
 * @code
 * QImage enhanced = EquHistoImage(image);
 * @endcode
 */
QImage EquHistoImage(const QImage& image);

// =============================================================================
// Histogram Analysis
// =============================================================================

/**
 * @brief Compute gray value histogram
 *
 * Equivalent to Halcon's gray_histo operator.
 *
 * @param image Input grayscale image
 * @param[out] absoluteHisto Absolute histogram (pixel counts, 256 bins)
 * @param[out] relativeHisto Relative histogram (normalized to sum=1)
 *
 * @code
 * std::vector<int64_t> absHisto;
 * std::vector<double> relHisto;
 * GrayHisto(image, absHisto, relHisto);
 * @endcode
 */
void GrayHisto(const QImage& image,
               std::vector<int64_t>& absoluteHisto,
               std::vector<double>& relativeHisto);

/**
 * @brief Compute absolute gray value histogram
 *
 * Equivalent to Halcon's gray_histo_abs operator.
 *
 * @param image Input grayscale image
 * @return Absolute histogram (256 bins)
 */
std::vector<int64_t> GrayHistoAbs(const QImage& image);

/**
 * @brief Get minimum and maximum gray values
 *
 * Equivalent to Halcon's min_max_gray operator.
 *
 * @param image Input image
 * @param[out] minGray Minimum gray value
 * @param[out] maxGray Maximum gray value
 * @param[out] range Gray value range (max - min)
 */
void MinMaxGray(const QImage& image, double& minGray, double& maxGray, double& range);

/**
 * @brief Compute gray value intensity statistics
 *
 * Equivalent to Halcon's intensity operator.
 *
 * @param image Input image
 * @param[out] mean Mean gray value
 * @param[out] deviation Standard deviation
 */
void Intensity(const QImage& image, double& mean, double& deviation);

/**
 * @brief Compute gray value entropy
 *
 * Equivalent to Halcon's entropy_gray operator.
 *
 * @param image Input grayscale image
 * @return Entropy in bits
 */
double EntropyGray(const QImage& image);

/**
 * @brief Compute histogram percentile value
 *
 * @param image Input image
 * @param percentile Percentile value (0-100)
 * @return Gray value at specified percentile
 */
double GrayHistoPercentile(const QImage& image, double percentile);

// =============================================================================
// White Balance
// =============================================================================

/**
 * @brief Apply automatic white balance
 *
 * @param image Input RGB image
 * @param method Method:
 *        - "gray_world": Assume average color is gray
 *        - "white_patch": Assume brightest pixel is white
 *        - "histogram_stretch": Per-channel histogram stretching
 * @return White-balanced image
 */
QImage AutoWhiteBalance(const QImage& image, const std::string& method = "gray_world");

/**
 * @brief Apply white balance with specified white point
 *
 * @param image Input RGB image
 * @param whiteR Reference R value for white
 * @param whiteG Reference G value for white
 * @param whiteB Reference B value for white
 * @return White-balanced image
 */
QImage ApplyWhiteBalance(const QImage& image, double whiteR, double whiteG, double whiteB);

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Get color space name string
 */
std::string GetColorSpaceName(ColorSpace space);

/**
 * @brief Parse color space from string
 */
ColorSpace ParseColorSpace(const std::string& name);

/**
 * @brief Get number of channels for color space
 */
int32_t GetChannelCount(ColorSpace space);

/**
 * @brief Check if color space has alpha channel
 */
bool HasAlphaChannel(ColorSpace space);

// =============================================================================
// Color Transform LUT (Look-Up Table)
// =============================================================================

/**
 * @brief Handle for color transformation look-up table
 *
 * LUT accelerates repeated color space conversions (8-bit only).
 * Memory usage: ~48MB per LUT.
 */
class ColorTransLut {
public:
    ColorTransLut();
    ~ColorTransLut();
    ColorTransLut(ColorTransLut&& other) noexcept;
    ColorTransLut& operator=(ColorTransLut&& other) noexcept;

    // Non-copyable (large memory)
    ColorTransLut(const ColorTransLut&) = delete;
    ColorTransLut& operator=(const ColorTransLut&) = delete;

    /// Check if LUT is valid
    bool IsValid() const;

    /// Get source color space
    ColorSpace GetSourceSpace() const { return fromSpace_; }

    /// Get target color space
    ColorSpace GetTargetSpace() const { return toSpace_; }

private:
    friend ColorTransLut CreateColorTransLut(const std::string&, const std::string&, int32_t);
    friend void ApplyColorTransLut(const QImage&, const QImage&, const QImage&,
                                   QImage&, QImage&, QImage&, const ColorTransLut&);
    friend QImage ApplyColorTransLut(const QImage&, const ColorTransLut&);
    friend void ClearColorTransLut(ColorTransLut&);

    std::vector<uint8_t> lut_;  // 256*256*256*3 = 48MB
    ColorSpace fromSpace_ = ColorSpace::RGB;
    ColorSpace toSpace_ = ColorSpace::RGB;
};

/**
 * @brief Create look-up table for color space transformation
 *
 * Equivalent to Halcon's create_color_trans_lut operator.
 *
 * @param colorSpace Target color space: "hsv", "hls", "hsi", "lab", "luv",
 *                   "xyz", "ycrcb", "yuv", "ihs"
 * @param transDirection Transformation direction: "from_rgb", "to_rgb"
 * @param numBits Bit depth (currently only 8 supported)
 * @return LUT handle
 *
 * @note LUT requires ~48MB memory. Use ClearColorTransLut to release.
 *
 * @code
 * // Create LUT for RGB -> HSV
 * ColorTransLut lut = CreateColorTransLut("hsv", "from_rgb", 8);
 *
 * // Apply to many images
 * for (auto& img : images) {
 *     QImage h, s, v;
 *     Decompose3(img, r, g, b);
 *     ApplyColorTransLut(r, g, b, h, s, v, lut);
 * }
 *
 * // Release when done
 * ClearColorTransLut(lut);
 * @endcode
 */
ColorTransLut CreateColorTransLut(const std::string& colorSpace,
                                   const std::string& transDirection = "from_rgb",
                                   int32_t numBits = 8);

/**
 * @brief Apply color transformation using pre-generated LUT
 *
 * Equivalent to Halcon's apply_color_trans_lut operator.
 *
 * @param image1 Input channel 1 (R or first channel of source space)
 * @param image2 Input channel 2 (G or second channel)
 * @param image3 Input channel 3 (B or third channel)
 * @param result1 [out] Output channel 1
 * @param result2 [out] Output channel 2
 * @param result3 [out] Output channel 3
 * @param lut LUT handle from CreateColorTransLut
 */
void ApplyColorTransLut(const QImage& image1, const QImage& image2, const QImage& image3,
                        QImage& result1, QImage& result2, QImage& result3,
                        const ColorTransLut& lut);

/**
 * @brief Apply color transformation using LUT (multi-channel version)
 *
 * @param image Input 3-channel image
 * @param lut LUT handle
 * @return Transformed 3-channel image
 */
QImage ApplyColorTransLut(const QImage& image, const ColorTransLut& lut);

/**
 * @brief Release look-up table memory
 *
 * Equivalent to Halcon's clear_color_trans_lut operator.
 *
 * @param lut LUT handle to clear
 */
void ClearColorTransLut(ColorTransLut& lut);

// =============================================================================
// Bayer Pattern (Color Filter Array) Conversion
// =============================================================================

/**
 * @brief Convert Bayer pattern image to RGB
 *
 * Equivalent to Halcon's cfa_to_rgb operator.
 * Used for single-chip CCD/CMOS camera raw images.
 *
 * @param cfaImage Input single-channel Bayer image (UInt8 or UInt16)
 * @param cfaType Bayer pattern type:
 *        - "bayer_rg": RGGB pattern (R at top-left)
 *        - "bayer_gr": GRBG pattern (G at top-left, R next)
 *        - "bayer_gb": GBRG pattern (G at top-left, B next)
 *        - "bayer_bg": BGGR pattern (B at top-left)
 * @param interpolation Interpolation method:
 *        - "bilinear": Standard bilinear interpolation
 *        - "bilinear_dir": Direction-aware bilinear (reduces zipper artifacts)
 * @return RGB image (3 channels, same bit depth as input)
 *
 * @code
 * // Read raw Bayer image from camera
 * QImage raw = ReadImage("camera_raw.pgm");
 *
 * // Convert to RGB (camera uses RGGB pattern)
 * QImage rgb = CfaToRgb(raw, "bayer_rg", "bilinear_dir");
 * @endcode
 */
QImage CfaToRgb(const QImage& cfaImage,
                const std::string& cfaType = "bayer_gb",
                const std::string& interpolation = "bilinear");

// =============================================================================
// Linear Color Transformation
// =============================================================================

/**
 * @brief Apply affine transformation to color values
 *
 * Equivalent to Halcon's linear_trans_color operator.
 * Performs: output[i] = sum(transMat[i][j] * input[j]) + transMat[i][n]
 *
 * @param image Input multichannel image
 * @param transMat Transformation matrix (row-major, m x (n+1)):
 *        - m = number of output channels
 *        - n = number of input channels
 *        - Last column = offset values
 * @param numOutputChannels Number of output channels (rows in matrix)
 * @return Transformed image (Float32 type)
 *
 * @code
 * // RGB to grayscale with custom weights
 * std::vector<double> mat = {
 *     0.299, 0.587, 0.114, 0.0  // Y = 0.299*R + 0.587*G + 0.114*B + 0
 * };
 * QImage gray = LinearTransColor(rgbImage, mat, 1);
 *
 * // Color correction matrix (3x4)
 * std::vector<double> colorMat = {
 *     1.2, -0.1, -0.1, 0.0,   // R' = 1.2*R - 0.1*G - 0.1*B
 *     -0.1, 1.2, -0.1, 0.0,   // G' = -0.1*R + 1.2*G - 0.1*B
 *     -0.1, -0.1, 1.2, 0.0    // B' = -0.1*R - 0.1*G + 1.2*B
 * };
 * QImage corrected = LinearTransColor(image, colorMat, 3);
 * @endcode
 */
QImage LinearTransColor(const QImage& image,
                        const std::vector<double>& transMat,
                        int32_t numOutputChannels);

/**
 * @brief Apply color matrix transformation (3x3 matrix, no offset)
 *
 * Simplified version for common 3x3 color matrix.
 *
 * @param image Input 3-channel image
 * @param matrix 3x3 transformation matrix (row-major, 9 elements)
 * @return Transformed 3-channel image
 */
QImage ApplyColorMatrix(const QImage& image, const std::vector<double>& matrix);

// =============================================================================
// Principal Component Analysis
// =============================================================================

/**
 * @brief Compute principal components of multichannel image
 *
 * Equivalent to Halcon's principal_comp operator.
 *
 * @param image Input multichannel image
 * @param numComponents Number of principal components to compute (0 = all)
 * @return Transformed image with principal component channels
 */
QImage PrincipalComp(const QImage& image, int32_t numComponents = 0);

/**
 * @brief Generate principal component transformation matrix
 *
 * Equivalent to Halcon's gen_principal_comp_trans operator.
 *
 * @param image Input multichannel image for training
 * @param transMat [out] Transformation matrix
 * @param mean [out] Mean values per channel
 * @param eigenvalues [out] Eigenvalues (variance explained)
 */
void GenPrincipalCompTrans(const QImage& image,
                           std::vector<double>& transMat,
                           std::vector<double>& mean,
                           std::vector<double>& eigenvalues);

} // namespace Qi::Vision::Color
