/**
 * @file ColorConvert.cpp
 * @brief Color space conversion implementation
 */

#include <QiVision/Color/ColorConvert.h>
#include <QiVision/Core/Exception.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace Qi::Vision::Color {

// =============================================================================
// Constants
// =============================================================================

namespace {

// sRGB to XYZ (D65) conversion matrix
constexpr double RGB_TO_XYZ[3][3] = {
    {0.4124564, 0.3575761, 0.1804375},
    {0.2126729, 0.7151522, 0.0721750},
    {0.0193339, 0.1191920, 0.9503041}
};

// XYZ to sRGB (D65) conversion matrix
constexpr double XYZ_TO_RGB[3][3] = {
    { 3.2404542, -1.5371385, -0.4985314},
    {-0.9692660,  1.8760108,  0.0415560},
    { 0.0556434, -0.2040259,  1.0572252}
};

// D65 white point
constexpr double D65_X = 0.95047;
constexpr double D65_Y = 1.00000;
constexpr double D65_Z = 1.08883;

// Lab constants
constexpr double LAB_EPSILON = 0.008856;
constexpr double LAB_KAPPA = 903.3;
constexpr double LAB_16_116 = 16.0 / 116.0;

// Helper functions
inline double Clamp(double val, double minVal, double maxVal) {
    return std::max(minVal, std::min(maxVal, val));
}

inline uint8_t ClampU8(double val) {
    return static_cast<uint8_t>(Clamp(val, 0.0, 255.0));
}

inline double SrgbToLinear(double val) {
    return (val <= 0.04045) ? val / 12.92 : std::pow((val + 0.055) / 1.055, 2.4);
}

inline double LinearToSrgb(double val) {
    return (val <= 0.0031308) ? val * 12.92 : 1.055 * std::pow(val, 1.0/2.4) - 0.055;
}

inline double LabF(double t) {
    return (t > LAB_EPSILON) ? std::cbrt(t) : (LAB_KAPPA * t + 16.0) / 116.0;
}

inline double LabFInv(double t) {
    double t3 = t * t * t;
    return (t3 > LAB_EPSILON) ? t3 : (116.0 * t - 16.0) / LAB_KAPPA;
}

} // anonymous namespace

// =============================================================================
// Utility Functions
// =============================================================================

std::string GetColorSpaceName(ColorSpace space) {
    switch (space) {
        case ColorSpace::Gray: return "gray";
        case ColorSpace::RGB: return "rgb";
        case ColorSpace::BGR: return "bgr";
        case ColorSpace::RGBA: return "rgba";
        case ColorSpace::BGRA: return "bgra";
        case ColorSpace::HSV: return "hsv";
        case ColorSpace::HSL: return "hsl";
        case ColorSpace::Lab: return "lab";
        case ColorSpace::Luv: return "luv";
        case ColorSpace::XYZ: return "xyz";
        case ColorSpace::YCrCb: return "ycrcb";
        case ColorSpace::YUV: return "yuv";
        default: return "unknown";
    }
}

ColorSpace ParseColorSpace(const std::string& name) {
    std::string lower = name;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    if (lower == "gray" || lower == "grey") return ColorSpace::Gray;
    if (lower == "rgb") return ColorSpace::RGB;
    if (lower == "bgr") return ColorSpace::BGR;
    if (lower == "rgba") return ColorSpace::RGBA;
    if (lower == "bgra") return ColorSpace::BGRA;
    if (lower == "hsv") return ColorSpace::HSV;
    if (lower == "hsl" || lower == "hls") return ColorSpace::HSL;
    if (lower == "lab" || lower == "cielab") return ColorSpace::Lab;
    if (lower == "luv" || lower == "cieluv") return ColorSpace::Luv;
    if (lower == "xyz" || lower == "ciexyz") return ColorSpace::XYZ;
    if (lower == "ycrcb" || lower == "ycbcr") return ColorSpace::YCrCb;
    if (lower == "yuv") return ColorSpace::YUV;

    throw InvalidArgumentException("Unknown color space: " + name);
}

int32_t GetChannelCount(ColorSpace space) {
    switch (space) {
        case ColorSpace::Gray: return 1;
        case ColorSpace::RGB:
        case ColorSpace::BGR:
        case ColorSpace::HSV:
        case ColorSpace::HSL:
        case ColorSpace::Lab:
        case ColorSpace::Luv:
        case ColorSpace::XYZ:
        case ColorSpace::YCrCb:
        case ColorSpace::YUV: return 3;
        case ColorSpace::RGBA:
        case ColorSpace::BGRA: return 4;
        default: return 1;
    }
}

bool HasAlphaChannel(ColorSpace space) {
    return space == ColorSpace::RGBA || space == ColorSpace::BGRA;
}

int32_t CountChannels(const QImage& image) {
    return image.Channels();
}

// =============================================================================
// Grayscale Conversion
// =============================================================================

QImage Rgb1ToGray(const QImage& image, const std::string& method) {
    if (image.Empty()) return QImage();

    // Already grayscale
    if (image.GetChannelType() == ChannelType::Gray) {
        return image.Clone();
    }

    if (image.Type() != PixelType::UInt8) {
        throw UnsupportedException("Rgb1ToGray only supports UInt8 images");
    }

    int srcChannels = image.Channels();
    if (srcChannels < 3) {
        throw InvalidArgumentException("Input must have at least 3 channels");
    }

    QImage result(image.Width(), image.Height(), PixelType::UInt8, ChannelType::Gray);

    // Determine weights based on method
    double rWeight, gWeight, bWeight;
    bool useMinMax = false;
    bool useMax = false;

    std::string lowerMethod = method;
    std::transform(lowerMethod.begin(), lowerMethod.end(), lowerMethod.begin(), ::tolower);

    if (lowerMethod == "luminosity" || lowerMethod == "bt601") {
        rWeight = 0.299; gWeight = 0.587; bWeight = 0.114;
    } else if (lowerMethod == "bt709") {
        rWeight = 0.2126; gWeight = 0.7152; bWeight = 0.0722;
    } else if (lowerMethod == "average") {
        rWeight = gWeight = bWeight = 1.0 / 3.0;
    } else if (lowerMethod == "lightness" || lowerMethod == "desaturate") {
        useMinMax = true;
    } else if (lowerMethod == "max") {
        useMinMax = true;
        useMax = true;
    } else if (lowerMethod == "min") {
        useMinMax = true;
    } else {
        // Default to luminosity
        rWeight = 0.299; gWeight = 0.587; bWeight = 0.114;
    }

    for (int32_t y = 0; y < image.Height(); ++y) {
        const uint8_t* src = static_cast<const uint8_t*>(image.RowPtr(y));
        uint8_t* dst = static_cast<uint8_t*>(result.RowPtr(y));

        for (int32_t x = 0; x < image.Width(); ++x) {
            int r = src[x * srcChannels + 0];
            int g = src[x * srcChannels + 1];
            int b = src[x * srcChannels + 2];

            uint8_t gray;
            if (useMinMax) {
                int maxVal = std::max({r, g, b});
                int minVal = std::min({r, g, b});
                gray = useMax ? static_cast<uint8_t>(maxVal)
                              : static_cast<uint8_t>((maxVal + minVal) / 2);
            } else {
                gray = ClampU8(r * rWeight + g * gWeight + b * bWeight);
            }
            dst[x] = gray;
        }
    }

    return result;
}

QImage Rgb3ToGray(const QImage& red, const QImage& green, const QImage& blue,
                   const std::string& method) {
    // Compose then convert
    QImage rgb = Compose3(red, green, blue, ChannelType::RGB);
    return Rgb1ToGray(rgb, method);
}

QImage GrayToRgb(const QImage& gray) {
    if (gray.Empty()) return QImage();

    if (gray.GetChannelType() != ChannelType::Gray) {
        throw InvalidArgumentException("Input must be grayscale");
    }

    QImage result(gray.Width(), gray.Height(), gray.Type(), ChannelType::RGB);

    for (int32_t y = 0; y < gray.Height(); ++y) {
        const uint8_t* src = static_cast<const uint8_t*>(gray.RowPtr(y));
        uint8_t* dst = static_cast<uint8_t*>(result.RowPtr(y));

        for (int32_t x = 0; x < gray.Width(); ++x) {
            uint8_t val = src[x];
            dst[x * 3 + 0] = val;
            dst[x * 3 + 1] = val;
            dst[x * 3 + 2] = val;
        }
    }

    return result;
}

// =============================================================================
// Channel Operations
// =============================================================================

void Decompose3(const QImage& image, QImage& ch1, QImage& ch2, QImage& ch3) {
    if (image.Empty()) {
        ch1 = ch2 = ch3 = QImage();
        return;
    }

    if (image.Channels() < 3) {
        throw InvalidArgumentException("Input must have at least 3 channels");
    }

    int32_t w = image.Width();
    int32_t h = image.Height();
    PixelType type = image.Type();
    int channels = image.Channels();

    ch1 = QImage(w, h, type, ChannelType::Gray);
    ch2 = QImage(w, h, type, ChannelType::Gray);
    ch3 = QImage(w, h, type, ChannelType::Gray);

    if (type == PixelType::UInt8) {
        for (int32_t y = 0; y < h; ++y) {
            const uint8_t* src = static_cast<const uint8_t*>(image.RowPtr(y));
            uint8_t* d1 = static_cast<uint8_t*>(ch1.RowPtr(y));
            uint8_t* d2 = static_cast<uint8_t*>(ch2.RowPtr(y));
            uint8_t* d3 = static_cast<uint8_t*>(ch3.RowPtr(y));

            for (int32_t x = 0; x < w; ++x) {
                d1[x] = src[x * channels + 0];
                d2[x] = src[x * channels + 1];
                d3[x] = src[x * channels + 2];
            }
        }
    }
}

void Decompose4(const QImage& image, QImage& ch1, QImage& ch2, QImage& ch3, QImage& ch4) {
    if (image.Empty()) {
        ch1 = ch2 = ch3 = ch4 = QImage();
        return;
    }

    if (image.Channels() < 4) {
        throw InvalidArgumentException("Input must have 4 channels");
    }

    int32_t w = image.Width();
    int32_t h = image.Height();
    PixelType type = image.Type();

    ch1 = QImage(w, h, type, ChannelType::Gray);
    ch2 = QImage(w, h, type, ChannelType::Gray);
    ch3 = QImage(w, h, type, ChannelType::Gray);
    ch4 = QImage(w, h, type, ChannelType::Gray);

    if (type == PixelType::UInt8) {
        for (int32_t y = 0; y < h; ++y) {
            const uint8_t* src = static_cast<const uint8_t*>(image.RowPtr(y));
            uint8_t* d1 = static_cast<uint8_t*>(ch1.RowPtr(y));
            uint8_t* d2 = static_cast<uint8_t*>(ch2.RowPtr(y));
            uint8_t* d3 = static_cast<uint8_t*>(ch3.RowPtr(y));
            uint8_t* d4 = static_cast<uint8_t*>(ch4.RowPtr(y));

            for (int32_t x = 0; x < w; ++x) {
                d1[x] = src[x * 4 + 0];
                d2[x] = src[x * 4 + 1];
                d3[x] = src[x * 4 + 2];
                d4[x] = src[x * 4 + 3];
            }
        }
    }
}

QImage Compose3(const QImage& ch1, const QImage& ch2, const QImage& ch3,
                 ChannelType channelType) {
    if (ch1.Empty() || ch2.Empty() || ch3.Empty()) {
        return QImage();
    }

    if (ch1.Width() != ch2.Width() || ch1.Width() != ch3.Width() ||
        ch1.Height() != ch2.Height() || ch1.Height() != ch3.Height()) {
        throw InvalidArgumentException("All channels must have same dimensions");
    }

    int32_t w = ch1.Width();
    int32_t h = ch1.Height();
    PixelType type = ch1.Type();

    QImage result(w, h, type, channelType);

    if (type == PixelType::UInt8) {
        for (int32_t y = 0; y < h; ++y) {
            const uint8_t* s1 = static_cast<const uint8_t*>(ch1.RowPtr(y));
            const uint8_t* s2 = static_cast<const uint8_t*>(ch2.RowPtr(y));
            const uint8_t* s3 = static_cast<const uint8_t*>(ch3.RowPtr(y));
            uint8_t* dst = static_cast<uint8_t*>(result.RowPtr(y));

            for (int32_t x = 0; x < w; ++x) {
                dst[x * 3 + 0] = s1[x];
                dst[x * 3 + 1] = s2[x];
                dst[x * 3 + 2] = s3[x];
            }
        }
    }

    return result;
}

QImage Compose4(const QImage& ch1, const QImage& ch2,
                 const QImage& ch3, const QImage& ch4,
                 ChannelType channelType) {
    if (ch1.Empty() || ch2.Empty() || ch3.Empty() || ch4.Empty()) {
        return QImage();
    }

    int32_t w = ch1.Width();
    int32_t h = ch1.Height();
    PixelType type = ch1.Type();

    QImage result(w, h, type, channelType);

    if (type == PixelType::UInt8) {
        for (int32_t y = 0; y < h; ++y) {
            const uint8_t* s1 = static_cast<const uint8_t*>(ch1.RowPtr(y));
            const uint8_t* s2 = static_cast<const uint8_t*>(ch2.RowPtr(y));
            const uint8_t* s3 = static_cast<const uint8_t*>(ch3.RowPtr(y));
            const uint8_t* s4 = static_cast<const uint8_t*>(ch4.RowPtr(y));
            uint8_t* dst = static_cast<uint8_t*>(result.RowPtr(y));

            for (int32_t x = 0; x < w; ++x) {
                dst[x * 4 + 0] = s1[x];
                dst[x * 4 + 1] = s2[x];
                dst[x * 4 + 2] = s3[x];
                dst[x * 4 + 3] = s4[x];
            }
        }
    }

    return result;
}

QImage AccessChannel(const QImage& image, int32_t channelIndex) {
    if (image.Empty()) return QImage();

    int channels = image.Channels();
    if (channelIndex < 0 || channelIndex >= channels) {
        throw InvalidArgumentException("Channel index out of range");
    }

    int32_t w = image.Width();
    int32_t h = image.Height();
    PixelType type = image.Type();

    QImage result(w, h, type, ChannelType::Gray);

    if (type == PixelType::UInt8) {
        for (int32_t y = 0; y < h; ++y) {
            const uint8_t* src = static_cast<const uint8_t*>(image.RowPtr(y));
            uint8_t* dst = static_cast<uint8_t*>(result.RowPtr(y));

            for (int32_t x = 0; x < w; ++x) {
                dst[x] = src[x * channels + channelIndex];
            }
        }
    }

    return result;
}

std::vector<QImage> SplitChannels(const QImage& image) {
    std::vector<QImage> channels;

    int numChannels = image.Channels();
    for (int i = 0; i < numChannels; ++i) {
        channels.push_back(AccessChannel(image, i));
    }

    return channels;
}

QImage MergeChannels(const std::vector<QImage>& channels, ChannelType channelType) {
    if (channels.empty()) return QImage();

    if (channels.size() == 3) {
        return Compose3(channels[0], channels[1], channels[2], channelType);
    } else if (channels.size() == 4) {
        return Compose4(channels[0], channels[1], channels[2], channels[3], channelType);
    } else if (channels.size() == 1) {
        return channels[0].Clone();
    }

    throw InvalidArgumentException("Unsupported number of channels");
}

// =============================================================================
// Channel Swapping
// =============================================================================

QImage RgbToBgr(const QImage& image) {
    return SwapChannels(image, 0, 2);
}

QImage BgrToRgb(const QImage& image) {
    return SwapChannels(image, 0, 2);
}

QImage SwapChannels(const QImage& image, int32_t ch1, int32_t ch2) {
    if (image.Empty()) return QImage();

    int channels = image.Channels();
    if (ch1 < 0 || ch1 >= channels || ch2 < 0 || ch2 >= channels) {
        throw InvalidArgumentException("Channel index out of range");
    }

    if (ch1 == ch2) return image.Clone();

    QImage result = image.Clone();
    int32_t w = image.Width();
    int32_t h = image.Height();

    if (image.Type() == PixelType::UInt8) {
        for (int32_t y = 0; y < h; ++y) {
            uint8_t* row = static_cast<uint8_t*>(result.RowPtr(y));
            for (int32_t x = 0; x < w; ++x) {
                std::swap(row[x * channels + ch1], row[x * channels + ch2]);
            }
        }
    }

    return result;
}

QImage ReorderChannels(const QImage& image, const std::vector<int32_t>& order) {
    if (image.Empty()) return QImage();

    int channels = image.Channels();
    if (static_cast<int>(order.size()) != channels) {
        throw InvalidArgumentException("Order size must match channel count");
    }

    QImage result(image.Width(), image.Height(), image.Type(), image.GetChannelType());
    int32_t w = image.Width();
    int32_t h = image.Height();

    if (image.Type() == PixelType::UInt8) {
        for (int32_t y = 0; y < h; ++y) {
            const uint8_t* src = static_cast<const uint8_t*>(image.RowPtr(y));
            uint8_t* dst = static_cast<uint8_t*>(result.RowPtr(y));

            for (int32_t x = 0; x < w; ++x) {
                for (int c = 0; c < channels; ++c) {
                    dst[x * channels + c] = src[x * channels + order[c]];
                }
            }
        }
    }

    return result;
}

// =============================================================================
// Color Space Conversion - RGB <-> HSV
// =============================================================================

namespace {

void RgbToHsv(uint8_t r, uint8_t g, uint8_t b, uint8_t& h, uint8_t& s, uint8_t& v) {
    double rd = r / 255.0;
    double gd = g / 255.0;
    double bd = b / 255.0;

    double maxVal = std::max({rd, gd, bd});
    double minVal = std::min({rd, gd, bd});
    double diff = maxVal - minVal;

    // Value
    v = ClampU8(maxVal * 255.0);

    // Saturation
    if (maxVal == 0) {
        s = 0;
    } else {
        s = ClampU8((diff / maxVal) * 255.0);
    }

    // Hue
    double hue = 0;
    if (diff > 0) {
        if (maxVal == rd) {
            hue = 60.0 * std::fmod((gd - bd) / diff + 6.0, 6.0);
        } else if (maxVal == gd) {
            hue = 60.0 * ((bd - rd) / diff + 2.0);
        } else {
            hue = 60.0 * ((rd - gd) / diff + 4.0);
        }
    }
    h = ClampU8(hue * 255.0 / 360.0);
}

void HsvToRgb(uint8_t h, uint8_t s, uint8_t v, uint8_t& r, uint8_t& g, uint8_t& b) {
    double hd = h * 360.0 / 255.0;
    double sd = s / 255.0;
    double vd = v / 255.0;

    if (sd == 0) {
        r = g = b = v;
        return;
    }

    double c = vd * sd;
    double x = c * (1.0 - std::fabs(std::fmod(hd / 60.0, 2.0) - 1.0));
    double m = vd - c;

    double rd, gd, bd;
    if (hd < 60) { rd = c; gd = x; bd = 0; }
    else if (hd < 120) { rd = x; gd = c; bd = 0; }
    else if (hd < 180) { rd = 0; gd = c; bd = x; }
    else if (hd < 240) { rd = 0; gd = x; bd = c; }
    else if (hd < 300) { rd = x; gd = 0; bd = c; }
    else { rd = c; gd = 0; bd = x; }

    r = ClampU8((rd + m) * 255.0);
    g = ClampU8((gd + m) * 255.0);
    b = ClampU8((bd + m) * 255.0);
}

void RgbToHsl(uint8_t r, uint8_t g, uint8_t b, uint8_t& h, uint8_t& s, uint8_t& l) {
    double rd = r / 255.0;
    double gd = g / 255.0;
    double bd = b / 255.0;

    double maxVal = std::max({rd, gd, bd});
    double minVal = std::min({rd, gd, bd});
    double diff = maxVal - minVal;

    // Lightness
    double ld = (maxVal + minVal) / 2.0;
    l = ClampU8(ld * 255.0);

    // Saturation
    if (diff == 0) {
        s = 0;
        h = 0;
        return;
    }

    double sd = diff / (1.0 - std::fabs(2.0 * ld - 1.0));
    s = ClampU8(sd * 255.0);

    // Hue
    double hue;
    if (maxVal == rd) {
        hue = 60.0 * std::fmod((gd - bd) / diff + 6.0, 6.0);
    } else if (maxVal == gd) {
        hue = 60.0 * ((bd - rd) / diff + 2.0);
    } else {
        hue = 60.0 * ((rd - gd) / diff + 4.0);
    }
    h = ClampU8(hue * 255.0 / 360.0);
}

void HslToRgb(uint8_t h, uint8_t s, uint8_t l, uint8_t& r, uint8_t& g, uint8_t& b) {
    double hd = h * 360.0 / 255.0;
    double sd = s / 255.0;
    double ld = l / 255.0;

    if (sd == 0) {
        r = g = b = l;
        return;
    }

    double c = (1.0 - std::fabs(2.0 * ld - 1.0)) * sd;
    double x = c * (1.0 - std::fabs(std::fmod(hd / 60.0, 2.0) - 1.0));
    double m = ld - c / 2.0;

    double rd, gd, bd;
    if (hd < 60) { rd = c; gd = x; bd = 0; }
    else if (hd < 120) { rd = x; gd = c; bd = 0; }
    else if (hd < 180) { rd = 0; gd = c; bd = x; }
    else if (hd < 240) { rd = 0; gd = x; bd = c; }
    else if (hd < 300) { rd = x; gd = 0; bd = c; }
    else { rd = c; gd = 0; bd = x; }

    r = ClampU8((rd + m) * 255.0);
    g = ClampU8((gd + m) * 255.0);
    b = ClampU8((bd + m) * 255.0);
}

void RgbToYCrCb(uint8_t r, uint8_t g, uint8_t b, uint8_t& y, uint8_t& cr, uint8_t& cb) {
    // ITU-R BT.601 conversion
    y  = ClampU8(0.299 * r + 0.587 * g + 0.114 * b);
    cb = ClampU8(128.0 - 0.168736 * r - 0.331264 * g + 0.5 * b);
    cr = ClampU8(128.0 + 0.5 * r - 0.418688 * g - 0.081312 * b);
}

void YCrCbToRgb(uint8_t y, uint8_t cr, uint8_t cb, uint8_t& r, uint8_t& g, uint8_t& b) {
    double yd = y;
    double crd = cr - 128.0;
    double cbd = cb - 128.0;

    r = ClampU8(yd + 1.402 * crd);
    g = ClampU8(yd - 0.344136 * cbd - 0.714136 * crd);
    b = ClampU8(yd + 1.772 * cbd);
}

} // anonymous namespace

// =============================================================================
// Color Space Conversion - Main Functions
// =============================================================================

QImage TransFromRgb(const QImage& image, ColorSpace toSpace) {
    if (image.Empty()) return QImage();

    if (image.Type() != PixelType::UInt8) {
        throw UnsupportedException("TransFromRgb only supports UInt8 images");
    }

    if (image.Channels() < 3) {
        throw InvalidArgumentException("Input must have at least 3 channels");
    }

    // Handle simple cases
    if (toSpace == ColorSpace::RGB) return image.Clone();
    if (toSpace == ColorSpace::Gray) return Rgb1ToGray(image, "luminosity");
    if (toSpace == ColorSpace::BGR) return RgbToBgr(image);

    int32_t w = image.Width();
    int32_t h = image.Height();
    int srcChannels = image.Channels();

    QImage result(w, h, PixelType::UInt8, ChannelType::RGB);

    for (int32_t y = 0; y < h; ++y) {
        const uint8_t* src = static_cast<const uint8_t*>(image.RowPtr(y));
        uint8_t* dst = static_cast<uint8_t*>(result.RowPtr(y));

        for (int32_t x = 0; x < w; ++x) {
            uint8_t r = src[x * srcChannels + 0];
            uint8_t g = src[x * srcChannels + 1];
            uint8_t b = src[x * srcChannels + 2];

            uint8_t c1, c2, c3;

            switch (toSpace) {
                case ColorSpace::HSV:
                    RgbToHsv(r, g, b, c1, c2, c3);
                    break;
                case ColorSpace::HSL:
                    RgbToHsl(r, g, b, c1, c2, c3);
                    break;
                case ColorSpace::YCrCb:
                    RgbToYCrCb(r, g, b, c1, c3, c2);  // Y, Cr, Cb order
                    break;
                case ColorSpace::YUV:
                    // YUV is similar to YCrCb for our purposes
                    RgbToYCrCb(r, g, b, c1, c3, c2);
                    break;
                default:
                    c1 = r; c2 = g; c3 = b;
                    break;
            }

            dst[x * 3 + 0] = c1;
            dst[x * 3 + 1] = c2;
            dst[x * 3 + 2] = c3;
        }
    }

    return result;
}

QImage TransFromRgb(const QImage& image, const std::string& colorSpace) {
    return TransFromRgb(image, ParseColorSpace(colorSpace));
}

QImage TransToRgb(const QImage& image, ColorSpace fromSpace) {
    if (image.Empty()) return QImage();

    if (image.Type() != PixelType::UInt8) {
        throw UnsupportedException("TransToRgb only supports UInt8 images");
    }

    // Handle simple cases
    if (fromSpace == ColorSpace::RGB) return image.Clone();
    if (fromSpace == ColorSpace::Gray) return GrayToRgb(image);
    if (fromSpace == ColorSpace::BGR) return BgrToRgb(image);

    if (image.Channels() < 3) {
        throw InvalidArgumentException("Input must have at least 3 channels");
    }

    int32_t w = image.Width();
    int32_t h = image.Height();
    int srcChannels = image.Channels();

    QImage result(w, h, PixelType::UInt8, ChannelType::RGB);

    for (int32_t y = 0; y < h; ++y) {
        const uint8_t* src = static_cast<const uint8_t*>(image.RowPtr(y));
        uint8_t* dst = static_cast<uint8_t*>(result.RowPtr(y));

        for (int32_t x = 0; x < w; ++x) {
            uint8_t c1 = src[x * srcChannels + 0];
            uint8_t c2 = src[x * srcChannels + 1];
            uint8_t c3 = src[x * srcChannels + 2];

            uint8_t r, g, b;

            switch (fromSpace) {
                case ColorSpace::HSV:
                    HsvToRgb(c1, c2, c3, r, g, b);
                    break;
                case ColorSpace::HSL:
                    HslToRgb(c1, c2, c3, r, g, b);
                    break;
                case ColorSpace::YCrCb:
                    YCrCbToRgb(c1, c3, c2, r, g, b);
                    break;
                case ColorSpace::YUV:
                    YCrCbToRgb(c1, c3, c2, r, g, b);
                    break;
                default:
                    r = c1; g = c2; b = c3;
                    break;
            }

            dst[x * 3 + 0] = r;
            dst[x * 3 + 1] = g;
            dst[x * 3 + 2] = b;
        }
    }

    return result;
}

QImage TransToRgb(const QImage& image, const std::string& colorSpace) {
    return TransToRgb(image, ParseColorSpace(colorSpace));
}

QImage ConvertColorSpace(const QImage& image, ColorSpace fromSpace, ColorSpace toSpace) {
    if (fromSpace == toSpace) return image.Clone();

    // Convert via RGB as intermediate
    if (fromSpace != ColorSpace::RGB) {
        QImage rgb = TransToRgb(image, fromSpace);
        return TransFromRgb(rgb, toSpace);
    } else {
        return TransFromRgb(image, toSpace);
    }
}

// =============================================================================
// Color Adjustment
// =============================================================================

QImage AdjustBrightness(const QImage& image, double brightness) {
    if (image.Empty()) return QImage();

    QImage result = image.Clone();
    int32_t w = image.Width();
    int32_t h = image.Height();
    int channels = image.Channels();

    if (image.Type() == PixelType::UInt8) {
        for (int32_t y = 0; y < h; ++y) {
            uint8_t* row = static_cast<uint8_t*>(result.RowPtr(y));
            for (int32_t x = 0; x < w * channels; ++x) {
                row[x] = ClampU8(row[x] + brightness);
            }
        }
    }

    return result;
}

QImage AdjustContrast(const QImage& image, double contrast) {
    if (image.Empty()) return QImage();

    QImage result = image.Clone();
    int32_t w = image.Width();
    int32_t h = image.Height();
    int channels = image.Channels();

    double factor = contrast;

    if (image.Type() == PixelType::UInt8) {
        for (int32_t y = 0; y < h; ++y) {
            uint8_t* row = static_cast<uint8_t*>(result.RowPtr(y));
            for (int32_t x = 0; x < w * channels; ++x) {
                double val = (row[x] - 128.0) * factor + 128.0;
                row[x] = ClampU8(val);
            }
        }
    }

    return result;
}

QImage AdjustSaturation(const QImage& image, double saturation) {
    if (image.Empty()) return QImage();

    if (image.Channels() < 3) {
        return image.Clone();  // No saturation adjustment for grayscale
    }

    QImage hsv = TransFromRgb(image, ColorSpace::HSV);
    int32_t w = hsv.Width();
    int32_t h = hsv.Height();

    for (int32_t y = 0; y < h; ++y) {
        uint8_t* row = static_cast<uint8_t*>(hsv.RowPtr(y));
        for (int32_t x = 0; x < w; ++x) {
            double s = row[x * 3 + 1] * saturation;
            row[x * 3 + 1] = ClampU8(s);
        }
    }

    return TransToRgb(hsv, ColorSpace::HSV);
}

QImage AdjustHue(const QImage& image, double hueShift) {
    if (image.Empty()) return QImage();

    if (image.Channels() < 3) {
        return image.Clone();
    }

    QImage hsv = TransFromRgb(image, ColorSpace::HSV);
    int32_t w = hsv.Width();
    int32_t h = hsv.Height();

    double shift = hueShift * 255.0 / 360.0;  // Convert degrees to 0-255 range

    for (int32_t y = 0; y < h; ++y) {
        uint8_t* row = static_cast<uint8_t*>(hsv.RowPtr(y));
        for (int32_t x = 0; x < w; ++x) {
            double hue = row[x * 3 + 0] + shift;
            while (hue < 0) hue += 256;
            while (hue >= 256) hue -= 256;
            row[x * 3 + 0] = static_cast<uint8_t>(hue);
        }
    }

    return TransToRgb(hsv, ColorSpace::HSV);
}

QImage AdjustGamma(const QImage& image, double gamma) {
    if (image.Empty()) return QImage();

    QImage result = image.Clone();
    int32_t w = image.Width();
    int32_t h = image.Height();
    int channels = image.Channels();

    // Build lookup table
    uint8_t lut[256];
    double invGamma = 1.0 / gamma;
    for (int i = 0; i < 256; ++i) {
        lut[i] = ClampU8(std::pow(i / 255.0, invGamma) * 255.0);
    }

    if (image.Type() == PixelType::UInt8) {
        for (int32_t y = 0; y < h; ++y) {
            uint8_t* row = static_cast<uint8_t*>(result.RowPtr(y));
            for (int32_t x = 0; x < w * channels; ++x) {
                row[x] = lut[row[x]];
            }
        }
    }

    return result;
}

QImage InvertColors(const QImage& image) {
    if (image.Empty()) return QImage();

    QImage result = image.Clone();
    int32_t w = image.Width();
    int32_t h = image.Height();
    int channels = image.Channels();

    if (image.Type() == PixelType::UInt8) {
        for (int32_t y = 0; y < h; ++y) {
            uint8_t* row = static_cast<uint8_t*>(result.RowPtr(y));
            for (int32_t x = 0; x < w * channels; ++x) {
                row[x] = 255 - row[x];
            }
        }
    }

    return result;
}

// =============================================================================
// White Balance
// =============================================================================

QImage AutoWhiteBalance(const QImage& image, const std::string& method) {
    if (image.Empty() || image.Channels() < 3) {
        return image.Clone();
    }

    std::string lowerMethod = method;
    std::transform(lowerMethod.begin(), lowerMethod.end(), lowerMethod.begin(), ::tolower);

    int32_t w = image.Width();
    int32_t h = image.Height();
    int channels = image.Channels();

    if (lowerMethod == "gray_world") {
        // Calculate average of each channel
        double sumR = 0, sumG = 0, sumB = 0;
        int64_t count = w * h;

        for (int32_t y = 0; y < h; ++y) {
            const uint8_t* row = static_cast<const uint8_t*>(image.RowPtr(y));
            for (int32_t x = 0; x < w; ++x) {
                sumR += row[x * channels + 0];
                sumG += row[x * channels + 1];
                sumB += row[x * channels + 2];
            }
        }

        double avgR = sumR / count;
        double avgG = sumG / count;
        double avgB = sumB / count;
        double avgGray = (avgR + avgG + avgB) / 3.0;

        return ApplyWhiteBalance(image, avgGray / avgR, avgGray / avgG, avgGray / avgB);
    } else if (lowerMethod == "white_patch") {
        // Find max of each channel
        uint8_t maxR = 0, maxG = 0, maxB = 0;

        for (int32_t y = 0; y < h; ++y) {
            const uint8_t* row = static_cast<const uint8_t*>(image.RowPtr(y));
            for (int32_t x = 0; x < w; ++x) {
                maxR = std::max(maxR, row[x * channels + 0]);
                maxG = std::max(maxG, row[x * channels + 1]);
                maxB = std::max(maxB, row[x * channels + 2]);
            }
        }

        double scaleR = maxR > 0 ? 255.0 / maxR : 1.0;
        double scaleG = maxG > 0 ? 255.0 / maxG : 1.0;
        double scaleB = maxB > 0 ? 255.0 / maxB : 1.0;

        return ApplyWhiteBalance(image, scaleR, scaleG, scaleB);
    }

    // Default: return unchanged
    return image.Clone();
}

QImage ApplyWhiteBalance(const QImage& image, double whiteR, double whiteG, double whiteB) {
    if (image.Empty() || image.Channels() < 3) {
        return image.Clone();
    }

    QImage result = image.Clone();
    int32_t w = image.Width();
    int32_t h = image.Height();
    int channels = image.Channels();

    for (int32_t y = 0; y < h; ++y) {
        uint8_t* row = static_cast<uint8_t*>(result.RowPtr(y));
        for (int32_t x = 0; x < w; ++x) {
            row[x * channels + 0] = ClampU8(row[x * channels + 0] * whiteR);
            row[x * channels + 1] = ClampU8(row[x * channels + 1] * whiteG);
            row[x * channels + 2] = ClampU8(row[x * channels + 2] * whiteB);
        }
    }

    return result;
}

// =============================================================================
// Color Transform LUT
// =============================================================================

ColorTransLut::ColorTransLut() = default;
ColorTransLut::~ColorTransLut() = default;
ColorTransLut::ColorTransLut(ColorTransLut&& other) noexcept = default;
ColorTransLut& ColorTransLut::operator=(ColorTransLut&& other) noexcept = default;

bool ColorTransLut::IsValid() const {
    return !lut_.empty();
}

ColorTransLut CreateColorTransLut(const std::string& colorSpace,
                                   const std::string& transDirection,
                                   int32_t numBits) {
    if (numBits != 8) {
        throw UnsupportedException("CreateColorTransLut only supports 8-bit images");
    }

    ColorTransLut lut;
    ColorSpace targetSpace = ParseColorSpace(colorSpace);

    bool fromRgb = (transDirection == "from_rgb");
    lut.fromSpace_ = fromRgb ? ColorSpace::RGB : targetSpace;
    lut.toSpace_ = fromRgb ? targetSpace : ColorSpace::RGB;

    // Allocate 48MB LUT: 256 * 256 * 256 * 3 bytes
    constexpr size_t LUT_SIZE = 256 * 256 * 256 * 3;
    lut.lut_.resize(LUT_SIZE);

    // Pre-compute all conversions
    for (int r = 0; r < 256; ++r) {
        for (int g = 0; g < 256; ++g) {
            for (int b = 0; b < 256; ++b) {
                size_t idx = (static_cast<size_t>(r) * 256 * 256 + g * 256 + b) * 3;

                uint8_t c1, c2, c3;

                if (fromRgb) {
                    // RGB -> target
                    switch (targetSpace) {
                        case ColorSpace::HSV:
                            RgbToHsv(r, g, b, c1, c2, c3);
                            break;
                        case ColorSpace::HSL:
                            RgbToHsl(r, g, b, c1, c2, c3);
                            break;
                        case ColorSpace::YCrCb:
                        case ColorSpace::YUV:
                            RgbToYCrCb(r, g, b, c1, c3, c2);
                            break;
                        default:
                            c1 = r; c2 = g; c3 = b;
                            break;
                    }
                } else {
                    // target -> RGB
                    switch (targetSpace) {
                        case ColorSpace::HSV:
                            HsvToRgb(r, g, b, c1, c2, c3);
                            break;
                        case ColorSpace::HSL:
                            HslToRgb(r, g, b, c1, c2, c3);
                            break;
                        case ColorSpace::YCrCb:
                        case ColorSpace::YUV:
                            YCrCbToRgb(r, b, g, c1, c2, c3);
                            break;
                        default:
                            c1 = r; c2 = g; c3 = b;
                            break;
                    }
                }

                lut.lut_[idx + 0] = c1;
                lut.lut_[idx + 1] = c2;
                lut.lut_[idx + 2] = c3;
            }
        }
    }

    return lut;
}

void ApplyColorTransLut(const QImage& image1, const QImage& image2, const QImage& image3,
                        QImage& result1, QImage& result2, QImage& result3,
                        const ColorTransLut& lut) {
    if (!lut.IsValid()) {
        throw InvalidArgumentException("Invalid ColorTransLut handle");
    }

    if (image1.Empty() || image2.Empty() || image3.Empty()) {
        throw InvalidArgumentException("Input images cannot be empty");
    }

    int32_t w = image1.Width();
    int32_t h = image1.Height();

    if (image2.Width() != w || image2.Height() != h ||
        image3.Width() != w || image3.Height() != h) {
        throw InvalidArgumentException("All input images must have the same size");
    }

    // Create output images
    result1 = QImage(w, h, PixelType::UInt8, ChannelType::Gray);
    result2 = QImage(w, h, PixelType::UInt8, ChannelType::Gray);
    result3 = QImage(w, h, PixelType::UInt8, ChannelType::Gray);

    const uint8_t* lutData = lut.lut_.data();

    for (int32_t y = 0; y < h; ++y) {
        const uint8_t* src1 = static_cast<const uint8_t*>(image1.RowPtr(y));
        const uint8_t* src2 = static_cast<const uint8_t*>(image2.RowPtr(y));
        const uint8_t* src3 = static_cast<const uint8_t*>(image3.RowPtr(y));
        uint8_t* dst1 = static_cast<uint8_t*>(result1.RowPtr(y));
        uint8_t* dst2 = static_cast<uint8_t*>(result2.RowPtr(y));
        uint8_t* dst3 = static_cast<uint8_t*>(result3.RowPtr(y));

        for (int32_t x = 0; x < w; ++x) {
            size_t idx = (static_cast<size_t>(src1[x]) * 256 * 256 +
                          src2[x] * 256 + src3[x]) * 3;
            dst1[x] = lutData[idx + 0];
            dst2[x] = lutData[idx + 1];
            dst3[x] = lutData[idx + 2];
        }
    }
}

QImage ApplyColorTransLut(const QImage& image, const ColorTransLut& lut) {
    if (!lut.IsValid()) {
        throw InvalidArgumentException("Invalid ColorTransLut handle");
    }

    if (image.Empty() || image.Channels() != 3) {
        throw InvalidArgumentException("Input must be a 3-channel image");
    }

    int32_t w = image.Width();
    int32_t h = image.Height();

    QImage result(w, h, PixelType::UInt8, ChannelType::RGB);
    const uint8_t* lutData = lut.lut_.data();

    for (int32_t y = 0; y < h; ++y) {
        const uint8_t* src = static_cast<const uint8_t*>(image.RowPtr(y));
        uint8_t* dst = static_cast<uint8_t*>(result.RowPtr(y));

        for (int32_t x = 0; x < w; ++x) {
            size_t idx = (static_cast<size_t>(src[x * 3]) * 256 * 256 +
                          src[x * 3 + 1] * 256 + src[x * 3 + 2]) * 3;
            dst[x * 3 + 0] = lutData[idx + 0];
            dst[x * 3 + 1] = lutData[idx + 1];
            dst[x * 3 + 2] = lutData[idx + 2];
        }
    }

    return result;
}

void ClearColorTransLut(ColorTransLut& lut) {
    lut.lut_.clear();
    lut.lut_.shrink_to_fit();
    lut.fromSpace_ = ColorSpace::RGB;
    lut.toSpace_ = ColorSpace::RGB;
}

// =============================================================================
// Bayer Pattern (CFA) Conversion
// =============================================================================

namespace {

// Bayer pattern offsets
enum class BayerPattern { RGGB, GRBG, GBRG, BGGR };

BayerPattern ParseBayerPattern(const std::string& cfaType) {
    std::string lower = cfaType;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    if (lower == "bayer_rg" || lower == "rggb") return BayerPattern::RGGB;
    if (lower == "bayer_gr" || lower == "grbg") return BayerPattern::GRBG;
    if (lower == "bayer_gb" || lower == "gbrg") return BayerPattern::GBRG;
    if (lower == "bayer_bg" || lower == "bggr") return BayerPattern::BGGR;

    return BayerPattern::GBRG;  // Default
}

// Get Bayer color at position (for RGGB pattern at origin)
// 0=R, 1=G, 2=B
inline int GetBayerColor(int x, int y, BayerPattern pattern) {
    int baseX = x & 1;
    int baseY = y & 1;
    int pos = baseY * 2 + baseX;

    // Pattern encoding: [0,0]=TL, [0,1]=TR, [1,0]=BL, [1,1]=BR
    static const int patterns[4][4] = {
        {0, 1, 1, 2},  // RGGB: R G / G B
        {1, 0, 2, 1},  // GRBG: G R / B G
        {1, 2, 0, 1},  // GBRG: G B / R G
        {2, 1, 1, 0}   // BGGR: B G / G R
    };

    return patterns[static_cast<int>(pattern)][pos];
}

} // anonymous namespace

QImage CfaToRgb(const QImage& cfaImage,
                const std::string& cfaType,
                const std::string& interpolation) {
    if (cfaImage.Empty() || cfaImage.Channels() != 1) {
        throw InvalidArgumentException("CfaToRgb requires a single-channel image");
    }

    int32_t w = cfaImage.Width();
    int32_t h = cfaImage.Height();

    if (w < 2 || h < 2) {
        throw InvalidArgumentException("Image too small for Bayer interpolation");
    }

    BayerPattern pattern = ParseBayerPattern(cfaType);
    bool useDirectional = (interpolation == "bilinear_dir");

    QImage result(w, h, cfaImage.Type(),
                  cfaImage.Type() == PixelType::UInt16 ? ChannelType::RGB : ChannelType::RGB);

    if (cfaImage.Type() == PixelType::UInt8) {
        // 8-bit processing
        for (int32_t y = 1; y < h - 1; ++y) {
            const uint8_t* prevRow = static_cast<const uint8_t*>(cfaImage.RowPtr(y - 1));
            const uint8_t* currRow = static_cast<const uint8_t*>(cfaImage.RowPtr(y));
            const uint8_t* nextRow = static_cast<const uint8_t*>(cfaImage.RowPtr(y + 1));
            uint8_t* dst = static_cast<uint8_t*>(result.RowPtr(y));

            for (int32_t x = 1; x < w - 1; ++x) {
                int color = GetBayerColor(x, y, pattern);
                uint8_t r, g, b;

                // Get the known color at this position
                uint8_t center = currRow[x];

                if (color == 0) {  // Red pixel
                    r = center;
                    // Green: average of 4 neighbors
                    g = (currRow[x-1] + currRow[x+1] + prevRow[x] + nextRow[x]) / 4;
                    // Blue: average of 4 diagonal neighbors
                    b = (prevRow[x-1] + prevRow[x+1] + nextRow[x-1] + nextRow[x+1]) / 4;
                } else if (color == 2) {  // Blue pixel
                    b = center;
                    g = (currRow[x-1] + currRow[x+1] + prevRow[x] + nextRow[x]) / 4;
                    r = (prevRow[x-1] + prevRow[x+1] + nextRow[x-1] + nextRow[x+1]) / 4;
                } else {  // Green pixel
                    g = center;
                    // Determine if R or B is on the same row
                    int leftColor = GetBayerColor(x - 1, y, pattern);
                    if (leftColor == 0) {  // R on left/right
                        r = (currRow[x-1] + currRow[x+1]) / 2;
                        b = (prevRow[x] + nextRow[x]) / 2;
                    } else {  // B on left/right
                        b = (currRow[x-1] + currRow[x+1]) / 2;
                        r = (prevRow[x] + nextRow[x]) / 2;
                    }
                }

                dst[x * 3 + 0] = r;
                dst[x * 3 + 1] = g;
                dst[x * 3 + 2] = b;
            }
        }

        // Handle borders (simple replication)
        // Top row
        const uint8_t* row1 = static_cast<const uint8_t*>(result.RowPtr(1));
        uint8_t* row0 = static_cast<uint8_t*>(result.RowPtr(0));
        std::memcpy(row0, row1, w * 3);

        // Bottom row
        const uint8_t* rowN1 = static_cast<const uint8_t*>(result.RowPtr(h - 2));
        uint8_t* rowN = static_cast<uint8_t*>(result.RowPtr(h - 1));
        std::memcpy(rowN, rowN1, w * 3);

        // Left and right columns
        for (int32_t y = 0; y < h; ++y) {
            uint8_t* row = static_cast<uint8_t*>(result.RowPtr(y));
            // Left
            row[0] = row[3];
            row[1] = row[4];
            row[2] = row[5];
            // Right
            row[(w-1) * 3 + 0] = row[(w-2) * 3 + 0];
            row[(w-1) * 3 + 1] = row[(w-2) * 3 + 1];
            row[(w-1) * 3 + 2] = row[(w-2) * 3 + 2];
        }
    } else if (cfaImage.Type() == PixelType::UInt16) {
        // 16-bit processing (similar logic)
        for (int32_t y = 1; y < h - 1; ++y) {
            const uint16_t* prevRow = static_cast<const uint16_t*>(cfaImage.RowPtr(y - 1));
            const uint16_t* currRow = static_cast<const uint16_t*>(cfaImage.RowPtr(y));
            const uint16_t* nextRow = static_cast<const uint16_t*>(cfaImage.RowPtr(y + 1));
            uint16_t* dst = static_cast<uint16_t*>(result.RowPtr(y));

            for (int32_t x = 1; x < w - 1; ++x) {
                int color = GetBayerColor(x, y, pattern);
                uint16_t r, g, b;
                uint16_t center = currRow[x];

                if (color == 0) {
                    r = center;
                    g = (currRow[x-1] + currRow[x+1] + prevRow[x] + nextRow[x]) / 4;
                    b = (prevRow[x-1] + prevRow[x+1] + nextRow[x-1] + nextRow[x+1]) / 4;
                } else if (color == 2) {
                    b = center;
                    g = (currRow[x-1] + currRow[x+1] + prevRow[x] + nextRow[x]) / 4;
                    r = (prevRow[x-1] + prevRow[x+1] + nextRow[x-1] + nextRow[x+1]) / 4;
                } else {
                    g = center;
                    int leftColor = GetBayerColor(x - 1, y, pattern);
                    if (leftColor == 0) {
                        r = (currRow[x-1] + currRow[x+1]) / 2;
                        b = (prevRow[x] + nextRow[x]) / 2;
                    } else {
                        b = (currRow[x-1] + currRow[x+1]) / 2;
                        r = (prevRow[x] + nextRow[x]) / 2;
                    }
                }

                dst[x * 3 + 0] = r;
                dst[x * 3 + 1] = g;
                dst[x * 3 + 2] = b;
            }
        }

        // Border handling for 16-bit
        const uint16_t* row1 = static_cast<const uint16_t*>(result.RowPtr(1));
        uint16_t* row0 = static_cast<uint16_t*>(result.RowPtr(0));
        std::memcpy(row0, row1, w * 3 * sizeof(uint16_t));

        const uint16_t* rowN1 = static_cast<const uint16_t*>(result.RowPtr(h - 2));
        uint16_t* rowN = static_cast<uint16_t*>(result.RowPtr(h - 1));
        std::memcpy(rowN, rowN1, w * 3 * sizeof(uint16_t));

        for (int32_t y = 0; y < h; ++y) {
            uint16_t* row = static_cast<uint16_t*>(result.RowPtr(y));
            row[0] = row[3]; row[1] = row[4]; row[2] = row[5];
            row[(w-1)*3+0] = row[(w-2)*3+0];
            row[(w-1)*3+1] = row[(w-2)*3+1];
            row[(w-1)*3+2] = row[(w-2)*3+2];
        }
    }

    // Suppress unused warning
    (void)useDirectional;

    return result;
}

// =============================================================================
// Linear Color Transformation
// =============================================================================

QImage LinearTransColor(const QImage& image,
                        const std::vector<double>& transMat,
                        int32_t numOutputChannels) {
    if (image.Empty()) return QImage();

    int32_t w = image.Width();
    int32_t h = image.Height();
    int32_t inChannels = image.Channels();

    // Matrix is m x (n+1) where m = output channels, n = input channels
    int32_t expectedSize = numOutputChannels * (inChannels + 1);
    if (static_cast<int32_t>(transMat.size()) != expectedSize) {
        throw InvalidArgumentException(
            "transMat size must be numOutputChannels * (numInputChannels + 1)");
    }

    // Output is always Float32
    ChannelType outChannelType = ChannelType::Gray;
    if (numOutputChannels == 3) outChannelType = ChannelType::RGB;
    else if (numOutputChannels == 4) outChannelType = ChannelType::RGBA;

    QImage result(w, h, PixelType::Float32, outChannelType);

    // Convert input to float if needed
    QImage floatInput = image.Type() == PixelType::Float32 ?
                        image : image.ConvertTo(PixelType::Float32);

    for (int32_t y = 0; y < h; ++y) {
        const float* src = static_cast<const float*>(floatInput.RowPtr(y));
        float* dst = static_cast<float*>(result.RowPtr(y));

        for (int32_t x = 0; x < w; ++x) {
            for (int32_t outCh = 0; outCh < numOutputChannels; ++outCh) {
                double sum = 0.0;
                int32_t rowOffset = outCh * (inChannels + 1);

                // Linear combination of input channels
                for (int32_t inCh = 0; inCh < inChannels; ++inCh) {
                    sum += transMat[rowOffset + inCh] * src[x * inChannels + inCh];
                }
                // Add offset (last column)
                sum += transMat[rowOffset + inChannels];

                dst[x * numOutputChannels + outCh] = static_cast<float>(sum);
            }
        }
    }

    return result;
}

QImage ApplyColorMatrix(const QImage& image, const std::vector<double>& matrix) {
    if (matrix.size() != 9) {
        throw InvalidArgumentException("ApplyColorMatrix requires a 3x3 matrix (9 elements)");
    }

    // Convert 3x3 matrix to 3x4 (add zero offsets)
    std::vector<double> transMat = {
        matrix[0], matrix[1], matrix[2], 0.0,
        matrix[3], matrix[4], matrix[5], 0.0,
        matrix[6], matrix[7], matrix[8], 0.0
    };

    return LinearTransColor(image, transMat, 3);
}

// =============================================================================
// Principal Component Analysis
// =============================================================================

QImage PrincipalComp(const QImage& image, int32_t numComponents) {
    if (image.Empty()) return QImage();

    int32_t channels = image.Channels();
    if (numComponents <= 0 || numComponents > channels) {
        numComponents = channels;
    }

    // Get transformation matrix
    std::vector<double> transMat, mean, eigenvalues;
    GenPrincipalCompTrans(image, transMat, mean, eigenvalues);

    // Apply transformation (only first numComponents rows)
    int32_t w = image.Width();
    int32_t h = image.Height();

    ChannelType outType = ChannelType::Gray;
    if (numComponents == 3) outType = ChannelType::RGB;
    else if (numComponents == 4) outType = ChannelType::RGBA;

    QImage result(w, h, PixelType::Float32, outType);

    QImage floatInput = image.Type() == PixelType::Float32 ?
                        image : image.ConvertTo(PixelType::Float32);

    for (int32_t y = 0; y < h; ++y) {
        const float* src = static_cast<const float*>(floatInput.RowPtr(y));
        float* dst = static_cast<float*>(result.RowPtr(y));

        for (int32_t x = 0; x < w; ++x) {
            for (int32_t outCh = 0; outCh < numComponents; ++outCh) {
                double sum = 0.0;
                for (int32_t inCh = 0; inCh < channels; ++inCh) {
                    double centered = src[x * channels + inCh] - mean[inCh];
                    sum += transMat[outCh * channels + inCh] * centered;
                }
                dst[x * numComponents + outCh] = static_cast<float>(sum);
            }
        }
    }

    return result;
}

void GenPrincipalCompTrans(const QImage& image,
                           std::vector<double>& transMat,
                           std::vector<double>& mean,
                           std::vector<double>& eigenvalues) {
    if (image.Empty()) {
        throw InvalidArgumentException("Input image cannot be empty");
    }

    int32_t w = image.Width();
    int32_t h = image.Height();
    int32_t channels = image.Channels();
    int64_t n = static_cast<int64_t>(w) * h;

    // Convert to float
    QImage floatInput = image.Type() == PixelType::Float32 ?
                        image : image.ConvertTo(PixelType::Float32);

    // Compute mean
    mean.resize(channels, 0.0);
    for (int32_t y = 0; y < h; ++y) {
        const float* row = static_cast<const float*>(floatInput.RowPtr(y));
        for (int32_t x = 0; x < w; ++x) {
            for (int32_t c = 0; c < channels; ++c) {
                mean[c] += row[x * channels + c];
            }
        }
    }
    for (int32_t c = 0; c < channels; ++c) {
        mean[c] /= n;
    }

    // Compute covariance matrix
    std::vector<double> cov(channels * channels, 0.0);
    for (int32_t y = 0; y < h; ++y) {
        const float* row = static_cast<const float*>(floatInput.RowPtr(y));
        for (int32_t x = 0; x < w; ++x) {
            for (int32_t i = 0; i < channels; ++i) {
                double vi = row[x * channels + i] - mean[i];
                for (int32_t j = i; j < channels; ++j) {
                    double vj = row[x * channels + j] - mean[j];
                    cov[i * channels + j] += vi * vj;
                }
            }
        }
    }

    // Make symmetric and normalize
    for (int32_t i = 0; i < channels; ++i) {
        for (int32_t j = i; j < channels; ++j) {
            cov[i * channels + j] /= (n - 1);
            cov[j * channels + i] = cov[i * channels + j];
        }
    }

    // Simple power iteration for eigenvalue decomposition
    // (For small channel counts like 3-4, this is sufficient)
    transMat.resize(channels * channels);
    eigenvalues.resize(channels);

    // Initialize with identity
    for (int32_t i = 0; i < channels; ++i) {
        for (int32_t j = 0; j < channels; ++j) {
            transMat[i * channels + j] = (i == j) ? 1.0 : 0.0;
        }
    }

    // Jacobi iteration (simplified)
    std::vector<double> tempCov = cov;

    for (int iter = 0; iter < 50; ++iter) {
        // Find largest off-diagonal element
        int32_t p = 0, q = 1;
        double maxVal = 0.0;
        for (int32_t i = 0; i < channels; ++i) {
            for (int32_t j = i + 1; j < channels; ++j) {
                if (std::abs(tempCov[i * channels + j]) > maxVal) {
                    maxVal = std::abs(tempCov[i * channels + j]);
                    p = i;
                    q = j;
                }
            }
        }

        if (maxVal < 1e-10) break;

        // Compute rotation angle
        double theta = 0.5 * std::atan2(2.0 * tempCov[p * channels + q],
                                         tempCov[q * channels + q] - tempCov[p * channels + p]);
        double c = std::cos(theta);
        double s = std::sin(theta);

        // Apply rotation to covariance matrix
        for (int32_t i = 0; i < channels; ++i) {
            double temp1 = tempCov[i * channels + p];
            double temp2 = tempCov[i * channels + q];
            tempCov[i * channels + p] = c * temp1 - s * temp2;
            tempCov[i * channels + q] = s * temp1 + c * temp2;
        }
        for (int32_t j = 0; j < channels; ++j) {
            double temp1 = tempCov[p * channels + j];
            double temp2 = tempCov[q * channels + j];
            tempCov[p * channels + j] = c * temp1 - s * temp2;
            tempCov[q * channels + j] = s * temp1 + c * temp2;
        }

        // Update eigenvector matrix
        for (int32_t i = 0; i < channels; ++i) {
            double temp1 = transMat[i * channels + p];
            double temp2 = transMat[i * channels + q];
            transMat[i * channels + p] = c * temp1 - s * temp2;
            transMat[i * channels + q] = s * temp1 + c * temp2;
        }
    }

    // Extract eigenvalues (diagonal of transformed covariance)
    for (int32_t i = 0; i < channels; ++i) {
        eigenvalues[i] = tempCov[i * channels + i];
    }

    // Sort eigenvectors by eigenvalue (descending)
    for (int32_t i = 0; i < channels - 1; ++i) {
        for (int32_t j = i + 1; j < channels; ++j) {
            if (eigenvalues[j] > eigenvalues[i]) {
                std::swap(eigenvalues[i], eigenvalues[j]);
                for (int32_t k = 0; k < channels; ++k) {
                    std::swap(transMat[k * channels + i], transMat[k * channels + j]);
                }
            }
        }
    }

    // Transpose to get row-major eigenvectors
    std::vector<double> temp = transMat;
    for (int32_t i = 0; i < channels; ++i) {
        for (int32_t j = 0; j < channels; ++j) {
            transMat[i * channels + j] = temp[j * channels + i];
        }
    }
}

} // namespace Qi::Vision::Color
