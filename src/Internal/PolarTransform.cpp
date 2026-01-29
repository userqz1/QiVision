#include <QiVision/Internal/PolarTransform.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace Qi::Vision::Internal {

// =============================================================================
// Constants
// =============================================================================

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;
constexpr double EPSILON = 1e-10;

// =============================================================================
// Helper Functions
// =============================================================================

namespace {

// Clamp coordinate to valid range
inline int32_t ClampCoord(int32_t val, int32_t maxVal) {
    return std::max(0, std::min(val, maxVal - 1));
}

// Get pixel with border handling
template<typename T>
T GetPixelWithBorder(const T* data, int32_t width, int32_t height,
                     int32_t x, int32_t y, BorderMode mode, T borderValue) {
    if (x >= 0 && x < width && y >= 0 && y < height) {
        return data[y * width + x];
    }

    switch (mode) {
        case BorderMode::Constant:
            return borderValue;
        case BorderMode::Replicate:
            x = ClampCoord(x, width);
            y = ClampCoord(y, height);
            return data[y * width + x];
        case BorderMode::Reflect:
            if (x < 0) x = -x;
            if (x >= width) x = 2 * width - x - 2;
            if (y < 0) y = -y;
            if (y >= height) y = 2 * height - y - 2;
            x = ClampCoord(x, width);
            y = ClampCoord(y, height);
            return data[y * width + x];
        case BorderMode::Reflect101:
            if (x < 0) x = -x - 1;
            if (x >= width) x = 2 * width - x - 1;
            if (y < 0) y = -y - 1;
            if (y >= height) y = 2 * height - y - 1;
            x = ClampCoord(x, width);
            y = ClampCoord(y, height);
            return data[y * width + x];
        case BorderMode::Wrap:
            x = ((x % width) + width) % width;
            y = ((y % height) + height) % height;
            return data[y * width + x];
        default:
            return borderValue;
    }
}

// Bilinear interpolation helper
template<typename T>
double BilinearSample(const T* data, int32_t width, int32_t height,
                      double x, double y, BorderMode mode, double borderValue) {
    int32_t x0 = static_cast<int32_t>(std::floor(x));
    int32_t y0 = static_cast<int32_t>(std::floor(y));
    int32_t x1 = x0 + 1;
    int32_t y1 = y0 + 1;

    double fx = x - x0;
    double fy = y - y0;

    T bv = static_cast<T>(borderValue);
    double v00 = GetPixelWithBorder(data, width, height, x0, y0, mode, bv);
    double v10 = GetPixelWithBorder(data, width, height, x1, y0, mode, bv);
    double v01 = GetPixelWithBorder(data, width, height, x0, y1, mode, bv);
    double v11 = GetPixelWithBorder(data, width, height, x1, y1, mode, bv);

    return v00 * (1 - fx) * (1 - fy) +
           v10 * fx * (1 - fy) +
           v01 * (1 - fx) * fy +
           v11 * fx * fy;
}

// Bicubic kernel (Catmull-Rom, a = -0.5)
inline double CubicKernel(double x) {
    x = std::abs(x);
    if (x < 1.0) {
        return (1.5 * x - 2.5) * x * x + 1.0;
    } else if (x < 2.0) {
        return ((-0.5 * x + 2.5) * x - 4.0) * x + 2.0;
    }
    return 0.0;
}

// Bicubic interpolation helper
template<typename T>
double BicubicSample(const T* data, int32_t width, int32_t height,
                     double x, double y, BorderMode mode, double borderValue) {
    int32_t x0 = static_cast<int32_t>(std::floor(x)) - 1;
    int32_t y0 = static_cast<int32_t>(std::floor(y)) - 1;

    double fx = x - std::floor(x);
    double fy = y - std::floor(y);

    T bv = static_cast<T>(borderValue);
    double result = 0.0;

    for (int j = 0; j < 4; ++j) {
        double wy = CubicKernel(fy - (j - 1));
        for (int i = 0; i < 4; ++i) {
            double wx = CubicKernel(fx - (i - 1));
            double v = GetPixelWithBorder(data, width, height, x0 + i, y0 + j, mode, bv);
            result += v * wx * wy;
        }
    }

    return result;
}

// Nearest neighbor interpolation helper
template<typename T>
double NearestSample(const T* data, int32_t width, int32_t height,
                     double x, double y, BorderMode mode, double borderValue) {
    int32_t xi = static_cast<int32_t>(std::round(x));
    int32_t yi = static_cast<int32_t>(std::round(y));
    T bv = static_cast<T>(borderValue);
    return GetPixelWithBorder(data, width, height, xi, yi, mode, bv);
}

// Sample pixel using specified interpolation method
template<typename T>
double SamplePixel(const T* data, int32_t width, int32_t height,
                   double x, double y,
                   InterpolationMethod method,
                   BorderMode borderMode, double borderValue) {
    switch (method) {
        case InterpolationMethod::Nearest:
            return NearestSample(data, width, height, x, y, borderMode, borderValue);
        case InterpolationMethod::Bicubic:
            return BicubicSample(data, width, height, x, y, borderMode, borderValue);
        case InterpolationMethod::Bilinear:
        default:
            return BilinearSample(data, width, height, x, y, borderMode, borderValue);
    }
}

// Get pixel with stride support
template<typename T>
T GetPixelWithStride(const T* data, int32_t width, int32_t height, int32_t stride,
                     int32_t x, int32_t y, BorderMode mode, T borderValue) {
    if (x >= 0 && x < width && y >= 0 && y < height) {
        return data[y * stride + x];
    }

    switch (mode) {
        case BorderMode::Constant:
            return borderValue;
        case BorderMode::Replicate:
            x = ClampCoord(x, width);
            y = ClampCoord(y, height);
            return data[y * stride + x];
        case BorderMode::Reflect:
            if (x < 0) x = -x;
            if (x >= width) x = 2 * width - x - 2;
            if (y < 0) y = -y;
            if (y >= height) y = 2 * height - y - 2;
            x = ClampCoord(x, width);
            y = ClampCoord(y, height);
            return data[y * stride + x];
        case BorderMode::Reflect101:
            if (x < 0) x = -x - 1;
            if (x >= width) x = 2 * width - x - 1;
            if (y < 0) y = -y - 1;
            if (y >= height) y = 2 * height - y - 1;
            x = ClampCoord(x, width);
            y = ClampCoord(y, height);
            return data[y * stride + x];
        case BorderMode::Wrap:
            x = ((x % width) + width) % width;
            y = ((y % height) + height) % height;
            return data[y * stride + x];
        default:
            return borderValue;
    }
}

// Bilinear interpolation with stride
template<typename T>
double BilinearSampleWithStride(const T* data, int32_t width, int32_t height, int32_t stride,
                                double x, double y, BorderMode mode, double borderValue) {
    int32_t x0 = static_cast<int32_t>(std::floor(x));
    int32_t y0 = static_cast<int32_t>(std::floor(y));
    int32_t x1 = x0 + 1;
    int32_t y1 = y0 + 1;

    double fx = x - x0;
    double fy = y - y0;

    T bv = static_cast<T>(borderValue);
    double v00 = GetPixelWithStride(data, width, height, stride, x0, y0, mode, bv);
    double v10 = GetPixelWithStride(data, width, height, stride, x1, y0, mode, bv);
    double v01 = GetPixelWithStride(data, width, height, stride, x0, y1, mode, bv);
    double v11 = GetPixelWithStride(data, width, height, stride, x1, y1, mode, bv);

    return v00 * (1 - fx) * (1 - fy) +
           v10 * fx * (1 - fy) +
           v01 * (1 - fx) * fy +
           v11 * fx * fy;
}

// Sample pixel with stride support
template<typename T>
double SamplePixelWithStride(const T* data, int32_t width, int32_t height, int32_t stride,
                             double x, double y,
                             InterpolationMethod method,
                             BorderMode borderMode, double borderValue) {
    // For simplicity, use bilinear for all methods (can extend later)
    return BilinearSampleWithStride(data, width, height, stride, x, y, borderMode, borderValue);
}

// Generic polar warp function: Cartesian to Polar
// For each (col, row) in dst:
//   col corresponds to angle [0, 2*pi)
//   row corresponds to radius [0, maxRadius]
template<typename T>
void WarpCartesianToPolar(const T* src, int32_t srcWidth, int32_t srcHeight, int32_t srcStride,
                          T* dst, int32_t dstWidth, int32_t dstHeight, int32_t dstStride,
                          const Point2d& center, double maxRadius,
                          PolarMode mode,
                          InterpolationMethod method,
                          BorderMode borderMode, double borderValue) {
    // Precompute log scale factor for SemiLog mode
    // M = maxRadius / log(maxRadius + 1) ensures full range mapping
    double logM = (mode == PolarMode::SemiLog && maxRadius > EPSILON)
                  ? maxRadius / std::log(maxRadius + 1.0)
                  : 1.0;

    // srcStride and dstStride are in elements (not bytes)
    for (int32_t row = 0; row < dstHeight; ++row) {
        // Map row to radius
        double rho = maxRadius * static_cast<double>(row) / static_cast<double>(dstHeight);
        double r;
        if (mode == PolarMode::Linear) {
            r = rho;
        } else {
            // SemiLog: rho = M * log(r + 1) => r = exp(rho / M) - 1
            r = LogPolarToLinear(rho, logM);
        }

        T* dstRow = dst + row * dstStride;
        for (int32_t col = 0; col < dstWidth; ++col) {
            // Map col to angle
            double theta = TWO_PI * static_cast<double>(col) / static_cast<double>(dstWidth);

            // Compute source position
            double srcX = center.x + r * std::cos(theta);
            double srcY = center.y + r * std::sin(theta);

            // Sample and store (use srcStride for sampling)
            double value = SamplePixelWithStride(src, srcWidth, srcHeight, srcStride,
                                                 srcX, srcY, method, borderMode, borderValue);

            // Clamp for integer types
            if constexpr (std::is_integral_v<T>) {
                value = std::round(value);
                value = std::max(static_cast<double>(std::numeric_limits<T>::min()),
                                std::min(static_cast<double>(std::numeric_limits<T>::max()), value));
            }
            dstRow[col] = static_cast<T>(value);
        }
    }
}

// Generic polar warp function: Polar to Cartesian (inverse)
// src is in polar coordinates, dst is in Cartesian
template<typename T>
void WarpPolarToCartesian(const T* src, int32_t srcWidth, int32_t srcHeight, int32_t srcStride,
                          T* dst, int32_t dstWidth, int32_t dstHeight, int32_t dstStride,
                          const Point2d& center, double maxRadius,
                          PolarMode mode,
                          InterpolationMethod method,
                          BorderMode borderMode, double borderValue) {
    // Precompute log scale factor for SemiLog mode
    double logM = (mode == PolarMode::SemiLog && maxRadius > EPSILON)
                  ? maxRadius / std::log(maxRadius + 1.0)
                  : 1.0;

    for (int32_t y = 0; y < dstHeight; ++y) {
        T* dstRow = dst + y * dstStride;
        for (int32_t x = 0; x < dstWidth; ++x) {
            // Convert (x, y) to polar coordinates relative to center
            double dx = static_cast<double>(x) - center.x;
            double dy = static_cast<double>(y) - center.y;
            double r = std::sqrt(dx * dx + dy * dy);
            double theta = std::atan2(dy, dx);

            // Normalize theta to [0, 2*pi)
            if (theta < 0) {
                theta += TWO_PI;
            }

            // Map r to rho (row in polar image)
            double rho;
            if (mode == PolarMode::Linear) {
                rho = r;
            } else {
                // SemiLog: rho = M * log(r + 1)
                rho = LinearToLogPolar(r, logM);
            }

            // Compute source position in polar image
            // srcX = angle mapped to [0, srcWidth)
            // srcY = radius mapped to [0, srcHeight)
            double polarX = theta / TWO_PI * static_cast<double>(srcWidth);
            double polarY = rho / maxRadius * static_cast<double>(srcHeight);

            // Sample and store (use srcStride for sampling)
            double value = SamplePixelWithStride(src, srcWidth, srcHeight, srcStride,
                                                 polarX, polarY, method, borderMode, borderValue);

            // Clamp for integer types
            if constexpr (std::is_integral_v<T>) {
                value = std::round(value);
                value = std::max(static_cast<double>(std::numeric_limits<T>::min()),
                                std::min(static_cast<double>(std::numeric_limits<T>::max()), value));
            }
            dstRow[x] = static_cast<T>(value);
        }
    }
}

} // anonymous namespace

// =============================================================================
// Public Functions
// =============================================================================

Point2d CartesianToPolar(const Point2d& pt, const Point2d& center) {
    double dx = pt.x - center.x;
    double dy = pt.y - center.y;
    double r = std::sqrt(dx * dx + dy * dy);
    double theta = std::atan2(dy, dx);

    // Normalize theta to [0, 2*pi)
    if (theta < 0) {
        theta += TWO_PI;
    }

    return Point2d(theta, r);  // x = angle, y = radius
}

Point2d PolarToCartesian(double r, double theta, const Point2d& center) {
    return Point2d(
        center.x + r * std::cos(theta),
        center.y + r * std::sin(theta)
    );
}

QImage WarpPolar(
    const QImage& src,
    const Point2d& center,
    double maxRadius,
    int32_t dstWidth,
    int32_t dstHeight,
    PolarMode mode,
    bool inverse,
    InterpolationMethod method,
    BorderMode borderMode,
    double borderValue) {

    if (src.Empty()) {
        return QImage();
    }

    // Validate maxRadius
    if (maxRadius <= 0) {
        return QImage();
    }

    int32_t srcWidth = src.Width();
    int32_t srcHeight = src.Height();

    // Calculate output size if not specified
    if (!inverse) {
        // Cartesian to Polar: output is (angle, radius)
        // Default: width = 2*pi * maxRadius (angle resolution)
        //          height = maxRadius (radial resolution)
        if (dstWidth <= 0) {
            dstWidth = static_cast<int32_t>(std::ceil(TWO_PI * maxRadius));
        }
        if (dstHeight <= 0) {
            dstHeight = static_cast<int32_t>(std::ceil(maxRadius));
        }
    } else {
        // Polar to Cartesian: output is Cartesian image
        // Default: 2 * maxRadius x 2 * maxRadius centered at (maxRadius, maxRadius)
        if (dstWidth <= 0) {
            dstWidth = static_cast<int32_t>(std::ceil(2.0 * maxRadius));
        }
        if (dstHeight <= 0) {
            dstHeight = static_cast<int32_t>(std::ceil(2.0 * maxRadius));
        }
    }

    // Get channel type
    ChannelType chType = src.GetChannelType();
    int32_t channels = src.Channels();

    // Create output image
    QImage dst(dstWidth, dstHeight, src.Type(), chType);

    // Get strides (in bytes) and convert to element counts
    int32_t srcStrideBytes = static_cast<int32_t>(src.Stride());
    int32_t dstStrideBytes = static_cast<int32_t>(dst.Stride());

    // For grayscale images, process directly with stride
    // For multi-channel, we need to handle interleaved format
    if (channels == 1) {
        switch (src.Type()) {
            case PixelType::UInt8: {
                const uint8_t* srcData = static_cast<const uint8_t*>(src.Data());
                uint8_t* dstData = static_cast<uint8_t*>(dst.Data());
                int32_t srcStride = srcStrideBytes;  // 1 byte per pixel
                int32_t dstStride = dstStrideBytes;

                if (!inverse) {
                    WarpCartesianToPolar(srcData, srcWidth, srcHeight, srcStride,
                                         dstData, dstWidth, dstHeight, dstStride,
                                         center, maxRadius, mode,
                                         method, borderMode, borderValue);
                } else {
                    WarpPolarToCartesian(srcData, srcWidth, srcHeight, srcStride,
                                         dstData, dstWidth, dstHeight, dstStride,
                                         center, maxRadius, mode,
                                         method, borderMode, borderValue);
                }
                break;
            }
            case PixelType::UInt16: {
                const uint16_t* srcData = static_cast<const uint16_t*>(src.Data());
                uint16_t* dstData = static_cast<uint16_t*>(dst.Data());
                int32_t srcStride = srcStrideBytes / 2;  // 2 bytes per pixel
                int32_t dstStride = dstStrideBytes / 2;

                if (!inverse) {
                    WarpCartesianToPolar(srcData, srcWidth, srcHeight, srcStride,
                                         dstData, dstWidth, dstHeight, dstStride,
                                         center, maxRadius, mode,
                                         method, borderMode, borderValue);
                } else {
                    WarpPolarToCartesian(srcData, srcWidth, srcHeight, srcStride,
                                         dstData, dstWidth, dstHeight, dstStride,
                                         center, maxRadius, mode,
                                         method, borderMode, borderValue);
                }
                break;
            }
            case PixelType::Int16: {
                const int16_t* srcData = static_cast<const int16_t*>(src.Data());
                int16_t* dstData = static_cast<int16_t*>(dst.Data());
                int32_t srcStride = srcStrideBytes / 2;
                int32_t dstStride = dstStrideBytes / 2;

                if (!inverse) {
                    WarpCartesianToPolar(srcData, srcWidth, srcHeight, srcStride,
                                         dstData, dstWidth, dstHeight, dstStride,
                                         center, maxRadius, mode,
                                         method, borderMode, borderValue);
                } else {
                    WarpPolarToCartesian(srcData, srcWidth, srcHeight, srcStride,
                                         dstData, dstWidth, dstHeight, dstStride,
                                         center, maxRadius, mode,
                                         method, borderMode, borderValue);
                }
                break;
            }
            case PixelType::Float32: {
                const float* srcData = static_cast<const float*>(src.Data());
                float* dstData = static_cast<float*>(dst.Data());
                int32_t srcStride = srcStrideBytes / 4;  // 4 bytes per pixel
                int32_t dstStride = dstStrideBytes / 4;

                if (!inverse) {
                    WarpCartesianToPolar(srcData, srcWidth, srcHeight, srcStride,
                                         dstData, dstWidth, dstHeight, dstStride,
                                         center, maxRadius, mode,
                                         method, borderMode, borderValue);
                } else {
                    WarpPolarToCartesian(srcData, srcWidth, srcHeight, srcStride,
                                         dstData, dstWidth, dstHeight, dstStride,
                                         center, maxRadius, mode,
                                         method, borderMode, borderValue);
                }
                break;
            }
        }
    }

    return dst;
}

} // namespace Qi::Vision::Internal
