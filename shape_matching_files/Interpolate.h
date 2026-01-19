#pragma once

/**
 * @file Interpolate.h
 * @brief Subpixel interpolation functions
 *
 * Provides:
 * - Nearest neighbor interpolation
 * - Bilinear interpolation
 * - Bicubic interpolation
 * - Batch interpolation for performance
 *
 * Used by:
 * - Subpixel edge detection
 * - Geometric transformations
 * - Template matching refinement
 * - Caliper measurement
 *
 * Precision:
 * - Bilinear: <0.01 pixel error
 * - Bicubic: <0.002 pixel error
 */

#include <QiVision/Core/Types.h>
#include <QiVision/Core/Constants.h>

#include <cstdint>
#include <cstddef>
#include <vector>
#include <cmath>

namespace Qi::Vision::Internal {

/**
 * @brief Interpolation methods
 */
enum class InterpolationMethod {
    Nearest,    ///< Nearest neighbor (fast, low quality)
    Bilinear,   ///< Bilinear (good balance)
    Bicubic     ///< Bicubic (high quality, slower)
};

/**
 * @brief Border handling for out-of-bounds access
 */
enum class BorderMode {
    Constant,   ///< Return constant value
    Replicate,  ///< Repeat edge pixels
    Reflect,    ///< Mirror at edge (includes edge)
    Reflect101, ///< Mirror at edge (excludes edge) - default
    Wrap        ///< Wrap around
};

// ============================================================================
// Single Point Interpolation
// ============================================================================

/**
 * @brief Bilinear interpolation at subpixel location
 * @param data Image data (row-major)
 * @param width Image width
 * @param height Image height
 * @param x Subpixel x coordinate
 * @param y Subpixel y coordinate
 * @param borderMode How to handle out-of-bounds
 * @param borderValue Value for Constant border mode
 * @return Interpolated value
 *
 * Precision: <0.01 pixel for smooth gradients
 */
template<typename T>
double InterpolateBilinear(const T* data, int32_t width, int32_t height,
                           double x, double y,
                           BorderMode borderMode = BorderMode::Reflect101,
                           double borderValue = 0.0);

/**
 * @brief Bicubic interpolation at subpixel location
 * @param data Image data (row-major)
 * @param width Image width
 * @param height Image height
 * @param x Subpixel x coordinate
 * @param y Subpixel y coordinate
 * @param borderMode How to handle out-of-bounds
 * @param borderValue Value for Constant border mode
 * @return Interpolated value
 *
 * Uses Catmull-Rom spline (a=-0.5)
 * Precision: <0.002 pixel for smooth gradients
 */
template<typename T>
double InterpolateBicubic(const T* data, int32_t width, int32_t height,
                          double x, double y,
                          BorderMode borderMode = BorderMode::Reflect101,
                          double borderValue = 0.0);

/**
 * @brief Nearest neighbor interpolation
 * @param data Image data (row-major)
 * @param width Image width
 * @param height Image height
 * @param x Subpixel x coordinate
 * @param y Subpixel y coordinate
 * @param borderMode How to handle out-of-bounds
 * @param borderValue Value for Constant border mode
 * @return Value of nearest pixel
 */
template<typename T>
double InterpolateNearest(const T* data, int32_t width, int32_t height,
                          double x, double y,
                          BorderMode borderMode = BorderMode::Reflect101,
                          double borderValue = 0.0);

/**
 * @brief Interpolate using specified method
 */
template<typename T>
double Interpolate(const T* data, int32_t width, int32_t height,
                   double x, double y,
                   InterpolationMethod method = InterpolationMethod::Bilinear,
                   BorderMode borderMode = BorderMode::Reflect101,
                   double borderValue = 0.0);

// ============================================================================
// Stride-aware Interpolation (for images with row padding)
// ============================================================================

/**
 * @brief Bilinear interpolation with explicit stride (row pitch in elements)
 */
template<typename T>
double InterpolateBilinearStrided(const T* data, int32_t width, int32_t height,
                                   size_t stride, double x, double y,
                                   BorderMode borderMode = BorderMode::Reflect101,
                                   double borderValue = 0.0);

/**
 * @brief Bicubic interpolation with explicit stride
 */
template<typename T>
double InterpolateBicubicStrided(const T* data, int32_t width, int32_t height,
                                  size_t stride, double x, double y,
                                  BorderMode borderMode = BorderMode::Reflect101,
                                  double borderValue = 0.0);

/**
 * @brief Nearest neighbor interpolation with explicit stride
 */
template<typename T>
double InterpolateNearestStrided(const T* data, int32_t width, int32_t height,
                                  size_t stride, double x, double y,
                                  BorderMode borderMode = BorderMode::Reflect101,
                                  double borderValue = 0.0);

/**
 * @brief Interpolate using specified method with explicit stride
 */
template<typename T>
double InterpolateStrided(const T* data, int32_t width, int32_t height,
                           size_t stride, double x, double y,
                           InterpolationMethod method = InterpolationMethod::Bilinear,
                           BorderMode borderMode = BorderMode::Reflect101,
                           double borderValue = 0.0);

// ============================================================================
// Gradient Interpolation (for subpixel edge detection)
// ============================================================================

/**
 * @brief Bilinear interpolation with gradient
 * @param data Image data
 * @param width Image width
 * @param height Image height
 * @param x Subpixel x coordinate
 * @param y Subpixel y coordinate
 * @param[out] dx Gradient in x direction
 * @param[out] dy Gradient in y direction
 * @param borderMode Border handling
 * @param borderValue Value for Constant mode
 * @return Interpolated value
 */
template<typename T>
double InterpolateBilinearWithGradient(const T* data, int32_t width, int32_t height,
                                        double x, double y,
                                        double& dx, double& dy,
                                        BorderMode borderMode = BorderMode::Reflect101,
                                        double borderValue = 0.0);

/**
 * @brief Bicubic interpolation with gradient
 * @param data Image data
 * @param width Image width
 * @param height Image height
 * @param x Subpixel x coordinate
 * @param y Subpixel y coordinate
 * @param[out] dx Gradient in x direction
 * @param[out] dy Gradient in y direction
 * @param borderMode Border handling
 * @param borderValue Value for Constant mode
 * @return Interpolated value
 */
template<typename T>
double InterpolateBicubicWithGradient(const T* data, int32_t width, int32_t height,
                                       double x, double y,
                                       double& dx, double& dy,
                                       BorderMode borderMode = BorderMode::Reflect101,
                                       double borderValue = 0.0);

// ============================================================================
// Batch Interpolation
// ============================================================================

/**
 * @brief Interpolate at multiple points (batch)
 * @param data Image data
 * @param width Image width
 * @param height Image height
 * @param points Array of (x, y) points
 * @param numPoints Number of points
 * @param[out] results Output array (must be pre-allocated)
 * @param method Interpolation method
 * @param borderMode Border handling
 * @param borderValue Value for Constant mode
 */
template<typename T>
void InterpolateBatch(const T* data, int32_t width, int32_t height,
                      const Point2d* points, size_t numPoints,
                      double* results,
                      InterpolationMethod method = InterpolationMethod::Bilinear,
                      BorderMode borderMode = BorderMode::Reflect101,
                      double borderValue = 0.0);

/**
 * @brief Interpolate along a line (for profiling)
 * @param data Image data
 * @param width Image width
 * @param height Image height
 * @param x0, y0 Start point
 * @param x1, y1 End point
 * @param numSamples Number of samples along line
 * @param[out] results Output array (must be pre-allocated)
 * @param method Interpolation method
 * @param borderMode Border handling
 */
template<typename T>
void InterpolateAlongLine(const T* data, int32_t width, int32_t height,
                          double x0, double y0, double x1, double y1,
                          size_t numSamples, double* results,
                          InterpolationMethod method = InterpolationMethod::Bilinear,
                          BorderMode borderMode = BorderMode::Reflect101);

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Clamp coordinate to valid range
 */
inline int32_t ClampCoord(int32_t coord, int32_t size) {
    if (coord < 0) return 0;
    if (coord >= size) return size - 1;
    return coord;
}

/**
 * @brief Handle border coordinate based on mode
 */
int32_t HandleBorder(int32_t coord, int32_t size, BorderMode mode);

/**
 * @brief Cubic interpolation weight (Catmull-Rom, a=-0.5)
 */
inline double CubicWeight(double t) {
    double at = std::abs(t);
    if (at <= 1.0) {
        return (1.5 * at - 2.5) * at * at + 1.0;
    } else if (at < 2.0) {
        return ((-0.5 * at + 2.5) * at - 4.0) * at + 2.0;
    }
    return 0.0;
}

/**
 * @brief Derivative of cubic weight
 */
inline double CubicWeightDerivative(double t) {
    double at = std::abs(t);
    double sign = (t >= 0) ? 1.0 : -1.0;
    if (at <= 1.0) {
        return sign * (4.5 * at * at - 5.0 * at);
    } else if (at < 2.0) {
        return sign * (-1.5 * at * at + 5.0 * at - 4.0);
    }
    return 0.0;
}

// ============================================================================
// Template Implementations
// ============================================================================

template<typename T>
double InterpolateNearest(const T* data, int32_t width, int32_t height,
                          double x, double y,
                          BorderMode borderMode, double borderValue) {
    int32_t ix = static_cast<int32_t>(std::round(x));
    int32_t iy = static_cast<int32_t>(std::round(y));

    // Check bounds
    if (ix < 0 || ix >= width || iy < 0 || iy >= height) {
        if (borderMode == BorderMode::Constant) {
            return borderValue;
        }
        ix = HandleBorder(ix, width, borderMode);
        iy = HandleBorder(iy, height, borderMode);
    }

    return static_cast<double>(data[iy * width + ix]);
}

template<typename T>
double InterpolateBilinear(const T* data, int32_t width, int32_t height,
                           double x, double y,
                           BorderMode borderMode, double borderValue) {
    // Integer and fractional parts
    int32_t x0 = static_cast<int32_t>(std::floor(x));
    int32_t y0 = static_cast<int32_t>(std::floor(y));
    double fx = x - x0;
    double fy = y - y0;

    int32_t x1 = x0 + 1;
    int32_t y1 = y0 + 1;

    // Get pixel values with border handling
    auto getPixel = [&](int32_t px, int32_t py) -> double {
        if (px < 0 || px >= width || py < 0 || py >= height) {
            if (borderMode == BorderMode::Constant) {
                return borderValue;
            }
            px = HandleBorder(px, width, borderMode);
            py = HandleBorder(py, height, borderMode);
        }
        return static_cast<double>(data[py * width + px]);
    };

    double v00 = getPixel(x0, y0);
    double v10 = getPixel(x1, y0);
    double v01 = getPixel(x0, y1);
    double v11 = getPixel(x1, y1);

    // Bilinear interpolation
    double v0 = v00 + fx * (v10 - v00);
    double v1 = v01 + fx * (v11 - v01);
    return v0 + fy * (v1 - v0);
}

template<typename T>
double InterpolateBicubic(const T* data, int32_t width, int32_t height,
                          double x, double y,
                          BorderMode borderMode, double borderValue) {
    int32_t x0 = static_cast<int32_t>(std::floor(x));
    int32_t y0 = static_cast<int32_t>(std::floor(y));
    double fx = x - x0;
    double fy = y - y0;

    auto getPixel = [&](int32_t px, int32_t py) -> double {
        if (px < 0 || px >= width || py < 0 || py >= height) {
            if (borderMode == BorderMode::Constant) {
                return borderValue;
            }
            px = HandleBorder(px, width, borderMode);
            py = HandleBorder(py, height, borderMode);
        }
        return static_cast<double>(data[py * width + px]);
    };

    // Compute weights
    double wx[4], wy[4];
    for (int i = 0; i < 4; ++i) {
        wx[i] = CubicWeight(fx - (i - 1));
        wy[i] = CubicWeight(fy - (i - 1));
    }

    // Sample 4x4 neighborhood
    double result = 0.0;
    for (int j = 0; j < 4; ++j) {
        double rowSum = 0.0;
        for (int i = 0; i < 4; ++i) {
            rowSum += wx[i] * getPixel(x0 + i - 1, y0 + j - 1);
        }
        result += wy[j] * rowSum;
    }

    return result;
}

template<typename T>
double Interpolate(const T* data, int32_t width, int32_t height,
                   double x, double y,
                   InterpolationMethod method,
                   BorderMode borderMode, double borderValue) {
    switch (method) {
        case InterpolationMethod::Nearest:
            return InterpolateNearest(data, width, height, x, y, borderMode, borderValue);
        case InterpolationMethod::Bilinear:
            return InterpolateBilinear(data, width, height, x, y, borderMode, borderValue);
        case InterpolationMethod::Bicubic:
            return InterpolateBicubic(data, width, height, x, y, borderMode, borderValue);
        default:
            return InterpolateBilinear(data, width, height, x, y, borderMode, borderValue);
    }
}

template<typename T>
double InterpolateBilinearWithGradient(const T* data, int32_t width, int32_t height,
                                        double x, double y,
                                        double& dx, double& dy,
                                        BorderMode borderMode, double borderValue) {
    int32_t x0 = static_cast<int32_t>(std::floor(x));
    int32_t y0 = static_cast<int32_t>(std::floor(y));
    double fx = x - x0;
    double fy = y - y0;

    int32_t x1 = x0 + 1;
    int32_t y1 = y0 + 1;

    auto getPixel = [&](int32_t px, int32_t py) -> double {
        if (px < 0 || px >= width || py < 0 || py >= height) {
            if (borderMode == BorderMode::Constant) {
                return borderValue;
            }
            px = HandleBorder(px, width, borderMode);
            py = HandleBorder(py, height, borderMode);
        }
        return static_cast<double>(data[py * width + px]);
    };

    double v00 = getPixel(x0, y0);
    double v10 = getPixel(x1, y0);
    double v01 = getPixel(x0, y1);
    double v11 = getPixel(x1, y1);

    // Value
    double v0 = v00 + fx * (v10 - v00);
    double v1 = v01 + fx * (v11 - v01);
    double value = v0 + fy * (v1 - v0);

    // Gradients
    // dx = (1-fy) * (v10 - v00) + fy * (v11 - v01)
    // dy = (1-fx) * (v01 - v00) + fx * (v11 - v10)
    dx = (1.0 - fy) * (v10 - v00) + fy * (v11 - v01);
    dy = (1.0 - fx) * (v01 - v00) + fx * (v11 - v10);

    return value;
}

template<typename T>
double InterpolateBicubicWithGradient(const T* data, int32_t width, int32_t height,
                                       double x, double y,
                                       double& dx, double& dy,
                                       BorderMode borderMode, double borderValue) {
    int32_t x0 = static_cast<int32_t>(std::floor(x));
    int32_t y0 = static_cast<int32_t>(std::floor(y));
    double fx = x - x0;
    double fy = y - y0;

    auto getPixel = [&](int32_t px, int32_t py) -> double {
        if (px < 0 || px >= width || py < 0 || py >= height) {
            if (borderMode == BorderMode::Constant) {
                return borderValue;
            }
            px = HandleBorder(px, width, borderMode);
            py = HandleBorder(py, height, borderMode);
        }
        return static_cast<double>(data[py * width + px]);
    };

    // Compute weights and derivatives
    double wx[4], wy[4], dwx[4], dwy[4];
    for (int i = 0; i < 4; ++i) {
        double tx = fx - (i - 1);
        double ty = fy - (i - 1);
        wx[i] = CubicWeight(tx);
        wy[i] = CubicWeight(ty);
        dwx[i] = CubicWeightDerivative(tx);
        dwy[i] = CubicWeightDerivative(ty);
    }

    // Sample 4x4 neighborhood
    double pixels[4][4];
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 4; ++i) {
            pixels[j][i] = getPixel(x0 + i - 1, y0 + j - 1);
        }
    }

    // Compute value and gradients
    double value = 0.0;
    dx = 0.0;
    dy = 0.0;

    for (int j = 0; j < 4; ++j) {
        double rowSum = 0.0;
        double rowSumDx = 0.0;
        for (int i = 0; i < 4; ++i) {
            rowSum += wx[i] * pixels[j][i];
            rowSumDx += dwx[i] * pixels[j][i];
        }
        value += wy[j] * rowSum;
        dx += wy[j] * rowSumDx;
        dy += dwy[j] * rowSum;
    }

    return value;
}

template<typename T>
void InterpolateBatch(const T* data, int32_t width, int32_t height,
                      const Point2d* points, size_t numPoints,
                      double* results,
                      InterpolationMethod method,
                      BorderMode borderMode, double borderValue) {
    for (size_t i = 0; i < numPoints; ++i) {
        results[i] = Interpolate(data, width, height, points[i].x, points[i].y,
                                  method, borderMode, borderValue);
    }
}

template<typename T>
void InterpolateAlongLine(const T* data, int32_t width, int32_t height,
                          double x0, double y0, double x1, double y1,
                          size_t numSamples, double* results,
                          InterpolationMethod method,
                          BorderMode borderMode) {
    if (numSamples == 0) return;
    if (numSamples == 1) {
        results[0] = Interpolate(data, width, height, x0, y0, method, borderMode, 0.0);
        return;
    }

    double dx = (x1 - x0) / static_cast<double>(numSamples - 1);
    double dy = (y1 - y0) / static_cast<double>(numSamples - 1);

    for (size_t i = 0; i < numSamples; ++i) {
        double x = x0 + i * dx;
        double y = y0 + i * dy;
        results[i] = Interpolate(data, width, height, x, y, method, borderMode, 0.0);
    }
}

// ============================================================================
// Stride-aware Template Implementations
// ============================================================================

template<typename T>
double InterpolateNearestStrided(const T* data, int32_t width, int32_t height,
                                  size_t stride, double x, double y,
                                  BorderMode borderMode, double borderValue) {
    int32_t ix = static_cast<int32_t>(std::round(x));
    int32_t iy = static_cast<int32_t>(std::round(y));

    if (ix < 0 || ix >= width || iy < 0 || iy >= height) {
        if (borderMode == BorderMode::Constant) {
            return borderValue;
        }
        ix = HandleBorder(ix, width, borderMode);
        iy = HandleBorder(iy, height, borderMode);
    }

    return static_cast<double>(data[iy * stride + ix]);
}

template<typename T>
double InterpolateBilinearStrided(const T* data, int32_t width, int32_t height,
                                   size_t stride, double x, double y,
                                   BorderMode borderMode, double borderValue) {
    int32_t x0 = static_cast<int32_t>(std::floor(x));
    int32_t y0 = static_cast<int32_t>(std::floor(y));
    double fx = x - x0;
    double fy = y - y0;

    int32_t x1 = x0 + 1;
    int32_t y1 = y0 + 1;

    auto getPixel = [&](int32_t px, int32_t py) -> double {
        if (px < 0 || px >= width || py < 0 || py >= height) {
            if (borderMode == BorderMode::Constant) {
                return borderValue;
            }
            px = HandleBorder(px, width, borderMode);
            py = HandleBorder(py, height, borderMode);
        }
        return static_cast<double>(data[py * stride + px]);
    };

    double v00 = getPixel(x0, y0);
    double v10 = getPixel(x1, y0);
    double v01 = getPixel(x0, y1);
    double v11 = getPixel(x1, y1);

    double v0 = v00 + fx * (v10 - v00);
    double v1 = v01 + fx * (v11 - v01);
    return v0 + fy * (v1 - v0);
}

template<typename T>
double InterpolateBicubicStrided(const T* data, int32_t width, int32_t height,
                                  size_t stride, double x, double y,
                                  BorderMode borderMode, double borderValue) {
    int32_t x0 = static_cast<int32_t>(std::floor(x));
    int32_t y0 = static_cast<int32_t>(std::floor(y));
    double fx = x - x0;
    double fy = y - y0;

    auto getPixel = [&](int32_t px, int32_t py) -> double {
        if (px < 0 || px >= width || py < 0 || py >= height) {
            if (borderMode == BorderMode::Constant) {
                return borderValue;
            }
            px = HandleBorder(px, width, borderMode);
            py = HandleBorder(py, height, borderMode);
        }
        return static_cast<double>(data[py * stride + px]);
    };

    double wx[4], wy[4];
    for (int i = 0; i < 4; ++i) {
        wx[i] = CubicWeight(fx - (i - 1));
        wy[i] = CubicWeight(fy - (i - 1));
    }

    double result = 0.0;
    for (int j = 0; j < 4; ++j) {
        double rowSum = 0.0;
        for (int i = 0; i < 4; ++i) {
            rowSum += wx[i] * getPixel(x0 + i - 1, y0 + j - 1);
        }
        result += wy[j] * rowSum;
    }

    return result;
}

template<typename T>
double InterpolateStrided(const T* data, int32_t width, int32_t height,
                           size_t stride, double x, double y,
                           InterpolationMethod method,
                           BorderMode borderMode, double borderValue) {
    switch (method) {
        case InterpolationMethod::Nearest:
            return InterpolateNearestStrided(data, width, height, stride, x, y, borderMode, borderValue);
        case InterpolationMethod::Bilinear:
            return InterpolateBilinearStrided(data, width, height, stride, x, y, borderMode, borderValue);
        case InterpolationMethod::Bicubic:
            return InterpolateBicubicStrided(data, width, height, stride, x, y, borderMode, borderValue);
        default:
            return InterpolateBilinearStrided(data, width, height, stride, x, y, borderMode, borderValue);
    }
}

} // namespace Qi::Vision::Internal
