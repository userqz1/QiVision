#pragma once

/**
 * @file Gradient.h
 * @brief Image gradient computation
 *
 * Provides:
 * - Sobel gradient (3x3, 5x5, 7x7)
 * - Scharr gradient (3x3, optimized accuracy)
 * - Prewitt gradient (3x3)
 * - Gradient magnitude and direction
 * - Separable implementation for efficiency
 *
 * Used by:
 * - Edge detection (Canny, Steger)
 * - Shape matching (gradient-based)
 * - Feature detection
 * - Texture analysis
 *
 * Border handling: Reflect101 (default)
 */

#include <QiVision/Core/Types.h>
#include <QiVision/Internal/Interpolate.h>

#include <cstdint>
#include <cstddef>
#include <vector>
#include <cmath>

namespace Qi::Vision::Internal {

/**
 * @brief Gradient operator types
 */
enum class GradientOperator {
    Sobel3x3,     ///< Sobel 3x3 (standard)
    Sobel5x5,     ///< Sobel 5x5 (smoother)
    Sobel7x7,     ///< Sobel 7x7 (very smooth)
    Scharr,       ///< Scharr 3x3 (higher accuracy)
    Prewitt,      ///< Prewitt 3x3 (simple)
    Central       ///< Central difference (1D)
};

// ============================================================================
// Gradient Kernels (for reference and explicit use)
// ============================================================================

/**
 * @brief Get Sobel derivative kernel (separable)
 * @param size Kernel size (3, 5, or 7)
 * @return Derivative kernel [d]
 */
std::vector<double> SobelDerivativeKernel(int32_t size = 3);

/**
 * @brief Get Sobel smoothing kernel (separable)
 * @param size Kernel size (3, 5, or 7)
 * @return Smoothing kernel [s]
 */
std::vector<double> SobelSmoothingKernel(int32_t size = 3);

/**
 * @brief Get Scharr derivative kernel (3x3 only)
 * @return Derivative kernel [3, 10, 3] normalized
 */
std::vector<double> ScharrDerivativeKernel();

/**
 * @brief Get Scharr smoothing kernel (3x3 only)
 * @return Smoothing kernel [-1, 0, 1]
 */
std::vector<double> ScharrSmoothingKernel();

/**
 * @brief Get Prewitt derivative kernel (3x3 only)
 * @return Derivative kernel [-1, 0, 1]
 */
std::vector<double> PrewittDerivativeKernel();

/**
 * @brief Get Prewitt smoothing kernel (3x3 only)
 * @return Smoothing kernel [1, 1, 1]
 */
std::vector<double> PrewittSmoothingKernel();

// ============================================================================
// Single-Direction Gradient (Gx or Gy)
// ============================================================================

/**
 * @brief Compute horizontal gradient (dI/dx)
 * @param src Source image
 * @param dst Destination gradient image (must be pre-allocated)
 * @param width Image width
 * @param height Image height
 * @param op Gradient operator
 * @param borderMode Border handling
 */
template<typename SrcT, typename DstT = float>
void GradientX(const SrcT* src, DstT* dst,
               int32_t width, int32_t height,
               GradientOperator op = GradientOperator::Sobel3x3,
               BorderMode borderMode = BorderMode::Reflect101);

/**
 * @brief Compute vertical gradient (dI/dy)
 * @param src Source image
 * @param dst Destination gradient image (must be pre-allocated)
 * @param width Image width
 * @param height Image height
 * @param op Gradient operator
 * @param borderMode Border handling
 */
template<typename SrcT, typename DstT = float>
void GradientY(const SrcT* src, DstT* dst,
               int32_t width, int32_t height,
               GradientOperator op = GradientOperator::Sobel3x3,
               BorderMode borderMode = BorderMode::Reflect101);

// ============================================================================
// Gradient Magnitude and Direction
// ============================================================================

/**
 * @brief Compute gradient magnitude from Gx and Gy
 * @param gx Horizontal gradient
 * @param gy Vertical gradient
 * @param mag Output magnitude (must be pre-allocated)
 * @param size Number of pixels (width * height)
 * @param normalize If true, scale to [0, 255] for uint8 output
 */
template<typename T>
void GradientMagnitude(const T* gx, const T* gy, T* mag, size_t size,
                       bool normalize = false);

/**
 * @brief Compute gradient magnitude (L2 norm: sqrt(gx^2 + gy^2))
 */
template<typename T>
inline T MagnitudeL2(T gx, T gy) {
    return static_cast<T>(std::sqrt(static_cast<double>(gx) * gx +
                                     static_cast<double>(gy) * gy));
}

/**
 * @brief Compute gradient magnitude (L1 norm: |gx| + |gy|)
 */
template<typename T>
inline T MagnitudeL1(T gx, T gy) {
    return static_cast<T>(std::abs(static_cast<double>(gx)) +
                          std::abs(static_cast<double>(gy)));
}

/**
 * @brief Compute gradient direction from Gx and Gy
 * @param gx Horizontal gradient
 * @param gy Vertical gradient
 * @param dir Output direction in radians [-PI, PI] (must be pre-allocated)
 * @param size Number of pixels
 */
template<typename T>
void GradientDirection(const T* gx, const T* gy, float* dir, size_t size);

/**
 * @brief Compute gradient direction for single pixel
 * @return Direction in radians [-PI, PI], 0 = horizontal, PI/2 = vertical
 */
template<typename T>
inline float DirectionFromGradient(T gx, T gy) {
    return static_cast<float>(std::atan2(static_cast<double>(gy),
                                          static_cast<double>(gx)));
}

// ============================================================================
// Combined Gradient Computation
// ============================================================================

/**
 * @brief Compute both Gx and Gy in one pass (more efficient)
 * @param src Source image
 * @param gx Output horizontal gradient
 * @param gy Output vertical gradient
 * @param width Image width
 * @param height Image height
 * @param op Gradient operator
 * @param borderMode Border handling
 */
template<typename SrcT, typename DstT = float>
void Gradient(const SrcT* src, DstT* gx, DstT* gy,
              int32_t width, int32_t height,
              GradientOperator op = GradientOperator::Sobel3x3,
              BorderMode borderMode = BorderMode::Reflect101);

/**
 * @brief Compute gradient magnitude and direction
 * @param src Source image
 * @param mag Output magnitude (may be nullptr if not needed)
 * @param dir Output direction in radians (may be nullptr if not needed)
 * @param width Image width
 * @param height Image height
 * @param op Gradient operator
 * @param borderMode Border handling
 */
template<typename SrcT>
void GradientMagDir(const SrcT* src, float* mag, float* dir,
                    int32_t width, int32_t height,
                    GradientOperator op = GradientOperator::Sobel3x3,
                    BorderMode borderMode = BorderMode::Reflect101);

// ============================================================================
// Gradient at Single Point (for subpixel edge detection)
// ============================================================================

/**
 * @brief Compute gradient at integer pixel using central difference
 * @param src Source image
 * @param width Image width
 * @param height Image height
 * @param x Pixel x coordinate
 * @param y Pixel y coordinate
 * @param[out] gx Horizontal gradient
 * @param[out] gy Vertical gradient
 * @param borderMode Border handling
 */
template<typename T>
void GradientAtPixel(const T* src, int32_t width, int32_t height,
                     int32_t x, int32_t y,
                     double& gx, double& gy,
                     BorderMode borderMode = BorderMode::Reflect101);

/**
 * @brief Compute gradient at subpixel location using interpolation
 * @param src Source image
 * @param width Image width
 * @param height Image height
 * @param x Subpixel x coordinate
 * @param y Subpixel y coordinate
 * @param[out] gx Horizontal gradient
 * @param[out] gy Vertical gradient
 * @param method Interpolation method
 * @param borderMode Border handling
 */
template<typename T>
void GradientAtSubpixel(const T* src, int32_t width, int32_t height,
                        double x, double y,
                        double& gx, double& gy,
                        InterpolationMethod method = InterpolationMethod::Bicubic,
                        BorderMode borderMode = BorderMode::Reflect101);

// ============================================================================
// Second Derivatives (for Hessian)
// ============================================================================

/**
 * @brief Compute second derivative dI/dx^2
 */
template<typename SrcT, typename DstT = float>
void GradientXX(const SrcT* src, DstT* dst,
                int32_t width, int32_t height,
                BorderMode borderMode = BorderMode::Reflect101);

/**
 * @brief Compute second derivative dI/dy^2
 */
template<typename SrcT, typename DstT = float>
void GradientYY(const SrcT* src, DstT* dst,
                int32_t width, int32_t height,
                BorderMode borderMode = BorderMode::Reflect101);

/**
 * @brief Compute mixed derivative dI/dxdy
 */
template<typename SrcT, typename DstT = float>
void GradientXY(const SrcT* src, DstT* dst,
                int32_t width, int32_t height,
                BorderMode borderMode = BorderMode::Reflect101);

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Apply 1D convolution (horizontal)
 */
template<typename SrcT, typename DstT>
void Convolve1DRow(const SrcT* src, DstT* dst,
                   int32_t width, int32_t height,
                   const double* kernel, int32_t kernelSize,
                   BorderMode borderMode = BorderMode::Reflect101);

/**
 * @brief Apply 1D convolution (vertical)
 */
template<typename SrcT, typename DstT>
void Convolve1DCol(const SrcT* src, DstT* dst,
                   int32_t width, int32_t height,
                   const double* kernel, int32_t kernelSize,
                   BorderMode borderMode = BorderMode::Reflect101);

/**
 * @brief Get kernel size for operator
 */
inline int32_t GetKernelSize(GradientOperator op) {
    switch (op) {
        case GradientOperator::Sobel5x5: return 5;
        case GradientOperator::Sobel7x7: return 7;
        default: return 3;
    }
}

// ============================================================================
// Template Implementations
// ============================================================================

template<typename T>
void GradientMagnitude(const T* gx, const T* gy, T* mag, size_t size,
                       bool normalize) {
    double maxVal = 0.0;
    if (normalize) {
        for (size_t i = 0; i < size; ++i) {
            double m = std::sqrt(static_cast<double>(gx[i]) * gx[i] +
                                  static_cast<double>(gy[i]) * gy[i]);
            mag[i] = static_cast<T>(m);
            if (m > maxVal) maxVal = m;
        }
        if (maxVal > 0) {
            double scale = 255.0 / maxVal;
            for (size_t i = 0; i < size; ++i) {
                mag[i] = static_cast<T>(static_cast<double>(mag[i]) * scale);
            }
        }
    } else {
        for (size_t i = 0; i < size; ++i) {
            mag[i] = static_cast<T>(std::sqrt(
                static_cast<double>(gx[i]) * gx[i] +
                static_cast<double>(gy[i]) * gy[i]));
        }
    }
}

template<typename T>
void GradientDirection(const T* gx, const T* gy, float* dir, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        dir[i] = static_cast<float>(std::atan2(
            static_cast<double>(gy[i]),
            static_cast<double>(gx[i])));
    }
}

template<typename T>
void GradientAtPixel(const T* src, int32_t width, int32_t height,
                     int32_t x, int32_t y,
                     double& gx, double& gy,
                     BorderMode borderMode) {
    auto getPixel = [&](int32_t px, int32_t py) -> double {
        if (px < 0 || px >= width || py < 0 || py >= height) {
            px = HandleBorder(px, width, borderMode);
            py = HandleBorder(py, height, borderMode);
        }
        return static_cast<double>(src[py * width + px]);
    };

    // Central difference
    gx = (getPixel(x + 1, y) - getPixel(x - 1, y)) * 0.5;
    gy = (getPixel(x, y + 1) - getPixel(x, y - 1)) * 0.5;
}

template<typename T>
void GradientAtSubpixel(const T* src, int32_t width, int32_t height,
                        double x, double y,
                        double& gx, double& gy,
                        InterpolationMethod method,
                        BorderMode borderMode) {
    if (method == InterpolationMethod::Bicubic) {
        InterpolateBicubicWithGradient(src, width, height, x, y,
                                        gx, gy, borderMode, 0.0);
    } else {
        InterpolateBilinearWithGradient(src, width, height, x, y,
                                         gx, gy, borderMode, 0.0);
    }
}

} // namespace Qi::Vision::Internal
