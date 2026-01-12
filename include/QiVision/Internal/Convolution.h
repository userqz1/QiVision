#pragma once

/**
 * @file Convolution.h
 * @brief Image convolution operations
 *
 * Provides:
 * - General 2D convolution
 * - Separable convolution (optimized)
 * - Domain-aware convolution
 * - Various border handling modes
 *
 * Used by:
 * - Gaussian blur (Filter)
 * - Gradient computation
 * - Edge detection
 * - Template matching preprocessing
 *
 * Performance:
 * - Separable convolution: O(2k) vs O(k²) for non-separable
 * - 64-byte aligned memory for SIMD optimization
 */

#include <QiVision/Core/Types.h>
#include <QiVision/Internal/Interpolate.h>

#include <cstdint>
#include <cstddef>
#include <vector>
#include <cmath>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace Qi::Vision::Internal {

// ============================================================================
// Separable Convolution
// ============================================================================

/**
 * @brief Apply separable 2D convolution (row then column)
 * @param src Source image
 * @param dst Destination image (must be pre-allocated)
 * @param width Image width
 * @param height Image height
 * @param kernelX Row (horizontal) kernel
 * @param kernelSizeX Size of row kernel
 * @param kernelY Column (vertical) kernel
 * @param kernelSizeY Size of column kernel
 * @param borderMode Border handling mode
 * @param borderValue Value for Constant border mode
 *
 * For Gaussian blur: kernelX = kernelY = Gaussian1D
 * For Sobel Gx: kernelX = derivative, kernelY = smooth
 */
template<typename SrcT, typename DstT = float>
void ConvolveSeparable(const SrcT* src, DstT* dst,
                       int32_t width, int32_t height,
                       const double* kernelX, int32_t kernelSizeX,
                       const double* kernelY, int32_t kernelSizeY,
                       BorderMode borderMode = BorderMode::Reflect101,
                       double borderValue = 0.0);

/**
 * @brief Apply separable convolution with same kernel for both directions
 * @param src Source image
 * @param dst Destination image
 * @param width Image width
 * @param height Image height
 * @param kernel Symmetric kernel (used for both row and column)
 * @param kernelSize Kernel size (must be odd)
 * @param borderMode Border handling
 */
template<typename SrcT, typename DstT = float>
void ConvolveSeparableSymmetric(const SrcT* src, DstT* dst,
                                int32_t width, int32_t height,
                                const double* kernel, int32_t kernelSize,
                                BorderMode borderMode = BorderMode::Reflect101);

// ============================================================================
// General 2D Convolution
// ============================================================================

/**
 * @brief Apply general 2D convolution
 * @param src Source image
 * @param dst Destination image (must be pre-allocated)
 * @param width Image width
 * @param height Image height
 * @param kernel 2D kernel (row-major, size = kernelWidth * kernelHeight)
 * @param kernelWidth Kernel width (must be odd)
 * @param kernelHeight Kernel height (must be odd)
 * @param borderMode Border handling mode
 * @param borderValue Value for Constant border mode
 *
 * Note: Prefer separable convolution when possible (much faster)
 */
template<typename SrcT, typename DstT = float>
void Convolve2D(const SrcT* src, DstT* dst,
                int32_t width, int32_t height,
                const double* kernel, int32_t kernelWidth, int32_t kernelHeight,
                BorderMode borderMode = BorderMode::Reflect101,
                double borderValue = 0.0);

// ============================================================================
// 1D Convolution (Row/Column)
// ============================================================================

/**
 * @brief Apply 1D convolution along rows (horizontal)
 * @param src Source image
 * @param dst Destination image
 * @param width Image width
 * @param height Image height
 * @param kernel 1D kernel
 * @param kernelSize Kernel size (must be odd)
 * @param borderMode Border handling
 * @param borderValue Value for Constant mode
 */
template<typename SrcT, typename DstT = float>
void ConvolveRow(const SrcT* src, DstT* dst,
                 int32_t width, int32_t height,
                 const double* kernel, int32_t kernelSize,
                 BorderMode borderMode = BorderMode::Reflect101,
                 double borderValue = 0.0);

/**
 * @brief Apply 1D convolution along columns (vertical)
 * @param src Source image
 * @param dst Destination image
 * @param width Image width
 * @param height Image height
 * @param kernel 1D kernel
 * @param kernelSize Kernel size (must be odd)
 * @param borderMode Border handling
 * @param borderValue Value for Constant mode
 */
template<typename SrcT, typename DstT = float>
void ConvolveCol(const SrcT* src, DstT* dst,
                 int32_t width, int32_t height,
                 const double* kernel, int32_t kernelSize,
                 BorderMode borderMode = BorderMode::Reflect101,
                 double borderValue = 0.0);

// ============================================================================
// Box Filter (Mean Filter)
// ============================================================================

/**
 * @brief Apply box (mean) filter
 * @param src Source image
 * @param dst Destination image
 * @param width Image width
 * @param height Image height
 * @param kernelWidth Box width (must be odd)
 * @param kernelHeight Box height (must be odd)
 * @param borderMode Border handling
 *
 * Optimized using integral image for large kernels
 */
template<typename SrcT, typename DstT = float>
void BoxFilter(const SrcT* src, DstT* dst,
               int32_t width, int32_t height,
               int32_t kernelWidth, int32_t kernelHeight,
               BorderMode borderMode = BorderMode::Reflect101);

/**
 * @brief Apply square box filter
 */
template<typename SrcT, typename DstT = float>
void BoxFilter(const SrcT* src, DstT* dst,
               int32_t width, int32_t height,
               int32_t kernelSize,
               BorderMode borderMode = BorderMode::Reflect101) {
    BoxFilter<SrcT, DstT>(src, dst, width, height, kernelSize, kernelSize, borderMode);
}

// ============================================================================
// Gaussian Blur
// ============================================================================

/**
 * @brief Apply Gaussian blur
 * @param src Source image
 * @param dst Destination image
 * @param width Image width
 * @param height Image height
 * @param sigmaX Sigma in X direction
 * @param sigmaY Sigma in Y direction (0 = same as sigmaX)
 * @param borderMode Border handling
 *
 * Kernel size is automatically computed as ceil(6*sigma) | 1
 */
template<typename SrcT, typename DstT = float>
void GaussianBlur(const SrcT* src, DstT* dst,
                  int32_t width, int32_t height,
                  double sigmaX, double sigmaY = 0.0,
                  BorderMode borderMode = BorderMode::Reflect101);

/**
 * @brief Apply Gaussian blur with specified kernel size
 * @param src Source image
 * @param dst Destination image
 * @param width Image width
 * @param height Image height
 * @param kernelSize Kernel size (must be odd)
 * @param sigma Gaussian sigma (0 = compute from kernel size)
 * @param borderMode Border handling
 */
template<typename SrcT, typename DstT = float>
void GaussianBlurFixed(const SrcT* src, DstT* dst,
                       int32_t width, int32_t height,
                       int32_t kernelSize, double sigma = 0.0,
                       BorderMode borderMode = BorderMode::Reflect101);

// ============================================================================
// Normalized Convolution (for handling Domain/masks)
// ============================================================================

/**
 * @brief Apply normalized convolution (handles missing data)
 * @param src Source image
 * @param mask Valid pixel mask (1 = valid, 0 = invalid)
 * @param dst Destination image
 * @param width Image width
 * @param height Image height
 * @param kernel 2D kernel
 * @param kernelWidth Kernel width
 * @param kernelHeight Kernel height
 *
 * Output is normalized by sum of weights for valid pixels only.
 * Useful for Domain-aware filtering.
 */
template<typename SrcT, typename DstT = float>
void ConvolveNormalized(const SrcT* src, const uint8_t* mask, DstT* dst,
                        int32_t width, int32_t height,
                        const double* kernel, int32_t kernelWidth, int32_t kernelHeight);

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Compute integral image (summed area table)
 * @param src Source image
 * @param integral Output integral image (size = (width+1) * (height+1))
 * @param width Image width
 * @param height Image height
 *
 * integral[y][x] = sum of all pixels above and to the left
 * Note: integral image has size (width+1) x (height+1)
 */
template<typename SrcT>
void ComputeIntegralImage(const SrcT* src, double* integral,
                          int32_t width, int32_t height);

/**
 * @brief Get sum of rectangle using integral image
 * @param integral Integral image
 * @param integralWidth Width of integral image (original width + 1)
 * @param x1, y1 Top-left corner (inclusive)
 * @param x2, y2 Bottom-right corner (exclusive)
 */
inline double GetRectSum(const double* integral, int32_t integralWidth,
                         int32_t x1, int32_t y1, int32_t x2, int32_t y2) {
    return integral[y2 * integralWidth + x2]
         - integral[y1 * integralWidth + x2]
         - integral[y2 * integralWidth + x1]
         + integral[y1 * integralWidth + x1];
}

/**
 * @brief Make kernel size odd (if even, add 1)
 */
inline int32_t MakeOdd(int32_t size) {
    return (size % 2 == 0) ? size + 1 : size;
}

/**
 * @brief Compute kernel size from sigma
 */
inline int32_t KernelSizeFromSigma(double sigma) {
    int32_t size = static_cast<int32_t>(std::ceil(sigma * 6.0));
    return MakeOdd(size);
}

/**
 * @brief Compute sigma from kernel size
 */
inline double SigmaFromKernelSize(int32_t size) {
    return static_cast<double>(size) / 6.0;
}

/**
 * @brief Generate 1D Gaussian kernel
 */
std::vector<double> GenerateGaussianKernel1D(double sigma, int32_t size = 0);

/**
 * @brief Generate 1D box kernel (all 1/n)
 */
std::vector<double> GenerateBoxKernel1D(int32_t size);

// ============================================================================
// Template Implementations
// ============================================================================

template<typename SrcT, typename DstT>
void ConvolveRow(const SrcT* src, DstT* dst,
                 int32_t width, int32_t height,
                 const double* kernel, int32_t kernelSize,
                 BorderMode borderMode, double borderValue) {
    int32_t halfK = kernelSize / 2;

    #pragma omp parallel for if(width * height > 100000)
    for (int32_t y = 0; y < height; ++y) {
        for (int32_t x = 0; x < width; ++x) {
            double sum = 0.0;
            for (int32_t k = -halfK; k <= halfK; ++k) {
                int32_t srcX = x + k;
                double pixelVal;

                if (srcX < 0 || srcX >= width) {
                    if (borderMode == BorderMode::Constant) {
                        pixelVal = borderValue;
                    } else {
                        srcX = HandleBorder(srcX, width, borderMode);
                        pixelVal = static_cast<double>(src[y * width + srcX]);
                    }
                } else {
                    pixelVal = static_cast<double>(src[y * width + srcX]);
                }
                sum += pixelVal * kernel[k + halfK];
            }
            dst[y * width + x] = static_cast<DstT>(sum);
        }
    }
}

template<typename SrcT, typename DstT>
void ConvolveCol(const SrcT* src, DstT* dst,
                 int32_t width, int32_t height,
                 const double* kernel, int32_t kernelSize,
                 BorderMode borderMode, double borderValue) {
    int32_t halfK = kernelSize / 2;

    #pragma omp parallel for if(width * height > 100000)
    for (int32_t y = 0; y < height; ++y) {
        for (int32_t x = 0; x < width; ++x) {
            double sum = 0.0;
            for (int32_t k = -halfK; k <= halfK; ++k) {
                int32_t srcY = y + k;
                double pixelVal;

                if (srcY < 0 || srcY >= height) {
                    if (borderMode == BorderMode::Constant) {
                        pixelVal = borderValue;
                    } else {
                        srcY = HandleBorder(srcY, height, borderMode);
                        pixelVal = static_cast<double>(src[srcY * width + x]);
                    }
                } else {
                    pixelVal = static_cast<double>(src[srcY * width + x]);
                }
                sum += pixelVal * kernel[k + halfK];
            }
            dst[y * width + x] = static_cast<DstT>(sum);
        }
    }
}

template<typename SrcT, typename DstT>
void ConvolveSeparable(const SrcT* src, DstT* dst,
                       int32_t width, int32_t height,
                       const double* kernelX, int32_t kernelSizeX,
                       const double* kernelY, int32_t kernelSizeY,
                       BorderMode borderMode, double borderValue) {
    // Allocate temporary buffer
    std::vector<double> temp(static_cast<size_t>(width) * height);

    // Step 1: Convolve rows (horizontal)
    ConvolveRow(src, temp.data(), width, height, kernelX, kernelSizeX,
                borderMode, borderValue);

    // Step 2: Convolve columns (vertical)
    ConvolveCol(temp.data(), dst, width, height, kernelY, kernelSizeY,
                borderMode, borderValue);
}

template<typename SrcT, typename DstT>
void ConvolveSeparableSymmetric(const SrcT* src, DstT* dst,
                                int32_t width, int32_t height,
                                const double* kernel, int32_t kernelSize,
                                BorderMode borderMode) {
    ConvolveSeparable<SrcT, DstT>(src, dst, width, height,
                                   kernel, kernelSize, kernel, kernelSize,
                                   borderMode, 0.0);
}

template<typename SrcT, typename DstT>
void Convolve2D(const SrcT* src, DstT* dst,
                int32_t width, int32_t height,
                const double* kernel, int32_t kernelWidth, int32_t kernelHeight,
                BorderMode borderMode, double borderValue) {
    int32_t halfKW = kernelWidth / 2;
    int32_t halfKH = kernelHeight / 2;

    auto getPixel = [&](int32_t px, int32_t py) -> double {
        if (px < 0 || px >= width || py < 0 || py >= height) {
            if (borderMode == BorderMode::Constant) {
                return borderValue;
            }
            px = HandleBorder(px, width, borderMode);
            py = HandleBorder(py, height, borderMode);
        }
        return static_cast<double>(src[py * width + px]);
    };

    for (int32_t y = 0; y < height; ++y) {
        for (int32_t x = 0; x < width; ++x) {
            double sum = 0.0;
            for (int32_t ky = -halfKH; ky <= halfKH; ++ky) {
                for (int32_t kx = -halfKW; kx <= halfKW; ++kx) {
                    double kVal = kernel[(ky + halfKH) * kernelWidth + (kx + halfKW)];
                    sum += getPixel(x + kx, y + ky) * kVal;
                }
            }
            dst[y * width + x] = static_cast<DstT>(sum);
        }
    }
}

template<typename SrcT>
void ComputeIntegralImage(const SrcT* src, double* integral,
                          int32_t width, int32_t height) {
    int32_t integralWidth = width + 1;

    // Initialize first row and column to zero
    for (int32_t x = 0; x <= width; ++x) {
        integral[x] = 0.0;
    }
    for (int32_t y = 0; y <= height; ++y) {
        integral[y * integralWidth] = 0.0;
    }

    // Compute integral image
    for (int32_t y = 0; y < height; ++y) {
        double rowSum = 0.0;
        for (int32_t x = 0; x < width; ++x) {
            rowSum += static_cast<double>(src[y * width + x]);
            integral[(y + 1) * integralWidth + (x + 1)] =
                rowSum + integral[y * integralWidth + (x + 1)];
        }
    }
}

template<typename SrcT, typename DstT>
void BoxFilter(const SrcT* src, DstT* dst,
               int32_t width, int32_t height,
               int32_t kernelWidth, int32_t kernelHeight,
               BorderMode borderMode) {
    // For small kernels, use direct convolution
    if (kernelWidth <= 5 && kernelHeight <= 5) {
        std::vector<double> kernel(kernelWidth * kernelHeight,
                                    1.0 / (kernelWidth * kernelHeight));
        Convolve2D<SrcT, DstT>(src, dst, width, height,
                                kernel.data(), kernelWidth, kernelHeight,
                                borderMode, 0.0);
        return;
    }

    // For larger kernels, use integral image (O(1) per pixel)
    std::vector<double> integral((width + 1) * (height + 1));
    ComputeIntegralImage(src, integral.data(), width, height);

    int32_t halfKW = kernelWidth / 2;
    int32_t halfKH = kernelHeight / 2;
    int32_t integralWidth = width + 1;

    for (int32_t y = 0; y < height; ++y) {
        for (int32_t x = 0; x < width; ++x) {
            // Compute rectangle bounds (clamped to image)
            int32_t x1 = std::max(0, x - halfKW);
            int32_t y1 = std::max(0, y - halfKH);
            int32_t x2 = std::min(width, x + halfKW + 1);
            int32_t y2 = std::min(height, y + halfKH + 1);

            double sum = GetRectSum(integral.data(), integralWidth, x1, y1, x2, y2);
            int32_t count = (x2 - x1) * (y2 - y1);

            dst[y * width + x] = static_cast<DstT>(sum / count);
        }
    }
}

template<typename SrcT, typename DstT>
void GaussianBlur(const SrcT* src, DstT* dst,
                  int32_t width, int32_t height,
                  double sigmaX, double sigmaY,
                  BorderMode borderMode) {
    if (sigmaY <= 0.0) sigmaY = sigmaX;

    auto kernelX = GenerateGaussianKernel1D(sigmaX);
    auto kernelY = GenerateGaussianKernel1D(sigmaY);

    ConvolveSeparable<SrcT, DstT>(src, dst, width, height,
                                   kernelX.data(), static_cast<int32_t>(kernelX.size()),
                                   kernelY.data(), static_cast<int32_t>(kernelY.size()),
                                   borderMode, 0.0);
}

template<typename SrcT, typename DstT>
void GaussianBlurFixed(const SrcT* src, DstT* dst,
                       int32_t width, int32_t height,
                       int32_t kernelSize, double sigma,
                       BorderMode borderMode) {
    kernelSize = MakeOdd(kernelSize);
    if (sigma <= 0.0) {
        sigma = SigmaFromKernelSize(kernelSize);
    }

    auto kernel = GenerateGaussianKernel1D(sigma, kernelSize);
    ConvolveSeparableSymmetric<SrcT, DstT>(src, dst, width, height,
                                            kernel.data(), kernelSize,
                                            borderMode);
}

template<typename SrcT, typename DstT>
void ConvolveNormalized(const SrcT* src, const uint8_t* mask, DstT* dst,
                        int32_t width, int32_t height,
                        const double* kernel, int32_t kernelWidth, int32_t kernelHeight) {
    int32_t halfKW = kernelWidth / 2;
    int32_t halfKH = kernelHeight / 2;

    for (int32_t y = 0; y < height; ++y) {
        for (int32_t x = 0; x < width; ++x) {
            double sum = 0.0;
            double weightSum = 0.0;

            for (int32_t ky = -halfKH; ky <= halfKH; ++ky) {
                for (int32_t kx = -halfKW; kx <= halfKW; ++kx) {
                    int32_t px = x + kx;
                    int32_t py = y + ky;

                    // Skip out-of-bounds and masked pixels
                    if (px < 0 || px >= width || py < 0 || py >= height) {
                        continue;
                    }
                    if (mask != nullptr && mask[py * width + px] == 0) {
                        continue;
                    }

                    double kVal = kernel[(ky + halfKH) * kernelWidth + (kx + halfKW)];
                    sum += static_cast<double>(src[py * width + px]) * kVal;
                    weightSum += kVal;
                }
            }

            if (weightSum > 0.0) {
                dst[y * width + x] = static_cast<DstT>(sum / weightSum);
            } else {
                dst[y * width + x] = static_cast<DstT>(0);
            }
        }
    }
}

} // namespace Qi::Vision::Internal
