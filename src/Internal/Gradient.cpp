/**
 * @file Gradient.cpp
 * @brief Image gradient computation implementation
 *
 * Optimized with OpenMP parallelization and AVX2 SIMD
 */

#include <QiVision/Internal/Gradient.h>

#include <algorithm>
#include <cstring>

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef __AVX2__
#include <immintrin.h>
#endif

namespace Qi::Vision::Internal {

// =============================================================================
// OpenMP 阈值控制
// =============================================================================

/// 启用 OpenMP 的最小像素数阈值 (约 700×700)
constexpr int32_t OPENMP_MIN_PIXELS = 500000;

/// 判断是否使用 OpenMP 并行
inline bool ShouldUseOpenMP(int32_t width, int32_t height) {
    return static_cast<int64_t>(width) * height >= OPENMP_MIN_PIXELS;
}

// ============================================================================
// Kernel Functions
// ============================================================================

std::vector<double> SobelDerivativeKernel(int32_t size) {
    switch (size) {
        case 3:
            return {-1.0, 0.0, 1.0};
        case 5:
            return {-1.0, -2.0, 0.0, 2.0, 1.0};
        case 7:
            return {-1.0, -4.0, -5.0, 0.0, 5.0, 4.0, 1.0};
        default:
            return {-1.0, 0.0, 1.0};
    }
}

std::vector<double> SobelSmoothingKernel(int32_t size) {
    switch (size) {
        case 3:
            return {1.0, 2.0, 1.0};
        case 5:
            return {1.0, 4.0, 6.0, 4.0, 1.0};
        case 7:
            return {1.0, 6.0, 15.0, 20.0, 15.0, 6.0, 1.0};
        default:
            return {1.0, 2.0, 1.0};
    }
}

std::vector<double> ScharrDerivativeKernel() {
    return {-1.0, 0.0, 1.0};
}

std::vector<double> ScharrSmoothingKernel() {
    // Scharr uses [3, 10, 3] / 16 for smoothing (better isotropy)
    return {3.0 / 16.0, 10.0 / 16.0, 3.0 / 16.0};
}

std::vector<double> PrewittDerivativeKernel() {
    return {-1.0, 0.0, 1.0};
}

std::vector<double> PrewittSmoothingKernel() {
    return {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};
}

// ============================================================================
// 1D Convolution Helpers (Optimized with OpenMP)
// ============================================================================

template<typename SrcT, typename DstT>
void Convolve1DRow(const SrcT* src, DstT* dst,
                   int32_t width, int32_t height,
                   const double* kernel, int32_t kernelSize,
                   BorderMode borderMode) {
    int32_t halfK = kernelSize / 2;
    const bool useParallel = ShouldUseOpenMP(width, height);

    auto processRow = [&](int32_t y) {
        for (int32_t x = 0; x < width; ++x) {
            double sum = 0.0;
            for (int32_t k = -halfK; k <= halfK; ++k) {
                int32_t srcX = x + k;
                if (srcX < 0 || srcX >= width) {
                    srcX = HandleBorder(srcX, width, borderMode);
                }
                sum += static_cast<double>(src[y * width + srcX]) *
                       kernel[k + halfK];
            }
            dst[y * width + x] = static_cast<DstT>(sum);
        }
    };

#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < height; ++y) { processRow(y); }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 0; y < height; ++y) { processRow(y); }
    }
}

template<typename SrcT, typename DstT>
void Convolve1DCol(const SrcT* src, DstT* dst,
                   int32_t width, int32_t height,
                   const double* kernel, int32_t kernelSize,
                   BorderMode borderMode) {
    int32_t halfK = kernelSize / 2;
    const bool useParallel = ShouldUseOpenMP(width, height);

    auto processRow = [&](int32_t y) {
        for (int32_t x = 0; x < width; ++x) {
            double sum = 0.0;
            for (int32_t k = -halfK; k <= halfK; ++k) {
                int32_t srcY = y + k;
                if (srcY < 0 || srcY >= height) {
                    srcY = HandleBorder(srcY, height, borderMode);
                }
                sum += static_cast<double>(src[srcY * width + x]) *
                       kernel[k + halfK];
            }
            dst[y * width + x] = static_cast<DstT>(sum);
        }
    };

#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < height; ++y) { processRow(y); }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 0; y < height; ++y) { processRow(y); }
    }
}

// ============================================================================
// Optimized Sobel 3x3 with AVX2
// ============================================================================

#ifdef __AVX2__
// Specialized Sobel 3x3 X gradient with AVX2
template<typename SrcT>
void SobelX_AVX2(const SrcT* src, float* dst, int32_t width, int32_t height, BorderMode borderMode) {
    // Sobel X: [-1 0 1] in X, [1 2 1] in Y
    // Combined as separable: smooth_y first, then deriv_x

    std::vector<float> temp(static_cast<size_t>(width) * height);
    const bool useParallel = ShouldUseOpenMP(width, height);

    // Step 1: Smooth in Y with [1 2 1]
    auto smoothY = [&](int32_t y) {
        int32_t ym1 = (y > 0) ? y - 1 : HandleBorder(-1, height, borderMode);
        int32_t yp1 = (y < height - 1) ? y + 1 : HandleBorder(height, height, borderMode);

        for (int32_t x = 0; x < width; ++x) {
            float v0 = static_cast<float>(src[ym1 * width + x]);
            float v1 = static_cast<float>(src[y * width + x]);
            float v2 = static_cast<float>(src[yp1 * width + x]);
            temp[y * width + x] = v0 + 2.0f * v1 + v2;
        }
    };

#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < height; ++y) { smoothY(y); }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 0; y < height; ++y) { smoothY(y); }
    }

    // Step 2: Derivative in X with [-1 0 1]
    auto derivX = [&](int32_t y) {
        const float* row = temp.data() + y * width;
        float* dstRow = dst + y * width;

        int32_t x = 0;

        // Handle left border
        {
            int32_t xm1 = HandleBorder(-1, width, borderMode);
            dstRow[0] = row[1] - row[xm1];
        }
        x = 1;

        // AVX2 main loop
        for (; x + 8 <= width - 1; x += 8) {
            __m256 left = _mm256_loadu_ps(row + x - 1);
            __m256 right = _mm256_loadu_ps(row + x + 1);
            __m256 result = _mm256_sub_ps(right, left);
            _mm256_storeu_ps(dstRow + x, result);
        }

        // Scalar for remaining
        for (; x < width - 1; ++x) {
            dstRow[x] = row[x + 1] - row[x - 1];
        }

        // Handle right border
        if (width > 1) {
            int32_t xp1 = HandleBorder(width, width, borderMode);
            dstRow[width - 1] = row[xp1] - row[width - 2];
        }
    };

#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < height; ++y) { derivX(y); }
    } else
#endif
    {
        for (int32_t y = 0; y < height; ++y) { derivX(y); }
    }
}

// Specialized Sobel 3x3 Y gradient with AVX2
template<typename SrcT>
void SobelY_AVX2(const SrcT* src, float* dst, int32_t width, int32_t height, BorderMode borderMode) {
    // Sobel Y: [1 2 1] in X, [-1 0 1] in Y

    std::vector<float> temp(static_cast<size_t>(width) * height);
    const bool useParallel = ShouldUseOpenMP(width, height);

    // Step 1: Smooth in X with [1 2 1]
    auto smoothX = [&](int32_t y) {
        const SrcT* row = src + y * width;
        float* tmpRow = temp.data() + y * width;

        // Left border
        {
            int32_t xm1 = HandleBorder(-1, width, borderMode);
            float v0 = static_cast<float>(row[xm1]);
            float v1 = static_cast<float>(row[0]);
            float v2 = static_cast<float>(row[1]);
            tmpRow[0] = v0 + 2.0f * v1 + v2;
        }

        int32_t x = 1;
        // AVX2 main loop
        for (; x + 8 <= width - 1; x += 8) {
            __m256 left, center, right;

            // Load and convert to float
            if constexpr (std::is_same_v<SrcT, float>) {
                left = _mm256_loadu_ps(row + x - 1);
                center = _mm256_loadu_ps(row + x);
                right = _mm256_loadu_ps(row + x + 1);
            } else {
                // For uint8_t or other types, load and convert
                float leftArr[8], centerArr[8], rightArr[8];
                for (int i = 0; i < 8; ++i) {
                    leftArr[i] = static_cast<float>(row[x - 1 + i]);
                    centerArr[i] = static_cast<float>(row[x + i]);
                    rightArr[i] = static_cast<float>(row[x + 1 + i]);
                }
                left = _mm256_loadu_ps(leftArr);
                center = _mm256_loadu_ps(centerArr);
                right = _mm256_loadu_ps(rightArr);
            }

            __m256 two = _mm256_set1_ps(2.0f);
            __m256 result = _mm256_add_ps(left, _mm256_fmadd_ps(two, center, right));
            _mm256_storeu_ps(tmpRow + x, result);
        }

        // Scalar for remaining
        for (; x < width - 1; ++x) {
            float v0 = static_cast<float>(row[x - 1]);
            float v1 = static_cast<float>(row[x]);
            float v2 = static_cast<float>(row[x + 1]);
            tmpRow[x] = v0 + 2.0f * v1 + v2;
        }

        // Right border
        if (width > 1) {
            int32_t xp1 = HandleBorder(width, width, borderMode);
            float v0 = static_cast<float>(row[width - 2]);
            float v1 = static_cast<float>(row[width - 1]);
            float v2 = static_cast<float>(row[xp1]);
            tmpRow[width - 1] = v0 + 2.0f * v1 + v2;
        }
    };

#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < height; ++y) { smoothX(y); }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 0; y < height; ++y) { smoothX(y); }
    }

    // Step 2: Derivative in Y with [-1 0 1]
    auto derivY = [&](int32_t y) {
        int32_t ym1 = (y > 0) ? y - 1 : HandleBorder(-1, height, borderMode);
        int32_t yp1 = (y < height - 1) ? y + 1 : HandleBorder(height, height, borderMode);

        const float* rowM1 = temp.data() + ym1 * width;
        const float* rowP1 = temp.data() + yp1 * width;
        float* dstRow = dst + y * width;

        int32_t x = 0;
        for (; x + 8 <= width; x += 8) {
            __m256 vm1 = _mm256_loadu_ps(rowM1 + x);
            __m256 vp1 = _mm256_loadu_ps(rowP1 + x);
            __m256 result = _mm256_sub_ps(vp1, vm1);
            _mm256_storeu_ps(dstRow + x, result);
        }

        for (; x < width; ++x) {
            dstRow[x] = rowP1[x] - rowM1[x];
        }
    };

#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < height; ++y) { derivY(y); }
    } else
#endif
    {
        for (int32_t y = 0; y < height; ++y) { derivY(y); }
    }
}
#endif

// ============================================================================
// Gradient X Implementation
// ============================================================================

template<typename SrcT, typename DstT>
void GradientX(const SrcT* src, DstT* dst,
               int32_t width, int32_t height,
               GradientOperator op, BorderMode borderMode) {

#ifdef __AVX2__
    // Use optimized AVX2 version for Sobel 3x3 with float output
    if (op == GradientOperator::Sobel3x3 && std::is_same_v<DstT, float>) {
        SobelX_AVX2(src, reinterpret_cast<float*>(dst), width, height, borderMode);
        return;
    }
#endif

    std::vector<double> smoothKernel, derivKernel;
    int32_t kernelSize;

    switch (op) {
        case GradientOperator::Sobel3x3:
            smoothKernel = SobelSmoothingKernel(3);
            derivKernel = SobelDerivativeKernel(3);
            kernelSize = 3;
            break;
        case GradientOperator::Sobel5x5:
            smoothKernel = SobelSmoothingKernel(5);
            derivKernel = SobelDerivativeKernel(5);
            kernelSize = 5;
            break;
        case GradientOperator::Sobel7x7:
            smoothKernel = SobelSmoothingKernel(7);
            derivKernel = SobelDerivativeKernel(7);
            kernelSize = 7;
            break;
        case GradientOperator::Scharr:
            smoothKernel = ScharrSmoothingKernel();
            derivKernel = ScharrDerivativeKernel();
            kernelSize = 3;
            break;
        case GradientOperator::Prewitt:
            smoothKernel = PrewittSmoothingKernel();
            derivKernel = PrewittDerivativeKernel();
            kernelSize = 3;
            break;
        case GradientOperator::Central: {
            // Simple central difference: (I(x+1) - I(x-1)) / 2
            const bool useParallel = ShouldUseOpenMP(width, height);
            auto processRow = [&](int32_t y) {
                for (int32_t x = 0; x < width; ++x) {
                    int32_t xm1 = (x > 0) ? x - 1 : HandleBorder(-1, width, borderMode);
                    int32_t xp1 = (x < width - 1) ? x + 1 : HandleBorder(width, width, borderMode);
                    dst[y * width + x] = static_cast<DstT>(
                        (static_cast<double>(src[y * width + xp1]) -
                         static_cast<double>(src[y * width + xm1])) * 0.5);
                }
            };
#ifdef _OPENMP
            if (useParallel) {
                #pragma omp parallel for schedule(static)
                for (int32_t y = 0; y < height; ++y) { processRow(y); }
            } else
#endif
            {
                (void)useParallel;
                for (int32_t y = 0; y < height; ++y) { processRow(y); }
            }
            return;
        }
        default:
            smoothKernel = SobelSmoothingKernel(3);
            derivKernel = SobelDerivativeKernel(3);
            kernelSize = 3;
    }

    // Separable convolution: Gx = deriv_x * smooth_y
    // Step 1: Smooth in Y direction
    std::vector<double> temp(width * height);
    Convolve1DCol(src, temp.data(), width, height,
                  smoothKernel.data(), kernelSize, borderMode);

    // Step 2: Derivative in X direction
    Convolve1DRow(temp.data(), dst, width, height,
                  derivKernel.data(), kernelSize, borderMode);
}

// ============================================================================
// Gradient Y Implementation
// ============================================================================

template<typename SrcT, typename DstT>
void GradientY(const SrcT* src, DstT* dst,
               int32_t width, int32_t height,
               GradientOperator op, BorderMode borderMode) {

#ifdef __AVX2__
    // Use optimized AVX2 version for Sobel 3x3 with float output
    if (op == GradientOperator::Sobel3x3 && std::is_same_v<DstT, float>) {
        SobelY_AVX2(src, reinterpret_cast<float*>(dst), width, height, borderMode);
        return;
    }
#endif

    std::vector<double> smoothKernel, derivKernel;
    int32_t kernelSize;

    switch (op) {
        case GradientOperator::Sobel3x3:
            smoothKernel = SobelSmoothingKernel(3);
            derivKernel = SobelDerivativeKernel(3);
            kernelSize = 3;
            break;
        case GradientOperator::Sobel5x5:
            smoothKernel = SobelSmoothingKernel(5);
            derivKernel = SobelDerivativeKernel(5);
            kernelSize = 5;
            break;
        case GradientOperator::Sobel7x7:
            smoothKernel = SobelSmoothingKernel(7);
            derivKernel = SobelDerivativeKernel(7);
            kernelSize = 7;
            break;
        case GradientOperator::Scharr:
            smoothKernel = ScharrSmoothingKernel();
            derivKernel = ScharrDerivativeKernel();
            kernelSize = 3;
            break;
        case GradientOperator::Prewitt:
            smoothKernel = PrewittSmoothingKernel();
            derivKernel = PrewittDerivativeKernel();
            kernelSize = 3;
            break;
        case GradientOperator::Central: {
            // Simple central difference: (I(y+1) - I(y-1)) / 2
            const bool useParallel = ShouldUseOpenMP(width, height);
            auto processRow = [&](int32_t y) {
                for (int32_t x = 0; x < width; ++x) {
                    int32_t ym1 = (y > 0) ? y - 1 : HandleBorder(-1, height, borderMode);
                    int32_t yp1 = (y < height - 1) ? y + 1 : HandleBorder(height, height, borderMode);
                    dst[y * width + x] = static_cast<DstT>(
                        (static_cast<double>(src[yp1 * width + x]) -
                         static_cast<double>(src[ym1 * width + x])) * 0.5);
                }
            };
#ifdef _OPENMP
            if (useParallel) {
                #pragma omp parallel for schedule(static)
                for (int32_t y = 0; y < height; ++y) { processRow(y); }
            } else
#endif
            {
                (void)useParallel;
                for (int32_t y = 0; y < height; ++y) { processRow(y); }
            }
            return;
        }
        default:
            smoothKernel = SobelSmoothingKernel(3);
            derivKernel = SobelDerivativeKernel(3);
            kernelSize = 3;
    }

    // Separable convolution: Gy = smooth_x * deriv_y
    // Step 1: Smooth in X direction
    std::vector<double> temp(width * height);
    Convolve1DRow(src, temp.data(), width, height,
                  smoothKernel.data(), kernelSize, borderMode);

    // Step 2: Derivative in Y direction
    Convolve1DCol(temp.data(), dst, width, height,
                  derivKernel.data(), kernelSize, borderMode);
}

// ============================================================================
// Combined Gradient (Optimized)
// ============================================================================

template<typename SrcT, typename DstT>
void Gradient(const SrcT* src, DstT* gx, DstT* gy,
              int32_t width, int32_t height,
              GradientOperator op, BorderMode borderMode) {

#ifdef __AVX2__
    // Use optimized AVX2 version for Sobel 3x3 with float output
    if (op == GradientOperator::Sobel3x3 && std::is_same_v<DstT, float>) {
        const bool useParallel = ShouldUseOpenMP(width, height);

#ifdef _OPENMP
        if (useParallel) {
            // Run both in parallel using OpenMP sections
            #pragma omp parallel sections
            {
                #pragma omp section
                {
                    SobelX_AVX2(src, reinterpret_cast<float*>(gx), width, height, borderMode);
                }
                #pragma omp section
                {
                    SobelY_AVX2(src, reinterpret_cast<float*>(gy), width, height, borderMode);
                }
            }
        } else
#endif
        {
            // 小图: 顺序执行
            (void)useParallel;
            SobelX_AVX2(src, reinterpret_cast<float*>(gx), width, height, borderMode);
            SobelY_AVX2(src, reinterpret_cast<float*>(gy), width, height, borderMode);
        }
        return;
    }
#endif

    std::vector<double> smoothKernel, derivKernel;
    int32_t kernelSize;

    switch (op) {
        case GradientOperator::Sobel3x3:
            smoothKernel = SobelSmoothingKernel(3);
            derivKernel = SobelDerivativeKernel(3);
            kernelSize = 3;
            break;
        case GradientOperator::Sobel5x5:
            smoothKernel = SobelSmoothingKernel(5);
            derivKernel = SobelDerivativeKernel(5);
            kernelSize = 5;
            break;
        case GradientOperator::Sobel7x7:
            smoothKernel = SobelSmoothingKernel(7);
            derivKernel = SobelDerivativeKernel(7);
            kernelSize = 7;
            break;
        case GradientOperator::Scharr:
            smoothKernel = ScharrSmoothingKernel();
            derivKernel = ScharrDerivativeKernel();
            kernelSize = 3;
            break;
        case GradientOperator::Prewitt:
            smoothKernel = PrewittSmoothingKernel();
            derivKernel = PrewittDerivativeKernel();
            kernelSize = 3;
            break;
        case GradientOperator::Central:
            // For central difference, compute separately
            GradientX<SrcT, DstT>(src, gx, width, height, op, borderMode);
            GradientY<SrcT, DstT>(src, gy, width, height, op, borderMode);
            return;
        default:
            smoothKernel = SobelSmoothingKernel(3);
            derivKernel = SobelDerivativeKernel(3);
            kernelSize = 3;
    }

    // Compute intermediates once
    std::vector<double> smoothX(width * height);
    std::vector<double> smoothY(width * height);

    // Smooth in X direction (for Gy)
    Convolve1DRow(src, smoothX.data(), width, height,
                  smoothKernel.data(), kernelSize, borderMode);

    // Smooth in Y direction (for Gx)
    Convolve1DCol(src, smoothY.data(), width, height,
                  smoothKernel.data(), kernelSize, borderMode);

    // Derivative in X direction (for Gx)
    Convolve1DRow(smoothY.data(), gx, width, height,
                  derivKernel.data(), kernelSize, borderMode);

    // Derivative in Y direction (for Gy)
    Convolve1DCol(smoothX.data(), gy, width, height,
                  derivKernel.data(), kernelSize, borderMode);
}

// ============================================================================
// Gradient Magnitude and Direction
// ============================================================================

template<typename SrcT>
void GradientMagDir(const SrcT* src, float* mag, float* dir,
                    int32_t width, int32_t height,
                    GradientOperator op, BorderMode borderMode) {
    size_t size = static_cast<size_t>(width) * height;

    std::vector<float> gx(size), gy(size);
    Gradient<SrcT, float>(src, gx.data(), gy.data(), width, height, op, borderMode);

    if (mag != nullptr) {
        GradientMagnitude(gx.data(), gy.data(), mag, size, false);
    }

    if (dir != nullptr) {
        GradientDirection(gx.data(), gy.data(), dir, size);
    }
}

// ============================================================================
// Second Derivatives
// ============================================================================

template<typename SrcT, typename DstT>
void GradientXX(const SrcT* src, DstT* dst,
                int32_t width, int32_t height,
                BorderMode borderMode) {
    // Second derivative in X: d²I/dx² = I(x+1) - 2*I(x) + I(x-1)
    const bool useParallel = ShouldUseOpenMP(width, height);

    auto processRow = [&](int32_t y) {
        for (int32_t x = 0; x < width; ++x) {
            int32_t xm1 = (x > 0) ? x - 1 : HandleBorder(-1, width, borderMode);
            int32_t xp1 = (x < width - 1) ? x + 1 : HandleBorder(width, width, borderMode);

            double val = static_cast<double>(src[y * width + xp1]) -
                         2.0 * static_cast<double>(src[y * width + x]) +
                         static_cast<double>(src[y * width + xm1]);
            dst[y * width + x] = static_cast<DstT>(val);
        }
    };

#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < height; ++y) { processRow(y); }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 0; y < height; ++y) { processRow(y); }
    }
}

template<typename SrcT, typename DstT>
void GradientYY(const SrcT* src, DstT* dst,
                int32_t width, int32_t height,
                BorderMode borderMode) {
    // Second derivative in Y: d²I/dy² = I(y+1) - 2*I(y) + I(y-1)
    const bool useParallel = ShouldUseOpenMP(width, height);

    auto processRow = [&](int32_t y) {
        for (int32_t x = 0; x < width; ++x) {
            int32_t ym1 = (y > 0) ? y - 1 : HandleBorder(-1, height, borderMode);
            int32_t yp1 = (y < height - 1) ? y + 1 : HandleBorder(height, height, borderMode);

            double val = static_cast<double>(src[yp1 * width + x]) -
                         2.0 * static_cast<double>(src[y * width + x]) +
                         static_cast<double>(src[ym1 * width + x]);
            dst[y * width + x] = static_cast<DstT>(val);
        }
    };

#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < height; ++y) { processRow(y); }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 0; y < height; ++y) { processRow(y); }
    }
}

template<typename SrcT, typename DstT>
void GradientXY(const SrcT* src, DstT* dst,
                int32_t width, int32_t height,
                BorderMode borderMode) {
    // Mixed derivative: d²I/dxdy
    // = (I(x+1,y+1) - I(x-1,y+1) - I(x+1,y-1) + I(x-1,y-1)) / 4
    const bool useParallel = ShouldUseOpenMP(width, height);

    auto getPixel = [&](int32_t px, int32_t py) -> double {
        if (px < 0 || px >= width) {
            px = HandleBorder(px, width, borderMode);
        }
        if (py < 0 || py >= height) {
            py = HandleBorder(py, height, borderMode);
        }
        return static_cast<double>(src[py * width + px]);
    };

    auto processRow = [&](int32_t y) {
        for (int32_t x = 0; x < width; ++x) {
            double val = (getPixel(x + 1, y + 1) - getPixel(x - 1, y + 1) -
                          getPixel(x + 1, y - 1) + getPixel(x - 1, y - 1)) * 0.25;
            dst[y * width + x] = static_cast<DstT>(val);
        }
    };

#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < height; ++y) { processRow(y); }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 0; y < height; ++y) { processRow(y); }
    }
}

// ============================================================================
// Explicit Template Instantiations
// ============================================================================

// Convolve1D
template void Convolve1DRow<uint8_t, double>(const uint8_t*, double*, int32_t, int32_t, const double*, int32_t, BorderMode);
template void Convolve1DRow<double, double>(const double*, double*, int32_t, int32_t, const double*, int32_t, BorderMode);
template void Convolve1DRow<double, float>(const double*, float*, int32_t, int32_t, const double*, int32_t, BorderMode);
template void Convolve1DCol<uint8_t, double>(const uint8_t*, double*, int32_t, int32_t, const double*, int32_t, BorderMode);
template void Convolve1DCol<double, double>(const double*, double*, int32_t, int32_t, const double*, int32_t, BorderMode);
template void Convolve1DCol<double, float>(const double*, float*, int32_t, int32_t, const double*, int32_t, BorderMode);

// GradientX
template void GradientX<uint8_t, float>(const uint8_t*, float*, int32_t, int32_t, GradientOperator, BorderMode);
template void GradientX<uint16_t, float>(const uint16_t*, float*, int32_t, int32_t, GradientOperator, BorderMode);
template void GradientX<int16_t, float>(const int16_t*, float*, int32_t, int32_t, GradientOperator, BorderMode);
template void GradientX<float, float>(const float*, float*, int32_t, int32_t, GradientOperator, BorderMode);
template void GradientX<double, double>(const double*, double*, int32_t, int32_t, GradientOperator, BorderMode);

// GradientY
template void GradientY<uint8_t, float>(const uint8_t*, float*, int32_t, int32_t, GradientOperator, BorderMode);
template void GradientY<uint16_t, float>(const uint16_t*, float*, int32_t, int32_t, GradientOperator, BorderMode);
template void GradientY<int16_t, float>(const int16_t*, float*, int32_t, int32_t, GradientOperator, BorderMode);
template void GradientY<float, float>(const float*, float*, int32_t, int32_t, GradientOperator, BorderMode);
template void GradientY<double, double>(const double*, double*, int32_t, int32_t, GradientOperator, BorderMode);

// Gradient
template void Gradient<uint8_t, float>(const uint8_t*, float*, float*, int32_t, int32_t, GradientOperator, BorderMode);
template void Gradient<uint16_t, float>(const uint16_t*, float*, float*, int32_t, int32_t, GradientOperator, BorderMode);
template void Gradient<int16_t, float>(const int16_t*, float*, float*, int32_t, int32_t, GradientOperator, BorderMode);
template void Gradient<float, float>(const float*, float*, float*, int32_t, int32_t, GradientOperator, BorderMode);
template void Gradient<double, double>(const double*, double*, double*, int32_t, int32_t, GradientOperator, BorderMode);

// GradientMagDir
template void GradientMagDir<uint8_t>(const uint8_t*, float*, float*, int32_t, int32_t, GradientOperator, BorderMode);
template void GradientMagDir<uint16_t>(const uint16_t*, float*, float*, int32_t, int32_t, GradientOperator, BorderMode);
template void GradientMagDir<float>(const float*, float*, float*, int32_t, int32_t, GradientOperator, BorderMode);

// Second derivatives
template void GradientXX<uint8_t, float>(const uint8_t*, float*, int32_t, int32_t, BorderMode);
template void GradientXX<float, float>(const float*, float*, int32_t, int32_t, BorderMode);
template void GradientYY<uint8_t, float>(const uint8_t*, float*, int32_t, int32_t, BorderMode);
template void GradientYY<float, float>(const float*, float*, int32_t, int32_t, BorderMode);
template void GradientXY<uint8_t, float>(const uint8_t*, float*, int32_t, int32_t, BorderMode);
template void GradientXY<float, float>(const float*, float*, int32_t, int32_t, BorderMode);

} // namespace Qi::Vision::Internal
