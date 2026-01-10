/**
 * @file AnglePyramid.cpp
 * @brief Implementation of angle pyramid for shape-based matching
 *
 * Optimized with OpenMP parallelization and AVX2 SIMD
 */

#include <QiVision/Internal/AnglePyramid.h>
#include <QiVision/Internal/Gradient.h>
#include <QiVision/Internal/Gaussian.h>
#include <QiVision/Internal/Convolution.h>
#include <QiVision/Internal/Pyramid.h>
#include <QiVision/Internal/Interpolate.h>
#include <QiVision/Internal/Geometry2d.h>
#include <QiVision/Core/Constants.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef __AVX2__
#include <immintrin.h>
#endif

namespace Qi::Vision::Internal {

// =============================================================================
// SIMD Helper Functions
// =============================================================================

#ifdef __AVX2__
// Fast atan2 approximation for 8 float values
// Using polynomial approximation, accurate to ~0.01 radians
static inline __m256 atan2_avx2(__m256 y, __m256 x) {
    // Constants
    const __m256 pi = _mm256_set1_ps(static_cast<float>(PI));
    const __m256 pi_2 = _mm256_set1_ps(static_cast<float>(PI / 2.0));
    const __m256 zero = _mm256_setzero_ps();
    const __m256 one = _mm256_set1_ps(1.0f);
    const __m256 neg_one = _mm256_set1_ps(-1.0f);

    // Polynomial coefficients for atan approximation
    const __m256 c1 = _mm256_set1_ps(0.9998660f);
    const __m256 c3 = _mm256_set1_ps(-0.3302995f);
    const __m256 c5 = _mm256_set1_ps(0.1801410f);
    const __m256 c7 = _mm256_set1_ps(-0.0851330f);

    // Compute |y| and |x|
    __m256 abs_y = _mm256_andnot_ps(_mm256_set1_ps(-0.0f), y);
    __m256 abs_x = _mm256_andnot_ps(_mm256_set1_ps(-0.0f), x);

    // Determine which octant we're in
    __m256 swap_mask = _mm256_cmp_ps(abs_y, abs_x, _CMP_GT_OQ);
    __m256 num = _mm256_blendv_ps(abs_y, abs_x, swap_mask);
    __m256 den = _mm256_blendv_ps(abs_x, abs_y, swap_mask);

    // Avoid division by zero
    den = _mm256_max_ps(den, _mm256_set1_ps(1e-10f));

    // t = num / den (in range [0, 1])
    __m256 t = _mm256_div_ps(num, den);
    __m256 t2 = _mm256_mul_ps(t, t);

    // Polynomial: atan(t) ≈ t * (c1 + t² * (c3 + t² * (c5 + t² * c7)))
    __m256 poly = _mm256_fmadd_ps(t2, c7, c5);
    poly = _mm256_fmadd_ps(t2, poly, c3);
    poly = _mm256_fmadd_ps(t2, poly, c1);
    __m256 atan_val = _mm256_mul_ps(t, poly);

    // Adjust for octant: if swapped, result = π/2 - atan
    atan_val = _mm256_blendv_ps(atan_val, _mm256_sub_ps(pi_2, atan_val), swap_mask);

    // Adjust for quadrant
    __m256 x_neg = _mm256_cmp_ps(x, zero, _CMP_LT_OQ);
    __m256 y_neg = _mm256_cmp_ps(y, zero, _CMP_LT_OQ);

    // If x < 0: result = π - result
    atan_val = _mm256_blendv_ps(atan_val, _mm256_sub_ps(pi, atan_val), x_neg);

    // If y < 0: result = -result
    atan_val = _mm256_blendv_ps(atan_val, _mm256_sub_ps(zero, atan_val), y_neg);

    // Normalize to [0, 2π]
    __m256 neg_mask = _mm256_cmp_ps(atan_val, zero, _CMP_LT_OQ);
    __m256 two_pi = _mm256_set1_ps(static_cast<float>(2.0 * PI));
    atan_val = _mm256_blendv_ps(atan_val, _mm256_add_ps(atan_val, two_pi), neg_mask);

    return atan_val;
}

// Fast sqrt for 8 floats using rsqrt + Newton-Raphson
static inline __m256 fast_sqrt_avx2(__m256 x) {
    // sqrt(x) = x * rsqrt(x)
    // With one Newton-Raphson iteration for better accuracy
    __m256 zero = _mm256_setzero_ps();
    __m256 half = _mm256_set1_ps(0.5f);
    __m256 three = _mm256_set1_ps(3.0f);

    // Initial estimate
    __m256 rsqrt_x = _mm256_rsqrt_ps(x);

    // Newton-Raphson: rsqrt' = 0.5 * rsqrt * (3 - x * rsqrt^2)
    __m256 x_rsqrt2 = _mm256_mul_ps(x, _mm256_mul_ps(rsqrt_x, rsqrt_x));
    rsqrt_x = _mm256_mul_ps(_mm256_mul_ps(half, rsqrt_x), _mm256_sub_ps(three, x_rsqrt2));

    // sqrt(x) = x * rsqrt(x), but handle zero case
    __m256 sqrt_x = _mm256_mul_ps(x, rsqrt_x);
    sqrt_x = _mm256_blendv_ps(sqrt_x, zero, _mm256_cmp_ps(x, zero, _CMP_EQ_OQ));

    return sqrt_x;
}
#endif

// =============================================================================
// Implementation Class
// =============================================================================

class AnglePyramid::Impl {
public:
    AnglePyramidParams params_;
    std::vector<PyramidLevelData> levels_;
    int32_t originalWidth_ = 0;
    int32_t originalHeight_ = 0;
    bool valid_ = false;
    AnglePyramidTiming timing_;  // Timing statistics

    bool BuildLevel(const std::vector<float>& srcData, int32_t width, int32_t height,
                    int32_t level, double scale);
    void ExtractEdgePointsForLevel(int32_t level);

    // Optimized functions (combined for speed when not timing)
    void ComputeGradientMagnitudeDirectionOpt(const float* gx, const float* gy,
                                               float* mag, float* dir,
                                               int32_t width, int32_t height,
                                               int32_t gxStride, int32_t gyStride,
                                               int32_t magStride, int32_t dirStride);

    // Separate functions for detailed timing
    void ComputeMagnitudeOnly(const float* gx, const float* gy, float* mag,
                              int32_t width, int32_t height,
                              int32_t gxStride, int32_t gyStride, int32_t magStride);

    void ComputeDirectionOnly(const float* gx, const float* gy, float* dir,
                              int32_t width, int32_t height,
                              int32_t gxStride, int32_t gyStride, int32_t dirStride);

    void QuantizeDirectionOpt(const float* dir, int16_t* bins,
                              int32_t width, int32_t height,
                              int32_t dirStride, int32_t binStride,
                              int32_t numBins);
};

void AnglePyramid::Impl::ComputeGradientMagnitudeDirectionOpt(
    const float* gx, const float* gy,
    float* mag, float* dir,
    int32_t width, int32_t height,
    int32_t gxStride, int32_t gyStride,
    int32_t magStride, int32_t dirStride)
{
#ifdef __AVX2__
    const float twoPi = static_cast<float>(2.0 * PI);

    #pragma omp parallel for schedule(static)
    for (int32_t y = 0; y < height; ++y) {
        const float* gxRow = gx + y * gxStride;
        const float* gyRow = gy + y * gyStride;
        float* magRow = mag + y * magStride;
        float* dirRow = dir + y * dirStride;

        int32_t x = 0;
        // AVX2: process 8 pixels at a time
        for (; x + 8 <= width; x += 8) {
            __m256 vgx = _mm256_loadu_ps(gxRow + x);
            __m256 vgy = _mm256_loadu_ps(gyRow + x);

            // Magnitude: sqrt(gx^2 + gy^2)
            __m256 gx2 = _mm256_mul_ps(vgx, vgx);
            __m256 gy2 = _mm256_mul_ps(vgy, vgy);
            __m256 vmag = fast_sqrt_avx2(_mm256_add_ps(gx2, gy2));
            _mm256_storeu_ps(magRow + x, vmag);

            // Direction: atan2(gy, gx) normalized to [0, 2π]
            __m256 vdir = atan2_avx2(vgy, vgx);
            _mm256_storeu_ps(dirRow + x, vdir);
        }

        // Scalar fallback for remaining pixels
        for (; x < width; ++x) {
            float gxVal = gxRow[x];
            float gyVal = gyRow[x];
            magRow[x] = std::sqrt(gxVal * gxVal + gyVal * gyVal);
            float angle = std::atan2(gyVal, gxVal);
            if (angle < 0) angle += twoPi;
            dirRow[x] = angle;
        }
    }
#else
    // Non-SIMD version with OpenMP
    const float twoPi = static_cast<float>(2.0 * PI);

    #pragma omp parallel for schedule(static)
    for (int32_t y = 0; y < height; ++y) {
        const float* gxRow = gx + y * gxStride;
        const float* gyRow = gy + y * gyStride;
        float* magRow = mag + y * magStride;
        float* dirRow = dir + y * dirStride;

        for (int32_t x = 0; x < width; ++x) {
            float gxVal = gxRow[x];
            float gyVal = gyRow[x];
            magRow[x] = std::sqrt(gxVal * gxVal + gyVal * gyVal);
            float angle = std::atan2(gyVal, gxVal);
            if (angle < 0) angle += twoPi;
            dirRow[x] = angle;
        }
    }
#endif
}

// =============================================================================
// Separate Magnitude and Direction functions for detailed timing
// =============================================================================

void AnglePyramid::Impl::ComputeMagnitudeOnly(
    const float* gx, const float* gy, float* mag,
    int32_t width, int32_t height,
    int32_t gxStride, int32_t gyStride, int32_t magStride)
{
#ifdef __AVX2__
    #pragma omp parallel for schedule(static)
    for (int32_t y = 0; y < height; ++y) {
        const float* gxRow = gx + y * gxStride;
        const float* gyRow = gy + y * gyStride;
        float* magRow = mag + y * magStride;

        int32_t x = 0;
        for (; x + 8 <= width; x += 8) {
            __m256 vgx = _mm256_loadu_ps(gxRow + x);
            __m256 vgy = _mm256_loadu_ps(gyRow + x);
            __m256 gx2 = _mm256_mul_ps(vgx, vgx);
            __m256 gy2 = _mm256_mul_ps(vgy, vgy);
            __m256 vmag = fast_sqrt_avx2(_mm256_add_ps(gx2, gy2));
            _mm256_storeu_ps(magRow + x, vmag);
        }
        for (; x < width; ++x) {
            float gxVal = gxRow[x];
            float gyVal = gyRow[x];
            magRow[x] = std::sqrt(gxVal * gxVal + gyVal * gyVal);
        }
    }
#else
    #pragma omp parallel for schedule(static)
    for (int32_t y = 0; y < height; ++y) {
        const float* gxRow = gx + y * gxStride;
        const float* gyRow = gy + y * gyStride;
        float* magRow = mag + y * magStride;
        for (int32_t x = 0; x < width; ++x) {
            float gxVal = gxRow[x];
            float gyVal = gyRow[x];
            magRow[x] = std::sqrt(gxVal * gxVal + gyVal * gyVal);
        }
    }
#endif
}

void AnglePyramid::Impl::ComputeDirectionOnly(
    const float* gx, const float* gy, float* dir,
    int32_t width, int32_t height,
    int32_t gxStride, int32_t gyStride, int32_t dirStride)
{
    const float twoPi = static_cast<float>(2.0 * PI);

#ifdef __AVX2__
    #pragma omp parallel for schedule(static)
    for (int32_t y = 0; y < height; ++y) {
        const float* gxRow = gx + y * gxStride;
        const float* gyRow = gy + y * gyStride;
        float* dirRow = dir + y * dirStride;

        int32_t x = 0;
        for (; x + 8 <= width; x += 8) {
            __m256 vgx = _mm256_loadu_ps(gxRow + x);
            __m256 vgy = _mm256_loadu_ps(gyRow + x);
            __m256 vdir = atan2_avx2(vgy, vgx);
            _mm256_storeu_ps(dirRow + x, vdir);
        }
        for (; x < width; ++x) {
            float angle = std::atan2(gyRow[x], gxRow[x]);
            if (angle < 0) angle += twoPi;
            dirRow[x] = angle;
        }
    }
#else
    #pragma omp parallel for schedule(static)
    for (int32_t y = 0; y < height; ++y) {
        const float* gxRow = gx + y * gxStride;
        const float* gyRow = gy + y * gyStride;
        float* dirRow = dir + y * dirStride;
        for (int32_t x = 0; x < width; ++x) {
            float angle = std::atan2(gyRow[x], gxRow[x]);
            if (angle < 0) angle += twoPi;
            dirRow[x] = angle;
        }
    }
#endif
}

void AnglePyramid::Impl::QuantizeDirectionOpt(
    const float* dir, int16_t* bins,
    int32_t width, int32_t height,
    int32_t dirStride, int32_t binStride,
    int32_t numBins)
{
    const float binScale = static_cast<float>(numBins / (2.0 * PI));
    const int16_t maxBin = static_cast<int16_t>(numBins - 1);

#ifdef __AVX2__
    const __m256 vBinScale = _mm256_set1_ps(binScale);
    const __m256i vMaxBin = _mm256_set1_epi32(numBins - 1);
    const __m256i vZero = _mm256_setzero_si256();

    #pragma omp parallel for schedule(static)
    for (int32_t y = 0; y < height; ++y) {
        const float* dirRow = dir + y * dirStride;
        int16_t* binRow = bins + y * binStride;

        int32_t x = 0;
        for (; x + 8 <= width; x += 8) {
            __m256 vdir = _mm256_loadu_ps(dirRow + x);
            __m256 vbin_f = _mm256_mul_ps(vdir, vBinScale);
            __m256i vbin = _mm256_cvttps_epi32(vbin_f);
            vbin = _mm256_max_epi32(vbin, vZero);
            vbin = _mm256_min_epi32(vbin, vMaxBin);
            __m128i lo = _mm256_castsi256_si128(vbin);
            __m128i hi = _mm256_extracti128_si256(vbin, 1);
            __m128i packed = _mm_packs_epi32(lo, hi);
            _mm_storeu_si128(reinterpret_cast<__m128i*>(binRow + x), packed);
        }
        for (; x < width; ++x) {
            int32_t bin = static_cast<int32_t>(dirRow[x] * binScale);
            binRow[x] = static_cast<int16_t>(std::clamp(bin, 0, static_cast<int32_t>(maxBin)));
        }
    }
#else
    #pragma omp parallel for schedule(static)
    for (int32_t y = 0; y < height; ++y) {
        const float* dirRow = dir + y * dirStride;
        int16_t* binRow = bins + y * binStride;
        for (int32_t x = 0; x < width; ++x) {
            int32_t bin = static_cast<int32_t>(dirRow[x] * binScale);
            binRow[x] = static_cast<int16_t>(std::clamp(bin, 0, static_cast<int32_t>(maxBin)));
        }
    }
#endif
}

bool AnglePyramid::Impl::BuildLevel(const std::vector<float>& srcData, int32_t width, int32_t height,
                                    int32_t level, double scale) {
    using Clock = std::chrono::high_resolution_clock;
    auto t0 = Clock::now(), t1 = t0;

    PyramidLevelData levelData;
    levelData.level = level;
    levelData.width = width;
    levelData.height = height;
    levelData.scale = scale;

    // Allocate contiguous buffers for gradients
    std::vector<float> gxBuffer(static_cast<size_t>(width) * height);
    std::vector<float> gyBuffer(static_cast<size_t>(width) * height);

    // Use Sobel operator for gradient computation (contiguous memory)
    t0 = Clock::now();
    Gradient<float, float>(srcData.data(), gxBuffer.data(), gyBuffer.data(),
                           width, height,
                           GradientOperator::Sobel3x3, BorderMode::Reflect101);
    t1 = Clock::now();
    if (params_.enableTiming) {
        timing_.sobelMs += std::chrono::duration<double, std::milli>(t1 - t0).count();
    }

    // Create QImage for storage
    QImage gx(width, height, PixelType::Float32, ChannelType::Gray);
    QImage gy(width, height, PixelType::Float32, ChannelType::Gray);

    float* gxDst = static_cast<float*>(gx.Data());
    float* gyDst = static_cast<float*>(gy.Data());
    int32_t gxStride = gx.Stride() / sizeof(float);
    int32_t gyStride = gy.Stride() / sizeof(float);

    // Copy with OpenMP parallelization
    t0 = Clock::now();
    #pragma omp parallel for schedule(static)
    for (int32_t y = 0; y < height; ++y) {
        for (int32_t x = 0; x < width; ++x) {
            gxDst[y * gxStride + x] = gxBuffer[y * width + x];
            gyDst[y * gyStride + x] = gyBuffer[y * width + x];
        }
    }
    t1 = Clock::now();
    if (params_.enableTiming) {
        timing_.copyMs += std::chrono::duration<double, std::milli>(t1 - t0).count();
    }

    if (gx.Empty() || gy.Empty()) {
        return false;
    }

    levelData.gradX = std::move(gx);
    levelData.gradY = std::move(gy);

    // Allocate magnitude and direction images
    levelData.gradMag = QImage(width, height, PixelType::Float32, ChannelType::Gray);
    levelData.gradDir = QImage(width, height, PixelType::Float32, ChannelType::Gray);

    // Compute magnitude and direction
    const float* gxData = static_cast<const float*>(levelData.gradX.Data());
    const float* gyData = static_cast<const float*>(levelData.gradY.Data());
    float* magData = static_cast<float*>(levelData.gradMag.Data());
    float* dirData = static_cast<float*>(levelData.gradDir.Data());

    int32_t gxStr = levelData.gradX.Stride() / sizeof(float);
    int32_t gyStr = levelData.gradY.Stride() / sizeof(float);
    int32_t magStr = levelData.gradMag.Stride() / sizeof(float);
    int32_t dirStr = levelData.gradDir.Stride() / sizeof(float);

    if (params_.enableTiming) {
        // Separate timing for sqrt and atan2
        t0 = Clock::now();
        ComputeMagnitudeOnly(gxData, gyData, magData, width, height, gxStr, gyStr, magStr);
        t1 = Clock::now();
        timing_.sqrtMs += std::chrono::duration<double, std::milli>(t1 - t0).count();

        t0 = Clock::now();
        ComputeDirectionOnly(gxData, gyData, dirData, width, height, gxStr, gyStr, dirStr);
        t1 = Clock::now();
        timing_.atan2Ms += std::chrono::duration<double, std::milli>(t1 - t0).count();
    } else {
        // Combined computation (slightly faster)
        ComputeGradientMagnitudeDirectionOpt(gxData, gyData, magData, dirData,
                                              width, height,
                                              gxStr, gyStr, magStr, dirStr);
    }

    // Allocate and quantize direction to angle bins
    levelData.angleBinImage = QImage(width, height, PixelType::Int16, ChannelType::Gray);
    int16_t* binData = static_cast<int16_t*>(levelData.angleBinImage.Data());
    int32_t binStr = levelData.angleBinImage.Stride() / sizeof(int16_t);

    t0 = Clock::now();
    QuantizeDirectionOpt(dirData, binData, width, height, dirStr, binStr, params_.angleBins);
    t1 = Clock::now();
    if (params_.enableTiming) {
        timing_.quantizeMs += std::chrono::duration<double, std::milli>(t1 - t0).count();
    }

    levels_.push_back(std::move(levelData));
    return true;
}

void AnglePyramid::Impl::ExtractEdgePointsForLevel(int32_t level) {
    if (level < 0 || level >= static_cast<int32_t>(levels_.size())) {
        return;
    }

    auto& levelData = levels_[level];
    levelData.edgePoints.clear();

    const float* magData = static_cast<const float*>(levelData.gradMag.Data());
    const float* dirData = static_cast<const float*>(levelData.gradDir.Data());
    const int16_t* binData = static_cast<const int16_t*>(levelData.angleBinImage.Data());

    int32_t magStride = levelData.gradMag.Stride() / sizeof(float);
    int32_t dirStride = levelData.gradDir.Stride() / sizeof(float);
    int32_t binStride = levelData.angleBinImage.Stride() / sizeof(int16_t);

    double minContrast = params_.minContrast;

    // For edge point extraction, we need thread-local vectors then merge
    #pragma omp parallel
    {
        std::vector<EdgePoint> localPoints;
        localPoints.reserve(1000);

        #pragma omp for schedule(static) nowait
        for (int32_t y = 1; y < levelData.height - 1; ++y) {
            for (int32_t x = 1; x < levelData.width - 1; ++x) {
                float mag = magData[y * magStride + x];

                if (mag >= minContrast) {
                    float dir = dirData[y * dirStride + x];
                    int16_t bin = binData[y * binStride + x];

                    localPoints.emplace_back(
                        static_cast<double>(x),
                        static_cast<double>(y),
                        static_cast<double>(dir),
                        static_cast<double>(mag),
                        static_cast<int32_t>(bin)
                    );
                }
            }
        }

        #pragma omp critical
        {
            levelData.edgePoints.insert(levelData.edgePoints.end(),
                                         localPoints.begin(), localPoints.end());
        }
    }
}

// =============================================================================
// AnglePyramid Implementation
// =============================================================================

AnglePyramid::AnglePyramid() : impl_(std::make_unique<Impl>()) {}

AnglePyramid::~AnglePyramid() = default;

AnglePyramid::AnglePyramid(const AnglePyramid& other)
    : impl_(std::make_unique<Impl>(*other.impl_)) {}

AnglePyramid::AnglePyramid(AnglePyramid&& other) noexcept = default;

AnglePyramid& AnglePyramid::operator=(const AnglePyramid& other) {
    if (this != &other) {
        impl_ = std::make_unique<Impl>(*other.impl_);
    }
    return *this;
}

AnglePyramid& AnglePyramid::operator=(AnglePyramid&& other) noexcept = default;

bool AnglePyramid::Build(const QImage& image, const AnglePyramidParams& params) {
    using Clock = std::chrono::high_resolution_clock;
    auto totalStart = Clock::now();
    auto t0 = totalStart, t1 = totalStart;

    Clear();

    if (image.Empty()) {
        return false;
    }

    // Reset timing
    impl_->timing_ = AnglePyramidTiming();

    // Validate parameters
    impl_->params_ = params;
    impl_->params_.numLevels = std::clamp(params.numLevels, 1, ANGLE_PYRAMID_MAX_LEVELS);
    impl_->params_.angleBins = std::clamp(params.angleBins, MIN_ANGLE_BINS, MAX_ANGLE_BINS);

    impl_->originalWidth_ = image.Width();
    impl_->originalHeight_ = image.Height();

    // Convert to grayscale if needed
    QImage grayImage = image;
    if (image.Channels() > 1) {
        // TODO: Convert to grayscale
        return false;
    }

    // Convert to float for processing (with OpenMP)
    t0 = Clock::now();
    QImage floatImage;
    if (image.Type() == PixelType::Float32) {
        floatImage = grayImage;
    } else {
        floatImage = QImage(grayImage.Width(), grayImage.Height(),
                            PixelType::Float32, ChannelType::Gray);
        const uint8_t* srcData = static_cast<const uint8_t*>(grayImage.Data());
        float* dstData = static_cast<float*>(floatImage.Data());
        int32_t srcStride = grayImage.Stride();
        int32_t dstStride = floatImage.Stride() / sizeof(float);
        int32_t h = grayImage.Height();
        int32_t w = grayImage.Width();

        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < h; ++y) {
            for (int32_t x = 0; x < w; ++x) {
                dstData[y * dstStride + x] = static_cast<float>(srcData[y * srcStride + x]);
            }
        }
    }
    t1 = Clock::now();
    if (params.enableTiming) {
        impl_->timing_.toFloatMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    }

    // Build Gaussian pyramid using the existing Pyramid module
    PyramidParams pyramidParams;
    pyramidParams.numLevels = impl_->params_.numLevels;
    pyramidParams.sigma = std::max(1.0, params.smoothSigma);
    pyramidParams.minDimension = 16;

    // Extract float data from the float QImage (handle stride correctly)
    int32_t floatWidth = floatImage.Width();
    int32_t floatHeight = floatImage.Height();
    int32_t floatStride = floatImage.Stride() / sizeof(float);
    const float* floatSrcData = static_cast<const float*>(floatImage.Data());

    std::vector<float> floatContiguous(floatWidth * floatHeight);

    #pragma omp parallel for schedule(static)
    for (int32_t y = 0; y < floatHeight; ++y) {
        for (int32_t x = 0; x < floatWidth; ++x) {
            floatContiguous[y * floatWidth + x] = floatSrcData[y * floatStride + x];
        }
    }

    // Use the float overload of BuildGaussianPyramid
    t0 = Clock::now();
    ImagePyramid gaussPyramid = BuildGaussianPyramid(floatContiguous.data(),
                                                      floatWidth, floatHeight, pyramidParams);
    t1 = Clock::now();
    if (params.enableTiming) {
        impl_->timing_.gaussPyramidMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    }

    // Update actual number of levels
    impl_->params_.numLevels = gaussPyramid.NumLevels();

    // Build angle pyramid for each level
    impl_->levels_.reserve(gaussPyramid.NumLevels());
    double scale = 1.0;

    for (int32_t level = 0; level < gaussPyramid.NumLevels(); ++level) {
        const auto& pyramidLevel = gaussPyramid.GetLevel(level);
        if (!impl_->BuildLevel(pyramidLevel.data, pyramidLevel.width, pyramidLevel.height,
                               level, scale)) {
            Clear();
            return false;
        }
        scale *= 0.5;
    }

    // Extract edge points for each level
    t0 = Clock::now();
    for (int32_t level = 0; level < static_cast<int32_t>(impl_->levels_.size()); ++level) {
        impl_->ExtractEdgePointsForLevel(level);
    }
    t1 = Clock::now();
    if (params.enableTiming) {
        impl_->timing_.extractEdgeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    }

    impl_->valid_ = true;

    // Calculate total time
    if (params.enableTiming) {
        impl_->timing_.totalMs = std::chrono::duration<double, std::milli>(Clock::now() - totalStart).count();
    }

    return true;
}

void AnglePyramid::Clear() {
    impl_->levels_.clear();
    impl_->originalWidth_ = 0;
    impl_->originalHeight_ = 0;
    impl_->valid_ = false;
}

bool AnglePyramid::IsValid() const {
    return impl_->valid_;
}

int32_t AnglePyramid::NumLevels() const {
    return static_cast<int32_t>(impl_->levels_.size());
}

int32_t AnglePyramid::AngleBins() const {
    return impl_->params_.angleBins;
}

int32_t AnglePyramid::OriginalWidth() const {
    return impl_->originalWidth_;
}

int32_t AnglePyramid::OriginalHeight() const {
    return impl_->originalHeight_;
}

const AnglePyramidParams& AnglePyramid::GetParams() const {
    return impl_->params_;
}

const AnglePyramidTiming& AnglePyramid::GetTiming() const {
    return impl_->timing_;
}

const PyramidLevelData& AnglePyramid::GetLevel(int32_t level) const {
    if (level < 0 || level >= static_cast<int32_t>(impl_->levels_.size())) {
        throw std::out_of_range("Pyramid level out of range");
    }
    return impl_->levels_[level];
}

int32_t AnglePyramid::GetWidth(int32_t level) const {
    if (level < 0 || level >= static_cast<int32_t>(impl_->levels_.size())) {
        return 0;
    }
    return impl_->levels_[level].width;
}

int32_t AnglePyramid::GetHeight(int32_t level) const {
    if (level < 0 || level >= static_cast<int32_t>(impl_->levels_.size())) {
        return 0;
    }
    return impl_->levels_[level].height;
}

double AnglePyramid::GetScale(int32_t level) const {
    if (level < 0 || level >= static_cast<int32_t>(impl_->levels_.size())) {
        return 0.0;
    }
    return impl_->levels_[level].scale;
}

double AnglePyramid::GetAngleAt(int32_t level, double x, double y) const {
    if (level < 0 || level >= static_cast<int32_t>(impl_->levels_.size())) {
        return -1.0;
    }

    const auto& levelData = impl_->levels_[level];

    if (x < 0 || x >= levelData.width - 1 || y < 0 || y >= levelData.height - 1) {
        return -1.0;
    }

    // Bilinear interpolation of angle (careful with wrap-around)
    int32_t x0 = static_cast<int32_t>(x);
    int32_t y0 = static_cast<int32_t>(y);
    double fx = x - x0;
    double fy = y - y0;

    const float* dirData = static_cast<const float*>(levelData.gradDir.Data());
    int32_t stride = levelData.gradDir.Stride() / sizeof(float);

    // Get four corner angles
    double a00 = dirData[y0 * stride + x0];
    double a10 = dirData[y0 * stride + x0 + 1];
    double a01 = dirData[(y0 + 1) * stride + x0];
    double a11 = dirData[(y0 + 1) * stride + x0 + 1];

    // Handle angle wrap-around: convert to unit vectors and interpolate
    double cx = (1 - fx) * (1 - fy) * std::cos(a00) +
                fx * (1 - fy) * std::cos(a10) +
                (1 - fx) * fy * std::cos(a01) +
                fx * fy * std::cos(a11);

    double cy = (1 - fx) * (1 - fy) * std::sin(a00) +
                fx * (1 - fy) * std::sin(a10) +
                (1 - fx) * fy * std::sin(a01) +
                fx * fy * std::sin(a11);

    return NormalizeAngle0To2PI(std::atan2(cy, cx));
}

double AnglePyramid::GetMagnitudeAt(int32_t level, double x, double y) const {
    if (level < 0 || level >= static_cast<int32_t>(impl_->levels_.size())) {
        return 0.0;
    }

    const auto& levelData = impl_->levels_[level];

    if (x < 0 || x >= levelData.width - 1 || y < 0 || y >= levelData.height - 1) {
        return 0.0;
    }

    // Bilinear interpolation
    int32_t x0 = static_cast<int32_t>(x);
    int32_t y0 = static_cast<int32_t>(y);
    double fx = x - x0;
    double fy = y - y0;

    const float* magData = static_cast<const float*>(levelData.gradMag.Data());
    int32_t stride = levelData.gradMag.Stride() / sizeof(float);

    double m00 = magData[y0 * stride + x0];
    double m10 = magData[y0 * stride + x0 + 1];
    double m01 = magData[(y0 + 1) * stride + x0];
    double m11 = magData[(y0 + 1) * stride + x0 + 1];

    return (1 - fx) * (1 - fy) * m00 +
           fx * (1 - fy) * m10 +
           (1 - fx) * fy * m01 +
           fx * fy * m11;
}

int32_t AnglePyramid::GetAngleBinAt(int32_t level, int32_t x, int32_t y) const {
    if (level < 0 || level >= static_cast<int32_t>(impl_->levels_.size())) {
        return -1;
    }

    const auto& levelData = impl_->levels_[level];

    if (x < 0 || x >= levelData.width || y < 0 || y >= levelData.height) {
        return -1;
    }

    const int16_t* binData = static_cast<const int16_t*>(levelData.angleBinImage.Data());
    int32_t stride = levelData.angleBinImage.Stride() / sizeof(int16_t);

    return binData[y * stride + x];
}

bool AnglePyramid::GetGradientAt(int32_t level, double x, double y,
                                  double& gx, double& gy) const {
    if (level < 0 || level >= static_cast<int32_t>(impl_->levels_.size())) {
        return false;
    }

    const auto& levelData = impl_->levels_[level];

    if (x < 0 || x >= levelData.width - 1 || y < 0 || y >= levelData.height - 1) {
        return false;
    }

    // Bilinear interpolation
    int32_t x0 = static_cast<int32_t>(x);
    int32_t y0 = static_cast<int32_t>(y);
    double fx = x - x0;
    double fy = y - y0;

    const float* gxData = static_cast<const float*>(levelData.gradX.Data());
    const float* gyData = static_cast<const float*>(levelData.gradY.Data());
    int32_t strideX = levelData.gradX.Stride() / sizeof(float);
    int32_t strideY = levelData.gradY.Stride() / sizeof(float);

    gx = (1 - fx) * (1 - fy) * gxData[y0 * strideX + x0] +
         fx * (1 - fy) * gxData[y0 * strideX + x0 + 1] +
         (1 - fx) * fy * gxData[(y0 + 1) * strideX + x0] +
         fx * fy * gxData[(y0 + 1) * strideX + x0 + 1];

    gy = (1 - fx) * (1 - fy) * gyData[y0 * strideY + x0] +
         fx * (1 - fy) * gyData[y0 * strideY + x0 + 1] +
         (1 - fx) * fy * gyData[(y0 + 1) * strideY + x0] +
         fx * fy * gyData[(y0 + 1) * strideY + x0 + 1];

    return true;
}

bool AnglePyramid::GetGradientAtFast(int32_t level, int32_t x, int32_t y,
                                      float& gx, float& gy) const {
    if (level < 0 || level >= static_cast<int32_t>(impl_->levels_.size())) {
        return false;
    }

    const auto& levelData = impl_->levels_[level];

    if (x < 0 || x >= levelData.width || y < 0 || y >= levelData.height) {
        return false;
    }

    const float* gxData = static_cast<const float*>(levelData.gradX.Data());
    const float* gyData = static_cast<const float*>(levelData.gradY.Data());
    int32_t stride = levelData.gradX.Stride() / sizeof(float);

    gx = gxData[y * stride + x];
    gy = gyData[y * stride + x];

    return true;
}

bool AnglePyramid::GetGradientData(int32_t level, const float*& gxData, const float*& gyData,
                                    int32_t& width, int32_t& height, int32_t& stride) const {
    if (level < 0 || level >= static_cast<int32_t>(impl_->levels_.size())) {
        return false;
    }

    const auto& levelData = impl_->levels_[level];
    gxData = static_cast<const float*>(levelData.gradX.Data());
    gyData = static_cast<const float*>(levelData.gradY.Data());
    width = levelData.width;
    height = levelData.height;
    stride = levelData.gradX.Stride() / sizeof(float);

    return true;
}

std::vector<EdgePoint> AnglePyramid::ExtractEdgePoints(int32_t level,
                                                        const Rect2i& roi,
                                                        double minContrast) const {
    std::vector<EdgePoint> result;

    if (level < 0 || level >= static_cast<int32_t>(impl_->levels_.size())) {
        return result;
    }

    const auto& levelData = impl_->levels_[level];

    // Determine ROI
    int32_t x0 = 1, y0 = 1;
    int32_t x1 = levelData.width - 1;
    int32_t y1 = levelData.height - 1;

    if (roi.width > 0 && roi.height > 0) {
        x0 = std::max(1, roi.x);
        y0 = std::max(1, roi.y);
        x1 = std::min(levelData.width - 1, roi.x + roi.width);
        y1 = std::min(levelData.height - 1, roi.y + roi.height);
    }

    double threshold = (minContrast > 0) ? minContrast : impl_->params_.minContrast;

    const float* magData = static_cast<const float*>(levelData.gradMag.Data());
    const float* dirData = static_cast<const float*>(levelData.gradDir.Data());
    const int16_t* binData = static_cast<const int16_t*>(levelData.angleBinImage.Data());

    int32_t magStride = levelData.gradMag.Stride() / sizeof(float);
    int32_t dirStride = levelData.gradDir.Stride() / sizeof(float);
    int32_t binStride = levelData.angleBinImage.Stride() / sizeof(int16_t);

    // Use OpenMP with thread-local vectors
    #pragma omp parallel
    {
        std::vector<EdgePoint> localPoints;
        localPoints.reserve(1000);

        #pragma omp for schedule(static) nowait
        for (int32_t y = y0; y < y1; ++y) {
            for (int32_t x = x0; x < x1; ++x) {
                float mag = magData[y * magStride + x];

                if (mag >= threshold) {
                    float dir = dirData[y * dirStride + x];
                    int16_t bin = binData[y * binStride + x];

                    localPoints.emplace_back(
                        static_cast<double>(x),
                        static_cast<double>(y),
                        static_cast<double>(dir),
                        static_cast<double>(mag),
                        static_cast<int32_t>(bin)
                    );
                }
            }
        }

        #pragma omp critical
        {
            result.insert(result.end(), localPoints.begin(), localPoints.end());
        }
    }

    return result;
}

const std::vector<EdgePoint>& AnglePyramid::GetEdgePoints(int32_t level) const {
    static const std::vector<EdgePoint> empty;
    if (level < 0 || level >= static_cast<int32_t>(impl_->levels_.size())) {
        return empty;
    }
    return impl_->levels_[level].edgePoints;
}

Point2d AnglePyramid::ToLevelCoords(int32_t level, const Point2d& original) const {
    double scale = GetScale(level);
    if (scale <= 0) return original;
    return Point2d{original.x * scale, original.y * scale};
}

Point2d AnglePyramid::ToOriginalCoords(int32_t level, const Point2d& levelCoords) const {
    double scale = GetScale(level);
    if (scale <= 0) return levelCoords;
    return Point2d{levelCoords.x / scale, levelCoords.y / scale};
}

int32_t AnglePyramid::AngleToBin(double angle) const {
    double normalized = NormalizeAngle0To2PI(angle);
    int32_t bin = static_cast<int32_t>(normalized * impl_->params_.angleBins / (2.0 * PI));
    return std::clamp(bin, 0, impl_->params_.angleBins - 1);
}

double AnglePyramid::BinToAngle(int32_t bin) const {
    return (bin + 0.5) * 2.0 * PI / impl_->params_.angleBins;
}

double AnglePyramid::AngleDifference(double angle1, double angle2) {
    double diff = std::abs(angle1 - angle2);
    if (diff > PI) {
        diff = 2.0 * PI - diff;
    }
    return diff;
}

double AnglePyramid::AngleSimilarity(double angle1, double angle2) {
    return std::cos(angle1 - angle2);
}

// =============================================================================
// Utility Functions (also optimized)
// =============================================================================

QImage ComputeGradientDirection(const QImage& gradX, const QImage& gradY) {
    if (gradX.Empty() || gradY.Empty()) {
        return QImage();
    }

    if (gradX.Width() != gradY.Width() || gradX.Height() != gradY.Height()) {
        return QImage();
    }

    int32_t width = gradX.Width();
    int32_t height = gradX.Height();

    QImage result(width, height, PixelType::Float32, ChannelType::Gray);

    const float* gxData = static_cast<const float*>(gradX.Data());
    const float* gyData = static_cast<const float*>(gradY.Data());
    float* outData = static_cast<float*>(result.Data());

    int32_t gxStride = gradX.Stride() / sizeof(float);
    int32_t gyStride = gradY.Stride() / sizeof(float);
    int32_t outStride = result.Stride() / sizeof(float);

    const float twoPi = static_cast<float>(2.0 * PI);

    #pragma omp parallel for schedule(static)
    for (int32_t y = 0; y < height; ++y) {
        for (int32_t x = 0; x < width; ++x) {
            float gx = gxData[y * gxStride + x];
            float gy = gyData[y * gyStride + x];
            float angle = std::atan2(gy, gx);
            if (angle < 0) angle += twoPi;
            outData[y * outStride + x] = angle;
        }
    }

    return result;
}

QImage ComputeGradientMagnitude(const QImage& gradX, const QImage& gradY) {
    if (gradX.Empty() || gradY.Empty()) {
        return QImage();
    }

    if (gradX.Width() != gradY.Width() || gradX.Height() != gradY.Height()) {
        return QImage();
    }

    int32_t width = gradX.Width();
    int32_t height = gradX.Height();

    QImage result(width, height, PixelType::Float32, ChannelType::Gray);

    const float* gxData = static_cast<const float*>(gradX.Data());
    const float* gyData = static_cast<const float*>(gradY.Data());
    float* outData = static_cast<float*>(result.Data());

    int32_t gxStride = gradX.Stride() / sizeof(float);
    int32_t gyStride = gradY.Stride() / sizeof(float);
    int32_t outStride = result.Stride() / sizeof(float);

#ifdef __AVX2__
    #pragma omp parallel for schedule(static)
    for (int32_t y = 0; y < height; ++y) {
        const float* gxRow = gxData + y * gxStride;
        const float* gyRow = gyData + y * gyStride;
        float* outRow = outData + y * outStride;

        int32_t x = 0;
        for (; x + 8 <= width; x += 8) {
            __m256 vgx = _mm256_loadu_ps(gxRow + x);
            __m256 vgy = _mm256_loadu_ps(gyRow + x);
            __m256 gx2 = _mm256_mul_ps(vgx, vgx);
            __m256 gy2 = _mm256_mul_ps(vgy, vgy);
            __m256 mag = _mm256_sqrt_ps(_mm256_add_ps(gx2, gy2));
            _mm256_storeu_ps(outRow + x, mag);
        }
        for (; x < width; ++x) {
            float gx = gxRow[x];
            float gy = gyRow[x];
            outRow[x] = std::sqrt(gx * gx + gy * gy);
        }
    }
#else
    #pragma omp parallel for schedule(static)
    for (int32_t y = 0; y < height; ++y) {
        for (int32_t x = 0; x < width; ++x) {
            float gx = gxData[y * gxStride + x];
            float gy = gyData[y * gyStride + x];
            outData[y * outStride + x] = std::sqrt(gx * gx + gy * gy);
        }
    }
#endif

    return result;
}

QImage QuantizeGradientDirection(const QImage& gradDir, int32_t numBins) {
    if (gradDir.Empty() || numBins <= 0) {
        return QImage();
    }

    int32_t width = gradDir.Width();
    int32_t height = gradDir.Height();

    QImage result(width, height, PixelType::Int16, ChannelType::Gray);

    const float* dirData = static_cast<const float*>(gradDir.Data());
    int16_t* outData = static_cast<int16_t*>(result.Data());

    int32_t dirStride = gradDir.Stride() / sizeof(float);
    int32_t outStride = result.Stride() / sizeof(int16_t);

    double binScale = numBins / (2.0 * PI);

    #pragma omp parallel for schedule(static)
    for (int32_t y = 0; y < height; ++y) {
        for (int32_t x = 0; x < width; ++x) {
            float angle = dirData[y * dirStride + x];
            int32_t bin = static_cast<int32_t>(angle * binScale);
            bin = std::clamp(bin, 0, numBins - 1);
            outData[y * outStride + x] = static_cast<int16_t>(bin);
        }
    }

    return result;
}

int32_t ComputeOptimalPyramidLevels(int32_t width, int32_t height, int32_t minSize) {
    int32_t minDim = std::min(width, height);
    int32_t levels = 1;

    while (minDim >= minSize * 2) {
        minDim /= 2;
        levels++;
    }

    return std::min(levels, ANGLE_PYRAMID_MAX_LEVELS);
}

} // namespace Qi::Vision::Internal
