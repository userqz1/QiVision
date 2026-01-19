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
#include <QiVision/Internal/NonMaxSuppression.h>
#include <QiVision/Core/Constants.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef __AVX2__
#include <immintrin.h>
#endif

namespace Qi::Vision::Internal {

// =============================================================================
// OpenMP 初始化
// =============================================================================

#ifdef _OPENMP
namespace {
/// 自动设置 OpenMP 线程数（只执行一次）
struct OpenMPInitializer {
    OpenMPInitializer() {
        // 获取可用核心数，使用一半（避免超线程开销）
        int numProcs = omp_get_num_procs();
        int optimalThreads = std::max(1, std::min(numProcs / 2, 8));
        omp_set_num_threads(optimalThreads);
    }
};
static OpenMPInitializer g_ompInit;
}
#endif

// =============================================================================
// OpenMP 阈值控制
// =============================================================================

/// 启用 OpenMP 的最小像素数阈值 (约 700×700)
constexpr int32_t OPENMP_MIN_PIXELS = 500000;

/// 判断是否使用 OpenMP 并行
inline bool ShouldUseOpenMP(int32_t width, int32_t height) {
    return static_cast<int64_t>(width) * height >= OPENMP_MIN_PIXELS;
}

// =============================================================================
// SIMD Helper Functions
// =============================================================================

#ifdef __AVX2__

// Fast reciprocal using rcp_ps + Newton-Raphson iteration
// More accurate than raw rcp_ps (~0.0003% error vs ~0.4% error)
// Faster than _mm256_div_ps by ~2-4x
static inline __m256 fast_rcp_avx2(__m256 x) {
    // Initial estimate: rcp_ps has ~12 bits of precision
    __m256 rcp = _mm256_rcp_ps(x);

    // Newton-Raphson iteration: rcp' = rcp * (2 - x * rcp)
    // This gives ~23 bits of precision (enough for float)
    __m256 two = _mm256_set1_ps(2.0f);
    __m256 x_rcp = _mm256_mul_ps(x, rcp);
    rcp = _mm256_mul_ps(rcp, _mm256_sub_ps(two, x_rcp));

    return rcp;
}

// Fast division: a / b = a * rcp(b) with Newton-Raphson
static inline __m256 fast_div_avx2(__m256 a, __m256 b) {
    return _mm256_mul_ps(a, fast_rcp_avx2(b));
}

// Fast bin quantization: directly compute bin index from gx/gy without full atan2
// Uses octant-based approach with linear interpolation within each octant
// For numBins=16: each octant has 2 bins, total 8 octants
static inline __m256i fast_quantize_bin_avx2(__m256 gy, __m256 gx, int32_t numBins) {
    const __m256 zero = _mm256_setzero_ps();
    const __m256 one = _mm256_set1_ps(1.0f);
    const __m256 eps = _mm256_set1_ps(1e-10f);

    // Compute absolute values
    const __m256 sign_mask = _mm256_set1_ps(-0.0f);
    __m256 abs_gx = _mm256_andnot_ps(sign_mask, gx);
    __m256 abs_gy = _mm256_andnot_ps(sign_mask, gy);

    // Determine octant using sign bits and |gy| vs |gx|
    // octant: 0-7 based on (sign_x, sign_y, |gy|>|gx|)
    __m256 gx_neg = _mm256_cmp_ps(gx, zero, _CMP_LT_OQ);
    __m256 gy_neg = _mm256_cmp_ps(gy, zero, _CMP_LT_OQ);
    __m256 gy_gt_gx = _mm256_cmp_ps(abs_gy, abs_gx, _CMP_GT_OQ);

    // Convert masks to integers (0 or 1)
    __m256i gx_neg_i = _mm256_and_si256(_mm256_castps_si256(gx_neg), _mm256_set1_epi32(1));
    __m256i gy_neg_i = _mm256_and_si256(_mm256_castps_si256(gy_neg), _mm256_set1_epi32(1));
    __m256i gy_gt_gx_i = _mm256_and_si256(_mm256_castps_si256(gy_gt_gx), _mm256_set1_epi32(1));

    // Compute ratio t = min(|gy|,|gx|) / max(|gy|,|gx|) in [0,1]
    __m256 num = _mm256_min_ps(abs_gy, abs_gx);
    __m256 den = _mm256_max_ps(abs_gy, abs_gx);
    den = _mm256_max_ps(den, eps);  // Avoid division by zero
    __m256 t = fast_div_avx2(num, den);  // rcp + Newton-Raphson

    // Fast atan approximation for t in [0,1]: atan(t) ≈ t * (π/4) * (1 + 0.273*(1-t))
    // Simplified: atan(t)/（π/4) ≈ t * (1.273 - 0.273*t) = t * 1.273 - t*t*0.273
    // This gives a value in [0, 1] representing fraction of 45 degrees
    const __m256 c1 = _mm256_set1_ps(1.273f);
    const __m256 c2 = _mm256_set1_ps(0.273f);
    __m256 t2 = _mm256_mul_ps(t, t);
    __m256 frac = _mm256_sub_ps(_mm256_mul_ps(t, c1), _mm256_mul_ps(t2, c2));
    frac = _mm256_min_ps(frac, one);  // Clamp to [0, 1]

    // Bins per octant
    float binsPerOctant = static_cast<float>(numBins) / 8.0f;
    __m256 vBinsPerOctant = _mm256_set1_ps(binsPerOctant);

    // Sub-bin within octant
    __m256 subBinF = _mm256_mul_ps(frac, vBinsPerOctant);
    __m256i subBin = _mm256_cvttps_epi32(subBinF);

    // Compute octant index based on signs and |gy|>|gx|
    // Octant layout (angle increases counter-clockwise from +x axis):
    // Oct 0: gx>0, gy>=0, |gx|>=|gy| -> angle [0, 45)
    // Oct 1: gx>0, gy>0, |gy|>|gx|   -> angle [45, 90)
    // Oct 2: gx<=0, gy>0, |gy|>|gx|  -> angle [90, 135)
    // Oct 3: gx<0, gy>=0, |gx|>=|gy| -> angle [135, 180)
    // Oct 4: gx<0, gy<0, |gx|>=|gy|  -> angle [180, 225)
    // Oct 5: gx<0, gy<0, |gy|>|gx|   -> angle [225, 270)
    // Oct 6: gx>=0, gy<0, |gy|>|gx|  -> angle [270, 315)
    // Oct 7: gx>0, gy<0, |gx|>=|gy|  -> angle [315, 360)

    // Simplified octant calculation using bit manipulation
    // octant = (gx<0)*4 + (gy<0)*2 + (|gy|>|gx|)*1, then map to actual octant
    __m256i raw_oct = _mm256_add_epi32(
        _mm256_add_epi32(
            _mm256_slli_epi32(gx_neg_i, 2),
            _mm256_slli_epi32(gy_neg_i, 1)
        ),
        gy_gt_gx_i
    );

    // Map raw_oct to actual octant (handles the non-linear mapping)
    // raw: 0->0, 1->1, 2->7, 3->6, 4->3, 5->2, 6->4, 7->5
    // Use lookup: stored as float for blending
    alignas(32) static const int32_t octant_map[8] = {0, 1, 7, 6, 3, 2, 4, 5};

    // Scalar lookup (AVX2 doesn't have good gather for small tables)
    alignas(32) int32_t raw_oct_arr[8];
    alignas(32) int32_t octant_arr[8];
    _mm256_store_si256(reinterpret_cast<__m256i*>(raw_oct_arr), raw_oct);
    for (int i = 0; i < 8; ++i) {
        octant_arr[i] = octant_map[raw_oct_arr[i]];
    }
    __m256i octant = _mm256_load_si256(reinterpret_cast<const __m256i*>(octant_arr));

    // Final bin = octant * binsPerOctant + subBin
    __m256i binsPerOctantI = _mm256_set1_epi32(static_cast<int32_t>(binsPerOctant));
    __m256i baseBin = _mm256_mullo_epi32(octant, binsPerOctantI);

    // Handle octant direction: even octants go forward, odd octants go backward
    // For odd octants: subBin = binsPerOctant - 1 - subBin
    __m256i octant_odd = _mm256_and_si256(octant, _mm256_set1_epi32(1));
    __m256i odd_mask = _mm256_cmpeq_epi32(octant_odd, _mm256_set1_epi32(1));
    __m256i subBin_rev = _mm256_sub_epi32(_mm256_sub_epi32(binsPerOctantI, _mm256_set1_epi32(1)), subBin);
    subBin = _mm256_blendv_epi8(subBin, subBin_rev, odd_mask);

    __m256i bin = _mm256_add_epi32(baseBin, subBin);

    // Clamp to valid range
    bin = _mm256_max_epi32(bin, _mm256_setzero_si256());
    bin = _mm256_min_epi32(bin, _mm256_set1_epi32(numBins - 1));

    return bin;
}

// Fast atan2 approximation for 8 float values
// Using polynomial approximation, accurate to ~0.01 radians
static inline __m256 atan2_avx2(__m256 y, __m256 x) {
    // Constants
    const __m256 pi = _mm256_set1_ps(static_cast<float>(PI));
    const __m256 pi_2 = _mm256_set1_ps(static_cast<float>(PI / 2.0));
    const __m256 zero = _mm256_setzero_ps();

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
    __m256 t = fast_div_avx2(num, den);  // rcp + Newton-Raphson
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
    bool BuildLevelFused(const std::vector<float>& srcData, int32_t width, int32_t height,
                         int32_t level, double scale);
    void ExtractEdgePointsForLevel(int32_t level);

    // Optimized functions
    void ComputeGradientMagnitudeDirectionOpt(const float* gx, const float* gy,
                                               float* mag, float* dir,
                                               int32_t width, int32_t height,
                                               int32_t gxStride, int32_t gyStride,
                                               int32_t magStride, int32_t dirStride);

    void QuantizeDirectionOpt(const float* dir, int16_t* bins,
                              int32_t width, int32_t height,
                              int32_t dirStride, int32_t binStride,
                              int32_t numBins);

    // Fused: Sobel + Mag + Dir + Quantize in one pass
    void FusedSobelMagDirBin(const float* src, int32_t width, int32_t height,
                              float* gx, float* gy, float* mag, float* dir, int16_t* bins,
                              int32_t numBins);

    // Optimized: Compute Gx/Gy + Mag + Bins (skip Dir for search mode)
    void FusedSobelGradMagBin(const float* src, int32_t width, int32_t height,
                               float* gx, float* gy, float* mag, int16_t* bins,
                               int32_t numBins);

    // Direct write versions: write directly to QImage with stride (no copy)
    void FusedSobelGradMagBinDirect(const float* src, int32_t width, int32_t height,
                                    float* gx, int32_t gxStride, float* gy, int32_t gyStride,
                                    float* mag, int32_t magStride, int16_t* bins, int32_t binStride,
                                    int32_t numBins);

    void FusedSobelMagDirBinDirect(const float* src, int32_t width, int32_t height,
                                   float* gx, int32_t gxStride, float* gy, int32_t gyStride,
                                   float* mag, int32_t magStride, float* dir, int32_t dirStride,
                                   int16_t* bins, int32_t binStride, int32_t numBins);
};

void AnglePyramid::Impl::ComputeGradientMagnitudeDirectionOpt(
    const float* gx, const float* gy,
    float* mag, float* dir,
    int32_t width, int32_t height,
    int32_t gxStride, int32_t gyStride,
    int32_t magStride, int32_t dirStride)
{
    const bool useParallel = ShouldUseOpenMP(width, height);

#ifdef __AVX2__
    const float twoPi = static_cast<float>(2.0 * PI);

    auto processRow = [&](int32_t y) {
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
    };

#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < height; ++y) {
            processRow(y);
        }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 0; y < height; ++y) {
            processRow(y);
        }
    }
#else
    // Non-SIMD version
    const float twoPi = static_cast<float>(2.0 * PI);

    auto processRow = [&](int32_t y) {
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
    };

#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < height; ++y) {
            processRow(y);
        }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 0; y < height; ++y) {
            processRow(y);
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
    const bool useParallel = ShouldUseOpenMP(width, height);

#ifdef __AVX2__
    const __m256 vBinScale = _mm256_set1_ps(binScale);
    const __m256i vMaxBin = _mm256_set1_epi32(numBins - 1);
    const __m256i vZero = _mm256_setzero_si256();

    auto processRow = [&](int32_t y) {
        const float* dirRow = dir + y * dirStride;
        int16_t* binRow = bins + y * binStride;

        int32_t x = 0;
        // AVX2: process 8 pixels at a time
        for (; x + 8 <= width; x += 8) {
            __m256 vdir = _mm256_loadu_ps(dirRow + x);
            __m256 vbin_f = _mm256_mul_ps(vdir, vBinScale);
            __m256i vbin = _mm256_cvttps_epi32(vbin_f);

            // Clamp to [0, numBins-1]
            vbin = _mm256_max_epi32(vbin, vZero);
            vbin = _mm256_min_epi32(vbin, vMaxBin);

            // Pack 32-bit integers to 16-bit
            __m128i lo = _mm256_castsi256_si128(vbin);
            __m128i hi = _mm256_extracti128_si256(vbin, 1);
            __m128i packed = _mm_packs_epi32(lo, hi);

            _mm_storeu_si128(reinterpret_cast<__m128i*>(binRow + x), packed);
        }

        // Scalar fallback
        for (; x < width; ++x) {
            int32_t bin = static_cast<int32_t>(dirRow[x] * binScale);
            binRow[x] = static_cast<int16_t>(std::clamp(bin, 0, static_cast<int32_t>(maxBin)));
        }
    };

#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < height; ++y) {
            processRow(y);
        }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 0; y < height; ++y) {
            processRow(y);
        }
    }
#else
    auto processRow = [&](int32_t y) {
        const float* dirRow = dir + y * dirStride;
        int16_t* binRow = bins + y * binStride;

        for (int32_t x = 0; x < width; ++x) {
            int32_t bin = static_cast<int32_t>(dirRow[x] * binScale);
            binRow[x] = static_cast<int16_t>(std::clamp(bin, 0, static_cast<int32_t>(maxBin)));
        }
    };

#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < height; ++y) {
            processRow(y);
        }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 0; y < height; ++y) {
            processRow(y);
        }
    }
#endif
}

// =============================================================================
// Fused Sobel + Magnitude + Direction + Quantize (single pass)
// =============================================================================

void AnglePyramid::Impl::FusedSobelMagDirBin(
    const float* src, int32_t width, int32_t height,
    float* gx, float* gy, float* mag, float* dir, int16_t* bins,
    int32_t numBins)
{
    const float twoPi = static_cast<float>(2.0 * PI);
    const float binScale = static_cast<float>(numBins) / twoPi;
    const int16_t maxBin = static_cast<int16_t>(numBins - 1);
    const bool useParallel = ShouldUseOpenMP(width, height);

    // Sobel 3x3 kernel weights: [-1,0,1], [1,2,1]^T for Gx
    // For center pixels (y=1..height-2, x=1..width-2)

#ifdef __AVX2__
    const __m256 vBinScale = _mm256_set1_ps(binScale);
    const __m256i vMaxBin = _mm256_set1_epi32(numBins - 1);
    const __m256i vZero = _mm256_setzero_si256();

    auto processRow = [&](int32_t y) {
        const float* row0 = src + (y - 1) * width;
        const float* row1 = src + y * width;
        const float* row2 = src + (y + 1) * width;

        float* gxRow = gx + y * width;
        float* gyRow = gy + y * width;
        float* magRow = mag + y * width;
        float* dirRow = dir + y * width;
        int16_t* binRow = bins + y * width;

        // Border: x=0
        gxRow[0] = 0; gyRow[0] = 0; magRow[0] = 0; dirRow[0] = 0; binRow[0] = 0;

        int32_t x = 1;

        // AVX2 vectorized: process 8 pixels at a time
        for (; x + 8 < width; x += 8) {
            // Load 10 values for Sobel (need x-1 to x+8)
            // Gx = (p02 - p00) + 2*(p12 - p10) + (p22 - p20)
            // Gy = (p20 - p00) + 2*(p21 - p01) + (p22 - p02)

            __m256 p00 = _mm256_loadu_ps(row0 + x - 1);
            __m256 p01 = _mm256_loadu_ps(row0 + x);
            __m256 p02 = _mm256_loadu_ps(row0 + x + 1);

            __m256 p10 = _mm256_loadu_ps(row1 + x - 1);
            __m256 p12 = _mm256_loadu_ps(row1 + x + 1);

            __m256 p20 = _mm256_loadu_ps(row2 + x - 1);
            __m256 p21 = _mm256_loadu_ps(row2 + x);
            __m256 p22 = _mm256_loadu_ps(row2 + x + 1);

            // Gx = (p02 - p00) + 2*(p12 - p10) + (p22 - p20)
            __m256 diff_top = _mm256_sub_ps(p02, p00);
            __m256 diff_mid = _mm256_sub_ps(p12, p10);
            __m256 diff_bot = _mm256_sub_ps(p22, p20);
            __m256 vGx = _mm256_add_ps(diff_top, _mm256_add_ps(_mm256_add_ps(diff_mid, diff_mid), diff_bot));

            // Gy = (p20 - p00) + 2*(p21 - p01) + (p22 - p02)
            __m256 vert_left = _mm256_sub_ps(p20, p00);
            __m256 vert_mid = _mm256_sub_ps(p21, p01);
            __m256 vert_right = _mm256_sub_ps(p22, p02);
            __m256 vGy = _mm256_add_ps(vert_left, _mm256_add_ps(_mm256_add_ps(vert_mid, vert_mid), vert_right));

            // Store gradients
            _mm256_storeu_ps(gxRow + x, vGx);
            _mm256_storeu_ps(gyRow + x, vGy);

            // Magnitude = sqrt(gx^2 + gy^2)
            __m256 gx2 = _mm256_mul_ps(vGx, vGx);
            __m256 gy2 = _mm256_mul_ps(vGy, vGy);
            __m256 vMag = fast_sqrt_avx2(_mm256_add_ps(gx2, gy2));
            _mm256_storeu_ps(magRow + x, vMag);

            // Direction = atan2(gy, gx) in [0, 2π]
            __m256 vDir = atan2_avx2(vGy, vGx);
            _mm256_storeu_ps(dirRow + x, vDir);

            // Quantize to bins
            __m256 vBinF = _mm256_mul_ps(vDir, vBinScale);
            __m256i vBin = _mm256_cvttps_epi32(vBinF);
            vBin = _mm256_max_epi32(vBin, vZero);
            vBin = _mm256_min_epi32(vBin, vMaxBin);

            // Pack to int16
            __m128i lo = _mm256_castsi256_si128(vBin);
            __m128i hi = _mm256_extracti128_si256(vBin, 1);
            __m128i packed = _mm_packs_epi32(lo, hi);
            _mm_storeu_si128(reinterpret_cast<__m128i*>(binRow + x), packed);
        }

        // Scalar fallback for remaining pixels
        for (; x < width - 1; ++x) {
            float p00 = row0[x - 1], p01 = row0[x], p02 = row0[x + 1];
            float p10 = row1[x - 1],               p12 = row1[x + 1];
            float p20 = row2[x - 1], p21 = row2[x], p22 = row2[x + 1];

            float gxVal = (p02 - p00) + 2.0f * (p12 - p10) + (p22 - p20);
            float gyVal = (p20 - p00) + 2.0f * (p21 - p01) + (p22 - p02);

            gxRow[x] = gxVal;
            gyRow[x] = gyVal;
            magRow[x] = std::sqrt(gxVal * gxVal + gyVal * gyVal);

            float angle = std::atan2(gyVal, gxVal);
            if (angle < 0) angle += twoPi;
            dirRow[x] = angle;

            int32_t bin = static_cast<int32_t>(angle * binScale);
            binRow[x] = static_cast<int16_t>(std::clamp(bin, 0, static_cast<int32_t>(maxBin)));
        }

        // Border: x=width-1
        gxRow[width-1] = 0; gyRow[width-1] = 0; magRow[width-1] = 0;
        dirRow[width-1] = 0; binRow[width-1] = 0;
    };

#else
    // Scalar version
    auto processRow = [&](int32_t y) {
        const float* row0 = src + (y - 1) * width;
        const float* row1 = src + y * width;
        const float* row2 = src + (y + 1) * width;

        float* gxRow = gx + y * width;
        float* gyRow = gy + y * width;
        float* magRow = mag + y * width;
        float* dirRow = dir + y * width;
        int16_t* binRow = bins + y * width;

        // Border: x=0
        gxRow[0] = 0; gyRow[0] = 0; magRow[0] = 0; dirRow[0] = 0; binRow[0] = 0;

        for (int32_t x = 1; x < width - 1; ++x) {
            float p00 = row0[x - 1], p01 = row0[x], p02 = row0[x + 1];
            float p10 = row1[x - 1],               p12 = row1[x + 1];
            float p20 = row2[x - 1], p21 = row2[x], p22 = row2[x + 1];

            float gxVal = (p02 - p00) + 2.0f * (p12 - p10) + (p22 - p20);
            float gyVal = (p20 - p00) + 2.0f * (p21 - p01) + (p22 - p02);

            gxRow[x] = gxVal;
            gyRow[x] = gyVal;
            magRow[x] = std::sqrt(gxVal * gxVal + gyVal * gyVal);

            float angle = std::atan2(gyVal, gxVal);
            if (angle < 0) angle += twoPi;
            dirRow[x] = angle;

            int32_t bin = static_cast<int32_t>(angle * binScale);
            binRow[x] = static_cast<int16_t>(std::clamp(bin, 0, static_cast<int32_t>(maxBin)));
        }

        // Border: x=width-1
        gxRow[width-1] = 0; gyRow[width-1] = 0; magRow[width-1] = 0;
        dirRow[width-1] = 0; binRow[width-1] = 0;
    };
#endif

    // First row (y=0): set to zero
    std::memset(gx, 0, width * sizeof(float));
    std::memset(gy, 0, width * sizeof(float));
    std::memset(mag, 0, width * sizeof(float));
    std::memset(dir, 0, width * sizeof(float));
    std::memset(bins, 0, width * sizeof(int16_t));

    // Process interior rows
#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 1; y < height - 1; ++y) {
            processRow(y);
        }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 1; y < height - 1; ++y) {
            processRow(y);
        }
    }

    // Last row (y=height-1): set to zero
    int32_t lastRow = height - 1;
    std::memset(gx + lastRow * width, 0, width * sizeof(float));
    std::memset(gy + lastRow * width, 0, width * sizeof(float));
    std::memset(mag + lastRow * width, 0, width * sizeof(float));
    std::memset(dir + lastRow * width, 0, width * sizeof(float));
    std::memset(bins + lastRow * width, 0, width * sizeof(int16_t));
}

// =============================================================================
// FusedSobelGradMagBin: Optimized for search (gx/gy + mag + bins, skip dir)
// =============================================================================

void AnglePyramid::Impl::FusedSobelGradMagBin(
    const float* src, int32_t width, int32_t height,
    float* gx, float* gy, float* mag, int16_t* bins, int32_t numBins)
{
    const bool useParallel = ShouldUseOpenMP(width, height);

#ifdef __AVX2__
    auto processRow = [&](int32_t y) {
        const float* row0 = src + (y - 1) * width;
        const float* row1 = src + y * width;
        const float* row2 = src + (y + 1) * width;

        float* gxRow = gx + y * width;
        float* gyRow = gy + y * width;
        float* magRow = mag + y * width;
        int16_t* binRow = bins + y * width;

        // Border: x=0
        gxRow[0] = 0; gyRow[0] = 0; magRow[0] = 0; binRow[0] = 0;

        int32_t x = 1;

        // AVX2 vectorized: process 8 pixels at a time
        for (; x + 8 < width; x += 8) {
            __m256 p00 = _mm256_loadu_ps(row0 + x - 1);
            __m256 p01 = _mm256_loadu_ps(row0 + x);
            __m256 p02 = _mm256_loadu_ps(row0 + x + 1);

            __m256 p10 = _mm256_loadu_ps(row1 + x - 1);
            __m256 p12 = _mm256_loadu_ps(row1 + x + 1);

            __m256 p20 = _mm256_loadu_ps(row2 + x - 1);
            __m256 p21 = _mm256_loadu_ps(row2 + x);
            __m256 p22 = _mm256_loadu_ps(row2 + x + 1);

            // Gx = (p02 - p00) + 2*(p12 - p10) + (p22 - p20)
            __m256 diff_top = _mm256_sub_ps(p02, p00);
            __m256 diff_mid = _mm256_sub_ps(p12, p10);
            __m256 diff_bot = _mm256_sub_ps(p22, p20);
            __m256 vGx = _mm256_add_ps(diff_top, _mm256_add_ps(_mm256_add_ps(diff_mid, diff_mid), diff_bot));

            // Gy = (p20 - p00) + 2*(p21 - p01) + (p22 - p02)
            __m256 vert_left = _mm256_sub_ps(p20, p00);
            __m256 vert_mid = _mm256_sub_ps(p21, p01);
            __m256 vert_right = _mm256_sub_ps(p22, p02);
            __m256 vGy = _mm256_add_ps(vert_left, _mm256_add_ps(_mm256_add_ps(vert_mid, vert_mid), vert_right));

            // Store Gx/Gy
            _mm256_storeu_ps(gxRow + x, vGx);
            _mm256_storeu_ps(gyRow + x, vGy);

            // Magnitude = sqrt(gx^2 + gy^2)
            __m256 gx2 = _mm256_mul_ps(vGx, vGx);
            __m256 gy2 = _mm256_mul_ps(vGy, vGy);
            __m256 vMag = fast_sqrt_avx2(_mm256_add_ps(gx2, gy2));
            _mm256_storeu_ps(magRow + x, vMag);

            // Fast bin quantization: directly compute bin from gx/gy
            __m256i vBin = fast_quantize_bin_avx2(vGy, vGx, numBins);

            // Pack to int16
            __m128i lo = _mm256_castsi256_si128(vBin);
            __m128i hi = _mm256_extracti128_si256(vBin, 1);
            __m128i packed = _mm_packs_epi32(lo, hi);
            _mm_storeu_si128(reinterpret_cast<__m128i*>(binRow + x), packed);
        }

        // Scalar fallback for remaining pixels (use fast scalar version)
        for (; x < width - 1; ++x) {
            float p00 = row0[x - 1], p01 = row0[x], p02 = row0[x + 1];
            float p10 = row1[x - 1],               p12 = row1[x + 1];
            float p20 = row2[x - 1], p21 = row2[x], p22 = row2[x + 1];

            float gxVal = (p02 - p00) + 2.0f * (p12 - p10) + (p22 - p20);
            float gyVal = (p20 - p00) + 2.0f * (p21 - p01) + (p22 - p02);

            gxRow[x] = gxVal;
            gyRow[x] = gyVal;
            magRow[x] = std::sqrt(gxVal * gxVal + gyVal * gyVal);

            // Fast scalar bin quantization
            float abs_gx = std::abs(gxVal);
            float abs_gy = std::abs(gyVal);
            float den = std::max(abs_gx, abs_gy) + 1e-10f;
            float t = std::min(abs_gx, abs_gy) / den;
            float frac = t * (1.273f - 0.273f * t);
            frac = std::min(frac, 1.0f);

            int gx_neg = (gxVal < 0) ? 1 : 0;
            int gy_neg = (gyVal < 0) ? 1 : 0;
            int gy_gt_gx = (abs_gy > abs_gx) ? 1 : 0;
            int raw_oct = (gx_neg << 2) | (gy_neg << 1) | gy_gt_gx;
            static const int octant_map[8] = {0, 1, 7, 6, 3, 2, 4, 5};
            int octant = octant_map[raw_oct];

            int binsPerOctant = numBins / 8;
            int subBin = static_cast<int>(frac * binsPerOctant);
            if (octant & 1) {
                subBin = binsPerOctant - 1 - subBin;
            }
            int bin = octant * binsPerOctant + subBin;
            binRow[x] = static_cast<int16_t>(std::clamp(bin, 0, numBins - 1));
        }

        // Border: x=width-1
        gxRow[width-1] = 0; gyRow[width-1] = 0; magRow[width-1] = 0; binRow[width-1] = 0;
    };

#else
    // Non-AVX2 scalar version
    auto processRow = [&](int32_t y) {
        const float* row0 = src + (y - 1) * width;
        const float* row1 = src + y * width;
        const float* row2 = src + (y + 1) * width;

        float* gxRow = gx + y * width;
        float* gyRow = gy + y * width;
        float* magRow = mag + y * width;
        int16_t* binRow = bins + y * width;

        gxRow[0] = 0; gyRow[0] = 0; magRow[0] = 0; binRow[0] = 0;

        for (int32_t x = 1; x < width - 1; ++x) {
            float p00 = row0[x - 1], p01 = row0[x], p02 = row0[x + 1];
            float p10 = row1[x - 1],               p12 = row1[x + 1];
            float p20 = row2[x - 1], p21 = row2[x], p22 = row2[x + 1];

            float gxVal = (p02 - p00) + 2.0f * (p12 - p10) + (p22 - p20);
            float gyVal = (p20 - p00) + 2.0f * (p21 - p01) + (p22 - p02);

            gxRow[x] = gxVal;
            gyRow[x] = gyVal;
            magRow[x] = std::sqrt(gxVal * gxVal + gyVal * gyVal);

            float angle = std::atan2(gyVal, gxVal);
            if (angle < 0) angle += twoPi;

            int32_t bin = static_cast<int32_t>(angle * binScale);
            binRow[x] = static_cast<int16_t>(std::clamp(bin, 0, static_cast<int32_t>(maxBin)));
        }

        gxRow[width-1] = 0; gyRow[width-1] = 0; magRow[width-1] = 0; binRow[width-1] = 0;
    };
#endif

    // First row: set to zero
    std::memset(gx, 0, width * sizeof(float));
    std::memset(gy, 0, width * sizeof(float));
    std::memset(mag, 0, width * sizeof(float));
    std::memset(bins, 0, width * sizeof(int16_t));

    // Process interior rows
#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 1; y < height - 1; ++y) {
            processRow(y);
        }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 1; y < height - 1; ++y) {
            processRow(y);
        }
    }

    // Last row: set to zero
    int32_t lastRow = height - 1;
    std::memset(mag + lastRow * width, 0, width * sizeof(float));
    std::memset(bins + lastRow * width, 0, width * sizeof(int16_t));
}

// =============================================================================
// FusedSobelGradMagBinDirect: Direct write to QImage with stride (no copy)
// =============================================================================

void AnglePyramid::Impl::FusedSobelGradMagBinDirect(
    const float* src, int32_t width, int32_t height,
    float* gx, int32_t gxStride, float* gy, int32_t gyStride,
    float* mag, int32_t magStride, int16_t* bins, int32_t binStride,
    int32_t numBins)
{
    const bool useParallel = ShouldUseOpenMP(width, height);

#ifdef __AVX2__
    auto processRow = [&](int32_t y) {
        const float* row0 = src + (y - 1) * width;
        const float* row1 = src + y * width;
        const float* row2 = src + (y + 1) * width;

        float* gxRow = gx + y * gxStride;
        float* gyRow = gy + y * gyStride;
        float* magRow = mag + y * magStride;
        int16_t* binRow = bins + y * binStride;

        // Border: x=0
        gxRow[0] = 0; gyRow[0] = 0; magRow[0] = 0; binRow[0] = 0;

        int32_t x = 1;

        // AVX2 vectorized: process 8 pixels at a time
        for (; x + 8 < width; x += 8) {
            __m256 p00 = _mm256_loadu_ps(row0 + x - 1);
            __m256 p01 = _mm256_loadu_ps(row0 + x);
            __m256 p02 = _mm256_loadu_ps(row0 + x + 1);

            __m256 p10 = _mm256_loadu_ps(row1 + x - 1);
            __m256 p12 = _mm256_loadu_ps(row1 + x + 1);

            __m256 p20 = _mm256_loadu_ps(row2 + x - 1);
            __m256 p21 = _mm256_loadu_ps(row2 + x);
            __m256 p22 = _mm256_loadu_ps(row2 + x + 1);

            // Gx = (p02 - p00) + 2*(p12 - p10) + (p22 - p20)
            __m256 diff_top = _mm256_sub_ps(p02, p00);
            __m256 diff_mid = _mm256_sub_ps(p12, p10);
            __m256 diff_bot = _mm256_sub_ps(p22, p20);
            __m256 vGx = _mm256_add_ps(diff_top, _mm256_add_ps(_mm256_add_ps(diff_mid, diff_mid), diff_bot));

            // Gy = (p20 - p00) + 2*(p21 - p01) + (p22 - p02)
            __m256 vert_left = _mm256_sub_ps(p20, p00);
            __m256 vert_mid = _mm256_sub_ps(p21, p01);
            __m256 vert_right = _mm256_sub_ps(p22, p02);
            __m256 vGy = _mm256_add_ps(vert_left, _mm256_add_ps(_mm256_add_ps(vert_mid, vert_mid), vert_right));

            // Store Gx/Gy
            _mm256_storeu_ps(gxRow + x, vGx);
            _mm256_storeu_ps(gyRow + x, vGy);

            // Magnitude = sqrt(gx^2 + gy^2)
            __m256 gx2 = _mm256_mul_ps(vGx, vGx);
            __m256 gy2 = _mm256_mul_ps(vGy, vGy);
            __m256 vMag = fast_sqrt_avx2(_mm256_add_ps(gx2, gy2));
            _mm256_storeu_ps(magRow + x, vMag);

            // Fast bin quantization: directly compute bin from gx/gy
            __m256i vBin = fast_quantize_bin_avx2(vGy, vGx, numBins);

            // Pack to int16
            __m128i lo = _mm256_castsi256_si128(vBin);
            __m128i hi = _mm256_extracti128_si256(vBin, 1);
            __m128i packed = _mm_packs_epi32(lo, hi);
            _mm_storeu_si128(reinterpret_cast<__m128i*>(binRow + x), packed);
        }

        // Scalar fallback for remaining pixels
        for (; x < width - 1; ++x) {
            float p00 = row0[x - 1], p01 = row0[x], p02 = row0[x + 1];
            float p10 = row1[x - 1],               p12 = row1[x + 1];
            float p20 = row2[x - 1], p21 = row2[x], p22 = row2[x + 1];

            float gxVal = (p02 - p00) + 2.0f * (p12 - p10) + (p22 - p20);
            float gyVal = (p20 - p00) + 2.0f * (p21 - p01) + (p22 - p02);

            gxRow[x] = gxVal;
            gyRow[x] = gyVal;
            magRow[x] = std::sqrt(gxVal * gxVal + gyVal * gyVal);

            // Fast scalar bin quantization
            float abs_gx = std::abs(gxVal);
            float abs_gy = std::abs(gyVal);
            float den = std::max(abs_gx, abs_gy) + 1e-10f;
            float t = std::min(abs_gx, abs_gy) / den;
            float frac = t * (1.273f - 0.273f * t);
            frac = std::min(frac, 1.0f);

            int gx_neg = (gxVal < 0) ? 1 : 0;
            int gy_neg = (gyVal < 0) ? 1 : 0;
            int gy_gt_gx = (abs_gy > abs_gx) ? 1 : 0;
            int raw_oct = (gx_neg << 2) | (gy_neg << 1) | gy_gt_gx;
            static const int octant_map[8] = {0, 1, 7, 6, 3, 2, 4, 5};
            int octant = octant_map[raw_oct];

            int binsPerOctant = numBins / 8;
            int subBin = static_cast<int>(frac * binsPerOctant);
            if (octant & 1) {
                subBin = binsPerOctant - 1 - subBin;
            }
            int bin = octant * binsPerOctant + subBin;
            binRow[x] = static_cast<int16_t>(std::clamp(bin, 0, numBins - 1));
        }

        // Border: x=width-1
        gxRow[width-1] = 0; gyRow[width-1] = 0; magRow[width-1] = 0; binRow[width-1] = 0;
    };

#else
    const double twoPi = 2.0 * M_PI;
    const double binScale = numBins / twoPi;
    const int32_t maxBin = numBins - 1;

    auto processRow = [&](int32_t y) {
        const float* row0 = src + (y - 1) * width;
        const float* row1 = src + y * width;
        const float* row2 = src + (y + 1) * width;

        float* gxRow = gx + y * gxStride;
        float* gyRow = gy + y * gyStride;
        float* magRow = mag + y * magStride;
        int16_t* binRow = bins + y * binStride;

        gxRow[0] = 0; gyRow[0] = 0; magRow[0] = 0; binRow[0] = 0;

        for (int32_t x = 1; x < width - 1; ++x) {
            float p00 = row0[x - 1], p01 = row0[x], p02 = row0[x + 1];
            float p10 = row1[x - 1],               p12 = row1[x + 1];
            float p20 = row2[x - 1], p21 = row2[x], p22 = row2[x + 1];

            float gxVal = (p02 - p00) + 2.0f * (p12 - p10) + (p22 - p20);
            float gyVal = (p20 - p00) + 2.0f * (p21 - p01) + (p22 - p02);

            gxRow[x] = gxVal;
            gyRow[x] = gyVal;
            magRow[x] = std::sqrt(gxVal * gxVal + gyVal * gyVal);

            float angle = std::atan2(gyVal, gxVal);
            if (angle < 0) angle += twoPi;

            int32_t bin = static_cast<int32_t>(angle * binScale);
            binRow[x] = static_cast<int16_t>(std::clamp(bin, 0, static_cast<int32_t>(maxBin)));
        }

        gxRow[width-1] = 0; gyRow[width-1] = 0; magRow[width-1] = 0; binRow[width-1] = 0;
    };
#endif

    // First row: set to zero (with stride)
    for (int32_t x = 0; x < width; ++x) {
        gx[x] = 0; gy[x] = 0; mag[x] = 0; bins[x] = 0;
    }

    // Process interior rows with schedule(static, 64) for reduced overhead
#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static, 64)
        for (int32_t y = 1; y < height - 1; ++y) {
            processRow(y);
        }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 1; y < height - 1; ++y) {
            processRow(y);
        }
    }

    // Last row: set to zero (with stride)
    int32_t lastRow = height - 1;
    float* lastGx = gx + lastRow * gxStride;
    float* lastGy = gy + lastRow * gyStride;
    float* lastMag = mag + lastRow * magStride;
    int16_t* lastBins = bins + lastRow * binStride;
    for (int32_t x = 0; x < width; ++x) {
        lastGx[x] = 0; lastGy[x] = 0; lastMag[x] = 0; lastBins[x] = 0;
    }
}

// =============================================================================
// FusedSobelMagDirBinDirect: Direct write with direction storage
// =============================================================================

void AnglePyramid::Impl::FusedSobelMagDirBinDirect(
    const float* src, int32_t width, int32_t height,
    float* gx, int32_t gxStride, float* gy, int32_t gyStride,
    float* mag, int32_t magStride, float* dir, int32_t dirStride,
    int16_t* bins, int32_t binStride, int32_t numBins)
{
    const bool useParallel = ShouldUseOpenMP(width, height);
    const double twoPi = 2.0 * M_PI;
    const double binScale = numBins / twoPi;
    const int32_t maxBin = numBins - 1;

    auto processRow = [&](int32_t y) {
        const float* row0 = src + (y - 1) * width;
        const float* row1 = src + y * width;
        const float* row2 = src + (y + 1) * width;

        float* gxRow = gx + y * gxStride;
        float* gyRow = gy + y * gyStride;
        float* magRow = mag + y * magStride;
        float* dirRow = dir + y * dirStride;
        int16_t* binRow = bins + y * binStride;

        gxRow[0] = 0; gyRow[0] = 0; magRow[0] = 0; dirRow[0] = 0; binRow[0] = 0;

        for (int32_t x = 1; x < width - 1; ++x) {
            float p00 = row0[x - 1], p01 = row0[x], p02 = row0[x + 1];
            float p10 = row1[x - 1],               p12 = row1[x + 1];
            float p20 = row2[x - 1], p21 = row2[x], p22 = row2[x + 1];

            float gxVal = (p02 - p00) + 2.0f * (p12 - p10) + (p22 - p20);
            float gyVal = (p20 - p00) + 2.0f * (p21 - p01) + (p22 - p02);

            gxRow[x] = gxVal;
            gyRow[x] = gyVal;
            magRow[x] = std::sqrt(gxVal * gxVal + gyVal * gyVal);

            float angle = std::atan2(gyVal, gxVal);
            if (angle < 0) angle += twoPi;
            dirRow[x] = angle;

            int32_t bin = static_cast<int32_t>(angle * binScale);
            binRow[x] = static_cast<int16_t>(std::clamp(bin, 0, static_cast<int32_t>(maxBin)));
        }

        gxRow[width-1] = 0; gyRow[width-1] = 0; magRow[width-1] = 0;
        dirRow[width-1] = 0; binRow[width-1] = 0;
    };

    // First row: set to zero (with stride)
    for (int32_t x = 0; x < width; ++x) {
        gx[x] = 0; gy[x] = 0; mag[x] = 0; dir[x] = 0; bins[x] = 0;
    }

    // Process interior rows with schedule(static, 64)
#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static, 64)
        for (int32_t y = 1; y < height - 1; ++y) {
            processRow(y);
        }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 1; y < height - 1; ++y) {
            processRow(y);
        }
    }

    // Last row: set to zero (with stride)
    int32_t lastRow = height - 1;
    float* lastGx = gx + lastRow * gxStride;
    float* lastGy = gy + lastRow * gyStride;
    float* lastMag = mag + lastRow * magStride;
    float* lastDir = dir + lastRow * dirStride;
    int16_t* lastBins = bins + lastRow * binStride;
    for (int32_t x = 0; x < width; ++x) {
        lastGx[x] = 0; lastGy[x] = 0; lastMag[x] = 0; lastDir[x] = 0; lastBins[x] = 0;
    }
}

// =============================================================================
// BuildLevelFused: Use fused computation
// =============================================================================

bool AnglePyramid::Impl::BuildLevelFused(const std::vector<float>& srcData, int32_t width, int32_t height,
                                          int32_t level, double scale) {
    PyramidLevelData levelData;
    levelData.level = level;
    levelData.width = width;
    levelData.height = height;
    levelData.scale = scale;

    size_t pixelCount = static_cast<size_t>(width) * height;
    const bool storeDir = params_.storeDirection;

    // Debug: Validate inputs
    if (srcData.size() < pixelCount) {
        fprintf(stderr, "[BuildLevelFused] ERROR: srcData size %zu < pixelCount %zu\n",
                srcData.size(), pixelCount);
        return false;
    }
    if (width < 3 || height < 3) {
        fprintf(stderr, "[BuildLevelFused] ERROR: width=%d, height=%d too small\n", width, height);
        return false;
    }

    // Create QImages first (gx/gy always needed for scoring)
    levelData.gradX = QImage(width, height, PixelType::Float32, ChannelType::Gray);
    levelData.gradY = QImage(width, height, PixelType::Float32, ChannelType::Gray);
    levelData.gradMag = QImage(width, height, PixelType::Float32, ChannelType::Gray);
    levelData.angleBinImage = QImage(width, height, PixelType::Int16, ChannelType::Gray);
    if (storeDir) {
        levelData.gradDir = QImage(width, height, PixelType::Float32, ChannelType::Gray);
    }

    // Get QImage pointers and strides for direct writing
    float* gxDst = static_cast<float*>(levelData.gradX.Data());
    float* gyDst = static_cast<float*>(levelData.gradY.Data());
    float* magDst = static_cast<float*>(levelData.gradMag.Data());
    int16_t* binDst = static_cast<int16_t*>(levelData.angleBinImage.Data());
    int32_t gxStride = levelData.gradX.Stride() / sizeof(float);
    int32_t gyStride = levelData.gradY.Stride() / sizeof(float);
    int32_t magStride = levelData.gradMag.Stride() / sizeof(float);
    int32_t binStride = levelData.angleBinImage.Stride() / sizeof(int16_t);

    // Fused computation: Sobel + Mag + [Dir] + Quantize - directly into QImage (no copy)
    if (storeDir) {
        // Full computation with direction storage
        float* dirDst = static_cast<float*>(levelData.gradDir.Data());
        int32_t dirStride = levelData.gradDir.Stride() / sizeof(float);
        FusedSobelMagDirBinDirect(srcData.data(), width, height,
                                  gxDst, gxStride, gyDst, gyStride,
                                  magDst, magStride, dirDst, dirStride,
                                  binDst, binStride, params_.angleBins);
    } else {
        // Optimized: compute gx/gy/mag/bins directly into QImage (skip dir)
        FusedSobelGradMagBinDirect(srcData.data(), width, height,
                                   gxDst, gxStride, gyDst, gyStride,
                                   magDst, magStride, binDst, binStride,
                                   params_.angleBins);
    }

    levels_.push_back(std::move(levelData));
    return true;
}

bool AnglePyramid::Impl::BuildLevel(const std::vector<float>& srcData, int32_t width, int32_t height,
                                    int32_t level, double scale) {
    PyramidLevelData levelData;
    levelData.level = level;
    levelData.width = width;
    levelData.height = height;
    levelData.scale = scale;

    // Allocate contiguous buffers for gradients
    std::vector<float> gxBuffer(static_cast<size_t>(width) * height);
    std::vector<float> gyBuffer(static_cast<size_t>(width) * height);

    // Use Sobel operator for gradient computation (contiguous memory)
    Gradient<float, float>(srcData.data(), gxBuffer.data(), gyBuffer.data(),
                           width, height,
                           GradientOperator::Sobel3x3, BorderMode::Reflect101);

    // Create QImage for storage
    QImage gx(width, height, PixelType::Float32, ChannelType::Gray);
    QImage gy(width, height, PixelType::Float32, ChannelType::Gray);

    float* gxDst = static_cast<float*>(gx.Data());
    float* gyDst = static_cast<float*>(gy.Data());
    int32_t gxStride = gx.Stride() / sizeof(float);
    int32_t gyStride = gy.Stride() / sizeof(float);

    // Copy with conditional OpenMP parallelization
    const bool useParallel = ShouldUseOpenMP(width, height);

    auto copyRow = [&](int32_t y) {
        for (int32_t x = 0; x < width; ++x) {
            gxDst[y * gxStride + x] = gxBuffer[y * width + x];
            gyDst[y * gyStride + x] = gyBuffer[y * width + x];
        }
    };

#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < height; ++y) {
            copyRow(y);
        }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 0; y < height; ++y) {
            copyRow(y);
        }
    }

    if (gx.Empty() || gy.Empty()) {
        return false;
    }

    levelData.gradX = std::move(gx);
    levelData.gradY = std::move(gy);

    // Allocate magnitude and direction images
    levelData.gradMag = QImage(width, height, PixelType::Float32, ChannelType::Gray);
    levelData.gradDir = QImage(width, height, PixelType::Float32, ChannelType::Gray);

    // Compute magnitude and direction with optimized SIMD + OpenMP
    const float* gxData = static_cast<const float*>(levelData.gradX.Data());
    const float* gyData = static_cast<const float*>(levelData.gradY.Data());
    float* magData = static_cast<float*>(levelData.gradMag.Data());
    float* dirData = static_cast<float*>(levelData.gradDir.Data());

    int32_t gxStr = levelData.gradX.Stride() / sizeof(float);
    int32_t gyStr = levelData.gradY.Stride() / sizeof(float);
    int32_t magStr = levelData.gradMag.Stride() / sizeof(float);
    int32_t dirStr = levelData.gradDir.Stride() / sizeof(float);

    ComputeGradientMagnitudeDirectionOpt(gxData, gyData, magData, dirData,
                                          width, height,
                                          gxStr, gyStr, magStr, dirStr);

    // Allocate and quantize direction to angle bins
    levelData.angleBinImage = QImage(width, height, PixelType::Int16, ChannelType::Gray);
    int16_t* binData = static_cast<int16_t*>(levelData.angleBinImage.Data());
    int32_t binStr = levelData.angleBinImage.Stride() / sizeof(int16_t);

    QuantizeDirectionOpt(dirData, binData, width, height, dirStr, binStr, params_.angleBins);

    levels_.push_back(std::move(levelData));
    return true;
}

void AnglePyramid::Impl::ExtractEdgePointsForLevel(int32_t level) {
    if (level < 0 || level >= static_cast<int32_t>(levels_.size())) {
        return;
    }

    auto& levelData = levels_[level];
    levelData.edgePoints.clear();

    const int32_t width = levelData.width;
    const int32_t height = levelData.height;

    const float* magData = static_cast<const float*>(levelData.gradMag.Data());
    const float* dirData = static_cast<const float*>(levelData.gradDir.Data());
    const int16_t* binData = static_cast<const int16_t*>(levelData.angleBinImage.Data());

    int32_t magStride = levelData.gradMag.Stride() / sizeof(float);
    int32_t dirStride = levelData.gradDir.Stride() / sizeof(float);
    int32_t binStride = levelData.angleBinImage.Stride() / sizeof(int16_t);

    const float minContrast = static_cast<float>(params_.minContrast);

    if (params_.useNMS) {
        // Original behavior: Apply Non-Maximum Suppression along gradient direction
        // This thins edges from "bands" to single-pixel-wide "ridges" (Canny-style)
        // NMS2DGradient expects contiguous data, so copy if stride != width
        std::vector<float> magContiguous, dirContiguous;
        const float* magForNms = magData;
        const float* dirForNms = dirData;

        if (magStride != width) {
            magContiguous.resize(width * height);
            for (int32_t y = 0; y < height; ++y) {
                std::memcpy(magContiguous.data() + y * width,
                            magData + y * magStride, width * sizeof(float));
            }
            magForNms = magContiguous.data();
        }
        if (dirStride != width) {
            dirContiguous.resize(width * height);
            for (int32_t y = 0; y < height; ++y) {
                std::memcpy(dirContiguous.data() + y * width,
                            dirData + y * dirStride, width * sizeof(float));
            }
            dirForNms = dirContiguous.data();
        }

        std::vector<float> nmsOutput(width * height, 0.0f);
        NMS2DGradient(magForNms, dirForNms, nmsOutput.data(), width, height, minContrast);

        // Extract edge points from NMS output with subpixel refinement
        levelData.edgePoints.reserve(width * height / 20);

        for (int32_t y = 2; y < height - 2; ++y) {
            for (int32_t x = 2; x < width - 2; ++x) {
                float nmsMag = nmsOutput[y * width + x];

                if (nmsMag >= minContrast) {
                    float dir = dirData[y * dirStride + x];
                    double cosDir = std::cos(dir);
                    double sinDir = std::sin(dir);

                    auto sampleMag = [&](double ox, double oy) -> double {
                        int32_t ix = x + static_cast<int32_t>(std::round(ox));
                        int32_t iy = y + static_cast<int32_t>(std::round(oy));
                        ix = std::clamp(ix, 0, width - 1);
                        iy = std::clamp(iy, 0, height - 1);
                        return magData[iy * magStride + ix];
                    };

                    double p0 = sampleMag(-cosDir, -sinDir);
                    double p1 = magData[y * magStride + x];
                    double p2 = sampleMag(cosDir, sinDir);

                    double denom = 2.0 * (p0 - 2.0 * p1 + p2);
                    double offset = 0.0;
                    if (std::abs(denom) > 1e-10) {
                        offset = (p0 - p2) / denom;
                        offset = std::clamp(offset, -0.5, 0.5);
                    }

                    double subX = x + offset * cosDir;
                    double subY = y + offset * sinDir;

                    int16_t bin = binData[y * binStride + x];

                    levelData.edgePoints.emplace_back(
                        subX, subY,
                        static_cast<double>(dir),
                        static_cast<double>(nmsMag),
                        static_cast<int32_t>(bin)
                    );
                }
            }
        }
    } else {
        // Shape-based matching mode (LINEMOD/Halcon style):
        // 1. NO Canny-style NMS
        // 2. Apply 3x3 neighborhood mode filter for robust orientation quantization
        //    IMPORTANT: Replace orientation with neighborhood mode, don't filter pixels!
        //
        // Reference: LINEMOD paper (Hinterstoisser et al. ICCV 2011)
        // "Quantized orientations are made more robust by taking the mode in a 3x3 neighborhood"
        //
        // Paper pseudo-code:
        //   for each pixel location x:
        //       quantized_value[x] = mode(quantized values in 3×3 neighborhood of x)

        levelData.edgePoints.reserve(width * height / 10);

        // Helper: compute mode (most frequent value) in 3x3 neighborhood
        // Returns both mode bin and the corresponding direction
        auto computeNeighborhoodMode = [&](int32_t cx, int32_t cy, float& modeDir) -> int16_t {
            // Count occurrences of each bin in 3x3 neighborhood
            // Only count pixels above contrast threshold
            std::array<int32_t, 64> binCounts = {};
            std::array<float, 64> binDirSum = {};  // Sum of directions for each bin
            int32_t maxCount = 0;
            int16_t modeBin = binData[cy * binStride + cx];

            for (int32_t dy = -1; dy <= 1; ++dy) {
                for (int32_t dx = -1; dx <= 1; ++dx) {
                    int32_t nx = cx + dx;
                    int32_t ny = cy + dy;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        float neighborMag = magData[ny * magStride + nx];
                        if (neighborMag >= minContrast) {
                            int16_t neighborBin = binData[ny * binStride + nx];
                            if (neighborBin >= 0 && neighborBin < 64) {
                                binCounts[neighborBin]++;
                                binDirSum[neighborBin] += dirData[ny * dirStride + nx];
                                if (binCounts[neighborBin] > maxCount) {
                                    maxCount = binCounts[neighborBin];
                                    modeBin = neighborBin;
                                }
                            }
                        }
                    }
                }
            }

            // Compute average direction for the mode bin
            if (binCounts[modeBin] > 0) {
                modeDir = binDirSum[modeBin] / binCounts[modeBin];
            } else {
                modeDir = dirData[cy * dirStride + cx];
            }

            return modeBin;
        };

        for (int32_t y = 1; y < height - 1; ++y) {
            for (int32_t x = 1; x < width - 1; ++x) {
                float mag = magData[y * magStride + x];

                if (mag >= minContrast) {
                    // Robust quantization: REPLACE orientation with neighborhood mode
                    // (Don't filter pixels - keep all pixels above threshold)
                    float modeDir;
                    int16_t modeBin = computeNeighborhoodMode(x, y, modeDir);

                    levelData.edgePoints.emplace_back(
                        static_cast<double>(x),
                        static_cast<double>(y),
                        static_cast<double>(modeDir),  // Use mode direction
                        static_cast<double>(mag),
                        static_cast<int32_t>(modeBin)  // Use mode bin
                    );
                }
            }
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
    Clear();

    if (image.Empty()) {
        return false;
    }

    // Reset timing
    impl_->timing_ = AnglePyramidTiming();
    auto tTotal = std::chrono::high_resolution_clock::now();

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

    // Convert to float for processing (with AVX2 SIMD + OpenMP)
    auto tToFloat = std::chrono::high_resolution_clock::now();
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

        const bool useParallel = ShouldUseOpenMP(w, h);

#ifdef __AVX2__
        // AVX2 optimized: convert 8 uint8 to 8 float at once
        auto convertRowAVX2 = [&](int32_t y) {
            const uint8_t* srcRow = srcData + y * srcStride;
            float* dstRow = dstData + y * dstStride;
            int32_t x = 0;
            for (; x + 8 <= w; x += 8) {
                __m128i v8 = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(srcRow + x));
                __m256i v32 = _mm256_cvtepu8_epi32(v8);
                __m256 vf = _mm256_cvtepi32_ps(v32);
                _mm256_storeu_ps(dstRow + x, vf);
            }
            // Scalar tail
            for (; x < w; ++x) {
                dstRow[x] = static_cast<float>(srcRow[x]);
            }
        };

#ifdef _OPENMP
        if (useParallel) {
            #pragma omp parallel for schedule(static)
            for (int32_t y = 0; y < h; ++y) { convertRowAVX2(y); }
        } else
#endif
        { for (int32_t y = 0; y < h; ++y) { convertRowAVX2(y); } }

#else
        // Scalar fallback
#ifdef _OPENMP
        if (useParallel) {
            #pragma omp parallel for schedule(static)
            for (int32_t y = 0; y < h; ++y) {
                for (int32_t x = 0; x < w; ++x) {
                    dstData[y * dstStride + x] = static_cast<float>(srcData[y * srcStride + x]);
                }
            }
        } else
#endif
        {
            (void)useParallel;
            for (int32_t y = 0; y < h; ++y) {
                for (int32_t x = 0; x < w; ++x) {
                    dstData[y * dstStride + x] = static_cast<float>(srcData[y * srcStride + x]);
                }
            }
        }
#endif
    }
    if (params.enableTiming) {
        impl_->timing_.toFloatMs = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - tToFloat).count();
    }

    // Build Gaussian pyramid using the existing Pyramid module
    PyramidParams pyramidParams;
    pyramidParams.numLevels = impl_->params_.numLevels;
    pyramidParams.sigma = std::max(1.0, params.smoothSigma);
    pyramidParams.minDimension = 8;  // Minimum 8 pixels at coarsest level

    // Extract float data from the float QImage (handle stride correctly)
    // Optimized: use memcpy instead of element-by-element copy
    auto tCopy = std::chrono::high_resolution_clock::now();
    int32_t floatWidth = floatImage.Width();
    int32_t floatHeight = floatImage.Height();
    int32_t floatStride = floatImage.Stride() / sizeof(float);
    const float* floatSrcData = static_cast<const float*>(floatImage.Data());

    std::vector<float> floatContiguous(floatWidth * floatHeight);

    if (floatStride == floatWidth) {
        // Contiguous memory: single memcpy for entire image
        std::memcpy(floatContiguous.data(), floatSrcData,
                    static_cast<size_t>(floatWidth) * floatHeight * sizeof(float));
    } else {
        // Strided memory: memcpy row by row
        const bool useParallelCopy = ShouldUseOpenMP(floatWidth, floatHeight);
#ifdef _OPENMP
        if (useParallelCopy) {
            #pragma omp parallel for schedule(static)
            for (int32_t y = 0; y < floatHeight; ++y) {
                std::memcpy(floatContiguous.data() + y * floatWidth,
                            floatSrcData + y * floatStride,
                            floatWidth * sizeof(float));
            }
        } else
#endif
        {
            (void)useParallelCopy;
            for (int32_t y = 0; y < floatHeight; ++y) {
                std::memcpy(floatContiguous.data() + y * floatWidth,
                            floatSrcData + y * floatStride,
                            floatWidth * sizeof(float));
            }
        }
    }
    if (params.enableTiming) {
        impl_->timing_.copyMs = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - tCopy).count();
    }

    // Use the float overload of BuildGaussianPyramid
    auto tGaussPyramid = std::chrono::high_resolution_clock::now();
    ImagePyramid gaussPyramid = BuildGaussianPyramid(floatContiguous.data(),
                                                      floatWidth, floatHeight, pyramidParams);
    if (params.enableTiming) {
        impl_->timing_.gaussPyramidMs = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - tGaussPyramid).count();
    }

    // Update actual number of levels
    impl_->params_.numLevels = gaussPyramid.NumLevels();

    // Build angle pyramid for each level
    auto tSobel = std::chrono::high_resolution_clock::now();
    impl_->levels_.reserve(gaussPyramid.NumLevels());
    double scale = 1.0;

    for (int32_t level = 0; level < gaussPyramid.NumLevels(); ++level) {
        const auto& pyramidLevel = gaussPyramid.GetLevel(level);
        // Use fused version: Sobel + Mag + Dir + Quantize in one pass
        if (!impl_->BuildLevelFused(pyramidLevel.data, pyramidLevel.width, pyramidLevel.height,
                                    level, scale)) {
            Clear();
            return false;
        }
        scale *= 0.5;
    }
    if (params.enableTiming) {
        impl_->timing_.sobelMs = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - tSobel).count();
    }

    // Extract edge points for each level (only if needed, e.g., for model creation)
    auto tExtractEdge = std::chrono::high_resolution_clock::now();
    if (params.extractEdgePoints) {
        for (int32_t level = 0; level < static_cast<int32_t>(impl_->levels_.size()); ++level) {
            impl_->ExtractEdgePointsForLevel(level);
        }
    }
    if (params.enableTiming) {
        impl_->timing_.extractEdgeMs = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - tExtractEdge).count();
        impl_->timing_.totalMs = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - tTotal).count();
    }

    impl_->valid_ = true;
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

bool AnglePyramid::GetAngleBinData(int32_t level, const int16_t*& binData,
                                    int32_t& width, int32_t& height, int32_t& stride,
                                    int32_t& numBins) const {
    if (level < 0 || level >= static_cast<int32_t>(impl_->levels_.size())) {
        return false;
    }

    const auto& levelData = impl_->levels_[level];
    binData = static_cast<const int16_t*>(levelData.angleBinImage.Data());
    width = levelData.width;
    height = levelData.height;
    stride = levelData.angleBinImage.Stride() / sizeof(int16_t);
    numBins = impl_->params_.angleBins;

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

    const bool useParallel = ShouldUseOpenMP(levelData.width, levelData.height);

#ifdef _OPENMP
    if (useParallel) {
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
    } else
#endif
    {
        // 小图: 单线程执行
        (void)useParallel;
        result.reserve(1000);

        for (int32_t y = y0; y < y1; ++y) {
            for (int32_t x = x0; x < x1; ++x) {
                float mag = magData[y * magStride + x];

                if (mag >= threshold) {
                    float dir = dirData[y * dirStride + x];
                    int16_t bin = binData[y * binStride + x];

                    result.emplace_back(
                        static_cast<double>(x),
                        static_cast<double>(y),
                        static_cast<double>(dir),
                        static_cast<double>(mag),
                        static_cast<int32_t>(bin)
                    );
                }
            }
        }
    }

    return result;
}

std::vector<EdgePoint> AnglePyramid::ExtractEdgePoints(int32_t level,
                                                        const QRegion& region,
                                                        double minContrast) const {
    std::vector<EdgePoint> result;

    if (level < 0 || level >= static_cast<int32_t>(impl_->levels_.size())) {
        return result;
    }

    if (region.Empty()) {
        // Empty region = full image, delegate to Rect2i version
        return ExtractEdgePoints(level, Rect2i(), minContrast);
    }

    const auto& levelData = impl_->levels_[level];

    // Get region bounding box for fast culling
    Rect2i bbox = region.BoundingBox();
    int32_t x0 = std::max(1, bbox.x);
    int32_t y0 = std::max(1, bbox.y);
    int32_t x1 = std::min(levelData.width - 1, bbox.x + bbox.width);
    int32_t y1 = std::min(levelData.height - 1, bbox.y + bbox.height);

    double threshold = (minContrast > 0) ? minContrast : impl_->params_.minContrast;

    const float* magData = static_cast<const float*>(levelData.gradMag.Data());
    const float* dirData = static_cast<const float*>(levelData.gradDir.Data());
    const int16_t* binData = static_cast<const int16_t*>(levelData.angleBinImage.Data());

    int32_t magStride = levelData.gradMag.Stride() / sizeof(float);
    int32_t dirStride = levelData.gradDir.Stride() / sizeof(float);
    int32_t binStride = levelData.angleBinImage.Stride() / sizeof(int16_t);

    const bool useParallel = ShouldUseOpenMP(levelData.width, levelData.height);

#ifdef _OPENMP
    if (useParallel) {
        // Use OpenMP with thread-local vectors
        #pragma omp parallel
        {
            std::vector<EdgePoint> localPoints;
            localPoints.reserve(1000);

            #pragma omp for schedule(static) nowait
            for (int32_t y = y0; y < y1; ++y) {
                for (int32_t x = x0; x < x1; ++x) {
                    // Check if point is within region
                    if (!region.Contains(x, y)) {
                        continue;
                    }

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
    } else
#endif
    {
        // Small image: single-threaded execution
        (void)useParallel;
        result.reserve(1000);

        for (int32_t y = y0; y < y1; ++y) {
            for (int32_t x = x0; x < x1; ++x) {
                // Check if point is within region
                if (!region.Contains(x, y)) {
                    continue;
                }

                float mag = magData[y * magStride + x];

                if (mag >= threshold) {
                    float dir = dirData[y * dirStride + x];
                    int16_t bin = binData[y * binStride + x];

                    result.emplace_back(
                        static_cast<double>(x),
                        static_cast<double>(y),
                        static_cast<double>(dir),
                        static_cast<double>(mag),
                        static_cast<int32_t>(bin)
                    );
                }
            }
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
    const bool useParallel = ShouldUseOpenMP(width, height);

    auto processRow = [&](int32_t y) {
        for (int32_t x = 0; x < width; ++x) {
            float gx = gxData[y * gxStride + x];
            float gy = gyData[y * gyStride + x];
            float angle = std::atan2(gy, gx);
            if (angle < 0) angle += twoPi;
            outData[y * outStride + x] = angle;
        }
    };

#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < height; ++y) {
            processRow(y);
        }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 0; y < height; ++y) {
            processRow(y);
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

    const bool useParallel = ShouldUseOpenMP(width, height);

#ifdef __AVX2__
    auto processRow = [&](int32_t y) {
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
    };

#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < height; ++y) {
            processRow(y);
        }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 0; y < height; ++y) {
            processRow(y);
        }
    }
#else
    auto processRow = [&](int32_t y) {
        for (int32_t x = 0; x < width; ++x) {
            float gx = gxData[y * gxStride + x];
            float gy = gyData[y * gyStride + x];
            outData[y * outStride + x] = std::sqrt(gx * gx + gy * gy);
        }
    };

#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < height; ++y) {
            processRow(y);
        }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 0; y < height; ++y) {
            processRow(y);
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
    const bool useParallel = ShouldUseOpenMP(width, height);

    auto processRow = [&](int32_t y) {
        for (int32_t x = 0; x < width; ++x) {
            float angle = dirData[y * dirStride + x];
            int32_t bin = static_cast<int32_t>(angle * binScale);
            bin = std::clamp(bin, 0, numBins - 1);
            outData[y * outStride + x] = static_cast<int16_t>(bin);
        }
    };

#ifdef _OPENMP
    if (useParallel) {
        #pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < height; ++y) {
            processRow(y);
        }
    } else
#endif
    {
        (void)useParallel;
        for (int32_t y = 0; y < height; ++y) {
            processRow(y);
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
