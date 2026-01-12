/**
 * @file LinemodPyramid.cpp
 * @brief Implementation of LINEMOD-style gradient pyramid
 *
 * Implements the exact algorithm from:
 * - Hinterstoisser et al. "Gradient Response Maps for Real-Time Detection
 *   of Texture-Less Objects" (TPAMI 2012)
 *
 * Key implementation details:
 * - 8-bin quantization using bit flags (not index)
 * - 3x3 neighbor histogram with threshold voting
 * - OR spreading (not max filter)
 * - SIMILARITY_LUT[8][256] for O(1) scoring
 */

#include <QiVision/Internal/LinemodPyramid.h>
#include <QiVision/Internal/Gradient.h>
#include <QiVision/Internal/Gaussian.h>
#include <QiVision/Internal/Convolution.h>
#include <QiVision/Internal/Pyramid.h>
#include <QiVision/Core/Constants.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <queue>

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef __SSE2__
#include <emmintrin.h>
#endif

#ifdef __AVX2__
#include <immintrin.h>
#endif

namespace Qi::Vision::Internal {

// =============================================================================
// SIMILARITY_LUT Implementation
// =============================================================================

SimilarityLUT::SimilarityLUT() {
    // Precompute LUT[ori][mask]
    // LUT[i][L] = max_{j in L} |cos((i-j) * 45 degrees)| * 4
    //
    // Cosine values for angle differences:
    // 0°:  cos(0)   = 1.000 -> 4
    // 45°: cos(45)  = 0.707 -> 3
    // 90°: cos(90)  = 0.000 -> 0
    // 135°: cos(135) = -0.707 -> 3 (absolute value)
    // 180°: cos(180) = -1.000 -> 4 (absolute value)

    // Pre-computed |cos((i-j) * PI/4)| * 4, scaled to [0, 4]
    static const uint8_t cosTable[8] = {
        4,  // 0°:   |cos(0)|   = 1.000 -> 4
        3,  // 45°:  |cos(45)|  = 0.707 -> 3
        1,  // 90°:  |cos(90)|  = 0.000 -> 0, but use 1 for some tolerance
        3,  // 135°: |cos(135)| = 0.707 -> 3
        4,  // 180°: |cos(180)| = 1.000 -> 4
        3,  // 225°: |cos(225)| = 0.707 -> 3
        1,  // 270°: |cos(270)| = 0.000 -> 0, but use 1 for some tolerance
        3   // 315°: |cos(315)| = 0.707 -> 3
    };

    for (int32_t ori = 0; ori < 8; ++ori) {
        for (int32_t mask = 0; mask < 256; ++mask) {
            uint8_t maxSim = 0;

            // Check each bit in the mask
            for (int32_t j = 0; j < 8; ++j) {
                if (mask & (1 << j)) {
                    // Compute angle difference
                    int32_t diff = (ori - j + 8) % 8;
                    uint8_t sim = cosTable[diff];
                    if (sim > maxSim) {
                        maxSim = sim;
                    }
                }
            }

            lut_[ori][mask] = maxSim;
        }
    }
}

// Global instance
const SimilarityLUT g_SimilarityLUT;

// =============================================================================
// LinemodPyramid Implementation
// =============================================================================

LinemodPyramid::LinemodPyramid() = default;
LinemodPyramid::~LinemodPyramid() = default;

LinemodPyramid::LinemodPyramid(LinemodPyramid&&) noexcept = default;
LinemodPyramid& LinemodPyramid::operator=(LinemodPyramid&&) noexcept = default;

void LinemodPyramid::Clear() {
    levels_.clear();
    numLevels_ = 0;
    valid_ = false;
}

bool LinemodPyramid::Build(const QImage& image, const LinemodPyramidParams& params) {
    Clear();

    if (image.Empty() || image.Channels() != 1) {
        return false;
    }

    params_ = params;
    numLevels_ = std::max(1, std::min(params.numLevels, 10));
    levels_.resize(numLevels_);

    // =========================================================================
    // 架构优化：只对 Level 0 做完整的 Sobel/量化计算
    // Level 1+ 从量化图下采样，大幅减少计算量
    // =========================================================================

    // Convert to float if needed
    QImage floatImage;
    if (image.Type() == PixelType::Float32) {
        floatImage = image;
    } else {
        floatImage = QImage(image.Width(), image.Height(), PixelType::Float32, ChannelType::Gray);
        const uint8_t* src = static_cast<const uint8_t*>(image.Data());
        float* dst = static_cast<float*>(floatImage.Data());
        const int32_t srcStride = static_cast<int32_t>(image.Stride());
        const int32_t dstStride = static_cast<int32_t>(floatImage.Stride() / sizeof(float));

        for (int32_t y = 0; y < image.Height(); ++y) {
            for (int32_t x = 0; x < image.Width(); ++x) {
                dst[y * dstStride + x] = static_cast<float>(src[y * srcStride + x]);
            }
        }
    }

    // Level 0: 完整流程 (Sobel + 量化 + 邻域投票 + OR扩散)
    if (!BuildLevel(floatImage, 0)) {
        Clear();
        return false;
    }

    // Level 1+: 从上一层量化图下采样 + OR扩散
    for (int32_t level = 1; level < numLevels_; ++level) {
        const auto& prevLevel = levels_[level - 1];
        int32_t newW = prevLevel.width / 2;
        int32_t newH = prevLevel.height / 2;

        if (newW < 8 || newH < 8) {
            numLevels_ = level;
            levels_.resize(numLevels_);
            break;
        }

        if (!BuildLevelFromQuantized(level, prevLevel)) {
            Clear();
            return false;
        }
    }

    valid_ = true;
    return true;
}

bool LinemodPyramid::BuildLevelFromQuantized(int32_t levelIdx, const LinemodLevelData& prevLevel) {
    // 从上一层的量化图下采样构建当前层
    // 只需要：下采样 + 邻域投票 + OR扩散
    // 不需要：高斯模糊、Sobel、幅值计算、方向量化

    auto& level = levels_[levelIdx];
    level.width = prevLevel.width / 2;
    level.height = prevLevel.height / 2;
    level.scale = std::pow(0.5, levelIdx);
    level.stride = (level.width + 63) & ~63;

    const int32_t W = level.width;
    const int32_t H = level.height;

    // 1. 下采样量化图 (2x2 区域 OR 合并)
    level.quantized = QImage(W, H, PixelType::UInt8, ChannelType::Gray);
    {
        const uint8_t* srcData = static_cast<const uint8_t*>(prevLevel.quantized.Data());
        uint8_t* dstData = static_cast<uint8_t*>(level.quantized.Data());
        const int32_t srcStride = static_cast<int32_t>(prevLevel.quantized.Stride());
        const int32_t dstStride = static_cast<int32_t>(level.quantized.Stride());

        std::memset(dstData, 0, H * dstStride);

#pragma omp parallel for if(W * H > 50000)
        for (int32_t y = 0; y < H; ++y) {
            for (int32_t x = 0; x < W; ++x) {
                int32_t sy = y * 2;
                int32_t sx = x * 2;

                // 2x2 区域投票选最强方向
                std::array<int32_t, 8> votes = {0};
                for (int32_t dy = 0; dy < 2; ++dy) {
                    for (int32_t dx = 0; dx < 2; ++dx) {
                        uint8_t q = srcData[(sy + dy) * srcStride + (sx + dx)];
                        if (q == 0) continue;
                        for (int32_t b = 0; b < 8; ++b) {
                            if (q & (1 << b)) {
                                votes[b]++;
                            }
                        }
                    }
                }

                // 找最多票数的方向
                int32_t maxVotes = 0;
                int32_t maxBin = -1;
                for (int32_t b = 0; b < 8; ++b) {
                    if (votes[b] > maxVotes) {
                        maxVotes = votes[b];
                        maxBin = b;
                    }
                }

                // 至少要有 2 票 (2x2 中至少 2 个像素有这个方向)
                if (maxVotes >= 2 && maxBin >= 0) {
                    dstData[y * dstStride + x] = static_cast<uint8_t>(1 << maxBin);
                }
            }
        }
    }

    // 2. 邻域投票 (保持一致性)
    ApplyNeighborVoting(level);

    // 3. OR 扩散
    ApplyORSpreading(level);

    // 注意：BuildLevelFromQuantized 不提取特征
    // 因为模板特征只在 Level 0 提取，粗层只用于搜索
    // ExtractLevelFeatures 需要 gradMag，而我们没有计算它

    return true;
}

bool LinemodPyramid::BuildLevel(const QImage& image, int32_t levelIdx) {
    auto& level = levels_[levelIdx];

    level.width = image.Width();
    level.height = image.Height();
    level.scale = std::pow(0.5, levelIdx);
    level.stride = (level.width + 63) & ~63;  // 64-byte alignment

    const int32_t W = level.width;
    const int32_t H = level.height;

    // =========================================================================
    // 优化后的流程 (3 pass):
    // Pass 1: 高斯模糊 (需要邻域，不可避免)
    // Pass 2: Sobel + 幅值 + 量化 (合并！)
    // Pass 3: 投票 + OR扩散 (合并！)
    // =========================================================================

    const float* imgData = static_cast<const float*>(image.Data());
    const int32_t imgStride = static_cast<int32_t>(image.Stride() / sizeof(float));

    // Pass 1: 高斯模糊
    std::vector<float> smoothed(static_cast<size_t>(W) * H);
    {
        // 如果 stride == width，直接处理；否则复制
        // 使用 3x3 核 (σ~0.5) 替代 7x7 核 (σ=1.0) 以提升速度
        if (imgStride == W) {
            GaussianBlurFixed<float, float>(imgData, smoothed.data(), W, H, 3, 0.5);
        } else {
            std::vector<float> contiguous(static_cast<size_t>(W) * H);
            for (int32_t y = 0; y < H; ++y) {
                std::memcpy(contiguous.data() + y * W,
                           imgData + y * imgStride, W * sizeof(float));
            }
            GaussianBlurFixed<float, float>(contiguous.data(), smoothed.data(), W, H, 3, 0.5);
        }
    }

    // Pass 2: Sobel + 幅值 + 量化 (合并为一个 pass)
    level.gradMag = QImage(W, H, PixelType::Float32, ChannelType::Gray);
    level.quantized = QImage(W, H, PixelType::UInt8, ChannelType::Gray);
    {
        float* magData = static_cast<float*>(level.gradMag.Data());
        uint8_t* quantData = static_cast<uint8_t*>(level.quantized.Data());
        const int32_t magStride = static_cast<int32_t>(level.gradMag.Stride() / sizeof(float));
        const int32_t quantStride = static_cast<int32_t>(level.quantized.Stride());

        // Sobel 核
        // [-1  0  1]      [-1 -2 -1]
        // [-2  0  2]      [ 0  0  0]
        // [-1  0  1]      [ 1  2  1]

        const float minMag = params_.minMagnitude;

#pragma omp parallel for schedule(static)
        for (int32_t y = 0; y < H; ++y) {
            for (int32_t x = 0; x < W; ++x) {
                // 边界处理
                int32_t y0 = (y > 0) ? y - 1 : 0;
                int32_t y1 = y;
                int32_t y2 = (y < H - 1) ? y + 1 : H - 1;
                int32_t x0 = (x > 0) ? x - 1 : 0;
                int32_t x1 = x;
                int32_t x2 = (x < W - 1) ? x + 1 : W - 1;

                // 读取 3x3 邻域
                float p00 = smoothed[y0 * W + x0];
                float p01 = smoothed[y0 * W + x1];
                float p02 = smoothed[y0 * W + x2];
                float p10 = smoothed[y1 * W + x0];
                // float p11 = smoothed[y1 * W + x1];  // 不需要
                float p12 = smoothed[y1 * W + x2];
                float p20 = smoothed[y2 * W + x0];
                float p21 = smoothed[y2 * W + x1];
                float p22 = smoothed[y2 * W + x2];

                // Sobel 梯度
                float gx = -p00 + p02 - 2.0f * p10 + 2.0f * p12 - p20 + p22;
                float gy = -p00 - 2.0f * p01 - p02 + p20 + 2.0f * p21 + p22;

                // 幅值
                float mag = std::sqrt(gx * gx + gy * gy);
                magData[y * magStride + x] = mag;

                // 量化
                if (mag < minMag) {
                    quantData[y * quantStride + x] = 0;
                } else {
                    // atan2 → 8 bins
                    double angle = std::atan2(gy, gx);
                    if (angle < 0) angle += 2.0 * PI;
                    int32_t bin16 = static_cast<int32_t>(angle * 16.0 / (2.0 * PI) + 0.5) % 16;
                    int32_t bin8 = bin16 & 7;
                    quantData[y * quantStride + x] = static_cast<uint8_t>(1 << bin8);
                }
            }
        }
    }

    // Pass 3: 投票 + OR扩散 (保持原有实现)
    ApplyNeighborVoting(level);
    ApplyORSpreading(level);

    // 提取特征 (模板创建时)
    if (params_.extractFeatures) {
        ExtractLevelFeatures(level);
    }

    return true;
}

void LinemodPyramid::ApplyNeighborVoting(LinemodLevelData& level) {
    // Paper: "Assign to each location the quantized value that occurs most
    //         often in a 3x3 (or 5x5) neighborhood"
    // "Only keep pixels where max_votes >= NEIGHBOR_THRESHOLD"

    const int32_t W = level.width;
    const int32_t H = level.height;
    const int32_t stride = static_cast<int32_t>(level.quantized.Stride());
    const uint8_t* src = static_cast<const uint8_t*>(level.quantized.Data());

    // Output to spread (will be replaced by OR spreading later)
    QImage voted(W, H, PixelType::UInt8, ChannelType::Gray);
    uint8_t* dst = static_cast<uint8_t*>(voted.Data());
    const int32_t dstStride = static_cast<int32_t>(voted.Stride());

    std::memset(dst, 0, voted.Height() * voted.Stride());

#pragma omp parallel for if(W * H > 100000)
    for (int32_t y = 1; y < H - 1; ++y) {
        for (int32_t x = 1; x < W - 1; ++x) {
            // Count votes for each orientation in 3x3 neighborhood
            std::array<int32_t, 8> votes = {0};
            int32_t totalVotes = 0;

            for (int32_t dy = -1; dy <= 1; ++dy) {
                for (int32_t dx = -1; dx <= 1; ++dx) {
                    uint8_t q = src[(y + dy) * stride + (x + dx)];
                    if (q == 0) continue;

                    // Count set bits (should be only one for valid pixels)
                    for (int32_t b = 0; b < 8; ++b) {
                        if (q & (1 << b)) {
                            votes[b]++;
                            totalVotes++;
                        }
                    }
                }
            }

            if (totalVotes == 0) {
                dst[y * dstStride + x] = 0;
                continue;
            }

            // Find max vote
            int32_t maxVotes = 0;
            int32_t maxBin = 0;
            for (int32_t b = 0; b < 8; ++b) {
                if (votes[b] > maxVotes) {
                    maxVotes = votes[b];
                    maxBin = b;
                }
            }

            // Threshold: only keep if max_votes >= threshold
            if (maxVotes >= params_.neighborThreshold) {
                dst[y * dstStride + x] = static_cast<uint8_t>(1 << maxBin);
            } else {
                dst[y * dstStride + x] = 0;
            }
        }
    }

    // Copy voted back to quantized
    level.quantized = std::move(voted);
}

void LinemodPyramid::ApplyORSpreading(LinemodLevelData& level) {
    // Paper: "Spread binary labels using OR operation"
    // spread[x] |= quantized[x + offset] for all offsets in T×T region

    const int32_t W = level.width;
    const int32_t H = level.height;
    const int32_t T = params_.spreadT;
    const int32_t srcStride = static_cast<int32_t>(level.quantized.Stride());
    const uint8_t* src = static_cast<const uint8_t*>(level.quantized.Data());

    level.spread = QImage(W, H, PixelType::UInt8, ChannelType::Gray);
    level.stride = static_cast<int32_t>(level.spread.Stride());
    uint8_t* dst = static_cast<uint8_t*>(level.spread.Data());
    const int32_t dstStride = level.stride;

    // Initialize with source
    std::memcpy(dst, src, H * dstStride);

    // OR spreading: spread each pixel's value to its T×T neighborhood
    // More efficient: for each offset, OR the shifted image
    for (int32_t dy = 0; dy < T; ++dy) {
        for (int32_t dx = 0; dx < T; ++dx) {
            if (dy == 0 && dx == 0) continue;

            // OR src[y+dy][x+dx] into dst[y][x]
#pragma omp parallel for if(W * H > 100000)
            for (int32_t y = 0; y < H - dy; ++y) {
                for (int32_t x = 0; x < W - dx; ++x) {
                    dst[y * dstStride + x] |= src[(y + dy) * srcStride + (x + dx)];
                }
            }
        }
    }

    // Also spread in negative direction for symmetry
    for (int32_t dy = 0; dy < T; ++dy) {
        for (int32_t dx = 0; dx < T; ++dx) {
            if (dy == 0 && dx == 0) continue;

            // OR src[y-dy][x-dx] into dst[y][x]
#pragma omp parallel for if(W * H > 100000)
            for (int32_t y = dy; y < H; ++y) {
                for (int32_t x = dx; x < W; ++x) {
                    dst[y * dstStride + x] |= src[(y - dy) * srcStride + (x - dx)];
                }
            }
        }
    }
}

void LinemodPyramid::ExtractLevelFeatures(LinemodLevelData& level) {
    // Extract all candidate features from this level
    const int32_t W = level.width;
    const int32_t H = level.height;
    const uint8_t* quantData = static_cast<const uint8_t*>(level.quantized.Data());
    const float* magData = static_cast<const float*>(level.gradMag.Data());
    const int32_t quantStride = static_cast<int32_t>(level.quantized.Stride());
    const int32_t magStride = static_cast<int32_t>(level.gradMag.Stride() / sizeof(float));

    level.features.clear();
    level.features.reserve(W * H / 16);  // Rough estimate

    for (int32_t y = 1; y < H - 1; ++y) {
        for (int32_t x = 1; x < W - 1; ++x) {
            uint8_t q = quantData[y * quantStride + x];
            if (q == 0) continue;

            float mag = magData[y * magStride + x];
            if (mag < params_.minMagnitude) continue;

            // Find the set bit (orientation)
            int32_t ori = 0;
            for (int32_t b = 0; b < 8; ++b) {
                if (q & (1 << b)) {
                    ori = b;
                    break;
                }
            }

            level.features.emplace_back(
                static_cast<int16_t>(x),
                static_cast<int16_t>(y),
                static_cast<uint8_t>(ori)
            );
        }
    }
}

std::vector<LinemodFeature> LinemodPyramid::ExtractFeatures(
    int32_t level, const Rect2i& roi, int32_t maxFeatures, float minDistance) const
{
    if (!valid_ || level < 0 || level >= numLevels_) {
        return {};
    }

    const auto& levelData = levels_[level];
    const int32_t W = levelData.width;
    const int32_t H = levelData.height;

    // Determine ROI
    Rect2i actualRoi = roi;
    if (actualRoi.width <= 0 || actualRoi.height <= 0) {
        actualRoi = Rect2i(0, 0, W, H);
    }

    // Clamp ROI
    actualRoi.x = std::max(0, actualRoi.x);
    actualRoi.y = std::max(0, actualRoi.y);
    actualRoi.width = std::min(actualRoi.width, W - actualRoi.x);
    actualRoi.height = std::min(actualRoi.height, H - actualRoi.y);

    // Center of ROI (features will be relative to this)
    float centerX = actualRoi.x + actualRoi.width / 2.0f;
    float centerY = actualRoi.y + actualRoi.height / 2.0f;

    // Collect candidates with magnitude
    struct Candidate {
        int16_t x, y;
        uint8_t ori;
        float mag;
    };
    std::vector<Candidate> candidates;

    const uint8_t* quantData = static_cast<const uint8_t*>(levelData.quantized.Data());
    const int32_t quantStride = static_cast<int32_t>(levelData.quantized.Stride());

    // gradMag 可能为空 (来自 BuildLevelFromQuantized 的层级没有 gradMag)
    const bool hasGradMag = !levelData.gradMag.Empty();
    const float* magData = hasGradMag ? static_cast<const float*>(levelData.gradMag.Data()) : nullptr;
    const int32_t magStride = hasGradMag ? static_cast<int32_t>(levelData.gradMag.Stride() / sizeof(float)) : 0;

    for (int32_t y = actualRoi.y; y < actualRoi.y + actualRoi.height; ++y) {
        for (int32_t x = actualRoi.x; x < actualRoi.x + actualRoi.width; ++x) {
            uint8_t q = quantData[y * quantStride + x];
            if (q == 0) continue;

            // 如果有 gradMag，使用它来过滤和排序；否则使用默认值
            float mag = hasGradMag ? magData[y * magStride + x] : params_.minMagnitude + 1.0f;
            if (hasGradMag && mag < params_.minMagnitude) continue;

            int32_t ori = 0;
            for (int32_t b = 0; b < 8; ++b) {
                if (q & (1 << b)) {
                    ori = b;
                    break;
                }
            }

            candidates.push_back({
                static_cast<int16_t>(x),
                static_cast<int16_t>(y),
                static_cast<uint8_t>(ori),
                mag
            });
        }
    }

    if (candidates.empty()) {
        return {};
    }

    // Sort by magnitude (strongest first)
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.mag > b.mag; });

    // Select scattered features using greedy algorithm
    std::vector<LinemodFeature> result;
    result.reserve(maxFeatures);

    const float minDistSq = minDistance * minDistance;

    for (const auto& c : candidates) {
        if (static_cast<int32_t>(result.size()) >= maxFeatures) break;

        // Check distance to existing features
        bool tooClose = false;
        for (const auto& f : result) {
            float dx = (c.x - centerX) - f.x;
            float dy = (c.y - centerY) - f.y;
            if (dx * dx + dy * dy < minDistSq) {
                tooClose = true;
                break;
            }
        }

        if (!tooClose) {
            result.emplace_back(
                static_cast<int16_t>(c.x - centerX),
                static_cast<int16_t>(c.y - centerY),
                c.ori
            );
        }
    }

    return result;
}

const std::vector<LinemodFeature>& LinemodPyramid::GetAllFeatures(int32_t level) const {
    static const std::vector<LinemodFeature> empty;
    if (!valid_ || level < 0 || level >= numLevels_) {
        return empty;
    }
    return levels_[level].features;
}

double LinemodPyramid::ComputeScore(const std::vector<LinemodFeature>& features,
                                     int32_t level, int32_t x, int32_t y) const
{
    if (!valid_ || level < 0 || level >= numLevels_ || features.empty()) {
        return 0.0;
    }

    const auto& levelData = levels_[level];
    const int32_t W = levelData.width;
    const int32_t H = levelData.height;
    const uint8_t* spreadData = static_cast<const uint8_t*>(levelData.spread.Data());
    const int32_t spreadStride = static_cast<int32_t>(levelData.spread.Stride());

    int32_t totalScore = 0;
    int32_t validCount = 0;

    for (const auto& f : features) {
        int32_t fx = x + f.x;
        int32_t fy = y + f.y;

        // Bounds check
        if (fx < 0 || fx >= W || fy < 0 || fy >= H) {
            continue;
        }

        // Get spread bitmask at this location
        uint8_t mask = spreadData[fy * spreadStride + fx];

        // Look up similarity using SIMILARITY_LUT
        uint8_t sim = g_SimilarityLUT.Get(f.ori, mask);
        totalScore += sim;
        validCount++;
    }

    if (validCount == 0) {
        return 0.0;
    }

    // Normalize to [0, 1]
    // Max score per feature is LINEMOD_MAX_RESPONSE (4)
    return static_cast<double>(totalScore) / (validCount * LINEMOD_MAX_RESPONSE);
}

double LinemodPyramid::ComputeScoreRotated(const std::vector<LinemodFeature>& features,
                                            int32_t level, int32_t x, int32_t y,
                                            double angle) const
{
    if (!valid_ || level < 0 || level >= numLevels_ || features.empty()) {
        return 0.0;
    }

    const auto& levelData = levels_[level];
    const int32_t W = levelData.width;
    const int32_t H = levelData.height;
    const uint8_t* spreadData = static_cast<const uint8_t*>(levelData.spread.Data());
    const int32_t spreadStride = static_cast<int32_t>(levelData.spread.Stride());

    const double cosA = std::cos(angle);
    const double sinA = std::sin(angle);

    // Precompute rotation bin offset (how many 45° steps)
    // angle in radians -> bins
    int32_t rotBinOffset = static_cast<int32_t>(std::round(angle * 8.0 / (2.0 * PI)));
    rotBinOffset = ((rotBinOffset % 8) + 8) % 8;  // Normalize to [0, 7]

    int32_t totalScore = 0;
    int32_t validCount = 0;

    for (const auto& f : features) {
        // Rotate feature coordinates
        double rx = cosA * f.x - sinA * f.y;
        double ry = sinA * f.x + cosA * f.y;

        int32_t fx = x + static_cast<int32_t>(std::round(rx));
        int32_t fy = y + static_cast<int32_t>(std::round(ry));

        // Bounds check
        if (fx < 0 || fx >= W || fy < 0 || fy >= H) {
            continue;
        }

        // Rotate orientation bin
        int32_t rotatedOri = (f.ori + rotBinOffset) & 7;

        // Get spread bitmask at this location
        uint8_t mask = spreadData[fy * spreadStride + fx];

        // Look up similarity using SIMILARITY_LUT
        uint8_t sim = g_SimilarityLUT.Get(rotatedOri, mask);
        totalScore += sim;
        validCount++;
    }

    if (validCount == 0) {
        return 0.0;
    }

    // Normalize to [0, 1]
    return static_cast<double>(totalScore) / (validCount * LINEMOD_MAX_RESPONSE);
}

double LinemodPyramid::ComputeScorePrecomputed(const std::vector<LinemodFeature>& rotatedFeatures,
                                               int32_t level, int32_t x, int32_t y) const
{
    if (!valid_ || level < 0 || level >= numLevels_ || rotatedFeatures.empty()) {
        return 0.0;
    }

    const auto& levelData = levels_[level];
    const int32_t W = levelData.width;
    const int32_t H = levelData.height;
    const uint8_t* spreadData = static_cast<const uint8_t*>(levelData.spread.Data());
    const int32_t spreadStride = static_cast<int32_t>(levelData.spread.Stride());

    int32_t matchCount = 0;
    int32_t validCount = 0;

    // Bit operation method: similarity = (spread_mask & model_bit) ? 1 : 0
    // Since spread map already contains OR-spread orientations, this gives
    // tolerance for small angle differences.
    for (const auto& f : rotatedFeatures) {
        int32_t fx = x + f.x;
        int32_t fy = y + f.y;

        // Bounds check
        if (fx < 0 || fx >= W || fy < 0 || fy >= H) {
            continue;
        }

        // Get spread bitmask at this location
        uint8_t spreadMask = spreadData[fy * spreadStride + fx];

        // Model orientation as bit flag
        uint8_t modelBit = static_cast<uint8_t>(1 << f.ori);

        // Bit AND: check if model orientation exists in spread mask
        if (spreadMask & modelBit) {
            matchCount++;
        }
        validCount++;
    }

    if (validCount == 0) {
        return 0.0;
    }

    // Normalize to [0, 1]: matchCount / validCount
    return static_cast<double>(matchCount) / validCount;
}

void LinemodPyramid::ComputeScoresBatch8(const std::vector<LinemodFeature>& rotatedFeatures,
                                          int32_t level, int32_t x, int32_t y,
                                          double* scoresOut) const
{
    // Initialize outputs to 0
    for (int32_t i = 0; i < 8; ++i) {
        scoresOut[i] = 0.0;
    }

    if (!valid_ || level < 0 || level >= numLevels_ || rotatedFeatures.empty()) {
        return;
    }

    const auto& levelData = levels_[level];
    const int32_t W = levelData.width;
    const int32_t H = levelData.height;
    const uint8_t* spreadData = static_cast<const uint8_t*>(levelData.spread.Data());
    const int32_t spreadStride = static_cast<int32_t>(levelData.spread.Stride());

    alignas(32) int32_t matchCounts8[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int32_t validCount = 0;

#ifdef __AVX2__
    // ═══════════════════════════════════════════════════════════════
    // 正确的 AVX2 实现：利用 spread 数据的连续性
    // spread[y][x], spread[y][x+1], ..., spread[y][x+7] 是连续存储的
    // ═══════════════════════════════════════════════════════════════

    __m256i vCounts = _mm256_setzero_si256();
    const __m256i vOne = _mm256_set1_epi32(1);
    const __m256i vZero = _mm256_setzero_si256();

    for (const auto& f : rotatedFeatures) {
        int32_t fy = y + f.y;
        int32_t fx = x + f.x;  // 起始 x 位置

        // 边界检查：需要 8 个连续位置都在范围内
        if (fy < 0 || fy >= H || fx < 0 || fx + 7 >= W) {
            continue;
        }

        validCount++;

        // ⭐关键：一次读取 8 个连续的 spread 字节
        const uint8_t* ptr = spreadData + fy * spreadStride + fx;
        __m128i spread8 = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(ptr));

        // 扩展为 8 × int32（为了后续比较）
        __m256i spread32 = _mm256_cvtepu8_epi32(spread8);

        // 广播 modelBit 到 8 个 int32
        __m256i model32 = _mm256_set1_epi32(1 << f.ori);

        // ⭐关键：一次 AND 操作处理 8 个位置
        __m256i matched = _mm256_and_si256(spread32, model32);

        // 非零检测: matched != 0 ? 1 : 0
        __m256i isNonZero = _mm256_cmpgt_epi32(matched, vZero);
        __m256i increment = _mm256_and_si256(isNonZero, vOne);

        // 累加到计数器
        vCounts = _mm256_add_epi32(vCounts, increment);
    }

    // 存储结果
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(matchCounts8), vCounts);

#else
    // 标量 fallback
    for (const auto& f : rotatedFeatures) {
        int32_t fy = y + f.y;
        int32_t fx_base = x + f.x;

        if (fy < 0 || fy >= H) continue;

        const uint8_t* rowPtr = spreadData + fy * spreadStride;
        uint8_t modelBit = static_cast<uint8_t>(1 << f.ori);

        // 检查是否所有 8 个位置都有效
        if (fx_base >= 0 && fx_base + 7 < W) {
            validCount++;
            for (int32_t i = 0; i < 8; ++i) {
                if (rowPtr[fx_base + i] & modelBit) {
                    matchCounts8[i]++;
                }
            }
        }
    }
#endif

    // 计算分数：matchCount / validCount
    if (validCount > 0) {
        for (int32_t i = 0; i < 8; ++i) {
            scoresOut[i] = static_cast<double>(matchCounts8[i]) / validCount;
        }
    }
}

void LinemodPyramid::ComputeScoresRow(const std::vector<LinemodFeature>& rotatedFeatures,
                                       int32_t level, int32_t xStart, int32_t xEnd, int32_t y,
                                       int32_t stride, double threshold,
                                       std::vector<int32_t>& xPositionsOut,
                                       std::vector<double>& scoresOut) const
{
    xPositionsOut.clear();
    scoresOut.clear();

    if (!valid_ || level < 0 || level >= numLevels_ || rotatedFeatures.empty()) {
        return;
    }

    const auto& levelData = levels_[level];
    const int32_t W = levelData.width;
    const int32_t H = levelData.height;
    const uint8_t* spreadData = static_cast<const uint8_t*>(levelData.spread.Data());
    const int32_t spreadStride = static_cast<int32_t>(levelData.spread.Stride());
    const int32_t numFeatures = static_cast<int32_t>(rotatedFeatures.size());

    // Precompute model bits for all features
    alignas(32) uint8_t modelBits[512];  // Max 512 features
    const int32_t actualFeatures = std::min(numFeatures, 512);
    for (int32_t i = 0; i < actualFeatures; ++i) {
        modelBits[i] = static_cast<uint8_t>(1 << rotatedFeatures[i].ori);
    }

    // Calculate threshold as integer count
    const int32_t thresholdCount = static_cast<int32_t>(threshold * numFeatures);

    // Process positions in batches of 16 using AVX2
    const int32_t numPositions = (xEnd - xStart) / stride + 1;

#ifdef __AVX2__
    // Process 16 positions at a time
    alignas(64) int32_t matchCounts[16];
    alignas(64) int32_t validCounts[16];

    int32_t posIdx = 0;
    for (; posIdx + 15 < numPositions; posIdx += 16) {
        // Initialize counts
        __m256i vMatch0 = _mm256_setzero_si256();
        __m256i vMatch1 = _mm256_setzero_si256();
        __m256i vValid0 = _mm256_setzero_si256();
        __m256i vValid1 = _mm256_setzero_si256();
        const __m256i vOne = _mm256_set1_epi32(1);

        // Calculate X positions for these 16 slots
        int32_t xPos[16];
        for (int32_t i = 0; i < 16; ++i) {
            xPos[i] = xStart + (posIdx + i) * stride;
        }

        // Process each feature
        for (int32_t fi = 0; fi < actualFeatures; ++fi) {
            const auto& f = rotatedFeatures[fi];
            int32_t fy = y + f.y;

            if (fy < 0 || fy >= H) continue;

            const uint8_t* rowPtr = spreadData + fy * spreadStride;
            uint8_t modelBit = modelBits[fi];

            // Gather spread values for 16 positions and check matches
            alignas(64) int32_t localMatch[16] = {0};
            alignas(64) int32_t localValid[16] = {0};

            for (int32_t i = 0; i < 16; ++i) {
                int32_t fx = xPos[i] + f.x;
                if (fx >= 0 && fx < W) {
                    if (rowPtr[fx] & modelBit) {
                        localMatch[i] = 1;
                    }
                    localValid[i] = 1;
                }
            }

            // Accumulate using SIMD
            __m256i vLM0 = _mm256_load_si256(reinterpret_cast<const __m256i*>(localMatch));
            __m256i vLM1 = _mm256_load_si256(reinterpret_cast<const __m256i*>(localMatch + 8));
            __m256i vLV0 = _mm256_load_si256(reinterpret_cast<const __m256i*>(localValid));
            __m256i vLV1 = _mm256_load_si256(reinterpret_cast<const __m256i*>(localValid + 8));

            vMatch0 = _mm256_add_epi32(vMatch0, vLM0);
            vMatch1 = _mm256_add_epi32(vMatch1, vLM1);
            vValid0 = _mm256_add_epi32(vValid0, vLV0);
            vValid1 = _mm256_add_epi32(vValid1, vLV1);
        }

        // Store counts
        _mm256_store_si256(reinterpret_cast<__m256i*>(matchCounts), vMatch0);
        _mm256_store_si256(reinterpret_cast<__m256i*>(matchCounts + 8), vMatch1);
        _mm256_store_si256(reinterpret_cast<__m256i*>(validCounts), vValid0);
        _mm256_store_si256(reinterpret_cast<__m256i*>(validCounts + 8), vValid1);

        // Check threshold and add passing positions
        for (int32_t i = 0; i < 16; ++i) {
            if (validCounts[i] > 0 && matchCounts[i] >= thresholdCount) {
                double score = static_cast<double>(matchCounts[i]) / validCounts[i];
                if (score >= threshold) {
                    xPositionsOut.push_back(xPos[i]);
                    scoresOut.push_back(score);
                }
            }
        }
    }

    // Handle remaining positions
    for (; posIdx < numPositions; ++posIdx) {
        int32_t x = xStart + posIdx * stride;
        int32_t matchCount = 0;
        int32_t validCount = 0;

        for (int32_t fi = 0; fi < actualFeatures; ++fi) {
            const auto& f = rotatedFeatures[fi];
            int32_t fx = x + f.x;
            int32_t fy = y + f.y;

            if (fx < 0 || fx >= W || fy < 0 || fy >= H) continue;

            uint8_t spreadMask = spreadData[fy * spreadStride + fx];
            if (spreadMask & modelBits[fi]) {
                matchCount++;
            }
            validCount++;
        }

        if (validCount > 0 && matchCount >= thresholdCount) {
            double score = static_cast<double>(matchCount) / validCount;
            if (score >= threshold) {
                xPositionsOut.push_back(x);
                scoresOut.push_back(score);
            }
        }
    }

#else
    // Scalar fallback
    for (int32_t x = xStart; x <= xEnd; x += stride) {
        int32_t matchCount = 0;
        int32_t validCount = 0;

        for (int32_t fi = 0; fi < actualFeatures; ++fi) {
            const auto& f = rotatedFeatures[fi];
            int32_t fx = x + f.x;
            int32_t fy = y + f.y;

            if (fx < 0 || fx >= W || fy < 0 || fy >= H) continue;

            uint8_t spreadMask = spreadData[fy * spreadStride + fx];
            if (spreadMask & modelBits[fi]) {
                matchCount++;
            }
            validCount++;
        }

        if (validCount > 0 && matchCount >= thresholdCount) {
            double score = static_cast<double>(matchCount) / validCount;
            if (score >= threshold) {
                xPositionsOut.push_back(x);
                scoresOut.push_back(score);
            }
        }
    }
#endif
}

LinemodFeature LinemodPyramid::RotateFeature(const LinemodFeature& f, double angle) {
    const double cosA = std::cos(angle);
    const double sinA = std::sin(angle);

    // Rotate coordinates
    double rx = cosA * f.x - sinA * f.y;
    double ry = sinA * f.x + cosA * f.y;

    // Rotate orientation bin
    int32_t rotBinOffset = static_cast<int32_t>(std::round(angle * 8.0 / (2.0 * PI)));
    int32_t rotatedOri = ((f.ori + rotBinOffset) % 8 + 8) % 8;

    return LinemodFeature(
        static_cast<int16_t>(std::round(rx)),
        static_cast<int16_t>(std::round(ry)),
        static_cast<uint8_t>(rotatedOri)
    );
}

std::vector<LinemodFeature> LinemodPyramid::RotateFeatures(
    const std::vector<LinemodFeature>& features, double angle)
{
    std::vector<LinemodFeature> result;
    result.reserve(features.size());

    for (const auto& f : features) {
        result.push_back(RotateFeature(f, angle));
    }

    return result;
}

const uint8_t* LinemodPyramid::GetSpreadData(int32_t level) const {
    if (!valid_ || level < 0 || level >= numLevels_) {
        return nullptr;
    }
    return static_cast<const uint8_t*>(levels_[level].spread.Data());
}

int32_t LinemodPyramid::GetSpreadStride(int32_t level) const {
    if (!valid_ || level < 0 || level >= numLevels_) {
        return 0;
    }
    return static_cast<int32_t>(levels_[level].spread.Stride());
}

int32_t LinemodPyramid::GetWidth(int32_t level) const {
    if (!valid_ || level < 0 || level >= numLevels_) return 0;
    return levels_[level].width;
}

int32_t LinemodPyramid::GetHeight(int32_t level) const {
    if (!valid_ || level < 0 || level >= numLevels_) return 0;
    return levels_[level].height;
}

double LinemodPyramid::GetScale(int32_t level) const {
    if (!valid_ || level < 0 || level >= numLevels_) return 1.0;
    return levels_[level].scale;
}

const LinemodLevelData& LinemodPyramid::GetLevel(int32_t level) const {
    static const LinemodLevelData empty;
    if (!valid_ || level < 0 || level >= numLevels_) return empty;
    return levels_[level];
}

} // namespace Qi::Vision::Internal
