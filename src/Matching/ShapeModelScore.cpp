/**
 * @file ShapeModelScore.cpp
 * @brief Score computation functions for ShapeModel
 *
 * Contains:
 * - FastCosTable
 * - ComputeScoreAtPosition
 * - ComputeScoreBilinearSSE
 * - ComputeScoreWithSinCos
 * - ComputeScoreQuantized
 * - RefinePosition
 */

#include "ShapeModelImpl.h"

#include <cmath>
#include <algorithm>

namespace Qi::Vision::Matching {
namespace Internal {

// =============================================================================
// FastCosTable Implementation
// =============================================================================

FastCosTable::FastCosTable() {
    for (int i = 0; i < TABLE_SIZE; ++i) {
        double angle = (i * 2.0 * PI) / TABLE_SIZE;
        table_[i] = static_cast<float>(std::cos(angle));
    }
}

float FastCosTable::FastCos(double angle) const {
    angle = angle - std::floor(angle * INV_2PI) * (2.0 * PI);
    int idx = static_cast<int>(angle * TABLE_SCALE) & TABLE_MASK;
    return table_[idx];
}

float FastCosTable::FastAbsCos(double angleDiff) const {
    angleDiff = std::fabs(angleDiff);
    angleDiff = angleDiff - std::floor(angleDiff * INV_PI) * PI;
    int idx = static_cast<int>(angleDiff * TABLE_SCALE) & TABLE_MASK;
    return std::fabs(table_[idx]);
}

// Global cosine lookup table
const FastCosTable g_cosTable;

// =============================================================================
// ShapeModelImpl::ComputeScoreAtPosition
// =============================================================================

double ShapeModelImpl::ComputeScoreAtPosition(
    const AnglePyramid& pyramid, int32_t level,
    double x, double y, double angle, double scale,
    double greediness, double* outCoverage, bool useGridPoints) const
{
    // Fast path: use SSE optimized scalar version for common cases
    if (params_.metric == MetricMode::IgnoreLocalPolarity ||
        params_.metric == MetricMode::IgnoreColorPolarity) {
        return ComputeScoreBilinearSSE(pyramid, level, x, y, angle, scale, greediness, outCoverage, useGridPoints);
    }

    // Slow path: handle UsePolarity and IgnoreGlobalPolarity
    if (level < 0 || level >= static_cast<int32_t>(levels_.size())) {
        if (outCoverage) *outCoverage = 0.0;
        return 0.0;
    }

    const auto& levelModel = levels_[level];
    const auto& pts = useGridPoints ? levelModel.gridPoints : levelModel.points;
    if (pts.empty()) {
        if (outCoverage) *outCoverage = 0.0;
        return 0.0;
    }

    const float* gxData;
    const float* gyData;
    int32_t width, height, stride;
    if (!pyramid.GetGradientData(level, gxData, gyData, width, height, stride)) {
        if (outCoverage) *outCoverage = 0.0;
        return 0.0;
    }

    const float cosR = static_cast<float>(std::cos(angle));
    const float sinR = static_cast<float>(std::sin(angle));
    const float scalef = static_cast<float>(scale);
    const float xf = static_cast<float>(x);
    const float yf = static_cast<float>(y);

    const float* soaX = useGridPoints ? levelModel.gridSoaX.data() : levelModel.soaX.data();
    const float* soaY = useGridPoints ? levelModel.gridSoaY.data() : levelModel.soaY.data();
    const float* soaCos = useGridPoints ? levelModel.gridSoaCosAngle.data() : levelModel.soaCosAngle.data();
    const float* soaSin = useGridPoints ? levelModel.gridSoaSinAngle.data() : levelModel.soaSinAngle.data();
    const float* soaWeight = useGridPoints ? levelModel.gridSoaWeight.data() : levelModel.soaWeight.data();

    float totalScore = 0.0f;
    float totalWeight = 0.0f;
    int32_t matchedCount = 0;
    const size_t numPoints = pts.size();

    float totalScoreInverted = 0.0f;
    float totalWeightInverted = 0.0f;
    int32_t matchedCountInverted = 0;

    const float earlyTermThreshold = static_cast<float>(greediness * numPoints);
    const int32_t maxX = width - 2;
    const int32_t maxY = height - 2;

    // Precompute minMagSq outside the loop (constant for all points)
    const float baseMin = (params_.minContrast > 0.0) ? static_cast<float>(params_.minContrast) : 5.0f;
    const float levelScale = static_cast<float>(pyramid.GetScale(level));
    const float minMag = std::max(2.0f, baseMin * levelScale);
    const float minMagSq = minMag * minMag;

    for (size_t i = 0; i < numPoints; ++i) {
        float rotX = cosR * soaX[i] - sinR * soaY[i];
        float rotY = sinR * soaX[i] + cosR * soaY[i];
        float imgX = xf + scalef * rotX;
        float imgY = yf + scalef * rotY;

        int32_t ix = static_cast<int32_t>(imgX);
        int32_t iy = static_cast<int32_t>(imgY);
        if (imgX < 0) ix--;
        if (imgY < 0) iy--;

        if (ix >= 0 && ix <= maxX && iy >= 0 && iy <= maxY) {
            float fx = imgX - ix;
            float fy = imgY - iy;
            float w00 = (1.0f - fx) * (1.0f - fy);
            float w10 = fx * (1.0f - fy);
            float w01 = (1.0f - fx) * fy;
            float w11 = fx * fy;

            int32_t idx = iy * stride + ix;
            float gx = w00 * gxData[idx] + w10 * gxData[idx + 1] +
                       w01 * gxData[idx + stride] + w11 * gxData[idx + stride + 1];
            float gy = w00 * gyData[idx] + w10 * gyData[idx + 1] +
                       w01 * gyData[idx + stride] + w11 * gyData[idx + stride + 1];

            float magSq = gx * gx + gy * gy;
            if (magSq >= minMagSq) {
                float rotCos = soaCos[i] * cosR - soaSin[i] * sinR;
                float rotSin = soaSin[i] * cosR + soaCos[i] * sinR;
                float dot = rotCos * gx + rotSin * gy;
                float mag = std::sqrt(magSq);
                float similarity = dot / mag;
                float simPos = std::max(0.0f, similarity);
                float simNeg = std::max(0.0f, -similarity);

                if (params_.metric == MetricMode::UsePolarity) {
                    float score = 0.0f;
                    if (params_.polarity == MatchPolarity::Opposite) {
                        score = simNeg;
                    } else if (params_.polarity == MatchPolarity::Any) {
                        score = std::max(simPos, simNeg);
                    } else {
                        score = simPos;
                    }

                    if (score > 0.0f) {
                        totalScore += soaWeight[i] * score;
                        totalWeight += soaWeight[i];
                        matchedCount++;
                    }
                } else {
                    if (params_.polarity == MatchPolarity::Opposite) {
                        if (simNeg > 0.0f) {
                            totalScore += soaWeight[i] * simNeg;
                            totalWeight += soaWeight[i];
                            matchedCount++;
                        }
                    } else if (params_.polarity == MatchPolarity::Any) {
                        totalScore += soaWeight[i] * simPos;
                        totalWeight += soaWeight[i];
                        matchedCount++;

                        totalScoreInverted += soaWeight[i] * simNeg;
                        totalWeightInverted += soaWeight[i];
                        matchedCountInverted++;
                    } else {
                        if (simPos > 0.0f) {
                            totalScore += soaWeight[i] * simPos;
                            totalWeight += soaWeight[i];
                            matchedCount++;
                        }
                    }
                }
            }
        }

        if (greediness > 0.0f && ((i + 1) & 15) == 0 && totalWeight > 0) {
            float currentAvg = totalScore / totalWeight;
            if (currentAvg * numPoints < earlyTermThreshold) {
                if (outCoverage) *outCoverage = static_cast<double>(matchedCount) / numPoints;
                return 0.0;
            }
        }
    }

    if (params_.metric == MetricMode::IgnoreGlobalPolarity) {
        float scoreNormal = (totalWeight > 0) ? totalScore / totalWeight : 0.0f;
        float scoreInverted = (totalWeightInverted > 0) ? totalScoreInverted / totalWeightInverted : 0.0f;

        if (scoreInverted > scoreNormal) {
            if (outCoverage) *outCoverage = static_cast<double>(matchedCountInverted) / numPoints;
            return static_cast<double>(scoreInverted);
        } else {
            if (outCoverage) *outCoverage = static_cast<double>(matchedCount) / numPoints;
            return static_cast<double>(scoreNormal);
        }
    }

    double coverage = static_cast<double>(matchedCount) / numPoints;
    if (outCoverage) *outCoverage = coverage;
    if (totalWeight <= 0) return 0.0;

    // Return pure similarity score (no coverage penalty)
    // Coverage penalty is applied at final output stage in SearchPyramid
    return static_cast<double>(totalScore) / totalWeight;
}

// =============================================================================
// ShapeModelImpl::ComputeScoreBilinearSSE
// =============================================================================

double ShapeModelImpl::ComputeScoreBilinearSSE(
    const AnglePyramid& pyramid, int32_t level,
    double x, double y, double angle, double scale,
    double greediness, double* outCoverage, bool useGridPoints) const
{
    const auto& levelModel = levels_[level];
    const auto& pts = useGridPoints ? levelModel.gridPoints : levelModel.points;
    const size_t numPoints = pts.size();
    if (numPoints == 0) {
        if (outCoverage) *outCoverage = 0.0;
        return 0.0;
    }

    const float* gxData;
    const float* gyData;
    int32_t width, height, stride;
    if (!pyramid.GetGradientData(level, gxData, gyData, width, height, stride)) {
        if (outCoverage) *outCoverage = 0.0;
        return 0.0;
    }

    const float cosR = static_cast<float>(std::cos(angle));
    const float sinR = static_cast<float>(std::sin(angle));
    const float scalef = static_cast<float>(scale);
    const float xf = static_cast<float>(x);
    const float yf = static_cast<float>(y);

    float baseMin = (params_.minContrast > 0.0) ? static_cast<float>(params_.minContrast) : 5.0f;
    float levelScale = static_cast<float>(pyramid.GetScale(level));
    float minMag = std::max(2.0f, baseMin * levelScale);
    const __m128 vMinMagSq = _mm_set_ss(minMag * minMag);

    float totalScore = 0.0f;
    float totalWeight = 0.0f;
    int32_t matchedCount = 0;

    const float earlyTermThreshold = static_cast<float>(greediness * numPoints);
    const int32_t checkInterval = 8;

    const float* soaX = useGridPoints ? levelModel.gridSoaX.data() : levelModel.soaX.data();
    const float* soaY = useGridPoints ? levelModel.gridSoaY.data() : levelModel.soaY.data();
    const float* soaCos = useGridPoints ? levelModel.gridSoaCosAngle.data() : levelModel.soaCosAngle.data();
    const float* soaSin = useGridPoints ? levelModel.gridSoaSinAngle.data() : levelModel.soaSinAngle.data();
    const float* soaWeight = useGridPoints ? levelModel.gridSoaWeight.data() : levelModel.soaWeight.data();

    const int32_t maxX = width - 2;
    const int32_t maxY = height - 2;

    for (size_t i = 0; i < numPoints; ++i) {
        float rotX = cosR * soaX[i] - sinR * soaY[i];
        float rotY = sinR * soaX[i] + cosR * soaY[i];
        float imgX = xf + scalef * rotX;
        float imgY = yf + scalef * rotY;

        int32_t ix = static_cast<int32_t>(imgX);
        int32_t iy = static_cast<int32_t>(imgY);
        if (imgX < 0) ix--;
        if (imgY < 0) iy--;

        if (ix >= 0 && ix <= maxX && iy >= 0 && iy <= maxY) {
            float fx = imgX - ix;
            float fy = imgY - iy;

            float w00 = (1.0f - fx) * (1.0f - fy);
            float w10 = fx * (1.0f - fy);
            float w01 = (1.0f - fx) * fy;
            float w11 = fx * fy;

            int32_t idx = iy * stride + ix;

            float g00 = gxData[idx];     float g10 = gxData[idx + 1];
            float g01 = gxData[idx + stride]; float g11 = gxData[idx + stride + 1];
            float gx = w00 * g00 + w10 * g10 + w01 * g01 + w11 * g11;

            float h00 = gyData[idx];     float h10 = gyData[idx + 1];
            float h01 = gyData[idx + stride]; float h11 = gyData[idx + stride + 1];
            float gy = w00 * h00 + w10 * h10 + w01 * h01 + w11 * h11;

            __m128 vGx = _mm_set_ss(gx);
            __m128 vGy = _mm_set_ss(gy);
            __m128 vMagSq = _mm_add_ss(_mm_mul_ss(vGx, vGx), _mm_mul_ss(vGy, vGy));

            if (_mm_comige_ss(vMagSq, vMinMagSq)) {
                float rotCos = soaCos[i] * cosR - soaSin[i] * sinR;
                float rotSin = soaSin[i] * cosR + soaCos[i] * sinR;
                float dot = rotCos * gx + rotSin * gy;

                __m128 vInvMag = _mm_rsqrt_ss(vMagSq);
                float invMag = 0.0f;
                _mm_store_ss(&invMag, vInvMag);

                float score = 0.0f;
                if (params_.metric == MetricMode::UsePolarity) {
                    if (dot > 0.0f) {
                        score = dot * invMag;
                    }
                } else {
                    score = std::fabs(dot) * invMag;
                }

                totalScore += soaWeight[i] * score;
                totalWeight += soaWeight[i];
                matchedCount++;
            }
        }

        if (greediness > 0.0 && ((i + 1) & (checkInterval - 1)) == 0) {
            if (totalWeight > 0) {
                if ((totalScore / totalWeight) * numPoints < earlyTermThreshold * 0.85f) {
                    if (outCoverage) *outCoverage = static_cast<double>(matchedCount) / numPoints;
                    return 0.0;
                }
            }
        }
    }

    double coverage = static_cast<double>(matchedCount) / numPoints;
    if (outCoverage) *outCoverage = coverage;
    if (totalWeight <= 0) return 0.0;

    // Return pure similarity score (no coverage penalty)
    return static_cast<double>(totalScore) / totalWeight;
}

// =============================================================================
// ShapeModelImpl::ComputeScoreWithSinCos
// =============================================================================

double ShapeModelImpl::ComputeScoreWithSinCos(
    const AnglePyramid& pyramid, int32_t level,
    double x, double y, float cosR, float sinR, double scale,
    double greediness, double* outCoverage, bool useGridPoints,
    float minMagSq) const
{
    const auto& levelModel = levels_[level];
    const auto& pts = useGridPoints ? levelModel.gridPoints : levelModel.points;
    const size_t numPoints = pts.size();
    if (numPoints == 0) {
        if (outCoverage) *outCoverage = 0.0;
        return 0.0;
    }

    const float* gxData;
    const float* gyData;
    int32_t width, height, stride;
    if (!pyramid.GetGradientData(level, gxData, gyData, width, height, stride)) {
        if (outCoverage) *outCoverage = 0.0;
        return 0.0;
    }

    const float scalef = static_cast<float>(scale);
    const float xf = static_cast<float>(x);
    const float yf = static_cast<float>(y);

    const __m128 vMinMagSq = _mm_set_ss(minMagSq);

    float totalScore = 0.0f;
    float totalWeight = 0.0f;
    int32_t matchedCount = 0;

    const float earlyTermThreshold = static_cast<float>(greediness * numPoints);
    const int32_t checkInterval = 8;

    const float* soaX = useGridPoints ? levelModel.gridSoaX.data() : levelModel.soaX.data();
    const float* soaY = useGridPoints ? levelModel.gridSoaY.data() : levelModel.soaY.data();
    const float* soaCos = useGridPoints ? levelModel.gridSoaCosAngle.data() : levelModel.soaCosAngle.data();
    const float* soaSin = useGridPoints ? levelModel.gridSoaSinAngle.data() : levelModel.soaSinAngle.data();
    const float* soaWeight = useGridPoints ? levelModel.gridSoaWeight.data() : levelModel.soaWeight.data();

    const int32_t maxX = width - 2;
    const int32_t maxY = height - 2;

    for (size_t i = 0; i < numPoints; ++i) {
        float rotX = cosR * soaX[i] - sinR * soaY[i];
        float rotY = sinR * soaX[i] + cosR * soaY[i];
        float imgX = xf + scalef * rotX;
        float imgY = yf + scalef * rotY;

        int32_t ix = static_cast<int32_t>(imgX);
        int32_t iy = static_cast<int32_t>(imgY);
        if (imgX < 0) ix--;
        if (imgY < 0) iy--;

        if (ix >= 0 && ix <= maxX && iy >= 0 && iy <= maxY) {
            float fx = imgX - ix;
            float fy = imgY - iy;

            float w00 = (1.0f - fx) * (1.0f - fy);
            float w10 = fx * (1.0f - fy);
            float w01 = (1.0f - fx) * fy;
            float w11 = fx * fy;

            int32_t idx = iy * stride + ix;

            float g00 = gxData[idx];     float g10 = gxData[idx + 1];
            float g01 = gxData[idx + stride]; float g11 = gxData[idx + stride + 1];
            float gx = w00 * g00 + w10 * g10 + w01 * g01 + w11 * g11;

            float h00 = gyData[idx];     float h10 = gyData[idx + 1];
            float h01 = gyData[idx + stride]; float h11 = gyData[idx + stride + 1];
            float gy = w00 * h00 + w10 * h10 + w01 * h01 + w11 * h11;

            __m128 vGx = _mm_set_ss(gx);
            __m128 vGy = _mm_set_ss(gy);
            __m128 vMagSq = _mm_add_ss(_mm_mul_ss(vGx, vGx), _mm_mul_ss(vGy, vGy));

            if (_mm_comige_ss(vMagSq, vMinMagSq)) {
                float rotCos = soaCos[i] * cosR - soaSin[i] * sinR;
                float rotSin = soaSin[i] * cosR + soaCos[i] * sinR;
                float dot = rotCos * gx + rotSin * gy;
                float mag = std::sqrt(_mm_cvtss_f32(vMagSq));
                float similarity = dot / mag;
                float simPos = std::max(0.0f, similarity);
                float simNeg = std::max(0.0f, -similarity);

                float score = 0.0f;
                if (params_.metric == MetricMode::UsePolarity) {
                    if (params_.polarity == MatchPolarity::Opposite) {
                        score = simNeg;
                    } else if (params_.polarity == MatchPolarity::Any) {
                        score = std::max(simPos, simNeg);
                    } else {
                        score = simPos;
                    }
                } else {
                    if (params_.polarity == MatchPolarity::Opposite) {
                        score = simNeg;
                    } else if (params_.polarity == MatchPolarity::Any) {
                        score = std::max(simPos, simNeg);
                    } else {
                        score = simPos;
                    }
                }

                if (score > 0.0f) {
                    totalScore += soaWeight[i] * score;
                    totalWeight += soaWeight[i];
                    matchedCount++;
                }
            }
        }

        if (greediness > 0.0 && ((i + 1) & (checkInterval - 1)) == 0) {
            if (totalWeight > 0) {
                if ((totalScore / totalWeight) * numPoints < earlyTermThreshold * 0.85f) {
                    if (outCoverage) *outCoverage = static_cast<double>(matchedCount) / numPoints;
                    return 0.0;
                }
            }
        }
    }

    double coverage = static_cast<double>(matchedCount) / numPoints;
    if (outCoverage) *outCoverage = coverage;
    if (totalWeight <= 0) return 0.0;

    // Return pure similarity score (no coverage penalty)
    return static_cast<double>(totalScore) / totalWeight;
}

// =============================================================================
// ShapeModelImpl::ComputeScoreNearestNeighbor
// =============================================================================

double ShapeModelImpl::ComputeScoreNearestNeighbor(
    const AnglePyramid& pyramid, int32_t level,
    int32_t x, int32_t y, float cosR, float sinR,
    double greediness, double* outCoverage) const
{
    // Use grid points for coarse search (faster)
    const auto& levelModel = levels_[level];
    const auto& pts = levelModel.gridPoints.empty() ? levelModel.points : levelModel.gridPoints;
    const size_t numPoints = pts.size();
    if (numPoints == 0) {
        if (outCoverage) *outCoverage = 0.0;
        return 0.0;
    }

    const float* gxData;
    const float* gyData;
    int32_t width, height, stride;
    if (!pyramid.GetGradientData(level, gxData, gyData, width, height, stride)) {
        if (outCoverage) *outCoverage = 0.0;
        return 0.0;
    }

    const float xf = static_cast<float>(x);
    const float yf = static_cast<float>(y);

    float totalScore = 0.0f;
    float totalWeight = 0.0f;
    int32_t matchedCount = 0;

    const float earlyTermThreshold = static_cast<float>(greediness * numPoints);
    const int32_t checkInterval = 8;

    const float* soaX = levelModel.gridPoints.empty() ? levelModel.soaX.data() : levelModel.gridSoaX.data();
    const float* soaY = levelModel.gridPoints.empty() ? levelModel.soaY.data() : levelModel.gridSoaY.data();
    const float* soaCos = levelModel.gridPoints.empty() ? levelModel.soaCosAngle.data() : levelModel.gridSoaCosAngle.data();
    const float* soaSin = levelModel.gridPoints.empty() ? levelModel.soaSinAngle.data() : levelModel.gridSoaSinAngle.data();
    const float* soaWeight = levelModel.gridPoints.empty() ? levelModel.soaWeight.data() : levelModel.gridSoaWeight.data();

    const int32_t maxX = width - 1;
    const int32_t maxY = height - 1;

    // Main loop with nearest-neighbor interpolation (single memory access per point)
    for (size_t i = 0; i < numPoints; ++i) {
        // Rotate model point
        float rotX = cosR * soaX[i] - sinR * soaY[i];
        float rotY = sinR * soaX[i] + cosR * soaY[i];

        // Nearest neighbor: round to integer
        int32_t imgX = static_cast<int32_t>(xf + rotX + 0.5f);
        int32_t imgY = static_cast<int32_t>(yf + rotY + 0.5f);

        if (imgX >= 0 && imgX <= maxX && imgY >= 0 && imgY <= maxY) {
            int32_t idx = imgY * stride + imgX;

            // Single memory access (vs 4 for bilinear)
            float gx = gxData[idx];
            float gy = gyData[idx];

            float magSq = gx * gx + gy * gy;
            if (magSq >= 25.0f) {
                float rotCos = soaCos[i] * cosR - soaSin[i] * sinR;
                float rotSin = soaSin[i] * cosR + soaCos[i] * sinR;
                float dot = rotCos * gx + rotSin * gy;

                // Fast inverse sqrt approximation
                float invMag = 1.0f / std::sqrt(magSq);
                float score = std::fabs(dot) * invMag;

                totalScore += soaWeight[i] * score;
                totalWeight += soaWeight[i];
                matchedCount++;
            }
        }

        // Early termination check
        if (greediness > 0.0 && ((i + 1) & (checkInterval - 1)) == 0) {
            if (totalWeight > 0) {
                if ((totalScore / totalWeight) * numPoints < earlyTermThreshold * 0.85f) {
                    if (outCoverage) *outCoverage = static_cast<double>(matchedCount) / numPoints;
                    return 0.0;
                }
            }
        }
    }

    double coverage = static_cast<double>(matchedCount) / numPoints;
    if (outCoverage) *outCoverage = coverage;
    if (totalWeight <= 0) return 0.0;

    // Return pure similarity score (no coverage penalty)
    return static_cast<double>(totalScore) / totalWeight;
}

// =============================================================================
// ShapeModelImpl::ComputeScoreQuantized
// =============================================================================

double ShapeModelImpl::ComputeScoreQuantized(
    const AnglePyramid& pyramid, int32_t level,
    double x, double y, float cosR, float sinR, int32_t rotationBin,
    double greediness, double* outCoverage, bool useGridPoints) const
{
    if (cosLUT_.empty() || numAngleBins_ <= 0) {
        float baseMin = (params_.minContrast > 0.0) ? static_cast<float>(params_.minContrast) : 5.0f;
        float levelScale = static_cast<float>(pyramid.GetScale(level));
        float minMag = std::max(2.0f, baseMin * levelScale);
        return ComputeScoreWithSinCos(pyramid, level, x, y, cosR, sinR, 1.0, greediness, outCoverage,
                                      useGridPoints, minMag * minMag);
    }

    const auto& levelModel = levels_[level];
    const auto& pts = useGridPoints ? levelModel.gridPoints : levelModel.points;
    const size_t numPoints = pts.size();
    if (numPoints == 0) {
        if (outCoverage) *outCoverage = 0.0;
        return 0.0;
    }

    const float* gxData;
    const float* gyData;
    int32_t width, height, stride;
    if (!pyramid.GetGradientData(level, gxData, gyData, width, height, stride)) {
        if (outCoverage) *outCoverage = 0.0;
        return 0.0;
    }

    const int16_t* binData;
    int32_t binWidth, binHeight, binStride, numBins;
    if (!pyramid.GetAngleBinData(level, binData, binWidth, binHeight, binStride, numBins)) {
        float baseMin = (params_.minContrast > 0.0) ? static_cast<float>(params_.minContrast) : 5.0f;
        float levelScale = static_cast<float>(pyramid.GetScale(level));
        float minMag = std::max(2.0f, baseMin * levelScale);
        return ComputeScoreWithSinCos(pyramid, level, x, y, cosR, sinR, 1.0, greediness, outCoverage,
                                      useGridPoints, minMag * minMag);
    }

    const float scalef = 1.0f;
    const float xf = static_cast<float>(x);
    const float yf = static_cast<float>(y);

    const float* soaX = useGridPoints ? levelModel.gridSoaX.data() : levelModel.soaX.data();
    const float* soaY = useGridPoints ? levelModel.gridSoaY.data() : levelModel.soaY.data();
    const int16_t* soaBin = useGridPoints ? levelModel.gridSoaAngleBin.data() : levelModel.soaAngleBin.data();
    const float* soaWeight = useGridPoints ? levelModel.gridSoaWeight.data() : levelModel.soaWeight.data();

    const float* cosLUT = cosLUT_.data();
    const int32_t binMask = numAngleBins_ - 1;

    float totalScore = 0.0f;
    float totalWeight = 0.0f;
    int32_t matchedCount = 0;

    const float earlyTermThreshold = static_cast<float>(greediness * numPoints);
    const int32_t checkInterval = 8;
    const float minMagSq = 25.0f;

    const int32_t maxX = width - 2;
    const int32_t maxY = height - 2;

    for (size_t i = 0; i < numPoints; ++i) {
        float rotX = cosR * soaX[i] - sinR * soaY[i];
        float rotY = sinR * soaX[i] + cosR * soaY[i];
        float imgX = xf + scalef * rotX;
        float imgY = yf + scalef * rotY;

        int32_t ix = static_cast<int32_t>(imgX);
        int32_t iy = static_cast<int32_t>(imgY);
        if (imgX < 0) ix--;
        if (imgY < 0) iy--;

        if (ix >= 0 && ix <= maxX && iy >= 0 && iy <= maxY) {
            float fx = imgX - ix;
            float fy = imgY - iy;

            float w00 = (1.0f - fx) * (1.0f - fy);
            float w10 = fx * (1.0f - fy);
            float w01 = (1.0f - fx) * fy;
            float w11 = fx * fy;

            int32_t idx = iy * stride + ix;
            float gx = w00 * gxData[idx] + w10 * gxData[idx + 1] +
                       w01 * gxData[idx + stride] + w11 * gxData[idx + stride + 1];
            float gy = w00 * gyData[idx] + w10 * gyData[idx + 1] +
                       w01 * gyData[idx + stride] + w11 * gyData[idx + stride + 1];

            float magSq = gx * gx + gy * gy;
            if (magSq >= minMagSq) {
                int32_t nearestX = static_cast<int32_t>(imgX + 0.5f);
                int32_t nearestY = static_cast<int32_t>(imgY + 0.5f);
                nearestX = std::max(0, std::min(binWidth - 1, nearestX));
                nearestY = std::max(0, std::min(binHeight - 1, nearestY));

                int16_t imageBin = binData[nearestY * binStride + nearestX];
                int16_t modelBin = soaBin[i];

                int32_t binDiff = (modelBin + rotationBin - imageBin) & binMask;
                float score = cosLUT[binDiff];
                float simPos = std::max(0.0f, score);
                float simNeg = std::max(0.0f, -score);

                float finalScore = 0.0f;
                if (params_.polarity == MatchPolarity::Opposite) {
                    finalScore = simNeg;
                } else if (params_.polarity == MatchPolarity::Any) {
                    finalScore = std::max(simPos, simNeg);
                } else {
                    finalScore = simPos;
                }

                if (finalScore > 0.0f) {
                    totalScore += soaWeight[i] * finalScore;
                    totalWeight += soaWeight[i];
                    matchedCount++;
                }
            }
        }

        if (greediness > 0.0 && ((i + 1) & (checkInterval - 1)) == 0) {
            if (totalWeight > 0) {
                if ((totalScore / totalWeight) * numPoints < earlyTermThreshold * 0.85f) {
                    if (outCoverage) *outCoverage = static_cast<double>(matchedCount) / numPoints;
                    return 0.0;
                }
            }
        }
    }

    double coverage = static_cast<double>(matchedCount) / numPoints;
    if (outCoverage) *outCoverage = coverage;
    if (totalWeight <= 0) return 0.0;

    // Return pure similarity score (no coverage penalty)
    return static_cast<double>(totalScore) / totalWeight;
}

// =============================================================================
// ShapeModelImpl::RefinePosition
// =============================================================================

void ShapeModelImpl::RefinePosition(
    const AnglePyramid& pyramid, MatchResult& match,
    SubpixelMethod method) const
{
    if (method == SubpixelMethod::None) {
        return;
    }

    constexpr int32_t level = 0;  // Always refine at finest level

    if (method == SubpixelMethod::Parabolic) {
        // =====================================================================
        // HALCON 'interpolation' style: Parabolic fitting for position + angle
        // Complexity: 13 Score evaluations, ~0.05px accuracy
        // =====================================================================

        constexpr double delta = 0.5;  // Position sampling step

        // === Position refinement: 3x3 sampling grid ===
        double scores[3][3];
        for (int32_t dy = -1; dy <= 1; ++dy) {
            for (int32_t dx = -1; dx <= 1; ++dx) {
                scores[dy + 1][dx + 1] = ComputeScoreAtPosition(
                    pyramid, level, match.x + dx * delta, match.y + dy * delta,
                    match.angle, 1.0, 0.0);
            }
        }

        // X-direction subpixel (use middle row)
        double denom_x = 2.0 * (scores[1][0] - 2.0 * scores[1][1] + scores[1][2]);
        if (std::abs(denom_x) > 1e-10) {
            double dx = delta * (scores[1][0] - scores[1][2]) / denom_x;
            dx = std::max(-delta, std::min(delta, dx));  // Clamp to prevent divergence
            match.x += dx;
        }

        // Y-direction subpixel (use middle column)
        double denom_y = 2.0 * (scores[0][1] - 2.0 * scores[1][1] + scores[2][1]);
        if (std::abs(denom_y) > 1e-10) {
            double dy = delta * (scores[0][1] - scores[2][1]) / denom_y;
            dy = std::max(-delta, std::min(delta, dy));  // Clamp to prevent divergence
            match.y += dy;
        }

        // === Angle refinement: 3-point sampling ===
        // Angle step based on model size (smaller model = larger angle effect)
        double modelRadius = std::max(modelMaxX_ - modelMinX_, modelMaxY_ - modelMinY_) / 2.0;
        double delta_angle = std::atan(0.25 / std::max(modelRadius, 10.0));  // ~0.5-2 degrees
        delta_angle = std::max(0.005, std::min(0.05, delta_angle));  // Clamp to 0.3-3 degrees

        double s_minus = ComputeScoreAtPosition(pyramid, level, match.x, match.y,
                                                 match.angle - delta_angle, 1.0, 0.0);
        double s_center = ComputeScoreAtPosition(pyramid, level, match.x, match.y,
                                                  match.angle, 1.0, 0.0);
        double s_plus = ComputeScoreAtPosition(pyramid, level, match.x, match.y,
                                                match.angle + delta_angle, 1.0, 0.0);

        double denom_a = 2.0 * (s_minus - 2.0 * s_center + s_plus);
        if (std::abs(denom_a) > 1e-10) {
            double da = delta_angle * (s_minus - s_plus) / denom_a;
            da = std::max(-delta_angle, std::min(delta_angle, da));  // Clamp
            match.angle += da;
        }

        // Final score at refined position
        match.score = ComputeScoreAtPosition(pyramid, level, match.x, match.y,
                                              match.angle, 1.0, 0.0);
    }
    else if (method == SubpixelMethod::LeastSquares ||
             method == SubpixelMethod::LeastSquaresHigh ||
             method == SubpixelMethod::LeastSquaresVeryHigh) {
        // =====================================================================
        // HALCON 'least_squares' style: Levenberg-Marquardt optimization
        // Jointly optimizes (x, y, angle) using numerical gradient + Hessian
        // =====================================================================

        // Parameters based on accuracy level
        int32_t maxIter;
        double tolerance;
        if (method == SubpixelMethod::LeastSquares) {
            maxIter = 10; tolerance = 1e-4;
        } else if (method == SubpixelMethod::LeastSquaresHigh) {
            maxIter = 20; tolerance = 1e-5;
        } else {  // LeastSquaresVeryHigh
            maxIter = 50; tolerance = 1e-6;
        }

        // Numerical differentiation step sizes
        constexpr double eps_pos = 0.1;      // Position step (pixels)
        constexpr double eps_angle = 0.001;  // Angle step (~0.06 degrees)

        // Current state
        double x = match.x;
        double y = match.y;
        double theta = match.angle;
        double score = match.score;

        // Levenberg-Marquardt damping factor
        double lambda = 0.001;

        // Track best result (in case of oscillation)
        double bestX = x, bestY = y, bestTheta = theta, bestScore = score;

        for (int32_t iter = 0; iter < maxIter; ++iter) {
            // Compute current score
            score = ComputeScoreAtPosition(pyramid, level, x, y, theta, 1.0, 0.0);

            if (score > bestScore) {
                bestX = x; bestY = y; bestTheta = theta; bestScore = score;
            }

            // Compute numerical gradient (6 Score evaluations)
            double s_xp = ComputeScoreAtPosition(pyramid, level, x + eps_pos, y, theta, 1.0, 0.0);
            double s_xm = ComputeScoreAtPosition(pyramid, level, x - eps_pos, y, theta, 1.0, 0.0);
            double s_yp = ComputeScoreAtPosition(pyramid, level, x, y + eps_pos, theta, 1.0, 0.0);
            double s_ym = ComputeScoreAtPosition(pyramid, level, x, y - eps_pos, theta, 1.0, 0.0);
            double s_tp = ComputeScoreAtPosition(pyramid, level, x, y, theta + eps_angle, 1.0, 0.0);
            double s_tm = ComputeScoreAtPosition(pyramid, level, x, y, theta - eps_angle, 1.0, 0.0);

            double grad_x = (s_xp - s_xm) / (2.0 * eps_pos);
            double grad_y = (s_yp - s_ym) / (2.0 * eps_pos);
            double grad_theta = (s_tp - s_tm) / (2.0 * eps_angle);

            // Compute diagonal Hessian elements (reuse samples from gradient)
            double H_xx = (s_xp - 2.0 * score + s_xm) / (eps_pos * eps_pos);
            double H_yy = (s_yp - 2.0 * score + s_ym) / (eps_pos * eps_pos);
            double H_tt = (s_tp - 2.0 * score + s_tm) / (eps_angle * eps_angle);

            // Check convergence (gradient norm)
            // Scale angle gradient by 1000 to make it comparable to position
            double grad_norm = std::sqrt(grad_x * grad_x + grad_y * grad_y +
                                          (grad_theta * grad_theta) * 1e6);
            if (grad_norm < tolerance) {
                break;  // Converged
            }

            // Levenberg-Marquardt update: delta = g / (|H| + lambda)
            // For maximization, we follow the positive gradient direction
            double delta_x = grad_x / (std::abs(H_xx) + lambda + 1e-10);
            double delta_y = grad_y / (std::abs(H_yy) + lambda + 1e-10);
            double delta_theta = grad_theta / (std::abs(H_tt) + lambda + 1e-10);

            // Clamp step size to prevent divergence
            delta_x = std::max(-1.0, std::min(1.0, delta_x));
            delta_y = std::max(-1.0, std::min(1.0, delta_y));
            delta_theta = std::max(-0.05, std::min(0.05, delta_theta));  // ~3 degrees max

            // Try update
            double x_new = x + delta_x;
            double y_new = y + delta_y;
            double theta_new = theta + delta_theta;

            double score_new = ComputeScoreAtPosition(pyramid, level, x_new, y_new,
                                                       theta_new, 1.0, 0.0);

            if (score_new > score) {
                // Accept update, decrease damping
                x = x_new;
                y = y_new;
                theta = theta_new;
                lambda *= 0.1;
                lambda = std::max(lambda, 1e-7);
            } else {
                // Reject update, increase damping
                lambda *= 10.0;
                lambda = std::min(lambda, 1e7);

                // If damping too high, we've converged
                if (lambda > 1e6) {
                    break;
                }
            }
        }

        // Use best result found
        match.x = bestX;
        match.y = bestY;
        match.angle = bestTheta;
        match.score = bestScore;
    }

    match.refined = true;
}

} // namespace Internal
} // namespace Qi::Vision::Matching
