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
        // Lightweight mode - fall back to ComputeScoreQuantized
        const float cosR = static_cast<float>(std::cos(angle));
        const float sinR = static_cast<float>(std::sin(angle));
        double normAngle = angle;
        while (normAngle < 0) normAngle += 2.0 * PI;
        while (normAngle >= 2.0 * PI) normAngle -= 2.0 * PI;
        int32_t rotationBin = static_cast<int32_t>(normAngle * numAngleBins_ / (2.0 * PI)) % numAngleBins_;
        return ComputeScoreQuantized(pyramid, level, x, y, cosR, sinR, rotationBin,
                                      greediness, outCoverage, useGridPoints);
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
            if (magSq >= 25.0f) {
                float rotCos = soaCos[i] * cosR - soaSin[i] * sinR;
                float rotSin = soaSin[i] * cosR + soaCos[i] * sinR;
                float dot = rotCos * gx + rotSin * gy;
                float mag = std::sqrt(magSq);

                if (params_.metric == MetricMode::UsePolarity) {
                    float similarity = std::max(0.0f, dot / mag);
                    totalScore += soaWeight[i] * similarity;
                    totalWeight += soaWeight[i];
                    matchedCount++;
                } else {
                    float similarity = dot / mag;
                    totalScore += soaWeight[i] * std::max(0.0f, similarity);
                    totalWeight += soaWeight[i];
                    matchedCount++;

                    totalScoreInverted += soaWeight[i] * std::max(0.0f, -similarity);
                    totalWeightInverted += soaWeight[i];
                    matchedCountInverted++;
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

    if (outCoverage) *outCoverage = static_cast<double>(matchedCount) / numPoints;
    if (totalWeight <= 0) return 0.0;

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
        // Lightweight mode - fall back to ComputeScoreQuantized
        const float cosR = static_cast<float>(std::cos(angle));
        const float sinR = static_cast<float>(std::sin(angle));
        double normAngle = angle;
        while (normAngle < 0) normAngle += 2.0 * PI;
        while (normAngle >= 2.0 * PI) normAngle -= 2.0 * PI;
        int32_t rotationBin = static_cast<int32_t>(normAngle * numAngleBins_ / (2.0 * PI)) % numAngleBins_;
        return ComputeScoreQuantized(pyramid, level, x, y, cosR, sinR, rotationBin,
                                      greediness, outCoverage, useGridPoints);
    }

    const float cosR = static_cast<float>(std::cos(angle));
    const float sinR = static_cast<float>(std::sin(angle));
    const float scalef = static_cast<float>(scale);
    const float xf = static_cast<float>(x);
    const float yf = static_cast<float>(y);

    const __m128 vMinMagSq = _mm_set_ss(25.0f);
    const __m128 vSignMask = _mm_set_ss(-0.0f);

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
                __m128 vDot = _mm_set_ss(dot);
                __m128 vAbsDot = _mm_andnot_ps(vSignMask, vDot);
                __m128 vScore = _mm_mul_ss(vAbsDot, vInvMag);

                float score;
                _mm_store_ss(&score, vScore);

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

    if (outCoverage) *outCoverage = static_cast<double>(matchedCount) / numPoints;
    if (totalWeight <= 0) return 0.0;

    return static_cast<double>(totalScore) / totalWeight;
}

// =============================================================================
// ShapeModelImpl::ComputeScoreWithSinCos
// =============================================================================

double ShapeModelImpl::ComputeScoreWithSinCos(
    const AnglePyramid& pyramid, int32_t level,
    double x, double y, float cosR, float sinR, double scale,
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
        // Lightweight mode - fall back to ComputeScoreQuantized
        // Need to compute rotation bin from cosR/sinR
        double angle = std::atan2(sinR, cosR);
        if (angle < 0) angle += 2.0 * PI;
        int32_t rotationBin = static_cast<int32_t>(angle * numAngleBins_ / (2.0 * PI)) % numAngleBins_;
        return ComputeScoreQuantized(pyramid, level, x, y, cosR, sinR, rotationBin,
                                      greediness, outCoverage, useGridPoints);
    }

    const float scalef = static_cast<float>(scale);
    const float xf = static_cast<float>(x);
    const float yf = static_cast<float>(y);

    const __m128 vMinMagSq = _mm_set_ss(25.0f);
    const __m128 vSignMask = _mm_set_ss(-0.0f);

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
                __m128 vDot = _mm_set_ss(dot);
                __m128 vAbsDot = _mm_andnot_ps(vSignMask, vDot);
                __m128 vScore = _mm_mul_ss(vAbsDot, vInvMag);

                float score;
                _mm_store_ss(&score, vScore);

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

    if (outCoverage) *outCoverage = static_cast<double>(matchedCount) / numPoints;
    if (totalWeight <= 0) return 0.0;

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

    if (outCoverage) *outCoverage = static_cast<double>(matchedCount) / numPoints;
    if (totalWeight <= 0) return 0.0;

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
        return ComputeScoreWithSinCos(pyramid, level, x, y, cosR, sinR, 1.0, greediness, outCoverage, useGridPoints);
    }

    const auto& levelModel = levels_[level];
    const auto& pts = useGridPoints ? levelModel.gridPoints : levelModel.points;
    const size_t numPoints = pts.size();
    if (numPoints == 0) {
        if (outCoverage) *outCoverage = 0.0;
        return 0.0;
    }

    // Get bin data (required for quantized scoring)
    const int16_t* binData;
    int32_t binWidth, binHeight, binStride, numBins;
    if (!pyramid.GetAngleBinData(level, binData, binWidth, binHeight, binStride, numBins)) {
        return ComputeScoreWithSinCos(pyramid, level, x, y, cosR, sinR, 1.0, greediness, outCoverage, useGridPoints);
    }

    // Try to get gradient data (for bilinear magnitude interpolation)
    const float* gxData = nullptr;
    const float* gyData = nullptr;
    int32_t gxWidth, gxHeight, gxStride;
    bool hasGradient = pyramid.GetGradientData(level, gxData, gyData, gxWidth, gxHeight, gxStride);

    // If no gradient, try to get magnitude data directly (lightweight mode)
    const float* magData = nullptr;
    int32_t magWidth, magHeight, magStride;
    bool magIsSquared = false;
    bool hasMagnitude = false;
    if (!hasGradient) {
        hasMagnitude = pyramid.GetMagnitudeData(level, magData, magWidth, magHeight, magStride, magIsSquared);
        if (!hasMagnitude) {
            // No gradient and no magnitude data - cannot compute score
            if (outCoverage) *outCoverage = 0.0;
            return 0.0;
        }
    }

    const int32_t width = hasGradient ? gxWidth : magWidth;
    const int32_t height = hasGradient ? gxHeight : magHeight;
    const int32_t stride = hasGradient ? gxStride : magStride;

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
            float magSq;

            if (hasGradient) {
                // Full mode: bilinear interpolate gx/gy, compute magSq
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
                magSq = gx * gx + gy * gy;
            } else {
                // Lightweight mode: nearest-neighbor magSq lookup
                int32_t nearestX = static_cast<int32_t>(imgX + 0.5f);
                int32_t nearestY = static_cast<int32_t>(imgY + 0.5f);
                nearestX = std::max(0, std::min(width - 1, nearestX));
                nearestY = std::max(0, std::min(height - 1, nearestY));
                float m = magData[nearestY * stride + nearestX];
                magSq = magIsSquared ? m : (m * m);
            }

            if (magSq >= minMagSq) {
                int32_t nearestX = static_cast<int32_t>(imgX + 0.5f);
                int32_t nearestY = static_cast<int32_t>(imgY + 0.5f);
                nearestX = std::max(0, std::min(binWidth - 1, nearestX));
                nearestY = std::max(0, std::min(binHeight - 1, nearestY));

                int16_t imageBin = binData[nearestY * binStride + nearestX];
                int16_t modelBin = soaBin[i];

                int32_t binDiff = (modelBin + rotationBin - imageBin) & binMask;
                float score = cosLUT[binDiff];

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

    if (outCoverage) *outCoverage = static_cast<double>(matchedCount) / numPoints;
    if (totalWeight <= 0) return 0.0;

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

    int32_t level = 0;

    if (method == SubpixelMethod::Parabolic) {
        double scores[3][3];

        for (int32_t dy = -1; dy <= 1; ++dy) {
            for (int32_t dx = -1; dx <= 1; ++dx) {
                scores[dy + 1][dx + 1] = ComputeScoreAtPosition(
                    pyramid, level, match.x + dx * 0.5, match.y + dy * 0.5,
                    match.angle, 1.0, 0.0);
            }
        }

        double denom = 2.0 * (scores[1][0] - 2.0 * scores[1][1] + scores[1][2]);
        if (std::abs(denom) > 1e-10) {
            double dx = (scores[1][0] - scores[1][2]) / denom;
            match.x += dx * 0.5;
        }

        denom = 2.0 * (scores[0][1] - 2.0 * scores[1][1] + scores[2][1]);
        if (std::abs(denom) > 1e-10) {
            double dy = (scores[0][1] - scores[2][1]) / denom;
            match.y += dy * 0.5;
        }

        match.score = ComputeScoreAtPosition(pyramid, level, match.x, match.y,
                                              match.angle, 1.0, 0.0);
    }
    else if (method == SubpixelMethod::LeastSquares) {
        const int32_t maxIter = 10;
        const double stepSize = 0.1;
        const double tolerance = 0.001;

        double bestScore = match.score;
        double bestX = match.x;
        double bestY = match.y;
        double bestAngle = match.angle;

        for (int32_t iter = 0; iter < maxIter; ++iter) {
            double eps = 0.1;
            double scoreX1 = ComputeScoreAtPosition(pyramid, level, match.x + eps, match.y,
                                                     match.angle, 1.0, 0.0);
            double scoreX0 = ComputeScoreAtPosition(pyramid, level, match.x - eps, match.y,
                                                     match.angle, 1.0, 0.0);
            double scoreY1 = ComputeScoreAtPosition(pyramid, level, match.x, match.y + eps,
                                                     match.angle, 1.0, 0.0);
            double scoreY0 = ComputeScoreAtPosition(pyramid, level, match.x, match.y - eps,
                                                     match.angle, 1.0, 0.0);

            double gradX = (scoreX1 - scoreX0) / (2.0 * eps);
            double gradY = (scoreY1 - scoreY0) / (2.0 * eps);

            match.x += stepSize * gradX;
            match.y += stepSize * gradY;

            double newScore = ComputeScoreAtPosition(pyramid, level, match.x, match.y,
                                                      match.angle, 1.0, 0.0);

            if (newScore > bestScore) {
                bestScore = newScore;
                bestX = match.x;
                bestY = match.y;
                bestAngle = match.angle;
            }

            if (std::abs(gradX) < tolerance && std::abs(gradY) < tolerance) {
                break;
            }
        }

        match.x = bestX;
        match.y = bestY;
        match.angle = bestAngle;
        match.score = bestScore;
    }

    match.refined = true;
}

} // namespace Internal
} // namespace Qi::Vision::Matching
