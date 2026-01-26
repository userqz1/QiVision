/**
 * @file NCCModelScore.cpp
 * @brief NCCModel score computation implementation
 *
 * Contains:
 * - NCC score computation using integral images
 * - Subpixel refinement using parabolic interpolation
 * - SIMD-optimized variants (future)
 *
 * NCC Formula:
 * NCC = sum((T - mean_T) * (I - mean_I)) / (n * stddev_T * stddev_I)
 *
 * Using integral images:
 * - mean_I = sum(I) / n  (O(1) using integral image)
 * - stddev_I = sqrt(sum(I^2)/n - mean_I^2)  (O(1) using squared integral)
 * - Cross-correlation is computed directly
 */

#include "NCCModelImpl.h"

#include <QiVision/Internal/IntegralImage.h>
#include <QiVision/Internal/Interpolate.h>
#include <QiVision/Core/Constants.h>

#include <cmath>
#include <algorithm>

namespace Qi::Vision::Matching {
namespace Internal {

// =============================================================================
// NCCModelImpl - Score Computation
// =============================================================================

double NCCModelImpl::ComputeNCCScore(
    const Qi::Vision::Internal::IntegralImage& integralImage,
    const float* imageData,
    int32_t imageWidth,
    int32_t imageHeight,
    int32_t x,
    int32_t y,
    int32_t angleIndex,
    int32_t level) const
{
    if (level < 0 || level >= static_cast<int32_t>(levels_.size())) {
        return -1.0;
    }

    if (angleIndex < 0 || angleIndex >= static_cast<int32_t>(rotatedTemplates_[level].size())) {
        return -1.0;
    }

    const auto& rotatedTemplate = rotatedTemplates_[level][angleIndex];

    if (!rotatedTemplate.IsValid()) {
        return -1.0;
    }

    // Get template dimensions
    int32_t tWidth = rotatedTemplate.width;
    int32_t tHeight = rotatedTemplate.height;

    // Check bounds
    if (x < 0 || y < 0 ||
        x + tWidth > imageWidth || y + tHeight > imageHeight) {
        return -1.0;
    }

    // For rotated templates, we need to compute image statistics only for valid pixels
    // (integral image gives wrong result for rotated bounding box with empty corners)

    const float* templateData = rotatedTemplate.data.data();
    int32_t n = rotatedTemplate.numPixels;

    if (n <= 1) {
        return -1.0;
    }

    // Template statistics (precomputed)
    double templateStddev = rotatedTemplate.stddev;
    if (templateStddev < 1e-6) {
        return 0.0;
    }

    // Compute image statistics and cross-correlation in single pass
    // Use mask to identify valid pixels (rotated templates always have mask)
    double imageSum = 0.0;
    double imageSumSq = 0.0;
    double crossCorr = 0.0;
    int32_t validCount = 0;

    // Rotated templates always have mask now
    if (!rotatedTemplate.mask.empty()) {
        const uint8_t* mask = rotatedTemplate.mask.data();

        for (int32_t ty = 0; ty < tHeight; ++ty) {
            for (int32_t tx = 0; tx < tWidth; ++tx) {
                if (mask[ty * tWidth + tx] > 0) {
                    int32_t imgX = x + tx;
                    int32_t imgY = y + ty;
                    float iVal = imageData[imgY * imageWidth + imgX];
                    imageSum += iVal;
                    imageSumSq += iVal * iVal;
                    validCount++;
                }
            }
        }
    } else {
        // Fallback for level 0 unrotated template (no mask)
        for (int32_t ty = 0; ty < tHeight; ++ty) {
            for (int32_t tx = 0; tx < tWidth; ++tx) {
                int32_t imgX = x + tx;
                int32_t imgY = y + ty;
                float iVal = imageData[imgY * imageWidth + imgX];
                imageSum += iVal;
                imageSumSq += iVal * iVal;
                validCount++;
            }
        }
    }

    if (validCount <= 1) {
        return -1.0;
    }

    double imageMean = imageSum / validCount;
    double imageVar = imageSumSq / validCount - imageMean * imageMean;

    // Handle low variance (flat region)
    if (imageVar < 1e-6) {
        return 0.0;
    }

    double imageStddev = std::sqrt(imageVar);

    // Second pass: compute cross-correlation with correct mean
    crossCorr = 0.0;
    if (!rotatedTemplate.mask.empty()) {
        const uint8_t* mask = rotatedTemplate.mask.data();

        for (int32_t ty = 0; ty < tHeight; ++ty) {
            for (int32_t tx = 0; tx < tWidth; ++tx) {
                if (mask[ty * tWidth + tx] > 0) {
                    int32_t imgX = x + tx;
                    int32_t imgY = y + ty;

                    float tVal = templateData[ty * tWidth + tx];  // Already zero-mean
                    float iVal = imageData[imgY * imageWidth + imgX] - static_cast<float>(imageMean);

                    crossCorr += tVal * iVal;
                }
            }
        }
    } else {
        for (int32_t ty = 0; ty < tHeight; ++ty) {
            for (int32_t tx = 0; tx < tWidth; ++tx) {
                int32_t imgX = x + tx;
                int32_t imgY = y + ty;

                float tVal = templateData[ty * tWidth + tx];  // Already zero-mean
                float iVal = imageData[imgY * imageWidth + imgX] - static_cast<float>(imageMean);

                crossCorr += tVal * iVal;
            }
        }
    }

    // Use validCount for normalization
    n = validCount;

    // Compute NCC
    double denominator = n * templateStddev * imageStddev;
    if (denominator < 1e-10) {
        return 0.0;
    }

    double ncc = crossCorr / denominator;

    // Handle ignore_global_polarity mode
    if (metric_ == MetricMode::IgnoreGlobalPolarity) {
        ncc = std::abs(ncc);
    }

    // Clamp to [-1, 1]
    return std::clamp(ncc, -1.0, 1.0);
}

double NCCModelImpl::ComputeNCCScoreSubpixel(
    const float* imageData,
    int32_t imageWidth,
    int32_t imageHeight,
    double x,
    double y,
    double angle,
    int32_t level) const
{
    // TODO: Implement bilinear interpolation for subpixel positions
    // For now, just round to nearest integer
    int32_t ix = static_cast<int32_t>(std::round(x));
    int32_t iy = static_cast<int32_t>(std::round(y));
    int32_t angleIdx = GetAngleIndex(angle);

    // This requires integral image which we don't have here
    // Return -1 to indicate not implemented
    (void)imageData;
    (void)imageWidth;
    (void)imageHeight;
    (void)ix;
    (void)iy;
    (void)angleIdx;

    return -1.0;
}

void NCCModelImpl::RefinePosition(
    const Qi::Vision::Internal::IntegralImage& integralImage,
    const float* imageData,
    int32_t imageWidth,
    int32_t imageHeight,
    MatchResult& match,
    int32_t level) const
{
    // Parabolic refinement for position
    // Sample 3x3 grid around current position

    const auto& modelLevel = levels_[level];
    int32_t angleIdx = GetAngleIndex(match.angle);
    const auto& rotatedTemplate = rotatedTemplates_[level][angleIdx];
    if (!rotatedTemplate.IsValid()) {
        return;
    }

    auto computeOriginOffset = [&](int32_t aIdx, const RotatedTemplate& tpl,
                                   double& outX, double& outY) {
        double angle = searchAngles_[aIdx];
        double cosA = std::cos(angle);
        double sinA = std::sin(angle);

        double levelCenterX = modelLevel.width * 0.5;
        double levelCenterY = modelLevel.height * 0.5;
        double originLevelX = origin_.x * modelLevel.scale;
        double originLevelY = origin_.y * modelLevel.scale;

        double dx = originLevelX - levelCenterX;
        double dy = originLevelY - levelCenterY;

        double rotDx = cosA * dx - sinA * dy;
        double rotDy = sinA * dx + cosA * dy;

        outX = tpl.offsetX + rotDx;
        outY = tpl.offsetY + rotDy;
    };

    double originOffsetX = 0.0;
    double originOffsetY = 0.0;
    computeOriginOffset(angleIdx, rotatedTemplate, originOffsetX, originOffsetY);

    int32_t cx = static_cast<int32_t>(std::round(match.x - originOffsetX));
    int32_t cy = static_cast<int32_t>(std::round(match.y - originOffsetY));

    // Sample scores in 3x3 neighborhood
    double scores[3][3];
    bool valid[3][3] = {{false}};

    for (int32_t dy = -1; dy <= 1; ++dy) {
        for (int32_t dx = -1; dx <= 1; ++dx) {
            int32_t x = cx + dx;
            int32_t y = cy + dy;

            if (x >= 0 && y >= 0 &&
                x + rotatedTemplate.width <= imageWidth &&
                y + rotatedTemplate.height <= imageHeight) {

                scores[dy + 1][dx + 1] = ComputeNCCScore(
                    integralImage, imageData, imageWidth, imageHeight,
                    x, y, angleIdx, level);
                valid[dy + 1][dx + 1] = true;
            }
        }
    }

    // Check if all samples are valid
    if (!valid[0][1] || !valid[2][1] || !valid[1][0] || !valid[1][2]) {
        return;
    }

    // Parabolic refinement in X
    double subX = 0.0;
    {
        double v0 = scores[1][0];  // left
        double v1 = scores[1][1];  // center
        double v2 = scores[1][2];  // right

        double denom = 2.0 * (v0 - 2.0 * v1 + v2);
        if (std::abs(denom) > 1e-10) {
            subX = (v0 - v2) / denom;
            subX = std::clamp(subX, -0.5, 0.5);
        }
    }

    // Parabolic refinement in Y
    double subY = 0.0;
    {
        double v0 = scores[0][1];  // top
        double v1 = scores[1][1];  // center
        double v2 = scores[2][1];  // bottom

        double denom = 2.0 * (v0 - 2.0 * v1 + v2);
        if (std::abs(denom) > 1e-10) {
            subY = (v0 - v2) / denom;
            subY = std::clamp(subY, -0.5, 0.5);
        }
    }

    // Update position
    match.x += subX;
    match.y += subY;
    match.refined = true;

    // Parabolic refinement in angle
    if (angleIdx > 0 && angleIdx < static_cast<int32_t>(searchAngles_.size()) - 1) {
        const auto& leftTemplate = rotatedTemplates_[level][angleIdx - 1];
        const auto& rightTemplate = rotatedTemplates_[level][angleIdx + 1];

        double sLeft = -1.0;
        double sRight = -1.0;

        if (leftTemplate.IsValid()) {
            double leftOriginOffsetX = 0.0;
            double leftOriginOffsetY = 0.0;
            computeOriginOffset(angleIdx - 1, leftTemplate, leftOriginOffsetX, leftOriginOffsetY);
            int32_t leftX = static_cast<int32_t>(std::round(match.x - leftOriginOffsetX));
            int32_t leftY = static_cast<int32_t>(std::round(match.y - leftOriginOffsetY));
            if (leftX >= 0 && leftY >= 0 &&
                leftX + leftTemplate.width <= imageWidth &&
                leftY + leftTemplate.height <= imageHeight) {
                sLeft = ComputeNCCScore(integralImage, imageData, imageWidth, imageHeight,
                                         leftX, leftY, angleIdx - 1, level);
            }
        }

        double sCenter = match.score;
        if (rightTemplate.IsValid()) {
            double rightOriginOffsetX = 0.0;
            double rightOriginOffsetY = 0.0;
            computeOriginOffset(angleIdx + 1, rightTemplate, rightOriginOffsetX, rightOriginOffsetY);
            int32_t rightX = static_cast<int32_t>(std::round(match.x - rightOriginOffsetX));
            int32_t rightY = static_cast<int32_t>(std::round(match.y - rightOriginOffsetY));
            if (rightX >= 0 && rightY >= 0 &&
                rightX + rightTemplate.width <= imageWidth &&
                rightY + rightTemplate.height <= imageHeight) {
                sRight = ComputeNCCScore(integralImage, imageData, imageWidth, imageHeight,
                                          rightX, rightY, angleIdx + 1, level);
            }
        }

        if (sLeft >= 0.0 && sRight >= 0.0) {
            double denom = 2.0 * (sLeft - 2.0 * sCenter + sRight);
            if (std::abs(denom) > 1e-10) {
                double subAngle = (sLeft - sRight) / denom;
                subAngle = std::clamp(subAngle, -0.5, 0.5);

                double angleStep = (searchAngles_.size() > 1) ?
                    (searchAngles_[1] - searchAngles_[0]) : 0.0;
                match.angle += subAngle * angleStep;
            }
        }
    }
}

} // namespace Internal
} // namespace Qi::Vision::Matching
