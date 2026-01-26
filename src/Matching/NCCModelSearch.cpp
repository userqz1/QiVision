/**
 * @file NCCModelSearch.cpp
 * @brief NCCModel search implementation
 *
 * Contains:
 * - Multi-level pyramid search
 * - Coarse-to-fine refinement
 * - Non-maximum suppression
 */

#include "NCCModelImpl.h"

#include <QiVision/Internal/Pyramid.h>
#include <QiVision/Core/Constants.h>

#include <algorithm>
#include <cmath>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace Qi::Vision::Matching {
namespace Internal {

// =============================================================================
// NCCModelImpl - Search Functions
// =============================================================================

std::vector<MatchResult> NCCModelImpl::Find(const QImage& image,
                                             const SearchParams& params) const
{
    std::vector<MatchResult> results;

    if (!valid_ || !image.IsValid()) {
        return results;
    }

    // Build image pyramid
    int32_t numLevels = params.numLevels;
    if (numLevels <= 0) {
        numLevels = static_cast<int32_t>(levels_.size());
    }
    numLevels = std::min(numLevels, static_cast<int32_t>(levels_.size()));

    Qi::Vision::Internal::PyramidParams pyramidParams;
    pyramidParams.numLevels = numLevels;
    pyramidParams.sigma = 1.0;

    auto imagePyramid = Qi::Vision::Internal::BuildGaussianPyramid(image, pyramidParams);

    if (imagePyramid.Empty()) {
        return results;
    }

    // Build integral image pyramid
    IntegralImagePyramid integralPyramid;
    if (!integralPyramid.Build(imagePyramid)) {
        return results;
    }

    // Start from coarsest level
    int32_t startLevel = numLevels - 1;

    // Coarse search at top level
    auto candidates = CoarseSearch(integralPyramid, imagePyramid, startLevel, params);

    // Refine through pyramid levels
    for (int32_t level = startLevel - 1; level >= 0; --level) {
        candidates = SearchLevel(integralPyramid, imagePyramid, level, candidates, params);

        if (candidates.empty()) {
            break;
        }
    }

    // Apply NMS
    if (!candidates.empty() && params.maxOverlap < 1.0) {
        double modelWidth = static_cast<double>(templateSize_.width);
        double modelHeight = static_cast<double>(templateSize_.height);
        candidates = NonMaxSuppressionOverlap(candidates, params.maxOverlap,
                                               modelWidth, modelHeight);
    }

    // Limit number of matches
    if (params.maxMatches > 0 &&
        static_cast<int32_t>(candidates.size()) > params.maxMatches) {
        candidates.resize(params.maxMatches);
    }

    return candidates;
}

std::vector<MatchResult> NCCModelImpl::CoarseSearch(
    const IntegralImagePyramid& integralPyramid,
    const Qi::Vision::Internal::ImagePyramid& imagePyramid,
    int32_t level,
    const SearchParams& params) const
{
    std::vector<MatchResult> candidates;

    if (level < 0 || level >= static_cast<int32_t>(levels_.size())) {
        return candidates;
    }

    const auto& modelLevel = levels_[level];
    const auto& imageLevel = imagePyramid.GetLevel(level);
    const auto& integralImage = integralPyramid.GetLevel(level);

    if (!modelLevel.IsValid() || !imageLevel.IsValid()) {
        return candidates;
    }

    int32_t imgWidth = imageLevel.width;
    int32_t imgHeight = imageLevel.height;

    // Search region
    int32_t searchStartX = 0;
    int32_t searchStartY = 0;
    int32_t searchEndX = imgWidth - modelLevel.width;
    int32_t searchEndY = imgHeight - modelLevel.height;

    if (searchEndX < 0 || searchEndY < 0) {
        return candidates;
    }

    // Search step at coarse level (skip pixels for speed)
    int32_t step = std::max(1, 2);

    // Get angle range to search
    double angleStart = params.angleStart;
    double angleExtent = params.angleExtent;

    // Find matching angles in our precomputed set
    std::vector<int32_t> angleIndices;
    for (size_t i = 0; i < searchAngles_.size(); ++i) {
        double angle = searchAngles_[i];
        if (angleExtent >= 0) {
            if (angle >= angleStart && angle <= angleStart + angleExtent) {
                angleIndices.push_back(static_cast<int32_t>(i));
            }
        } else {
            if (angle <= angleStart && angle >= angleStart + angleExtent) {
                angleIndices.push_back(static_cast<int32_t>(i));
            }
        }
    }

    if (angleIndices.empty()) {
        // Use all angles if none match
        for (size_t i = 0; i < searchAngles_.size(); ++i) {
            angleIndices.push_back(static_cast<int32_t>(i));
        }
    }

    auto computeOriginOffset = [&](int32_t angleIdx, const RotatedTemplate& rotatedTemplate,
                                   double& outX, double& outY) {
        double angle = searchAngles_[angleIdx];
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

        outX = rotatedTemplate.offsetX + rotDx;
        outY = rotatedTemplate.offsetY + rotDy;
    };

    // Use smallest rotated template size to avoid missing valid positions
    int32_t minRotWidth = modelLevel.width;
    int32_t minRotHeight = modelLevel.height;
    for (int32_t angleIdx : angleIndices) {
        const auto& rotatedTemplate = rotatedTemplates_[level][angleIdx];
        if (!rotatedTemplate.IsValid()) {
            continue;
        }
        minRotWidth = std::min(minRotWidth, rotatedTemplate.width);
        minRotHeight = std::min(minRotHeight, rotatedTemplate.height);
    }
    searchEndX = imgWidth - minRotWidth;
    searchEndY = imgHeight - minRotHeight;

    // Minimum score for coarse level (lower than final threshold)
    // Reference: DennisLiu uses 0.9^level decay
    double coarseMinScore = params.minScore * 0.7;  // ~70% of target

    // Search all positions and angles
    std::vector<MatchResult> localCandidates;

#ifdef _OPENMP
    #pragma omp parallel
    {
        std::vector<MatchResult> threadCandidates;
        #pragma omp for collapse(2) schedule(dynamic)
#endif
        for (int32_t y = searchStartY; y <= searchEndY; y += step) {
            for (int32_t x = searchStartX; x <= searchEndX; x += step) {
                for (int32_t angleIdx : angleIndices) {
                    const auto& rotatedTemplate = rotatedTemplates_[level][angleIdx];
                    if (!rotatedTemplate.IsValid()) {
                        continue;
                    }
                    if (x + rotatedTemplate.width > imgWidth ||
                        y + rotatedTemplate.height > imgHeight) {
                        continue;
                    }

                    double score = ComputeNCCScore(integralImage,
                                                   imageLevel.data.data(),
                                                   imgWidth, imgHeight,
                                                   x, y, angleIdx, level);

                    if (score >= coarseMinScore) {
                        double originOffsetX = 0.0;
                        double originOffsetY = 0.0;
                        computeOriginOffset(angleIdx, rotatedTemplate, originOffsetX, originOffsetY);

                        MatchResult match;
                        match.x = x + originOffsetX;
                        match.y = y + originOffsetY;
                        match.angle = searchAngles_[angleIdx];
                        match.score = score;
                        match.pyramidLevel = level;
#ifdef _OPENMP
                        threadCandidates.push_back(match);
#else
                        localCandidates.push_back(match);
#endif
                    }
                }
            }
        }
#ifdef _OPENMP
        #pragma omp critical
        {
            localCandidates.insert(localCandidates.end(),
                                   threadCandidates.begin(),
                                   threadCandidates.end());
        }
    }
#endif

    // Sort by score
    std::sort(localCandidates.begin(), localCandidates.end());

    // Keep top candidates for refinement
    size_t maxCoarseCandidates = 1000;
    if (localCandidates.size() > maxCoarseCandidates) {
        localCandidates.resize(maxCoarseCandidates);
    }

    return localCandidates;
}

std::vector<MatchResult> NCCModelImpl::SearchLevel(
    const IntegralImagePyramid& integralPyramid,
    const Qi::Vision::Internal::ImagePyramid& imagePyramid,
    int32_t level,
    const std::vector<MatchResult>& candidates,
    const SearchParams& params) const
{
    std::vector<MatchResult> refined;

    if (level < 0 || level >= static_cast<int32_t>(levels_.size())) {
        return refined;
    }

    const auto& modelLevel = levels_[level];
    const auto& imageLevel = imagePyramid.GetLevel(level);
    const auto& integralImage = integralPyramid.GetLevel(level);

    if (!modelLevel.IsValid() || !imageLevel.IsValid()) {
        return refined;
    }

    int32_t imgWidth = imageLevel.width;
    int32_t imgHeight = imageLevel.height;

    // Refinement search radius (pixels)
    int32_t posRadius = 4;    // Larger for rotation
    int32_t angleRadius = 2;  // Angle indices

    // Minimum score for this level (90% decay per level)
    double minScore = (level == 0) ? params.minScore : params.minScore * 0.85;

    auto computeOriginOffset = [&](int32_t angleIdx, const RotatedTemplate& rotatedTemplate,
                                   double& outX, double& outY) {
        double angle = searchAngles_[angleIdx];
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

        outX = rotatedTemplate.offsetX + rotDx;
        outY = rotatedTemplate.offsetY + rotDy;
    };

    for (const auto& candidate : candidates) {
        // Scale position from previous level
        double scale = levels_[level].scale / levels_[level + 1].scale;
        double originX = candidate.x * scale;
        double originY = candidate.y * scale;

        // Get candidate angle index
        int32_t centerAngleIdx = GetAngleIndex(candidate.angle);

        // Search around candidate
        MatchResult best;
        best.score = -1.0;

        for (int32_t dy = -posRadius; dy <= posRadius; ++dy) {
            for (int32_t dx = -posRadius; dx <= posRadius; ++dx) {
                for (int32_t da = -angleRadius; da <= angleRadius; ++da) {
                    int32_t angleIdx = centerAngleIdx + da;
                    if (angleIdx < 0 || angleIdx >= static_cast<int32_t>(searchAngles_.size())) {
                        continue;
                    }

                    const auto& rotatedTemplate = rotatedTemplates_[level][angleIdx];
                    if (!rotatedTemplate.IsValid()) {
                        continue;
                    }

                    double originOffsetX = 0.0;
                    double originOffsetY = 0.0;
                    computeOriginOffset(angleIdx, rotatedTemplate, originOffsetX, originOffsetY);

                    int32_t x = static_cast<int32_t>(std::round(originX - originOffsetX)) + dx;
                    int32_t y = static_cast<int32_t>(std::round(originY - originOffsetY)) + dy;
                    if (x < 0 || y < 0 ||
                        x + rotatedTemplate.width > imgWidth ||
                        y + rotatedTemplate.height > imgHeight) {
                        continue;
                    }

                    double score = ComputeNCCScore(integralImage,
                                                   imageLevel.data.data(),
                                                   imgWidth, imgHeight,
                                                   x, y, angleIdx, level);

                    if (score > best.score) {
                        best.x = x + originOffsetX;
                        best.y = y + originOffsetY;
                        best.angle = searchAngles_[angleIdx];
                        best.score = score;
                        best.pyramidLevel = level;
                    }
                }
            }
        }

        if (best.score >= minScore) {
            // Subpixel refinement at level 0
            if (level == 0 && params.subpixelMethod != SubpixelMethod::None) {
                RefinePosition(integralImage, imageLevel.data.data(),
                               imgWidth, imgHeight, best, level);
            }

            refined.push_back(best);
        }
    }

    // Sort by score
    std::sort(refined.begin(), refined.end());

    // Remove duplicates (close positions)
    if (refined.size() > 1) {
        double minDist = std::min(templateSize_.width, templateSize_.height) * 0.25;
        refined = NonMaxSuppression(refined, minDist);
    }

    return refined;
}

int32_t NCCModelImpl::GetAngleIndex(double angle) const
{
    if (searchAngles_.empty()) {
        return 0;
    }

    // Find closest angle
    int32_t bestIdx = 0;
    double bestDiff = std::abs(angle - searchAngles_[0]);

    for (size_t i = 1; i < searchAngles_.size(); ++i) {
        double diff = std::abs(angle - searchAngles_[i]);
        if (diff < bestDiff) {
            bestDiff = diff;
            bestIdx = static_cast<int32_t>(i);
        }
    }

    return bestIdx;
}

double NCCModelImpl::InterpolateAngle(double angle, int32_t& lowerIdx, int32_t& upperIdx,
                                       double& weight) const
{
    if (searchAngles_.empty()) {
        lowerIdx = 0;
        upperIdx = 0;
        weight = 0.0;
        return 0.0;
    }

    // Find bracketing angles
    for (size_t i = 0; i < searchAngles_.size() - 1; ++i) {
        if (angle >= searchAngles_[i] && angle <= searchAngles_[i + 1]) {
            lowerIdx = static_cast<int32_t>(i);
            upperIdx = static_cast<int32_t>(i + 1);
            double range = searchAngles_[i + 1] - searchAngles_[i];
            weight = (range > 1e-9) ? (angle - searchAngles_[i]) / range : 0.0;
            return angle;
        }
    }

    // Angle outside range, use nearest
    lowerIdx = GetAngleIndex(angle);
    upperIdx = lowerIdx;
    weight = 0.0;
    return searchAngles_[lowerIdx];
}

} // namespace Internal
} // namespace Qi::Vision::Matching
