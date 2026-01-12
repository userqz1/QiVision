/**
 * @file ShapeModelSearch.cpp
 * @brief Search functions for ShapeModel
 *
 * Contains:
 * - SearchPyramid
 * - SearchPyramidWithResponseMap
 * - SearchLevel
 */

#include "ShapeModelImpl.h"

#include <algorithm>
#include <chrono>
#include <cstdio>

namespace Qi::Vision::Matching {
namespace Internal {

// =============================================================================
// ShapeModelImpl::SearchPyramid
// =============================================================================

std::vector<MatchResult> ShapeModelImpl::SearchPyramid(
    const AnglePyramid& targetPyramid,
    const SearchParams& params) const
{
    if (!valid_ || levels_.empty()) {
        return {};
    }

    // Performance timing
    auto t0 = std::chrono::high_resolution_clock::now();
    auto t1 = t0, t2 = t0, t3 = t0, t4 = t0;

    // Start from coarsest level
    int32_t startLevel = std::min(static_cast<int32_t>(levels_.size()) - 1,
                                   targetPyramid.NumLevels() - 1);

    // Initialize candidates at top level (search all positions)
    std::vector<MatchResult> candidates;

    // Fine angle step for final refinement (used in SearchLevel)
    double fineAngleStep = params.angleStep;
    if (fineAngleStep <= 0) {
        fineAngleStep = EstimateAngleStep(std::max(templateSize_.width, templateSize_.height));
    }

    // COARSE angle step for top-level search: 6° = ~0.1 radians
    constexpr double COARSE_ANGLE_STEP = 0.1;
    double coarseAngleStep = std::max(fineAngleStep, COARSE_ANGLE_STEP);

    // Coarse search at top level
    const auto& topLevel = levels_[startLevel];
    int32_t targetWidth = targetPyramid.GetWidth(startLevel);
    int32_t targetHeight = targetPyramid.GetHeight(startLevel);

    // Search grid at top level
    int32_t stepSize = 2;

    // Build COARSE angle list for parallel processing
    std::vector<double> angles;
    for (double angle = params.angleStart;
         angle <= params.angleStart + params.angleExtent;
         angle += coarseAngleStep) {
        angles.push_back(angle);
    }

    // OpenMP parallelization over angles
    #pragma omp parallel
    {
        std::vector<MatchResult> localCandidates;

        #pragma omp for schedule(dynamic)
        for (size_t ai = 0; ai < angles.size(); ++ai) {
            const double angle = angles[ai];

            // Precompute sin/cos once per angle
            const float cosR = static_cast<float>(std::cos(angle));
            const float sinR = static_cast<float>(std::sin(angle));

            // Compute rotation bin for quantized scoring
            double normAngle = angle - std::floor(angle / (2.0 * PI)) * (2.0 * PI);
            int32_t rotationBin = (numAngleBins_ > 0) ?
                static_cast<int32_t>(normAngle * numAngleBins_ / (2.0 * PI)) % numAngleBins_ : 0;
            (void)rotationBin;  // Reserved for future use

            // Compute rotated model bounds
            double rMinX, rMaxX, rMinY, rMaxY;
            ComputeRotatedBounds(topLevel.points, angle, rMinX, rMaxX, rMinY, rMaxY);

            // Valid search region
            int32_t searchXMin = static_cast<int32_t>(std::ceil(-rMinX));
            int32_t searchXMax = static_cast<int32_t>(std::floor(targetWidth - 1 - rMaxX));
            int32_t searchYMin = static_cast<int32_t>(std::ceil(-rMinY));
            int32_t searchYMax = static_cast<int32_t>(std::floor(targetHeight - 1 - rMaxY));

            searchXMin = std::max(0, searchXMin);
            searchYMin = std::max(0, searchYMin);
            searchXMax = std::min(targetWidth - 1, searchXMax);
            searchYMax = std::min(targetHeight - 1, searchYMax);

            if (searchXMin > searchXMax || searchYMin > searchYMax) {
                continue;
            }

            for (int32_t y = searchYMin; y <= searchYMax; y += stepSize) {
                for (int32_t x = searchXMin; x <= searchXMax; x += stepSize) {
                    double coverage = 0.0;
                    double score = ComputeScoreWithSinCos(targetPyramid, startLevel,
                                                           x, y, cosR, sinR, 1.0, params.greediness, &coverage,
                                                           false);

                    if (score >= params.minScore * 0.7 && coverage >= 0.7) {
                        MatchResult result;
                        result.x = x;
                        result.y = y;
                        result.angle = angle;
                        result.score = score;
                        result.pyramidLevel = startLevel;
                        localCandidates.push_back(result);
                    }
                }
            }
        }

        #pragma omp critical
        {
            candidates.insert(candidates.end(), localCandidates.begin(), localCandidates.end());
        }
    }

    // Sort candidates by score
    std::sort(candidates.begin(), candidates.end());

    t1 = std::chrono::high_resolution_clock::now();
    size_t coarseCandidates = candidates.size();

    if (timingParams_.enableTiming) {
        findTiming_.coarseSearchMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
        findTiming_.numCoarseCandidates = static_cast<int32_t>(coarseCandidates);
    }

    // Limit candidates for efficiency
    if (candidates.size() > 200) {
        candidates.resize(200);
    }

    // Refine through pyramid levels
    for (int32_t level = startLevel - 1; level >= 0; --level) {
        double angleRadiusDeg = 6.0 / (1 << (startLevel - 1 - level));
        angleRadiusDeg = std::max(1.0, angleRadiusDeg);

        candidates = SearchLevel(targetPyramid, level, candidates, params, -1, angleRadiusDeg);

        std::sort(candidates.begin(), candidates.end());
        size_t limit = (level == 0) ? 50 : 100;
        if (candidates.size() > limit) {
            candidates.resize(limit);
        }
    }

    t2 = std::chrono::high_resolution_clock::now();

    if (timingParams_.enableTiming) {
        findTiming_.pyramidRefineMs = std::chrono::duration<double, std::milli>(t2 - t1).count();
    }

    // Final refinement at level 0
    for (auto& match : candidates) {
        if (params.subpixelMethod != SubpixelMethod::None) {
            RefinePosition(targetPyramid, match, params.subpixelMethod);
        }

        double scale = levels_[0].scale;
        if (scale != 1.0) {
            match.x /= scale;
            match.y /= scale;
        }
    }

    t3 = std::chrono::high_resolution_clock::now();

    if (timingParams_.enableTiming) {
        findTiming_.subpixelRefineMs = std::chrono::duration<double, std::milli>(t3 - t2).count();
    }

    // Filter by final score threshold
    std::vector<MatchResult> results;
    for (const auto& match : candidates) {
        if (match.score >= params.minScore) {
            results.push_back(match);
        }
    }

    // Apply non-maximum suppression
    results = NonMaxSuppression(results, 10.0);

    // Limit results
    if (params.maxMatches > 0 && static_cast<int32_t>(results.size()) > params.maxMatches) {
        results.resize(params.maxMatches);
    }

    t4 = std::chrono::high_resolution_clock::now();

    if (timingParams_.enableTiming) {
        findTiming_.nmsMs = std::chrono::duration<double, std::milli>(t4 - t3).count();
    }

    if (timingParams_.printTiming) {
        auto ms = [](auto start, auto end) {
            return std::chrono::duration<double, std::milli>(end - start).count();
        };
        fprintf(stderr, "[Timing] Coarse: %.1fms (%zu candidates), Refine: %.1fms, SubPix: %.1fms, NMS: %.1fms | Total: %.1fms\n",
                ms(t0, t1), coarseCandidates, ms(t1, t2), ms(t2, t3), ms(t3, t4), ms(t0, t4));
    }

    return results;
}

// =============================================================================
// ShapeModelImpl::SearchPyramidWithResponseMap
// =============================================================================

std::vector<MatchResult> ShapeModelImpl::SearchPyramidWithResponseMap(
    const AnglePyramid& targetPyramid,
    const ResponseMap& responseMap,
    const SearchParams& params) const
{
    if (!valid_ || levels_.empty()) {
        return {};
    }

    // Start from coarsest level
    int32_t startLevel = std::min(static_cast<int32_t>(levels_.size()) - 1,
                                   targetPyramid.NumLevels() - 1);
    startLevel = std::min(startLevel, responseMap.NumLevels() - 1);

    double angleStep = params.angleStep;
    if (angleStep <= 0) {
        angleStep = EstimateAngleStep(std::max(templateSize_.width, templateSize_.height));
    }

    // Precompute rotated models for all angles
    const auto& topLevelModel = levels_[startLevel];
    auto rotatedModels = ResponseMap::PrepareAllRotations(
        topLevelModel.points, params.angleStart, params.angleExtent, angleStep);

    // Coarse search at top level using Response Map
    std::vector<MatchResult> candidates;
    int32_t targetWidth = responseMap.GetWidth(startLevel);
    int32_t targetHeight = responseMap.GetHeight(startLevel);

    int32_t stepSize = 2;

    #pragma omp parallel
    {
        std::vector<MatchResult> localCandidates;

        #pragma omp for schedule(dynamic)
        for (size_t angleIdx = 0; angleIdx < rotatedModels.size(); ++angleIdx) {
            const auto& rotatedModel = rotatedModels[angleIdx];

            int32_t searchXMin = std::max(0, -static_cast<int32_t>(rotatedModel.minX));
            int32_t searchXMax = std::min(targetWidth - 1,
                                           targetWidth - 1 - static_cast<int32_t>(rotatedModel.maxX));
            int32_t searchYMin = std::max(0, -static_cast<int32_t>(rotatedModel.minY));
            int32_t searchYMax = std::min(targetHeight - 1,
                                           targetHeight - 1 - static_cast<int32_t>(rotatedModel.maxY));

            if (searchXMin > searchXMax || searchYMin > searchYMax) {
                continue;
            }

            for (int32_t y = searchYMin; y <= searchYMax; y += stepSize) {
                for (int32_t x = searchXMin; x <= searchXMax; x += stepSize) {
                    double coverage = 0.0;
                    double score = responseMap.ComputeScore(rotatedModel, startLevel,
                                                            x, y, &coverage);

                    if (score >= params.minScore * 0.5 && coverage >= 0.7) {
                        MatchResult result;
                        result.x = x;
                        result.y = y;
                        result.angle = rotatedModel.angle;
                        result.score = score;
                        result.pyramidLevel = startLevel;
                        localCandidates.push_back(result);
                    }
                }
            }
        }

        #pragma omp critical
        {
            candidates.insert(candidates.end(), localCandidates.begin(), localCandidates.end());
        }
    }

    std::sort(candidates.begin(), candidates.end());

    if (candidates.size() > 1000) {
        candidates.resize(1000);
    }

    // Refine through pyramid levels using PRECISE scoring
    for (int32_t level = startLevel - 1; level >= 0; --level) {
        std::vector<MatchResult> refined;
        refined.reserve(candidates.size());

        double scaleFactor = 2.0;
        int32_t searchRadius = 2;
        double angleRadius = 0.1;

        int32_t levelWidth = targetPyramid.GetWidth(level);
        int32_t levelHeight = targetPyramid.GetHeight(level);

        for (const auto& candidate : candidates) {
            double baseX = candidate.x * scaleFactor;
            double baseY = candidate.y * scaleFactor;
            double baseAngle = candidate.angle;

            MatchResult bestMatch;
            bestMatch.score = -1.0;
            double bestCoverage = 0.0;

            for (int32_t dy = -searchRadius; dy <= searchRadius; ++dy) {
                for (int32_t dx = -searchRadius; dx <= searchRadius; ++dx) {
                    double px = baseX + dx;
                    double py = baseY + dy;

                    if (px < 0 || px >= levelWidth || py < 0 || py >= levelHeight) {
                        continue;
                    }

                    for (double dAngle = -angleRadius; dAngle <= angleRadius; dAngle += 0.02) {
                        double angle = baseAngle + dAngle;
                        double coverage = 0.0;
                        double score = ComputeScoreAtPosition(targetPyramid, level,
                                                              px, py, angle, 1.0,
                                                              params.greediness, &coverage);

                        if (coverage >= 0.7 && score > bestMatch.score) {
                            bestMatch.x = px;
                            bestMatch.y = py;
                            bestMatch.angle = angle;
                            bestMatch.score = score;
                            bestMatch.pyramidLevel = level;
                            bestCoverage = coverage;
                        }
                    }
                }
            }

            if (bestMatch.score >= params.minScore * 0.5 && bestCoverage >= 0.6) {
                refined.push_back(bestMatch);
            }
        }

        candidates = std::move(refined);

        std::sort(candidates.begin(), candidates.end());
        if (candidates.size() > 500) {
            candidates.resize(500);
        }
    }

    // Final refinement at level 0
    for (auto& match : candidates) {
        double coverage = 0.0;
        match.score = ComputeScoreAtPosition(targetPyramid, 0,
                                              match.x, match.y, match.angle, 1.0,
                                              0.0, &coverage);

        if (params.subpixelMethod != SubpixelMethod::None) {
            RefinePosition(targetPyramid, match, params.subpixelMethod);
        }

        double scale = levels_[0].scale;
        if (scale != 1.0) {
            match.x /= scale;
            match.y /= scale;
        }
    }

    // Filter by final score threshold
    std::vector<MatchResult> results;
    for (const auto& match : candidates) {
        if (match.score >= params.minScore) {
            results.push_back(match);
        }
    }

    results = NonMaxSuppression(results, 10.0);

    if (params.maxMatches > 0 && static_cast<int32_t>(results.size()) > params.maxMatches) {
        results.resize(params.maxMatches);
    }

    return results;
}

// =============================================================================
// ShapeModelImpl::SearchLevel
// =============================================================================

std::vector<MatchResult> ShapeModelImpl::SearchLevel(
    const AnglePyramid& targetPyramid,
    int32_t level,
    const std::vector<MatchResult>& candidates,
    const SearchParams& params,
    int32_t positionRadius,
    double angleRadiusDeg) const
{
    std::vector<MatchResult> refined;
    refined.reserve(candidates.size());

    double scaleFactor = 2.0;

    int32_t searchRadius = (positionRadius >= 0) ? positionRadius : ((level == 0) ? 2 : 1);

    double angleRadius, angleStep;
    if (angleRadiusDeg >= 0) {
        angleRadius = angleRadiusDeg * PI / 180.0;
        angleStep = std::max(0.005, angleRadius / 3.0);
    } else {
        angleRadius = (level == 0) ? 0.035 : 0.1;
        angleStep = (level == 0) ? 0.01 : 0.035;
    }

    int32_t targetWidth = targetPyramid.GetWidth(level);
    int32_t targetHeight = targetPyramid.GetHeight(level);

    for (const auto& candidate : candidates) {
        double baseX = candidate.x * scaleFactor;
        double baseY = candidate.y * scaleFactor;
        double baseAngle = candidate.angle;

        MatchResult bestMatch;
        bestMatch.score = -1.0;
        bestMatch.x = baseX;
        bestMatch.y = baseY;
        bestMatch.pyramidLevel = level;

        for (int32_t dy = -searchRadius; dy <= searchRadius; ++dy) {
            for (int32_t dx = -searchRadius; dx <= searchRadius; ++dx) {
                double x = baseX + dx;
                double y = baseY + dy;

                if (x < 0 || x >= targetWidth || y < 0 || y >= targetHeight) {
                    continue;
                }

                for (double dAngle = -angleRadius; dAngle <= angleRadius; dAngle += angleStep) {
                    double angle = baseAngle + dAngle;

                    double coverage = 0.0;
                    double score = ComputeScoreAtPosition(targetPyramid, level,
                                                           x, y, angle, 1.0, params.greediness, &coverage,
                                                           false);

                    if (coverage >= 0.7 && score > bestMatch.score) {
                        bestMatch.x = x;
                        bestMatch.y = y;
                        bestMatch.angle = angle;
                        bestMatch.score = score;
                    }
                }
            }
        }

        if (bestMatch.score >= params.minScore * 0.7) {
            refined.push_back(bestMatch);
        }
    }

    return refined;
}

} // namespace Internal
} // namespace Qi::Vision::Matching
