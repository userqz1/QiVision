/**
 * @file ShapeModelSearch.cpp
 * @brief Search functions for ShapeModel
 *
 * Contains:
 * - SearchPyramid
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

    // Coarse angle step for top-level search
    // Rule: Pyramid refinement searches ±6° at first level, halving each level down.
    // So coarse step can be up to 2*6°=12° to ensure overlap with refinement.
    // We use 10° as a safe default, but scale down for small models.
    //
    // For small models: fine step might be ~0.5° (model radius ~100px)
    //   -> coarse step = max(fine * 4, 10°) = 10° (~36 angles for 360°)
    // For large models: fine step might be ~0.3° (model radius ~200px)
    //   -> coarse step = max(fine * 4, 10°) = 10°
    //
    // The factor of 4 ensures coarse step is at least 4x fine step for efficiency,
    // while the 10° minimum prevents too many angles in coarse search.
    constexpr double PYRAMID_REFINE_ANGLE_RANGE = 6.0 * PI / 180.0;  // ±6° at first refine level

    // Coarse step can be larger than 2*6°=12° because:
    // 1. Refinement has some tolerance for nearby angles
    // 2. Score function still gives reasonable scores within ±10° of true angle
    // Tested: 15° works reliably, 20° works for most cases, 25° fails some
    constexpr double COARSE_ANGLE_STEP_BASE = 15.0 * PI / 180.0;  // ~15°

    // Start with model-based estimate, but ensure minimum efficiency
    double coarseAngleStep = std::max(fineAngleStep * 4.0, COARSE_ANGLE_STEP_BASE);

    // Cap at 18° (aggressive but tested safe for most cases)
    constexpr double MAX_COARSE_ANGLE_STEP = 18.0 * PI / 180.0;  // ~18°
    coarseAngleStep = std::min(coarseAngleStep, MAX_COARSE_ANGLE_STEP);

    // Coarse search at top level
    int32_t targetWidth = targetPyramid.GetWidth(startLevel);
    int32_t targetHeight = targetPyramid.GetHeight(startLevel);

    // Search grid at top level
    int32_t stepSize = 2;

    // Use pregenerated angle cache if available (Halcon pregeneration strategy)
    // This avoids computing cos/sin and bounds for each angle during search
    const bool usePregenCache = !searchAngleCache_.empty() &&
                                 static_cast<size_t>(startLevel) < levels_.size() &&
                                 searchAngleStep_ > 0;

    if (usePregenCache) {
        // Calculate coarse angle stride (skip angles for faster coarse search)
        // e.g., if cache has 755 angles and we want ~60 for coarse search, stride = ~12
        const int32_t coarseStride = std::max(1, static_cast<int32_t>(
            coarseAngleStep / searchAngleStep_));

        // Build angle index list for search range
        std::vector<size_t> angleIndices;
        for (size_t ai = 0; ai < searchAngleCache_.size(); ai += coarseStride) {
            const double angle = searchAngleCache_[ai].angle;
            // Filter by search params range
            if (angle >= params.angleStart - 0.001 &&
                angle <= params.angleStart + params.angleExtent + 0.001) {
                angleIndices.push_back(ai);
            }
        }

        // OpenMP parallelization over angle indices
        #pragma omp parallel
        {
            std::vector<MatchResult> localCandidates;

            #pragma omp for schedule(dynamic)
            for (size_t ii = 0; ii < angleIndices.size(); ++ii) {
                const size_t ai = angleIndices[ii];
                const SearchAngleData& angleData = searchAngleCache_[ai];

                // Use precomputed cos/sin (no std::cos/sin call!)
                const float cosR = angleData.cosA;
                const float sinR = angleData.sinA;
                const double angle = angleData.angle;

                // Use precomputed rotated bounds (no ComputeRotatedBounds call!)
                const auto& bounds = angleData.levelBounds[startLevel];

                // Valid search region (direct lookup instead of computation)
                int32_t searchXMin = std::max(0, -bounds.minX);
                int32_t searchXMax = std::min(targetWidth - 1, targetWidth - 1 - bounds.maxX);
                int32_t searchYMin = std::max(0, -bounds.minY);
                int32_t searchYMax = std::min(targetHeight - 1, targetHeight - 1 - bounds.maxY);

                if (searchXMin > searchXMax || searchYMin > searchYMax) {
                    continue;
                }

                for (int32_t y = searchYMin; y <= searchYMax; y += stepSize) {
                    for (int32_t x = searchXMin; x <= searchXMax; x += stepSize) {
                        double coverage = 0.0;
                        // Bilinear interpolation for accurate scoring
                        double score = ComputeScoreWithSinCos(targetPyramid, startLevel,
                                                               x, y, cosR, sinR, 1.0, params.greediness, &coverage,
                                                               false);

                        if (score >= params.minScore * 0.7 && coverage >= minCoverage_) {
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
    } else {
        // Fallback: Build angle list and compute bounds at runtime
        std::vector<double> angles;
        for (double angle = params.angleStart;
             angle <= params.angleStart + params.angleExtent;
             angle += coarseAngleStep) {
            angles.push_back(angle);
        }

        const auto& topLevel = levels_[startLevel];

        #pragma omp parallel
        {
            std::vector<MatchResult> localCandidates;

            #pragma omp for schedule(dynamic)
            for (size_t ai = 0; ai < angles.size(); ++ai) {
                const double angle = angles[ai];

                // Compute sin/cos at runtime
                const float cosR = static_cast<float>(std::cos(angle));
                const float sinR = static_cast<float>(std::sin(angle));

                // Compute rotated model bounds at runtime
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
                        // Bilinear interpolation for accurate scoring
                        double score = ComputeScoreWithSinCos(targetPyramid, startLevel,
                                                               x, y, cosR, sinR, 1.0, params.greediness, &coverage,
                                                               false);

                        if (score >= params.minScore * 0.7 && coverage >= minCoverage_) {
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

    // Apply coverage penalty and filter by final score threshold
    // Coverage penalty is only applied at final output stage (not during search)
    // This ensures search doesn't miss true matches due to coverage penalty
    std::vector<MatchResult> results;
    for (auto match : candidates) {
        // Recompute score with coverage at level 0
        double coverage = 0.0;
        double similarity = ComputeScoreAtPosition(targetPyramid, 0,
                                                    match.x, match.y, match.angle, 1.0,
                                                    0.0, &coverage, false);

        // Apply coverage penalty: score = similarity * coverage^0.75
        double coveragePenalty = std::pow(coverage, 0.75);
        match.score = similarity * coveragePenalty;

        if (match.score >= params.minScore) {
            results.push_back(match);
        }
    }

    // Apply non-maximum suppression
    // Use half of model's smaller dimension as minimum distance
    double nmsDistance = std::min(templateSize_.width, templateSize_.height) * 0.5;
    nmsDistance = std::max(nmsDistance, 10.0);  // At least 10 pixels
    results = NonMaxSuppression(results, nmsDistance);

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

    // Parallel refinement over candidates (each candidate is independent)
    const int32_t numCandidates = static_cast<int32_t>(candidates.size());
    std::vector<MatchResult> allResults(numCandidates);
    std::vector<bool> validResults(numCandidates, false);

    #pragma omp parallel for schedule(dynamic, 4)
    for (int32_t ci = 0; ci < numCandidates; ++ci) {
        const auto& candidate = candidates[ci];
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

                    if (coverage >= minCoverage_ && score > bestMatch.score) {
                        bestMatch.x = x;
                        bestMatch.y = y;
                        bestMatch.angle = angle;
                        bestMatch.score = score;
                    }
                }
            }
        }

        if (bestMatch.score >= params.minScore * 0.7) {
            allResults[ci] = bestMatch;
            validResults[ci] = true;
        }
    }

    // Collect valid results
    for (int32_t ci = 0; ci < numCandidates; ++ci) {
        if (validResults[ci]) {
            refined.push_back(allResults[ci]);
        }
    }

    return refined;
}

} // namespace Internal
} // namespace Qi::Vision::Matching
