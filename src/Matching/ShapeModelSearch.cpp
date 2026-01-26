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

    // HALCON search domain constraint:
    // The search ROI defines where the reference point can be located
    // Scale ROI to pyramid level coordinates
    double levelScale = targetPyramid.GetScale(startLevel);
    Rect2i levelROI;
    bool hasSearchROI = (params.searchROI.width > 0 && params.searchROI.height > 0);
    if (hasSearchROI) {
        levelROI.x = static_cast<int32_t>(params.searchROI.x * levelScale);
        levelROI.y = static_cast<int32_t>(params.searchROI.y * levelScale);
        levelROI.width = static_cast<int32_t>(params.searchROI.width * levelScale);
        levelROI.height = static_cast<int32_t>(params.searchROI.height * levelScale);
    }

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

                // Scale bounds by search scale factor
                const double scaleFactor = params.scaleMin;  // For scaled search
                int32_t scaledMinX = static_cast<int32_t>(bounds.minX * scaleFactor);
                int32_t scaledMaxX = static_cast<int32_t>(bounds.maxX * scaleFactor);
                int32_t scaledMinY = static_cast<int32_t>(bounds.minY * scaleFactor);
                int32_t scaledMaxY = static_cast<int32_t>(bounds.maxY * scaleFactor);

                // Valid search region (direct lookup instead of computation)
                int32_t searchXMin = std::max(0, -scaledMinX);
                int32_t searchXMax = std::min(targetWidth - 1, targetWidth - 1 - scaledMaxX);
                int32_t searchYMin = std::max(0, -scaledMinY);
                int32_t searchYMax = std::min(targetHeight - 1, targetHeight - 1 - scaledMaxY);

                // HALCON search domain constraint: intersect with ROI
                // The reference point must fall within the search domain
                if (hasSearchROI) {
                    searchXMin = std::max(searchXMin, levelROI.x);
                    searchXMax = std::min(searchXMax, levelROI.x + levelROI.width - 1);
                    searchYMin = std::max(searchYMin, levelROI.y);
                    searchYMax = std::min(searchYMax, levelROI.y + levelROI.height - 1);
                }

                if (searchXMin > searchXMax || searchYMin > searchYMax) {
                    continue;
                }

                for (int32_t y = searchYMin; y <= searchYMax; y += stepSize) {
                    for (int32_t x = searchXMin; x <= searchXMax; x += stepSize) {
                        double coverage = 0.0;
                        // Bilinear interpolation for accurate scoring
                        double score = ComputeScoreWithSinCos(targetPyramid, startLevel,
                                                               x, y, cosR, sinR, params.scaleMin, params.greediness, &coverage,
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

                // Scale bounds by search scale factor
                const double scaleFactor = params.scaleMin;
                rMinX *= scaleFactor;
                rMaxX *= scaleFactor;
                rMinY *= scaleFactor;
                rMaxY *= scaleFactor;

                // Valid search region
                int32_t searchXMin = static_cast<int32_t>(std::ceil(-rMinX));
                int32_t searchXMax = static_cast<int32_t>(std::floor(targetWidth - 1 - rMaxX));
                int32_t searchYMin = static_cast<int32_t>(std::ceil(-rMinY));
                int32_t searchYMax = static_cast<int32_t>(std::floor(targetHeight - 1 - rMaxY));

                searchXMin = std::max(0, searchXMin);
                searchYMin = std::max(0, searchYMin);
                searchXMax = std::min(targetWidth - 1, searchXMax);
                searchYMax = std::min(targetHeight - 1, searchYMax);

                // HALCON search domain constraint: intersect with ROI
                if (hasSearchROI) {
                    searchXMin = std::max(searchXMin, levelROI.x);
                    searchXMax = std::min(searchXMax, levelROI.x + levelROI.width - 1);
                    searchYMin = std::max(searchYMin, levelROI.y);
                    searchYMax = std::min(searchYMax, levelROI.y + levelROI.height - 1);
                }

                if (searchXMin > searchXMax || searchYMin > searchYMax) {
                    continue;
                }

                for (int32_t y = searchYMin; y <= searchYMax; y += stepSize) {
                    for (int32_t x = searchXMin; x <= searchXMax; x += stepSize) {
                        double coverage = 0.0;
                        // Bilinear interpolation for accurate scoring
                        double score = ComputeScoreWithSinCos(targetPyramid, startLevel,
                                                               x, y, cosR, sinR, params.scaleMin, params.greediness, &coverage,
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
                                                    match.x, match.y, match.angle, params.scaleMin,
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
// ShapeModelImpl::SearchPyramidScaled
// =============================================================================

std::vector<MatchResult> ShapeModelImpl::SearchPyramidScaled(
    const AnglePyramid& targetPyramid,
    const SearchParams& params,
    double scale) const
{
    const auto* scaledModel = GetScaledModelData(scale);
    if (!scaledModel) {
        // Fallback to runtime scaling if cache not available
        SearchParams scaledParams = params;
        scaledParams.scaleMode = ScaleSearchMode::Uniform;
        scaledParams.scaleMin = scale;
        scaledParams.scaleMax = scale;
        return SearchPyramid(targetPyramid, scaledParams);
    }

    // Use cached scaled model (scale baked into model points)
    SearchParams scaledParams = params;
    scaledParams.scaleMode = ScaleSearchMode::Uniform;
    scaledParams.scaleMin = 1.0;
    scaledParams.scaleMax = 1.0;

    struct ModelSwapGuard {
        ShapeModelImpl& impl;
        std::vector<LevelModel> levelsBackup;
        Size2i templateSizeBackup;
        double modelMinXBackup;
        double modelMaxXBackup;
        double modelMinYBackup;
        double modelMaxYBackup;
        double minCoverageBackup;
        std::vector<SearchAngleData> searchAngleCacheBackup;
        double searchAngleStartBackup;
        double searchAngleExtentBackup;
        double searchAngleStepBackup;

        ModelSwapGuard(ShapeModelImpl& implRef, const ScaledModelData& scaled)
            : impl(implRef),
              levelsBackup(implRef.levels_),
              templateSizeBackup(implRef.templateSize_),
              modelMinXBackup(implRef.modelMinX_),
              modelMaxXBackup(implRef.modelMaxX_),
              modelMinYBackup(implRef.modelMinY_),
              modelMaxYBackup(implRef.modelMaxY_),
              minCoverageBackup(implRef.minCoverage_),
              searchAngleCacheBackup(implRef.searchAngleCache_),
              searchAngleStartBackup(implRef.searchAngleStart_),
              searchAngleExtentBackup(implRef.searchAngleExtent_),
              searchAngleStepBackup(implRef.searchAngleStep_) {
            impl.levels_ = scaled.levels;
            impl.templateSize_ = scaled.templateSize;
            impl.modelMinX_ = scaled.modelMinX;
            impl.modelMaxX_ = scaled.modelMaxX;
            impl.modelMinY_ = scaled.modelMinY;
            impl.modelMaxY_ = scaled.modelMaxY;
            impl.minCoverage_ = scaled.minCoverage;
            impl.searchAngleCache_ = scaled.searchAngleCache;
            impl.searchAngleStart_ = scaled.searchAngleStart;
            impl.searchAngleExtent_ = scaled.searchAngleExtent;
            impl.searchAngleStep_ = scaled.searchAngleStep;
        }

        ~ModelSwapGuard() {
            impl.levels_ = std::move(levelsBackup);
            impl.templateSize_ = templateSizeBackup;
            impl.modelMinX_ = modelMinXBackup;
            impl.modelMaxX_ = modelMaxXBackup;
            impl.modelMinY_ = modelMinYBackup;
            impl.modelMaxY_ = modelMaxYBackup;
            impl.minCoverage_ = minCoverageBackup;
            impl.searchAngleCache_ = std::move(searchAngleCacheBackup);
            impl.searchAngleStart_ = searchAngleStartBackup;
            impl.searchAngleExtent_ = searchAngleExtentBackup;
            impl.searchAngleStep_ = searchAngleStepBackup;
        }
    };

    ModelSwapGuard guard(const_cast<ShapeModelImpl&>(*this), *scaledModel);
    if (timingParams_.debugCreateModel) {
        double adjusted = scaledModel->minCoverage;
        double delta = std::fabs(scale - 1.0);
        if (delta > 0.05) adjusted = std::min(adjusted, 0.6);
        if (scale < 0.8 || scale > 1.2) adjusted = std::min(adjusted, 0.5);
        guard.impl.minCoverage_ = adjusted;
        std::printf("[ScaledSearch] scale=%.3f minCoverage=%.2f\n", scale, adjusted);
        std::fflush(stdout);
    }
    return SearchPyramid(targetPyramid, scaledParams);
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
                                                           x, y, angle, params.scaleMin, params.greediness, &coverage,
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
