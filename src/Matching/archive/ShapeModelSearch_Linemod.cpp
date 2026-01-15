/**
 * @file ShapeModelSearch_Linemod.cpp
 * @brief ARCHIVED: LINEMOD-based search implementation (8-bin quantization)
 *
 * This file contains the LINEMOD search code that was archived in favor of XLD.
 * LINEMOD uses 8-bin orientation quantization which causes accuracy issues
 * at bin boundaries (e.g., 155°, 337°).
 *
 * Archived: January 2025
 * Reason: XLD (continuous gradient) provides better accuracy for all angles
 *
 * @note To restore this functionality:
 * 1. Copy SearchPyramidLinemod back to ShapeModelSearch.cpp
 * 2. Add the declaration to ShapeModelImpl.h
 * 3. Add LINEMOD data members (linemodFeatures_, etc.)
 * 4. Update ShapeModel.cpp to call SearchPyramidLinemod when useLinemod=true
 */

#if 0  // ARCHIVED CODE - NOT COMPILED

#include "ShapeModelImpl.h"
#include <algorithm>
#include <chrono>

namespace Qi::Vision::Matching {
namespace Internal {

// =============================================================================
// ShapeModelImpl::SearchPyramidLinemod
// =============================================================================

std::vector<MatchResult> ShapeModelImpl::SearchPyramidLinemod(
    const LinemodPyramid& targetPyramid,
    const SearchParams& params) const
{
    if (!valid_ || levels_.empty() || linemodFeatures_.empty()) {
        return {};
    }

    // Performance timing
    auto t0 = std::chrono::high_resolution_clock::now();
    auto t1 = t0, t2 = t0, t3 = t0;

    // Start from coarsest level that has enough features for reliable matching
    constexpr int32_t MIN_FEATURES_FOR_COARSE = 32;

    int32_t startLevel = std::min(static_cast<int32_t>(linemodFeatures_.size()) - 1,
                                   targetPyramid.NumLevels() - 1);

    while (startLevel > 0 &&
           (linemodFeatures_[startLevel].empty() ||
            static_cast<int32_t>(linemodFeatures_[startLevel].size()) < MIN_FEATURES_FOR_COARSE)) {
        startLevel--;
    }

    if (startLevel < 0 || linemodFeatures_[startLevel].empty()) {
        return {};
    }

    std::vector<MatchResult> candidates;

    int32_t targetWidth = targetPyramid.GetWidth(startLevel);
    int32_t targetHeight = targetPyramid.GetHeight(startLevel);

    // Search grid step
    constexpr int32_t stepSize = 3;

    // Coarse search threshold
    double coarseThreshold = std::max(0.5, params.minScore * 0.7);

    // =========================================================================
    // HIERARCHICAL ANGLE SEARCH using pre-generated angle templates
    // Phase 1: Coarse search with pre-generated templates (10° step)
    // Phase 2: Fine angle search in candidate regions (2° step)
    // =========================================================================

    // Check if we have pre-generated angle templates
    bool usePregenTemplates = !linemodAngleTemplates_.empty() &&
                               !linemodTemplateAngles_.empty();

    if (usePregenTemplates) {
        // Phase 1: Coarse search using pre-generated angle templates
        // These templates have accurate orientation (no 8-bin quantization error)

        #pragma omp parallel
        {
            std::vector<MatchResult> localCandidates;

            #pragma omp for schedule(dynamic)
            for (size_t ti = 0; ti < linemodTemplateAngles_.size(); ++ti) {
                double templateAngle = linemodTemplateAngles_[ti];

                // Skip templates outside search angle range
                if (templateAngle < params.angleStart - 0.1 ||
                    templateAngle > params.angleStart + params.angleExtent + 0.1) {
                    continue;
                }

                // Get pre-generated features for this angle
                if (ti >= linemodAngleTemplates_.size() ||
                    startLevel >= static_cast<int32_t>(linemodAngleTemplates_[ti].size())) {
                    continue;
                }

                const auto& templateFeatures = linemodAngleTemplates_[ti][startLevel];
                if (templateFeatures.empty()) continue;

                // Compute template bounds
                int32_t rotMinX = 0, rotMaxX = 0, rotMinY = 0, rotMaxY = 0;
                for (const auto& f : templateFeatures) {
                    rotMinX = std::min(rotMinX, static_cast<int32_t>(f.x));
                    rotMaxX = std::max(rotMaxX, static_cast<int32_t>(f.x));
                    rotMinY = std::min(rotMinY, static_cast<int32_t>(f.y));
                    rotMaxY = std::max(rotMaxY, static_cast<int32_t>(f.y));
                }

                // Valid search region
                int32_t searchXMin = std::max(0, -rotMinX);
                int32_t searchXMax = std::min(targetWidth - 1, targetWidth - 1 - rotMaxX);
                int32_t searchYMin = std::max(0, -rotMinY);
                int32_t searchYMax = std::min(targetHeight - 1, targetHeight - 1 - rotMaxY);

                if (searchXMin > searchXMax || searchYMin > searchYMax) {
                    continue;
                }

                // Search all positions
                alignas(32) double scores8[8];

                for (int32_t y = searchYMin; y <= searchYMax; y += stepSize) {
                    int32_t xOffset = searchXMin % stepSize;
                    int32_t x = searchXMin;

                    for (; x + 7 <= searchXMax; x += 8) {
                        targetPyramid.ComputeScoresBatch8(templateFeatures, startLevel, x, y, scores8);

                        for (int32_t i = 0; i < 8; ++i) {
                            if (((x + i) % stepSize) != xOffset) continue;

                            if (scores8[i] >= coarseThreshold) {
                                MatchResult result;
                                result.x = x + i;
                                result.y = y;
                                result.angle = templateAngle;
                                result.score = scores8[i];
                                result.pyramidLevel = startLevel;
                                localCandidates.push_back(result);
                            }
                        }
                    }

                    for (; x <= searchXMax; x += stepSize) {
                        double score = targetPyramid.ComputeScorePrecomputed(
                            templateFeatures, startLevel, x, y);

                        if (score >= coarseThreshold) {
                            MatchResult result;
                            result.x = x;
                            result.y = y;
                            result.angle = templateAngle;
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

        // Phase 2: Fine angle search around coarse candidates
        // Search ±5° around each candidate with 2° step
        if (!candidates.empty()) {
            std::sort(candidates.begin(), candidates.end());
            if (candidates.size() > 100) {
                candidates.resize(100);
            }

            constexpr double FINE_ANGLE_RADIUS = 5.0 * PI / 180.0;  // ±5°
            constexpr double FINE_ANGLE_STEP = 2.0 * PI / 180.0;    // 2°

            const auto& modelFeatures = linemodFeatures_[startLevel];

            std::vector<MatchResult> refinedCandidates;
            refinedCandidates.reserve(candidates.size());

            for (const auto& c : candidates) {
                MatchResult best = c;

                // Search fine angles around coarse angle
                for (double dAngle = -FINE_ANGLE_RADIUS; dAngle <= FINE_ANGLE_RADIUS; dAngle += FINE_ANGLE_STEP) {
                    if (std::abs(dAngle) < 1e-6) continue;  // Skip center (already have coarse score)

                    double fineAngle = c.angle + dAngle;

                    // Use real-time rotation for fine search (small range = small error)
                    auto rotatedFeatures = LinemodPyramid::RotateFeatures(modelFeatures, fineAngle);

                    // Search small position neighborhood
                    for (int32_t dy = -1; dy <= 1; ++dy) {
                        for (int32_t dx = -1; dx <= 1; ++dx) {
                            int32_t px = c.x + dx;
                            int32_t py = c.y + dy;

                            if (px < 0 || px >= targetWidth || py < 0 || py >= targetHeight) {
                                continue;
                            }

                            double score = targetPyramid.ComputeScorePrecomputed(
                                rotatedFeatures, startLevel, px, py);

                            if (score > best.score) {
                                best.x = px;
                                best.y = py;
                                best.angle = fineAngle;
                                best.score = score;
                            }
                        }
                    }
                }

                refinedCandidates.push_back(best);
            }

            candidates = std::move(refinedCandidates);
        }

    } else {
        // Fallback: Original method with real-time rotation (for backward compatibility)
        constexpr double COARSE_ANGLE_STEP = 6.0 * PI / 180.0;
        double angleStep = std::max(params.angleStep, COARSE_ANGLE_STEP);

        std::vector<double> angles;
        for (double angle = params.angleStart;
             angle <= params.angleStart + params.angleExtent;
             angle += angleStep) {
            angles.push_back(angle);
        }

        const auto& modelFeatures = linemodFeatures_[startLevel];

        #pragma omp parallel
        {
            std::vector<MatchResult> localCandidates;

            #pragma omp for schedule(dynamic)
            for (size_t ai = 0; ai < angles.size(); ++ai) {
                const double angle = angles[ai];
                auto rotatedFeatures = LinemodPyramid::RotateFeatures(modelFeatures, angle);

                int32_t rotMinX = 0, rotMaxX = 0, rotMinY = 0, rotMaxY = 0;
                for (const auto& f : rotatedFeatures) {
                    rotMinX = std::min(rotMinX, static_cast<int32_t>(f.x));
                    rotMaxX = std::max(rotMaxX, static_cast<int32_t>(f.x));
                    rotMinY = std::min(rotMinY, static_cast<int32_t>(f.y));
                    rotMaxY = std::max(rotMaxY, static_cast<int32_t>(f.y));
                }

                int32_t searchXMin = std::max(0, -rotMinX);
                int32_t searchXMax = std::min(targetWidth - 1, targetWidth - 1 - rotMaxX);
                int32_t searchYMin = std::max(0, -rotMinY);
                int32_t searchYMax = std::min(targetHeight - 1, targetHeight - 1 - rotMaxY);

                if (searchXMin > searchXMax || searchYMin > searchYMax) {
                    continue;
                }

                alignas(32) double scores8[8];

                for (int32_t y = searchYMin; y <= searchYMax; y += stepSize) {
                    int32_t xOffset = searchXMin % stepSize;
                    int32_t x = searchXMin;

                    for (; x + 7 <= searchXMax; x += 8) {
                        targetPyramid.ComputeScoresBatch8(rotatedFeatures, startLevel, x, y, scores8);

                        for (int32_t i = 0; i < 8; ++i) {
                            if (((x + i) % stepSize) != xOffset) continue;

                            if (scores8[i] >= coarseThreshold) {
                                MatchResult result;
                                result.x = x + i;
                                result.y = y;
                                result.angle = angle;
                                result.score = scores8[i];
                                result.pyramidLevel = startLevel;
                                localCandidates.push_back(result);
                            }
                        }
                    }

                    for (; x <= searchXMax; x += stepSize) {
                        double score = targetPyramid.ComputeScorePrecomputed(
                            rotatedFeatures, startLevel, x, y);

                        if (score >= coarseThreshold) {
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

    // Sort candidates by score (descending)
    std::sort(candidates.begin(), candidates.end());

    t1 = std::chrono::high_resolution_clock::now();
    size_t coarseCandidates = candidates.size();

    if (timingParams_.enableTiming) {
        findTiming_.coarseSearchMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
        findTiming_.numCoarseCandidates = static_cast<int32_t>(coarseCandidates);
    }

    // Limit candidates for efficiency - be more aggressive
    if (candidates.size() > 200) {
        candidates.resize(200);
    }

    // Refine through pyramid levels using LINEMOD scoring
    for (int32_t level = startLevel - 1; level >= 0; --level) {
        if (level >= static_cast<int32_t>(linemodFeatures_.size()) ||
            linemodFeatures_[level].empty()) {
            continue;
        }

        std::vector<MatchResult> refined;
        refined.reserve(candidates.size());

        double scaleFactor = 2.0;
        int32_t searchRadius = (level == 0) ? 3 : 2;
        double angleRadius = (level == 0) ? 0.05 : 0.1;  // ~3 or ~6 degrees
        double levelAngleStep = (level == 0) ? 0.01 : 0.025;

        int32_t levelWidth = targetPyramid.GetWidth(level);
        int32_t levelHeight = targetPyramid.GetHeight(level);
        const auto& levelFeatures = linemodFeatures_[level];

        // Build angle list for this level's refinement
        std::vector<double> refineAngles;
        for (double dAngle = -angleRadius; dAngle <= angleRadius; dAngle += levelAngleStep) {
            refineAngles.push_back(dAngle);
        }

        // Precompute rotated features for each angle offset
        std::vector<std::vector<LinemodFeature>> rotatedTemplates;
        rotatedTemplates.resize(refineAngles.size());

        for (const auto& candidate : candidates) {
            double baseX = candidate.x * scaleFactor;
            double baseY = candidate.y * scaleFactor;
            double baseAngle = candidate.angle;

            // Precompute rotated templates for all angle variations around this candidate
            for (size_t ai = 0; ai < refineAngles.size(); ++ai) {
                double angle = baseAngle + refineAngles[ai];
                rotatedTemplates[ai] = LinemodPyramid::RotateFeatures(levelFeatures, angle);
            }

            MatchResult bestMatch;
            bestMatch.score = -1.0;

            for (int32_t dy = -searchRadius; dy <= searchRadius; ++dy) {
                for (int32_t dx = -searchRadius; dx <= searchRadius; ++dx) {
                    int32_t px = static_cast<int32_t>(baseX) + dx;
                    int32_t py = static_cast<int32_t>(baseY) + dy;

                    if (px < 0 || px >= levelWidth || py < 0 || py >= levelHeight) {
                        continue;
                    }

                    // Use precomputed templates for scoring
                    for (size_t ai = 0; ai < refineAngles.size(); ++ai) {
                        double score = targetPyramid.ComputeScorePrecomputed(
                            rotatedTemplates[ai], level, px, py);

                        if (score > bestMatch.score) {
                            bestMatch.x = px;
                            bestMatch.y = py;
                            bestMatch.angle = baseAngle + refineAngles[ai];
                            bestMatch.score = score;
                            bestMatch.pyramidLevel = level;
                        }
                    }
                }
            }

            if (bestMatch.score >= params.minScore * 0.8) {
                refined.push_back(bestMatch);
            }
        }

        candidates = std::move(refined);

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

    // Convert from pyramid level 0 to image coordinates
    double scale = levels_.empty() ? 1.0 : levels_[0].scale;
    for (auto& match : candidates) {
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

    // Apply non-maximum suppression
    results = NonMaxSuppression(results, 10.0);

    // Limit results
    if (params.maxMatches > 0 && static_cast<int32_t>(results.size()) > params.maxMatches) {
        results.resize(params.maxMatches);
    }

    t3 = std::chrono::high_resolution_clock::now();

    if (timingParams_.enableTiming) {
        findTiming_.nmsMs = std::chrono::duration<double, std::milli>(t3 - t2).count();
    }

    if (timingParams_.printTiming) {
        auto ms = [](auto start, auto end) {
            return std::chrono::duration<double, std::milli>(end - start).count();
        };
        fprintf(stderr, "[LINEMOD Timing] Coarse: %.1fms (%zu candidates), Refine: %.1fms, NMS: %.1fms | Total: %.1fms\n",
                ms(t0, t1), coarseCandidates, ms(t1, t2), ms(t2, t3), ms(t0, t3));
    }

    return results;
}

} // namespace Internal
} // namespace Qi::Vision::Matching

#endif  // ARCHIVED CODE
