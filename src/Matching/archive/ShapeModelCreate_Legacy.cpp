/**
 * @file ShapeModelCreate_Legacy.cpp
 * @brief ARCHIVED: Traditional edge-based model creation (Auto/XLDContour modes)
 *
 * This file contains the legacy model creation code that was replaced by LINEMOD.
 * Preserved for research and comparison purposes.
 *
 * Methods archived:
 * - CreateModel (using AnglePyramid)
 * - ExtractModelPoints (edge-based extraction)
 * - ExtractModelPointsXLD (XLD contour-based extraction)
 * - OptimizeModel (point reduction)
 *
 * @note To restore this functionality:
 * 1. Copy the CreateModel method back to ShapeModelImpl
 * 2. Add a conditional in the public API to call CreateModel vs CreateModelLinemod
 * 3. Ensure AnglePyramid is included and available
 */

#if 0  // ARCHIVED CODE - NOT COMPILED

#include "ShapeModelImpl.h"
#include <QiVision/Internal/Pyramid.h>
#include <QiVision/Internal/Canny.h>
#include <QiVision/Internal/ContourProcess.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <queue>
#include <unordered_map>

namespace Qi::Vision::Matching {
namespace Internal {

// =============================================================================
// ShapeModelImpl::CreateModel (LEGACY)
// =============================================================================

bool ShapeModelImpl::CreateModel(const QImage& image, const Rect2i& roi, const Point2d& origin) {
    // Reset timing
    createTiming_ = ShapeModelCreateTiming();
    auto tTotal = std::chrono::high_resolution_clock::now();
    auto tStep = tTotal;

    auto elapsedMs = [](auto start) {
        return std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - start).count();
    };

    // Build angle pyramid for template
    AnglePyramidParams pyramidParams;

    // Auto pyramid levels if not specified (Halcon: 'auto')
    if (params_.numLevels <= 0) {
        // Estimate based on template size
        int32_t templateSize = std::max(roi.width > 0 ? roi.width : image.Width(),
                                        roi.height > 0 ? roi.height : image.Height());
        pyramidParams.numLevels = EstimateOptimalLevels(image.Width(), image.Height(),
                                                         templateSize, templateSize);
    } else {
        pyramidParams.numLevels = params_.numLevels;
    }
    pyramidParams.smoothSigma = 0.5;

    // Disable NMS for shape-based matching (Halcon-style)
    // Shape matching uses gradient direction similarity, not edge detection
    // All pixels above contrast threshold participate in matching
    pyramidParams.useNMS = false;

    // For contrast auto-detection, use a very low initial threshold to get all gradients
    bool needAutoContrast = (params_.contrastMode == ContrastMode::Auto ||
                             params_.contrastMode == ContrastMode::AutoHysteresis ||
                             params_.contrastMode == ContrastMode::AutoMinSize);

    if (needAutoContrast) {
        pyramidParams.minContrast = 1.0;  // Capture all gradients for analysis
    } else {
        // Manual mode: use contrastLow if hysteresis, otherwise contrastHigh
        pyramidParams.minContrast = (params_.contrastLow > 0) ? params_.contrastLow : params_.contrastHigh;
    }

    // Extract ROI if specified
    QImage templateImg;
    if (roi.width > 0 && roi.height > 0) {
        templateImg = image.SubImage(roi.x, roi.y, roi.width, roi.height);
        templateSize_ = Size2i{roi.width, roi.height};
    } else {
        templateImg = image;
        templateSize_ = Size2i{image.Width(), image.Height()};
    }

    if (templateImg.Empty()) {
        return false;
    }

    // Build pyramid (with timing)
    tStep = std::chrono::high_resolution_clock::now();
    AnglePyramid pyramid;
    if (!pyramid.Build(templateImg, pyramidParams)) {
        return false;
    }
    if (timingParams_.enableTiming) {
        createTiming_.pyramidBuildMs = elapsedMs(tStep);
    }

    // Auto-detect contrast threshold if requested
    tStep = std::chrono::high_resolution_clock::now();
    if (needAutoContrast) {
        const auto& edgePoints = pyramid.GetEdgePoints(0);
        if (!edgePoints.empty()) {
            // Collect magnitudes and sort (ascending for Otsu)
            std::vector<double> magnitudes;
            magnitudes.reserve(edgePoints.size());
            for (const auto& ep : edgePoints) {
                magnitudes.push_back(ep.magnitude);
            }
            std::sort(magnitudes.begin(), magnitudes.end());

            double minMag = magnitudes.front();
            double maxMag = magnitudes.back();

            // Estimate target point count based on template area
            int32_t templateArea = templateSize_.width * templateSize_.height;
            int32_t targetPoints;
            if (templateArea < 2500) {
                targetPoints = std::min(300, static_cast<int32_t>(magnitudes.size() / 4));
            } else if (templateArea < 10000) {
                targetPoints = std::min(800, static_cast<int32_t>(magnitudes.size() / 4));
            } else if (templateArea < 40000) {
                targetPoints = std::min(1500, static_cast<int32_t>(magnitudes.size() / 5));
            } else {
                targetPoints = std::min(2500, static_cast<int32_t>(magnitudes.size() / 6));
            }

            // Helper: Compute Otsu threshold
            auto computeOtsu = [&]() -> double {
                const int32_t numBins = 256;
                double range = maxMag - minMag;
                if (range < 1e-6) return minMag;

                // Build histogram
                std::vector<int32_t> hist(numBins, 0);
                for (double val : magnitudes) {
                    int32_t bin = static_cast<int32_t>((val - minMag) / range * (numBins - 1));
                    bin = std::clamp(bin, 0, numBins - 1);
                    hist[bin]++;
                }

                // Otsu algorithm
                int32_t total = static_cast<int32_t>(magnitudes.size());
                double sumAll = 0.0;
                for (int32_t i = 0; i < numBins; ++i) {
                    sumAll += i * hist[i];
                }

                double sumBg = 0.0;
                int32_t weightBg = 0;
                double maxVariance = 0.0;
                int32_t bestThreshold = 0;

                for (int32_t t = 0; t < numBins; ++t) {
                    weightBg += hist[t];
                    if (weightBg == 0) continue;

                    int32_t weightFg = total - weightBg;
                    if (weightFg == 0) break;

                    sumBg += t * hist[t];
                    double meanBg = sumBg / weightBg;
                    double meanFg = (sumAll - sumBg) / weightFg;

                    double variance = static_cast<double>(weightBg) * weightFg *
                                      (meanBg - meanFg) * (meanBg - meanFg);

                    if (variance > maxVariance) {
                        maxVariance = variance;
                        bestThreshold = t;
                    }
                }

                return minMag + (bestThreshold + 0.5) * range / numBins;
            };

            if (params_.contrastMode == ContrastMode::Auto) {
                size_t targetIdx = (magnitudes.size() > static_cast<size_t>(targetPoints))
                    ? magnitudes.size() - targetPoints : 0;
                double percentileThreshold = magnitudes[targetIdx];
                double otsuThreshold = computeOtsu();

                params_.contrastHigh = std::max(percentileThreshold, otsuThreshold) * 0.6 +
                                       std::min(percentileThreshold, otsuThreshold) * 0.4;
                params_.contrastHigh = std::clamp(params_.contrastHigh, 5.0, maxMag * 0.9);
                params_.contrastLow = 0.0;
            }
            else if (params_.contrastMode == ContrastMode::AutoHysteresis) {
                double otsuThreshold = computeOtsu();
                size_t highIdx = magnitudes.size() * 3 / 4;
                double percentileHigh = magnitudes[highIdx];

                params_.contrastHigh = std::max(otsuThreshold, percentileHigh);
                params_.contrastHigh = std::clamp(params_.contrastHigh, 8.0, maxMag * 0.85);

                size_t lowIdx = magnitudes.size() / 2;
                double medianMag = magnitudes[lowIdx];

                params_.contrastLow = std::max(medianMag * 0.6, params_.contrastHigh * 0.35);
                params_.contrastLow = std::clamp(params_.contrastLow, 3.0, params_.contrastHigh * 0.7);
            }
            else if (params_.contrastMode == ContrastMode::AutoMinSize) {
                double otsuThreshold = computeOtsu();
                params_.contrastHigh = std::clamp(otsuThreshold * 0.8, 5.0, maxMag * 0.9);
                params_.contrastLow = 0.0;
            }
        } else {
            params_.contrastHigh = 10.0;
            params_.contrastLow = 0.0;
        }
    }
    if (timingParams_.enableTiming) {
        createTiming_.contrastAutoMs = elapsedMs(tStep);
    }

    // Set origin
    origin_ = origin;
    if (origin_.x == 0 && origin_.y == 0) {
        origin_.x = templateSize_.width / 2.0;
        origin_.y = templateSize_.height / 2.0;
    }

    // Extract model points from pyramid
    tStep = std::chrono::high_resolution_clock::now();
    if (params_.optimization == OptimizationMode::XLDContour) {
        // Use XLD contour-based extraction (Halcon-style, ~20-50 key points)
        ExtractModelPointsXLD(templateImg, pyramid);
    } else {
        // Traditional edge pixel-based extraction
        ExtractModelPoints(pyramid);
    }
    if (timingParams_.enableTiming) {
        createTiming_.extractPointsMs = elapsedMs(tStep);
    }

    if (levels_.empty() || levels_[0].points.empty()) {
        return false;
    }

    // Apply optimization (point reduction) based on mode
    tStep = std::chrono::high_resolution_clock::now();
    if (params_.optimization != OptimizationMode::None) {
        OptimizeModel();
    }
    if (timingParams_.enableTiming) {
        createTiming_.optimizeMs = elapsedMs(tStep);
    }

    // Compute model bounding box for search constraints
    ComputeModelBounds();

    // Build SoA data for SIMD optimization
    tStep = std::chrono::high_resolution_clock::now();
    for (auto& level : levels_) {
        level.BuildSoA();
    }

    // Build cosine lookup table for direction-quantized scoring
    const int16_t* binData;
    int32_t w, h, s, numBins;
    if (pyramid.GetAngleBinData(0, binData, w, h, s, numBins)) {
        BuildCosLUT(numBins);
    } else {
        BuildCosLUT(64);
    }

    if (timingParams_.enableTiming) {
        createTiming_.buildSoAMs = elapsedMs(tStep);
        createTiming_.totalMs = elapsedMs(tTotal);

        if (timingParams_.printTiming) {
            createTiming_.Print();
        }
    }

    valid_ = true;
    return true;
}

// =============================================================================
// ShapeModelImpl::ExtractModelPoints (LEGACY)
// =============================================================================

void ShapeModelImpl::ExtractModelPoints(const AnglePyramid& pyramid) {
    levels_.clear();
    levels_.resize(pyramid.NumLevels());

    // Get contrast thresholds from parameters
    double contrastHigh = params_.contrastHigh;
    double contrastLow = (params_.contrastLow > 0) ? params_.contrastLow : contrastHigh;
    double contrastMax = params_.contrastMax;
    bool useHysteresis = (params_.contrastLow > 0 && params_.contrastLow < params_.contrastHigh);

    for (int32_t level = 0; level < pyramid.NumLevels(); ++level) {
        const auto& levelData = pyramid.GetLevel(level);
        auto& levelModel = levels_[level];

        levelModel.width = levelData.width;
        levelModel.height = levelData.height;
        levelModel.scale = levelData.scale;

        double levelOriginX = origin_.x * levelData.scale;
        double levelOriginY = origin_.y * levelData.scale;
        double levelScale = levelData.scale;

        double levelContrastHigh = contrastHigh * levelScale;
        double levelContrastLow = contrastLow * levelScale;
        double levelContrastMax = contrastMax * levelScale;

        levelContrastHigh = std::max(2.0, levelContrastHigh);
        levelContrastLow = std::max(1.0, levelContrastLow);

        const auto& edgePoints = pyramid.GetEdgePoints(level);

        std::vector<ModelPoint> allPoints;
        allPoints.reserve(edgePoints.size());

        if (useHysteresis) {
            // Hysteresis thresholding with BFS propagation
            const double gridSize = 1.5;
            const double gridSizeSq = gridSize * gridSize;

            std::vector<int32_t> strongIndices;
            std::vector<int32_t> weakIndices;
            std::vector<ModelPoint> allCandidates;

            for (size_t i = 0; i < edgePoints.size(); ++i) {
                const auto& ep = edgePoints[i];
                if (ep.magnitude > levelContrastMax) continue;

                double relX = ep.x - levelOriginX;
                double relY = ep.y - levelOriginY;

                if (ep.magnitude >= levelContrastHigh) {
                    strongIndices.push_back(static_cast<int32_t>(allCandidates.size()));
                    allCandidates.emplace_back(relX, relY, ep.angle, ep.magnitude, ep.angleBin, 1.0);
                } else if (ep.magnitude >= levelContrastLow) {
                    weakIndices.push_back(static_cast<int32_t>(allCandidates.size()));
                    allCandidates.emplace_back(relX, relY, ep.angle, ep.magnitude, ep.angleBin, 1.0);
                }
            }

            std::vector<int8_t> keepFlag(allCandidates.size(), 0);

            for (int32_t idx : strongIndices) {
                keepFlag[idx] = 1;
            }

            std::unordered_map<int64_t, std::vector<int32_t>> weakGrid;
            auto toGridKey = [gridSize](double x, double y) -> int64_t {
                int32_t gx = static_cast<int32_t>(std::floor(x / gridSize));
                int32_t gy = static_cast<int32_t>(std::floor(y / gridSize));
                return (static_cast<int64_t>(gx) << 32) | static_cast<uint32_t>(gy);
            };

            for (int32_t idx : weakIndices) {
                int64_t key = toGridKey(allCandidates[idx].x, allCandidates[idx].y);
                weakGrid[key].push_back(idx);
            }

            std::queue<int32_t> bfsQueue;
            for (int32_t idx : strongIndices) {
                bfsQueue.push(idx);
            }

            while (!bfsQueue.empty()) {
                int32_t currentIdx = bfsQueue.front();
                bfsQueue.pop();

                const auto& currentPt = allCandidates[currentIdx];
                int32_t gx = static_cast<int32_t>(std::floor(currentPt.x / gridSize));
                int32_t gy = static_cast<int32_t>(std::floor(currentPt.y / gridSize));

                for (int32_t dy = -1; dy <= 1; ++dy) {
                    for (int32_t dx = -1; dx <= 1; ++dx) {
                        int64_t neighborKey = (static_cast<int64_t>(gx + dx) << 32) |
                                               static_cast<uint32_t>(gy + dy);

                        auto it = weakGrid.find(neighborKey);
                        if (it == weakGrid.end()) continue;

                        for (int32_t weakIdx : it->second) {
                            if (keepFlag[weakIdx] != 0) continue;

                            double ddx = allCandidates[weakIdx].x - currentPt.x;
                            double ddy = allCandidates[weakIdx].y - currentPt.y;
                            if (ddx * ddx + ddy * ddy <= gridSizeSq) {
                                keepFlag[weakIdx] = 1;
                                bfsQueue.push(weakIdx);
                            }
                        }
                    }
                }
            }

            for (size_t i = 0; i < allCandidates.size(); ++i) {
                if (keepFlag[i] == 1) {
                    allPoints.push_back(allCandidates[i]);
                }
            }
        } else {
            for (const auto& ep : edgePoints) {
                if (ep.magnitude >= levelContrastHigh && ep.magnitude <= levelContrastMax) {
                    double relX = ep.x - levelOriginX;
                    double relY = ep.y - levelOriginY;
                    allPoints.emplace_back(relX, relY, ep.angle, ep.magnitude, ep.angleBin, 1.0);
                }
            }
        }

        // Determine maximum points based on optimization mode
        int32_t maxPoints;
        switch (params_.optimization) {
            case OptimizationMode::None:
                maxPoints = 100000;
                break;
            case OptimizationMode::PointReductionLow:
                maxPoints = (level == 0) ? 3000 : (level == 1) ? 600 : 200;
                break;
            case OptimizationMode::PointReductionMedium:
                maxPoints = (level == 0) ? 2000 : (level == 1) ? 400 : 150;
                break;
            case OptimizationMode::PointReductionHigh:
                maxPoints = (level == 0) ? 1000 : (level == 1) ? 200 : 80;
                break;
            case OptimizationMode::Auto:
            default:
                int32_t templateDim = std::max(templateSize_.width, templateSize_.height);
                if (templateDim <= 100) {
                    maxPoints = (level == 0) ? 500 : (level == 1) ? 150 : 50;
                } else if (templateDim <= 300) {
                    maxPoints = (level == 0) ? 1500 : (level == 1) ? 300 : 100;
                } else {
                    maxPoints = (level == 0) ? 2500 : (level == 1) ? 500 : 180;
                }
                break;
        }

        if (static_cast<int32_t>(allPoints.size()) > maxPoints) {
            std::sort(allPoints.begin(), allPoints.end(),
                [](const ModelPoint& a, const ModelPoint& b) {
                    return a.magnitude > b.magnitude;
                });
            allPoints.resize(maxPoints);
        }

        levelModel.points = allPoints;

        // Block 2: Integer grid sample points
        {
            std::set<std::pair<int32_t, int32_t>> uniqueGridCoords;
            std::vector<ModelPoint> gridPts;
            gridPts.reserve(allPoints.size() * 2);

            for (const auto& pt : allPoints) {
                int32_t gx = static_cast<int32_t>(std::round(pt.x));
                int32_t gy = static_cast<int32_t>(std::round(pt.y));

                auto key = std::make_pair(gx, gy);
                if (uniqueGridCoords.find(key) == uniqueGridCoords.end()) {
                    uniqueGridCoords.insert(key);
                    gridPts.emplace_back(static_cast<double>(gx), static_cast<double>(gy),
                                        pt.angle, pt.magnitude, pt.angleBin, pt.weight);
                }
            }

            std::sort(gridPts.begin(), gridPts.end(),
                [](const ModelPoint& a, const ModelPoint& b) {
                    if (static_cast<int32_t>(a.y) != static_cast<int32_t>(b.y))
                        return a.y < b.y;
                    return a.x < b.x;
                });

            levelModel.gridPoints = std::move(gridPts);
        }
    }
}

// =============================================================================
// ShapeModelImpl::ExtractModelPointsXLD (LEGACY)
// =============================================================================

void ShapeModelImpl::ExtractModelPointsXLD(const QImage& templateImg, const AnglePyramid& pyramid) {
    (void)templateImg;  // Not used - we use AnglePyramid's edge points directly

    levels_.clear();
    levels_.resize(pyramid.NumLevels());

    // Get contrast thresholds from parameters
    double contrastHigh = params_.contrastHigh;
    double contrastLow = (params_.contrastLow > 0) ? params_.contrastLow : contrastHigh;
    double contrastMax = params_.contrastMax;
    bool useHysteresis = (params_.contrastLow > 0 && params_.contrastLow < params_.contrastHigh);

    for (int32_t level = 0; level < pyramid.NumLevels(); ++level) {
        const auto& levelData = pyramid.GetLevel(level);
        auto& levelModel = levels_[level];

        levelModel.width = levelData.width;
        levelModel.height = levelData.height;
        levelModel.scale = levelData.scale;

        double levelOriginX = origin_.x * levelData.scale;
        double levelOriginY = origin_.y * levelData.scale;

        // Scale contrast thresholds with pyramid level
        double levelContrastHigh = contrastHigh * levelData.scale;
        double levelContrastLow = contrastLow * levelData.scale;
        double levelContrastMax = contrastMax * levelData.scale;

        levelContrastHigh = std::max(2.0, levelContrastHigh);
        levelContrastLow = std::max(1.0, levelContrastLow);

        const auto& edgePoints = pyramid.GetEdgePoints(level);

        std::vector<ModelPoint> allPoints;
        allPoints.reserve(edgePoints.size());

        if (useHysteresis) {
            // Hysteresis thresholding with BFS propagation (same as Auto mode)
            const double gridSize = 1.5;
            const double gridSizeSq = gridSize * gridSize;

            std::vector<int32_t> strongIndices;
            std::vector<int32_t> weakIndices;
            std::vector<ModelPoint> allCandidates;

            for (size_t i = 0; i < edgePoints.size(); ++i) {
                const auto& ep = edgePoints[i];
                if (ep.magnitude > levelContrastMax) continue;

                double relX = ep.x - levelOriginX;
                double relY = ep.y - levelOriginY;

                if (ep.magnitude >= levelContrastHigh) {
                    strongIndices.push_back(static_cast<int32_t>(allCandidates.size()));
                    allCandidates.emplace_back(relX, relY, ep.angle, ep.magnitude, ep.angleBin, 1.0);
                } else if (ep.magnitude >= levelContrastLow) {
                    weakIndices.push_back(static_cast<int32_t>(allCandidates.size()));
                    allCandidates.emplace_back(relX, relY, ep.angle, ep.magnitude, ep.angleBin, 1.0);
                }
            }

            std::vector<int8_t> keepFlag(allCandidates.size(), 0);

            for (int32_t idx : strongIndices) {
                keepFlag[idx] = 1;
            }

            // Spatial hash grid for fast neighbor lookup
            std::unordered_map<int64_t, std::vector<int32_t>> weakGrid;
            auto toGridKey = [gridSize](double x, double y) -> int64_t {
                int32_t gx = static_cast<int32_t>(std::floor(x / gridSize));
                int32_t gy = static_cast<int32_t>(std::floor(y / gridSize));
                return (static_cast<int64_t>(gx) << 32) | static_cast<uint32_t>(gy);
            };

            for (int32_t idx : weakIndices) {
                int64_t key = toGridKey(allCandidates[idx].x, allCandidates[idx].y);
                weakGrid[key].push_back(idx);
            }

            // BFS propagation from strong points
            std::queue<int32_t> bfsQueue;
            for (int32_t idx : strongIndices) {
                bfsQueue.push(idx);
            }

            while (!bfsQueue.empty()) {
                int32_t currentIdx = bfsQueue.front();
                bfsQueue.pop();

                const auto& currentPt = allCandidates[currentIdx];
                int32_t gx = static_cast<int32_t>(std::floor(currentPt.x / gridSize));
                int32_t gy = static_cast<int32_t>(std::floor(currentPt.y / gridSize));

                for (int32_t dy = -1; dy <= 1; ++dy) {
                    for (int32_t dx = -1; dx <= 1; ++dx) {
                        int64_t neighborKey = (static_cast<int64_t>(gx + dx) << 32) |
                                               static_cast<uint32_t>(gy + dy);

                        auto it = weakGrid.find(neighborKey);
                        if (it == weakGrid.end()) continue;

                        for (int32_t weakIdx : it->second) {
                            if (keepFlag[weakIdx] != 0) continue;

                            double ddx = allCandidates[weakIdx].x - currentPt.x;
                            double ddy = allCandidates[weakIdx].y - currentPt.y;
                            if (ddx * ddx + ddy * ddy <= gridSizeSq) {
                                keepFlag[weakIdx] = 1;
                                bfsQueue.push(weakIdx);
                            }
                        }
                    }
                }
            }

            for (size_t i = 0; i < allCandidates.size(); ++i) {
                if (keepFlag[i] == 1) {
                    allPoints.push_back(allCandidates[i]);
                }
            }
        } else {
            // Simple threshold - keep all points above contrast threshold
            for (const auto& ep : edgePoints) {
                if (ep.magnitude >= levelContrastHigh && ep.magnitude <= levelContrastMax) {
                    double relX = ep.x - levelOriginX;
                    double relY = ep.y - levelOriginY;
                    allPoints.emplace_back(relX, relY, ep.angle, ep.magnitude, ep.angleBin, 1.0);
                }
            }
        }

        // Apply maxPoints limit (same as Auto mode)
        int32_t maxPoints;
        int32_t templateDim = std::max(templateSize_.width, templateSize_.height);
        if (templateDim <= 100) {
            maxPoints = (level == 0) ? 500 : (level == 1) ? 150 : 50;
        } else if (templateDim <= 300) {
            maxPoints = (level == 0) ? 1500 : (level == 1) ? 300 : 100;
        } else {
            maxPoints = (level == 0) ? 2500 : (level == 1) ? 500 : 180;
        }

        if (static_cast<int32_t>(allPoints.size()) > maxPoints) {
            std::sort(allPoints.begin(), allPoints.end(),
                [](const ModelPoint& a, const ModelPoint& b) {
                    return a.magnitude > b.magnitude;
                });
            allPoints.resize(maxPoints);
        }

        levelModel.points = allPoints;

        // Generate grid points (unique integer coordinates)
        std::set<std::pair<int32_t, int32_t>> uniqueGridCoords;
        std::vector<ModelPoint> gridPts;
        gridPts.reserve(allPoints.size() * 2);

        for (const auto& pt : allPoints) {
            int32_t gx = static_cast<int32_t>(std::round(pt.x));
            int32_t gy = static_cast<int32_t>(std::round(pt.y));

            auto key = std::make_pair(gx, gy);
            if (uniqueGridCoords.find(key) == uniqueGridCoords.end()) {
                uniqueGridCoords.insert(key);
                gridPts.emplace_back(static_cast<double>(gx), static_cast<double>(gy),
                                    pt.angle, pt.magnitude, pt.angleBin, pt.weight);
            }
        }

        std::sort(gridPts.begin(), gridPts.end(),
            [](const ModelPoint& a, const ModelPoint& b) {
                if (static_cast<int32_t>(a.y) != static_cast<int32_t>(b.y))
                    return a.y < b.y;
                return a.x < b.x;
            });

        levelModel.gridPoints = std::move(gridPts);
    }
}

// =============================================================================
// ShapeModelImpl::OptimizeModel (LEGACY)
// =============================================================================

void ShapeModelImpl::OptimizeModel() {
    double minSpacing = 1.0;
    switch (params_.optimization) {
        case OptimizationMode::None:
            minSpacing = 0.0;
            break;
        case OptimizationMode::XLDContour:
            // XLDContour uses same spacing as Auto for consistent performance
            {
                int32_t templateDim = std::max(templateSize_.width, templateSize_.height);
                if (templateDim <= 100) {
                    minSpacing = 1.5;
                } else if (templateDim <= 300) {
                    minSpacing = 2.0;
                } else {
                    minSpacing = 2.5;
                }
            }
            break;
        case OptimizationMode::PointReductionLow:
            minSpacing = 1.5;
            break;
        case OptimizationMode::PointReductionMedium:
            minSpacing = 2.0;
            break;
        case OptimizationMode::PointReductionHigh:
            minSpacing = 3.0;
            break;
        case OptimizationMode::Auto:
        default:
            int32_t templateDim = std::max(templateSize_.width, templateSize_.height);
            if (templateDim <= 100) {
                minSpacing = 1.5;
            } else if (templateDim <= 300) {
                minSpacing = 2.0;
            } else {
                minSpacing = 2.5;
            }
            break;
    }

    if (minSpacing > 0.5) {
        double minDistSq = minSpacing * minSpacing;

        for (auto& level : levels_) {
            if (level.points.empty()) continue;

            std::vector<ModelPoint> filtered;
            filtered.reserve(level.points.size());

            std::sort(level.points.begin(), level.points.end(),
                [](const ModelPoint& a, const ModelPoint& b) {
                    return a.magnitude > b.magnitude;
                });

            for (const auto& pt : level.points) {
                bool tooClose = false;
                for (const auto& kept : filtered) {
                    double dx = pt.x - kept.x;
                    double dy = pt.y - kept.y;
                    if (dx * dx + dy * dy < minDistSq) {
                        tooClose = true;
                        break;
                    }
                }
                if (!tooClose) {
                    filtered.push_back(pt);
                }
            }

            level.points = std::move(filtered);
        }
    }

    // Normalize weights
    for (auto& level : levels_) {
        if (level.points.empty()) continue;

        double totalWeight = 0.0;
        for (const auto& pt : level.points) {
            totalWeight += pt.weight;
        }
        if (totalWeight > 0) {
            for (auto& pt : level.points) {
                pt.weight /= totalWeight;
            }
        }
    }
}

} // namespace Internal
} // namespace Qi::Vision::Matching

#endif  // ARCHIVED CODE
