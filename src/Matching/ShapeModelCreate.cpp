/**
 * @file ShapeModelCreate.cpp
 * @brief Model creation functions for ShapeModel
 *
 * Contains:
 * - CreateModel
 * - ExtractModelPoints
 * - OptimizeModel
 * - BuildCosLUT
 * - BuildAngleCache
 * - ComputeModelBounds
 * - ComputeRotatedBounds
 * - LevelModel methods
 */

#include "ShapeModelImpl.h"
#include <QiVision/Internal/Pyramid.h>

#include <algorithm>
#include <chrono>
#include <queue>
#include <unordered_map>

namespace Qi::Vision::Matching {
namespace Internal {

// =============================================================================
// LevelModel Implementation
// =============================================================================

void LevelModel::BuildSoA() {
    // Regenerate gridPoints from points if empty (e.g., after loading from file)
    if (gridPoints.empty() && !points.empty()) {
        RegenerateGridPoints();
    }

    // Build SoA for Block 1 (subpixel points)
    BuildSoAForPoints(points, soaX, soaY, soaCosAngle, soaSinAngle, soaWeight, soaAngleBin);

    // Build SoA for Block 2 (grid points)
    BuildSoAForPoints(gridPoints, gridSoaX, gridSoaY, gridSoaCosAngle, gridSoaSinAngle, gridSoaWeight, gridSoaAngleBin);
}

void LevelModel::RegenerateGridPoints() {
    std::set<std::pair<int32_t, int32_t>> uniqueGridCoords;
    gridPoints.clear();
    gridPoints.reserve(points.size());

    for (const auto& pt : points) {
        int32_t gx = static_cast<int32_t>(std::round(pt.x));
        int32_t gy = static_cast<int32_t>(std::round(pt.y));

        auto key = std::make_pair(gx, gy);
        if (uniqueGridCoords.find(key) == uniqueGridCoords.end()) {
            uniqueGridCoords.insert(key);
            gridPoints.emplace_back(static_cast<double>(gx), static_cast<double>(gy),
                                   pt.angle, pt.magnitude, pt.angleBin, pt.weight);
        }
    }

    // Sort by Y then X
    std::sort(gridPoints.begin(), gridPoints.end(),
        [](const ModelPoint& a, const ModelPoint& b) {
            if (static_cast<int32_t>(a.y) != static_cast<int32_t>(b.y))
                return a.y < b.y;
            return a.x < b.x;
        });
}

void LevelModel::BuildSoAForPoints(const std::vector<ModelPoint>& pts,
                                    std::vector<float>& x, std::vector<float>& y,
                                    std::vector<float>& cosA, std::vector<float>& sinA,
                                    std::vector<float>& w, std::vector<int16_t>& bins) {
    const size_t n = pts.size();
    const size_t paddedN = (n + 7) & ~7;  // Pad to multiple of 8 for AVX2

    x.resize(paddedN, 0.0f);
    y.resize(paddedN, 0.0f);
    cosA.resize(paddedN, 1.0f);
    sinA.resize(paddedN, 0.0f);
    w.resize(paddedN, 0.0f);
    bins.resize(paddedN, 0);

    for (size_t i = 0; i < n; ++i) {
        x[i] = static_cast<float>(pts[i].x);
        y[i] = static_cast<float>(pts[i].y);
        cosA[i] = static_cast<float>(pts[i].cosAngle);
        sinA[i] = static_cast<float>(pts[i].sinAngle);
        w[i] = static_cast<float>(pts[i].weight);
        bins[i] = static_cast<int16_t>(pts[i].angleBin);
    }
}

// =============================================================================
// ShapeModelImpl::CreateModel
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
    ExtractModelPoints(pyramid);
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
// ShapeModelImpl::ExtractModelPoints
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
// ShapeModelImpl::OptimizeModel
// =============================================================================

void ShapeModelImpl::OptimizeModel() {
    double minSpacing = 1.0;
    switch (params_.optimization) {
        case OptimizationMode::None:
            minSpacing = 0.0;
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

// =============================================================================
// ShapeModelImpl::BuildCosLUT
// =============================================================================

void ShapeModelImpl::BuildCosLUT(int32_t numBins) {
    numAngleBins_ = numBins;
    cosLUT_.resize(numBins);

    const double step = 2.0 * PI / numBins;
    for (int32_t i = 0; i < numBins; ++i) {
        cosLUT_[i] = static_cast<float>(std::fabs(std::cos(i * step)));
    }
}

// =============================================================================
// ShapeModelImpl::BuildAngleCache
// =============================================================================

void ShapeModelImpl::BuildAngleCache(double angleStart, double angleExtent, double angleStep) {
    angleCache_.clear();

    if (angleStep <= 0) {
        int32_t modelSize = std::max(templateSize_.width, templateSize_.height);
        angleStep = EstimateAngleStep(modelSize);
    }

    int32_t numAngles = static_cast<int32_t>(std::ceil(angleExtent / angleStep)) + 1;
    angleCache_.resize(levels_.size());

    for (size_t level = 0; level < levels_.size(); ++level) {
        angleCache_[level].resize(numAngles);

        for (int32_t i = 0; i < numAngles; ++i) {
            double angle = angleStart + i * angleStep;
            angleCache_[level][i].angle = angle;
            angleCache_[level][i].cosA = std::cos(angle);
            angleCache_[level][i].sinA = std::sin(angle);
        }
    }
}

// =============================================================================
// ShapeModelImpl::ComputeModelBounds
// =============================================================================

void ShapeModelImpl::ComputeModelBounds() {
    if (levels_.empty() || levels_[0].points.empty()) {
        modelMinX_ = modelMaxX_ = modelMinY_ = modelMaxY_ = 0;
        return;
    }

    modelMinX_ = modelMinY_ = std::numeric_limits<double>::max();
    modelMaxX_ = modelMaxY_ = std::numeric_limits<double>::lowest();

    for (const auto& pt : levels_[0].points) {
        modelMinX_ = std::min(modelMinX_, pt.x);
        modelMaxX_ = std::max(modelMaxX_, pt.x);
        modelMinY_ = std::min(modelMinY_, pt.y);
        modelMaxY_ = std::max(modelMaxY_, pt.y);
    }
}

// =============================================================================
// ShapeModelImpl::ComputeRotatedBounds
// =============================================================================

void ShapeModelImpl::ComputeRotatedBounds(const std::vector<ModelPoint>& points, double angle,
                                          double& minX, double& maxX, double& minY, double& maxY) {
    if (points.empty()) {
        minX = maxX = minY = maxY = 0;
        return;
    }

    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    minX = minY = std::numeric_limits<double>::max();
    maxX = maxY = std::numeric_limits<double>::lowest();

    for (const auto& pt : points) {
        double rx = cosA * pt.x - sinA * pt.y;
        double ry = sinA * pt.x + cosA * pt.y;
        minX = std::min(minX, rx);
        maxX = std::max(maxX, rx);
        minY = std::min(minY, ry);
        maxY = std::max(maxY, ry);
    }
}

} // namespace Internal
} // namespace Qi::Vision::Matching
