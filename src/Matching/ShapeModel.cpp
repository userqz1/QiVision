/**
 * @file ShapeModel.cpp
 * @brief Implementation of shape-based template matching
 *
 * Performance: 86-407ms for 360° search on 640×512 images
 *
 * @note For optimization attempts and results, see:
 *       docs/design/ShapeModel_Optimization_Notes.md
 */

#include <QiVision/Matching/ShapeModel.h>
#include <QiVision/Internal/AnglePyramid.h>
#include <QiVision/Internal/ResponseMap.h>
#include <QiVision/Internal/Pyramid.h>
#include <QiVision/Internal/Interpolate.h>
#include <QiVision/Platform/FileIO.h>
#include <QiVision/Core/Constants.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>

#ifdef _OPENMP
#include <omp.h>
#endif

// SIMD intrinsics
#if defined(__AVX2__)
#include <immintrin.h>
#define HAVE_AVX2 1
#elif defined(__SSE4_1__)
#include <smmintrin.h>
#define HAVE_SSE4 1
#endif

namespace Qi::Vision::Matching {

// Import Internal types
using Qi::Vision::Internal::AnglePyramid;
using Qi::Vision::Internal::AnglePyramidParams;
using Qi::Vision::Internal::PyramidLevelData;
using Qi::Vision::Internal::EdgePoint;
using Qi::Vision::Internal::ResponseMap;
using Qi::Vision::Internal::RotatedResponseModel;

// =============================================================================
// Implementation Class
// =============================================================================

namespace Internal {

/**
 * @brief Model data for a single pyramid level
 */
struct LevelModel {
    std::vector<ModelPoint> points;
    int32_t width = 0;
    int32_t height = 0;
    double scale = 1.0;

    // Structure of Arrays (SoA) for SIMD optimization
    // Aligned to 32 bytes for AVX2
    std::vector<float> soaX;        // x coordinates
    std::vector<float> soaY;        // y coordinates
    std::vector<float> soaCosAngle; // precomputed cos(angle)
    std::vector<float> soaSinAngle; // precomputed sin(angle)
    std::vector<float> soaWeight;   // weights

    void BuildSoA() {
        const size_t n = points.size();
        // Pad to multiple of 8 for AVX2
        const size_t paddedN = (n + 7) & ~7;

        soaX.resize(paddedN, 0.0f);
        soaY.resize(paddedN, 0.0f);
        soaCosAngle.resize(paddedN, 1.0f);
        soaSinAngle.resize(paddedN, 0.0f);
        soaWeight.resize(paddedN, 0.0f);

        for (size_t i = 0; i < n; ++i) {
            soaX[i] = static_cast<float>(points[i].x);
            soaY[i] = static_cast<float>(points[i].y);
            soaCosAngle[i] = static_cast<float>(points[i].cosAngle);
            soaSinAngle[i] = static_cast<float>(points[i].sinAngle);
            soaWeight[i] = static_cast<float>(points[i].weight);
        }
    }
};

/**
 * @brief Precomputed data for angle search
 */
struct AngleData {
    double angle = 0.0;
    double cosA = 1.0;
    double sinA = 0.0;
};

/**
 * @brief Precomputed rotated model point for fast score computation
 */
struct RotatedPoint {
    double offsetX;      // cosA * pt.x - sinA * pt.y
    double offsetY;      // sinA * pt.x + cosA * pt.y
    double expectedAngle; // pt.angle + angle
    double weight;
};

/**
 * @brief Fast cosine lookup table for angle difference computation
 * Optimized with O(1) angle normalization instead of while loops
 */
class FastCosTable {
public:
    static constexpr int TABLE_SIZE = 2048;
    static constexpr int TABLE_MASK = TABLE_SIZE - 1;
    static constexpr double TABLE_SCALE = TABLE_SIZE / (2.0 * PI);
    static constexpr double INV_2PI = 1.0 / (2.0 * PI);
    static constexpr double INV_PI = 1.0 / PI;

    FastCosTable() {
        for (int i = 0; i < TABLE_SIZE; ++i) {
            double angle = (i * 2.0 * PI) / TABLE_SIZE;
            table_[i] = static_cast<float>(std::cos(angle));
        }
    }

    // Fast cosine with angle in radians - O(1) normalization
    float FastCos(double angle) const {
        // Fast normalization to [0, 2*PI) using floor
        angle = angle - std::floor(angle * INV_2PI) * (2.0 * PI);
        int idx = static_cast<int>(angle * TABLE_SCALE) & TABLE_MASK;
        return table_[idx];
    }

    // Fast abs(cos) for symmetric similarity - O(1) normalization
    // cos(x) has period 2π, |cos(x)| has period π
    float FastAbsCos(double angleDiff) const {
        // Fast normalization: |cos(x)| = |cos(x mod π)|
        // Use fabs first, then normalize to [0, PI)
        angleDiff = std::fabs(angleDiff);
        angleDiff = angleDiff - std::floor(angleDiff * INV_PI) * PI;
        int idx = static_cast<int>(angleDiff * TABLE_SCALE) & TABLE_MASK;
        return std::fabs(table_[idx]);
    }

private:
    float table_[TABLE_SIZE];
};

// Global cosine lookup table
static const FastCosTable g_cosTable;

// =============================================================================
// Shape Model Implementation Class
// =============================================================================

class ShapeModelImpl {
public:
    // Model data
    std::vector<LevelModel> levels_;
    ModelParams params_;
    Point2d origin_;
    Size2i templateSize_;
    bool valid_ = false;

    // Timing configuration and results
    ShapeModelTimingParams timingParams_;
    ShapeModelCreateTiming createTiming_;
    mutable ShapeModelFindTiming findTiming_;  // mutable for const Find()

    // Model bounding box (cached)
    double modelMinX_ = 0, modelMaxX_ = 0;
    double modelMinY_ = 0, modelMaxY_ = 0;

    // Precomputed angle data for common angles
    std::vector<std::vector<AngleData>> angleCache_;

    // Methods
    bool CreateModel(const QImage& image, const Rect2i& roi, const Point2d& origin);
    void ExtractModelPoints(const AnglePyramid& pyramid);
    void OptimizeModel();
    void BuildAngleCache(double angleStart, double angleExtent, double angleStep);
    void ComputeModelBounds();

    std::vector<MatchResult> SearchPyramid(const AnglePyramid& targetPyramid,
                                            const SearchParams& params) const;

    // NEW: Fast search using Response Map
    std::vector<MatchResult> SearchPyramidWithResponseMap(
        const AnglePyramid& targetPyramid,
        const ResponseMap& responseMap,
        const SearchParams& params) const;

    std::vector<MatchResult> SearchLevel(const AnglePyramid& targetPyramid,
                                          int32_t level,
                                          const std::vector<MatchResult>& candidates,
                                          const SearchParams& params) const;

    double ComputeScoreAtPosition(const AnglePyramid& pyramid, int32_t level,
                                   double x, double y, double angle, double scale,
                                   double greediness, double* outCoverage = nullptr) const;

    // Fast scoring for coarse levels - uses direct array access, no bilinear interpolation
    double ComputeScoreAtPositionFast(const AnglePyramid& pyramid, int32_t level,
                                       int32_t x, int32_t y, double angle, double scale,
                                       double* outCoverage = nullptr) const;

    // Optimized bilinear with direct pointer access + periodic early termination
    double ComputeScoreBilinearScalar(const AnglePyramid& pyramid, int32_t level,
                                       double x, double y, double angle, double scale,
                                       double greediness, double* outCoverage = nullptr) const;

    // SSE optimized scalar: keeps finest-grained greediness, uses SSE for math (rsqrt)
    double ComputeScoreBilinearSSE(const AnglePyramid& pyramid, int32_t level,
                                    double x, double y, double angle, double scale,
                                    double greediness, double* outCoverage = nullptr) const;

    void RefinePosition(const AnglePyramid& pyramid, MatchResult& match,
                        SubpixelMethod method) const;

    // Helper to compute rotated bounds
    static void ComputeRotatedBounds(const std::vector<ModelPoint>& points, double angle,
                                     double& minX, double& maxX, double& minY, double& maxY);
};

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

    // Auto-detect contrast threshold if requested (Halcon: 'auto_contrast', 'auto_contrast_hyst')
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
            if (templateArea < 2500) {           // < 50x50
                targetPoints = std::min(300, static_cast<int32_t>(magnitudes.size() / 4));
            } else if (templateArea < 10000) {   // < 100x100
                targetPoints = std::min(800, static_cast<int32_t>(magnitudes.size() / 4));
            } else if (templateArea < 40000) {   // < 200x200
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
                // Method A: Percentile based on target points
                size_t targetIdx = (magnitudes.size() > static_cast<size_t>(targetPoints))
                    ? magnitudes.size() - targetPoints : 0;
                double percentileThreshold = magnitudes[targetIdx];

                // Method B: Otsu threshold
                double otsuThreshold = computeOtsu();

                // Combine: use weighted average favoring the more conservative threshold
                params_.contrastHigh = std::max(percentileThreshold, otsuThreshold) * 0.6 +
                                       std::min(percentileThreshold, otsuThreshold) * 0.4;

                // Boundary protection
                params_.contrastHigh = std::clamp(params_.contrastHigh, 5.0, maxMag * 0.9);
                params_.contrastLow = 0.0;  // No hysteresis for Auto mode
            }
            else if (params_.contrastMode == ContrastMode::AutoHysteresis) {
                // High threshold: based on Otsu or top percentile
                double otsuThreshold = computeOtsu();
                size_t highIdx = magnitudes.size() * 3 / 4;  // 75th percentile
                double percentileHigh = magnitudes[highIdx];

                params_.contrastHigh = std::max(otsuThreshold, percentileHigh);
                params_.contrastHigh = std::clamp(params_.contrastHigh, 8.0, maxMag * 0.85);

                // Low threshold: 30-50% of high, based on gradient distribution
                size_t lowIdx = magnitudes.size() / 2;  // 50th percentile (median)
                double medianMag = magnitudes[lowIdx];

                // Low = max(median * 0.6, high * 0.35)
                params_.contrastLow = std::max(medianMag * 0.6, params_.contrastHigh * 0.35);
                params_.contrastLow = std::clamp(params_.contrastLow, 3.0, params_.contrastHigh * 0.7);
            }
            else if (params_.contrastMode == ContrastMode::AutoMinSize) {
                // Use Otsu with adjustment for small components
                double otsuThreshold = computeOtsu();
                params_.contrastHigh = std::clamp(otsuThreshold * 0.8, 5.0, maxMag * 0.9);
                params_.contrastLow = 0.0;
            }
        } else {
            params_.contrastHigh = 10.0;  // Default fallback
            params_.contrastLow = 0.0;
        }
    }
    if (timingParams_.enableTiming) {
        createTiming_.contrastAutoMs = elapsedMs(tStep);
    }

    // Set origin
    origin_ = origin;
    if (origin_.x == 0 && origin_.y == 0) {
        // Default: center of template
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

        // Scale origin and thresholds to this level
        double levelOriginX = origin_.x * levelData.scale;
        double levelOriginY = origin_.y * levelData.scale;
        double levelScale = levelData.scale;  // e.g., 0.5 for level 1, 0.25 for level 2

        // Scale contrast thresholds for pyramid level (gradients are weaker at coarser levels)
        // Gradient magnitude scales with image resolution
        double levelContrastHigh = contrastHigh * levelScale;
        double levelContrastLow = contrastLow * levelScale;
        double levelContrastMax = contrastMax * levelScale;

        // Ensure minimum thresholds
        levelContrastHigh = std::max(2.0, levelContrastHigh);
        levelContrastLow = std::max(1.0, levelContrastLow);

        // Extract edge points with sufficient contrast
        const auto& edgePoints = pyramid.GetEdgePoints(level);

        // Collect all valid points
        std::vector<ModelPoint> allPoints;
        allPoints.reserve(edgePoints.size());

        if (useHysteresis) {
            // Hysteresis thresholding with BFS propagation (like Canny):
            // 1. Classify points into strong/weak/discard
            // 2. BFS from strong points to connect adjacent weak points
            // 3. Connected weak points are kept, isolated weak points are discarded

            const double gridSize = 1.5;  // Connectivity radius
            const double gridSizeSq = gridSize * gridSize;

            // Classify edge points
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
                // magnitude < levelContrastLow: discard
            }

            // Keep flag: 0=unvisited, 1=keep
            std::vector<int8_t> keepFlag(allCandidates.size(), 0);

            // All strong points are kept
            for (int32_t idx : strongIndices) {
                keepFlag[idx] = 1;
            }

            // Build spatial hash grid for weak points (accelerate neighbor search)
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

                // Check 3x3 neighboring grid cells
                for (int32_t dy = -1; dy <= 1; ++dy) {
                    for (int32_t dx = -1; dx <= 1; ++dx) {
                        int64_t neighborKey = (static_cast<int64_t>(gx + dx) << 32) |
                                               static_cast<uint32_t>(gy + dy);

                        auto it = weakGrid.find(neighborKey);
                        if (it == weakGrid.end()) continue;

                        for (int32_t weakIdx : it->second) {
                            if (keepFlag[weakIdx] != 0) continue;  // Already processed

                            // Check distance
                            double ddx = allCandidates[weakIdx].x - currentPt.x;
                            double ddy = allCandidates[weakIdx].y - currentPt.y;
                            if (ddx * ddx + ddy * ddy <= gridSizeSq) {
                                keepFlag[weakIdx] = 1;  // Keep this weak point
                                bfsQueue.push(weakIdx);  // Continue propagation
                            }
                        }
                    }
                }
            }

            // Collect all kept points
            for (size_t i = 0; i < allCandidates.size(); ++i) {
                if (keepFlag[i] == 1) {
                    allPoints.push_back(allCandidates[i]);
                }
            }
        } else {
            // Single threshold: simple magnitude check
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
                maxPoints = 100000;  // Essentially unlimited
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
                // Auto: based on template size
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
            // Sort by magnitude (keep strongest)
            std::sort(allPoints.begin(), allPoints.end(),
                [](const ModelPoint& a, const ModelPoint& b) {
                    return a.magnitude > b.magnitude;
                });
            allPoints.resize(maxPoints);
        }

        levelModel.points = std::move(allPoints);
    }
}

void ShapeModelImpl::OptimizeModel() {
    // Determine minimum point spacing based on optimization mode
    double minSpacing = 1.0;
    switch (params_.optimization) {
        case OptimizationMode::None:
            minSpacing = 0.0;  // No spacing filter
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
            // Auto spacing based on template size
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

    // Apply minimum spacing filter
    if (minSpacing > 0.5) {
        double minDistSq = minSpacing * minSpacing;

        for (auto& level : levels_) {
            if (level.points.empty()) continue;

            std::vector<ModelPoint> filtered;
            filtered.reserve(level.points.size());

            // Sort by magnitude first (keep strongest)
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

void ShapeModelImpl::BuildAngleCache(double angleStart, double angleExtent, double angleStep) {
    angleCache_.clear();

    if (angleStep <= 0) {
        // Auto-compute angle step based on model size
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

    // =========================================================================
    // TWO-PHASE ANGLE SEARCH STRATEGY
    // =========================================================================
    // Problem: Fine angle step (0.6°) leads to 720 angles for 360° search → SLOW
    // Solution: Coarse angle search (6°) → 60 angles, then refine best candidates
    //
    // Phase 1 (Coarse): Search with 6° step at top pyramid level
    // Phase 2 (Fine):   Refine each candidate ±6° with 0.5° step in SearchLevel
    // =========================================================================

    // Fine angle step for final refinement (used in SearchLevel)
    double fineAngleStep = params.angleStep;
    if (fineAngleStep <= 0) {
        fineAngleStep = EstimateAngleStep(std::max(templateSize_.width, templateSize_.height));
    }

    // COARSE angle step for top-level search: 6° = ~0.1 radians
    // This reduces angles from 720 to 60 for 360° search (12× speedup!)
    // Note: 12° step missed matches - 6° is the safe limit
    constexpr double COARSE_ANGLE_STEP = 0.1;  // ~6 degrees
    double coarseAngleStep = std::max(fineAngleStep, COARSE_ANGLE_STEP);

    // Coarse search at top level
    const auto& topLevel = levels_[startLevel];
    int32_t targetWidth = targetPyramid.GetWidth(startLevel);
    int32_t targetHeight = targetPyramid.GetHeight(startLevel);

    // Search grid at top level
    // Step 2: ~1280 positions for 80×64 image
    // Note: Step 3 causes misses - search radius can't compensate
    int32_t stepSize = 2;

    // Build COARSE angle list for parallel processing
    std::vector<double> angles;
    for (double angle = params.angleStart;
         angle <= params.angleStart + params.angleExtent;
         angle += coarseAngleStep) {
        angles.push_back(angle);
    }

    // OpenMP parallelization over angles (coarse-grained parallelism)
    #pragma omp parallel
    {
        std::vector<MatchResult> localCandidates;

        #pragma omp for schedule(dynamic)
        for (size_t ai = 0; ai < angles.size(); ++ai) {
            const double angle = angles[ai];

            // Compute rotated model bounds
            double rMinX, rMaxX, rMinY, rMaxY;
            ComputeRotatedBounds(topLevel.points, angle, rMinX, rMaxX, rMinY, rMaxY);

            // Valid search region: ensure all model points stay inside image
            int32_t searchXMin = static_cast<int32_t>(std::ceil(-rMinX));
            int32_t searchXMax = static_cast<int32_t>(std::floor(targetWidth - 1 - rMaxX));
            int32_t searchYMin = static_cast<int32_t>(std::ceil(-rMinY));
            int32_t searchYMax = static_cast<int32_t>(std::floor(targetHeight - 1 - rMaxY));

            // Clamp to valid range
            searchXMin = std::max(0, searchXMin);
            searchYMin = std::max(0, searchYMin);
            searchXMax = std::min(targetWidth - 1, searchXMax);
            searchYMax = std::min(targetHeight - 1, searchYMax);

            // Skip if no valid search region
            if (searchXMin > searchXMax || searchYMin > searchYMax) {
                continue;
            }

            for (int32_t y = searchYMin; y <= searchYMax; y += stepSize) {
                for (int32_t x = searchXMin; x <= searchXMax; x += stepSize) {
                    double coverage = 0.0;
                    // Use SSE-optimized scoring with early termination
                    double score = ComputeScoreAtPosition(targetPyramid, startLevel,
                                                           x, y, angle, 1.0, params.greediness, &coverage);

                    // Require good coverage (at least 70% of model points visible)
                    // Use 0.7 * minScore as coarse threshold (was 0.5, too many false candidates)
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

        // Merge local results into global candidates
        #pragma omp critical
        {
            candidates.insert(candidates.end(), localCandidates.begin(), localCandidates.end());
        }
    }

    // Sort candidates by score
    std::sort(candidates.begin(), candidates.end());

    t1 = std::chrono::high_resolution_clock::now();
    size_t coarseCandidates = candidates.size();

    // Record coarse search timing
    if (timingParams_.enableTiming) {
        findTiming_.coarseSearchMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
        findTiming_.numCoarseCandidates = static_cast<int32_t>(coarseCandidates);
    }

    // Limit candidates for efficiency (tighter limit for speed)
    if (candidates.size() > 200) {
        candidates.resize(200);
    }

    // Refine through pyramid levels
    for (int32_t level = startLevel - 1; level >= 0; --level) {
        candidates = SearchLevel(targetPyramid, level, candidates, params);

        // Re-sort and limit
        std::sort(candidates.begin(), candidates.end());
        size_t limit = (level == 0) ? 50 : 100;
        if (candidates.size() > limit) {
            candidates.resize(limit);
        }
    }

    t2 = std::chrono::high_resolution_clock::now();

    // Record pyramid refinement timing
    if (timingParams_.enableTiming) {
        findTiming_.pyramidRefineMs = std::chrono::duration<double, std::milli>(t2 - t1).count();
    }

    // Final refinement at level 0
    for (auto& match : candidates) {
        if (params.subpixelMethod != SubpixelMethod::None) {
            RefinePosition(targetPyramid, match, params.subpixelMethod);
        }

        // Scale coordinates back to original image
        double scale = levels_[0].scale;
        if (scale != 1.0) {
            match.x /= scale;
            match.y /= scale;
        }
    }

    t3 = std::chrono::high_resolution_clock::now();

    // Record subpixel refinement timing
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

    // Record NMS timing (for struct-based timing, stderr print is kept for backward compatibility)
    if (timingParams_.enableTiming) {
        findTiming_.nmsMs = std::chrono::duration<double, std::milli>(t4 - t3).count();
    }

    // Print timing breakdown (keep for backward compatibility, will be suppressed if timing struct is used)
    auto ms = [](auto start, auto end) {
        return std::chrono::duration<double, std::milli>(end - start).count();
    };
    if (!timingParams_.enableTiming) {
        // Only print to stderr if struct-based timing is not enabled
        fprintf(stderr, "[Timing] Coarse: %.1fms (%zu candidates), Refine: %.1fms, SubPix: %.1fms, NMS: %.1fms | Total: %.1fms\n",
                ms(t0, t1), coarseCandidates, ms(t1, t2), ms(t2, t3), ms(t3, t4), ms(t0, t4));
    }

    return results;
}

// =============================================================================
// Response Map Based Fast Search
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

    // Build angle search list
    double angleStep = params.angleStep;
    if (angleStep <= 0) {
        angleStep = EstimateAngleStep(std::max(templateSize_.width, templateSize_.height));
    }

    // Precompute rotated models for all angles
    const auto& topLevelModel = levels_[startLevel];
    auto rotatedModels = ResponseMap::PrepareAllRotations(
        topLevelModel.points, params.angleStart, params.angleExtent, angleStep);

#ifdef QIVISION_DEBUG
    fprintf(stderr, "[SearchRM] startLevel=%d, model levels=%zu, model points=%zu\n",
            startLevel, levels_.size(), topLevelModel.points.size());
    fprintf(stderr, "[SearchRM] angleStart=%.3f, angleExtent=%.3f, angleStep=%.4f, numAngles=%zu\n",
            params.angleStart, params.angleExtent, angleStep, rotatedModels.size());
#endif

    // Coarse search at top level using Response Map with O(1) lookups
    std::vector<MatchResult> candidates;
    int32_t targetWidth = responseMap.GetWidth(startLevel);
    int32_t targetHeight = responseMap.GetHeight(startLevel);

    // Search grid (every 2 pixels at top level)
    int32_t stepSize = 2;

    // OpenMP parallelization over angles (same as original SearchPyramid)
    #pragma omp parallel
    {
        std::vector<MatchResult> localCandidates;

        #pragma omp for schedule(dynamic)
        for (size_t angleIdx = 0; angleIdx < rotatedModels.size(); ++angleIdx) {
            const auto& rotatedModel = rotatedModels[angleIdx];

            // Valid search region based on bounding box
            int32_t searchXMin = std::max(0, -static_cast<int32_t>(rotatedModel.minX));
            int32_t searchXMax = std::min(targetWidth - 1,
                                           targetWidth - 1 - static_cast<int32_t>(rotatedModel.maxX));
            int32_t searchYMin = std::max(0, -static_cast<int32_t>(rotatedModel.minY));
            int32_t searchYMax = std::min(targetHeight - 1,
                                           targetHeight - 1 - static_cast<int32_t>(rotatedModel.maxY));

            // Skip if no valid search region
            if (searchXMin > searchXMax || searchYMin > searchYMax) {
                continue;
            }

            for (int32_t y = searchYMin; y <= searchYMax; y += stepSize) {
                for (int32_t x = searchXMin; x <= searchXMax; x += stepSize) {
                    double coverage = 0.0;
                    double score = responseMap.ComputeScore(rotatedModel, startLevel,
                                                            x, y, &coverage);

                    // Use same thresholds as original SearchPyramid
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

        // Merge local results into global candidates
        #pragma omp critical
        {
            candidates.insert(candidates.end(), localCandidates.begin(), localCandidates.end());
        }
    }

    // Sort candidates by score
    std::sort(candidates.begin(), candidates.end());

    // Limit candidates for efficiency (same as original)
    if (candidates.size() > 1000) {
        candidates.resize(1000);
    }

#ifdef QIVISION_DEBUG
    if (!candidates.empty()) {
        fprintf(stderr, "[SearchRM] Best candidate: score=%.4f at (%.1f, %.1f) angle=%.3f\n",
                candidates[0].score, candidates[0].x, candidates[0].y, candidates[0].angle);
    }
#endif

    // Refine through pyramid levels using PRECISE scoring (bilinear interpolation)
    // Response Map is O(1) but lacks subpixel precision - use only for coarse search
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

            // Local search around candidate using PRECISE bilinear interpolation
            for (int32_t dy = -searchRadius; dy <= searchRadius; ++dy) {
                for (int32_t dx = -searchRadius; dx <= searchRadius; ++dx) {
                    double px = baseX + dx;
                    double py = baseY + dy;

                    if (px < 0 || px >= levelWidth || py < 0 || py >= levelHeight) {
                        continue;
                    }

                    // Angle refinement
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

        // Re-sort and limit
        std::sort(candidates.begin(), candidates.end());
        if (candidates.size() > 500) {
            candidates.resize(500);
        }
    }

    // Final precise refinement at level 0
    for (auto& match : candidates) {
        // Re-score with precise method
        double coverage = 0.0;
        match.score = ComputeScoreAtPosition(targetPyramid, 0,
                                              match.x, match.y, match.angle, 1.0,
                                              0.0, &coverage);

        // Subpixel refinement
        if (params.subpixelMethod != SubpixelMethod::None) {
            RefinePosition(targetPyramid, match, params.subpixelMethod);
        }

        // Scale coordinates back to original image
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

    // Apply non-maximum suppression
    results = NonMaxSuppression(results, 10.0);

    // Limit results
    if (params.maxMatches > 0 && static_cast<int32_t>(results.size()) > params.maxMatches) {
        results.resize(params.maxMatches);
    }

    return results;
}

std::vector<MatchResult> ShapeModelImpl::SearchLevel(
    const AnglePyramid& targetPyramid,
    int32_t level,
    const std::vector<MatchResult>& candidates,
    const SearchParams& params) const
{
    std::vector<MatchResult> refined;
    refined.reserve(candidates.size());

    // Scale factor between levels
    double scaleFactor = 2.0;

    // Search window at this level (smaller at higher levels for efficiency)
    int32_t searchRadius = (level == 0) ? 2 : 1;

    // Angle refinement strategy:
    // - Higher levels (coarse): range ±6°, step 2°
    // - Level 0 (fine): range ±2°, step 0.5°
    double angleRadius = (level == 0) ? 0.035 : 0.1;  // ±2° at level 0, ±6° at higher levels
    double angleStep = (level == 0) ? 0.01 : 0.035;   // 0.5° at level 0, 2° at higher levels

    int32_t targetWidth = targetPyramid.GetWidth(level);
    int32_t targetHeight = targetPyramid.GetHeight(level);

    // Simple scalar loop - no Pipe8
    // Pipe8 was slower for refinement due to:
    // 1. Spatial waste: computing 8 positions when only 5 needed (37.5% waste)
    // 2. Lane dragging: good lanes prevent early exit for bad lanes
    // SSE scalar version maintains finest-grained greediness for best performance
    for (const auto& candidate : candidates) {
        double baseX = candidate.x * scaleFactor;
        double baseY = candidate.y * scaleFactor;
        double baseAngle = candidate.angle;

        MatchResult bestMatch;
        bestMatch.score = -1.0;
        bestMatch.x = baseX;
        bestMatch.y = baseY;
        bestMatch.pyramidLevel = level;

        // Local search around candidate
        for (int32_t dy = -searchRadius; dy <= searchRadius; ++dy) {
            for (int32_t dx = -searchRadius; dx <= searchRadius; ++dx) {
                double x = baseX + dx;
                double y = baseY + dy;

                // Skip if outside image bounds
                if (x < 0 || x >= targetWidth || y < 0 || y >= targetHeight) {
                    continue;
                }

                // Angle refinement with level-dependent step
                for (double dAngle = -angleRadius; dAngle <= angleRadius; dAngle += angleStep) {
                    double angle = baseAngle + dAngle;

                    double coverage = 0.0;
                    // ComputeScoreAtPosition now dispatches to SSE optimized scalar
                    double score = ComputeScoreAtPosition(targetPyramid, level,
                                                           x, y, angle, 1.0, params.greediness, &coverage);

                    // Require good coverage
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

double ShapeModelImpl::ComputeScoreAtPosition(
    const AnglePyramid& pyramid, int32_t level,
    double x, double y, double angle, double scale,
    double greediness, double* outCoverage) const
{
    // Fast path: use SSE optimized scalar version for common cases
    // SSE scalar maintains finest-grained greediness (critical for performance!)
    // while using SSE for fast math (rsqrt instead of sqrt)
    if (params_.metric == MetricMode::IgnoreLocalPolarity ||
        params_.metric == MetricMode::IgnoreColorPolarity) {
        return ComputeScoreBilinearSSE(pyramid, level, x, y, angle, scale, greediness, outCoverage);
    }

    // Slow path: handle UsePolarity and IgnoreGlobalPolarity with original logic
    if (level < 0 || level >= static_cast<int32_t>(levels_.size())) {
        if (outCoverage) *outCoverage = 0.0;
        return 0.0;
    }

    const auto& levelModel = levels_[level];
    if (levelModel.points.empty()) {
        if (outCoverage) *outCoverage = 0.0;
        return 0.0;
    }

    // Get raw pointers for direct access
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

    const float* soaX = levelModel.soaX.data();
    const float* soaY = levelModel.soaY.data();
    const float* soaCos = levelModel.soaCosAngle.data();
    const float* soaSin = levelModel.soaSinAngle.data();
    const float* soaWeight = levelModel.soaWeight.data();

    float totalScore = 0.0f;
    float totalWeight = 0.0f;
    int32_t matchedCount = 0;
    const size_t numPoints = levelModel.points.size();

    // For IgnoreGlobalPolarity
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
                } else { // IgnoreGlobalPolarity
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

        // Periodic early termination
        if (greediness > 0.0f && ((i + 1) & 15) == 0 && totalWeight > 0) {
            float currentAvg = totalScore / totalWeight;
            if (currentAvg * numPoints < earlyTermThreshold) {
                if (outCoverage) *outCoverage = static_cast<double>(matchedCount) / numPoints;
                return 0.0;
            }
        }
    }

    // For IgnoreGlobalPolarity, select the better polarity
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
// ComputeScoreBilinearSSE: SSE optimized scalar version
// =============================================================================
// Strategy: Keep scalar loop structure (finest-grained greediness), but use
// SSE instructions to accelerate core math (especially rsqrt instead of sqrt)
//
// Advantages:
// - Retains single-position early termination (most important for performance!)
// - Uses _mm_rsqrt_ss for fast 1/sqrt approximation
// - Avoids expensive float/int conversions
// - No wasted computation on unneeded positions
// =============================================================================
double ShapeModelImpl::ComputeScoreBilinearSSE(
    const AnglePyramid& pyramid, int32_t level,
    double x, double y, double angle, double scale,
    double greediness, double* outCoverage) const
{
    const auto& levelModel = levels_[level];
    const size_t numPoints = levelModel.points.size();
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

    // Precompute rotation parameters
    const float cosR = static_cast<float>(std::cos(angle));
    const float sinR = static_cast<float>(std::sin(angle));
    const float scalef = static_cast<float>(scale);
    const float xf = static_cast<float>(x);
    const float yf = static_cast<float>(y);

    // SSE constants
    const __m128 vMinMagSq = _mm_set_ss(25.0f);
    const __m128 vSignMask = _mm_set_ss(-0.0f); // For abs()

    float totalScore = 0.0f;
    float totalWeight = 0.0f;
    int32_t matchedCount = 0;

    // Greediness parameters
    const float earlyTermThreshold = static_cast<float>(greediness * numPoints);
    const int32_t checkInterval = 8; // Check more frequently than AVX2 version

    // SoA data pointers
    const float* soaX = levelModel.soaX.data();
    const float* soaY = levelModel.soaY.data();
    const float* soaCos = levelModel.soaCosAngle.data();
    const float* soaSin = levelModel.soaSinAngle.data();
    const float* soaWeight = levelModel.soaWeight.data();

    const int32_t maxX = width - 2;
    const int32_t maxY = height - 2;

    for (size_t i = 0; i < numPoints; ++i) {
        // --- Coordinate transformation (Scalar) ---
        float rotX = cosR * soaX[i] - sinR * soaY[i];
        float rotY = sinR * soaX[i] + cosR * soaY[i];
        float imgX = xf + scalef * rotX;
        float imgY = yf + scalef * rotY;

        // Fast floor (handle negative values correctly)
        int32_t ix = static_cast<int32_t>(imgX);
        int32_t iy = static_cast<int32_t>(imgY);
        if (imgX < 0) ix--;
        if (imgY < 0) iy--;

        if (ix >= 0 && ix <= maxX && iy >= 0 && iy <= maxY) {
            float fx = imgX - ix;
            float fy = imgY - iy;

            // --- Bilinear interpolation (Scalar optimized) ---
            float w00 = (1.0f - fx) * (1.0f - fy);
            float w10 = fx * (1.0f - fy);
            float w01 = (1.0f - fx) * fy;
            float w11 = fx * fy;

            int32_t idx = iy * stride + ix;

            // Unroll to reduce index calculations
            float g00 = gxData[idx];     float g10 = gxData[idx + 1];
            float g01 = gxData[idx + stride]; float g11 = gxData[idx + stride + 1];
            float gx = w00 * g00 + w10 * g10 + w01 * g01 + w11 * g11;

            float h00 = gyData[idx];     float h10 = gyData[idx + 1];
            float h01 = gyData[idx + stride]; float h11 = gyData[idx + stride + 1];
            float gy = w00 * h00 + w10 * h10 + w01 * h01 + w11 * h11;

            // --- Similarity computation (SSE Scalar) ---
            // Use SSE scalar instructions (_mm_..._ss) for faster rsqrt
            __m128 vGx = _mm_set_ss(gx);
            __m128 vGy = _mm_set_ss(gy);
            __m128 vMagSq = _mm_add_ss(_mm_mul_ss(vGx, vGx), _mm_mul_ss(vGy, vGy));

            // if (magSq >= 25.0f)
            if (_mm_comige_ss(vMagSq, vMinMagSq)) {
                float rotCos = soaCos[i] * cosR - soaSin[i] * sinR;
                float rotSin = soaSin[i] * cosR + soaCos[i] * sinR;

                // dot = rotCos * gx + rotSin * gy
                float dot = rotCos * gx + rotSin * gy;

                // rsqrtss (approximate 1/sqrt, precision sufficient for scoring)
                __m128 vInvMag = _mm_rsqrt_ss(vMagSq);

                // score = abs(dot) * invMag
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

        // --- High-frequency greediness check ---
        // Scalar code has no SIMD overhead, can check more frequently
        if (greediness > 0.0 && ((i + 1) & (checkInterval - 1)) == 0) {
            if (totalWeight > 0) {
                // Simple average score prediction
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

// NOTE: AVX2 optimization functions removed
// Using SSE version (ComputeScoreBilinearSSE) which provides good performance
// across all x86-64 platforms. AVX2 versions showed minimal improvement
// due to memory-bound nature of the algorithm.

void ShapeModelImpl::RefinePosition(
    const AnglePyramid& pyramid, MatchResult& match,
    SubpixelMethod method) const
{
    if (method == SubpixelMethod::None) {
        return;
    }

    int32_t level = 0;  // Finest level

    if (method == SubpixelMethod::Parabolic) {
        // Parabolic fitting in 3x3 neighborhood
        double scores[3][3];

        for (int32_t dy = -1; dy <= 1; ++dy) {
            for (int32_t dx = -1; dx <= 1; ++dx) {
                scores[dy + 1][dx + 1] = ComputeScoreAtPosition(
                    pyramid, level, match.x + dx * 0.5, match.y + dy * 0.5,
                    match.angle, 1.0, 0.0);
            }
        }

        // Parabolic interpolation in X
        double denom = 2.0 * (scores[1][0] - 2.0 * scores[1][1] + scores[1][2]);
        if (std::abs(denom) > 1e-10) {
            double dx = (scores[1][0] - scores[1][2]) / denom;
            match.x += dx * 0.5;
        }

        // Parabolic interpolation in Y
        denom = 2.0 * (scores[0][1] - 2.0 * scores[1][1] + scores[2][1]);
        if (std::abs(denom) > 1e-10) {
            double dy = (scores[0][1] - scores[2][1]) / denom;
            match.y += dy * 0.5;
        }

        // Recompute score
        match.score = ComputeScoreAtPosition(pyramid, level, match.x, match.y,
                                              match.angle, 1.0, 0.0);
    }
    else if (method == SubpixelMethod::LeastSquares) {
        // Gradient descent optimization
        const int32_t maxIter = 10;
        const double stepSize = 0.1;
        const double tolerance = 0.001;

        double bestScore = match.score;
        double bestX = match.x;
        double bestY = match.y;
        double bestAngle = match.angle;

        for (int32_t iter = 0; iter < maxIter; ++iter) {
            // Compute gradient numerically
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

            // Update position
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

// =============================================================================
// ShapeModel Implementation
// =============================================================================

ShapeModel::ShapeModel() : impl_(std::make_unique<Internal::ShapeModelImpl>()) {}

ShapeModel::~ShapeModel() = default;

ShapeModel::ShapeModel(const ShapeModel& other)
    : impl_(std::make_unique<Internal::ShapeModelImpl>(*other.impl_)) {}

ShapeModel::ShapeModel(ShapeModel&& other) noexcept = default;

ShapeModel& ShapeModel::operator=(const ShapeModel& other) {
    if (this != &other) {
        impl_ = std::make_unique<Internal::ShapeModelImpl>(*other.impl_);
    }
    return *this;
}

ShapeModel& ShapeModel::operator=(ShapeModel&& other) noexcept = default;

bool ShapeModel::Create(const QImage& templateImage, const ModelParams& params) {
    impl_->params_ = params;
    return impl_->CreateModel(templateImage, Rect2i{}, Point2d{0, 0});
}

bool ShapeModel::Create(const QImage& templateImage, const Rect2i& roi,
                         const ModelParams& params) {
    impl_->params_ = params;
    return impl_->CreateModel(templateImage, roi, Point2d{0, 0});
}

bool ShapeModel::CreateWithOrigin(const QImage& templateImage, const Point2d& origin,
                                   const ModelParams& params) {
    impl_->params_ = params;
    return impl_->CreateModel(templateImage, Rect2i{}, origin);
}

void ShapeModel::Clear() {
    impl_->levels_.clear();
    impl_->valid_ = false;
}

bool ShapeModel::IsValid() const {
    return impl_->valid_;
}

std::vector<MatchResult> ShapeModel::Find(const QImage& image,
                                           const SearchParams& params) const {
    if (!impl_->valid_) {
        return {};
    }

    // Reset timing
    impl_->findTiming_ = ShapeModelFindTiming();
    auto tTotal = std::chrono::high_resolution_clock::now();

    // Build angle pyramid for target image
    AnglePyramidParams pyramidParams;
    pyramidParams.numLevels = static_cast<int32_t>(impl_->levels_.size());

    // Use minContrast if specified, otherwise use 50% of contrastHigh
    double searchContrast = (impl_->params_.minContrast > 0)
        ? impl_->params_.minContrast
        : impl_->params_.contrastHigh * 0.5;
    pyramidParams.minContrast = searchContrast;
    pyramidParams.smoothSigma = 0.5;
    pyramidParams.extractEdgePoints = false;  // Not needed for search pyramid

    auto tPyramid = std::chrono::high_resolution_clock::now();
    AnglePyramid targetPyramid;
    if (!targetPyramid.Build(image, pyramidParams)) {
        return {};
    }

    if (impl_->timingParams_.enableTiming) {
        impl_->findTiming_.pyramidBuildMs = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - tPyramid).count();
    }

    // Use traditional pyramid search with bilinear interpolation
    // Response Map was tested but adds overhead without improving performance
    // for this image size range (640x512) due to ResponseMap build cost
    auto results = impl_->SearchPyramid(targetPyramid, params);

    // Finalize total timing
    if (impl_->timingParams_.enableTiming) {
        impl_->findTiming_.totalMs = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - tTotal).count();
        impl_->findTiming_.numFinalMatches = static_cast<int32_t>(results.size());

        if (impl_->timingParams_.printTiming) {
            impl_->findTiming_.Print();
        }
    }

    return results;
}

MatchResult ShapeModel::FindBest(const QImage& image, const SearchParams& params) const {
    auto results = Find(image, params);
    if (results.empty()) {
        return MatchResult{};
    }
    return results[0];
}

std::vector<MatchResult> ShapeModel::FindInROI(const QImage& image, const Rect2i& roi,
                                                const SearchParams& params) const {
    SearchParams modifiedParams = params;
    modifiedParams.searchROI = roi;
    return Find(image, modifiedParams);
}

ModelStats ShapeModel::GetStats() const {
    ModelStats stats;

    if (!impl_->valid_ || impl_->levels_.empty()) {
        return stats;
    }

    stats.numLevels = static_cast<int32_t>(impl_->levels_.size());
    stats.pointsPerLevel.resize(stats.numLevels);

    stats.minX = std::numeric_limits<double>::max();
    stats.maxX = std::numeric_limits<double>::lowest();
    stats.minY = std::numeric_limits<double>::max();
    stats.maxY = std::numeric_limits<double>::lowest();

    double totalContrast = 0.0;
    stats.minContrast = std::numeric_limits<double>::max();
    stats.maxContrast = 0.0;

    for (int32_t level = 0; level < stats.numLevels; ++level) {
        const auto& levelModel = impl_->levels_[level];
        stats.pointsPerLevel[level] = static_cast<int32_t>(levelModel.points.size());

        if (level == 0) {
            stats.numPoints = stats.pointsPerLevel[0];

            for (const auto& pt : levelModel.points) {
                stats.minX = std::min(stats.minX, pt.x);
                stats.maxX = std::max(stats.maxX, pt.x);
                stats.minY = std::min(stats.minY, pt.y);
                stats.maxY = std::max(stats.maxY, pt.y);

                totalContrast += pt.magnitude;
                stats.minContrast = std::min(stats.minContrast, pt.magnitude);
                stats.maxContrast = std::max(stats.maxContrast, pt.magnitude);
            }

            if (stats.numPoints > 0) {
                stats.meanContrast = totalContrast / stats.numPoints;
            }
        }
    }

    return stats;
}

std::vector<ModelPoint> ShapeModel::GetModelPoints(int32_t level) const {
    if (level < 0 || level >= static_cast<int32_t>(impl_->levels_.size())) {
        return {};
    }
    return impl_->levels_[level].points;
}

int32_t ShapeModel::NumLevels() const {
    return static_cast<int32_t>(impl_->levels_.size());
}

Point2d ShapeModel::GetOrigin() const {
    return impl_->origin_;
}

Size2i ShapeModel::GetSize() const {
    return impl_->templateSize_;
}

const ModelParams& ShapeModel::GetParams() const {
    return impl_->params_;
}

void ShapeModel::SetTimingParams(const ShapeModelTimingParams& params) {
    impl_->timingParams_ = params;
}

const ShapeModelCreateTiming& ShapeModel::GetCreateTiming() const {
    return impl_->createTiming_;
}

const ShapeModelFindTiming& ShapeModel::GetFindTiming() const {
    return impl_->findTiming_;
}

std::vector<Point2d> ShapeModel::GetModelContour(int32_t level) const {
    std::vector<Point2d> contour;
    if (level < 0 || level >= static_cast<int32_t>(impl_->levels_.size())) {
        return contour;
    }

    const auto& points = impl_->levels_[level].points;
    contour.reserve(points.size());

    for (const auto& pt : points) {
        contour.push_back(Point2d{pt.x, pt.y});
    }

    return contour;
}

std::vector<Point2d> ShapeModel::GetMatchContour(const MatchResult& match) const {
    auto modelContour = GetModelContour(0);
    std::vector<Point2d> transformed;
    transformed.reserve(modelContour.size());

    for (const auto& pt : modelContour) {
        transformed.push_back(match.TransformPoint(pt));
    }

    return transformed;
}

bool ShapeModel::Save(const std::string& filename) const {
    if (!impl_ || !impl_->valid_) {
        return false;
    }

    using Platform::BinaryWriter;
    BinaryWriter writer(filename);
    if (!writer.IsOpen()) {
        return false;
    }

    // Magic number and version
    const uint32_t MAGIC = 0x4D495351;  // "QISM" - QiVision Shape Model
    const uint32_t VERSION = 3;         // v3: Halcon-compatible parameters
    writer.Write(MAGIC);
    writer.Write(VERSION);

    // Model parameters - v3 format (Halcon-compatible)
    writer.Write(static_cast<int32_t>(impl_->params_.contrastMode));
    writer.Write(impl_->params_.contrastHigh);
    writer.Write(impl_->params_.contrastLow);
    writer.Write(impl_->params_.contrastMax);
    writer.Write(impl_->params_.minComponentSize);
    writer.Write(impl_->params_.minContrast);
    writer.Write(static_cast<int32_t>(impl_->params_.optimization));
    writer.Write(impl_->params_.pregeneration);
    writer.Write(static_cast<int32_t>(impl_->params_.metric));
    writer.Write(impl_->params_.numLevels);
    writer.Write(impl_->params_.startLevel);
    writer.Write(impl_->params_.angleStart);
    writer.Write(impl_->params_.angleExtent);
    writer.Write(impl_->params_.angleStep);
    writer.Write(impl_->params_.scaleMin);
    writer.Write(impl_->params_.scaleMax);
    writer.Write(static_cast<int32_t>(impl_->params_.polarity));

    // Origin and template size
    writer.Write(impl_->origin_.x);
    writer.Write(impl_->origin_.y);
    writer.Write(impl_->templateSize_.width);
    writer.Write(impl_->templateSize_.height);

    // Model bounds
    writer.Write(impl_->modelMinX_);
    writer.Write(impl_->modelMaxX_);
    writer.Write(impl_->modelMinY_);
    writer.Write(impl_->modelMaxY_);

    // Pyramid levels
    uint32_t numLevels = static_cast<uint32_t>(impl_->levels_.size());
    writer.Write(numLevels);

    for (const auto& level : impl_->levels_) {
        // Level metadata
        writer.Write(level.width);
        writer.Write(level.height);
        writer.Write(level.scale);

        // Model points
        uint32_t numPoints = static_cast<uint32_t>(level.points.size());
        writer.Write(numPoints);

        for (const auto& pt : level.points) {
            writer.Write(pt.x);
            writer.Write(pt.y);
            writer.Write(pt.angle);
            writer.Write(pt.magnitude);
            writer.Write(pt.angleBin);
            writer.Write(pt.weight);
            writer.Write(pt.cosAngle);
            writer.Write(pt.sinAngle);
        }
    }

    writer.Close();
    return true;
}

bool ShapeModel::Load(const std::string& filename) {
    using Platform::BinaryReader;
    BinaryReader reader(filename);
    if (!reader.IsOpen()) {
        return false;
    }

    // Check magic number
    const uint32_t MAGIC = 0x4D495351;
    uint32_t magic = reader.Read<uint32_t>();
    if (magic != MAGIC) {
        return false;  // Not a valid shape model file
    }

    // Check version (support v1, v2, v3)
    uint32_t version = reader.Read<uint32_t>();
    if (version < 1 || version > 3) {
        return false;  // Unsupported version
    }

    // Create new implementation
    impl_ = std::make_unique<Internal::ShapeModelImpl>();

    // Model parameters - version-dependent format
    if (version >= 3) {
        // v3: Halcon-compatible parameters
        impl_->params_.contrastMode = static_cast<ContrastMode>(reader.Read<int32_t>());
        impl_->params_.contrastHigh = reader.Read<double>();
        impl_->params_.contrastLow = reader.Read<double>();
        impl_->params_.contrastMax = reader.Read<double>();
        impl_->params_.minComponentSize = reader.Read<int32_t>();
        impl_->params_.minContrast = reader.Read<double>();
        impl_->params_.optimization = static_cast<OptimizationMode>(reader.Read<int32_t>());
        impl_->params_.pregeneration = reader.Read<bool>();
        impl_->params_.metric = static_cast<MetricMode>(reader.Read<int32_t>());
        impl_->params_.numLevels = reader.Read<int32_t>();
        impl_->params_.startLevel = reader.Read<int32_t>();
        impl_->params_.angleStart = reader.Read<double>();
        impl_->params_.angleExtent = reader.Read<double>();
        impl_->params_.angleStep = reader.Read<double>();
        impl_->params_.scaleMin = reader.Read<double>();
        impl_->params_.scaleMax = reader.Read<double>();
        impl_->params_.polarity = static_cast<MatchPolarity>(reader.Read<int32_t>());
    } else {
        // v1/v2: Legacy format - convert to new format
        double minContrast = reader.Read<double>();
        double hysteresisContrast = 0.0;
        if (version >= 2) {
            hysteresisContrast = reader.Read<double>();
        }
        double maxContrast = reader.Read<double>();

        // Convert to new format
        impl_->params_.contrastMode = (hysteresisContrast > 0)
            ? ContrastMode::Manual : ContrastMode::Manual;
        impl_->params_.contrastHigh = minContrast;
        impl_->params_.contrastLow = hysteresisContrast;
        impl_->params_.contrastMax = maxContrast;
        impl_->params_.minContrast = minContrast;

        impl_->params_.numLevels = reader.Read<int32_t>();
        impl_->params_.startLevel = reader.Read<int32_t>();
        impl_->params_.angleStart = reader.Read<double>();
        impl_->params_.angleExtent = reader.Read<double>();
        impl_->params_.scaleMin = reader.Read<double>();
        impl_->params_.scaleMax = reader.Read<double>();

        bool optimizeModel = reader.Read<bool>();
        int32_t maxModelPoints = reader.Read<int32_t>();
        double modelPointSpacing = reader.Read<double>();
        (void)maxModelPoints;
        (void)modelPointSpacing;

        // Convert optimization flag to mode
        impl_->params_.optimization = optimizeModel
            ? OptimizationMode::Auto : OptimizationMode::None;

        impl_->params_.polarity = static_cast<MatchPolarity>(reader.Read<int32_t>());
        impl_->params_.metric = MetricMode::UsePolarity;  // Default
    }

    // Origin and template size
    impl_->origin_.x = reader.Read<double>();
    impl_->origin_.y = reader.Read<double>();
    impl_->templateSize_.width = reader.Read<int32_t>();
    impl_->templateSize_.height = reader.Read<int32_t>();

    // Model bounds
    impl_->modelMinX_ = reader.Read<double>();
    impl_->modelMaxX_ = reader.Read<double>();
    impl_->modelMinY_ = reader.Read<double>();
    impl_->modelMaxY_ = reader.Read<double>();

    // Pyramid levels
    uint32_t numLevels = reader.Read<uint32_t>();
    impl_->levels_.resize(numLevels);

    for (uint32_t i = 0; i < numLevels; ++i) {
        auto& level = impl_->levels_[i];

        // Level metadata
        level.width = reader.Read<int32_t>();
        level.height = reader.Read<int32_t>();
        level.scale = reader.Read<double>();

        // Model points
        uint32_t numPoints = reader.Read<uint32_t>();
        level.points.resize(numPoints);

        for (uint32_t j = 0; j < numPoints; ++j) {
            auto& pt = level.points[j];
            pt.x = reader.Read<double>();
            pt.y = reader.Read<double>();
            pt.angle = reader.Read<double>();
            pt.magnitude = reader.Read<double>();
            pt.angleBin = reader.Read<int32_t>();
            pt.weight = reader.Read<double>();
            pt.cosAngle = reader.Read<double>();
            pt.sinAngle = reader.Read<double>();
        }

        // Rebuild SoA data for SIMD
        level.BuildSoA();
    }

    impl_->valid_ = true;
    return true;
}

double ShapeModel::ComputeScore(const QImage& image,
                                 double x, double y,
                                 double angle, double scale) const {
    if (!impl_->valid_) {
        return 0.0;
    }

    // Build angle pyramid for target
    AnglePyramidParams params;
    params.numLevels = 1;

    // Use minContrast if specified, otherwise use 50% of contrastHigh
    double searchContrast = (impl_->params_.minContrast > 0)
        ? impl_->params_.minContrast
        : impl_->params_.contrastHigh * 0.5;
    params.minContrast = searchContrast;

    AnglePyramid pyramid;
    if (!pyramid.Build(image, params)) {
        return 0.0;
    }

    return impl_->ComputeScoreAtPosition(pyramid, 0, x, y, angle, scale, 0.0);
}

bool ShapeModel::RefineMatch(const QImage& image, MatchResult& match,
                              SubpixelMethod method) const {
    if (!impl_->valid_) {
        return false;
    }

    AnglePyramidParams params;
    params.numLevels = 1;

    // Use minContrast if specified, otherwise use 50% of contrastHigh
    double searchContrast = (impl_->params_.minContrast > 0)
        ? impl_->params_.minContrast
        : impl_->params_.contrastHigh * 0.5;
    params.minContrast = searchContrast;

    AnglePyramid pyramid;
    if (!pyramid.Build(image, params)) {
        return false;
    }

    impl_->RefinePosition(pyramid, match, method);
    return true;
}

// =============================================================================
// Utility Functions
// =============================================================================

ShapeModel CreateShapeModelFromFile(const std::string& /*filename*/,
                                     const ModelParams& /*params*/) {
    // TODO: Implement file loading
    return ShapeModel();
}

int32_t EstimateOptimalLevels(int32_t imageWidth, int32_t imageHeight,
                               int32_t modelWidth, int32_t modelHeight) {
    int32_t minImageDim = std::min(imageWidth, imageHeight);
    int32_t minModelDim = std::min(modelWidth, modelHeight);

    int32_t levels = 1;
    while (minImageDim >= 64 && minModelDim >= 8) {
        minImageDim /= 2;
        minModelDim /= 2;
        levels++;
    }

    return std::min(levels, 6);
}

double EstimateAngleStep(int32_t modelSize) {
    // Angle step should be small enough that rotating by one step
    // doesn't move any model point by more than ~1 pixel
    // For a point at distance r from center: displacement = r * angleStep
    // Want displacement <= 1, so angleStep <= 1/r

    double maxRadius = modelSize / 2.0;
    if (maxRadius < 1.0) maxRadius = 1.0;

    double step = 1.0 / maxRadius;

    // Clamp to reasonable range
    return std::clamp(step, 0.01, 0.2);  // ~0.6 to ~11 degrees
}

} // namespace Qi::Vision::Matching
