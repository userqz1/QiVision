/**
 * @file ShapeModel.cpp
 * @brief Halcon-style ShapeModel API implementation
 *
 * Provides Halcon-compatible free functions that wrap the internal
 * ShapeModelImpl class.
 */

#include "ShapeModelImpl.h"
#include <QiVision/Core/QContourArray.h>
#include <QiVision/Platform/FileIO.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace Qi::Vision::Matching {

// Import Internal types for pyramid operations
using Qi::Vision::Internal::AnglePyramid;
using Qi::Vision::Internal::AnglePyramidParams;

namespace {
bool g_debugCreateModel = false;
}

// =============================================================================
// String to Enum Conversion Helpers
// =============================================================================

namespace {

OptimizationMode ParseOptimization(const std::string& str) {
    if (str == "none") return OptimizationMode::None;
    if (str == "auto") return OptimizationMode::Auto;
    if (str == "point_reduction_low") return OptimizationMode::PointReductionLow;
    if (str == "point_reduction_medium") return OptimizationMode::PointReductionMedium;
    if (str == "point_reduction_high") return OptimizationMode::PointReductionHigh;
    return OptimizationMode::Auto;  // default
}

MetricMode ParseMetric(const std::string& str) {
    if (str == "use_polarity") return MetricMode::UsePolarity;
    if (str == "ignore_global_polarity") return MetricMode::IgnoreGlobalPolarity;
    if (str == "ignore_local_polarity") return MetricMode::IgnoreLocalPolarity;
    if (str == "ignore_color_polarity") return MetricMode::IgnoreColorPolarity;
    return MetricMode::UsePolarity;  // default
}

SubpixelMethod ParseSubpixel(const std::string& str) {
    if (str == "none" || str == "false") return SubpixelMethod::None;
    if (str == "interpolation" || str == "true") return SubpixelMethod::Parabolic;
    if (str == "least_squares") return SubpixelMethod::LeastSquares;
    if (str == "least_squares_high") return SubpixelMethod::LeastSquaresHigh;
    if (str == "least_squares_very_high") return SubpixelMethod::LeastSquaresVeryHigh;
    return SubpixelMethod::LeastSquares;  // default
}

std::string MetricToString(MetricMode mode) {
    switch (mode) {
        case MetricMode::UsePolarity: return "use_polarity";
        case MetricMode::IgnoreGlobalPolarity: return "ignore_global_polarity";
        case MetricMode::IgnoreLocalPolarity: return "ignore_local_polarity";
        case MetricMode::IgnoreColorPolarity: return "ignore_color_polarity";
        default: return "use_polarity";
    }
}

double EstimateAutoMinContrastFromPyramid(const AnglePyramid& pyramid) {
    const auto& level0 = pyramid.GetLevel(0);
    const QImage& mag = level0.gradMag;
    if (!mag.IsValid() || mag.Empty()) {
        return 10.0;
    }

    const int32_t width = mag.Width();
    const int32_t height = mag.Height();
    if (width <= 0 || height <= 0) {
        return 10.0;
    }

    const double targetSamples = 20000.0;
    const double total = static_cast<double>(width) * static_cast<double>(height);
    const int32_t step = std::max(1, static_cast<int32_t>(std::sqrt(total / targetSamples)));

    std::vector<float> samples;
    samples.reserve(static_cast<size_t>((width / step + 1) * (height / step + 1)));

    for (int32_t y = 0; y < height; y += step) {
        const float* row = static_cast<const float*>(mag.RowPtr(y));
        for (int32_t x = 0; x < width; x += step) {
            samples.push_back(row[x]);
        }
    }

    if (samples.empty()) {
        return 10.0;
    }

    const size_t idx90 = static_cast<size_t>(samples.size() * 0.90);
    std::nth_element(samples.begin(), samples.begin() + idx90, samples.end());
    const float p90 = samples[idx90];

    const size_t idx50 = samples.size() / 2;
    std::nth_element(samples.begin(), samples.begin() + idx50, samples.end());
    const float p50 = samples[idx50];

    double minContrast = std::max(3.0, std::min(static_cast<double>(p90) * 0.35,
                                                static_cast<double>(p90) * 0.70));
    minContrast = std::max(minContrast, static_cast<double>(p50) * 0.80);
    if (!std::isfinite(minContrast) || minContrast <= 0.0) {
        minContrast = 10.0;
    }

    return minContrast;
}

/**
 * @brief Parse contrast parameter string
 *
 * Supports Halcon-style contrast formats:
 * - "auto" or "auto_contrast": Auto-detect threshold
 * - "auto_contrast_hyst": Auto-detect hysteresis thresholds
 * - "auto_min_size": Auto with minimum component size filter
 * - Numeric string (e.g., "30"): Manual single threshold
 * - "[low,high]" (e.g., "[10,30]"): Manual hysteresis thresholds
 * - "[low,high,minSize]" (e.g., "[10,30,15]"): Hysteresis + min component filter
 *
 * @param str Contrast parameter string
 * @param[out] mode Contrast mode
 * @param[out] contrastHigh High threshold value
 * @param[out] contrastLow Low threshold value (for hysteresis)
 * @param[out] minComponentSize Minimum component size (for filtering)
 */
void ParseContrast(const std::string& str, ContrastMode& mode,
                   double& contrastHigh, double& contrastLow, int32_t& minComponentSize) {
    // Default values
    mode = ContrastMode::Manual;
    contrastHigh = 30.0;
    contrastLow = 0.0;
    minComponentSize = 3;

    if (str.empty()) {
        return;
    }

    // Check for auto modes
    if (str == "auto" || str == "auto_contrast") {
        mode = ContrastMode::Auto;
        contrastHigh = 0.0;  // Will be auto-detected
        contrastLow = 0.0;
        return;
    }

    if (str == "auto_contrast_hyst" || str == "auto_hyst") {
        mode = ContrastMode::AutoHysteresis;
        contrastHigh = 0.0;
        contrastLow = 0.0;
        return;
    }

    if (str == "auto_min_size") {
        mode = ContrastMode::AutoMinSize;
        contrastHigh = 0.0;
        contrastLow = 0.0;
        return;
    }

    // Check for [low,high] or [low,high,minSize] format
    if (str.size() > 2 && str.front() == '[' && str.back() == ']') {
        std::string inner = str.substr(1, str.size() - 2);

        // Split by commas
        std::vector<std::string> parts;
        size_t start = 0;
        size_t pos;
        while ((pos = inner.find(',', start)) != std::string::npos) {
            parts.push_back(inner.substr(start, pos - start));
            start = pos + 1;
        }
        parts.push_back(inner.substr(start));

        try {
            if (parts.size() >= 2) {
                contrastLow = std::stod(parts[0]);
                contrastHigh = std::stod(parts[1]);
                mode = ContrastMode::Manual;

                if (parts.size() >= 3) {
                    minComponentSize = std::stoi(parts[2]);
                }
                return;
            }
        } catch (...) {
            // Parse error, fall through to single value
        }
    }

    // Try to parse as single numeric value
    try {
        contrastHigh = std::stod(str);
        contrastLow = 0.0;
        mode = ContrastMode::Manual;
    } catch (...) {
        // Invalid format, use default
        mode = ContrastMode::Manual;
        contrastHigh = 30.0;
        contrastLow = 0.0;
    }
}

} // anonymous namespace

// =============================================================================
// ShapeModel Class (Handle)
// =============================================================================

ShapeModel::ShapeModel() : impl_(std::make_unique<Internal::ShapeModelImpl>()) {}

ShapeModel::~ShapeModel() = default;

ShapeModel::ShapeModel(const ShapeModel& other)
    : impl_(other.impl_ ? std::make_unique<Internal::ShapeModelImpl>(*other.impl_) : nullptr) {}

ShapeModel::ShapeModel(ShapeModel&& other) noexcept = default;

ShapeModel& ShapeModel::operator=(const ShapeModel& other) {
    if (this != &other) {
        impl_ = other.impl_ ? std::make_unique<Internal::ShapeModelImpl>(*other.impl_) : nullptr;
    }
    return *this;
}

ShapeModel& ShapeModel::operator=(ShapeModel&& other) noexcept = default;

bool ShapeModel::IsValid() const {
    return impl_ && impl_->valid_;
}

// =============================================================================
// Model Creation Functions
// =============================================================================

void CreateShapeModel(
    const QImage& templateImage,
    ShapeModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast)
{
    CreateShapeModel(templateImage, Rect2i{}, model, numLevels,
                     angleStart, angleExtent, angleStep,
                     optimization, metric, contrast, minContrast);
}

void CreateShapeModel(
    const QImage& templateImage,
    const Rect2i& roi,
    ShapeModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast)
{
    model = ShapeModel();

    // Set up parameters
    ModelParams params;
    params.numLevels = numLevels;
    params.angleStart = angleStart;
    params.angleExtent = angleExtent;
    params.angleStep = angleStep;
    params.optimization = ParseOptimization(optimization);
    params.metric = ParseMetric(metric);

    // Parse contrast parameter (supports "auto", numeric, "[low,high]", or "[low,high,minSize]")
    ParseContrast(contrast, params.contrastMode, params.contrastHigh, params.contrastLow, params.minComponentSize);
    params.minContrast = minContrast;

    model.Impl()->params_ = params;
    model.Impl()->timingParams_.debugCreateModel = g_debugCreateModel;

    // Create model
    Point2d origin{0, 0};
    if (!model.Impl()->CreateModel(templateImage, roi, origin)) {
        model = ShapeModel();  // Set to invalid model
    }
}

void CreateShapeModel(
    const QImage& templateImage,
    const QRegion& region,
    ShapeModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast)
{
    model = ShapeModel();

    // Set up parameters
    ModelParams params;
    params.numLevels = numLevels;
    params.angleStart = angleStart;
    params.angleExtent = angleExtent;
    params.angleStep = angleStep;
    params.optimization = ParseOptimization(optimization);
    params.metric = ParseMetric(metric);

    // Parse contrast parameter
    ParseContrast(contrast, params.contrastMode, params.contrastHigh, params.contrastLow, params.minComponentSize);
    params.minContrast = minContrast;

    model.Impl()->params_ = params;

    // Create model with QRegion
    Point2d origin{0, 0};
    if (!model.Impl()->CreateModel(templateImage, region, origin)) {
        model = ShapeModel();  // Set to invalid model
    }
    if (g_debugCreateModel && model.IsValid()) {
        auto* impl = model.Impl();
        std::printf("[CreateShapeModel] contrast=[%.2f, %.2f] minContrast=%.2f points=%zu\n",
                    impl->params_.contrastLow, impl->params_.contrastHigh, impl->params_.minContrast,
                    impl->levels_.empty() ? 0u : impl->levels_[0].points.size());
        std::fflush(stdout);
    }
}

void CreateScaledShapeModel(
    const QImage& templateImage,
    ShapeModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    double scaleMin,
    double scaleMax,
    double scaleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast)
{
    CreateScaledShapeModel(templateImage, Rect2i{}, model, numLevels,
                           angleStart, angleExtent, angleStep,
                           scaleMin, scaleMax, scaleStep,
                           optimization, metric, contrast, minContrast);
}

void CreateScaledShapeModel(
    const QImage& templateImage,
    const Rect2i& roi,
    ShapeModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    double scaleMin,
    double scaleMax,
    double scaleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast)
{
    model = ShapeModel();

    ModelParams params;
    params.numLevels = numLevels;
    params.angleStart = angleStart;
    params.angleExtent = angleExtent;
    params.angleStep = angleStep;
    params.scaleMin = scaleMin;
    params.scaleMax = scaleMax;
    params.scaleStep = scaleStep;
    params.optimization = ParseOptimization(optimization);
    params.metric = ParseMetric(metric);

    // Parse contrast parameter (supports "auto", numeric, "[low,high]", or "[low,high,minSize]")
    ParseContrast(contrast, params.contrastMode, params.contrastHigh, params.contrastLow, params.minComponentSize);
    params.minContrast = minContrast;

    model.Impl()->params_ = params;
    model.Impl()->timingParams_.debugCreateModel = g_debugCreateModel;

    Point2d origin{0, 0};
    if (!model.Impl()->CreateModel(templateImage, roi, origin)) {
        model = ShapeModel();
    }
}

void CreateScaledShapeModel(
    const QImage& templateImage,
    const QRegion& region,
    ShapeModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    double scaleMin,
    double scaleMax,
    double scaleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast)
{
    model = ShapeModel();

    ModelParams params;
    params.numLevels = numLevels;
    params.angleStart = angleStart;
    params.angleExtent = angleExtent;
    params.angleStep = angleStep;
    params.scaleMin = scaleMin;
    params.scaleMax = scaleMax;
    params.scaleStep = scaleStep;
    params.optimization = ParseOptimization(optimization);
    params.metric = ParseMetric(metric);

    // Parse contrast parameter (supports "auto", numeric, "[low,high]", or "[low,high,minSize]")
    ParseContrast(contrast, params.contrastMode, params.contrastHigh, params.contrastLow, params.minComponentSize);
    params.minContrast = minContrast;

    model.Impl()->params_ = params;

    Point2d origin{0, 0};
    if (!model.Impl()->CreateModel(templateImage, region, origin)) {
        model = ShapeModel();
    }
    if (g_debugCreateModel && model.IsValid()) {
        auto* impl = model.Impl();
        std::printf("[CreateScaledShapeModel] contrast=[%.2f, %.2f] minContrast=%.2f points=%zu\n",
                    impl->params_.contrastLow, impl->params_.contrastHigh, impl->params_.minContrast,
                    impl->levels_.empty() ? 0u : impl->levels_[0].points.size());
        std::fflush(stdout);
    }
}

// =============================================================================
// Model Search Functions
// =============================================================================

void FindShapeModel(
    const QImage& image,
    const ShapeModel& model,
    double angleStart,
    double angleExtent,
    double minScore,
    int32_t numMatches,
    double maxOverlap,
    const std::string& subPixel,
    int32_t numLevels,
    double greediness,
    std::vector<double>& rows,
    std::vector<double>& cols,
    std::vector<double>& angles,
    std::vector<double>& scores)
{
    // Clear outputs
    rows.clear();
    cols.clear();
    angles.clear();
    scores.clear();

    if (!model.IsValid()) {
        return;
    }

    auto* impl = const_cast<Internal::ShapeModelImpl*>(model.Impl());

    // Set up search parameters
    SearchParams params;
    params.angleStart = angleStart;
    params.angleExtent = angleExtent;
    params.minScore = minScore;
    params.maxMatches = (numMatches == 0) ? 1000 : numMatches;
    params.maxOverlap = maxOverlap;
    params.subpixelMethod = ParseSubpixel(subPixel);
    params.greediness = greediness;
    // numLevels: 0 means use model levels
    (void)numLevels;

    // Build target pyramid (with timing)
    auto tPyramidStart = std::chrono::high_resolution_clock::now();

    Internal::AnglePyramidParams pyramidParams;
    pyramidParams.numLevels = static_cast<int32_t>(impl->levels_.size());

    // Keep pyramid minContrast low; scoring applies search-time thresholds
    pyramidParams.minContrast = 1.0;
    pyramidParams.smoothSigma = 0.5;
    pyramidParams.extractEdgePoints = false;
    pyramidParams.storeDirection = false;  // Search mode: skip storing gradDir

    Internal::AnglePyramid targetPyramid;
    if (!targetPyramid.Build(image, pyramidParams)) {
        return;
    }

    const double savedMinContrast = impl->params_.minContrast;
    if (savedMinContrast <= 0.0) {
        impl->params_.minContrast = EstimateAutoMinContrastFromPyramid(targetPyramid);
        if (impl->timingParams_.debugCreateModel) {
            std::printf("[Find] auto minContrast=%.2f\n", impl->params_.minContrast);
        }
    }

    // Print detailed pyramid timing
    targetPyramid.GetTiming().Print();

    auto tPyramidEnd = std::chrono::high_resolution_clock::now();
    auto tSearchStart = tPyramidEnd;

    std::vector<MatchResult> results = impl->SearchPyramid(targetPyramid, params);

    auto tSearchEnd = std::chrono::high_resolution_clock::now();
    fprintf(stderr, "[Find] Pyramid: %.1fms, Search: %.1fms\n",
            std::chrono::duration<double, std::milli>(tPyramidEnd - tPyramidStart).count(),
            std::chrono::duration<double, std::milli>(tSearchEnd - tSearchStart).count());

    // Convert results to output vectors
    rows.reserve(results.size());
    cols.reserve(results.size());
    angles.reserve(results.size());
    scores.reserve(results.size());

    for (const auto& r : results) {
        rows.push_back(r.y);    // Halcon uses row (y) first
        cols.push_back(r.x);    // then column (x)
        angles.push_back(r.angle);
        scores.push_back(r.score);
    }

    impl->params_.minContrast = savedMinContrast;
}

void FindScaledShapeModel(
    const QImage& image,
    const ShapeModel& model,
    double angleStart,
    double angleExtent,
    double scaleMin,
    double scaleMax,
    double minScore,
    int32_t numMatches,
    double maxOverlap,
    const std::string& subPixel,
    int32_t numLevels,
    double greediness,
    std::vector<double>& rows,
    std::vector<double>& cols,
    std::vector<double>& angles,
    std::vector<double>& scales,
    std::vector<double>& scores)
{
    // Clear outputs
    rows.clear();
    cols.clear();
    angles.clear();
    scales.clear();
    scores.clear();

    if (!model.IsValid()) {
        return;
    }

    // Validate scale range
    if (scaleMin <= 0 || scaleMax <= 0 || scaleMin > scaleMax) {
        return;
    }

    // If scale range is essentially 1.0, use regular FindShapeModel
    constexpr double SCALE_TOLERANCE = 1e-6;
    if (std::abs(scaleMin - 1.0) < SCALE_TOLERANCE &&
        std::abs(scaleMax - 1.0) < SCALE_TOLERANCE) {
        FindShapeModel(image, model, angleStart, angleExtent, minScore,
                       numMatches, maxOverlap, subPixel, numLevels, greediness,
                       rows, cols, angles, scores);
        scales.resize(scores.size(), 1.0);
        return;
    }

    // Get model implementation
    auto* impl = const_cast<Internal::ShapeModelImpl*>(model.Impl());

    // Auto-compute scale step if not specified
    // Rule: step = (max - min) / 10 for reasonable coverage, but at least 0.01
    // For typical range [0.9, 1.1], this gives step = 0.02 (10 scales)
    double scaleStep = impl->params_.scaleStep;
    if (scaleStep <= 0.0) {
        scaleStep = 0.02;
        if (scaleMax > scaleMin) {
            scaleStep = std::max(0.01, (scaleMax - scaleMin) / 10.0);
        }
    }

    // Build pyramid for search image (reused for all scales)
    AnglePyramidParams pyramidParams;
    pyramidParams.numLevels = (numLevels > 0) ? numLevels : impl->params_.numLevels;
    pyramidParams.smoothSigma = 0.5;
    pyramidParams.minContrast = 1.0;
    pyramidParams.useNMS = true;
    pyramidParams.extractEdgePoints = false;   // search pyramid无需边缘点
    pyramidParams.storeDirection = false;      // 不存方向图，节省构建时间与内存

    AnglePyramid targetPyramid;
    if (!targetPyramid.Build(image, pyramidParams)) {
        return;  // Failed to build pyramid
    }

    const double savedMinContrast = impl->params_.minContrast;
    if (savedMinContrast <= 0.0) {
        impl->params_.minContrast = EstimateAutoMinContrastFromPyramid(targetPyramid);
        if (impl->timingParams_.debugCreateModel) {
            std::printf("[FindScaled] auto minContrast=%.2f\n", impl->params_.minContrast);
        }
    }

    // Collect all matches from different scales
    struct ScaledMatch {
        double row = 0.0;
        double col = 0.0;
        double angle = 0.0;
        double scale = 1.0;
        double score = 0.0;
    };
    std::vector<ScaledMatch> allMatches;

    // Search at each scale level
    for (double scale = scaleMin; scale <= scaleMax + SCALE_TOLERANCE; scale += scaleStep) {
        SearchParams params;
        params.angleStart = angleStart;
        params.angleExtent = angleExtent;
        params.minScore = minScore;
        params.maxMatches = (numMatches == 0) ? 1000 : numMatches;
        params.maxOverlap = maxOverlap;
        params.subpixelMethod = ParseSubpixel(subPixel);
        params.numLevels = numLevels;
        params.greediness = greediness;

        // Set scale for this iteration
        params.scaleMode = ScaleSearchMode::Uniform;
        params.scaleMin = scale;
        params.scaleMax = scale;

        // Search with this scale
        auto results = impl->SearchPyramidScaled(targetPyramid, params, scale);
        if (impl->timingParams_.debugCreateModel) {
            std::printf("[FindScaled] scale=%.3f results=%zu\n", scale, results.size());
            std::fflush(stdout);
        }

        // Collect results
        for (const auto& r : results) {
            ScaledMatch m;
            m.row = r.y;
            m.col = r.x;
            m.angle = r.angle;
            m.scale = scale;
            m.score = r.score;
            allMatches.push_back(m);
        }
    }

    // Sort by score descending
    std::sort(allMatches.begin(), allMatches.end(),
              [](const ScaledMatch& a, const ScaledMatch& b) { return a.score > b.score; });

    // Apply NMS across all scales
    // Use model bounds for overlap calculation
    double modelRadius = std::max(impl->templateSize_.width, impl->templateSize_.height) / 2.0;

    std::vector<bool> suppressed(allMatches.size(), false);
    for (size_t i = 0; i < allMatches.size(); ++i) {
        if (suppressed[i]) continue;

        // Suppress nearby matches with lower score
        for (size_t j = i + 1; j < allMatches.size(); ++j) {
            if (suppressed[j]) continue;

            double dx = allMatches[i].col - allMatches[j].col;
            double dy = allMatches[i].row - allMatches[j].row;
            double dist = std::sqrt(dx * dx + dy * dy);

            // Consider scale difference in overlap
            double avgScale = (allMatches[i].scale + allMatches[j].scale) / 2.0;
            double overlapThreshold = modelRadius * avgScale * (1.0 - maxOverlap);

            if (dist < overlapThreshold) {
                suppressed[j] = true;
            }
        }
    }

    // Collect final results
    int32_t matchCount = 0;
    int32_t maxMatchesToReturn = (numMatches == 0) ? static_cast<int32_t>(allMatches.size()) : numMatches;

    for (size_t i = 0; i < allMatches.size() && matchCount < maxMatchesToReturn; ++i) {
        if (!suppressed[i]) {
            rows.push_back(allMatches[i].row);
            cols.push_back(allMatches[i].col);
            angles.push_back(allMatches[i].angle);
            scales.push_back(allMatches[i].scale);
            scores.push_back(allMatches[i].score);
            ++matchCount;
        }
    }

    impl->params_.minContrast = savedMinContrast;
}

// =============================================================================
// Model Property Functions
// =============================================================================

void GetShapeModelContours(
    const ShapeModel& model,
    int32_t level,
    std::vector<double>& contourRows,
    std::vector<double>& contourCols)
{
    contourRows.clear();
    contourCols.clear();

    if (!model.IsValid()) {
        return;
    }

    auto* impl = model.Impl();
    int32_t actualLevel = (level >= 1) ? level - 1 : 0;  // Halcon uses 1-based

    if (actualLevel < 0 || actualLevel >= static_cast<int32_t>(impl->levels_.size())) {
        return;
    }

    const auto& points = impl->levels_[actualLevel].points;
    contourRows.reserve(points.size());
    contourCols.reserve(points.size());

    for (const auto& pt : points) {
        contourRows.push_back(pt.y);  // Halcon: row = y
        contourCols.push_back(pt.x);  // Halcon: col = x
    }
}

void GetShapeModelXLD(
    const ShapeModel& model,
    int32_t level,
    QContourArray& contours)
{
    contours = QContourArray();

    if (!model.IsValid()) {
        return;
    }

    auto* impl = model.Impl();
    int32_t actualLevel = (level >= 1) ? level - 1 : 0;

    if (actualLevel < 0 || actualLevel >= static_cast<int32_t>(impl->levels_.size())) {
        return;
    }

    const auto& levelModel = impl->levels_[actualLevel];
    const auto& points = levelModel.points;
    const auto& contourStarts = levelModel.contourStarts;
    const auto& contourClosed = levelModel.contourClosed;

    if (points.empty()) {
        return;
    }

    // Use stored contour topology (from XLD tracing during model creation)
    if (!contourStarts.empty() && contourStarts.size() > 1) {
        // Proper contour topology available
        size_t numContours = contourStarts.size() - 1;  // Last value is sentinel

        for (size_t c = 0; c < numContours; ++c) {
            int32_t startIdx = contourStarts[c];
            int32_t endIdx = contourStarts[c + 1];

            if (endIdx <= startIdx) continue;

            QContour contour;
            for (int32_t i = startIdx; i < endIdx; ++i) {
                contour.AddPoint(points[i].x, points[i].y);
            }

            // Close contour if marked as closed
            if (c < contourClosed.size() && contourClosed[c] && contour.Size() > 2) {
                contour.AddPoint(points[startIdx].x, points[startIdx].y);
            }

            if (contour.Size() > 0) {
                contours.Add(contour);
            }
        }
    } else {
        // Fallback: no topology info, return points as single contour
        QContour contour;
        for (const auto& pt : points) {
            contour.AddPoint(pt.x, pt.y);
        }
        if (contour.Size() > 0) {
            contours.Add(contour);
        }
    }
}

void GetShapeModelParams(
    const ShapeModel& model,
    int32_t& numLevels,
    double& angleStart,
    double& angleExtent,
    double& angleStep,
    double& scaleMin,
    double& scaleMax,
    double& scaleStep,
    std::string& metric)
{
    if (!model.IsValid()) {
        numLevels = 0;
        angleStart = 0;
        angleExtent = 0;
        angleStep = 0;
        scaleMin = 1.0;
        scaleMax = 1.0;
        scaleStep = 0;
        metric = "use_polarity";
        return;
    }

    auto* impl = model.Impl();
    numLevels = static_cast<int32_t>(impl->levels_.size());
    angleStart = impl->params_.angleStart;
    angleExtent = impl->params_.angleExtent;
    angleStep = impl->params_.angleStep;
    scaleMin = impl->params_.scaleMin;
    scaleMax = impl->params_.scaleMax;
    scaleStep = impl->params_.scaleStep;
    metric = MetricToString(impl->params_.metric);
}

void GetShapeModelOrigin(
    const ShapeModel& model,
    double& row,
    double& col)
{
    if (!model.IsValid()) {
        row = 0;
        col = 0;
        return;
    }

    auto* impl = model.Impl();
    row = impl->origin_.y;  // Halcon: row = y
    col = impl->origin_.x;  // Halcon: col = x
}

void SetShapeModelOrigin(
    ShapeModel& model,
    double row,
    double col)
{
    if (!model.IsValid()) {
        return;
    }

    auto* impl = model.Impl();
    impl->origin_.y = row;
    impl->origin_.x = col;
}

// =============================================================================
// Model I/O Functions
// =============================================================================

void WriteShapeModel(
    const ShapeModel& model,
    const std::string& filename)
{
    if (!model.IsValid()) {
        throw std::runtime_error("Cannot write invalid shape model");
    }

    auto* impl = model.Impl();

    using Platform::BinaryWriter;
    BinaryWriter writer(filename);
    if (!writer.IsOpen()) {
        throw std::runtime_error("Cannot open file for writing: " + filename);
    }

    // Magic number and version
    const uint32_t MAGIC = 0x4D495351;  // "QISM"
    const uint32_t VERSION = 4;
    writer.Write(MAGIC);
    writer.Write(VERSION);

    // Model parameters
    writer.Write(static_cast<int32_t>(impl->params_.contrastMode));
    writer.Write(impl->params_.contrastHigh);
    writer.Write(impl->params_.contrastLow);
    writer.Write(impl->params_.contrastMax);
    writer.Write(impl->params_.minComponentSize);
    writer.Write(impl->params_.minContrast);
    writer.Write(static_cast<int32_t>(impl->params_.optimization));
    writer.Write(impl->params_.pregeneration);
    writer.Write(static_cast<int32_t>(impl->params_.metric));
    writer.Write(impl->params_.numLevels);
    writer.Write(impl->params_.startLevel);
    writer.Write(impl->params_.angleStart);
    writer.Write(impl->params_.angleExtent);
    writer.Write(impl->params_.angleStep);
    writer.Write(impl->params_.scaleMin);
    writer.Write(impl->params_.scaleMax);
    writer.Write(impl->params_.scaleStep);
    writer.Write(static_cast<int32_t>(impl->params_.polarity));

    // Origin and template size
    writer.Write(impl->origin_.x);
    writer.Write(impl->origin_.y);
    writer.Write(impl->templateSize_.width);
    writer.Write(impl->templateSize_.height);

    // Model bounds
    writer.Write(impl->modelMinX_);
    writer.Write(impl->modelMaxX_);
    writer.Write(impl->modelMinY_);
    writer.Write(impl->modelMaxY_);

    // Pyramid levels
    uint32_t numLevels = static_cast<uint32_t>(impl->levels_.size());
    writer.Write(numLevels);

    for (const auto& level : impl->levels_) {
        writer.Write(level.width);
        writer.Write(level.height);
        writer.Write(level.scale);

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
}

void ReadShapeModel(
    const std::string& filename,
    ShapeModel& model)
{
    using Platform::BinaryReader;
    BinaryReader reader(filename);
    if (!reader.IsOpen()) {
        throw std::runtime_error("Cannot open file for reading: " + filename);
    }

    const uint32_t MAGIC = 0x4D495351;
    uint32_t magic = reader.Read<uint32_t>();
    if (magic != MAGIC) {
        throw std::runtime_error("Invalid shape model file format");
    }

    uint32_t version = reader.Read<uint32_t>();
    if (version < 1 || version > 4) {
        throw std::runtime_error("Unsupported shape model version");
    }

    model = ShapeModel();
    auto* impl = model.Impl();

    if (version >= 3) {
        impl->params_.contrastMode = static_cast<ContrastMode>(reader.Read<int32_t>());
        impl->params_.contrastHigh = reader.Read<double>();
        impl->params_.contrastLow = reader.Read<double>();
        impl->params_.contrastMax = reader.Read<double>();
        impl->params_.minComponentSize = reader.Read<int32_t>();
        impl->params_.minContrast = reader.Read<double>();
        impl->params_.optimization = static_cast<OptimizationMode>(reader.Read<int32_t>());
        impl->params_.pregeneration = reader.Read<bool>();
        impl->params_.metric = static_cast<MetricMode>(reader.Read<int32_t>());
        impl->params_.numLevels = reader.Read<int32_t>();
        impl->params_.startLevel = reader.Read<int32_t>();
        impl->params_.angleStart = reader.Read<double>();
        impl->params_.angleExtent = reader.Read<double>();
        impl->params_.angleStep = reader.Read<double>();
        impl->params_.scaleMin = reader.Read<double>();
        impl->params_.scaleMax = reader.Read<double>();
        if (version >= 4) {
            impl->params_.scaleStep = reader.Read<double>();
        } else {
            impl->params_.scaleStep = 0.0;
        }
        impl->params_.polarity = static_cast<MatchPolarity>(reader.Read<int32_t>());
    } else {
        // Legacy v1/v2 format
        double minContrast = reader.Read<double>();
        double hysteresisContrast = 0.0;
        if (version >= 2) {
            hysteresisContrast = reader.Read<double>();
        }
        double maxContrast = reader.Read<double>();

        impl->params_.contrastMode = ContrastMode::Manual;
        impl->params_.contrastHigh = minContrast;
        impl->params_.contrastLow = hysteresisContrast;
        impl->params_.contrastMax = maxContrast;
        impl->params_.minContrast = minContrast;

        impl->params_.numLevels = reader.Read<int32_t>();
        impl->params_.startLevel = reader.Read<int32_t>();
        impl->params_.angleStart = reader.Read<double>();
        impl->params_.angleExtent = reader.Read<double>();
        impl->params_.scaleMin = reader.Read<double>();
        impl->params_.scaleMax = reader.Read<double>();
        impl->params_.scaleStep = 0.0;

        bool optimizeModel = reader.Read<bool>();
        (void)reader.Read<int32_t>();  // maxModelPoints
        (void)reader.Read<double>();   // modelPointSpacing

        impl->params_.optimization = optimizeModel
            ? OptimizationMode::Auto : OptimizationMode::None;

        impl->params_.polarity = static_cast<MatchPolarity>(reader.Read<int32_t>());
        impl->params_.metric = MetricMode::UsePolarity;
    }

    impl->origin_.x = reader.Read<double>();
    impl->origin_.y = reader.Read<double>();
    impl->templateSize_.width = reader.Read<int32_t>();
    impl->templateSize_.height = reader.Read<int32_t>();

    impl->modelMinX_ = reader.Read<double>();
    impl->modelMaxX_ = reader.Read<double>();
    impl->modelMinY_ = reader.Read<double>();
    impl->modelMaxY_ = reader.Read<double>();

    uint32_t numLevels = reader.Read<uint32_t>();
    impl->levels_.resize(numLevels);

    for (uint32_t i = 0; i < numLevels; ++i) {
        auto& level = impl->levels_[i];

        level.width = reader.Read<int32_t>();
        level.height = reader.Read<int32_t>();
        level.scale = reader.Read<double>();

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

        level.BuildSoA();
    }

    impl->valid_ = true;
}

void ClearShapeModel(ShapeModel& model)
{
    if (model.Impl()) {
        model.Impl()->levels_.clear();
        model.Impl()->scaledModels_.clear();
        model.Impl()->valid_ = false;
    }
}

// =============================================================================
// Utility Functions
// =============================================================================

void DetermineShapeModelParams(
    const QImage& templateImage,
    const Rect2i& roi,
    int32_t& numLevels,
    double& angleStart,
    double& angleExtent,
    double& angleStep,
    double& scaleMin,
    double& scaleMax,
    double& scaleStep,
    double& contrast,
    double& minContrast)
{
    // Determine ROI dimensions
    int32_t roiWidth = (roi.width > 0) ? roi.width : templateImage.Width();
    int32_t roiHeight = (roi.height > 0) ? roi.height : templateImage.Height();
    int32_t minDim = std::min(roiWidth, roiHeight);

    // Auto-determine pyramid levels (stop when top level ~30px)
    numLevels = 1;
    int32_t dim = minDim;
    while (dim > 30 && numLevels < 6) {
        dim /= 2;
        numLevels++;
    }

    // Default angle parameters (full rotation)
    angleStart = 0.0;
    angleExtent = 2.0 * 3.14159265358979323846;

    // Auto-determine angle step based on model size
    double maxRadius = minDim / 2.0;
    if (maxRadius < 1.0) maxRadius = 1.0;
    angleStep = 2.0 * std::asin(1.0 / (2.0 * maxRadius));
    angleStep = std::clamp(angleStep, 0.0087, 0.175);  // ~0.5° to ~10°

    // Default scale (no scaling)
    scaleMin = 1.0;
    scaleMax = 1.0;
    scaleStep = 0.0;

    // Auto-determine contrast (simple heuristic)
    // TODO: Implement proper auto-contrast using histogram analysis
    contrast = 30.0;
    minContrast = 10.0;
}

void InspectShapeModel(
    const ShapeModel& model,
    int32_t level,
    QImage& contrastImage,
    int32_t& numPoints)
{
    contrastImage = QImage();
    numPoints = 0;

    if (!model.IsValid()) {
        return;
    }

    auto* impl = model.Impl();
    int32_t actualLevel = (level >= 1) ? level - 1 : 0;

    if (actualLevel < 0 || actualLevel >= static_cast<int32_t>(impl->levels_.size())) {
        return;
    }

    const auto& levelData = impl->levels_[actualLevel];
    numPoints = static_cast<int32_t>(levelData.points.size());

    // Create contrast visualization image
    int32_t w = levelData.width;
    int32_t h = levelData.height;
    if (w <= 0 || h <= 0) {
        return;
    }

    contrastImage = QImage(w, h, PixelType::UInt8, ChannelType::Gray);
    std::memset(contrastImage.Data(), 0, contrastImage.Height() * contrastImage.Stride());

    uint8_t* data = static_cast<uint8_t*>(contrastImage.Data());
    int32_t stride = static_cast<int32_t>(contrastImage.Stride());

    // Mark model points
    for (const auto& pt : levelData.points) {
        int32_t px = static_cast<int32_t>(pt.x + w / 2);
        int32_t py = static_cast<int32_t>(pt.y + h / 2);
        if (px >= 0 && px < w && py >= 0 && py < h) {
            data[py * stride + px] = 255;
        }
    }
}

void SetShapeModelDebugCreate(ShapeModel& model, bool enable)
{
    if (model.Impl()) {
        model.Impl()->timingParams_.debugCreateModel = enable;
    }
    g_debugCreateModel = enable;
}

void SetShapeModelDebugCreateGlobal(bool enable)
{
    g_debugCreateModel = enable;
    std::printf("[ShapeModel] debugCreateModel=%d\n", enable ? 1 : 0);
    std::fflush(stdout);
}

} // namespace Qi::Vision::Matching
