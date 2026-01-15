/**
 * @file ShapeModel.cpp
 * @brief Halcon-style ShapeModel API implementation
 *
 * Provides Halcon-compatible free functions that wrap the internal
 * ShapeModelImpl class.
 */

#include "ShapeModelImpl.h"
#include <QiVision/Platform/FileIO.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace Qi::Vision::Matching {

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
    if (str == "none") return SubpixelMethod::None;
    if (str == "interpolation") return SubpixelMethod::Parabolic;
    if (str == "least_squares") return SubpixelMethod::LeastSquares;
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

ShapeModel CreateShapeModel(
    const QImage& templateImage,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast)
{
    return CreateShapeModel(templateImage, Rect2i{}, numLevels,
                            angleStart, angleExtent, angleStep,
                            optimization, metric, contrast, minContrast);
}

ShapeModel CreateShapeModel(
    const QImage& templateImage,
    const Rect2i& roi,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast)
{
    ShapeModel model;

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

    // Create model
    Point2d origin{0, 0};
    if (!model.Impl()->CreateModel(templateImage, roi, origin)) {
        return ShapeModel();  // Return invalid model
    }

    return model;
}

ShapeModel CreateShapeModel(
    const QImage& templateImage,
    const QRegion& region,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast)
{
    ShapeModel model;

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
        return ShapeModel();  // Return invalid model
    }

    return model;
}

ShapeModel CreateScaledShapeModel(
    const QImage& templateImage,
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
    ShapeModel model;

    ModelParams params;
    params.numLevels = numLevels;
    params.angleStart = angleStart;
    params.angleExtent = angleExtent;
    params.angleStep = angleStep;
    params.scaleMin = scaleMin;
    params.scaleMax = scaleMax;
    // params.scaleStep = scaleStep;  // TODO: Add to ModelParams if needed
    (void)scaleStep;
    params.optimization = ParseOptimization(optimization);
    params.metric = ParseMetric(metric);

    // Parse contrast parameter (supports "auto", numeric, "[low,high]", or "[low,high,minSize]")
    ParseContrast(contrast, params.contrastMode, params.contrastHigh, params.contrastLow, params.minComponentSize);
    params.minContrast = minContrast;

    model.Impl()->params_ = params;

    Point2d origin{0, 0};
    if (!model.Impl()->CreateModel(templateImage, Rect2i{}, origin)) {
        return ShapeModel();
    }

    return model;
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

    auto* impl = model.Impl();

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

    const bool useLinemod = impl->params_.useLinemod && !impl->linemodFeatures_.empty();

    std::vector<MatchResult> results;

    if (useLinemod) {
        Internal::LinemodPyramidParams linemodParams;
        linemodParams.numLevels = static_cast<int32_t>(impl->linemodFeatures_.size());
        linemodParams.minMagnitude = static_cast<float>(impl->params_.contrastHigh);
        linemodParams.spreadT = 4;
        linemodParams.neighborThreshold = 5;
        linemodParams.smoothSigma = 1.0;
        linemodParams.extractFeatures = false;

        Internal::LinemodPyramid targetPyramid;
        if (!targetPyramid.Build(image, linemodParams)) {
            return;
        }

        auto tPyramidEnd = std::chrono::high_resolution_clock::now();
        auto tSearchStart = tPyramidEnd;

        results = impl->SearchPyramidLinemod(targetPyramid, params);

        auto tSearchEnd = std::chrono::high_resolution_clock::now();
        fprintf(stderr, "[Find] Pyramid: %.1fms, Search: %.1fms\n",
                std::chrono::duration<double, std::milli>(tPyramidEnd - tPyramidStart).count(),
                std::chrono::duration<double, std::milli>(tSearchEnd - tSearchStart).count());
    } else {
        Internal::AnglePyramidParams pyramidParams;
        pyramidParams.numLevels = static_cast<int32_t>(impl->levels_.size());

        // Use same contrast as model creation to avoid detecting weak edges
        double searchContrast = (impl->params_.minContrast > 0)
            ? impl->params_.minContrast
            : impl->params_.contrastHigh;
        pyramidParams.minContrast = searchContrast;
        pyramidParams.smoothSigma = 0.5;
        pyramidParams.extractEdgePoints = false;

        Internal::AnglePyramid targetPyramid;
        if (!targetPyramid.Build(image, pyramidParams)) {
            return;
        }

        auto tPyramidEnd = std::chrono::high_resolution_clock::now();
        auto tSearchStart = tPyramidEnd;

        results = impl->SearchPyramid(targetPyramid, params);

        auto tSearchEnd = std::chrono::high_resolution_clock::now();
        fprintf(stderr, "[Find] Pyramid: %.1fms, Search: %.1fms\n",
                std::chrono::duration<double, std::milli>(tPyramidEnd - tPyramidStart).count(),
                std::chrono::duration<double, std::milli>(tSearchEnd - tSearchStart).count());
    }

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
    // For now, call FindShapeModel (scale search not fully implemented)
    (void)scaleMin;
    (void)scaleMax;

    FindShapeModel(image, model, angleStart, angleExtent, minScore,
                   numMatches, maxOverlap, subPixel, numLevels, greediness,
                   rows, cols, angles, scores);

    // Fill scales with 1.0
    scales.resize(scores.size(), 1.0);
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
    scaleStep = 0;  // Not stored currently
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
    const uint32_t VERSION = 3;
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

ShapeModel ReadShapeModel(const std::string& filename)
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
    if (version < 1 || version > 3) {
        throw std::runtime_error("Unsupported shape model version");
    }

    ShapeModel model;
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
    return model;
}

void ClearShapeModel(ShapeModel& model)
{
    if (model.Impl()) {
        model.Impl()->levels_.clear();
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

} // namespace Qi::Vision::Matching
