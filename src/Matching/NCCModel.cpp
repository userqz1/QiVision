/**
 * @file NCCModel.cpp
 * @brief NCCModel class implementation and public API functions
 *
 * This file contains:
 * - NCCModel class (handle) implementation
 * - Public API functions (CreateNCCModel, FindNCCModel, etc.)
 * - Model I/O functions
 *
 * Actual algorithm implementation is in:
 * - NCCModelCreate.cpp: Model creation
 * - NCCModelSearch.cpp: Search functions
 * - NCCModelScore.cpp: NCC score computation
 */

#include <QiVision/Matching/NCCModel.h>
#include "NCCModelImpl.h"

#include <QiVision/Core/Exception.h>

#include <algorithm>
#include <fstream>
#include <cmath>

namespace Qi::Vision::Matching {

// =============================================================================
// NCCModel Class Implementation
// =============================================================================

NCCModel::NCCModel()
    : impl_(std::make_unique<Internal::NCCModelImpl>())
{
}

NCCModel::~NCCModel() = default;

NCCModel::NCCModel(const NCCModel& other)
    : impl_(nullptr)
{
    if (other.impl_) {
        // Deep copy by creating new impl
        // Note: Full copy not implemented yet
        impl_ = std::make_unique<Internal::NCCModelImpl>();
        // TODO: Copy implementation data
    }
}

NCCModel::NCCModel(NCCModel&& other) noexcept = default;

NCCModel& NCCModel::operator=(const NCCModel& other)
{
    if (this != &other) {
        if (other.impl_) {
            impl_ = std::make_unique<Internal::NCCModelImpl>();
            // TODO: Copy implementation data
        } else {
            impl_.reset();
        }
    }
    return *this;
}

NCCModel& NCCModel::operator=(NCCModel&& other) noexcept = default;

bool NCCModel::IsValid() const
{
    return impl_ && impl_->valid_;
}

// =============================================================================
// Model Creation Functions
// =============================================================================

void CreateNCCModel(
    const QImage& templateImage,
    NCCModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    const std::string& metric)
{
    if (!templateImage.IsValid()) {
        throw Exception("CreateNCCModel: Invalid template image");
    }

    // Set parameters
    auto* impl = model.Impl();
    impl->params_.numLevels = numLevels;
    impl->params_.angleStart = angleStart;
    impl->params_.angleExtent = angleExtent;
    impl->params_.angleStep = angleStep;

    // Parse metric
    if (metric == "use_polarity") {
        impl->metric_ = MetricMode::UsePolarity;
    } else if (metric == "ignore_global_polarity") {
        impl->metric_ = MetricMode::IgnoreGlobalPolarity;
    } else {
        impl->metric_ = MetricMode::UsePolarity;
    }

    // Create model
    if (!impl->CreateModel(templateImage)) {
        throw Exception("CreateNCCModel: Failed to create model");
    }
}

void CreateNCCModel(
    const QImage& templateImage,
    const Rect2i& roi,
    NCCModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    const std::string& metric)
{
    if (!templateImage.IsValid()) {
        throw Exception("CreateNCCModel: Invalid template image");
    }

    // Set parameters
    auto* impl = model.Impl();
    impl->params_.numLevels = numLevels;
    impl->params_.angleStart = angleStart;
    impl->params_.angleExtent = angleExtent;
    impl->params_.angleStep = angleStep;

    // Parse metric
    if (metric == "use_polarity") {
        impl->metric_ = MetricMode::UsePolarity;
    } else if (metric == "ignore_global_polarity") {
        impl->metric_ = MetricMode::IgnoreGlobalPolarity;
    } else {
        impl->metric_ = MetricMode::UsePolarity;
    }

    // Create model with ROI
    if (!impl->CreateModel(templateImage, roi)) {
        throw Exception("CreateNCCModel: Failed to create model from ROI");
    }
}

void CreateNCCModel(
    const QImage& templateImage,
    const QRegion& region,
    NCCModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    const std::string& metric)
{
    if (!templateImage.IsValid()) {
        throw Exception("CreateNCCModel: Invalid template image");
    }

    if (region.Empty()) {
        throw Exception("CreateNCCModel: Empty region");
    }

    // Set parameters
    auto* impl = model.Impl();
    impl->params_.numLevels = numLevels;
    impl->params_.angleStart = angleStart;
    impl->params_.angleExtent = angleExtent;
    impl->params_.angleStep = angleStep;

    // Parse metric
    if (metric == "use_polarity") {
        impl->metric_ = MetricMode::UsePolarity;
    } else if (metric == "ignore_global_polarity") {
        impl->metric_ = MetricMode::IgnoreGlobalPolarity;
    } else {
        impl->metric_ = MetricMode::UsePolarity;
    }

    // Create model with QRegion
    if (!impl->CreateModel(templateImage, region)) {
        throw Exception("CreateNCCModel: Failed to create model from region");
    }
}

void CreateScaledNCCModel(
    const QImage& templateImage,
    NCCModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    double scaleMin,
    double scaleMax,
    double scaleStep,
    const std::string& metric)
{
    if (!templateImage.IsValid()) {
        throw Exception("CreateScaledNCCModel: Invalid template image");
    }

    // Set parameters
    auto* impl = model.Impl();
    impl->params_.numLevels = numLevels;
    impl->params_.angleStart = angleStart;
    impl->params_.angleExtent = angleExtent;
    impl->params_.angleStep = angleStep;
    impl->params_.scaleMin = scaleMin;
    impl->params_.scaleMax = scaleMax;

    // Parse metric
    if (metric == "use_polarity") {
        impl->metric_ = MetricMode::UsePolarity;
    } else if (metric == "ignore_global_polarity") {
        impl->metric_ = MetricMode::IgnoreGlobalPolarity;
    } else {
        impl->metric_ = MetricMode::UsePolarity;
    }

    // Create model (scale search done during Find)
    if (!impl->CreateModel(templateImage)) {
        throw Exception("CreateScaledNCCModel: Failed to create model");
    }
}

// =============================================================================
// Model Search Functions
// =============================================================================

void FindNCCModel(
    const QImage& image,
    const NCCModel& model,
    double angleStart,
    double angleExtent,
    double minScore,
    int32_t numMatches,
    double maxOverlap,
    const std::string& subPixel,
    int32_t numLevels,
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

    if (!image.IsValid()) {
        return;  // Empty result for invalid input
    }

    if (!model.IsValid()) {
        throw Exception("FindNCCModel: Invalid model");
    }

    // Build search parameters
    SearchParams params;
    params.minScore = minScore;
    params.maxMatches = numMatches;
    params.maxOverlap = maxOverlap;
    params.angleStart = angleStart;
    params.angleExtent = angleExtent;
    params.numLevels = numLevels;

    // Parse subpixel mode
    if (subPixel == "none") {
        params.subpixelMethod = SubpixelMethod::None;
    } else if (subPixel == "true" || subPixel == "interpolation") {
        params.subpixelMethod = SubpixelMethod::Parabolic;
    } else {
        params.subpixelMethod = SubpixelMethod::Parabolic;
    }

    // Find matches
    const auto* impl = model.Impl();
    auto matches = impl->Find(image, params);

    // Convert to output format
    rows.reserve(matches.size());
    cols.reserve(matches.size());
    angles.reserve(matches.size());
    scores.reserve(matches.size());

    for (const auto& m : matches) {
        rows.push_back(m.y);
        cols.push_back(m.x);
        angles.push_back(m.angle);
        scores.push_back(m.score);
    }
}

void FindScaledNCCModel(
    const QImage& image,
    const NCCModel& model,
    double angleStart,
    double angleExtent,
    double scaleMin,
    double scaleMax,
    double minScore,
    int32_t numMatches,
    double maxOverlap,
    const std::string& subPixel,
    int32_t numLevels,
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

    if (!image.IsValid()) {
        return;
    }

    if (!model.IsValid()) {
        throw Exception("FindScaledNCCModel: Invalid model");
    }

    // Build search parameters
    SearchParams params;
    params.minScore = minScore;
    params.maxMatches = numMatches;
    params.maxOverlap = maxOverlap;
    params.angleStart = angleStart;
    params.angleExtent = angleExtent;
    params.scaleMode = ScaleSearchMode::Uniform;
    params.scaleMin = scaleMin;
    params.scaleMax = scaleMax;
    params.numLevels = numLevels;

    // Parse subpixel mode
    if (subPixel == "none") {
        params.subpixelMethod = SubpixelMethod::None;
    } else {
        params.subpixelMethod = SubpixelMethod::Parabolic;
    }

    // Find matches
    const auto* impl = model.Impl();
    auto matches = impl->Find(image, params);

    // Convert to output format
    rows.reserve(matches.size());
    cols.reserve(matches.size());
    angles.reserve(matches.size());
    scales.reserve(matches.size());
    scores.reserve(matches.size());

    for (const auto& m : matches) {
        rows.push_back(m.y);
        cols.push_back(m.x);
        angles.push_back(m.angle);
        scales.push_back(m.scaleX);  // Uniform scale
        scores.push_back(m.score);
    }
}

// =============================================================================
// Model Property Functions
// =============================================================================

void GetNCCModelParams(
    const NCCModel& model,
    int32_t& numLevels,
    double& angleStart,
    double& angleExtent,
    double& angleStep,
    std::string& metric)
{
    if (!model.IsValid()) {
        throw Exception("GetNCCModelParams: Invalid model");
    }

    const auto* impl = model.Impl();
    numLevels = static_cast<int32_t>(impl->levels_.size());
    angleStart = impl->params_.angleStart;
    angleExtent = impl->params_.angleExtent;
    angleStep = impl->params_.angleStep;

    switch (impl->metric_) {
        case MetricMode::UsePolarity:
            metric = "use_polarity";
            break;
        case MetricMode::IgnoreGlobalPolarity:
            metric = "ignore_global_polarity";
            break;
        default:
            metric = "use_polarity";
            break;
    }
}

void GetNCCModelOrigin(
    const NCCModel& model,
    double& row,
    double& col)
{
    if (!model.IsValid()) {
        throw Exception("GetNCCModelOrigin: Invalid model");
    }

    const auto* impl = model.Impl();
    row = impl->origin_.y;
    col = impl->origin_.x;
}

void SetNCCModelOrigin(
    NCCModel& model,
    double row,
    double col)
{
    if (!model.IsValid()) {
        throw Exception("SetNCCModelOrigin: Invalid model");
    }

    auto* impl = model.Impl();
    impl->origin_.y = row;
    impl->origin_.x = col;
}

void GetNCCModelSize(
    const NCCModel& model,
    int32_t& width,
    int32_t& height)
{
    if (!model.IsValid()) {
        throw Exception("GetNCCModelSize: Invalid model");
    }

    const auto* impl = model.Impl();
    width = impl->templateSize_.width;
    height = impl->templateSize_.height;
}

// =============================================================================
// Model I/O Functions
// =============================================================================

void WriteNCCModel(
    const NCCModel& model,
    const std::string& filename)
{
    if (!model.IsValid()) {
        throw Exception("WriteNCCModel: Invalid model");
    }

    // TODO: Implement model serialization
    throw Exception("WriteNCCModel: Not implemented yet");
}

void ReadNCCModel(
    const std::string& filename,
    NCCModel& model)
{
    // TODO: Implement model deserialization
    throw Exception("ReadNCCModel: Not implemented yet");
}

void ClearNCCModel(
    NCCModel& model)
{
    auto* impl = model.Impl();
    if (impl) {
        impl->levels_.clear();
        impl->rotatedTemplates_.clear();
        impl->searchAngles_.clear();
        impl->valid_ = false;
    }
}

// =============================================================================
// Utility Functions
// =============================================================================

void DetermineNCCModelParams(
    const QImage& templateImage,
    const Rect2i& roi,
    int32_t& numLevels,
    double& angleStep)
{
    if (!templateImage.IsValid()) {
        throw Exception("DetermineNCCModelParams: Invalid template image");
    }

    // Get template dimensions
    int32_t width = templateImage.Width();
    int32_t height = templateImage.Height();

    if (roi.width > 0 && roi.height > 0) {
        width = roi.width;
        height = roi.height;
    }

    // Compute optimal pyramid levels
    int32_t minDim = std::min(width, height);
    numLevels = 1;
    while (minDim >= 16 && numLevels < 6) {
        minDim /= 2;
        numLevels++;
    }

    // Compute optimal angle step (~1 pixel arc at edge)
    double radius = std::max(width, height) * 0.5;
    if (radius < 10) radius = 10;
    angleStep = std::atan(1.0 / radius);

    // Clamp to reasonable range
    constexpr double MIN_ANGLE_STEP = 0.005;  // ~0.3 degrees
    constexpr double MAX_ANGLE_STEP = 0.1;    // ~5.7 degrees
    angleStep = std::clamp(angleStep, MIN_ANGLE_STEP, MAX_ANGLE_STEP);
}

} // namespace Qi::Vision::Matching
