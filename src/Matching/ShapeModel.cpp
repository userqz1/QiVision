/**
 * @file ShapeModel.cpp
 * @brief Public API implementation for ShapeModel
 *
 * Performance: 86-407ms for 360° search on 640×512 images
 *
 * @note For optimization attempts and results, see:
 *       docs/design/ShapeModel_Optimization_Notes.md
 */

#include "ShapeModelImpl.h"
#include <QiVision/Platform/FileIO.h>

#include <algorithm>
#include <chrono>
#include <limits>

namespace Qi::Vision::Matching {

// =============================================================================
// ShapeModel Public API Implementation
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

// =============================================================================
// Model Creation
// =============================================================================

bool ShapeModel::Create(const QImage& templateImage, const ModelParams& params) {
    impl_->params_ = params;

    // Use LINEMOD mode if requested
    if (params.useLinemod) {
        return impl_->CreateModelLinemod(templateImage, Rect2i{}, Point2d{0, 0});
    }
    return impl_->CreateModel(templateImage, Rect2i{}, Point2d{0, 0});
}

bool ShapeModel::Create(const QImage& templateImage, const Rect2i& roi,
                         const ModelParams& params) {
    impl_->params_ = params;

    // Use LINEMOD mode if requested
    if (params.useLinemod) {
        return impl_->CreateModelLinemod(templateImage, roi, Point2d{0, 0});
    }
    return impl_->CreateModel(templateImage, roi, Point2d{0, 0});
}

bool ShapeModel::CreateWithOrigin(const QImage& templateImage, const Point2d& origin,
                                   const ModelParams& params) {
    impl_->params_ = params;

    // Use LINEMOD mode if requested
    if (params.useLinemod) {
        return impl_->CreateModelLinemod(templateImage, Rect2i{}, origin);
    }
    return impl_->CreateModel(templateImage, Rect2i{}, origin);
}

void ShapeModel::Clear() {
    impl_->levels_.clear();
    impl_->valid_ = false;
}

bool ShapeModel::IsValid() const {
    return impl_->valid_;
}

// =============================================================================
// Searching
// =============================================================================

std::vector<MatchResult> ShapeModel::Find(const QImage& image,
                                           const SearchParams& params) const {
    if (!impl_->valid_) {
        return {};
    }

    // Reset timing
    impl_->findTiming_ = ShapeModelFindTiming();
    auto tTotal = std::chrono::high_resolution_clock::now();

    std::vector<MatchResult> results;

    // Check if LINEMOD mode is enabled (has LINEMOD features)
    const bool useLinemod = impl_->params_.useLinemod && !impl_->linemodFeatures_.empty();

    if (useLinemod) {
        // LINEMOD search path
        Internal::LinemodPyramidParams linemodParams;
        linemodParams.numLevels = static_cast<int32_t>(impl_->linemodFeatures_.size());
        linemodParams.minMagnitude = static_cast<float>(impl_->params_.contrastHigh);
        linemodParams.spreadT = 4;
        linemodParams.neighborThreshold = 5;
        linemodParams.smoothSigma = 1.0;
        linemodParams.extractFeatures = false;  // Don't extract features for search image

        auto tPyramid = std::chrono::high_resolution_clock::now();
        Internal::LinemodPyramid targetPyramid;
        if (!targetPyramid.Build(image, linemodParams)) {
            return {};
        }

        if (impl_->timingParams_.enableTiming) {
            impl_->findTiming_.pyramidBuildMs = std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now() - tPyramid).count();
        }

        results = impl_->SearchPyramidLinemod(targetPyramid, params);
    } else {
        // Standard AnglePyramid search path
        Internal::AnglePyramidParams pyramidParams;
        pyramidParams.numLevels = static_cast<int32_t>(impl_->levels_.size());

        double searchContrast = (impl_->params_.minContrast > 0)
            ? impl_->params_.minContrast
            : impl_->params_.contrastHigh * 0.5;
        pyramidParams.minContrast = searchContrast;
        pyramidParams.smoothSigma = 0.5;
        pyramidParams.extractEdgePoints = false;

        auto tPyramid = std::chrono::high_resolution_clock::now();
        Internal::AnglePyramid targetPyramid;
        if (!targetPyramid.Build(image, pyramidParams)) {
            return {};
        }

        if (impl_->timingParams_.enableTiming) {
            impl_->findTiming_.pyramidBuildMs = std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now() - tPyramid).count();
        }

        results = impl_->SearchPyramid(targetPyramid, params);
    }

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

// =============================================================================
// Model Properties
// =============================================================================

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

// =============================================================================
// Timing
// =============================================================================

void ShapeModel::SetTimingParams(const ShapeModelTimingParams& params) {
    impl_->timingParams_ = params;
}

const ShapeModelCreateTiming& ShapeModel::GetCreateTiming() const {
    return impl_->createTiming_;
}

const ShapeModelFindTiming& ShapeModel::GetFindTiming() const {
    return impl_->findTiming_;
}

// =============================================================================
// Visualization
// =============================================================================

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

// =============================================================================
// Serialization
// =============================================================================

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
    const uint32_t MAGIC = 0x4D495351;  // "QISM"
    const uint32_t VERSION = 3;
    writer.Write(MAGIC);
    writer.Write(VERSION);

    // Model parameters - v3 format
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
    return true;
}

bool ShapeModel::Load(const std::string& filename) {
    using Platform::BinaryReader;
    BinaryReader reader(filename);
    if (!reader.IsOpen()) {
        return false;
    }

    const uint32_t MAGIC = 0x4D495351;
    uint32_t magic = reader.Read<uint32_t>();
    if (magic != MAGIC) {
        return false;
    }

    uint32_t version = reader.Read<uint32_t>();
    if (version < 1 || version > 3) {
        return false;
    }

    impl_ = std::make_unique<Internal::ShapeModelImpl>();

    if (version >= 3) {
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
        // v1/v2: Legacy format
        double minContrast = reader.Read<double>();
        double hysteresisContrast = 0.0;
        if (version >= 2) {
            hysteresisContrast = reader.Read<double>();
        }
        double maxContrast = reader.Read<double>();

        impl_->params_.contrastMode = ContrastMode::Manual;
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
        (void)reader.Read<int32_t>();  // maxModelPoints
        (void)reader.Read<double>();   // modelPointSpacing

        impl_->params_.optimization = optimizeModel
            ? OptimizationMode::Auto : OptimizationMode::None;

        impl_->params_.polarity = static_cast<MatchPolarity>(reader.Read<int32_t>());
        impl_->params_.metric = MetricMode::UsePolarity;
    }

    impl_->origin_.x = reader.Read<double>();
    impl_->origin_.y = reader.Read<double>();
    impl_->templateSize_.width = reader.Read<int32_t>();
    impl_->templateSize_.height = reader.Read<int32_t>();

    impl_->modelMinX_ = reader.Read<double>();
    impl_->modelMaxX_ = reader.Read<double>();
    impl_->modelMinY_ = reader.Read<double>();
    impl_->modelMaxY_ = reader.Read<double>();

    uint32_t numLevels = reader.Read<uint32_t>();
    impl_->levels_.resize(numLevels);

    for (uint32_t i = 0; i < numLevels; ++i) {
        auto& level = impl_->levels_[i];

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

    impl_->valid_ = true;
    return true;
}

// =============================================================================
// Advanced
// =============================================================================

double ShapeModel::ComputeScore(const QImage& image,
                                 double x, double y,
                                 double angle, double scale) const {
    if (!impl_->valid_) {
        return 0.0;
    }

    Internal::AnglePyramidParams params;
    params.numLevels = 1;

    double searchContrast = (impl_->params_.minContrast > 0)
        ? impl_->params_.minContrast
        : impl_->params_.contrastHigh * 0.5;
    params.minContrast = searchContrast;

    Internal::AnglePyramid pyramid;
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

    Internal::AnglePyramidParams params;
    params.numLevels = 1;

    double searchContrast = (impl_->params_.minContrast > 0)
        ? impl_->params_.minContrast
        : impl_->params_.contrastHigh * 0.5;
    params.minContrast = searchContrast;

    Internal::AnglePyramid pyramid;
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
    double maxRadius = modelSize / 2.0;
    if (maxRadius < 1.0) maxRadius = 1.0;

    double step = 2.0 * std::asin(1.0 / (2.0 * maxRadius));

    return std::clamp(step, 0.0087, 0.175);
}

} // namespace Qi::Vision::Matching
