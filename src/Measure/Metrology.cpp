/**
 * @file Metrology.cpp
 * @brief Implementation of Metrology module
 */

#include <QiVision/Measure/Metrology.h>
#include <QiVision/Measure/Caliper.h>
#include <QiVision/Internal/Fitting.h>
#include <QiVision/Internal/Profiler.h>
#include <QiVision/Internal/Edge1D.h>

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace Qi::Vision::Measure {

namespace {
    constexpr double PI = 3.14159265358979323846;
    constexpr double TWO_PI = 2.0 * PI;

    // Helper: Parse vector<int> params into MetrologyMeasureParams
    MetrologyMeasureParams ParseVectorParams(
        double measureLength1, double measureLength2,
        const std::string& transition, const std::string& select,
        const std::vector<int>& params)
    {
        MetrologyMeasureParams result;
        result.measureLength1 = measureLength1;
        result.measureLength2 = measureLength2;

        // Parse transition string
        if (transition == "positive" || transition == "Positive") {
            result.measureTransition = EdgeTransition::Positive;
        } else if (transition == "negative" || transition == "Negative") {
            result.measureTransition = EdgeTransition::Negative;
        } else {
            result.measureTransition = EdgeTransition::All;
        }

        // Parse select string
        if (select == "first" || select == "First") {
            result.measureSelect = EdgeSelectMode::First;
        } else if (select == "last" || select == "Last") {
            result.measureSelect = EdgeSelectMode::Last;
        } else if (select == "best" || select == "Best" || select == "strongest" || select == "Strongest") {
            result.measureSelect = EdgeSelectMode::Strongest;
        } else {
            result.measureSelect = EdgeSelectMode::All;
        }

        // Parse key-value pairs from vector<int>
        for (size_t i = 0; i + 1 < params.size(); i += 2) {
            int key = params[i];
            int value = params[i + 1];

            switch (key) {
                case METROLOGY_NUM_INSTANCES:
                    result.numInstances = value;
                    break;
                case METROLOGY_MEASURE_SIGMA:
                    result.measureSigma = value / 100.0;  // e.g., 150 -> 1.5
                    break;
                case METROLOGY_MEASURE_THRESHOLD:
                    result.measureThreshold = static_cast<double>(value);
                    break;
                case METROLOGY_NUM_MEASURES:
                    result.numMeasures = value;
                    break;
                case METROLOGY_MIN_SCORE:
                    result.minScore = value / 100.0;  // e.g., 70 -> 0.7
                    break;
                case METROLOGY_FIT_METHOD:
                    switch (value) {
                        case 0: result.fitMethod = MetrologyFitMethod::RANSAC; break;
                        case 1: result.fitMethod = MetrologyFitMethod::Huber; break;
                        case 2: result.fitMethod = MetrologyFitMethod::Tukey; break;
                    }
                    break;
                case METROLOGY_DISTANCE_THRESHOLD:
                    result.distanceThreshold = value / 100.0;  // e.g., 350 -> 3.5
                    break;
                case METROLOGY_MAX_ITERATIONS:
                    result.maxIterations = value;
                    break;
                case METROLOGY_RAND_SEED:
                    result.randSeed = value;
                    break;
                case METROLOGY_THRESHOLD_MODE:
                    result.thresholdMode = (value == 0) ? ThresholdMode::Manual : ThresholdMode::Auto;
                    break;
            }
        }

        return result;
    }

    // Normalize angle to [0, 2*PI) - may be used for arc handling
    [[maybe_unused]] double NormalizeAngle(double angle) {
        while (angle < 0.0) angle += TWO_PI;
        while (angle >= TWO_PI) angle -= TWO_PI;
        return angle;
    }

    // Rotate point around origin
    Point2d RotatePoint(const Point2d& p, double phi, const Point2d& center) {
        double cosPhi = std::cos(phi);
        double sinPhi = std::sin(phi);
        double dx = p.x - center.x;
        double dy = p.y - center.y;
        return {
            center.x + dx * cosPhi - dy * sinPhi,
            center.y + dx * sinPhi + dy * cosPhi
        };
    }
}

// =============================================================================
// MetrologyObjectLine Implementation
// =============================================================================

MetrologyObjectLine::MetrologyObjectLine(double row1, double col1,
                                          double row2, double col2,
                                          const MetrologyMeasureParams& params)
    : row1_(row1), col1_(col1), row2_(row2), col2_(col2) {
    params_ = params;
}

double MetrologyObjectLine::Length() const {
    double dx = col2_ - col1_;
    double dy = row2_ - row1_;
    return std::sqrt(dx * dx + dy * dy);
}

double MetrologyObjectLine::Angle() const {
    return std::atan2(row2_ - row1_, col2_ - col1_);
}

std::vector<MeasureRectangle2> MetrologyObjectLine::GetCalipers() const {
    std::vector<MeasureRectangle2> calipers;
    int32_t numMeasures = params_.numMeasures;
    if (numMeasures < 1) numMeasures = 1;

    double length = Length();
    if (length < 1e-6) return calipers;

    // Line angle (perpendicular to profile direction)
    double lineAngle = Angle();
    // Profile is perpendicular to the line direction
    double profilePhi = lineAngle;  // Phi for rectangle2 (perpendicular to profile)

    // Distribute calipers evenly along the line
    for (int32_t i = 0; i < numMeasures; ++i) {
        double t = (numMeasures == 1) ? 0.5 : static_cast<double>(i) / (numMeasures - 1);
        double row = row1_ + t * (row2_ - row1_);
        double col = col1_ + t * (col2_ - col1_);

        MeasureRectangle2 caliper(row, col, profilePhi,
                                   params_.measureLength1, params_.measureLength2);
        calipers.push_back(caliper);
    }

    return calipers;
}

QContour MetrologyObjectLine::GetContour() const {
    QContour contour;
    contour.AddPoint(Point2d{col1_, row1_});
    contour.AddPoint(Point2d{col2_, row2_});
    return contour;
}

void MetrologyObjectLine::Transform(double rowOffset, double colOffset, double phi) {
    if (std::abs(phi) > 1e-9) {
        // Rotate around the line center
        double centerRow = (row1_ + row2_) * 0.5;
        double centerCol = (col1_ + col2_) * 0.5;

        Point2d p1 = RotatePoint({col1_, row1_}, phi, {centerCol, centerRow});
        Point2d p2 = RotatePoint({col2_, row2_}, phi, {centerCol, centerRow});

        row1_ = p1.y;
        col1_ = p1.x;
        row2_ = p2.y;
        col2_ = p2.x;
    }

    row1_ += rowOffset;
    col1_ += colOffset;
    row2_ += rowOffset;
    col2_ += colOffset;
}

// =============================================================================
// MetrologyObjectCircle Implementation
// =============================================================================

MetrologyObjectCircle::MetrologyObjectCircle(double row, double column, double radius,
                                              const MetrologyMeasureParams& params)
    : row_(row), column_(column), radius_(radius),
      angleStart_(0.0), angleEnd_(TWO_PI) {
    params_ = params;
}

MetrologyObjectCircle::MetrologyObjectCircle(double row, double column, double radius,
                                              double angleStart, double angleEnd,
                                              const MetrologyMeasureParams& params)
    : row_(row), column_(column), radius_(radius),
      angleStart_(angleStart), angleEnd_(angleEnd) {
    params_ = params;
}

bool MetrologyObjectCircle::IsFullCircle() const {
    return std::abs(angleEnd_ - angleStart_ - TWO_PI) < 1e-6 ||
           std::abs(angleEnd_ - angleStart_ + TWO_PI) < 1e-6;
}

std::vector<MeasureRectangle2> MetrologyObjectCircle::GetCalipers() const {
    std::vector<MeasureRectangle2> calipers;
    int32_t numMeasures = params_.numMeasures;
    if (numMeasures < 1) numMeasures = 1;

    double angleExtent = angleEnd_ - angleStart_;
    bool fullCircle = IsFullCircle();

    for (int32_t i = 0; i < numMeasures; ++i) {
        double t;
        if (fullCircle) {
            t = static_cast<double>(i) / numMeasures;  // Don't overlap at start/end
        } else {
            t = (numMeasures == 1) ? 0.5 : static_cast<double>(i) / (numMeasures - 1);
        }

        double angle = angleStart_ + t * angleExtent;

        // Position on circle
        double row = row_ + radius_ * std::sin(angle);
        double col = column_ + radius_ * std::cos(angle);

        // MeasureRectangle2::ProfileAngle() returns phi + PI/2
        // To get radial search direction, we need phi = angle - PI/2
        double profilePhi = angle - PI / 2.0;  // Radial direction after ProfileAngle transform

        MeasureRectangle2 caliper(row, col, profilePhi,
                                   params_.measureLength1, params_.measureLength2);
        calipers.push_back(caliper);
    }

    return calipers;
}

QContour MetrologyObjectCircle::GetContour() const {
    QContour contour;
    int32_t numPoints = std::max(32, static_cast<int32_t>(radius_ * 0.5));
    double angleExtent = angleEnd_ - angleStart_;

    for (int32_t i = 0; i <= numPoints; ++i) {
        double t = static_cast<double>(i) / numPoints;
        double angle = angleStart_ + t * angleExtent;
        double x = column_ + radius_ * std::cos(angle);
        double y = row_ + radius_ * std::sin(angle);
        contour.AddPoint(Point2d{x, y});
    }

    return contour;
}

void MetrologyObjectCircle::Transform(double rowOffset, double colOffset, double phi) {
    if (std::abs(phi) > 1e-9) {
        // Rotate angles
        angleStart_ += phi;
        angleEnd_ += phi;
    }

    row_ += rowOffset;
    column_ += colOffset;
}

// =============================================================================
// MetrologyObjectEllipse Implementation
// =============================================================================

MetrologyObjectEllipse::MetrologyObjectEllipse(double row, double column, double phi,
                                                double ra, double rb,
                                                const MetrologyMeasureParams& params)
    : row_(row), column_(column), phi_(phi), ra_(ra), rb_(rb) {
    params_ = params;
}

std::vector<MeasureRectangle2> MetrologyObjectEllipse::GetCalipers() const {
    std::vector<MeasureRectangle2> calipers;
    int32_t numMeasures = params_.numMeasures;
    if (numMeasures < 1) numMeasures = 1;

    double cosPhi = std::cos(phi_);
    double sinPhi = std::sin(phi_);

    for (int32_t i = 0; i < numMeasures; ++i) {
        double t = static_cast<double>(i) / numMeasures;  // Full ellipse
        double angle = t * TWO_PI;

        // Point on ellipse (local coordinates)
        double localX = ra_ * std::cos(angle);
        double localY = rb_ * std::sin(angle);

        // Rotate by phi and translate
        double col = column_ + localX * cosPhi - localY * sinPhi;
        double row = row_ + localX * sinPhi + localY * cosPhi;

        // Normal direction at this point (gradient of ellipse equation)
        // For ellipse: (x/ra)^2 + (y/rb)^2 = 1
        // Gradient: (2x/ra^2, 2y/rb^2) -> normal angle
        double gradX = localX / (ra_ * ra_);
        double gradY = localY / (rb_ * rb_);

        // Rotate gradient by phi
        double normalX = gradX * cosPhi - gradY * sinPhi;
        double normalY = gradX * sinPhi + gradY * cosPhi;
        double profilePhi = std::atan2(normalY, normalX);

        MeasureRectangle2 caliper(row, col, profilePhi,
                                   params_.measureLength1, params_.measureLength2);
        calipers.push_back(caliper);
    }

    return calipers;
}

QContour MetrologyObjectEllipse::GetContour() const {
    QContour contour;
    int32_t numPoints = std::max(32, static_cast<int32_t>((ra_ + rb_) * 0.5));

    double cosPhi = std::cos(phi_);
    double sinPhi = std::sin(phi_);

    for (int32_t i = 0; i <= numPoints; ++i) {
        double t = static_cast<double>(i) / numPoints;
        double angle = t * TWO_PI;

        double localX = ra_ * std::cos(angle);
        double localY = rb_ * std::sin(angle);

        double x = column_ + localX * cosPhi - localY * sinPhi;
        double y = row_ + localX * sinPhi + localY * cosPhi;
        contour.AddPoint(Point2d{x, y});
    }

    return contour;
}

void MetrologyObjectEllipse::Transform(double rowOffset, double colOffset, double phi) {
    if (std::abs(phi) > 1e-9) {
        phi_ += phi;
    }

    row_ += rowOffset;
    column_ += colOffset;
}

// =============================================================================
// MetrologyObjectRectangle2 Implementation
// =============================================================================

MetrologyObjectRectangle2::MetrologyObjectRectangle2(double row, double column, double phi,
                                                       double length1, double length2,
                                                       const MetrologyMeasureParams& params)
    : row_(row), column_(column), phi_(phi), length1_(length1), length2_(length2) {
    params_ = params;
}

std::vector<MeasureRectangle2> MetrologyObjectRectangle2::GetCalipers() const {
    std::vector<MeasureRectangle2> calipers;
    int32_t numMeasures = params_.numMeasures;
    if (numMeasures < 1) numMeasures = 4;  // At least one per side

    double cosPhi = std::cos(phi_);
    double sinPhi = std::sin(phi_);

    // Distribute calipers along 4 sides
    int32_t perSide = std::max(1, numMeasures / 4);

    // Corners (local coordinates)
    std::vector<std::pair<Point2d, Point2d>> sides = {
        // Side 1: top (from corner to corner)
        {{-length1_, -length2_}, {length1_, -length2_}},
        // Side 2: right
        {{length1_, -length2_}, {length1_, length2_}},
        // Side 3: bottom
        {{length1_, length2_}, {-length1_, length2_}},
        // Side 4: left
        {{-length1_, length2_}, {-length1_, -length2_}}
    };

    // Edge directions for each side (caliper phi should be edge direction,
    // so profile direction = phi + 90° is perpendicular to edge)
    std::vector<double> edgeAngles = {
        phi_,              // Top: horizontal edge, profile perpendicular (vertical)
        phi_ + PI * 0.5,   // Right: vertical edge, profile perpendicular (horizontal)
        phi_,              // Bottom: horizontal edge, profile perpendicular (vertical)
        phi_ + PI * 0.5    // Left: vertical edge, profile perpendicular (horizontal)
    };

    for (size_t side = 0; side < 4; ++side) {
        auto& s = sides[side];

        for (int32_t i = 0; i < perSide; ++i) {
            double t = (perSide == 1) ? 0.5 : static_cast<double>(i) / (perSide - 1);

            // Interpolate along side
            double localX = s.first.x + t * (s.second.x - s.first.x);
            double localY = s.first.y + t * (s.second.y - s.first.y);

            // Transform to image coordinates
            double col = column_ + localX * cosPhi - localY * sinPhi;
            double row = row_ + localX * sinPhi + localY * cosPhi;

            MeasureRectangle2 caliper(row, col, edgeAngles[side],
                                       params_.measureLength1, params_.measureLength2);
            calipers.push_back(caliper);
        }
    }

    return calipers;
}

QContour MetrologyObjectRectangle2::GetContour() const {
    QContour contour;

    double cosPhi = std::cos(phi_);
    double sinPhi = std::sin(phi_);

    // 4 corners
    std::vector<std::pair<double, double>> corners = {
        {-length1_, -length2_},
        {length1_, -length2_},
        {length1_, length2_},
        {-length1_, length2_},
        {-length1_, -length2_}  // Close the contour
    };

    for (auto& c : corners) {
        double x = column_ + c.first * cosPhi - c.second * sinPhi;
        double y = row_ + c.first * sinPhi + c.second * cosPhi;
        contour.AddPoint(Point2d{x, y});
    }

    return contour;
}

void MetrologyObjectRectangle2::Transform(double rowOffset, double colOffset, double phi) {
    if (std::abs(phi) > 1e-9) {
        phi_ += phi;
    }

    row_ += rowOffset;
    column_ += colOffset;
}

// =============================================================================
// MetrologyModel Implementation
// =============================================================================

struct MetrologyModel::Impl {
    std::vector<std::unique_ptr<MetrologyObject>> objects;
    MetrologyMeasureParams defaultParams;

    // Per-object results
    std::unordered_map<int32_t, std::vector<MetrologyLineResult>> lineResults;
    std::unordered_map<int32_t, std::vector<MetrologyCircleResult>> circleResults;
    std::unordered_map<int32_t, std::vector<MetrologyEllipseResult>> ellipseResults;
    std::unordered_map<int32_t, std::vector<MetrologyRectangle2Result>> rectangleResults;

    // Per-object measured points
    std::unordered_map<int32_t, std::vector<Point2d>> measuredPoints;

    // Per-object point weights (from robust fitting, e.g., Huber/Tukey)
    std::unordered_map<int32_t, std::vector<double>> pointWeights;

    // Alignment state
    double alignRow = 0.0;
    double alignCol = 0.0;
    double alignPhi = 0.0;

    void ClearResults() {
        lineResults.clear();
        circleResults.clear();
        ellipseResults.clear();
        rectangleResults.clear();
        measuredPoints.clear();
        pointWeights.clear();
    }
};

MetrologyModel::MetrologyModel() : impl_(std::make_unique<Impl>()) {}

MetrologyModel::~MetrologyModel() = default;

MetrologyModel::MetrologyModel(MetrologyModel&&) noexcept = default;
MetrologyModel& MetrologyModel::operator=(MetrologyModel&&) noexcept = default;

int32_t MetrologyModel::AddLineMeasure(double row1, double col1,
                                         double row2, double col2,
                                         const MetrologyMeasureParams& params) {
    auto obj = std::make_unique<MetrologyObjectLine>(row1, col1, row2, col2, params);
    int32_t idx = static_cast<int32_t>(impl_->objects.size());
    obj->index_ = idx;
    impl_->objects.push_back(std::move(obj));
    return idx;
}

int32_t MetrologyModel::AddLineMeasure(double row1, double col1,
                                         double row2, double col2,
                                         double measureLength1, double measureLength2,
                                         const std::string& transition,
                                         const std::string& select,
                                         const std::vector<int>& params) {
    auto parsed = ParseVectorParams(measureLength1, measureLength2, transition, select, params);
    return AddLineMeasure(row1, col1, row2, col2, parsed);
}

int32_t MetrologyModel::AddCircleMeasure(double row, double column, double radius,
                                          const MetrologyMeasureParams& params) {
    auto obj = std::make_unique<MetrologyObjectCircle>(row, column, radius, params);
    int32_t idx = static_cast<int32_t>(impl_->objects.size());
    obj->index_ = idx;
    impl_->objects.push_back(std::move(obj));
    return idx;
}

int32_t MetrologyModel::AddCircleMeasure(double row, double column, double radius,
                                          double measureLength1, double measureLength2,
                                          const std::string& transition,
                                          const std::string& select,
                                          const std::vector<int>& params) {
    auto parsed = ParseVectorParams(measureLength1, measureLength2, transition, select, params);
    return AddCircleMeasure(row, column, radius, parsed);
}

int32_t MetrologyModel::AddArcMeasure(double row, double column, double radius,
                                        double angleStart, double angleEnd,
                                        const MetrologyMeasureParams& params) {
    auto obj = std::make_unique<MetrologyObjectCircle>(row, column, radius,
                                                         angleStart, angleEnd, params);
    int32_t idx = static_cast<int32_t>(impl_->objects.size());
    obj->index_ = idx;
    impl_->objects.push_back(std::move(obj));
    return idx;
}

int32_t MetrologyModel::AddArcMeasure(double row, double column, double radius,
                                        double angleStart, double angleEnd,
                                        double measureLength1, double measureLength2,
                                        const std::string& transition,
                                        const std::string& select,
                                        const std::vector<int>& params) {
    auto parsed = ParseVectorParams(measureLength1, measureLength2, transition, select, params);
    return AddArcMeasure(row, column, radius, angleStart, angleEnd, parsed);
}

int32_t MetrologyModel::AddEllipseMeasure(double row, double column, double phi,
                                           double ra, double rb,
                                           const MetrologyMeasureParams& params) {
    auto obj = std::make_unique<MetrologyObjectEllipse>(row, column, phi, ra, rb, params);
    int32_t idx = static_cast<int32_t>(impl_->objects.size());
    obj->index_ = idx;
    impl_->objects.push_back(std::move(obj));
    return idx;
}

int32_t MetrologyModel::AddEllipseMeasure(double row, double column, double phi,
                                           double ra, double rb,
                                           double measureLength1, double measureLength2,
                                           const std::string& transition,
                                           const std::string& select,
                                           const std::vector<int>& params) {
    auto parsed = ParseVectorParams(measureLength1, measureLength2, transition, select, params);
    return AddEllipseMeasure(row, column, phi, ra, rb, parsed);
}

int32_t MetrologyModel::AddRectangle2Measure(double row, double column, double phi,
                                               double length1, double length2,
                                               const MetrologyMeasureParams& params) {
    auto obj = std::make_unique<MetrologyObjectRectangle2>(row, column, phi, length1, length2, params);
    int32_t idx = static_cast<int32_t>(impl_->objects.size());
    obj->index_ = idx;
    impl_->objects.push_back(std::move(obj));
    return idx;
}

int32_t MetrologyModel::AddRectangle2Measure(double row, double column, double phi,
                                               double length1, double length2,
                                               double measureLength1, double measureLength2,
                                               const std::string& transition,
                                               const std::string& select,
                                               const std::vector<int>& params) {
    auto parsed = ParseVectorParams(measureLength1, measureLength2, transition, select, params);
    return AddRectangle2Measure(row, column, phi, length1, length2, parsed);
}

void MetrologyModel::ClearObject(int32_t index) {
    if (index >= 0 && index < static_cast<int32_t>(impl_->objects.size())) {
        impl_->objects[index].reset();
    }
}

void MetrologyModel::ClearAll() {
    impl_->objects.clear();
    impl_->ClearResults();
}

int32_t MetrologyModel::NumObjects() const {
    return static_cast<int32_t>(impl_->objects.size());
}

const MetrologyObject* MetrologyModel::GetObject(int32_t index) const {
    if (index >= 0 && index < static_cast<int32_t>(impl_->objects.size())) {
        return impl_->objects[index].get();
    }
    return nullptr;
}

bool MetrologyModel::Apply(const QImage& image) {
    if (image.Empty()) return false;

    impl_->ClearResults();

    // Convert to grayscale if needed (for Hough detection)
    QImage grayImage = image;
    if (image.Channels() > 1) {
        grayImage = image.ToGray();
    }

    for (auto& objPtr : impl_->objects) {
        if (!objPtr) continue;

        auto& obj = *objPtr;
        int32_t idx = obj.Index();

        // Get calipers
        auto calipers = obj.GetCalipers();

        // Measure edge positions
        std::vector<Point2d> edgePoints;
        double sigma = obj.Params().measureSigma;
        double userThreshold = obj.Params().measureThreshold;
        ThresholdMode thresholdMode = obj.Params().thresholdMode;

        int caliperIdx = 0;
        for (auto& caliper : calipers) {
            // Extract profile directly for adaptive threshold calculation
            Internal::RectProfileParams profParams;
            profParams.centerX = caliper.Column();
            profParams.centerY = caliper.Row();
            profParams.length = caliper.ProfileLength();
            profParams.width = 2.0 * caliper.Length2();
            profParams.angle = caliper.ProfileAngle();
            profParams.numLines = caliper.NumLines();
            profParams.samplesPerPixel = caliper.SamplesPerPixel();
            profParams.interp = Internal::InterpolationMethod::Bilinear;
            profParams.method = Internal::ProfileMethod::Average;

            auto profile = Internal::ExtractRectProfile(image, profParams);
            if (profile.data.size() < 3) continue;

            // Determine threshold for edge detection
            // Each profile region computes its own threshold independently
            double threshold;
            if (thresholdMode == ThresholdMode::Manual) {
                // User specified threshold - use directly
                threshold = userThreshold;
            } else {
                // Auto threshold mode: compute based on this profile's statistics
                const auto& data = profile.data;
                size_t n = data.size();

                // 1. Compute contrast (max - min)
                double minVal = *std::min_element(data.begin(), data.end());
                double maxVal = *std::max_element(data.begin(), data.end());
                double contrast = maxVal - minVal;

                // 2. Compute gradient (central difference)
                std::vector<double> gradient(n);
                gradient[0] = data[1] - data[0];
                for (size_t i = 1; i < n - 1; ++i) {
                    gradient[i] = (data[i + 1] - data[i - 1]) * 0.5;
                }
                gradient[n - 1] = data[n - 1] - data[n - 2];

                // 3. Estimate noise using MAD (Median Absolute Deviation)
                //    More robust than standard deviation
                std::vector<double> absGrad(n);
                for (size_t i = 0; i < n; ++i) {
                    absGrad[i] = std::abs(gradient[i]);
                }
                std::nth_element(absGrad.begin(), absGrad.begin() + n / 2, absGrad.end());
                double medianAbsGrad = absGrad[n / 2];
                double noiseSigma = medianAbsGrad / 0.6745;  // MAD to sigma conversion

                // 4. Compute threshold = max(base, contrast*ratio, k*noise)
                constexpr double BASE_THRESHOLD = 5.0;      // Absolute minimum
                constexpr double CONTRAST_RATIO = 0.2;      // 20% of contrast
                constexpr double NOISE_MULTIPLIER = 4.0;    // 4 sigma confidence

                double contrastThreshold = contrast * CONTRAST_RATIO;
                double noiseThreshold = noiseSigma * NOISE_MULTIPLIER;
                threshold = std::max({BASE_THRESHOLD, contrastThreshold, noiseThreshold});
            }

            // Detect edges with threshold
            auto edges1D = Internal::DetectEdges1D(
                profile.data.data(),
                profile.data.size(),
                threshold,
                Internal::EdgePolarity::Both,
                sigma
            );

            if (edges1D.empty()) {
                caliperIdx++;
                continue;
            }

            // Find the strongest edge
            auto strongest = std::max_element(edges1D.begin(), edges1D.end(),
                [](const auto& a, const auto& b) {
                    return std::abs(a.amplitude) < std::abs(b.amplitude);
                });

            // Convert profile position to image coordinates
            double profileLength = caliper.ProfileLength();
            double stepSize = profileLength / (profile.data.size() - 1);
            double profilePos = strongest->position * stepSize;

            // ProfileToImage calculation
            double profileAngle = caliper.ProfileAngle();
            double halfLen = caliper.Length1();
            double startX = caliper.Column() - halfLen * std::cos(profileAngle);
            double startY = caliper.Row() - halfLen * std::sin(profileAngle);

            double edgeX = startX + profilePos * std::cos(profileAngle);
            double edgeY = startY + profilePos * std::sin(profileAngle);

            edgePoints.push_back({edgeX, edgeY});
            caliperIdx++;
        }

        impl_->measuredPoints[idx] = edgePoints;

        // Get fitting parameters
        auto fitMethod = obj.Params().fitMethod;
        double distThreshold = obj.Params().distanceThreshold;
        int32_t maxIter = obj.Params().maxIterations;

        // Setup RANSAC parameters
        Internal::RansacParams ransacParams;
        ransacParams.threshold = distThreshold;
        ransacParams.maxIterations = (maxIter < 0) ? 10000 : maxIter;  // -1 means high limit
        ransacParams.confidence = 0.99;

        // Setup fit params
        Internal::FitParams fitParams;
        fitParams.computeResiduals = true;
        fitParams.computeInlierMask = true;  // For outlier visualization

        // Fit geometric primitive based on object type
        switch (obj.Type()) {
            case MetrologyObjectType::Line: {
                MetrologyLineResult result;
                if (edgePoints.size() >= 2) {
                    Internal::LineFitResult fitResult;

                    // Select fitting method based on params
                    switch (fitMethod) {
                        case MetrologyFitMethod::RANSAC:
                            fitResult = Internal::FitLineRANSAC(edgePoints, ransacParams, fitParams);
                            break;
                        case MetrologyFitMethod::Huber:
                            fitResult = Internal::FitLineHuber(edgePoints, 0.0, fitParams);
                            break;
                        case MetrologyFitMethod::Tukey:
                            fitResult = Internal::FitLineTukey(edgePoints, 0.0, fitParams);
                            break;
                    }

                    if (fitResult.success) {
                        // Get line endpoints from fitted line
                        auto& line = fitResult.line;

                        // Project original endpoints onto fitted line
                        auto* lineObj = static_cast<MetrologyObjectLine*>(&obj);
                        Point2d p1 = {lineObj->Col1(), lineObj->Row1()};
                        Point2d p2 = {lineObj->Col2(), lineObj->Row2()};

                        // Project points onto line
                        auto project = [&line](const Point2d& p) -> Point2d {
                            double d = line.a * p.x + line.b * p.y + line.c;
                            return {p.x - d * line.a, p.y - d * line.b};
                        };

                        Point2d proj1 = project(p1);
                        Point2d proj2 = project(p2);

                        result.row1 = proj1.y;
                        result.col1 = proj1.x;
                        result.row2 = proj2.y;
                        result.col2 = proj2.x;
                        result.nr = line.b;  // Normal row component
                        result.nc = line.a;  // Normal column component
                        result.dist = -line.c;  // Distance from origin
                        result.numUsed = fitResult.numInliers > 0 ? fitResult.numInliers : static_cast<int>(edgePoints.size());
                        result.rmsError = fitResult.residualRMS;
                        result.score = result.numUsed > 0 ? 1.0 / (1.0 + result.rmsError) : 0.0;

                        // Store weights for visualization
                        if (!fitResult.weights.empty()) {
                            impl_->pointWeights[idx] = fitResult.weights;
                        }
                    }
                }
                impl_->lineResults[idx].push_back(result);
                break;
            }

            case MetrologyObjectType::Circle: {
                MetrologyCircleResult result;
                if (edgePoints.size() >= 3) {
                    Internal::CircleFitResult fitResult;

                    // Select fitting method based on params
                    switch (fitMethod) {
                        case MetrologyFitMethod::RANSAC:
                            fitResult = Internal::FitCircleRANSAC(edgePoints, ransacParams, fitParams);
                            break;
                        case MetrologyFitMethod::Huber:
                            fitResult = Internal::FitCircleHuber(edgePoints, true, 0.0, fitParams);
                            break;
                        case MetrologyFitMethod::Tukey:
                            fitResult = Internal::FitCircleTukey(edgePoints, true, 0.0, fitParams);
                            break;
                    }

                    if (fitResult.success) {
                        result.row = fitResult.circle.center.y;
                        result.column = fitResult.circle.center.x;
                        result.radius = fitResult.circle.radius;
                        result.numUsed = fitResult.numInliers;
                        result.rmsError = fitResult.residualRMS;
                        result.score = result.numUsed > 0 ? 1.0 / (1.0 + result.rmsError) : 0.0;

                        // Get angle range from original object
                        auto* circleObj = static_cast<MetrologyObjectCircle*>(&obj);
                        result.startAngle = circleObj->AngleStart();
                        result.endAngle = circleObj->AngleEnd();

                        // Save weights for outlier visualization
                        if (!fitResult.weights.empty()) {
                            impl_->pointWeights[idx] = fitResult.weights;
                        } else if (!fitResult.inlierMask.empty()) {
                            // Convert inlier mask to weights for RANSAC
                            std::vector<double> weights(fitResult.inlierMask.size());
                            for (size_t i = 0; i < fitResult.inlierMask.size(); ++i) {
                                weights[i] = fitResult.inlierMask[i] ? 1.0 : 0.0;
                            }
                            impl_->pointWeights[idx] = weights;
                        }
                    }
                }
                impl_->circleResults[idx].push_back(result);
                break;
            }

            case MetrologyObjectType::Ellipse: {
                MetrologyEllipseResult result;
                if (edgePoints.size() >= 5) {
                    Internal::EllipseFitResult fitResult;

                    // Select fitting method based on params
                    switch (fitMethod) {
                        case MetrologyFitMethod::RANSAC:
                            fitResult = Internal::FitEllipseRANSAC(edgePoints, ransacParams, fitParams);
                            break;
                        case MetrologyFitMethod::Huber:
                            fitResult = Internal::FitEllipseHuber(edgePoints, 0.0, fitParams);
                            break;
                        case MetrologyFitMethod::Tukey:
                            fitResult = Internal::FitEllipseTukey(edgePoints, 0.0, fitParams);
                            break;
                    }

                    if (fitResult.success) {
                        result.row = fitResult.ellipse.center.y;
                        result.column = fitResult.ellipse.center.x;
                        result.phi = fitResult.ellipse.angle;
                        result.ra = fitResult.ellipse.a;
                        result.rb = fitResult.ellipse.b;
                        result.numUsed = fitResult.numInliers > 0 ? fitResult.numInliers : static_cast<int>(edgePoints.size());
                        result.rmsError = fitResult.residualRMS;
                        result.score = result.numUsed > 0 ? 1.0 / (1.0 + result.rmsError) : 0.0;

                        // Store weights for GetPointWeights
                        if (!fitResult.weights.empty()) {
                            impl_->pointWeights[idx] = fitResult.weights;
                        } else if (!fitResult.inlierMask.empty()) {
                            std::vector<double> weights(fitResult.inlierMask.size());
                            for (size_t i = 0; i < fitResult.inlierMask.size(); ++i) {
                                weights[i] = fitResult.inlierMask[i] ? 1.0 : 0.0;
                            }
                            impl_->pointWeights[idx] = weights;
                        }
                    }
                }
                impl_->ellipseResults[idx].push_back(result);
                break;
            }

            case MetrologyObjectType::Rectangle2: {
                MetrologyRectangle2Result result;
                if (edgePoints.size() >= 8) {  // At least 2 points per side
                    auto* rectObj = static_cast<MetrologyObjectRectangle2*>(&obj);

                    // Create initial rectangle estimate from object parameters
                    RotatedRect2d initialRect;
                    initialRect.center = {rectObj->Column(), rectObj->Row()};
                    initialRect.width = rectObj->Length1() * 2.0;   // full width
                    initialRect.height = rectObj->Length2() * 2.0;  // full height
                    initialRect.angle = rectObj->Phi();

                    // Get fitting parameters
                    auto fitMethod = obj.Params().fitMethod;
                    double distThreshold = obj.Params().distanceThreshold;
                    int32_t maxIter = obj.Params().maxIterations;

                    Internal::RectangleFitResult fitResult;
                    Internal::FitParams fitParams;
                    fitParams.SetComputeResiduals(true);
                    fitParams.computeInlierMask = true;  // For outlier visualization

                    if (fitMethod == MetrologyFitMethod::RANSAC) {
                        // For RANSAC: segment points by side, fit each with RANSAC, compute rectangle
                        auto sidedPoints = Internal::SegmentPointsByRectangleSide(edgePoints, initialRect);

                        std::array<Internal::LineFitResult, 4> sideResults;
                        std::array<Line2d, 4> fittedLines;
                        bool allSidesOK = true;
                        size_t totalInliers = 0;
                        double totalResidual = 0.0;
                        int validSides = 0;

                        // Setup RANSAC parameters
                        Internal::RansacParams ransacParams;
                        ransacParams.threshold = distThreshold;
                        ransacParams.maxIterations = (maxIter < 0) ? 1000 : maxIter;

                        for (int side = 0; side < 4; ++side) {
                            if (sidedPoints[side].size() >= 2) {
                                sideResults[side] = Internal::FitLineRANSAC(
                                    sidedPoints[side], ransacParams, fitParams);

                                if (sideResults[side].success) {
                                    fittedLines[side] = sideResults[side].line;
                                    totalInliers += sideResults[side].numInliers;
                                    totalResidual += sideResults[side].residualRMS * sideResults[side].numInliers;
                                    validSides++;
                                } else {
                                    allSidesOK = false;
                                }
                            } else {
                                allSidesOK = false;
                            }
                        }

                        if (allSidesOK && validSides == 4) {
                            auto rectOpt = Internal::RectangleFromLines(fittedLines);
                            if (rectOpt.has_value()) {
                                fitResult.success = true;
                                fitResult.rect = rectOpt.value();
                                fitResult.numInliers = totalInliers;
                                fitResult.residualRMS = totalInliers > 0 ? totalResidual / totalInliers : 0.0;
                                fitResult.sideResults = sideResults;

                                // Build weights: iterate through sidedPoints and map back to edgePoints
                                std::vector<double> weights(edgePoints.size(), 0.0);
                                for (int side = 0; side < 4; ++side) {
                                    const auto& sidePoints = sidedPoints[side];
                                    const auto& mask = sideResults[side].inlierMask;
                                    for (size_t j = 0; j < sidePoints.size(); ++j) {
                                        const auto& pt = sidePoints[j];
                                        // Find this point in edgePoints
                                        for (size_t i = 0; i < edgePoints.size(); ++i) {
                                            if (std::abs(edgePoints[i].x - pt.x) < 1e-6 &&
                                                std::abs(edgePoints[i].y - pt.y) < 1e-6) {
                                                weights[i] = (j < mask.size() && mask[j]) ? 1.0 : 0.0;
                                                break;
                                            }
                                        }
                                    }
                                }
                                impl_->pointWeights[idx] = weights;
                            }
                        }
                    } else {
                        // Use Huber or Tukey with iterative rectangle fitting
                        fitResult = Internal::FitRectangleIterative(
                            edgePoints, initialRect, 10, 0.01, fitParams
                        );

                        // Apply distance threshold as post-filter for consistency
                        if (fitResult.success && distThreshold > 0) {
                            // Count inliers based on distance threshold
                            size_t inlierCount = 0;
                            for (const auto& pt : edgePoints) {
                                double dist = std::numeric_limits<double>::max();
                                // Compute distance to nearest rectangle edge
                                for (int side = 0; side < 4; ++side) {
                                    if (fitResult.sideResults[side].success) {
                                        double d = fitResult.sideResults[side].line.Distance(pt);
                                        dist = std::min(dist, d);
                                    }
                                }
                                if (dist <= distThreshold) {
                                    inlierCount++;
                                }
                            }
                            fitResult.numInliers = inlierCount;
                        }
                    }

                    if (fitResult.success) {
                        result.row = fitResult.rect.center.y;
                        result.column = fitResult.rect.center.x;
                        result.phi = fitResult.rect.angle;
                        result.length1 = fitResult.rect.width / 2.0;   // half-length
                        result.length2 = fitResult.rect.height / 2.0;  // half-length
                        result.numUsed = static_cast<int32_t>(fitResult.numInliers);
                        result.rmsError = fitResult.residualRMS;
                        result.score = result.numUsed > 0 ? 1.0 / (1.0 + result.rmsError) : 0.0;
                    } else {
                        // Fallback to original parameters
                        result.row = rectObj->Row();
                        result.column = rectObj->Column();
                        result.phi = rectObj->Phi();
                        result.length1 = rectObj->Length1();
                        result.length2 = rectObj->Length2();
                        result.numUsed = static_cast<int32_t>(edgePoints.size());
                        result.score = 0.5;  // Lower confidence for fallback
                    }
                } else if (edgePoints.size() >= 4) {
                    // Not enough points for robust fitting, use original
                    auto* rectObj = static_cast<MetrologyObjectRectangle2*>(&obj);
                    result.row = rectObj->Row();
                    result.column = rectObj->Column();
                    result.phi = rectObj->Phi();
                    result.length1 = rectObj->Length1();
                    result.length2 = rectObj->Length2();
                    result.numUsed = static_cast<int32_t>(edgePoints.size());
                    result.score = 0.3;  // Low confidence
                }
                impl_->rectangleResults[idx].push_back(result);
                break;
            }
        }
    }

    return true;
}

MetrologyLineResult MetrologyModel::GetLineResult(int32_t index, int32_t instanceIndex) const {
    auto it = impl_->lineResults.find(index);
    if (it != impl_->lineResults.end() && instanceIndex < static_cast<int32_t>(it->second.size())) {
        return it->second[instanceIndex];
    }
    return MetrologyLineResult();
}

MetrologyCircleResult MetrologyModel::GetCircleResult(int32_t index, int32_t instanceIndex) const {
    auto it = impl_->circleResults.find(index);
    if (it != impl_->circleResults.end() && instanceIndex < static_cast<int32_t>(it->second.size())) {
        return it->second[instanceIndex];
    }
    return MetrologyCircleResult();
}

MetrologyEllipseResult MetrologyModel::GetEllipseResult(int32_t index, int32_t instanceIndex) const {
    auto it = impl_->ellipseResults.find(index);
    if (it != impl_->ellipseResults.end() && instanceIndex < static_cast<int32_t>(it->second.size())) {
        return it->second[instanceIndex];
    }
    return MetrologyEllipseResult();
}

MetrologyRectangle2Result MetrologyModel::GetRectangle2Result(int32_t index, int32_t instanceIndex) const {
    auto it = impl_->rectangleResults.find(index);
    if (it != impl_->rectangleResults.end() && instanceIndex < static_cast<int32_t>(it->second.size())) {
        return it->second[instanceIndex];
    }
    return MetrologyRectangle2Result();
}

QContour MetrologyModel::GetResultContour(int32_t index, int32_t instanceIndex) const {
    auto* obj = GetObject(index);
    if (!obj) return QContour();

    switch (obj->Type()) {
        case MetrologyObjectType::Line: {
            auto result = GetLineResult(index, instanceIndex);
            if (result.IsValid()) {
                QContour contour;
                contour.AddPoint(Point2d{result.col1, result.row1});
                contour.AddPoint(Point2d{result.col2, result.row2});
                return contour;
            }
            break;
        }

        case MetrologyObjectType::Circle: {
            auto result = GetCircleResult(index, instanceIndex);
            if (result.IsValid()) {
                QContour contour;
                int32_t numPoints = std::max(32, static_cast<int32_t>(result.radius * 0.5));
                double angleExtent = result.endAngle - result.startAngle;
                for (int32_t i = 0; i <= numPoints; ++i) {
                    double t = static_cast<double>(i) / numPoints;
                    double angle = result.startAngle + t * angleExtent;
                    double x = result.column + result.radius * std::cos(angle);
                    double y = result.row + result.radius * std::sin(angle);
                    contour.AddPoint(Point2d{x, y});
                }
                return contour;
            }
            break;
        }

        case MetrologyObjectType::Ellipse: {
            auto result = GetEllipseResult(index, instanceIndex);
            if (result.IsValid()) {
                QContour contour;
                int32_t numPoints = std::max(32, static_cast<int32_t>((result.ra + result.rb) * 0.5));
                double cosPhi = std::cos(result.phi);
                double sinPhi = std::sin(result.phi);
                for (int32_t i = 0; i <= numPoints; ++i) {
                    double t = static_cast<double>(i) / numPoints;
                    double angle = t * TWO_PI;
                    double localX = result.ra * std::cos(angle);
                    double localY = result.rb * std::sin(angle);
                    double x = result.column + localX * cosPhi - localY * sinPhi;
                    double y = result.row + localX * sinPhi + localY * cosPhi;
                    contour.AddPoint(Point2d{x, y});
                }
                return contour;
            }
            break;
        }

        case MetrologyObjectType::Rectangle2: {
            auto result = GetRectangle2Result(index, instanceIndex);
            if (result.IsValid()) {
                QContour contour;
                double cosPhi = std::cos(result.phi);
                double sinPhi = std::sin(result.phi);
                std::vector<std::pair<double, double>> corners = {
                    {-result.length1, -result.length2},
                    {result.length1, -result.length2},
                    {result.length1, result.length2},
                    {-result.length1, result.length2},
                    {-result.length1, -result.length2}
                };
                for (auto& c : corners) {
                    double x = result.column + c.first * cosPhi - c.second * sinPhi;
                    double y = result.row + c.first * sinPhi + c.second * cosPhi;
                    contour.AddPoint(Point2d{x, y});
                }
                return contour;
            }
            break;
        }
    }

    return QContour();
}

std::vector<Point2d> MetrologyModel::GetMeasuredPoints(int32_t index) const {
    auto it = impl_->measuredPoints.find(index);
    if (it != impl_->measuredPoints.end()) {
        return it->second;
    }
    return {};
}

std::vector<double> MetrologyModel::GetPointWeights(int32_t index) const {
    auto it = impl_->pointWeights.find(index);
    if (it != impl_->pointWeights.end()) {
        return it->second;
    }
    return {};
}

void MetrologyModel::Align(double row, double column, double phi) {
    double deltaRow = row - impl_->alignRow;
    double deltaCol = column - impl_->alignCol;
    double deltaPhi = phi - impl_->alignPhi;

    for (auto& obj : impl_->objects) {
        if (obj) {
            obj->Transform(deltaRow, deltaCol, deltaPhi);
        }
    }

    impl_->alignRow = row;
    impl_->alignCol = column;
    impl_->alignPhi = phi;
}

void MetrologyModel::ResetAlignment() {
    Align(0.0, 0.0, 0.0);
}

void MetrologyModel::SetDefaultParams(const MetrologyMeasureParams& params) {
    impl_->defaultParams = params;
}

const MetrologyMeasureParams& MetrologyModel::DefaultParams() const {
    return impl_->defaultParams;
}

void MetrologyModel::SetObjectParams(int32_t index, const MetrologyMeasureParams& params) {
    if (index >= 0 && index < static_cast<int32_t>(impl_->objects.size()) && impl_->objects[index]) {
        impl_->objects[index]->SetParams(params);
    }
}

} // namespace Qi::Vision::Measure
