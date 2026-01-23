/**
 * @file CaliperArray.cpp
 * @brief Implementation of multi-caliper array measurement
 */

#include <QiVision/Measure/CaliperArray.h>
#include <QiVision/Internal/Fitting.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace Qi::Vision::Measure {

namespace {

constexpr double PI = 3.14159265358979323846;

// Normalize angle to [-PI, PI]
double NormalizeAngle(double angle) {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

} // anonymous namespace

// =============================================================================
// CaliperArray Implementation
// =============================================================================

class CaliperArray::Impl {
public:
    PathType pathType_ = PathType::Line;
    CaliperArrayParams params_;

    // Path definition
    Point2d lineStart_;
    Point2d lineEnd_;
    Point2d arcCenter_;
    double arcRadius_ = 0.0;
    double arcStartAngle_ = 0.0;
    double arcSweepAngle_ = 0.0;
    std::vector<Point2d> contourPoints_;
    std::vector<double> contourCumLengths_;

    // Generated handles
    std::vector<MeasureRectangle2> handles_;
    std::vector<double> pathPositions_;
    std::vector<double> pathRatios_;
    std::vector<Point2d> pathPoints_;
    std::vector<double> pathAngles_;

    double pathLength_ = 0.0;

    // Generate handles along a line
    bool GenerateLineHandles(const Point2d& p1, const Point2d& p2,
                             const CaliperArrayParams& params);

    // Generate handles along an arc/circle
    bool GenerateArcHandles(const Point2d& center, double radius,
                            double startAngle, double sweepAngle,
                            const CaliperArrayParams& params);

    // Generate handles along a contour
    bool GenerateContourHandles(const std::vector<Point2d>& points,
                                const CaliperArrayParams& params);

    // Helper: get point on path at ratio t [0, 1]
    Point2d GetPointOnPath(double t) const;

    // Helper: get tangent angle on path at ratio t [0, 1]
    double GetTangentOnPath(double t) const;

    // Compute contour cumulative lengths
    void ComputeContourLengths();

    // Get contour point and tangent at ratio t
    Point2d GetContourPoint(double t) const;
    double GetContourTangent(double t) const;
};

bool CaliperArray::Impl::GenerateLineHandles(const Point2d& p1, const Point2d& p2,
                                              const CaliperArrayParams& params) {
    pathType_ = PathType::Line;
    params_ = params;
    lineStart_ = p1;
    lineEnd_ = p2;

    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    pathLength_ = std::sqrt(dx * dx + dy * dy);

    if (pathLength_ < 1e-6) {
        return false;
    }

    double lineAngle = std::atan2(dy, dx);

    // Determine number of calipers
    int32_t count = params.caliperCount;
    if (count <= 0) {
        count = static_cast<int32_t>(pathLength_ / params.caliperSpacing);
        count = std::max(count, MIN_CALIPER_COUNT);
    }
    count = std::clamp(count, MIN_CALIPER_COUNT, MAX_CALIPER_COUNT);

    // Compute coverage
    double startT = params.startRatio;
    double endT = params.endRatio;
    if (startT >= endT) {
        return false;
    }

    handles_.clear();
    handles_.reserve(count);
    pathPositions_.reserve(count);
    pathRatios_.reserve(count);
    pathPoints_.reserve(count);
    pathAngles_.reserve(count);

    // Handle phi angle
    // MeasureRectangle2: phi is perpendicular to profile direction
    // Profile direction = phi + PI/2
    // For perpendicular mode: profile perpendicular to path, so profile_dir = lineAngle + PI/2
    //   => phi = profile_dir - PI/2 = lineAngle
    // For parallel mode: profile along path, so profile_dir = lineAngle
    //   => phi = profile_dir - PI/2 = lineAngle - PI/2
    double phi = params.profilePerpendicular
                 ? lineAngle
                 : (lineAngle - PI / 2.0);

    for (int32_t i = 0; i < count; ++i) {
        double t = startT + (endT - startT) * (i + 0.5) / count;
        double pos = t * pathLength_;

        // Point on path
        Point2d pt;
        pt.x = p1.x + t * dx;
        pt.y = p1.y + t * dy;

        // Apply offset (perpendicular to line)
        if (std::abs(params.pathOffset) > 1e-6) {
            double perpAngle = lineAngle + PI / 2.0;
            pt.x += params.pathOffset * std::cos(perpAngle);
            pt.y += params.pathOffset * std::sin(perpAngle);
        }

        // Create handle (row, col, phi, length, width)
        // Note: MeasureRectangle2 expects (row, col) = (y, x)
        MeasureRectangle2 handle(
            pt.y, pt.x,
            phi,
            params.profileLength,
            params.handleWidth,
            params.numLines,
            params.samplesPerPixel
        );

        handles_.push_back(handle);
        pathPositions_.push_back(pos);
        pathRatios_.push_back(t);
        pathPoints_.push_back(pt);
        pathAngles_.push_back(lineAngle);
    }

    return !handles_.empty();
}

bool CaliperArray::Impl::GenerateArcHandles(const Point2d& center, double radius,
                                             double startAngle, double sweepAngle,
                                             const CaliperArrayParams& params) {
    pathType_ = (std::abs(sweepAngle - 2.0 * PI) < 0.01) ? PathType::Circle : PathType::Arc;
    params_ = params;
    arcCenter_ = center;
    arcRadius_ = radius;
    arcStartAngle_ = startAngle;
    arcSweepAngle_ = sweepAngle;

    pathLength_ = std::abs(sweepAngle * radius);

    if (pathLength_ < 1e-6 || radius < 1e-6) {
        return false;
    }

    // Determine number of calipers
    int32_t count = params.caliperCount;
    if (count <= 0) {
        count = static_cast<int32_t>(pathLength_ / params.caliperSpacing);
        count = std::max(count, MIN_CALIPER_COUNT);
    }
    count = std::clamp(count, MIN_CALIPER_COUNT, MAX_CALIPER_COUNT);

    double startT = params.startRatio;
    double endT = params.endRatio;
    if (startT >= endT) {
        return false;
    }

    handles_.clear();
    handles_.reserve(count);
    pathPositions_.reserve(count);
    pathRatios_.reserve(count);
    pathPoints_.reserve(count);
    pathAngles_.reserve(count);

    for (int32_t i = 0; i < count; ++i) {
        double t = startT + (endT - startT) * (i + 0.5) / count;
        double angle = startAngle + t * sweepAngle;
        double pos = t * pathLength_;

        // Point on arc
        Point2d pt;
        pt.x = center.x + radius * std::cos(angle);
        pt.y = center.y + radius * std::sin(angle);

        // Tangent angle (along the arc, perpendicular to radial)
        double tangentAngle = angle + PI / 2.0;

        // Handle phi angle
        // MeasureRectangle2: phi is perpendicular to profile direction
        // Profile direction = phi + PI/2
        // For perpendicular mode: profile radial (perpendicular to tangent)
        //   profile_dir = angle, phi = angle - PI/2
        // For parallel mode: profile along tangent
        //   profile_dir = angle + PI/2, phi = angle
        double phi = params.profilePerpendicular
                     ? (angle - PI / 2.0)  // Radial profile
                     : angle;               // Tangential profile

        // Apply offset (radial direction)
        if (std::abs(params.pathOffset) > 1e-6) {
            double radialDir = angle;
            pt.x += params.pathOffset * std::cos(radialDir);
            pt.y += params.pathOffset * std::sin(radialDir);
        }

        // Create handle
        MeasureRectangle2 handle(
            pt.y, pt.x,
            phi,
            params.profileLength,
            params.handleWidth,
            params.numLines,
            params.samplesPerPixel
        );

        handles_.push_back(handle);
        pathPositions_.push_back(pos);
        pathRatios_.push_back(t);
        pathPoints_.push_back(pt);
        pathAngles_.push_back(tangentAngle);
    }

    return !handles_.empty();
}

void CaliperArray::Impl::ComputeContourLengths() {
    contourCumLengths_.clear();
    contourCumLengths_.reserve(contourPoints_.size());
    contourCumLengths_.push_back(0.0);

    double cumLen = 0.0;
    for (size_t i = 1; i < contourPoints_.size(); ++i) {
        double dx = contourPoints_[i].x - contourPoints_[i-1].x;
        double dy = contourPoints_[i].y - contourPoints_[i-1].y;
        cumLen += std::sqrt(dx * dx + dy * dy);
        contourCumLengths_.push_back(cumLen);
    }

    pathLength_ = cumLen;
}

Point2d CaliperArray::Impl::GetContourPoint(double t) const {
    if (contourPoints_.empty()) {
        return {0, 0};
    }
    if (contourPoints_.size() == 1) {
        return contourPoints_[0];
    }

    t = std::clamp(t, 0.0, 1.0);
    double targetLen = t * pathLength_;

    // Binary search for segment
    auto it = std::lower_bound(contourCumLengths_.begin(), contourCumLengths_.end(), targetLen);
    size_t idx = std::distance(contourCumLengths_.begin(), it);

    if (idx == 0) {
        return contourPoints_[0];
    }
    if (idx >= contourPoints_.size()) {
        return contourPoints_.back();
    }

    // Interpolate within segment
    double segStart = contourCumLengths_[idx - 1];
    double segEnd = contourCumLengths_[idx];
    double segLen = segEnd - segStart;

    double localT = (segLen > 1e-9) ? (targetLen - segStart) / segLen : 0.0;

    const Point2d& p0 = contourPoints_[idx - 1];
    const Point2d& p1 = contourPoints_[idx];

    return {
        p0.x + localT * (p1.x - p0.x),
        p0.y + localT * (p1.y - p0.y)
    };
}

double CaliperArray::Impl::GetContourTangent(double t) const {
    if (contourPoints_.size() < 2) {
        return 0.0;
    }

    t = std::clamp(t, 0.0, 1.0);
    double targetLen = t * pathLength_;

    // Find segment
    auto it = std::lower_bound(contourCumLengths_.begin(), contourCumLengths_.end(), targetLen);
    size_t idx = std::distance(contourCumLengths_.begin(), it);

    if (idx == 0) idx = 1;
    if (idx >= contourPoints_.size()) idx = contourPoints_.size() - 1;

    const Point2d& p0 = contourPoints_[idx - 1];
    const Point2d& p1 = contourPoints_[idx];

    return std::atan2(p1.y - p0.y, p1.x - p0.x);
}

bool CaliperArray::Impl::GenerateContourHandles(const std::vector<Point2d>& points,
                                                 const CaliperArrayParams& params) {
    if (points.size() < 2) {
        return false;
    }

    pathType_ = PathType::Contour;
    params_ = params;
    contourPoints_ = points;
    ComputeContourLengths();

    if (pathLength_ < 1e-6) {
        return false;
    }

    // Determine number of calipers
    int32_t count = params.caliperCount;
    if (count <= 0) {
        count = static_cast<int32_t>(pathLength_ / params.caliperSpacing);
        count = std::max(count, MIN_CALIPER_COUNT);
    }
    count = std::clamp(count, MIN_CALIPER_COUNT, MAX_CALIPER_COUNT);

    double startT = params.startRatio;
    double endT = params.endRatio;
    if (startT >= endT) {
        return false;
    }

    handles_.clear();
    handles_.reserve(count);
    pathPositions_.reserve(count);
    pathRatios_.reserve(count);
    pathPoints_.reserve(count);
    pathAngles_.reserve(count);

    for (int32_t i = 0; i < count; ++i) {
        double t = startT + (endT - startT) * (i + 0.5) / count;
        double pos = t * pathLength_;

        Point2d pt = GetContourPoint(t);
        double tangentAngle = GetContourTangent(t);

        // Handle phi angle
        // MeasureRectangle2: phi is perpendicular to profile direction
        // Profile direction = phi + PI/2
        // For perpendicular mode: profile perpendicular to tangent
        //   profile_dir = tangent + PI/2, phi = tangent
        // For parallel mode: profile along tangent
        //   profile_dir = tangent, phi = tangent - PI/2
        double phi = params.profilePerpendicular
                     ? tangentAngle
                     : (tangentAngle - PI / 2.0);

        // Apply offset
        if (std::abs(params.pathOffset) > 1e-6) {
            double perpAngle = tangentAngle + PI / 2.0;
            pt.x += params.pathOffset * std::cos(perpAngle);
            pt.y += params.pathOffset * std::sin(perpAngle);
        }

        MeasureRectangle2 handle(
            pt.y, pt.x,
            phi,
            params.profileLength,
            params.handleWidth,
            params.numLines,
            params.samplesPerPixel
        );

        handles_.push_back(handle);
        pathPositions_.push_back(pos);
        pathRatios_.push_back(t);
        pathPoints_.push_back(pt);
        pathAngles_.push_back(tangentAngle);
    }

    return !handles_.empty();
}

// =============================================================================
// CaliperArray Public Methods
// =============================================================================

CaliperArray::CaliperArray() : impl_(std::make_unique<Impl>()) {}
CaliperArray::~CaliperArray() = default;

CaliperArray::CaliperArray(const CaliperArray& other)
    : impl_(std::make_unique<Impl>(*other.impl_)) {}

CaliperArray::CaliperArray(CaliperArray&& other) noexcept = default;

CaliperArray& CaliperArray::operator=(const CaliperArray& other) {
    if (this != &other) {
        impl_ = std::make_unique<Impl>(*other.impl_);
    }
    return *this;
}

CaliperArray& CaliperArray::operator=(CaliperArray&& other) noexcept = default;

bool CaliperArray::CreateAlongLine(const Point2d& p1, const Point2d& p2,
                                    const CaliperArrayParams& params) {
    return impl_->GenerateLineHandles(p1, p2, params);
}

bool CaliperArray::CreateAlongLine(const Point2d& p1, const Point2d& p2,
                                    int32_t caliperCount,
                                    double profileLength,
                                    double handleWidth) {
    CaliperArrayParams params;
    params.caliperCount = caliperCount;
    params.profileLength = profileLength;
    params.handleWidth = handleWidth;
    return CreateAlongLine(p1, p2, params);
}

bool CaliperArray::CreateAlongLine(const Segment2d& segment,
                                    const CaliperArrayParams& params) {
    return CreateAlongLine(segment.p1, segment.p2, params);
}

bool CaliperArray::CreateAlongLine(const Segment2d& segment,
                                    int32_t caliperCount,
                                    double profileLength,
                                    double handleWidth) {
    return CreateAlongLine(segment.p1, segment.p2, caliperCount, profileLength, handleWidth);
}

bool CaliperArray::CreateAlongArc(const Point2d& center, double radius,
                                   double startAngle, double sweepAngle,
                                   const CaliperArrayParams& params) {
    return impl_->GenerateArcHandles(center, radius, startAngle, sweepAngle, params);
}

bool CaliperArray::CreateAlongArc(const Point2d& center, double radius,
                                   double startAngle, double sweepAngle,
                                   int32_t caliperCount,
                                   double profileLength,
                                   double handleWidth) {
    CaliperArrayParams params;
    params.caliperCount = caliperCount;
    params.profileLength = profileLength;
    params.handleWidth = handleWidth;
    return CreateAlongArc(center, radius, startAngle, sweepAngle, params);
}

bool CaliperArray::CreateAlongArc(const Arc2d& arc,
                                   const CaliperArrayParams& params) {
    return CreateAlongArc(arc.center, arc.radius, arc.startAngle, arc.sweepAngle, params);
}

bool CaliperArray::CreateAlongArc(const Arc2d& arc,
                                   int32_t caliperCount,
                                   double profileLength,
                                   double handleWidth) {
    return CreateAlongArc(arc.center, arc.radius, arc.startAngle, arc.sweepAngle,
                          caliperCount, profileLength, handleWidth);
}

bool CaliperArray::CreateAlongCircle(const Point2d& center, double radius,
                                      const CaliperArrayParams& params) {
    return impl_->GenerateArcHandles(center, radius, 0.0, 2.0 * PI, params);
}

bool CaliperArray::CreateAlongCircle(const Point2d& center, double radius,
                                      int32_t caliperCount,
                                      double profileLength,
                                      double handleWidth) {
    CaliperArrayParams params;
    params.caliperCount = caliperCount;
    params.profileLength = profileLength;
    params.handleWidth = handleWidth;
    return CreateAlongCircle(center, radius, params);
}

bool CaliperArray::CreateAlongCircle(const Circle2d& circle,
                                      const CaliperArrayParams& params) {
    return CreateAlongCircle(circle.center, circle.radius, params);
}

bool CaliperArray::CreateAlongCircle(const Circle2d& circle,
                                      int32_t caliperCount,
                                      double profileLength,
                                      double handleWidth) {
    return CreateAlongCircle(circle.center, circle.radius, caliperCount, profileLength, handleWidth);
}

bool CaliperArray::CreateAlongContour(const QContour& contour,
                                       const CaliperArrayParams& params) {
    return impl_->GenerateContourHandles(contour.GetPoints(), params);
}

bool CaliperArray::CreateAlongContour(const QContour& contour,
                                       int32_t caliperCount,
                                       double profileLength,
                                       double handleWidth) {
    CaliperArrayParams params;
    params.caliperCount = caliperCount;
    params.profileLength = profileLength;
    params.handleWidth = handleWidth;
    return CreateAlongContour(contour, params);
}

void CaliperArray::Clear() {
    impl_->handles_.clear();
    impl_->pathPositions_.clear();
    impl_->pathRatios_.clear();
    impl_->pathPoints_.clear();
    impl_->pathAngles_.clear();
    impl_->pathLength_ = 0.0;
}

bool CaliperArray::IsValid() const {
    return !impl_->handles_.empty();
}

int32_t CaliperArray::Size() const {
    return static_cast<int32_t>(impl_->handles_.size());
}

PathType CaliperArray::GetPathType() const {
    return impl_->pathType_;
}

double CaliperArray::GetPathLength() const {
    return impl_->pathLength_;
}

const CaliperArrayParams& CaliperArray::GetParams() const {
    return impl_->params_;
}

const MeasureRectangle2& CaliperArray::GetHandle(int32_t index) const {
    if (index < 0 || index >= static_cast<int32_t>(impl_->handles_.size())) {
        throw std::out_of_range("CaliperArray index out of range");
    }
    return impl_->handles_[index];
}

const std::vector<MeasureRectangle2>& CaliperArray::GetHandles() const {
    return impl_->handles_;
}

double CaliperArray::GetPathPosition(int32_t index) const {
    if (index < 0 || index >= static_cast<int32_t>(impl_->pathPositions_.size())) {
        return 0.0;
    }
    return impl_->pathPositions_[index];
}

double CaliperArray::GetPathRatio(int32_t index) const {
    if (index < 0 || index >= static_cast<int32_t>(impl_->pathRatios_.size())) {
        return 0.0;
    }
    return impl_->pathRatios_[index];
}

Point2d CaliperArray::GetPathPoint(int32_t index) const {
    if (index < 0 || index >= static_cast<int32_t>(impl_->pathPoints_.size())) {
        return {0, 0};
    }
    return impl_->pathPoints_[index];
}

CaliperArrayResult CaliperArray::MeasurePos(const QImage& image,
                                             double sigma,
                                             double threshold,
                                             const std::string& transition,
                                             const std::string& select,
                                             CaliperArrayStats* stats) const {
    CaliperArrayResult result;
    result.numCalipers = Size();

    if (!IsValid()) {
        return result;
    }

    result.results.reserve(result.numCalipers);
    result.validMask.resize(result.numCalipers, false);

    double totalAmplitude = 0.0;
    double minAmp = std::numeric_limits<double>::max();
    double maxAmp = 0.0;
    int edgeCount = 0;

    for (int32_t i = 0; i < result.numCalipers; ++i) {
        CaliperResult cr;
        cr.caliperIndex = i;
        cr.pathPosition = impl_->pathPositions_[i];
        cr.pathRatio = impl_->pathRatios_[i];
        cr.pathPoint = impl_->pathPoints_[i];
        cr.pathAngle = impl_->pathAngles_[i];

        // Measure using new API
        cr.edges = Measure::MeasurePos(image, impl_->handles_[i],
                                        sigma, threshold, transition, select);

        if (!cr.edges.empty()) {
            cr.hasResult = true;
            const auto& firstEdge = cr.edges[0];
            cr.resultPoint = firstEdge.Position();
            cr.resultScore = firstEdge.confidence;

            result.firstEdgePoints.push_back(cr.resultPoint);
            result.validMask[i] = true;
            result.numValid++;

            // Statistics
            for (const auto& e : cr.edges) {
                result.allEdgePoints.push_back(e.Position());
                totalAmplitude += e.amplitude;
                minAmp = std::min(minAmp, e.amplitude);
                maxAmp = std::max(maxAmp, e.amplitude);
                edgeCount++;
            }
        } else {
            result.numInvalid++;
        }

        result.results.push_back(std::move(cr));
    }

    result.validRatio = (result.numCalipers > 0)
                        ? static_cast<double>(result.numValid) / result.numCalipers
                        : 0.0;

    // Compute mean score
    if (result.numValid > 0) {
        double totalScore = 0.0;
        for (const auto& r : result.results) {
            if (r.hasResult) {
                totalScore += r.resultScore;
            }
        }
        result.meanScore = totalScore / result.numValid;
    }

    // Update stats
    if (stats) {
        stats->totalEdgesFound = edgeCount;
        stats->meanAmplitude = (edgeCount > 0) ? totalAmplitude / edgeCount : 0.0;
        stats->minAmplitude = (edgeCount > 0) ? minAmp : 0.0;
        stats->maxAmplitude = maxAmp;
    }

    return result;
}

CaliperArrayResult CaliperArray::FuzzyMeasurePos(const QImage& image,
                                                  double sigma,
                                                  double threshold,
                                                  const std::string& transition,
                                                  const std::string& select,
                                                  double fuzzyThresh,
                                                  CaliperArrayStats* stats) const {
    CaliperArrayResult result;
    result.numCalipers = Size();

    if (!IsValid()) {
        return result;
    }

    result.results.reserve(result.numCalipers);
    result.validMask.resize(result.numCalipers, false);

    for (int32_t i = 0; i < result.numCalipers; ++i) {
        CaliperResult cr;
        cr.caliperIndex = i;
        cr.pathPosition = impl_->pathPositions_[i];
        cr.pathRatio = impl_->pathRatios_[i];
        cr.pathPoint = impl_->pathPoints_[i];
        cr.pathAngle = impl_->pathAngles_[i];

        cr.edges = Measure::FuzzyMeasurePos(image, impl_->handles_[i],
                                             sigma, threshold, transition, select,
                                             fuzzyThresh);

        if (!cr.edges.empty()) {
            cr.hasResult = true;
            const auto& firstEdge = cr.edges[0];
            cr.resultPoint = firstEdge.Position();
            cr.resultScore = firstEdge.score;

            result.firstEdgePoints.push_back(cr.resultPoint);
            result.validMask[i] = true;
            result.numValid++;

            for (const auto& e : cr.edges) {
                result.allEdgePoints.push_back(e.Position());
            }
        } else {
            result.numInvalid++;
        }

        result.results.push_back(std::move(cr));
    }

    result.validRatio = (result.numCalipers > 0)
                        ? static_cast<double>(result.numValid) / result.numCalipers
                        : 0.0;

    if (result.numValid > 0) {
        double totalScore = 0.0;
        for (const auto& r : result.results) {
            if (r.hasResult) {
                totalScore += r.resultScore;
            }
        }
        result.meanScore = totalScore / result.numValid;
    }

    if (stats) {
        stats->totalEdgesFound = static_cast<int32_t>(result.allEdgePoints.size());
    }

    return result;
}

CaliperArrayResult CaliperArray::MeasurePairs(const QImage& image,
                                               double sigma,
                                               double threshold,
                                               const std::string& transition,
                                               const std::string& select,
                                               CaliperArrayStats* stats) const {
    CaliperArrayResult result;
    result.numCalipers = Size();

    if (!IsValid()) {
        return result;
    }

    result.results.reserve(result.numCalipers);
    result.validMask.resize(result.numCalipers, false);

    std::vector<double> validWidths;
    validWidths.reserve(result.numCalipers);

    for (int32_t i = 0; i < result.numCalipers; ++i) {
        CaliperResult cr;
        cr.caliperIndex = i;
        cr.pathPosition = impl_->pathPositions_[i];
        cr.pathRatio = impl_->pathRatios_[i];
        cr.pathPoint = impl_->pathPoints_[i];
        cr.pathAngle = impl_->pathAngles_[i];

        cr.pairs = Measure::MeasurePairs(image, impl_->handles_[i],
                                          sigma, threshold, transition, select);

        if (!cr.pairs.empty()) {
            cr.hasResult = true;
            const auto& firstPair = cr.pairs[0];
            cr.resultPoint = firstPair.Center();
            cr.resultScore = firstPair.score;
            cr.width = firstPair.width;
            cr.firstEdge = firstPair.first.Position();
            cr.secondEdge = firstPair.second.Position();

            result.centerPoints.push_back(cr.resultPoint);
            result.firstEdgePoints.push_back(cr.resultPoint);
            result.firstPairEdges.push_back(cr.firstEdge);
            result.secondPairEdges.push_back(cr.secondEdge);
            result.widths.push_back(cr.width);
            validWidths.push_back(cr.width);

            result.validMask[i] = true;
            result.numValid++;
        } else {
            result.numInvalid++;
        }

        result.results.push_back(std::move(cr));
    }

    result.validRatio = (result.numCalipers > 0)
                        ? static_cast<double>(result.numValid) / result.numCalipers
                        : 0.0;

    // Width statistics
    if (!validWidths.empty()) {
        double sum = std::accumulate(validWidths.begin(), validWidths.end(), 0.0);
        result.meanWidth = sum / validWidths.size();

        double sqSum = 0.0;
        for (double w : validWidths) {
            sqSum += (w - result.meanWidth) * (w - result.meanWidth);
        }
        result.stdWidth = std::sqrt(sqSum / validWidths.size());

        result.minWidth = *std::min_element(validWidths.begin(), validWidths.end());
        result.maxWidth = *std::max_element(validWidths.begin(), validWidths.end());
    }

    if (result.numValid > 0) {
        double totalScore = 0.0;
        for (const auto& r : result.results) {
            if (r.hasResult) {
                totalScore += r.resultScore;
            }
        }
        result.meanScore = totalScore / result.numValid;
    }

    if (stats) {
        stats->totalPairsFound = result.numValid;
    }

    return result;
}

CaliperArrayResult CaliperArray::FuzzyMeasurePairs(const QImage& image,
                                                    double sigma,
                                                    double threshold,
                                                    const std::string& transition,
                                                    const std::string& select,
                                                    double fuzzyThresh,
                                                    CaliperArrayStats* stats) const {
    CaliperArrayResult result;
    result.numCalipers = Size();

    if (!IsValid()) {
        return result;
    }

    result.results.reserve(result.numCalipers);
    result.validMask.resize(result.numCalipers, false);

    std::vector<double> validWidths;
    validWidths.reserve(result.numCalipers);

    for (int32_t i = 0; i < result.numCalipers; ++i) {
        CaliperResult cr;
        cr.caliperIndex = i;
        cr.pathPosition = impl_->pathPositions_[i];
        cr.pathRatio = impl_->pathRatios_[i];
        cr.pathPoint = impl_->pathPoints_[i];
        cr.pathAngle = impl_->pathAngles_[i];

        cr.pairs = Measure::FuzzyMeasurePairs(image, impl_->handles_[i],
                                               sigma, threshold, transition, select,
                                               fuzzyThresh);

        if (!cr.pairs.empty()) {
            cr.hasResult = true;
            const auto& firstPair = cr.pairs[0];
            cr.resultPoint = firstPair.Center();
            cr.resultScore = firstPair.score;
            cr.width = firstPair.width;
            cr.firstEdge = firstPair.first.Position();
            cr.secondEdge = firstPair.second.Position();

            result.centerPoints.push_back(cr.resultPoint);
            result.firstEdgePoints.push_back(cr.resultPoint);
            result.firstPairEdges.push_back(cr.firstEdge);
            result.secondPairEdges.push_back(cr.secondEdge);
            result.widths.push_back(cr.width);
            validWidths.push_back(cr.width);

            result.validMask[i] = true;
            result.numValid++;
        } else {
            result.numInvalid++;
        }

        result.results.push_back(std::move(cr));
    }

    result.validRatio = (result.numCalipers > 0)
                        ? static_cast<double>(result.numValid) / result.numCalipers
                        : 0.0;

    if (!validWidths.empty()) {
        double sum = std::accumulate(validWidths.begin(), validWidths.end(), 0.0);
        result.meanWidth = sum / validWidths.size();

        double sqSum = 0.0;
        for (double w : validWidths) {
            sqSum += (w - result.meanWidth) * (w - result.meanWidth);
        }
        result.stdWidth = std::sqrt(sqSum / validWidths.size());

        result.minWidth = *std::min_element(validWidths.begin(), validWidths.end());
        result.maxWidth = *std::max_element(validWidths.begin(), validWidths.end());
    }

    if (result.numValid > 0) {
        double totalScore = 0.0;
        for (const auto& r : result.results) {
            if (r.hasResult) {
                totalScore += r.resultScore;
            }
        }
        result.meanScore = totalScore / result.numValid;
    }

    if (stats) {
        stats->totalPairsFound = result.numValid;
    }

    return result;
}

std::vector<Point2d> CaliperArray::MeasureFirstEdges(const QImage& image,
                                                      double sigma,
                                                      double threshold,
                                                      const std::string& transition,
                                                      const std::string& select) const {
    auto result = MeasurePos(image, sigma, threshold, transition, select);
    return result.firstEdgePoints;
}

std::vector<Point2d> CaliperArray::MeasurePairCenters(const QImage& image,
                                                       double sigma,
                                                       double threshold,
                                                       const std::string& transition,
                                                       const std::string& select) const {
    auto result = MeasurePairs(image, sigma, threshold, transition, select);
    return result.centerPoints;
}

std::vector<double> CaliperArray::MeasureWidths(const QImage& image,
                                                 double sigma,
                                                 double threshold,
                                                 const std::string& transition,
                                                 const std::string& select,
                                                 double& meanWidth,
                                                 double& stdWidth) const {
    auto result = MeasurePairs(image, sigma, threshold, transition, select);
    meanWidth = result.meanWidth;
    stdWidth = result.stdWidth;
    return result.widths;
}

std::vector<RotatedRect2d> CaliperArray::GetHandleRects() const {
    std::vector<RotatedRect2d> rects;
    rects.reserve(impl_->handles_.size());

    for (const auto& handle : impl_->handles_) {
        rects.push_back(handle.ToRotatedRect());
    }

    return rects;
}

std::vector<Point2d> CaliperArray::GetPathPoints(int32_t numPoints) const {
    if (numPoints <= 0) {
        numPoints = static_cast<int32_t>(impl_->pathLength_ / 2.0);
        numPoints = std::max(numPoints, 10);
    }

    std::vector<Point2d> points;
    points.reserve(numPoints);

    if (impl_->pathType_ == PathType::Line) {
        for (int32_t i = 0; i < numPoints; ++i) {
            double t = static_cast<double>(i) / (numPoints - 1);
            Point2d pt;
            pt.x = impl_->lineStart_.x + t * (impl_->lineEnd_.x - impl_->lineStart_.x);
            pt.y = impl_->lineStart_.y + t * (impl_->lineEnd_.y - impl_->lineStart_.y);
            points.push_back(pt);
        }
    } else if (impl_->pathType_ == PathType::Arc || impl_->pathType_ == PathType::Circle) {
        for (int32_t i = 0; i < numPoints; ++i) {
            double t = static_cast<double>(i) / (numPoints - 1);
            double angle = impl_->arcStartAngle_ + t * impl_->arcSweepAngle_;
            Point2d pt;
            pt.x = impl_->arcCenter_.x + impl_->arcRadius_ * std::cos(angle);
            pt.y = impl_->arcCenter_.y + impl_->arcRadius_ * std::sin(angle);
            points.push_back(pt);
        }
    } else if (impl_->pathType_ == PathType::Contour) {
        for (int32_t i = 0; i < numPoints; ++i) {
            double t = static_cast<double>(i) / (numPoints - 1);
            points.push_back(impl_->GetContourPoint(t));
        }
    }

    return points;
}

std::vector<Point2d> CaliperArray::GetCaliperCenters() const {
    return impl_->pathPoints_;
}

// =============================================================================
// Factory Functions
// =============================================================================

CaliperArray CreateCaliperArrayLine(const Point2d& p1, const Point2d& p2,
                                     int32_t caliperCount,
                                     double profileLength,
                                     double handleWidth) {
    CaliperArray array;
    CaliperArrayParams params;
    params.caliperCount = caliperCount;
    params.profileLength = profileLength;
    params.handleWidth = handleWidth;
    array.CreateAlongLine(p1, p2, params);
    return array;
}

CaliperArray CreateCaliperArrayLine(const Segment2d& segment,
                                     int32_t caliperCount,
                                     double profileLength,
                                     double handleWidth) {
    return CreateCaliperArrayLine(segment.p1, segment.p2, caliperCount, profileLength, handleWidth);
}

CaliperArray CreateCaliperArrayArc(const Point2d& center, double radius,
                                    double startAngle, double sweepAngle,
                                    int32_t caliperCount,
                                    double profileLength,
                                    double handleWidth) {
    CaliperArray array;
    CaliperArrayParams params;
    params.caliperCount = caliperCount;
    params.profileLength = profileLength;
    params.handleWidth = handleWidth;
    array.CreateAlongArc(center, radius, startAngle, sweepAngle, params);
    return array;
}

CaliperArray CreateCaliperArrayArc(const Arc2d& arc,
                                    int32_t caliperCount,
                                    double profileLength,
                                    double handleWidth) {
    return CreateCaliperArrayArc(arc.center, arc.radius, arc.startAngle, arc.sweepAngle,
                                  caliperCount, profileLength, handleWidth);
}

CaliperArray CreateCaliperArrayCircle(const Point2d& center, double radius,
                                       int32_t caliperCount,
                                       double profileLength,
                                       double handleWidth) {
    CaliperArray array;
    CaliperArrayParams params;
    params.caliperCount = caliperCount;
    params.profileLength = profileLength;
    params.handleWidth = handleWidth;
    array.CreateAlongCircle(center, radius, params);
    return array;
}

CaliperArray CreateCaliperArrayCircle(const Circle2d& circle,
                                       int32_t caliperCount,
                                       double profileLength,
                                       double handleWidth) {
    return CreateCaliperArrayCircle(circle.center, circle.radius, caliperCount, profileLength, handleWidth);
}

CaliperArray CreateCaliperArrayContour(const QContour& contour,
                                        int32_t caliperCount,
                                        double profileLength,
                                        double handleWidth) {
    CaliperArray array;
    CaliperArrayParams params;
    params.caliperCount = caliperCount;
    params.profileLength = profileLength;
    params.handleWidth = handleWidth;
    array.CreateAlongContour(contour, params);
    return array;
}

// =============================================================================
// Convenience Functions
// =============================================================================

std::optional<Line2d> MeasureAndFitLine(const QImage& image,
                                         const Point2d& p1, const Point2d& p2,
                                         int32_t caliperCount,
                                         double sigma,
                                         double threshold,
                                         const std::string& transition,
                                         const std::string& select,
                                         std::vector<Point2d>* measuredPoints) {
    CaliperArray array = CreateCaliperArrayLine(p1, p2, caliperCount);
    auto result = array.MeasurePos(image, sigma, threshold, transition, select);

    if (measuredPoints) {
        *measuredPoints = result.firstEdgePoints;
    }

    if (!result.CanFitLine()) {
        return std::nullopt;
    }

    auto fitResult = Internal::FitLine(result.firstEdgePoints);
    if (!fitResult.success) {
        return std::nullopt;
    }

    return fitResult.line;
}

std::optional<Circle2d> MeasureAndFitCircle(const QImage& image,
                                             const Point2d& approxCenter,
                                             double approxRadius,
                                             int32_t caliperCount,
                                             double sigma,
                                             double threshold,
                                             const std::string& transition,
                                             const std::string& select,
                                             std::vector<Point2d>* measuredPoints) {
    CaliperArray array = CreateCaliperArrayCircle(approxCenter, approxRadius, caliperCount);
    auto result = array.MeasurePos(image, sigma, threshold, transition, select);

    if (measuredPoints) {
        *measuredPoints = result.firstEdgePoints;
    }

    if (!result.CanFitCircle()) {
        return std::nullopt;
    }

    auto fitResult = Internal::FitCircleGeometric(result.firstEdgePoints);
    if (!fitResult.success) {
        return std::nullopt;
    }

    return fitResult.circle;
}

bool MeasureWidthsAlongLine(const QImage& image,
                             const Point2d& p1, const Point2d& p2,
                             int32_t caliperCount,
                             double sigma,
                             double threshold,
                             const std::string& transition,
                             const std::string& select,
                             double& meanWidth,
                             double& stdWidth,
                             std::vector<double>* widths) {
    CaliperArray array = CreateCaliperArrayLine(p1, p2, caliperCount);
    auto result = array.MeasurePairs(image, sigma, threshold, transition, select);

    meanWidth = result.meanWidth;
    stdWidth = result.stdWidth;

    if (widths) {
        *widths = result.widths;
    }

    return result.numValid > 0;
}

bool MeasureWidthsAlongArc(const QImage& image,
                            const Arc2d& arc,
                            int32_t caliperCount,
                            double sigma,
                            double threshold,
                            const std::string& transition,
                            const std::string& select,
                            double& meanWidth,
                            double& stdWidth,
                            std::vector<double>* widths) {
    CaliperArray array = CreateCaliperArrayArc(arc, caliperCount);
    auto result = array.MeasurePairs(image, sigma, threshold, transition, select);

    meanWidth = result.meanWidth;
    stdWidth = result.stdWidth;

    if (widths) {
        *widths = result.widths;
    }

    return result.numValid > 0;
}

} // namespace Qi::Vision::Measure
