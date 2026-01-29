/**
 * @file Hough.cpp
 * @brief Hough Transform public API implementation
 *
 * Wraps Internal layer functions with Halcon-style API
 */

#include <QiVision/Hough/Hough.h>
#include <QiVision/Core/Exception.h>
#include <QiVision/Internal/Hough.h>
#include <QiVision/Display/Draw.h>

#include <algorithm>
#include <cmath>

namespace Qi::Vision::Hough {

// =============================================================================
// Helper Functions
// =============================================================================

namespace {

constexpr double PI = 3.14159265358979323846;
constexpr double EPSILON = 1e-10;

void ValidateBinaryInput(const QImage& image, const char* funcName) {
    if (image.Empty()) {
        return;  // Empty input -> empty output (don't throw)
    }

    if (image.Type() != PixelType::UInt8) {
        throw UnsupportedException(std::string(funcName) + " requires UInt8 image");
    }

    if (image.Channels() != 1) {
        throw UnsupportedException(std::string(funcName) + " requires single-channel image");
    }
}

Internal::HoughLineParams BuildInternalLineParams(
    double rhoResolution,
    double thetaResolution,
    int32_t threshold,
    int32_t maxLines
) {
    Internal::HoughLineParams params;
    params.rhoResolution = rhoResolution;
    params.thetaResolution = thetaResolution;

    // Convert absolute threshold to ratio if needed
    if (threshold > 0) {
        params.threshold = threshold;
        params.thresholdIsRatio = false;
    } else {
        params.threshold = 0.3;
        params.thresholdIsRatio = true;
    }

    params.maxLines = maxLines;
    params.minDistance = rhoResolution * 10.0;
    params.suppressOverlapping = true;

    return params;
}

Internal::HoughLineProbParams BuildInternalLinePParams(
    double rhoResolution,
    double thetaResolution,
    int32_t threshold,
    double minLineLength,
    double maxLineGap
) {
    Internal::HoughLineProbParams params;
    params.rhoResolution = rhoResolution;
    params.thetaResolution = thetaResolution;
    params.threshold = threshold;
    params.minLineLength = minLineLength;
    params.maxLineGap = maxLineGap;
    params.maxLines = 0;  // Unlimited
    return params;
}

Internal::HoughCircleParams BuildInternalCircleParams(
    double dp,
    double minDist,
    double param1,
    double param2,
    int32_t minRadius,
    int32_t maxRadius
) {
    Internal::HoughCircleParams params;
    params.dp = dp;
    params.minDist = minDist;
    params.param1 = param1;
    params.param2 = param2;
    params.minRadius = minRadius;
    params.maxRadius = maxRadius;
    params.maxCircles = 0;  // Unlimited
    return params;
}

// Convert Internal HoughLine to public HoughLine
HoughLine ConvertLine(const Internal::HoughLine& internalLine) {
    HoughLine line;
    line.rho = internalLine.rho;
    line.theta = internalLine.theta;
    line.score = internalLine.score;

    // Get endpoints for visualization
    auto [pt1, pt2] = internalLine.GetTwoPoints(2000.0);
    line.p1 = pt1;
    line.p2 = pt2;

    return line;
}

// Convert Internal HoughCircle to public HoughCircle
HoughCircle ConvertCircle(const Internal::HoughCircle& internalCircle) {
    HoughCircle circle;
    circle.row = internalCircle.center.y;
    circle.column = internalCircle.center.x;
    circle.radius = internalCircle.radius;
    circle.score = internalCircle.score;
    return circle;
}

} // anonymous namespace

// =============================================================================
// HoughLine Methods
// =============================================================================

Line2d HoughLine::ToLine2d() const {
    // Line equation: x*cos(theta) + y*sin(theta) = rho
    // Convert to ax + by + c = 0 form
    double a = std::cos(theta);
    double b = std::sin(theta);
    double c = -rho;
    return Line2d(a, b, c);
}

void HoughLine::GetEndPoints(double length, Point2d& pt1, Point2d& pt2) const {
    // Get a point on the line
    double x0 = rho * std::cos(theta);
    double y0 = rho * std::sin(theta);

    // Direction perpendicular to normal (along the line)
    double dx = -std::sin(theta);
    double dy = std::cos(theta);

    pt1 = Point2d(x0 - length * 0.5 * dx, y0 - length * 0.5 * dy);
    pt2 = Point2d(x0 + length * 0.5 * dx, y0 + length * 0.5 * dy);
}

// =============================================================================
// Standard Hough Line Transform
// =============================================================================

void HoughLines(
    const QImage& edgeImage,
    std::vector<HoughLine>& lines,
    double rhoResolution,
    double thetaResolution,
    int32_t threshold,
    int32_t maxLines
) {
    lines.clear();

    if (edgeImage.Empty()) {
        return;
    }

    ValidateBinaryInput(edgeImage, "HoughLines");

    // Build parameters
    Internal::HoughLineParams params = BuildInternalLineParams(
        rhoResolution, thetaResolution, threshold, maxLines);

    // Call Internal implementation
    auto internalLines = Internal::HoughLines(edgeImage, params);

    // Convert results
    lines.reserve(internalLines.size());
    for (const auto& line : internalLines) {
        lines.push_back(ConvertLine(line));
    }
}

void HoughLines(
    const std::vector<Point2d>& points,
    int32_t imageWidth,
    int32_t imageHeight,
    std::vector<HoughLine>& lines,
    double rhoResolution,
    double thetaResolution,
    int32_t threshold,
    int32_t maxLines
) {
    lines.clear();

    if (points.empty()) {
        return;
    }

    Internal::HoughLineParams params = BuildInternalLineParams(
        rhoResolution, thetaResolution, threshold, maxLines);

    auto internalLines = Internal::HoughLines(points, imageWidth, imageHeight, params);

    lines.reserve(internalLines.size());
    for (const auto& line : internalLines) {
        lines.push_back(ConvertLine(line));
    }
}

// =============================================================================
// Probabilistic Hough Transform
// =============================================================================

void HoughLinesP(
    const QImage& edgeImage,
    std::vector<Segment2d>& segments,
    double rhoResolution,
    double thetaResolution,
    int32_t threshold,
    double minLineLength,
    double maxLineGap
) {
    segments.clear();

    if (edgeImage.Empty()) {
        return;
    }

    ValidateBinaryInput(edgeImage, "HoughLinesP");

    Internal::HoughLineProbParams params = BuildInternalLinePParams(
        rhoResolution, thetaResolution, threshold, minLineLength, maxLineGap);

    auto internalSegments = Internal::HoughLinesP(edgeImage, params);

    segments.reserve(internalSegments.size());
    for (const auto& seg : internalSegments) {
        segments.push_back(seg.ToSegment2d());
    }
}

// =============================================================================
// Hough Lines from Contours (XLD)
// =============================================================================

void HoughLinesXld(
    const QContourArray& contours,
    std::vector<Line2d>& lines,
    int32_t threshold,
    double angleResolution,
    int32_t maxLines
) {
    lines.clear();

    if (contours.Empty()) {
        return;
    }

    // Collect all contour points
    std::vector<Point2d> points;
    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();

    for (size_t i = 0; i < contours.Size(); ++i) {
        const QContour& contour = contours[i];
        for (size_t j = 0; j < contour.Size(); ++j) {
            Point2d pt = contour.PointAt(j);
            points.push_back(pt);
            minX = std::min(minX, pt.x);
            maxX = std::max(maxX, pt.x);
            minY = std::min(minY, pt.y);
            maxY = std::max(maxY, pt.y);
        }
    }

    if (points.empty()) {
        return;
    }

    int32_t imageWidth = static_cast<int32_t>(maxX - minX) + 1;
    int32_t imageHeight = static_cast<int32_t>(maxY - minY) + 1;

    Internal::HoughLineParams params;
    params.rhoResolution = 1.0;
    params.thetaResolution = angleResolution;
    params.threshold = static_cast<double>(threshold) / static_cast<double>(points.size());
    params.thresholdIsRatio = true;
    params.maxLines = maxLines;
    params.suppressOverlapping = true;

    auto internalLines = Internal::HoughLines(points, imageWidth, imageHeight, params);

    lines.reserve(internalLines.size());
    for (const auto& line : internalLines) {
        lines.push_back(line.ToLine2d());
    }
}

// =============================================================================
// Hough Circle Transform
// =============================================================================

void HoughCircles(
    const QImage& edgeImage,
    std::vector<HoughCircle>& circles,
    double dp,
    double minDist,
    double param1,
    double param2,
    int32_t minRadius,
    int32_t maxRadius
) {
    circles.clear();

    if (edgeImage.Empty()) {
        return;
    }

    ValidateBinaryInput(edgeImage, "HoughCircles");

    Internal::HoughCircleParams params = BuildInternalCircleParams(
        dp, minDist, param1, param2, minRadius, maxRadius);

    auto internalCircles = Internal::HoughCircles(edgeImage, params);

    circles.reserve(internalCircles.size());
    for (const auto& circle : internalCircles) {
        circles.push_back(ConvertCircle(circle));
    }
}

void HoughCircles(
    const QImage& edgeImage,
    const QImage& gradientX,
    const QImage& gradientY,
    std::vector<HoughCircle>& circles,
    double minDist,
    double threshold,
    int32_t minRadius,
    int32_t maxRadius
) {
    circles.clear();

    if (edgeImage.Empty()) {
        return;
    }

    ValidateBinaryInput(edgeImage, "HoughCircles");

    Internal::HoughCircleParams params;
    params.dp = 1.0;
    params.minDist = minDist;
    params.param1 = 100.0;
    params.param2 = threshold;
    params.minRadius = minRadius;
    params.maxRadius = maxRadius;

    auto internalCircles = Internal::HoughCircles(edgeImage, gradientX, gradientY, params);

    circles.reserve(internalCircles.size());
    for (const auto& circle : internalCircles) {
        circles.push_back(ConvertCircle(circle));
    }
}

// =============================================================================
// Hough Circles from Contours (XLD)
// =============================================================================

void HoughCirclesXld(
    const QContourArray& contours,
    std::vector<Circle2d>& circles,
    double minRadius,
    double maxRadius,
    int32_t threshold,
    double radiusResolution
) {
    circles.clear();

    if (contours.Empty()) {
        return;
    }

    // Collect all contour points
    std::vector<Point2d> points;
    for (size_t i = 0; i < contours.Size(); ++i) {
        const QContour& contour = contours[i];
        for (size_t j = 0; j < contour.Size(); ++j) {
            points.push_back(contour.PointAt(j));
        }
    }

    if (points.empty()) {
        return;
    }

    int32_t minR = static_cast<int32_t>(minRadius / radiusResolution);
    int32_t maxR = static_cast<int32_t>(maxRadius / radiusResolution);

    double thresholdRatio = static_cast<double>(threshold) / static_cast<double>(points.size());

    auto internalCircles = Internal::HoughCircles(points, minR, maxR, thresholdRatio, 0);

    circles.reserve(internalCircles.size());
    for (const auto& circle : internalCircles) {
        circles.push_back(circle.ToCircle2d());
    }
}

// =============================================================================
// Visualization Functions
// =============================================================================

void DrawHoughLines(
    QImage& image,
    const std::vector<HoughLine>& lines,
    const Scalar& color,
    int32_t thickness
) {
    if (image.Empty() || lines.empty()) {
        return;
    }

    int32_t width = image.Width();
    int32_t height = image.Height();

    for (const auto& line : lines) {
        // Get endpoints clipped to image
        Segment2d seg = ClipHoughLineToImage(line, width, height);

        if (seg.Length() > EPSILON) {
            Draw::Line(image, seg.p1, seg.p2, color, thickness);
        }
    }
}

void DrawHoughCircles(
    QImage& image,
    const std::vector<HoughCircle>& circles,
    const Scalar& color,
    int32_t thickness
) {
    if (image.Empty() || circles.empty()) {
        return;
    }

    for (const auto& circle : circles) {
        Point2d center(circle.column, circle.row);
        Draw::Circle(image, center, circle.radius, color, thickness);

        // Draw small cross at center
        double crossSize = 5.0;
        Draw::Line(image,
                   Point2d(circle.column - crossSize, circle.row),
                   Point2d(circle.column + crossSize, circle.row),
                   color, 1);
        Draw::Line(image,
                   Point2d(circle.column, circle.row - crossSize),
                   Point2d(circle.column, circle.row + crossSize),
                   color, 1);
    }
}

// =============================================================================
// Advanced Options
// =============================================================================

void DetectHoughLines(
    const QImage& edgeImage,
    std::vector<HoughLine>& lines,
    const HoughLineParams& params
) {
    lines.clear();

    if (edgeImage.Empty()) {
        return;
    }

    ValidateBinaryInput(edgeImage, "DetectHoughLines");

    Internal::HoughLineParams internalParams;
    internalParams.rhoResolution = params.rhoResolution;
    internalParams.thetaResolution = params.thetaResolution;
    internalParams.threshold = params.threshold;
    internalParams.thresholdIsRatio = params.thresholdIsRatio;
    internalParams.maxLines = params.maxLines;
    internalParams.minDistance = params.minDistance;
    internalParams.suppressOverlapping = params.suppressOverlapping;

    auto internalLines = Internal::HoughLines(edgeImage, internalParams);

    lines.reserve(internalLines.size());
    for (const auto& line : internalLines) {
        lines.push_back(ConvertLine(line));
    }
}

void DetectHoughLinesP(
    const QImage& edgeImage,
    std::vector<Segment2d>& segments,
    const HoughLinePParams& params
) {
    segments.clear();

    if (edgeImage.Empty()) {
        return;
    }

    ValidateBinaryInput(edgeImage, "DetectHoughLinesP");

    Internal::HoughLineProbParams internalParams;
    internalParams.rhoResolution = params.rhoResolution;
    internalParams.thetaResolution = params.thetaResolution;
    internalParams.threshold = params.threshold;
    internalParams.minLineLength = params.minLineLength;
    internalParams.maxLineGap = params.maxLineGap;
    internalParams.maxLines = params.maxLines;

    auto internalSegments = Internal::HoughLinesP(edgeImage, internalParams);

    segments.reserve(internalSegments.size());
    for (const auto& seg : internalSegments) {
        segments.push_back(seg.ToSegment2d());
    }
}

void DetectHoughCircles(
    const QImage& edgeImage,
    std::vector<HoughCircle>& circles,
    const HoughCircleParams& params
) {
    circles.clear();

    if (edgeImage.Empty()) {
        return;
    }

    ValidateBinaryInput(edgeImage, "DetectHoughCircles");

    Internal::HoughCircleParams internalParams;
    internalParams.dp = params.dp;
    internalParams.minDist = params.minDist;
    internalParams.param1 = params.param1;
    internalParams.param2 = params.param2;
    internalParams.minRadius = params.minRadius;
    internalParams.maxRadius = params.maxRadius;
    internalParams.maxCircles = params.maxCircles;

    auto internalCircles = Internal::HoughCircles(edgeImage, internalParams);

    circles.reserve(internalCircles.size());
    for (const auto& circle : internalCircles) {
        circles.push_back(ConvertCircle(circle));
    }
}

// =============================================================================
// Utility Functions
// =============================================================================

std::vector<HoughLine> MergeHoughLines(
    const std::vector<HoughLine>& lines,
    double rhoThreshold,
    double thetaThreshold
) {
    if (lines.empty()) {
        return {};
    }

    // Convert to Internal format
    std::vector<Internal::HoughLine> internalLines;
    internalLines.reserve(lines.size());
    for (const auto& line : lines) {
        internalLines.emplace_back(line.rho, line.theta, line.score);
    }

    // Call Internal merge
    auto mergedInternal = Internal::MergeHoughLines(internalLines, rhoThreshold, thetaThreshold);

    // Convert back
    std::vector<HoughLine> merged;
    merged.reserve(mergedInternal.size());
    for (const auto& line : mergedInternal) {
        merged.push_back(ConvertLine(line));
    }

    return merged;
}

std::vector<HoughCircle> MergeHoughCircles(
    const std::vector<HoughCircle>& circles,
    double centerThreshold,
    double radiusThreshold
) {
    if (circles.empty()) {
        return {};
    }

    // Convert to Internal format
    std::vector<Internal::HoughCircle> internalCircles;
    internalCircles.reserve(circles.size());
    for (const auto& circle : circles) {
        internalCircles.emplace_back(
            Point2d(circle.column, circle.row),
            circle.radius,
            circle.score
        );
    }

    // Call Internal merge
    auto mergedInternal = Internal::MergeHoughCircles(internalCircles, centerThreshold, radiusThreshold);

    // Convert back
    std::vector<HoughCircle> merged;
    merged.reserve(mergedInternal.size());
    for (const auto& circle : mergedInternal) {
        merged.push_back(ConvertCircle(circle));
    }

    return merged;
}

Segment2d ClipHoughLineToImage(
    const HoughLine& line,
    int32_t imageWidth,
    int32_t imageHeight
) {
    Internal::HoughLine internalLine(line.rho, line.theta, line.score);
    return Internal::ClipHoughLineToImage(internalLine, imageWidth, imageHeight);
}

bool HoughLinesIntersection(
    const HoughLine& line1,
    const HoughLine& line2,
    Point2d& intersection
) {
    Internal::HoughLine internal1(line1.rho, line1.theta, line1.score);
    Internal::HoughLine internal2(line2.rho, line2.theta, line2.score);

    return Internal::HoughLinesIntersection(internal1, internal2, intersection);
}

bool AreHoughLinesParallel(
    const HoughLine& line1,
    const HoughLine& line2,
    double angleTolerance
) {
    Internal::HoughLine internal1(line1.rho, line1.theta, line1.score);
    Internal::HoughLine internal2(line2.rho, line2.theta, line2.score);

    return Internal::AreHoughLinesParallel(internal1, internal2, angleTolerance);
}

double PointToHoughLineDistance(
    const Point2d& point,
    const HoughLine& line
) {
    Internal::HoughLine internalLine(line.rho, line.theta, line.score);
    return Internal::PointToHoughLineDistance(point, internalLine);
}

} // namespace Qi::Vision::Hough
