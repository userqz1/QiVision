#pragma once

/**
 * @file Hough.h
 * @brief Hough Transform for line and circle detection (Halcon-style API)
 *
 * API Style: void Func(const QImage& in, output..., params...)
 *
 * Halcon reference operators:
 * - hough_lines (standard Hough transform for lines)
 * - hough_lines_dir (gradient-directed Hough)
 * - hough_circles (Hough transform for circles)
 *
 * This module provides public API for Hough transform, wrapping
 * Internal layer implementations.
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/QContour.h>
#include <QiVision/Core/QContourArray.h>
#include <QiVision/Core/Types.h>
#include <QiVision/Display/Draw.h>

#include <cstdint>
#include <string>
#include <vector>

namespace Qi::Vision::Hough {

// =============================================================================
// Result Structures
// =============================================================================

/**
 * @brief Hough line detection result (polar form)
 *
 * Line equation: x*cos(theta) + y*sin(theta) = rho
 */
struct HoughLine {
    double rho = 0.0;      ///< Distance from origin to line (can be negative)
    double theta = 0.0;    ///< Angle in radians [0, PI)
    double score = 0.0;    ///< Accumulator vote count / strength
    Point2d p1;            ///< Optional: line endpoint 1 (for visualization)
    Point2d p2;            ///< Optional: line endpoint 2 (for visualization)

    HoughLine() = default;
    HoughLine(double r, double t, double s = 0.0)
        : rho(r), theta(t), score(s) {}

    /// Convert to Line2d (normalized form: ax + by + c = 0)
    Line2d ToLine2d() const;

    /// Get two points on the line for visualization
    void GetEndPoints(double length, Point2d& pt1, Point2d& pt2) const;
};

/**
 * @brief Hough circle detection result
 */
struct HoughCircle {
    double row = 0.0;      ///< Circle center Y
    double column = 0.0;   ///< Circle center X
    double radius = 0.0;   ///< Circle radius
    double score = 0.0;    ///< Accumulator vote count / strength

    HoughCircle() = default;
    HoughCircle(double r, double c, double rad, double s = 0.0)
        : row(r), column(c), radius(rad), score(s) {}

    /// Get center as Point2d
    Point2d Center() const { return Point2d(column, row); }

    /// Convert to Circle2d
    Circle2d ToCircle2d() const { return Circle2d(column, row, radius); }
};

// =============================================================================
// Standard Hough Line Transform
// =============================================================================

/**
 * @brief Standard Hough Transform for line detection
 *
 * Detects infinite lines in a binary edge image using accumulator voting.
 * Lines are returned in polar form (rho, theta).
 *
 * @param edgeImage Binary edge image (non-zero pixels are edge points)
 * @param lines [out] Detected lines sorted by score (descending)
 * @param rhoResolution Distance resolution in pixels (default: 1.0)
 * @param thetaResolution Angle resolution in radians (default: ~1 degree)
 * @param threshold Minimum accumulator votes (0 = auto)
 * @param maxLines Maximum number of lines to return (0 = unlimited)
 *
 * @code
 * QImage edgeImage;
 * EdgesImage(image, edgeImage, "canny", 1.0, 20.0, 40.0);
 *
 * std::vector<HoughLine> lines;
 * HoughLines(edgeImage, lines, 1.0, 0.01745, 100, 10);
 *
 * for (const auto& line : lines) {
 *     std::cout << "Line: rho=" << line.rho << ", theta=" << line.theta << std::endl;
 * }
 * @endcode
 */
void HoughLines(
    const QImage& edgeImage,
    std::vector<HoughLine>& lines,
    double rhoResolution = 1.0,
    double thetaResolution = 0.01745329,  // ~1 degree
    int32_t threshold = 100,
    int32_t maxLines = 0
);

/**
 * @brief Hough Transform from edge point list
 *
 * More efficient when edge points are already extracted.
 *
 * @param points List of edge points
 * @param imageWidth Image width (for rho range calculation)
 * @param imageHeight Image height
 * @param lines [out] Detected lines
 * @param rhoResolution Distance resolution
 * @param thetaResolution Angle resolution
 * @param threshold Minimum votes
 * @param maxLines Maximum lines
 */
void HoughLines(
    const std::vector<Point2d>& points,
    int32_t imageWidth,
    int32_t imageHeight,
    std::vector<HoughLine>& lines,
    double rhoResolution = 1.0,
    double thetaResolution = 0.01745329,
    int32_t threshold = 100,
    int32_t maxLines = 0
);

// =============================================================================
// Probabilistic Hough Transform
// =============================================================================

/**
 * @brief Probabilistic Hough Transform for line segment detection
 *
 * Faster than standard Hough, returns actual line segments instead of
 * infinite lines. Good for detecting finite line segments in images.
 *
 * Based on: Matas, Galambos, Kittler (2000) "Robust Detection of Lines Using
 * the Progressive Probabilistic Hough Transform"
 *
 * @param edgeImage Binary edge image
 * @param segments [out] Detected line segments
 * @param rhoResolution Distance resolution (default: 1.0)
 * @param thetaResolution Angle resolution (default: ~1 degree)
 * @param threshold Minimum accumulator votes
 * @param minLineLength Minimum length of line segment (pixels)
 * @param maxLineGap Maximum gap between points on same line (pixels)
 *
 * @code
 * std::vector<Segment2d> segments;
 * HoughLinesP(edgeImage, segments, 1.0, 0.01745, 50, 30.0, 10.0);
 *
 * for (const auto& seg : segments) {
 *     Draw::Line(displayImage, seg.p1, seg.p2, Scalar::Green());
 * }
 * @endcode
 */
void HoughLinesP(
    const QImage& edgeImage,
    std::vector<Segment2d>& segments,
    double rhoResolution = 1.0,
    double thetaResolution = 0.01745329,
    int32_t threshold = 50,
    double minLineLength = 30.0,
    double maxLineGap = 10.0
);

// =============================================================================
// Hough Lines from Contours (XLD)
// =============================================================================

/**
 * @brief Detect lines from contours using Hough Transform
 *
 * Operates on XLD contours rather than binary edge images.
 * Useful for detecting dominant line orientations in contour data.
 *
 * @param contours Input contour array
 * @param lines [out] Detected lines as Line2d
 * @param threshold Minimum votes per contour point (default: 5)
 * @param angleResolution Angle resolution in radians
 * @param maxLines Maximum lines to return (0 = unlimited)
 *
 * @code
 * QContourArray contours;
 * EdgesSubPix(image, contours, "canny", 1.0, 20.0, 40.0);
 *
 * std::vector<Line2d> lines;
 * HoughLinesXld(contours, lines, 5, 0.01745, 10);
 * @endcode
 */
void HoughLinesXld(
    const QContourArray& contours,
    std::vector<Line2d>& lines,
    int32_t threshold = 5,
    double angleResolution = 0.01745329,
    int32_t maxLines = 0
);

// =============================================================================
// Hough Circle Transform
// =============================================================================

/**
 * @brief Hough Circle Transform (gradient-based)
 *
 * Detects circles using gradient direction voting.
 * More efficient than standard 3D Hough.
 *
 * @param edgeImage Binary edge image or grayscale (edge detection done internally)
 * @param circles [out] Detected circles sorted by score (descending)
 * @param dp Inverse ratio of accumulator resolution to image (default: 1.0)
 * @param minDist Minimum distance between circle centers (pixels)
 * @param param1 Higher Canny edge threshold (if using internal edge detection)
 * @param param2 Accumulator threshold for circle centers
 * @param minRadius Minimum circle radius (0 = auto)
 * @param maxRadius Maximum circle radius (0 = use image diagonal)
 *
 * @code
 * std::vector<HoughCircle> circles;
 * HoughCircles(edgeImage, circles, 1.0, 20.0, 100.0, 30.0, 10, 100);
 *
 * for (const auto& circle : circles) {
 *     Draw::Circle(displayImage, Point2d(circle.column, circle.row),
 *                  circle.radius, Scalar::Green());
 * }
 * @endcode
 */
void HoughCircles(
    const QImage& edgeImage,
    std::vector<HoughCircle>& circles,
    double dp = 1.0,
    double minDist = 20.0,
    double param1 = 100.0,
    double param2 = 30.0,
    int32_t minRadius = 0,
    int32_t maxRadius = 0
);

/**
 * @brief Hough Circle Transform with pre-computed gradient
 *
 * Use this overload when you have already computed gradient information.
 *
 * @param edgeImage Binary edge image
 * @param gradientX X-component of gradient
 * @param gradientY Y-component of gradient
 * @param circles [out] Detected circles
 * @param minDist Minimum distance between centers
 * @param threshold Accumulator threshold
 * @param minRadius Minimum radius
 * @param maxRadius Maximum radius
 */
void HoughCircles(
    const QImage& edgeImage,
    const QImage& gradientX,
    const QImage& gradientY,
    std::vector<HoughCircle>& circles,
    double minDist = 20.0,
    double threshold = 30.0,
    int32_t minRadius = 0,
    int32_t maxRadius = 0
);

// =============================================================================
// Hough Circles from Contours (XLD)
// =============================================================================

/**
 * @brief Detect circles from contours using Hough Transform
 *
 * @param contours Input contour array
 * @param circles [out] Detected circles as Circle2d
 * @param minRadius Minimum circle radius
 * @param maxRadius Maximum circle radius
 * @param threshold Minimum votes (default: 5)
 * @param radiusResolution Radius resolution (default: 1.0)
 *
 * @code
 * std::vector<Circle2d> circles;
 * HoughCirclesXld(contours, circles, 10.0, 100.0, 5, 1.0);
 * @endcode
 */
void HoughCirclesXld(
    const QContourArray& contours,
    std::vector<Circle2d>& circles,
    double minRadius,
    double maxRadius,
    int32_t threshold = 5,
    double radiusResolution = 1.0
);

// =============================================================================
// Visualization Functions
// =============================================================================

/**
 * @brief Draw detected Hough lines on image
 *
 * @param image Image to draw on (modified in place)
 * @param lines Lines to draw
 * @param color Line color (default: green)
 * @param thickness Line thickness (default: 1)
 *
 * @code
 * std::vector<HoughLine> lines;
 * HoughLines(edgeImage, lines, 1.0, 0.01745, 100, 10);
 * DrawHoughLines(displayImage, lines, Scalar::Green(), 2);
 * @endcode
 */
void DrawHoughLines(
    QImage& image,
    const std::vector<HoughLine>& lines,
    const Scalar& color = Scalar::Green(),
    int32_t thickness = 1
);

/**
 * @brief Draw detected Hough circles on image
 *
 * @param image Image to draw on (modified in place)
 * @param circles Circles to draw
 * @param color Circle color (default: green)
 * @param thickness Line thickness (default: 1)
 *
 * @code
 * std::vector<HoughCircle> circles;
 * HoughCircles(edgeImage, circles, 1.0, 20.0, 100.0, 30.0, 10, 100);
 * DrawHoughCircles(displayImage, circles, Scalar::Red(), 2);
 * @endcode
 */
void DrawHoughCircles(
    QImage& image,
    const std::vector<HoughCircle>& circles,
    const Scalar& color = Scalar::Green(),
    int32_t thickness = 1
);

// =============================================================================
// Advanced Options
// =============================================================================

/**
 * @brief Parameters for Hough line transform
 */
struct HoughLineParams {
    double rhoResolution = 1.0;         ///< Distance resolution (pixels)
    double thetaResolution = 0.01745329; ///< Angle resolution (radians)
    double threshold = 0.3;             ///< Threshold (ratio if <1, absolute if >=1)
    bool thresholdIsRatio = true;       ///< Interpret threshold as ratio of max
    int32_t maxLines = 0;               ///< Maximum lines (0 = unlimited)
    double minDistance = 10.0;          ///< Min distance between lines (NMS)
    bool suppressOverlapping = true;    ///< Merge similar lines

    /// Create with default settings
    static HoughLineParams Default() {
        return HoughLineParams();
    }

    /// Create for fine detection (smaller resolution)
    static HoughLineParams Fine() {
        HoughLineParams p;
        p.rhoResolution = 0.5;
        p.thetaResolution = 0.00872665;  // 0.5 degree
        return p;
    }
};

/**
 * @brief Parameters for probabilistic Hough line transform
 */
struct HoughLinePParams {
    double rhoResolution = 1.0;
    double thetaResolution = 0.01745329;
    int32_t threshold = 50;
    double minLineLength = 30.0;
    double maxLineGap = 10.0;
    int32_t maxLines = 0;

    static HoughLinePParams Default() {
        return HoughLinePParams();
    }

    /// Create for short line detection
    static HoughLinePParams ShortLines(double minLength = 10.0) {
        HoughLinePParams p;
        p.minLineLength = minLength;
        p.threshold = 30;
        return p;
    }
};

/**
 * @brief Parameters for Hough circle transform
 */
struct HoughCircleParams {
    double dp = 1.0;                    ///< Accumulator resolution ratio
    double minDist = 20.0;              ///< Min distance between centers
    double param1 = 100.0;              ///< Canny high threshold
    double param2 = 50.0;               ///< Accumulator threshold
    int32_t minRadius = 5;              ///< Min radius
    int32_t maxRadius = 0;              ///< Max radius (0 = auto)
    int32_t maxCircles = 0;             ///< Max circles (0 = unlimited)

    static HoughCircleParams Default() {
        return HoughCircleParams();
    }

    /// Create for small circles
    static HoughCircleParams SmallCircles(int32_t minR = 5, int32_t maxR = 50) {
        HoughCircleParams p;
        p.minRadius = minR;
        p.maxRadius = maxR;
        p.minDist = static_cast<double>(minR);
        return p;
    }
};

/**
 * @brief Hough lines with full parameter control
 *
 * @param edgeImage Binary edge image
 * @param lines [out] Detected lines
 * @param params Detection parameters
 */
void DetectHoughLines(
    const QImage& edgeImage,
    std::vector<HoughLine>& lines,
    const HoughLineParams& params
);

/**
 * @brief Probabilistic Hough with full parameter control
 *
 * @param edgeImage Binary edge image
 * @param segments [out] Detected segments
 * @param params Detection parameters
 */
void DetectHoughLinesP(
    const QImage& edgeImage,
    std::vector<Segment2d>& segments,
    const HoughLinePParams& params
);

/**
 * @brief Hough circles with full parameter control
 *
 * @param edgeImage Binary edge image
 * @param circles [out] Detected circles
 * @param params Detection parameters
 */
void DetectHoughCircles(
    const QImage& edgeImage,
    std::vector<HoughCircle>& circles,
    const HoughCircleParams& params
);

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Merge similar Hough lines (non-maximum suppression)
 *
 * @param lines Input lines
 * @param rhoThreshold Maximum rho difference for merging
 * @param thetaThreshold Maximum theta difference for merging (radians)
 * @return Merged lines
 */
std::vector<HoughLine> MergeHoughLines(
    const std::vector<HoughLine>& lines,
    double rhoThreshold = 10.0,
    double thetaThreshold = 0.1
);

/**
 * @brief Merge similar Hough circles (non-maximum suppression)
 *
 * @param circles Input circles
 * @param centerThreshold Maximum center distance for merging
 * @param radiusThreshold Maximum radius difference for merging
 * @return Merged circles
 */
std::vector<HoughCircle> MergeHoughCircles(
    const std::vector<HoughCircle>& circles,
    double centerThreshold = 10.0,
    double radiusThreshold = 5.0
);

/**
 * @brief Clip Hough line to image bounds
 *
 * Returns a segment representing the visible portion of the line.
 *
 * @param line Hough line
 * @param imageWidth Image width
 * @param imageHeight Image height
 * @return Segment clipped to image, or zero-length segment if no intersection
 */
Segment2d ClipHoughLineToImage(
    const HoughLine& line,
    int32_t imageWidth,
    int32_t imageHeight
);

/**
 * @brief Get intersection of two Hough lines
 *
 * @param line1 First line
 * @param line2 Second line
 * @param intersection [out] Intersection point
 * @return True if lines intersect (not parallel)
 */
bool HoughLinesIntersection(
    const HoughLine& line1,
    const HoughLine& line2,
    Point2d& intersection
);

/**
 * @brief Check if two Hough lines are approximately parallel
 *
 * @param line1 First line
 * @param line2 Second line
 * @param angleTolerance Maximum angle difference (radians)
 * @return True if parallel within tolerance
 */
bool AreHoughLinesParallel(
    const HoughLine& line1,
    const HoughLine& line2,
    double angleTolerance = 0.05
);

/**
 * @brief Calculate distance from point to Hough line
 *
 * @param point Query point
 * @param line Hough line
 * @return Signed perpendicular distance
 */
double PointToHoughLineDistance(
    const Point2d& point,
    const HoughLine& line
);

} // namespace Qi::Vision::Hough
