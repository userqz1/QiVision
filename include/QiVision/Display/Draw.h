#pragma once

/**
 * @file Draw.h
 * @brief Drawing functions for visualization (Halcon-style API)
 *
 * Provides drawing primitives for debugging and result visualization.
 * Supports grayscale and RGB images.
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/Types.h>
#include <QiVision/Core/QContour.h>
#include <QiVision/Core/QRegion.h>
#include <QiVision/Matching/MatchTypes.h>

#include <cmath>
#include <vector>
#include <string>

namespace Qi::Vision {

// Forward declarations for Metrology
namespace Measure {
    class MeasureRectangle2;
    class MeasureArc;
    class MeasureConcentricCircles;
    class MetrologyModel;
    class MetrologyObject;
    struct MetrologyLineResult;
    struct MetrologyCircleResult;
    struct MetrologyEllipseResult;
    struct MetrologyRectangle2Result;
}

// Forward declarations for Matching
namespace Matching {
    class ShapeModel;
}

/**
 * @brief Scalar for drawing (RGB)
 */
struct Scalar {
    uint8_t r = 255;
    uint8_t g = 255;
    uint8_t b = 255;

    Scalar() = default;
    Scalar(uint8_t r_, uint8_t g_, uint8_t b_) : r(r_), g(g_), b(b_) {}
    Scalar(uint8_t gray) : r(gray), g(gray), b(gray) {}

    // Predefined colors
    static Scalar Red()     { return Scalar(255, 0, 0); }
    static Scalar Green()   { return Scalar(0, 255, 0); }
    static Scalar Blue()    { return Scalar(0, 0, 255); }
    static Scalar Yellow()  { return Scalar(255, 255, 0); }
    static Scalar Cyan()    { return Scalar(0, 255, 255); }
    static Scalar Magenta() { return Scalar(255, 0, 255); }
    static Scalar White()   { return Scalar(255, 255, 255); }
    static Scalar Black()   { return Scalar(0, 0, 0); }
    static Scalar Orange()  { return Scalar(255, 165, 0); }
    static Scalar Pink()    { return Scalar(255, 192, 203); }
    static Scalar Purple()  { return Scalar(128, 0, 128); }
    static Scalar Brown()   { return Scalar(139, 69, 19); }
    static Scalar Gray()    { return Scalar(128, 128, 128); }
    static Scalar LightGray() { return Scalar(192, 192, 192); }
    static Scalar DarkGray()  { return Scalar(64, 64, 64); }

    // Create color from HSV (h: 0-360, s: 0-1, v: 0-1)
    static Scalar FromHSV(double h, double s, double v);

    // Create color with alpha blend
    Scalar Blend(const Scalar& other, double alpha) const;
};

/**
 * @brief Drawing style parameters
 */
struct DrawStyle {
    Scalar color = Scalar::Green();
    int32_t thickness = 1;      // Line thickness (pixels)
    int32_t fontSize = 1;       // Font scale (1 = 5x7 base)
    bool filled = false;        // Fill shapes
    bool antiAlias = false;     // Anti-aliased drawing (slower)

    DrawStyle() = default;
    DrawStyle(const Scalar& c) : color(c) {}
    DrawStyle(const Scalar& c, int32_t thick) : color(c), thickness(thick) {}
    DrawStyle(const Scalar& c, int32_t thick, int32_t font)
        : color(c), thickness(thick), fontSize(font) {}
};

/**
 * @brief Drawing utility class (Halcon-style static functions)
 */
class Draw {
public:
    // =========================================================================
    // Basic Primitives
    // =========================================================================

    /** @brief Draw a single pixel */
    static void Pixel(QImage& image, int32_t x, int32_t y, const Scalar& color);

    /** @brief Draw pixel with alpha blending */
    static void PixelAlpha(QImage& image, int32_t x, int32_t y, const Scalar& color, double alpha);

    /** @brief Draw a line (Bresenham algorithm) */
    static void Line(QImage& image, int32_t x1, int32_t y1, int32_t x2, int32_t y2,
                     const Scalar& color, int32_t thickness = 1);

    /** @brief Draw a line from Point2d */
    static void Line(QImage& image, const Point2d& p1, const Point2d& p2,
                     const Scalar& color, int32_t thickness = 1);

    /** @brief Draw anti-aliased line (Wu's algorithm) */
    static void LineAA(QImage& image, double x1, double y1, double x2, double y2,
                       const Scalar& color);

    /** @brief Draw a rectangle outline */
    static void Rectangle(QImage& image, int32_t x, int32_t y, int32_t width, int32_t height,
                          const Scalar& color, int32_t thickness = 1);

    /** @brief Draw a rectangle from Rect2i */
    static void Rectangle(QImage& image, const Rect2i& rect,
                          const Scalar& color, int32_t thickness = 1);

    /** @brief Draw a filled rectangle */
    static void FilledRectangle(QImage& image, const Rect2i& rect, const Scalar& color);

    /** @brief Draw a circle outline */
    static void Circle(QImage& image, int32_t cx, int32_t cy, int32_t radius,
                       const Scalar& color, int32_t thickness = 1);

    /** @brief Draw a circle from Point2d center */
    static void Circle(QImage& image, const Point2d& center, double radius,
                       const Scalar& color, int32_t thickness = 1);

    /** @brief Draw a filled circle */
    static void FilledCircle(QImage& image, int32_t cx, int32_t cy, int32_t radius,
                             const Scalar& color);

    /** @brief Draw a filled circle from Point2d */
    static void FilledCircle(QImage& image, const Point2d& center, double radius,
                             const Scalar& color);

    /** @brief Draw an ellipse outline */
    static void Ellipse(QImage& image, int32_t cx, int32_t cy,
                        int32_t radiusX, int32_t radiusY,
                        const Scalar& color, int32_t thickness = 1);

    /** @brief Draw a rotated ellipse */
    static void Ellipse(QImage& image, const Point2d& center,
                        double radiusX, double radiusY, double angle,
                        const Scalar& color, int32_t thickness = 1);

    /** @brief Draw a filled ellipse */
    static void FilledEllipse(QImage& image, int32_t cx, int32_t cy,
                              int32_t radiusX, int32_t radiusY, const Scalar& color);

    /** @brief Draw a cross marker */
    static void Cross(QImage& image, int32_t cx, int32_t cy, int32_t size,
                      const Scalar& color, int32_t thickness = 1);

    /** @brief Draw a cross at Point2d */
    static void Cross(QImage& image, const Point2d& center, int32_t size,
                      const Scalar& color, int32_t thickness = 1);

    /** @brief Draw a rotated cross marker */
    static void Cross(QImage& image, const Point2d& center, int32_t size,
                      double angle, const Scalar& color, int32_t thickness = 1);

    /** @brief Draw an arrow */
    static void Arrow(QImage& image, const Point2d& from, const Point2d& to,
                      const Scalar& color, int32_t thickness = 1, double arrowSize = 10.0);

    // =========================================================================
    // Polylines and Contours
    // =========================================================================

    /** @brief Draw a polyline (connected line segments) */
    static void Polyline(QImage& image, const std::vector<Point2d>& points,
                         const Scalar& color, int32_t thickness = 1, bool closed = false);

    /** @brief Draw a filled polygon */
    static void FilledPolygon(QImage& image, const std::vector<Point2d>& points,
                              const Scalar& color);

    /** @brief Draw a rotated rectangle */
    static void RotatedRectangle(QImage& image, const Point2d& center,
                                  double width, double height, double angle,
                                  const Scalar& color, int32_t thickness = 1);

    /** @brief Draw an arc */
    static void Arc(QImage& image, const Point2d& center, double radius,
                    double startAngle, double endAngle,
                    const Scalar& color, int32_t thickness = 1);

    // =========================================================================
    // Match Result Visualization
    // =========================================================================

    /** @brief Draw match result with cross marker and angle indicator */
    static void MatchResult(QImage& image, const Matching::MatchResult& match,
                            const Scalar& color, int32_t markerSize = 20);

    /** @brief Draw match result with model contour */
    static void MatchResultWithContour(QImage& image, const Matching::MatchResult& match,
                                        const std::vector<Point2d>& modelContour,
                                        const Scalar& color, int32_t thickness = 1);

    /** @brief Draw multiple match results */
    static void MatchResults(QImage& image, const std::vector<Matching::MatchResult>& matches,
                             const Scalar& color, int32_t markerSize = 20);

    /**
     * @brief Draw shape matching results with quality coloring (Halcon-style)
     *
     * Similar to Halcon's dev_display_shape_matching_results.
     * Draws model contour at match position with each segment colored by match quality:
     * - Green: matched points (gradient direction matches expected)
     * - Red: unmatched points (no gradient or direction mismatch)
     *
     * @param image Target image (also used for gradient comparison)
     * @param model ShapeModel used for matching
     * @param matches Match results from FindShapeModel
     * @param matchedColor Scalar for matched segments (default green)
     * @param unmatchedColor Scalar for unmatched segments (default red)
     * @param thickness Line thickness
     * @param threshold Quality threshold for "matched" (default 0.5)
     */
    static void ShapeMatchingResults(QImage& image,
                                      const Matching::ShapeModel& model,
                                      const std::vector<Matching::MatchResult>& matches,
                                      const Scalar& matchedColor = Scalar::Green(),
                                      const Scalar& unmatchedColor = Scalar::Red(),
                                      int32_t thickness = 1,
                                      double threshold = 0.5);

    /**
     * @brief Draw matched contour with quality coloring
     *
     * @param image Target image
     * @param contour MatchedContour from GetShapeModelContoursWithQuality
     * @param matchedColor Scalar for matched segments
     * @param unmatchedColor Scalar for unmatched segments
     * @param thickness Line thickness
     */
    static void MatchedContour(QImage& image,
                                const Matching::MatchedContour& contour,
                                const Scalar& matchedColor = Scalar::Green(),
                                const Scalar& unmatchedColor = Scalar::Red(),
                                int32_t thickness = 1);

    /**
     * @brief Draw matched contour segment with quality coloring (internal use)
     *
     * Unlike MatchedContour, this does not auto-close the contour.
     *
     * @param image Target image
     * @param contour MatchedContour with points
     * @param matchedColor Scalar for matched segments
     * @param unmatchedColor Scalar for unmatched segments
     * @param thickness Line thickness
     * @param autoClose Whether to close the contour
     */
    static void MatchedContourSegment(QImage& image,
                                       const Matching::MatchedContour& contour,
                                       const Scalar& matchedColor,
                                       const Scalar& unmatchedColor,
                                       int32_t thickness,
                                       bool autoClose);

    // =========================================================================
    // Metrology Visualization (Halcon-style)
    // =========================================================================

    /**
     * @brief Draw a MeasureRectangle2 (caliper handle)
     * @param image Target image
     * @param handle MeasureRectangle2 to draw
     * @param color Drawing color
     * @param thickness Line thickness
     */
    static void MeasureRect(QImage& image, const Measure::MeasureRectangle2& handle,
                            const Scalar& color, int32_t thickness = 1);

    /**
     * @brief Draw multiple MeasureRectangle2 handles
     */
    static void MeasureRects(QImage& image,
                             const std::vector<Measure::MeasureRectangle2>& handles,
                             const Scalar& color, int32_t thickness = 1);

    /**
     * @brief Draw a MeasureArc (arc caliper handle)
     * @param image Target image
     * @param handle MeasureArc to draw
     * @param color Drawing color
     * @param thickness Line thickness
     */
    static void MeasureArc(QImage& image, const Measure::MeasureArc& handle,
                           const Scalar& color, int32_t thickness = 1);

    /**
     * @brief Draw a MeasureConcentricCircles (radial caliper handle)
     * @param image Target image
     * @param handle MeasureConcentricCircles to draw
     * @param color Drawing color
     * @param thickness Line thickness
     */
    static void MeasureConcentric(QImage& image, const Measure::MeasureConcentricCircles& handle,
                                  const Scalar& color, int32_t thickness = 1);

    /**
     * @brief Draw a QContour
     * @param image Target image
     * @param contour Contour to draw
     * @param color Drawing color
     * @param thickness Line thickness
     */
    static void Contour(QImage& image, const QContour& contour,
                        const Scalar& color, int32_t thickness = 1);

    // =========================================================================
    // Region Drawing
    // =========================================================================

    /**
     * @brief Fill a region with color
     *
     * Equivalent to Halcon's paint_region operator.
     *
     * @param image Target image (must be RGB or will be drawn as grayscale)
     * @param region Region to fill
     * @param color Fill color
     */
    static void Region(QImage& image, const QRegion& region, const Scalar& color);

    /**
     * @brief Draw region boundary/contour
     *
     * Draws only the outline of the region, not filled.
     *
     * @param image Target image
     * @param region Region whose boundary to draw
     * @param color Line color
     * @param thickness Line thickness (1 = single pixel boundary)
     */
    static void RegionContour(QImage& image, const QRegion& region,
                              const Scalar& color, int32_t thickness = 1);

    /**
     * @brief Fill a region with semi-transparent color overlay
     *
     * Blends the color with existing image pixels.
     *
     * @param image Target image
     * @param region Region to fill
     * @param color Fill color
     * @param alpha Transparency (0.0 = fully transparent, 1.0 = fully opaque)
     */
    static void RegionAlpha(QImage& image, const QRegion& region,
                            const Scalar& color, double alpha);

    /**
     * @brief Draw edge points with cross markers
     * @param image Target image
     * @param points Edge points to draw
     * @param color Drawing color
     * @param markerSize Cross marker size
     */
    static void EdgePoints(QImage& image, const std::vector<Point2d>& points,
                           const Scalar& color, int32_t markerSize = 5);

    /**
     * @brief Draw edge points with weight-based coloring (auto-detects weight type)
     * @param image Target image
     * @param points Edge points to draw
     * @param weights Point weights
     *   - Binary (RANSAC/Tukey): green (inlier >=0.5), red (outlier <0.5)
     *   - Continuous (Huber): green (>=0.8), yellow (0.3-0.8), red (<0.3)
     * @param markerSize Filled circle marker size
     */
    static void EdgePointsWeighted(QImage& image, const std::vector<Point2d>& points,
                                   const std::vector<double>& weights, int32_t markerSize = 3);

    /**
     * @brief Draw Metrology line result
     * @param image Target image
     * @param result Line measurement result
     * @param color Drawing color
     * @param thickness Line thickness
     */
    static void MetrologyLine(QImage& image, const Measure::MetrologyLineResult& result,
                              const Scalar& color, int32_t thickness = 2);

    /**
     * @brief Draw Metrology circle result
     * @param image Target image
     * @param result Circle measurement result
     * @param color Drawing color
     * @param thickness Line thickness
     */
    static void MetrologyCircle(QImage& image, const Measure::MetrologyCircleResult& result,
                                const Scalar& color, int32_t thickness = 2);

    /**
     * @brief Draw Metrology ellipse result
     * @param image Target image
     * @param result Ellipse measurement result
     * @param color Drawing color
     * @param thickness Line thickness
     */
    static void MetrologyEllipse(QImage& image, const Measure::MetrologyEllipseResult& result,
                                 const Scalar& color, int32_t thickness = 2);

    /**
     * @brief Draw Metrology rectangle result
     * @param image Target image
     * @param result Rectangle measurement result
     * @param color Drawing color
     * @param thickness Line thickness
     */
    static void MetrologyRectangle(QImage& image, const Measure::MetrologyRectangle2Result& result,
                                   const Scalar& color, int32_t thickness = 2);

    /**
     * @brief Draw complete Metrology model visualization
     *
     * Draws:
     * - Object contours (measurement regions)
     * - Caliper handles
     * - Detected edge points
     * - Fitted results
     *
     * @param image Target image
     * @param model MetrologyModel after Apply()
     * @param objectColor Scalar for object outlines and calipers
     * @param resultColor Scalar for fitted results
     * @param pointColor Scalar for edge points
     * @param drawCalipers Whether to draw caliper handles
     * @param drawPoints Whether to draw edge points
     */
    static void MetrologyModelResult(QImage& image, const Measure::MetrologyModel& model,
                                     const Scalar& objectColor = Scalar::Cyan(),
                                     const Scalar& resultColor = Scalar::Green(),
                                     const Scalar& pointColor = Scalar::Red(),
                                     bool drawCalipers = true,
                                     bool drawPoints = true);

    // =========================================================================
    // Text Drawing
    // =========================================================================

    /**
     * @brief Draw text (supports A-Z, a-z, 0-9, punctuation)
     * @param scale Font scale (1 = 5x7 base, 2 = 10x14, etc.)
     */
    static void Text(QImage& image, int32_t x, int32_t y, const std::string& text,
                     const Scalar& color, int32_t scale = 1);

    /**
     * @brief Draw text with DrawStyle
     */
    static void Text(QImage& image, int32_t x, int32_t y, const std::string& text,
                     const DrawStyle& style);

    /**
     * @brief Get text dimensions
     * @return (width, height) in pixels
     */
    static std::pair<int32_t, int32_t> TextSize(const std::string& text, int32_t scale = 1);

    // =========================================================================
    // Utility
    // =========================================================================

    /** @brief Generate N distinct colors for visualization */
    static std::vector<Scalar> GenerateColors(int32_t n);
};

// =============================================================================
// Inline Implementations
// =============================================================================

inline Scalar Scalar::FromHSV(double h, double s, double v) {
    double c = v * s;
    double x = c * (1.0 - std::fabs(std::fmod(h / 60.0, 2.0) - 1.0));
    double m = v - c;

    double r, g, b;
    if (h < 60)       { r = c; g = x; b = 0; }
    else if (h < 120) { r = x; g = c; b = 0; }
    else if (h < 180) { r = 0; g = c; b = x; }
    else if (h < 240) { r = 0; g = x; b = c; }
    else if (h < 300) { r = x; g = 0; b = c; }
    else              { r = c; g = 0; b = x; }

    return Scalar(
        static_cast<uint8_t>((r + m) * 255),
        static_cast<uint8_t>((g + m) * 255),
        static_cast<uint8_t>((b + m) * 255)
    );
}

inline Scalar Scalar::Blend(const Scalar& other, double alpha) const {
    return Scalar(
        static_cast<uint8_t>(r * (1 - alpha) + other.r * alpha),
        static_cast<uint8_t>(g * (1 - alpha) + other.g * alpha),
        static_cast<uint8_t>(b * (1 - alpha) + other.b * alpha)
    );
}

inline void Draw::Pixel(QImage& image, int32_t x, int32_t y, const Scalar& color) {
    if (x < 0 || x >= image.Width() || y < 0 || y >= image.Height()) {
        return;
    }

    uint8_t* data = static_cast<uint8_t*>(image.Data());
    size_t stride = image.Stride();
    int channels = image.Channels();

    if (channels == 1) {
        data[y * stride + x] = static_cast<uint8_t>(0.299 * color.r + 0.587 * color.g + 0.114 * color.b);
    } else if (channels == 3) {
        uint8_t* px = data + y * stride + x * 3;
        px[0] = color.r;
        px[1] = color.g;
        px[2] = color.b;
    }
}

inline void Draw::PixelAlpha(QImage& image, int32_t x, int32_t y, const Scalar& color, double alpha) {
    if (x < 0 || x >= image.Width() || y < 0 || y >= image.Height() || alpha <= 0) {
        return;
    }
    if (alpha >= 1.0) {
        Pixel(image, x, y, color);
        return;
    }

    uint8_t* data = static_cast<uint8_t*>(image.Data());
    size_t stride = image.Stride();
    int channels = image.Channels();

    if (channels == 1) {
        uint8_t newVal = static_cast<uint8_t>(0.299 * color.r + 0.587 * color.g + 0.114 * color.b);
        uint8_t& pixel = data[y * stride + x];
        pixel = static_cast<uint8_t>(pixel * (1 - alpha) + newVal * alpha);
    } else if (channels == 3) {
        uint8_t* px = data + y * stride + x * 3;
        px[0] = static_cast<uint8_t>(px[0] * (1 - alpha) + color.r * alpha);
        px[1] = static_cast<uint8_t>(px[1] * (1 - alpha) + color.g * alpha);
        px[2] = static_cast<uint8_t>(px[2] * (1 - alpha) + color.b * alpha);
    }
}

inline void Draw::Cross(QImage& image, int32_t cx, int32_t cy, int32_t size,
                        const Scalar& color, int32_t thickness) {
    Line(image, cx - size, cy, cx + size, cy, color, thickness);
    Line(image, cx, cy - size, cx, cy + size, color, thickness);
}

inline void Draw::Cross(QImage& image, const Point2d& center, int32_t size,
                        const Scalar& color, int32_t thickness) {
    Cross(image, static_cast<int32_t>(center.x + 0.5), static_cast<int32_t>(center.y + 0.5),
          size, color, thickness);
}

inline void Draw::Cross(QImage& image, const Point2d& center, int32_t size,
                        double angle, const Scalar& color, int32_t thickness) {
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);
    double s = static_cast<double>(size);

    Point2d h1{center.x - s * cosA, center.y - s * sinA};
    Point2d h2{center.x + s * cosA, center.y + s * sinA};
    Line(image, h1, h2, color, thickness);

    Point2d v1{center.x + s * sinA, center.y - s * cosA};
    Point2d v2{center.x - s * sinA, center.y + s * cosA};
    Line(image, v1, v2, color, thickness);
}

inline void Draw::Line(QImage& image, const Point2d& p1, const Point2d& p2,
                       const Scalar& color, int32_t thickness) {
    Line(image, static_cast<int32_t>(p1.x + 0.5), static_cast<int32_t>(p1.y + 0.5),
         static_cast<int32_t>(p2.x + 0.5), static_cast<int32_t>(p2.y + 0.5),
         color, thickness);
}

inline void Draw::Rectangle(QImage& image, const Rect2i& rect,
                            const Scalar& color, int32_t thickness) {
    Rectangle(image, rect.x, rect.y, rect.width, rect.height, color, thickness);
}

inline void Draw::Circle(QImage& image, const Point2d& center, double radius,
                         const Scalar& color, int32_t thickness) {
    Circle(image, static_cast<int32_t>(center.x + 0.5), static_cast<int32_t>(center.y + 0.5),
           static_cast<int32_t>(radius + 0.5), color, thickness);
}

inline void Draw::FilledCircle(QImage& image, const Point2d& center, double radius,
                               const Scalar& color) {
    FilledCircle(image, static_cast<int32_t>(center.x + 0.5),
                 static_cast<int32_t>(center.y + 0.5),
                 static_cast<int32_t>(radius + 0.5), color);
}

inline void Draw::Text(QImage& image, int32_t x, int32_t y, const std::string& text,
                       const DrawStyle& style) {
    Text(image, x, y, text, style.color, style.fontSize);
}

inline std::vector<Scalar> Draw::GenerateColors(int32_t n) {
    std::vector<Scalar> colors;
    colors.reserve(n);
    for (int32_t i = 0; i < n; ++i) {
        double hue = (360.0 * i) / n;
        colors.push_back(Scalar::FromHSV(hue, 0.8, 0.9));
    }
    return colors;
}

} // namespace Qi::Vision
