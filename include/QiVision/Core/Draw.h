#pragma once

/**
 * @file Draw.h
 * @brief Simple drawing functions for visualization
 *
 * Provides basic drawing primitives for debugging and result visualization.
 * All functions draw on grayscale or RGB images.
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/Types.h>
#include <QiVision/Matching/MatchTypes.h>

#include <cmath>
#include <vector>
#include <string>

namespace Qi::Vision {

/**
 * @brief Color for drawing (RGB)
 */
struct Color {
    uint8_t r = 255;
    uint8_t g = 255;
    uint8_t b = 255;

    Color() = default;
    Color(uint8_t r_, uint8_t g_, uint8_t b_) : r(r_), g(g_), b(b_) {}
    Color(uint8_t gray) : r(gray), g(gray), b(gray) {}

    // Predefined colors
    static Color Red()     { return Color(255, 0, 0); }
    static Color Green()   { return Color(0, 255, 0); }
    static Color Blue()    { return Color(0, 0, 255); }
    static Color Yellow()  { return Color(255, 255, 0); }
    static Color Cyan()    { return Color(0, 255, 255); }
    static Color Magenta() { return Color(255, 0, 255); }
    static Color White()   { return Color(255, 255, 255); }
    static Color Black()   { return Color(0, 0, 0); }
    static Color Orange()  { return Color(255, 165, 0); }
};

/**
 * @brief Drawing utility class
 */
class Draw {
public:
    // =========================================================================
    // Basic Primitives
    // =========================================================================

    /**
     * @brief Draw a single pixel
     */
    static void Pixel(QImage& image, int32_t x, int32_t y, const Color& color);

    /**
     * @brief Draw a line (Bresenham algorithm)
     */
    static void Line(QImage& image, int32_t x1, int32_t y1, int32_t x2, int32_t y2,
                     const Color& color, int32_t thickness = 1);

    /**
     * @brief Draw a line from Point2d
     */
    static void Line(QImage& image, const Point2d& p1, const Point2d& p2,
                     const Color& color, int32_t thickness = 1);

    /**
     * @brief Draw a rectangle outline
     */
    static void Rectangle(QImage& image, int32_t x, int32_t y, int32_t width, int32_t height,
                          const Color& color, int32_t thickness = 1);

    /**
     * @brief Draw a rectangle from Rect2i
     */
    static void Rectangle(QImage& image, const Rect2i& rect,
                          const Color& color, int32_t thickness = 1);

    /**
     * @brief Draw a filled rectangle
     */
    static void FilledRectangle(QImage& image, const Rect2i& rect, const Color& color);

    /**
     * @brief Draw a circle outline
     */
    static void Circle(QImage& image, int32_t cx, int32_t cy, int32_t radius,
                       const Color& color, int32_t thickness = 1);

    /**
     * @brief Draw a circle from Point2d center
     */
    static void Circle(QImage& image, const Point2d& center, double radius,
                       const Color& color, int32_t thickness = 1);

    /**
     * @brief Draw a cross marker
     */
    static void Cross(QImage& image, int32_t cx, int32_t cy, int32_t size,
                      const Color& color, int32_t thickness = 1);

    /**
     * @brief Draw a cross at Point2d
     */
    static void Cross(QImage& image, const Point2d& center, int32_t size,
                      const Color& color, int32_t thickness = 1);

    /**
     * @brief Draw a rotated cross marker
     * @param angle Rotation angle in radians
     */
    static void Cross(QImage& image, const Point2d& center, int32_t size,
                      double angle, const Color& color, int32_t thickness = 1);

    /**
     * @brief Draw an arrow
     */
    static void Arrow(QImage& image, const Point2d& from, const Point2d& to,
                      const Color& color, int32_t thickness = 1, double arrowSize = 10.0);

    // =========================================================================
    // Polylines and Contours
    // =========================================================================

    /**
     * @brief Draw a polyline (connected line segments)
     */
    static void Polyline(QImage& image, const std::vector<Point2d>& points,
                         const Color& color, int32_t thickness = 1, bool closed = false);

    /**
     * @brief Draw a rotated rectangle
     */
    static void RotatedRectangle(QImage& image, const Point2d& center,
                                  double width, double height, double angle,
                                  const Color& color, int32_t thickness = 1);

    // =========================================================================
    // Match Result Visualization
    // =========================================================================

    /**
     * @brief Draw match result with cross marker and angle indicator
     */
    static void MatchResult(QImage& image, const Matching::MatchResult& match,
                            const Color& color, int32_t markerSize = 20);

    /**
     * @brief Draw match result with model contour
     * @param modelContour Points of the model contour (relative to origin)
     */
    static void MatchResultWithContour(QImage& image, const Matching::MatchResult& match,
                                        const std::vector<Point2d>& modelContour,
                                        const Color& color, int32_t thickness = 1);

    /**
     * @brief Draw multiple match results
     */
    static void MatchResults(QImage& image, const std::vector<Matching::MatchResult>& matches,
                             const Color& color, int32_t markerSize = 20);

    // =========================================================================
    // Text (Simple)
    // =========================================================================

    /**
     * @brief Draw text annotation (simple bitmap font)
     * Only supports digits, letters, and basic punctuation
     */
    static void Text(QImage& image, int32_t x, int32_t y, const std::string& text,
                     const Color& color, int32_t scale = 1);

    // =========================================================================
    // Image Conversion
    // =========================================================================

    /**
     * @brief Convert grayscale image to RGB for colored drawing
     */
    static QImage ToRGB(const QImage& gray);

    /**
     * @brief Create a copy suitable for drawing (converts to RGB if needed)
     */
    static QImage PrepareForDrawing(const QImage& image);
};

// =============================================================================
// Inline Implementations
// =============================================================================

inline void Draw::Pixel(QImage& image, int32_t x, int32_t y, const Color& color) {
    if (x < 0 || x >= image.Width() || y < 0 || y >= image.Height()) {
        return;
    }

    uint8_t* data = static_cast<uint8_t*>(image.Data());
    size_t stride = image.Stride();
    int channels = image.Channels();

    if (channels == 1) {
        // Grayscale - use luminance
        data[y * stride + x] = static_cast<uint8_t>(0.299 * color.r + 0.587 * color.g + 0.114 * color.b);
    } else if (channels == 3) {
        // RGB
        uint8_t* px = data + y * stride + x * 3;
        px[0] = color.r;
        px[1] = color.g;
        px[2] = color.b;
    }
}

inline void Draw::Cross(QImage& image, int32_t cx, int32_t cy, int32_t size,
                        const Color& color, int32_t thickness) {
    Line(image, cx - size, cy, cx + size, cy, color, thickness);
    Line(image, cx, cy - size, cx, cy + size, color, thickness);
}

inline void Draw::Cross(QImage& image, const Point2d& center, int32_t size,
                        const Color& color, int32_t thickness) {
    Cross(image, static_cast<int32_t>(center.x + 0.5), static_cast<int32_t>(center.y + 0.5),
          size, color, thickness);
}

inline void Draw::Cross(QImage& image, const Point2d& center, int32_t size,
                        double angle, const Color& color, int32_t thickness) {
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);
    double s = static_cast<double>(size);

    // Horizontal arm rotated
    Point2d h1{center.x - s * cosA, center.y - s * sinA};
    Point2d h2{center.x + s * cosA, center.y + s * sinA};
    Line(image, h1, h2, color, thickness);

    // Vertical arm rotated
    Point2d v1{center.x + s * sinA, center.y - s * cosA};
    Point2d v2{center.x - s * sinA, center.y + s * cosA};
    Line(image, v1, v2, color, thickness);
}

inline void Draw::Line(QImage& image, const Point2d& p1, const Point2d& p2,
                       const Color& color, int32_t thickness) {
    Line(image, static_cast<int32_t>(p1.x + 0.5), static_cast<int32_t>(p1.y + 0.5),
         static_cast<int32_t>(p2.x + 0.5), static_cast<int32_t>(p2.y + 0.5),
         color, thickness);
}

inline void Draw::Rectangle(QImage& image, const Rect2i& rect,
                            const Color& color, int32_t thickness) {
    Rectangle(image, rect.x, rect.y, rect.width, rect.height, color, thickness);
}

inline void Draw::Circle(QImage& image, const Point2d& center, double radius,
                         const Color& color, int32_t thickness) {
    Circle(image, static_cast<int32_t>(center.x + 0.5), static_cast<int32_t>(center.y + 0.5),
           static_cast<int32_t>(radius + 0.5), color, thickness);
}

} // namespace Qi::Vision
