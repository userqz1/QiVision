/**
 * @file Draw.cpp
 * @brief Implementation of drawing functions
 */

#include <QiVision/Display/Draw.h>
#include <QiVision/Core/QContourArray.h>
#include <QiVision/Matching/ShapeModel.h>
#include <QiVision/Matching/MatchTypes.h>
#include <QiVision/Measure/MeasureHandle.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_map>

namespace Qi::Vision {

// =============================================================================
// Line Drawing (Bresenham with thickness)
// =============================================================================

// Helper: draw single-pixel Bresenham line
static void BresenhamLine(QImage& image, int32_t x1, int32_t y1, int32_t x2, int32_t y2,
                          const Scalar& color) {
    int32_t dx = std::abs(x2 - x1);
    int32_t dy = std::abs(y2 - y1);
    int32_t sx = (x1 < x2) ? 1 : -1;
    int32_t sy = (y1 < y2) ? 1 : -1;
    int32_t err = dx - dy;

    while (true) {
        Draw::Pixel(image, x1, y1, color);
        if (x1 == x2 && y1 == y2) break;

        int32_t e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x1 += sx; }
        if (e2 < dx) { err += dx; y1 += sy; }
    }
}

void Draw::Line(QImage& image, int32_t x1, int32_t y1, int32_t x2, int32_t y2,
                const Scalar& color, int32_t thickness) {
    if (thickness <= 1) {
        BresenhamLine(image, x1, y1, x2, y2, color);
    } else {
        // Thick line: draw multiple parallel Bresenham lines
        double dx = x2 - x1;
        double dy = y2 - y1;
        double len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-6) {
            FilledCircle(image, x1, y1, thickness / 2, color);
            return;
        }

        // Perpendicular unit vector
        double perpX = -dy / len;
        double perpY = dx / len;

        // Draw parallel lines offset from center
        int32_t halfT = thickness / 2;
        for (int32_t offset = -halfT; offset <= halfT; ++offset) {
            int32_t ox = static_cast<int32_t>(std::round(offset * perpX));
            int32_t oy = static_cast<int32_t>(std::round(offset * perpY));
            BresenhamLine(image, x1 + ox, y1 + oy, x2 + ox, y2 + oy, color);
        }
    }
}

// =============================================================================
// Anti-aliased Line (Wu's algorithm)
// =============================================================================

void Draw::LineAA(QImage& image, double x1, double y1, double x2, double y2,
                  const Scalar& color) {
    auto ipart = [](double x) { return static_cast<int32_t>(std::floor(x)); };
    auto fpart = [](double x) { return x - std::floor(x); };
    auto rfpart = [&fpart](double x) { return 1.0 - fpart(x); };

    bool steep = std::abs(y2 - y1) > std::abs(x2 - x1);
    if (steep) {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }
    if (x1 > x2) {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    double dx = x2 - x1;
    double dy = y2 - y1;
    double gradient = (dx < 1e-6) ? 1.0 : dy / dx;

    // First endpoint
    int32_t xend = static_cast<int32_t>(std::round(x1));
    double yend = y1 + gradient * (xend - x1);
    double xgap = rfpart(x1 + 0.5);
    int32_t xpxl1 = xend;
    int32_t ypxl1 = ipart(yend);

    if (steep) {
        PixelAlpha(image, ypxl1, xpxl1, color, rfpart(yend) * xgap);
        PixelAlpha(image, ypxl1 + 1, xpxl1, color, fpart(yend) * xgap);
    } else {
        PixelAlpha(image, xpxl1, ypxl1, color, rfpart(yend) * xgap);
        PixelAlpha(image, xpxl1, ypxl1 + 1, color, fpart(yend) * xgap);
    }
    double intery = yend + gradient;

    // Second endpoint
    xend = static_cast<int32_t>(std::round(x2));
    yend = y2 + gradient * (xend - x2);
    xgap = fpart(x2 + 0.5);
    int32_t xpxl2 = xend;
    int32_t ypxl2 = ipart(yend);

    if (steep) {
        PixelAlpha(image, ypxl2, xpxl2, color, rfpart(yend) * xgap);
        PixelAlpha(image, ypxl2 + 1, xpxl2, color, fpart(yend) * xgap);
    } else {
        PixelAlpha(image, xpxl2, ypxl2, color, rfpart(yend) * xgap);
        PixelAlpha(image, xpxl2, ypxl2 + 1, color, fpart(yend) * xgap);
    }

    // Main loop
    if (steep) {
        for (int32_t x = xpxl1 + 1; x < xpxl2; ++x) {
            PixelAlpha(image, ipart(intery), x, color, rfpart(intery));
            PixelAlpha(image, ipart(intery) + 1, x, color, fpart(intery));
            intery += gradient;
        }
    } else {
        for (int32_t x = xpxl1 + 1; x < xpxl2; ++x) {
            PixelAlpha(image, x, ipart(intery), color, rfpart(intery));
            PixelAlpha(image, x, ipart(intery) + 1, color, fpart(intery));
            intery += gradient;
        }
    }
}

// =============================================================================
// Rectangle
// =============================================================================

void Draw::Rectangle(QImage& image, int32_t x, int32_t y, int32_t width, int32_t height,
                     const Scalar& color, int32_t thickness) {
    Line(image, x, y, x + width - 1, y, color, thickness);
    Line(image, x, y + height - 1, x + width - 1, y + height - 1, color, thickness);
    Line(image, x, y, x, y + height - 1, color, thickness);
    Line(image, x + width - 1, y, x + width - 1, y + height - 1, color, thickness);
}

void Draw::FilledRectangle(QImage& image, const Rect2i& rect, const Scalar& color) {
    int32_t x0 = std::max(0, rect.x);
    int32_t y0 = std::max(0, rect.y);
    int32_t x1 = std::min(image.Width(), rect.x + rect.width);
    int32_t y1 = std::min(image.Height(), rect.y + rect.height);

    for (int32_t y = y0; y < y1; ++y) {
        for (int32_t x = x0; x < x1; ++x) {
            Pixel(image, x, y, color);
        }
    }
}

// =============================================================================
// Circle (Parametric method for smooth drawing)
// =============================================================================

void Draw::Circle(QImage& image, int32_t cx, int32_t cy, int32_t radius,
                  const Scalar& color, int32_t thickness) {
    if (radius <= 0) return;

    // Use parametric method: connect points with lines for smooth result
    // Number of segments based on circumference
    int numSegments = std::max(36, static_cast<int>(2.0 * M_PI * radius));

    double angleStep = 2.0 * M_PI / numSegments;

    // First point
    double prevX = cx + radius;
    double prevY = cy;

    for (int i = 1; i <= numSegments; ++i) {
        double angle = i * angleStep;
        double currX = cx + radius * std::cos(angle);
        double currY = cy + radius * std::sin(angle);

        // Draw line segment between consecutive points
        Line(image,
             static_cast<int32_t>(prevX + 0.5), static_cast<int32_t>(prevY + 0.5),
             static_cast<int32_t>(currX + 0.5), static_cast<int32_t>(currY + 0.5),
             color, thickness);

        prevX = currX;
        prevY = currY;
    }
}

void Draw::FilledCircle(QImage& image, int32_t cx, int32_t cy, int32_t radius,
                        const Scalar& color) {
    if (radius <= 0) return;

    for (int32_t y = -radius; y <= radius; ++y) {
        int32_t halfWidth = static_cast<int32_t>(std::sqrt(radius * radius - y * y));
        for (int32_t x = -halfWidth; x <= halfWidth; ++x) {
            Pixel(image, cx + x, cy + y, color);
        }
    }
}

// =============================================================================
// Ellipse (Parametric method for smooth drawing)
// =============================================================================

void Draw::Ellipse(QImage& image, int32_t cx, int32_t cy,
                   int32_t radiusX, int32_t radiusY,
                   const Scalar& color, int32_t thickness) {
    if (radiusX <= 0 || radiusY <= 0) return;

    // Use parametric method with enough segments for smooth result
    int maxRadius = std::max(radiusX, radiusY);
    int numSegments = std::max(72, static_cast<int>(2.0 * M_PI * maxRadius));

    double angleStep = 2.0 * M_PI / numSegments;

    double prevX = cx + radiusX;
    double prevY = cy;

    for (int i = 1; i <= numSegments; ++i) {
        double t = i * angleStep;
        double currX = cx + radiusX * std::cos(t);
        double currY = cy + radiusY * std::sin(t);

        Line(image,
             static_cast<int32_t>(prevX + 0.5), static_cast<int32_t>(prevY + 0.5),
             static_cast<int32_t>(currX + 0.5), static_cast<int32_t>(currY + 0.5),
             color, thickness);

        prevX = currX;
        prevY = currY;
    }
}

void Draw::Ellipse(QImage& image, const Point2d& center,
                   double radiusX, double radiusY, double angle,
                   const Scalar& color, int32_t thickness) {
    if (radiusX <= 0 || radiusY <= 0) return;

    // Use parametric method with rotation
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    int maxRadius = static_cast<int>(std::max(radiusX, radiusY));
    int numSegments = std::max(72, static_cast<int>(2.0 * M_PI * maxRadius));

    double angleStep = 2.0 * M_PI / numSegments;

    // First point
    double x0 = radiusX;
    double y0 = 0;
    double prevX = center.x + x0 * cosA - y0 * sinA;
    double prevY = center.y + x0 * sinA + y0 * cosA;

    for (int i = 1; i <= numSegments; ++i) {
        double t = i * angleStep;
        double ex = radiusX * std::cos(t);
        double ey = radiusY * std::sin(t);
        double currX = center.x + ex * cosA - ey * sinA;
        double currY = center.y + ex * sinA + ey * cosA;

        Line(image,
             static_cast<int32_t>(prevX + 0.5), static_cast<int32_t>(prevY + 0.5),
             static_cast<int32_t>(currX + 0.5), static_cast<int32_t>(currY + 0.5),
             color, thickness);

        prevX = currX;
        prevY = currY;
    }
}

void Draw::FilledEllipse(QImage& image, int32_t cx, int32_t cy,
                         int32_t radiusX, int32_t radiusY, const Scalar& color) {
    if (radiusX <= 0 || radiusY <= 0) return;

    for (int32_t y = -radiusY; y <= radiusY; ++y) {
        double ratio = static_cast<double>(y) / radiusY;
        int32_t halfWidth = static_cast<int32_t>(radiusX * std::sqrt(1.0 - ratio * ratio));
        for (int32_t x = -halfWidth; x <= halfWidth; ++x) {
            Pixel(image, cx + x, cy + y, color);
        }
    }
}

// =============================================================================
// Arrow
// =============================================================================

void Draw::Arrow(QImage& image, const Point2d& from, const Point2d& to,
                 const Scalar& color, int32_t thickness, double arrowSize) {
    Line(image, from, to, color, thickness);

    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-6) return;

    dx /= len;
    dy /= len;

    double angle = 0.5;  // ~30 degrees
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    Point2d left{
        to.x - arrowSize * (dx * cosA + dy * sinA),
        to.y - arrowSize * (-dx * sinA + dy * cosA)
    };
    Line(image, to, left, color, thickness);

    Point2d right{
        to.x - arrowSize * (dx * cosA - dy * sinA),
        to.y - arrowSize * (dx * sinA + dy * cosA)
    };
    Line(image, to, right, color, thickness);
}

// =============================================================================
// Arc
// =============================================================================

void Draw::Arc(QImage& image, const Point2d& center, double radius,
               double startAngle, double endAngle,
               const Scalar& color, int32_t thickness) {
    if (radius <= 0) return;

    // Normalize angles
    while (endAngle < startAngle) endAngle += 2.0 * 3.14159265358979323846;

    double angleRange = endAngle - startAngle;
    int32_t numPoints = std::max(16, static_cast<int32_t>(angleRange * radius / 4.0));

    for (int32_t i = 0; i < numPoints; ++i) {
        double t1 = startAngle + angleRange * i / numPoints;
        double t2 = startAngle + angleRange * (i + 1) / numPoints;

        Point2d p1{center.x + radius * std::cos(t1), center.y + radius * std::sin(t1)};
        Point2d p2{center.x + radius * std::cos(t2), center.y + radius * std::sin(t2)};

        Line(image, p1, p2, color, thickness);
    }
}

// =============================================================================
// Polyline and Polygon
// =============================================================================

void Draw::Polyline(QImage& image, const std::vector<Point2d>& points,
                    const Scalar& color, int32_t thickness, bool closed) {
    if (points.size() < 2) return;

    for (size_t i = 0; i < points.size() - 1; ++i) {
        Line(image, points[i], points[i + 1], color, thickness);
    }

    if (closed && points.size() > 2) {
        Line(image, points.back(), points.front(), color, thickness);
    }
}

void Draw::FilledPolygon(QImage& image, const std::vector<Point2d>& points,
                         const Scalar& color) {
    if (points.size() < 3) return;

    // Find bounding box
    double minY = points[0].y, maxY = points[0].y;
    for (const auto& p : points) {
        minY = std::min(minY, p.y);
        maxY = std::max(maxY, p.y);
    }

    int32_t yStart = std::max(0, static_cast<int32_t>(minY));
    int32_t yEnd = std::min(image.Height() - 1, static_cast<int32_t>(maxY));

    // Scanline fill
    for (int32_t y = yStart; y <= yEnd; ++y) {
        std::vector<double> intersections;

        for (size_t i = 0; i < points.size(); ++i) {
            const Point2d& p1 = points[i];
            const Point2d& p2 = points[(i + 1) % points.size()];

            if ((p1.y <= y && p2.y > y) || (p2.y <= y && p1.y > y)) {
                double x = p1.x + (y - p1.y) / (p2.y - p1.y) * (p2.x - p1.x);
                intersections.push_back(x);
            }
        }

        std::sort(intersections.begin(), intersections.end());

        for (size_t i = 0; i + 1 < intersections.size(); i += 2) {
            int32_t xStart = std::max(0, static_cast<int32_t>(intersections[i]));
            int32_t xEnd = std::min(image.Width() - 1, static_cast<int32_t>(intersections[i + 1]));
            for (int32_t x = xStart; x <= xEnd; ++x) {
                Pixel(image, x, y, color);
            }
        }
    }
}

// =============================================================================
// Rotated Rectangle
// =============================================================================

void Draw::RotatedRectangle(QImage& image, const Point2d& center,
                            double width, double height, double angle,
                            const Scalar& color, int32_t thickness) {
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);
    double hw = width / 2.0;
    double hh = height / 2.0;

    std::vector<Point2d> corners = {
        {center.x + cosA * (-hw) - sinA * (-hh), center.y + sinA * (-hw) + cosA * (-hh)},
        {center.x + cosA * ( hw) - sinA * (-hh), center.y + sinA * ( hw) + cosA * (-hh)},
        {center.x + cosA * ( hw) - sinA * ( hh), center.y + sinA * ( hw) + cosA * ( hh)},
        {center.x + cosA * (-hw) - sinA * ( hh), center.y + sinA * (-hw) + cosA * ( hh)}
    };

    Polyline(image, corners, color, thickness, true);
}

// =============================================================================
// Match Result Visualization
// =============================================================================

void Draw::MatchResult(QImage& image, const Matching::MatchResult& match,
                       const Scalar& color, int32_t markerSize) {
    Cross(image, Point2d{match.x, match.y}, markerSize, match.angle, color, 2);

    double cosA = std::cos(match.angle);
    double sinA = std::sin(match.angle);
    Point2d lineEnd{
        match.x + markerSize * 1.5 * cosA,
        match.y + markerSize * 1.5 * sinA
    };
    Arrow(image, Point2d{match.x, match.y}, lineEnd, color, 2, 8);
}

void Draw::MatchResultWithContour(QImage& image, const Matching::MatchResult& match,
                                   const std::vector<Point2d>& modelContour,
                                   const Scalar& color, int32_t /*thickness*/) {
    if (modelContour.empty()) {
        MatchResult(image, match, color, 20);
        return;
    }

    double cosA = std::cos(match.angle);
    double sinA = std::sin(match.angle);

    for (const auto& pt : modelContour) {
        double tx = match.x + match.scaleX * (cosA * pt.x - sinA * pt.y);
        double ty = match.y + match.scaleY * (sinA * pt.x + cosA * pt.y);

        int32_t px = static_cast<int32_t>(tx + 0.5);
        int32_t py = static_cast<int32_t>(ty + 0.5);
        Pixel(image, px, py, color);
        Pixel(image, px + 1, py, color);
        Pixel(image, px, py + 1, color);
        Pixel(image, px + 1, py + 1, color);
    }

    Cross(image, Point2d{match.x, match.y}, 15, match.angle, color, 2);
}

void Draw::MatchResults(QImage& image, const std::vector<Matching::MatchResult>& matches,
                        const Scalar& color, int32_t markerSize) {
    std::vector<Scalar> colors = {
        Scalar::Green(), Scalar::Red(), Scalar::Blue(),
        Scalar::Yellow(), Scalar::Cyan(), Scalar::Magenta()
    };

    for (size_t i = 0; i < matches.size(); ++i) {
        const Scalar& c = (matches.size() == 1) ? color : colors[i % colors.size()];
        MatchResult(image, matches[i], c, markerSize);
    }
}

// =============================================================================
// Bitmap Font (5x7) - Full ASCII support
// =============================================================================

// 5x7 bitmap font data for ASCII 32-126
static const uint8_t FONT_5x7[][7] = {
    // Space (32)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    // ! (33)
    {0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04},
    // " (34)
    {0x0A, 0x0A, 0x0A, 0x00, 0x00, 0x00, 0x00},
    // # (35)
    {0x0A, 0x0A, 0x1F, 0x0A, 0x1F, 0x0A, 0x0A},
    // $ (36)
    {0x04, 0x0F, 0x14, 0x0E, 0x05, 0x1E, 0x04},
    // % (37)
    {0x18, 0x19, 0x02, 0x04, 0x08, 0x13, 0x03},
    // & (38)
    {0x0C, 0x12, 0x14, 0x08, 0x15, 0x12, 0x0D},
    // ' (39)
    {0x06, 0x04, 0x08, 0x00, 0x00, 0x00, 0x00},
    // ( (40)
    {0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02},
    // ) (41)
    {0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08},
    // * (42)
    {0x00, 0x04, 0x15, 0x0E, 0x15, 0x04, 0x00},
    // + (43)
    {0x00, 0x04, 0x04, 0x1F, 0x04, 0x04, 0x00},
    // , (44)
    {0x00, 0x00, 0x00, 0x00, 0x06, 0x04, 0x08},
    // - (45)
    {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00},
    // . (46)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C},
    // / (47)
    {0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x00},
    // 0 (48)
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E},
    // 1 (49)
    {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
    // 2 (50)
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F},
    // 3 (51)
    {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E},
    // 4 (52)
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02},
    // 5 (53)
    {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},
    // 6 (54)
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E},
    // 7 (55)
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
    // 8 (56)
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E},
    // 9 (57)
    {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},
    // : (58)
    {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00},
    // ; (59)
    {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x04, 0x08},
    // < (60)
    {0x02, 0x04, 0x08, 0x10, 0x08, 0x04, 0x02},
    // = (61)
    {0x00, 0x00, 0x1F, 0x00, 0x1F, 0x00, 0x00},
    // > (62)
    {0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08},
    // ? (63)
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x00, 0x04},
    // @ (64)
    {0x0E, 0x11, 0x17, 0x15, 0x17, 0x10, 0x0F},
    // A (65)
    {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},
    // B (66)
    {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E},
    // C (67)
    {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E},
    // D (68)
    {0x1C, 0x12, 0x11, 0x11, 0x11, 0x12, 0x1C},
    // E (69)
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F},
    // F (70)
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10},
    // G (71)
    {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F},
    // H (72)
    {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},
    // I (73)
    {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E},
    // J (74)
    {0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C},
    // K (75)
    {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11},
    // L (76)
    {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F},
    // M (77)
    {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11},
    // N (78)
    {0x11, 0x11, 0x19, 0x15, 0x13, 0x11, 0x11},
    // O (79)
    {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E},
    // P (80)
    {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10},
    // Q (81)
    {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D},
    // R (82)
    {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11},
    // S (83)
    {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E},
    // T (84)
    {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
    // U (85)
    {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E},
    // V (86)
    {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04},
    // W (87)
    {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A},
    // X (88)
    {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11},
    // Y (89)
    {0x11, 0x11, 0x11, 0x0A, 0x04, 0x04, 0x04},
    // Z (90)
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F},
    // [ (91)
    {0x0E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0E},
    // \ (92)
    {0x00, 0x10, 0x08, 0x04, 0x02, 0x01, 0x00},
    // ] (93)
    {0x0E, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0E},
    // ^ (94)
    {0x04, 0x0A, 0x11, 0x00, 0x00, 0x00, 0x00},
    // _ (95)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F},
    // ` (96)
    {0x08, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00},
    // a (97)
    {0x00, 0x00, 0x0E, 0x01, 0x0F, 0x11, 0x0F},
    // b (98)
    {0x10, 0x10, 0x16, 0x19, 0x11, 0x11, 0x1E},
    // c (99)
    {0x00, 0x00, 0x0E, 0x10, 0x10, 0x11, 0x0E},
    // d (100)
    {0x01, 0x01, 0x0D, 0x13, 0x11, 0x11, 0x0F},
    // e (101)
    {0x00, 0x00, 0x0E, 0x11, 0x1F, 0x10, 0x0E},
    // f (102)
    {0x06, 0x09, 0x08, 0x1C, 0x08, 0x08, 0x08},
    // g (103)
    {0x00, 0x0F, 0x11, 0x11, 0x0F, 0x01, 0x0E},
    // h (104)
    {0x10, 0x10, 0x16, 0x19, 0x11, 0x11, 0x11},
    // i (105)
    {0x04, 0x00, 0x0C, 0x04, 0x04, 0x04, 0x0E},
    // j (106)
    {0x02, 0x00, 0x06, 0x02, 0x02, 0x12, 0x0C},
    // k (107)
    {0x10, 0x10, 0x12, 0x14, 0x18, 0x14, 0x12},
    // l (108)
    {0x0C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E},
    // m (109)
    {0x00, 0x00, 0x1A, 0x15, 0x15, 0x11, 0x11},
    // n (110)
    {0x00, 0x00, 0x16, 0x19, 0x11, 0x11, 0x11},
    // o (111)
    {0x00, 0x00, 0x0E, 0x11, 0x11, 0x11, 0x0E},
    // p (112)
    {0x00, 0x00, 0x1E, 0x11, 0x1E, 0x10, 0x10},
    // q (113)
    {0x00, 0x00, 0x0D, 0x13, 0x0F, 0x01, 0x01},
    // r (114)
    {0x00, 0x00, 0x16, 0x19, 0x10, 0x10, 0x10},
    // s (115)
    {0x00, 0x00, 0x0E, 0x10, 0x0E, 0x01, 0x1E},
    // t (116)
    {0x08, 0x08, 0x1C, 0x08, 0x08, 0x09, 0x06},
    // u (117)
    {0x00, 0x00, 0x11, 0x11, 0x11, 0x13, 0x0D},
    // v (118)
    {0x00, 0x00, 0x11, 0x11, 0x11, 0x0A, 0x04},
    // w (119)
    {0x00, 0x00, 0x11, 0x11, 0x15, 0x15, 0x0A},
    // x (120)
    {0x00, 0x00, 0x11, 0x0A, 0x04, 0x0A, 0x11},
    // y (121)
    {0x00, 0x00, 0x11, 0x11, 0x0F, 0x01, 0x0E},
    // z (122)
    {0x00, 0x00, 0x1F, 0x02, 0x04, 0x08, 0x1F},
    // { (123)
    {0x02, 0x04, 0x04, 0x08, 0x04, 0x04, 0x02},
    // | (124)
    {0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
    // } (125)
    {0x08, 0x04, 0x04, 0x02, 0x04, 0x04, 0x08},
    // ~ (126)
    {0x00, 0x00, 0x08, 0x15, 0x02, 0x00, 0x00},
};

void Draw::Text(QImage& image, int32_t x, int32_t y, const std::string& text,
                const Scalar& color, int32_t scale) {
    if (scale < 1) scale = 1;
    int32_t curX = x;

    for (unsigned char c : text) {
        int fontIdx = -1;

        // Map ASCII to font index (32-126)
        if (c >= 32 && c <= 126) {
            fontIdx = c - 32;
        }

        if (fontIdx >= 0 && fontIdx < 95) {
            for (int32_t row = 0; row < 7; ++row) {
                uint8_t bits = FONT_5x7[fontIdx][row];
                for (int32_t col = 0; col < 5; ++col) {
                    if (bits & (0x10 >> col)) {
                        for (int32_t sy = 0; sy < scale; ++sy) {
                            for (int32_t sx = 0; sx < scale; ++sx) {
                                Pixel(image, curX + col * scale + sx,
                                      y + row * scale + sy, color);
                            }
                        }
                    }
                }
            }
        }

        curX += 6 * scale;  // 5 pixels + 1 spacing
    }
}

std::pair<int32_t, int32_t> Draw::TextSize(const std::string& text, int32_t scale) {
    if (scale < 1) scale = 1;
    int32_t width = static_cast<int32_t>(text.length()) * 6 * scale - scale;  // Remove trailing space
    int32_t height = 7 * scale;
    return {width, height};
}

// =============================================================================
// Metrology Visualization
// =============================================================================

} // namespace Qi::Vision

// Include Metrology headers here (after namespace close) to avoid circular deps
#include <QiVision/Measure/MeasureHandle.h>
#include <QiVision/Measure/Metrology.h>

namespace Qi::Vision {

void Draw::MeasureRect(QImage& image, const Measure::MeasureRectangle2& handle,
                       const Scalar& color, int32_t thickness) {
    // MeasureRectangle2 geometry (Halcon compatible):
    // - Row, Column: Rectangle center
    // - Phi: Edge direction (perpendicular to profile/measurement direction)
    // - Length1: Half-length along profile direction (phi + PI/2)
    // - Length2: Half-length along phi direction (edge direction)

    double cx = handle.Column();
    double cy = handle.Row();
    double phi = handle.Phi();
    double len1 = handle.Length1();  // Along profile direction (phi + PI/2)
    double len2 = handle.Length2();  // Along phi direction (edge direction)

    // Profile direction = phi + PI/2 (measurement/search direction)
    double profileAngle = phi + M_PI / 2.0;

    // Direction vectors
    double profileDirX = std::cos(profileAngle);  // Profile direction: extends ±len1
    double profileDirY = std::sin(profileAngle);
    double edgeDirX = std::cos(phi);              // Edge direction (phi): extends ±len2
    double edgeDirY = std::sin(phi);

    // Four corners: center ± len1*profileDir ± len2*edgeDir
    double x1 = cx - len1 * profileDirX - len2 * edgeDirX;
    double y1 = cy - len1 * profileDirY - len2 * edgeDirY;
    double x2 = cx - len1 * profileDirX + len2 * edgeDirX;
    double y2 = cy - len1 * profileDirY + len2 * edgeDirY;
    double x3 = cx + len1 * profileDirX + len2 * edgeDirX;
    double y3 = cy + len1 * profileDirY + len2 * edgeDirY;
    double x4 = cx + len1 * profileDirX - len2 * edgeDirX;
    double y4 = cy + len1 * profileDirY - len2 * edgeDirY;

    // Draw four edges (closed rectangle)
    Line(image, Point2d{x1, y1}, Point2d{x2, y2}, color, thickness);
    Line(image, Point2d{x2, y2}, Point2d{x3, y3}, color, thickness);
    Line(image, Point2d{x3, y3}, Point2d{x4, y4}, color, thickness);
    Line(image, Point2d{x4, y4}, Point2d{x1, y1}, color, thickness);

    // Draw projection line (center line along profile direction)
    double pStartX = cx - len1 * profileDirX;
    double pStartY = cy - len1 * profileDirY;
    double pEndX = cx + len1 * profileDirX;
    double pEndY = cy + len1 * profileDirY;
    Line(image, Point2d{pStartX, pStartY}, Point2d{pEndX, pEndY}, color, thickness);
}

void Draw::MeasureArc(QImage& image, const Measure::MeasureArc& handle,
                      const Scalar& color, int32_t thickness) {
    // MeasureArc geometry (Halcon compatible):
    // - CenterRow, CenterCol: Arc center
    // - Radius: Arc radius (center of annular region)
    // - AngleStart, AngleExtent: Arc angular span
    // - AnnulusRadius: Half-width of annular region for averaging

    double cx = handle.CenterCol();
    double cy = handle.CenterRow();
    double radius = handle.Radius();
    double angleStart = handle.AngleStart();
    double angleExtent = handle.AngleExtent();
    double annulusRadius = handle.AnnulusRadius();

    double angleEnd = angleStart + angleExtent;

    // Inner and outer radii
    double innerRadius = radius - annulusRadius;
    double outerRadius = radius + annulusRadius;
    if (innerRadius < 0) innerRadius = 0;

    // Draw inner arc
    Arc(image, Point2d{cx, cy}, innerRadius, angleStart, angleEnd, color, thickness);

    // Draw outer arc
    Arc(image, Point2d{cx, cy}, outerRadius, angleStart, angleEnd, color, thickness);

    // Draw radial end lines (connecting inner and outer arcs)
    double cosStart = std::cos(angleStart);
    double sinStart = std::sin(angleStart);
    double cosEnd = std::cos(angleEnd);
    double sinEnd = std::sin(angleEnd);

    Point2d innerStart{cx + innerRadius * cosStart, cy + innerRadius * sinStart};
    Point2d outerStart{cx + outerRadius * cosStart, cy + outerRadius * sinStart};
    Point2d innerEnd{cx + innerRadius * cosEnd, cy + innerRadius * sinEnd};
    Point2d outerEnd{cx + outerRadius * cosEnd, cy + outerRadius * sinEnd};

    Line(image, innerStart, outerStart, color, thickness);
    Line(image, innerEnd, outerEnd, color, thickness);

    // Draw projection arc (center arc along radius - the measurement path)
    Arc(image, Point2d{cx, cy}, radius, angleStart, angleEnd, color, thickness);
}

void Draw::MeasureConcentric(QImage& image, const Measure::MeasureConcentricCircles& handle,
                              const Scalar& color, int32_t thickness) {
    // MeasureConcentricCircles geometry:
    // - CenterRow, CenterCol: Circle center
    // - InnerRadius, OuterRadius: Radial extent
    // - Angle: Direction angle for radial profile
    // - AngularWidth: Angular width for averaging

    double cx = handle.CenterCol();
    double cy = handle.CenterRow();
    double innerRadius = handle.InnerRadius();
    double outerRadius = handle.OuterRadius();
    double angle = handle.Angle();
    double angularWidth = handle.AngularWidth();

    // Angular span of the measurement region
    double angleStart = angle - angularWidth / 2.0;
    double angleEnd = angle + angularWidth / 2.0;

    // Draw inner arc
    Arc(image, Point2d{cx, cy}, innerRadius, angleStart, angleEnd, color, thickness);

    // Draw outer arc
    Arc(image, Point2d{cx, cy}, outerRadius, angleStart, angleEnd, color, thickness);

    // Draw angular boundary lines (connecting inner and outer arcs)
    double cosStart = std::cos(angleStart);
    double sinStart = std::sin(angleStart);
    double cosEnd = std::cos(angleEnd);
    double sinEnd = std::sin(angleEnd);

    Point2d innerStart{cx + innerRadius * cosStart, cy + innerRadius * sinStart};
    Point2d outerStart{cx + outerRadius * cosStart, cy + outerRadius * sinStart};
    Point2d innerEnd{cx + innerRadius * cosEnd, cy + innerRadius * sinEnd};
    Point2d outerEnd{cx + outerRadius * cosEnd, cy + outerRadius * sinEnd};

    Line(image, innerStart, outerStart, color, thickness);
    Line(image, innerEnd, outerEnd, color, thickness);

    // Draw projection line (radial center line - the measurement path)
    double cosAngle = std::cos(angle);
    double sinAngle = std::sin(angle);
    Point2d innerCenter{cx + innerRadius * cosAngle, cy + innerRadius * sinAngle};
    Point2d outerCenter{cx + outerRadius * cosAngle, cy + outerRadius * sinAngle};
    Line(image, innerCenter, outerCenter, color, thickness);
}

void Draw::MeasureRects(QImage& image,
                        const std::vector<Measure::MeasureRectangle2>& handles,
                        const Scalar& color, int32_t thickness) {
    if (handles.empty()) return;

    // 1. Draw curve connecting caliper centers
    if (handles.size() >= 2) {
        for (size_t i = 0; i < handles.size(); ++i) {
            size_t j = (i + 1) % handles.size();  // Connect last to first for closed contour
            Point2d p1{handles[i].Column(), handles[i].Row()};
            Point2d p2{handles[j].Column(), handles[j].Row()};
            Line(image, p1, p2, color, thickness);
        }
    }

    // 2. Draw each caliper rectangle
    for (const auto& handle : handles) {
        MeasureRect(image, handle, color, thickness);
    }
}

void Draw::Contour(QImage& image, const QContour& contour,
                   const Scalar& color, int32_t thickness) {
    auto points = contour.GetPoints();
    if (points.size() < 2) return;

    for (size_t i = 0; i < points.size() - 1; ++i) {
        Line(image, points[i], points[i + 1], color, thickness);
    }

    // Close contour if it's closed
    if (contour.IsClosed() && points.size() >= 3) {
        Line(image, points.back(), points.front(), color, thickness);
    }
}

// =============================================================================
// Region Drawing
// =============================================================================

void Draw::Region(QImage& image, const QRegion& region, const Scalar& color) {
    if (region.Empty() || image.Empty()) return;

    uint8_t* data = static_cast<uint8_t*>(image.Data());
    int32_t stride = image.Stride();
    int32_t channels = image.Channels();
    int32_t width = image.Width();
    int32_t height = image.Height();

    for (const auto& run : region.Runs()) {
        int32_t y = run.row;
        if (y < 0 || y >= height) continue;

        int32_t x1 = std::max(0, run.colBegin);
        int32_t x2 = std::min(width - 1, run.colEnd);

        uint8_t* row = data + y * stride;
        for (int32_t x = x1; x <= x2; ++x) {
            if (channels >= 3) {
                row[x * channels + 0] = color.r;
                row[x * channels + 1] = color.g;
                row[x * channels + 2] = color.b;
            } else {
                row[x] = static_cast<uint8_t>((color.r + color.g + color.b) / 3);
            }
        }
    }
}

void Draw::RegionContour(QImage& image, const QRegion& region,
                         const Scalar& color, int32_t thickness) {
    if (region.Empty() || image.Empty()) return;

    int32_t width = image.Width();
    int32_t height = image.Height();

    // Build a quick lookup for region pixels
    std::unordered_map<int32_t, std::vector<std::pair<int32_t, int32_t>>> rowRanges;
    for (const auto& run : region.Runs()) {
        rowRanges[run.row].emplace_back(run.colBegin, run.colEnd);
    }

    auto isInRegion = [&rowRanges](int32_t r, int32_t c) -> bool {
        auto it = rowRanges.find(r);
        if (it == rowRanges.end()) return false;
        for (const auto& range : it->second) {
            if (c >= range.first && c <= range.second) return true;
        }
        return false;
    };

    // Find and draw boundary pixels
    for (const auto& run : region.Runs()) {
        int32_t y = run.row;
        if (y < 0 || y >= height) continue;

        for (int32_t x = run.colBegin; x <= run.colEnd; ++x) {
            if (x < 0 || x >= width) continue;

            // Check if this is a boundary pixel
            bool isBoundary = false;
            for (int dy = -1; dy <= 1 && !isBoundary; ++dy) {
                for (int dx = -1; dx <= 1 && !isBoundary; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    int32_t ny = y + dy;
                    int32_t nx = x + dx;
                    if (ny < 0 || ny >= height || nx < 0 || nx >= width) {
                        isBoundary = true;
                    } else if (!isInRegion(ny, nx)) {
                        isBoundary = true;
                    }
                }
            }

            if (isBoundary) {
                if (thickness == 1) {
                    Pixel(image, x, y, color);
                } else {
                    FilledCircle(image, x, y, thickness / 2, color);
                }
            }
        }
    }
}

void Draw::RegionAlpha(QImage& image, const QRegion& region,
                       const Scalar& color, double alpha) {
    if (region.Empty() || image.Empty()) return;
    if (alpha <= 0.0) return;
    if (alpha >= 1.0) {
        Region(image, region, color);
        return;
    }

    uint8_t* data = static_cast<uint8_t*>(image.Data());
    int32_t stride = image.Stride();
    int32_t channels = image.Channels();
    int32_t width = image.Width();
    int32_t height = image.Height();

    double invAlpha = 1.0 - alpha;

    for (const auto& run : region.Runs()) {
        int32_t y = run.row;
        if (y < 0 || y >= height) continue;

        int32_t x1 = std::max(0, run.colBegin);
        int32_t x2 = std::min(width - 1, run.colEnd);

        uint8_t* row = data + y * stride;
        for (int32_t x = x1; x <= x2; ++x) {
            if (channels >= 3) {
                row[x * channels + 0] = static_cast<uint8_t>(row[x * channels + 0] * invAlpha + color.r * alpha);
                row[x * channels + 1] = static_cast<uint8_t>(row[x * channels + 1] * invAlpha + color.g * alpha);
                row[x * channels + 2] = static_cast<uint8_t>(row[x * channels + 2] * invAlpha + color.b * alpha);
            } else {
                uint8_t gray = static_cast<uint8_t>((color.r + color.g + color.b) / 3);
                row[x] = static_cast<uint8_t>(row[x] * invAlpha + gray * alpha);
            }
        }
    }
}

void Draw::EdgePoints(QImage& image, const std::vector<Point2d>& points,
                      const Scalar& color, int32_t markerSize) {
    for (const auto& pt : points) {
        Cross(image, pt, markerSize, color, 1);
    }
}

void Draw::MetrologyLine(QImage& image, const Measure::MetrologyLineResult& result,
                         const Scalar& color, int32_t thickness) {
    if (!result.IsValid()) return;

    // Draw the fitted line segment
    Point2d p1{result.col1, result.row1};
    Point2d p2{result.col2, result.row2};
    Line(image, p1, p2, color, thickness);
}

void Draw::MetrologyCircle(QImage& image, const Measure::MetrologyCircleResult& result,
                           const Scalar& color, int32_t thickness) {
    if (!result.IsValid()) return;

    // Draw the fitted circle
    Point2d center{result.column, result.row};
    Circle(image, center, result.radius, color, thickness);
}

void Draw::MetrologyEllipse(QImage& image, const Measure::MetrologyEllipseResult& result,
                            const Scalar& color, int32_t thickness) {
    if (!result.IsValid()) return;

    // Draw the fitted ellipse (ra = semi-major, rb = semi-minor)
    Point2d center{result.column, result.row};
    Ellipse(image, center, result.ra, result.rb, result.phi, color, thickness);
}

void Draw::MetrologyRectangle(QImage& image, const Measure::MetrologyRectangle2Result& result,
                              const Scalar& color, int32_t thickness) {
    if (!result.IsValid()) return;

    // Draw the fitted rectangle
    Point2d center{result.column, result.row};
    double width = 2.0 * result.length1;
    double height = 2.0 * result.length2;
    RotatedRectangle(image, center, width, height, result.phi, color, thickness);
}

void Draw::EdgePointsWeighted(QImage& image, const std::vector<Point2d>& points,
                               const std::vector<double>& weights, int32_t markerSize) {
    if (points.empty()) return;

    // Detect if weights are binary (RANSAC/Tukey) or continuous (Huber)
    bool isBinary = true;
    for (double w : weights) {
        if (w > 0.01 && w < 0.99) {  // Has intermediate values
            isBinary = false;
            break;
        }
    }

    for (size_t i = 0; i < points.size(); ++i) {
        double w = (i < weights.size()) ? weights[i] : 1.0;

        Scalar color;
        if (isBinary) {
            // Binary weights (RANSAC/Tukey): two colors
            color = (w >= 0.5) ? Scalar::Green() : Scalar::Red();
        } else {
            // Continuous weights (Huber): three colors
            if (w >= 0.8) {
                color = Scalar::Green();   // Strong inlier
            } else if (w >= 0.3) {
                color = Scalar::Yellow();  // Moderate weight
            } else {
                color = Scalar::Red();     // Weak/outlier
            }
        }

        FilledCircle(image, points[i], markerSize, color);
    }
}

void Draw::MetrologyModelResult(QImage& image, const Measure::MetrologyModel& model,
                                const Scalar& objectColor,
                                const Scalar& resultColor,
                                const Scalar& pointColor,
                                bool drawCalipers,
                                bool drawPoints) {
    using namespace Measure;

    int32_t numObjects = model.NumObjects();

    for (int32_t idx = 0; idx < numObjects; ++idx) {
        const MetrologyObject* obj = model.GetObject(idx);
        if (!obj) continue;

        // Draw calipers (rectangles + connecting curve)
        if (drawCalipers) {
            auto calipers = obj->GetCalipers();
            MeasureRects(image, calipers, objectColor, 1);

            // Draw geometric center for objects that have one
            if (obj->HasCenter()) {
                Point2d center = obj->GetCenter();
                Cross(image, center, 10, 0, objectColor, 1);
            }
        }

        // Draw edge points with weight-based coloring
        if (drawPoints) {
            auto points = model.GetMeasuredPoints(idx);
            auto weights = model.GetPointWeights(idx);
            if (!weights.empty()) {
                EdgePointsWeighted(image, points, weights, 3);
            } else {
                EdgePoints(image, points, pointColor, 3);
            }
        }

        // Draw result based on object type (thickness=1 for single-pixel lines)
        switch (obj->Type()) {
            case MetrologyObjectType::Line: {
                auto result = model.GetLineResult(idx);
                MetrologyLine(image, result, resultColor, 1);
                break;
            }
            case MetrologyObjectType::Circle: {
                auto result = model.GetCircleResult(idx);
                MetrologyCircle(image, result, resultColor, 1);
                break;
            }
            case MetrologyObjectType::Ellipse: {
                auto result = model.GetEllipseResult(idx);
                MetrologyEllipse(image, result, resultColor, 1);
                break;
            }
            case MetrologyObjectType::Rectangle2: {
                auto result = model.GetRectangle2Result(idx);
                MetrologyRectangle(image, result, resultColor, 1);
                break;
            }
        }
    }
}

// =============================================================================
// Shape Matching Visualization (Halcon-style dev_display_shape_matching_results)
// =============================================================================

void Draw::ShapeMatchingResults(QImage& image,
                                 const Matching::ShapeModel& model,
                                 const std::vector<Matching::MatchResult>& matches,
                                 const Scalar& matchedColor,
                                 const Scalar& unmatchedColor,
                                 int32_t thickness,
                                 double threshold)
{
    (void)unmatchedColor;  // Reserved for quality-based coloring
    (void)threshold;

    if (!model.IsValid() || matches.empty()) {
        return;
    }

    // Get model contours at level 1 (highest resolution)
    QContourArray contours;
    Matching::GetShapeModelXLD(model, 1, contours);

    for (const auto& match : matches) {
        double cosA = std::cos(match.angle);
        double sinA = std::sin(match.angle);

        // Transform and draw each contour
        for (size_t c = 0; c < contours.Size(); ++c) {
            const QContour& contour = contours[c];
            if (contour.Size() < 2) continue;

            for (size_t i = 1; i < contour.Size(); ++i) {
                auto p0 = contour.GetPoint(i - 1);
                auto p1 = contour.GetPoint(i);

                // Rotate and translate to match position
                double x0 = match.x + cosA * p0.x - sinA * p0.y;
                double y0 = match.y + sinA * p0.x + cosA * p0.y;
                double x1 = match.x + cosA * p1.x - sinA * p1.y;
                double y1 = match.y + sinA * p1.x + cosA * p1.y;

                Line(image,
                     static_cast<int32_t>(x0), static_cast<int32_t>(y0),
                     static_cast<int32_t>(x1), static_cast<int32_t>(y1),
                     matchedColor, thickness);
            }
        }
    }
}

void Draw::MatchedContour(QImage& image,
                           const Matching::MatchedContour& contour,
                           const Scalar& matchedColor,
                           const Scalar& unmatchedColor,
                           int32_t thickness)
{
    MatchedContourSegment(image, contour, matchedColor, unmatchedColor, thickness, true);
}

void Draw::MatchedContourSegment(QImage& image,
                                  const Matching::MatchedContour& contour,
                                  const Scalar& matchedColor,
                                  const Scalar& unmatchedColor,
                                  int32_t thickness,
                                  bool autoClose)
{
    if (contour.points.size() < 2) return;

    for (size_t i = 1; i < contour.points.size(); ++i) {
        const auto& p0 = contour.points[i - 1];
        const auto& p1 = contour.points[i];

        // Determine color based on quality
        double avgQuality = (p0.quality + p1.quality) * 0.5;
        const Scalar& color = (avgQuality >= 0.5) ? matchedColor : unmatchedColor;

        Line(image,
             static_cast<int32_t>(p0.x), static_cast<int32_t>(p0.y),
             static_cast<int32_t>(p1.x), static_cast<int32_t>(p1.y),
             color, thickness);
    }

    // Close contour if requested
    if (autoClose && contour.points.size() > 2) {
        const auto& first = contour.points.front();
        const auto& last = contour.points.back();
        double avgQuality = (first.quality + last.quality) * 0.5;
        const Scalar& color = (avgQuality >= 0.5) ? matchedColor : unmatchedColor;

        Line(image,
             static_cast<int32_t>(last.x), static_cast<int32_t>(last.y),
             static_cast<int32_t>(first.x), static_cast<int32_t>(first.y),
             color, thickness);
    }
}

} // namespace Qi::Vision
