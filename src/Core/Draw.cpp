/**
 * @file Draw.cpp
 * @brief Implementation of drawing functions
 */

#include <QiVision/Core/Draw.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace Qi::Vision {

// =============================================================================
// Line Drawing (Bresenham with thickness)
// =============================================================================

void Draw::Line(QImage& image, int32_t x1, int32_t y1, int32_t x2, int32_t y2,
                const Color& color, int32_t thickness) {
    int32_t dx = std::abs(x2 - x1);
    int32_t dy = std::abs(y2 - y1);
    int32_t sx = (x1 < x2) ? 1 : -1;
    int32_t sy = (y1 < y2) ? 1 : -1;
    int32_t err = dx - dy;

    int32_t halfThick = thickness / 2;

    while (true) {
        if (thickness <= 1) {
            Pixel(image, x1, y1, color);
        } else {
            for (int32_t ty = -halfThick; ty <= halfThick; ++ty) {
                for (int32_t tx = -halfThick; tx <= halfThick; ++tx) {
                    Pixel(image, x1 + tx, y1 + ty, color);
                }
            }
        }

        if (x1 == x2 && y1 == y2) break;

        int32_t e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
}

// =============================================================================
// Anti-aliased Line (Wu's algorithm)
// =============================================================================

void Draw::LineAA(QImage& image, double x1, double y1, double x2, double y2,
                  const Color& color) {
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
                     const Color& color, int32_t thickness) {
    Line(image, x, y, x + width - 1, y, color, thickness);
    Line(image, x, y + height - 1, x + width - 1, y + height - 1, color, thickness);
    Line(image, x, y, x, y + height - 1, color, thickness);
    Line(image, x + width - 1, y, x + width - 1, y + height - 1, color, thickness);
}

void Draw::FilledRectangle(QImage& image, const Rect2i& rect, const Color& color) {
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
// Circle (Midpoint algorithm)
// =============================================================================

void Draw::Circle(QImage& image, int32_t cx, int32_t cy, int32_t radius,
                  const Color& color, int32_t thickness) {
    if (radius <= 0) return;

    int32_t x = radius;
    int32_t y = 0;
    int32_t err = 0;

    auto drawCirclePoints = [&](int32_t px, int32_t py) {
        if (thickness <= 1) {
            Pixel(image, cx + px, cy + py, color);
            Pixel(image, cx - px, cy + py, color);
            Pixel(image, cx + px, cy - py, color);
            Pixel(image, cx - px, cy - py, color);
            Pixel(image, cx + py, cy + px, color);
            Pixel(image, cx - py, cy + px, color);
            Pixel(image, cx + py, cy - px, color);
            Pixel(image, cx - py, cy - px, color);
        } else {
            int32_t halfThick = thickness / 2;
            for (int32_t ty = -halfThick; ty <= halfThick; ++ty) {
                for (int32_t tx = -halfThick; tx <= halfThick; ++tx) {
                    Pixel(image, cx + px + tx, cy + py + ty, color);
                    Pixel(image, cx - px + tx, cy + py + ty, color);
                    Pixel(image, cx + px + tx, cy - py + ty, color);
                    Pixel(image, cx - px + tx, cy - py + ty, color);
                    Pixel(image, cx + py + tx, cy + px + ty, color);
                    Pixel(image, cx - py + tx, cy + px + ty, color);
                    Pixel(image, cx + py + tx, cy - px + ty, color);
                    Pixel(image, cx - py + tx, cy - px + ty, color);
                }
            }
        }
    };

    while (x >= y) {
        drawCirclePoints(x, y);
        y++;
        if (err <= 0) {
            err += 2 * y + 1;
        }
        if (err > 0) {
            x--;
            err -= 2 * x + 1;
        }
    }
}

void Draw::FilledCircle(QImage& image, int32_t cx, int32_t cy, int32_t radius,
                        const Color& color) {
    if (radius <= 0) return;

    for (int32_t y = -radius; y <= radius; ++y) {
        int32_t halfWidth = static_cast<int32_t>(std::sqrt(radius * radius - y * y));
        for (int32_t x = -halfWidth; x <= halfWidth; ++x) {
            Pixel(image, cx + x, cy + y, color);
        }
    }
}

// =============================================================================
// Ellipse
// =============================================================================

void Draw::Ellipse(QImage& image, int32_t cx, int32_t cy,
                   int32_t radiusX, int32_t radiusY,
                   const Color& color, int32_t thickness) {
    if (radiusX <= 0 || radiusY <= 0) return;

    // Midpoint ellipse algorithm
    int64_t rx2 = static_cast<int64_t>(radiusX) * radiusX;
    int64_t ry2 = static_cast<int64_t>(radiusY) * radiusY;
    int64_t twoRx2 = 2 * rx2;
    int64_t twoRy2 = 2 * ry2;

    int32_t x = 0;
    int32_t y = radiusY;
    int64_t px = 0;
    int64_t py = twoRx2 * y;

    auto plot4 = [&](int32_t px, int32_t py) {
        if (thickness <= 1) {
            Pixel(image, cx + px, cy + py, color);
            Pixel(image, cx - px, cy + py, color);
            Pixel(image, cx + px, cy - py, color);
            Pixel(image, cx - px, cy - py, color);
        } else {
            int32_t ht = thickness / 2;
            for (int32_t ty = -ht; ty <= ht; ++ty) {
                for (int32_t tx = -ht; tx <= ht; ++tx) {
                    Pixel(image, cx + px + tx, cy + py + ty, color);
                    Pixel(image, cx - px + tx, cy + py + ty, color);
                    Pixel(image, cx + px + tx, cy - py + ty, color);
                    Pixel(image, cx - px + tx, cy - py + ty, color);
                }
            }
        }
    };

    // Region 1
    int64_t p = ry2 - rx2 * radiusY + rx2 / 4;
    while (px < py) {
        plot4(x, y);
        x++;
        px += twoRy2;
        if (p < 0) {
            p += ry2 + px;
        } else {
            y--;
            py -= twoRx2;
            p += ry2 + px - py;
        }
    }

    // Region 2
    p = ry2 * (x * 2 + 1) * (x * 2 + 1) / 4 + rx2 * (y - 1) * (y - 1) - rx2 * ry2;
    while (y >= 0) {
        plot4(x, y);
        y--;
        py -= twoRx2;
        if (p > 0) {
            p += rx2 - py;
        } else {
            x++;
            px += twoRy2;
            p += rx2 - py + px;
        }
    }
}

void Draw::Ellipse(QImage& image, const Point2d& center,
                   double radiusX, double radiusY, double angle,
                   const Color& color, int32_t thickness) {
    // Draw rotated ellipse using parametric form
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    constexpr int32_t numPoints = 72;  // 5 degree steps
    std::vector<Point2d> points;
    points.reserve(numPoints);

    for (int32_t i = 0; i < numPoints; ++i) {
        double t = 2.0 * 3.14159265358979323846 * i / numPoints;
        double x = radiusX * std::cos(t);
        double y = radiusY * std::sin(t);
        // Rotate
        double rx = x * cosA - y * sinA;
        double ry = x * sinA + y * cosA;
        points.push_back({center.x + rx, center.y + ry});
    }

    Polyline(image, points, color, thickness, true);
}

void Draw::FilledEllipse(QImage& image, int32_t cx, int32_t cy,
                         int32_t radiusX, int32_t radiusY, const Color& color) {
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
                 const Color& color, int32_t thickness, double arrowSize) {
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
               const Color& color, int32_t thickness) {
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
                    const Color& color, int32_t thickness, bool closed) {
    if (points.size() < 2) return;

    for (size_t i = 0; i < points.size() - 1; ++i) {
        Line(image, points[i], points[i + 1], color, thickness);
    }

    if (closed && points.size() > 2) {
        Line(image, points.back(), points.front(), color, thickness);
    }
}

void Draw::FilledPolygon(QImage& image, const std::vector<Point2d>& points,
                         const Color& color) {
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
                            const Color& color, int32_t thickness) {
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
                       const Color& color, int32_t markerSize) {
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
                                   const Color& color, int32_t /*thickness*/) {
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
                        const Color& color, int32_t markerSize) {
    std::vector<Color> colors = {
        Color::Green(), Color::Red(), Color::Blue(),
        Color::Yellow(), Color::Cyan(), Color::Magenta()
    };

    for (size_t i = 0; i < matches.size(); ++i) {
        const Color& c = (matches.size() == 1) ? color : colors[i % colors.size()];
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
                const Color& color, int32_t scale) {
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
// Image Conversion
// =============================================================================

QImage Draw::ToRGB(const QImage& gray) {
    if (gray.Channels() == 3) {
        return gray.Clone();
    }

    QImage rgb(gray.Width(), gray.Height(), PixelType::UInt8, ChannelType::RGB);
    const uint8_t* src = static_cast<const uint8_t*>(gray.Data());
    uint8_t* dst = static_cast<uint8_t*>(rgb.Data());
    size_t srcStride = gray.Stride();
    size_t dstStride = rgb.Stride();

    for (int32_t y = 0; y < gray.Height(); ++y) {
        for (int32_t x = 0; x < gray.Width(); ++x) {
            uint8_t val = src[y * srcStride + x];
            uint8_t* px = dst + y * dstStride + x * 3;
            px[0] = val;
            px[1] = val;
            px[2] = val;
        }
    }

    return rgb;
}

QImage Draw::PrepareForDrawing(const QImage& image) {
    if (image.Channels() == 3) {
        return image.Clone();
    }
    return ToRGB(image);
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
                       const Color& color, int32_t thickness) {
    // MeasureRectangle2 uses (row, column) = (y, x) convention
    Point2d center{handle.Column(), handle.Row()};
    // Length1 is along profile direction (phi), Length2 is perpendicular
    double width = 2.0 * handle.Length1();   // Along phi direction
    double height = 2.0 * handle.Length2();  // Perpendicular to phi

    RotatedRectangle(image, center, width, height, handle.Phi(), color, thickness);
}

void Draw::MeasureRects(QImage& image,
                        const std::vector<Measure::MeasureRectangle2>& handles,
                        const Color& color, int32_t thickness) {
    for (const auto& handle : handles) {
        MeasureRect(image, handle, color, thickness);
    }
}

void Draw::Contour(QImage& image, const QContour& contour,
                   const Color& color, int32_t thickness) {
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

void Draw::EdgePoints(QImage& image, const std::vector<Point2d>& points,
                      const Color& color, int32_t markerSize) {
    for (const auto& pt : points) {
        Cross(image, pt, markerSize, color, 1);
    }
}

void Draw::MetrologyLine(QImage& image, const Measure::MetrologyLineResult& result,
                         const Color& color, int32_t thickness) {
    if (!result.IsValid()) return;

    // Draw the fitted line segment
    Point2d p1{result.col1, result.row1};
    Point2d p2{result.col2, result.row2};
    Line(image, p1, p2, color, thickness);
}

void Draw::MetrologyCircle(QImage& image, const Measure::MetrologyCircleResult& result,
                           const Color& color, int32_t thickness) {
    if (!result.IsValid()) return;

    // Draw the fitted circle
    Point2d center{result.column, result.row};
    Circle(image, center, result.radius, color, thickness);
}

void Draw::MetrologyEllipse(QImage& image, const Measure::MetrologyEllipseResult& result,
                            const Color& color, int32_t thickness) {
    if (!result.IsValid()) return;

    // Draw the fitted ellipse (ra = semi-major, rb = semi-minor)
    Point2d center{result.column, result.row};
    Ellipse(image, center, result.ra, result.rb, result.phi, color, thickness);
}

void Draw::MetrologyRectangle(QImage& image, const Measure::MetrologyRectangle2Result& result,
                              const Color& color, int32_t thickness) {
    if (!result.IsValid()) return;

    // Draw the fitted rectangle
    Point2d center{result.column, result.row};
    double width = 2.0 * result.length1;
    double height = 2.0 * result.length2;
    RotatedRectangle(image, center, width, height, result.phi, color, thickness);
}

void Draw::MetrologyModelResult(QImage& image, const Measure::MetrologyModel& model,
                                const Color& objectColor,
                                const Color& resultColor,
                                const Color& pointColor,
                                bool drawCalipers,
                                bool drawPoints) {
    using namespace Measure;

    int32_t numObjects = model.NumObjects();

    for (int32_t idx = 0; idx < numObjects; ++idx) {
        const MetrologyObject* obj = model.GetObject(idx);
        if (!obj) continue;

        // Draw calipers
        if (drawCalipers) {
            auto calipers = obj->GetCalipers();
            MeasureRects(image, calipers, objectColor, 1);
        }

        // Draw edge points
        if (drawPoints) {
            auto points = model.GetMeasuredPoints(idx);
            EdgePoints(image, points, pointColor, 3);
        }

        // Draw result based on object type
        switch (obj->Type()) {
            case MetrologyObjectType::Line: {
                auto result = model.GetLineResult(idx);
                MetrologyLine(image, result, resultColor, 2);
                break;
            }
            case MetrologyObjectType::Circle: {
                auto result = model.GetCircleResult(idx);
                MetrologyCircle(image, result, resultColor, 2);
                break;
            }
            case MetrologyObjectType::Ellipse: {
                auto result = model.GetEllipseResult(idx);
                MetrologyEllipse(image, result, resultColor, 2);
                break;
            }
            case MetrologyObjectType::Rectangle2: {
                auto result = model.GetRectangle2Result(idx);
                MetrologyRectangle(image, result, resultColor, 2);
                break;
            }
        }
    }
}

} // namespace Qi::Vision
