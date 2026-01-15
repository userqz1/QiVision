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
    // Bresenham's line algorithm
    int32_t dx = std::abs(x2 - x1);
    int32_t dy = std::abs(y2 - y1);
    int32_t sx = (x1 < x2) ? 1 : -1;
    int32_t sy = (y1 < y2) ? 1 : -1;
    int32_t err = dx - dy;

    int32_t halfThick = thickness / 2;

    while (true) {
        // Draw thick pixel
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
// Rectangle
// =============================================================================

void Draw::Rectangle(QImage& image, int32_t x, int32_t y, int32_t width, int32_t height,
                     const Color& color, int32_t thickness) {
    // Top
    Line(image, x, y, x + width - 1, y, color, thickness);
    // Bottom
    Line(image, x, y + height - 1, x + width - 1, y + height - 1, color, thickness);
    // Left
    Line(image, x, y, x, y + height - 1, color, thickness);
    // Right
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

// =============================================================================
// Arrow
// =============================================================================

void Draw::Arrow(QImage& image, const Point2d& from, const Point2d& to,
                 const Color& color, int32_t thickness, double arrowSize) {
    // Draw main line
    Line(image, from, to, color, thickness);

    // Calculate arrow head
    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-6) return;

    dx /= len;
    dy /= len;

    // Arrow head angle (~30 degrees)
    double angle = 0.5;  // ~30 degrees
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    // Left arrow head
    Point2d left{
        to.x - arrowSize * (dx * cosA + dy * sinA),
        to.y - arrowSize * (-dx * sinA + dy * cosA)
    };
    Line(image, to, left, color, thickness);

    // Right arrow head
    Point2d right{
        to.x - arrowSize * (dx * cosA - dy * sinA),
        to.y - arrowSize * (dx * sinA + dy * cosA)
    };
    Line(image, to, right, color, thickness);
}

// =============================================================================
// Polyline
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

    // Four corners relative to center
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
    // Draw rotated cross at match position
    Cross(image, Point2d{match.x, match.y}, markerSize, match.angle, color, 2);

    // Draw angle indicator line
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
        // Fall back to simple marker
        MatchResult(image, match, color, 20);
        return;
    }

    // Transform contour points and draw each as a pixel
    // Model points are scattered edge points, not a continuous contour
    double cosA = std::cos(match.angle);
    double sinA = std::sin(match.angle);

    for (const auto& pt : modelContour) {
        double tx = match.x + match.scaleX * (cosA * pt.x - sinA * pt.y);
        double ty = match.y + match.scaleY * (sinA * pt.x + cosA * pt.y);

        // Draw as 2x2 pixels for visibility
        int32_t px = static_cast<int32_t>(tx + 0.5);
        int32_t py = static_cast<int32_t>(ty + 0.5);
        Pixel(image, px, py, color);
        Pixel(image, px + 1, py, color);
        Pixel(image, px, py + 1, color);
        Pixel(image, px + 1, py + 1, color);
    }

    // Draw rotated center cross
    Cross(image, Point2d{match.x, match.y}, 15, match.angle, color, 2);
}

void Draw::MatchResults(QImage& image, const std::vector<Matching::MatchResult>& matches,
                        const Color& color, int32_t markerSize) {
    // Use different colors for multiple matches
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
// Simple Bitmap Font (5x7 font)
// =============================================================================

// Simple 5x7 bitmap font for digits and basic chars
static const uint8_t FONT_5x7[][7] = {
    // 0-9
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}, // 0
    {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E}, // 1
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}, // 2
    {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E}, // 3
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}, // 4
    {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E}, // 5
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}, // 6
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}, // 7
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}, // 8
    {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C}, // 9
    // Special chars
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // space (10)
    {0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 0x04}, // ! (11)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C}, // . (12)
    {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}, // - (13)
    {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00}, // : (14)
    {0x00, 0x00, 0x02, 0x04, 0x08, 0x00, 0x00}, // / (15)
};

void Draw::Text(QImage& image, int32_t x, int32_t y, const std::string& text,
                const Color& color, int32_t scale) {
    int32_t curX = x;

    for (char c : text) {
        int fontIdx = -1;

        if (c >= '0' && c <= '9') {
            fontIdx = c - '0';
        } else if (c == ' ') {
            fontIdx = 10;
        } else if (c == '!') {
            fontIdx = 11;
        } else if (c == '.') {
            fontIdx = 12;
        } else if (c == '-') {
            fontIdx = 13;
        } else if (c == ':') {
            fontIdx = 14;
        } else if (c == '/') {
            fontIdx = 15;
        }

        if (fontIdx >= 0 && fontIdx < 16) {
            // Draw character
            for (int32_t row = 0; row < 7; ++row) {
                uint8_t bits = FONT_5x7[fontIdx][row];
                for (int32_t col = 0; col < 5; ++col) {
                    if (bits & (0x10 >> col)) {
                        // Draw scaled pixel
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

        curX += 6 * scale;  // Character width + spacing
    }
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

} // namespace Qi::Vision
