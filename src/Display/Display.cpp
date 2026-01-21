/**
 * @file Display.cpp
 * @brief Image display and drawing functions implementation
 */

#include <QiVision/Display/Display.h>
#include <QiVision/IO/ImageIO.h>
#include <QiVision/Core/Constants.h>
#include <QiVision/Core/QContourArray.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace Qi::Vision {

namespace {

// Output directory for temporary images
std::string g_outputDir;

// Get default output directory
std::string GetDefaultOutputDir() {
#ifdef _WIN32
    const char* temp = std::getenv("TEMP");
    if (temp) {
        return std::string(temp) + "\\qivision\\";
    }
    return "C:\\Temp\\qivision\\";
#else
    return "/tmp/qivision/";
#endif
}

// Sanitize filename
std::string SanitizeFilename(const std::string& name) {
    std::string result;
    result.reserve(name.size());
    for (char c : name) {
        if (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-') {
            result += c;
        } else if (c == ' ') {
            result += '_';
        }
    }
    return result.empty() ? "image" : result;
}

// Check if running in WSL
bool IsWSL() {
    static int isWsl = -1;
    if (isWsl < 0) {
        isWsl = 0;
#ifndef _WIN32
        std::filesystem::path procVersion("/proc/version");
        if (std::filesystem::exists(procVersion)) {
            std::ifstream file(procVersion);
            std::string content;
            std::getline(file, content);
            std::transform(content.begin(), content.end(), content.begin(),
                [](unsigned char c) { return std::tolower(c); });
            if (content.find("microsoft") != std::string::npos ||
                content.find("wsl") != std::string::npos) {
                isWsl = 1;
            }
        }
#endif
    }
    return isWsl == 1;
}

// Open image with system viewer
bool OpenWithViewer(const std::string& filepath) {
#ifdef _WIN32
    std::string cmd = "start \"\" \"" + filepath + "\"";
    return std::system(cmd.c_str()) == 0;
#else
    if (IsWSL()) {
        std::string cmd = "explorer.exe \"$(wslpath -w '" + filepath + "')\" 2>/dev/null &";
        return std::system(cmd.c_str()) == 0;
    } else {
        std::string cmd = "xdg-open \"" + filepath + "\" 2>/dev/null &";
        return std::system(cmd.c_str()) == 0;
    }
#endif
}

// Draw pixel with bounds checking
void DrawPixel(QImage& image, int32_t col, int32_t row, const Scalar& color) {
    if (col < 0 || col >= image.Width() || row < 0 || row >= image.Height()) {
        return;
    }

    uint8_t* data = static_cast<uint8_t*>(image.Data());
    size_t stride = image.Stride();
    int channels = image.Channels();

    if (channels == 1) {
        uint8_t gray = static_cast<uint8_t>(0.299 * color.r + 0.587 * color.g + 0.114 * color.b);
        data[row * stride + col] = gray;
    } else if (channels >= 3) {
        uint8_t* px = data + row * stride + col * channels;
        px[0] = color.r;
        px[1] = color.g;
        px[2] = color.b;
    }
}

// Bresenham line algorithm
void DrawLineBresenham(QImage& image, int32_t x0, int32_t y0, int32_t x1, int32_t y1,
                       const Scalar& color, int32_t thickness) {
    int32_t dx = std::abs(x1 - x0);
    int32_t dy = std::abs(y1 - y0);
    int32_t sx = x0 < x1 ? 1 : -1;
    int32_t sy = y0 < y1 ? 1 : -1;
    int32_t err = dx - dy;

    while (true) {
        // Draw thick line by drawing a small circle at each point
        if (thickness <= 1) {
            DrawPixel(image, x0, y0, color);
        } else {
            int32_t r = thickness / 2;
            for (int32_t dy2 = -r; dy2 <= r; ++dy2) {
                for (int32_t dx2 = -r; dx2 <= r; ++dx2) {
                    if (dx2 * dx2 + dy2 * dy2 <= r * r) {
                        DrawPixel(image, x0 + dx2, y0 + dy2, color);
                    }
                }
            }
        }

        if (x0 == x1 && y0 == y1) break;

        int32_t e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

// Midpoint circle algorithm
void DrawCircleBresenham(QImage& image, int32_t cx, int32_t cy, int32_t radius,
                         const Scalar& color, int32_t thickness) {
    if (radius <= 0) return;

    int32_t x = radius;
    int32_t y = 0;
    int32_t err = 0;

    auto drawPoints = [&](int32_t px, int32_t py) {
        if (thickness <= 1) {
            DrawPixel(image, cx + px, cy + py, color);
            DrawPixel(image, cx - px, cy + py, color);
            DrawPixel(image, cx + px, cy - py, color);
            DrawPixel(image, cx - px, cy - py, color);
            DrawPixel(image, cx + py, cy + px, color);
            DrawPixel(image, cx - py, cy + px, color);
            DrawPixel(image, cx + py, cy - px, color);
            DrawPixel(image, cx - py, cy - px, color);
        } else {
            int32_t r = thickness / 2;
            for (int32_t dy = -r; dy <= r; ++dy) {
                for (int32_t dx = -r; dx <= r; ++dx) {
                    if (dx * dx + dy * dy <= r * r) {
                        DrawPixel(image, cx + px + dx, cy + py + dy, color);
                        DrawPixel(image, cx - px + dx, cy + py + dy, color);
                        DrawPixel(image, cx + px + dx, cy - py + dy, color);
                        DrawPixel(image, cx - px + dx, cy - py + dy, color);
                        DrawPixel(image, cx + py + dx, cy + px + dy, color);
                        DrawPixel(image, cx - py + dx, cy + px + dy, color);
                        DrawPixel(image, cx + py + dx, cy - px + dy, color);
                        DrawPixel(image, cx - py + dx, cy - px + dy, color);
                    }
                }
            }
        }
    };

    while (x >= y) {
        drawPoints(x, y);
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

} // anonymous namespace

// =============================================================================
// Image Display Functions
// =============================================================================

bool DispImage(const QImage& image, const std::string& title) {
    if (image.Empty()) {
        return false;
    }

    std::string outDir = g_outputDir.empty() ? GetDefaultOutputDir() : g_outputDir;
    std::filesystem::create_directories(outDir);

    std::string filename = SanitizeFilename(title) + ".png";
    std::string filepath = outDir + filename;

    if (!IO::WriteImage(image, filepath)) {
        return false;
    }

    return OpenWithViewer(filepath);
}

void SetDispOutputDir(const std::string& path) {
    g_outputDir = path;
    if (!g_outputDir.empty() && g_outputDir.back() != '/' && g_outputDir.back() != '\\') {
#ifdef _WIN32
        g_outputDir += '\\';
#else
        g_outputDir += '/';
#endif
    }
}

void CleanDispImages() {
    std::string outDir = g_outputDir.empty() ? GetDefaultOutputDir() : g_outputDir;
    try {
        for (const auto& entry : std::filesystem::directory_iterator(outDir)) {
            if (entry.is_regular_file()) {
                auto ext = entry.path().extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(),
                    [](unsigned char c) { return std::tolower(c); });
                if (ext == ".png" || ext == ".jpg" || ext == ".bmp") {
                    std::filesystem::remove(entry.path());
                }
            }
        }
    } catch (...) {}
}

// =============================================================================
// Drawing Primitives - Lines
// =============================================================================

void DispLine(QImage& image, double x1, double y1, double x2, double y2,
              const Scalar& color, int32_t thickness) {
    DrawLineBresenham(image,
                      static_cast<int32_t>(x1 + 0.5), static_cast<int32_t>(y1 + 0.5),
                      static_cast<int32_t>(x2 + 0.5), static_cast<int32_t>(y2 + 0.5),
                      color, thickness);
}

void DispLine(QImage& image, const Line2d& line, double length,
              const Scalar& color, int32_t thickness) {
    // Line2d: ax + by + c = 0, with a^2 + b^2 = 1
    // Direction perpendicular to (a, b) is (-b, a)
    double halfLen = length / 2.0;
    double cx = -line.a * line.c;  // Point on line
    double cy = -line.b * line.c;
    double dx = -line.b * halfLen;
    double dy = line.a * halfLen;

    DispLine(image, cx - dx, cy - dy, cx + dx, cy + dy, color, thickness);
}

// =============================================================================
// Drawing Primitives - Circles
// =============================================================================

void DispCircle(QImage& image, double cx, double cy, double radius,
                const Scalar& color, int32_t thickness) {
    DrawCircleBresenham(image,
                        static_cast<int32_t>(cx + 0.5),
                        static_cast<int32_t>(cy + 0.5),
                        static_cast<int32_t>(radius + 0.5),
                        color, thickness);
}

void DispCircle(QImage& image, const Circle2d& circle,
                const Scalar& color, int32_t thickness) {
    DispCircle(image, circle.center.x, circle.center.y, circle.radius, color, thickness);
}

void DispEllipse(QImage& image, double cx, double cy, double angle,
                 double radiusX, double radiusY,
                 const Scalar& color, int32_t thickness) {
    // Draw ellipse using parametric form
    int numPoints = std::max(64, static_cast<int>((radiusX + radiusY) * PI / 5.0));
    double step = 2.0 * PI / numPoints;
    double cosAngle = std::cos(angle);
    double sinAngle = std::sin(angle);

    for (int i = 0; i < numPoints; ++i) {
        double t1 = i * step;
        double t2 = (i + 1) * step;

        double lx1 = radiusX * std::cos(t1);
        double ly1 = radiusY * std::sin(t1);
        double lx2 = radiusX * std::cos(t2);
        double ly2 = radiusY * std::sin(t2);

        double x1 = cx + lx1 * cosAngle - ly1 * sinAngle;
        double y1 = cy + lx1 * sinAngle + ly1 * cosAngle;
        double x2 = cx + lx2 * cosAngle - ly2 * sinAngle;
        double y2 = cy + lx2 * sinAngle + ly2 * cosAngle;

        DispLine(image, x1, y1, x2, y2, color, thickness);
    }
}

void DispEllipse(QImage& image, const Ellipse2d& ellipse,
                 const Scalar& color, int32_t thickness) {
    DispEllipse(image, ellipse.center.x, ellipse.center.y, ellipse.angle,
                ellipse.a, ellipse.b, color, thickness);
}

// =============================================================================
// Drawing Primitives - Rectangles
// =============================================================================

void DispRectangle1(QImage& image, double x1, double y1, double x2, double y2,
                    const Scalar& color, int32_t thickness) {
    DispLine(image, x1, y1, x2, y1, color, thickness);  // Top
    DispLine(image, x1, y2, x2, y2, color, thickness);  // Bottom
    DispLine(image, x1, y1, x1, y2, color, thickness);  // Left
    DispLine(image, x2, y1, x2, y2, color, thickness);  // Right
}

void DispRectangle1(QImage& image, const Rect2i& rect,
                    const Scalar& color, int32_t thickness) {
    DispRectangle1(image, rect.x, rect.y,
                   rect.x + rect.width - 1, rect.y + rect.height - 1,
                   color, thickness);
}

void DispRectangle2(QImage& image, double cx, double cy, double angle,
                    double halfWidth, double halfHeight,
                    const Scalar& color, int32_t thickness) {
    double cosAngle = std::cos(angle);
    double sinAngle = std::sin(angle);

    // 4 corners in local coordinates (x, y)
    double corners[4][2] = {
        {-halfWidth, -halfHeight},
        { halfWidth, -halfHeight},
        { halfWidth,  halfHeight},
        {-halfWidth,  halfHeight}
    };

    // Transform corners to image coordinates
    double imgCorners[4][2];
    for (int i = 0; i < 4; ++i) {
        imgCorners[i][0] = cx + corners[i][0] * cosAngle - corners[i][1] * sinAngle;  // x
        imgCorners[i][1] = cy + corners[i][0] * sinAngle + corners[i][1] * cosAngle;  // y
    }

    // Draw 4 edges
    for (int i = 0; i < 4; ++i) {
        int j = (i + 1) % 4;
        DispLine(image, imgCorners[i][0], imgCorners[i][1],
                 imgCorners[j][0], imgCorners[j][1], color, thickness);
    }
}

// =============================================================================
// Drawing Primitives - Markers
// =============================================================================

void DispCross(QImage& image, double cx, double cy, int32_t size,
               double angle, const Scalar& color, int32_t thickness) {
    double s = static_cast<double>(size);
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    // Horizontal arm (along angle direction)
    DispLine(image, cx - s * cosA, cy - s * sinA,
             cx + s * cosA, cy + s * sinA, color, thickness);
    // Vertical arm (perpendicular to angle)
    DispLine(image, cx + s * sinA, cy - s * cosA,
             cx - s * sinA, cy + s * cosA, color, thickness);
}

void DispArrow(QImage& image, double x1, double y1, double x2, double y2,
               double headSize, const Scalar& color, int32_t thickness) {
    // Main line
    DispLine(image, x1, y1, x2, y2, color, thickness);

    // Arrow head
    double angle = std::atan2(y2 - y1, x2 - x1);
    double headAngle = PI / 6.0;  // 30 degrees

    double hx1 = x2 - headSize * std::cos(angle + headAngle);
    double hy1 = y2 - headSize * std::sin(angle + headAngle);
    double hx2 = x2 - headSize * std::cos(angle - headAngle);
    double hy2 = y2 - headSize * std::sin(angle - headAngle);

    DispLine(image, x2, y2, hx1, hy1, color, thickness);
    DispLine(image, x2, y2, hx2, hy2, color, thickness);
}

// =============================================================================
// Drawing Primitives - Polygons and Contours
// =============================================================================

void DispPolygon(QImage& image, const std::vector<double>& xs,
                 const std::vector<double>& ys,
                 const Scalar& color, int32_t thickness) {
    if (xs.size() != ys.size() || xs.size() < 2) {
        return;
    }

    for (size_t i = 0; i < xs.size(); ++i) {
        size_t j = (i + 1) % xs.size();
        DispLine(image, xs[i], ys[i], xs[j], ys[j], color, thickness);
    }
}

void DispContour(QImage& image, const QContour& contour,
                 const Scalar& color, int32_t thickness) {
    if (contour.Size() < 2) {
        return;
    }

    for (size_t i = 0; i < contour.Size() - 1; ++i) {
        const auto& p1 = contour.PointAt(i);
        const auto& p2 = contour.PointAt(i + 1);
        DispLine(image, p1.x, p1.y, p2.x, p2.y, color, thickness);
    }

    // Close if contour is closed
    if (contour.IsClosed() && contour.Size() >= 3) {
        const auto& p1 = contour.PointAt(contour.Size() - 1);
        const auto& p2 = contour.PointAt(0);
        DispLine(image, p1.x, p1.y, p2.x, p2.y, color, thickness);
    }
}

void DispContours(QImage& image, const QContourArray& contours,
                  const Scalar& color, int32_t thickness) {
    for (size_t i = 0; i < contours.Size(); ++i) {
        DispContour(image, contours[i], color, thickness);
    }
}

// =============================================================================
// Drawing Primitives - Points
// =============================================================================

void DispPoint(QImage& image, double x, double y, const Scalar& color) {
    DrawPixel(image, static_cast<int32_t>(x + 0.5),
              static_cast<int32_t>(y + 0.5), color);
}

void DispPoints(QImage& image, const std::vector<double>& xs,
                const std::vector<double>& ys, const Scalar& color) {
    size_t n = std::min(xs.size(), ys.size());
    for (size_t i = 0; i < n; ++i) {
        DispPoint(image, xs[i], ys[i], color);
    }
}

// =============================================================================
// Drawing Primitives - Text (Simple bitmap font)
// =============================================================================

namespace {
// Simple 5x7 bitmap font for digits and uppercase letters
const uint8_t FONT_5X7[][7] = {
    // 0-9
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E},  // 0
    {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},  // 1
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F},  // 2
    {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E},  // 3
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02},  // 4
    {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},  // 5
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E},  // 6
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},  // 7
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E},  // 8
    {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},  // 9
};

void DrawChar(QImage& image, int32_t col, int32_t row, char c,
              const Scalar& color, int32_t scale) {
    const uint8_t* bitmap = nullptr;

    if (c >= '0' && c <= '9') {
        bitmap = FONT_5X7[c - '0'];
    } else {
        return;  // Unsupported character
    }

    for (int y = 0; y < 7; ++y) {
        for (int x = 0; x < 5; ++x) {
            if (bitmap[y] & (0x10 >> x)) {
                for (int sy = 0; sy < scale; ++sy) {
                    for (int sx = 0; sx < scale; ++sx) {
                        DrawPixel(image, col + x * scale + sx, row + y * scale + sy, color);
                    }
                }
            }
        }
    }
}
}

void DispText(QImage& image, double x, double y, const std::string& text,
              const Scalar& color, int32_t scale) {
    int32_t px = static_cast<int32_t>(x + 0.5);
    int32_t py = static_cast<int32_t>(y + 0.5);
    int32_t charWidth = 6 * scale;

    for (char c : text) {
        DrawChar(image, px, py, c, color, scale);
        px += charWidth;
    }
}

// =============================================================================
// High-Level Drawing Functions
// =============================================================================

void DispMatchResult(QImage& image, double x, double y, double angle,
                     double score, const Scalar& color, int32_t markerSize) {
    // Draw cross at match position
    DispCross(image, x, y, markerSize, angle, color, 2);

    // Draw angle indicator line
    double indicatorLen = markerSize * 1.5;
    DispLine(image, x, y,
             x + indicatorLen * std::cos(angle),
             y + indicatorLen * std::sin(angle),
             color, 2);

    // Display score text if valid
    if (score > 0.0 && score <= 1.0) {
        char scoreText[16];
        std::snprintf(scoreText, sizeof(scoreText), "%.2f", score);
        DispText(image, x - 10, y + markerSize + 5, scoreText, color, 1);
    }
}

void DispEdgeResult(QImage& image, double x, double y,
                    const Scalar& color, int32_t markerSize) {
    DispCross(image, x, y, markerSize, 0.0, color, 1);
}

} // namespace Qi::Vision
