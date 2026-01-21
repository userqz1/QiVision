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

void DispLine(QImage& image, double row1, double col1, double row2, double col2,
              const Scalar& color, int32_t thickness) {
    DrawLineBresenham(image,
                      static_cast<int32_t>(col1 + 0.5), static_cast<int32_t>(row1 + 0.5),
                      static_cast<int32_t>(col2 + 0.5), static_cast<int32_t>(row2 + 0.5),
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

    DispLine(image, cy - dy, cx - dx, cy + dy, cx + dx, color, thickness);
}

// =============================================================================
// Drawing Primitives - Circles
// =============================================================================

void DispCircle(QImage& image, double row, double column, double radius,
                const Scalar& color, int32_t thickness) {
    DrawCircleBresenham(image,
                        static_cast<int32_t>(column + 0.5),
                        static_cast<int32_t>(row + 0.5),
                        static_cast<int32_t>(radius + 0.5),
                        color, thickness);
}

void DispCircle(QImage& image, const Circle2d& circle,
                const Scalar& color, int32_t thickness) {
    DispCircle(image, circle.center.y, circle.center.x, circle.radius, color, thickness);
}

void DispEllipse(QImage& image, double row, double column, double phi,
                 double ra, double rb,
                 const Scalar& color, int32_t thickness) {
    // Draw ellipse using parametric form
    int numPoints = std::max(64, static_cast<int>((ra + rb) * PI / 5.0));
    double step = 2.0 * PI / numPoints;
    double cosPhi = std::cos(phi);
    double sinPhi = std::sin(phi);

    for (int i = 0; i < numPoints; ++i) {
        double t1 = i * step;
        double t2 = (i + 1) * step;

        double x1 = ra * std::cos(t1);
        double y1 = rb * std::sin(t1);
        double x2 = ra * std::cos(t2);
        double y2 = rb * std::sin(t2);

        double col1 = column + x1 * cosPhi - y1 * sinPhi;
        double row1_pt = row + x1 * sinPhi + y1 * cosPhi;
        double col2 = column + x2 * cosPhi - y2 * sinPhi;
        double row2_pt = row + x2 * sinPhi + y2 * cosPhi;

        DispLine(image, row1_pt, col1, row2_pt, col2, color, thickness);
    }
}

void DispEllipse(QImage& image, const Ellipse2d& ellipse,
                 const Scalar& color, int32_t thickness) {
    DispEllipse(image, ellipse.center.y, ellipse.center.x, ellipse.angle,
                ellipse.a, ellipse.b, color, thickness);
}

// =============================================================================
// Drawing Primitives - Rectangles
// =============================================================================

void DispRectangle1(QImage& image, double row1, double col1, double row2, double col2,
                    const Scalar& color, int32_t thickness) {
    DispLine(image, row1, col1, row1, col2, color, thickness);  // Top
    DispLine(image, row2, col1, row2, col2, color, thickness);  // Bottom
    DispLine(image, row1, col1, row2, col1, color, thickness);  // Left
    DispLine(image, row1, col2, row2, col2, color, thickness);  // Right
}

void DispRectangle1(QImage& image, const Rect2i& rect,
                    const Scalar& color, int32_t thickness) {
    DispRectangle1(image, rect.y, rect.x,
                   rect.y + rect.height - 1, rect.x + rect.width - 1,
                   color, thickness);
}

void DispRectangle2(QImage& image, double row, double column, double phi,
                    double length1, double length2,
                    const Scalar& color, int32_t thickness) {
    double cosPhi = std::cos(phi);
    double sinPhi = std::sin(phi);

    // 4 corners in local coordinates
    double corners[4][2] = {
        {-length1, -length2},
        { length1, -length2},
        { length1,  length2},
        {-length1,  length2}
    };

    // Transform corners to image coordinates
    double imgCorners[4][2];
    for (int i = 0; i < 4; ++i) {
        imgCorners[i][0] = row + corners[i][0] * sinPhi + corners[i][1] * cosPhi;    // row
        imgCorners[i][1] = column + corners[i][0] * cosPhi - corners[i][1] * sinPhi; // col
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

void DispCross(QImage& image, double row, double column, int32_t size,
               double angle, const Scalar& color, int32_t thickness) {
    double s = static_cast<double>(size);
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    // Horizontal arm
    DispLine(image, row - s * sinA, column - s * cosA,
             row + s * sinA, column + s * cosA, color, thickness);
    // Vertical arm
    DispLine(image, row - s * cosA, column + s * sinA,
             row + s * cosA, column - s * sinA, color, thickness);
}

void DispArrow(QImage& image, double row1, double col1, double row2, double col2,
               double headSize, const Scalar& color, int32_t thickness) {
    // Main line
    DispLine(image, row1, col1, row2, col2, color, thickness);

    // Arrow head
    double angle = std::atan2(row2 - row1, col2 - col1);
    double headAngle = PI / 6.0;  // 30 degrees

    double r1 = row2 - headSize * std::sin(angle + headAngle);
    double c1 = col2 - headSize * std::cos(angle + headAngle);
    double r2 = row2 - headSize * std::sin(angle - headAngle);
    double c2 = col2 - headSize * std::cos(angle - headAngle);

    DispLine(image, row2, col2, r1, c1, color, thickness);
    DispLine(image, row2, col2, r2, c2, color, thickness);
}

// =============================================================================
// Drawing Primitives - Polygons and Contours
// =============================================================================

void DispPolygon(QImage& image, const std::vector<double>& rows,
                 const std::vector<double>& cols,
                 const Scalar& color, int32_t thickness) {
    if (rows.size() != cols.size() || rows.size() < 2) {
        return;
    }

    for (size_t i = 0; i < rows.size(); ++i) {
        size_t j = (i + 1) % rows.size();
        DispLine(image, rows[i], cols[i], rows[j], cols[j], color, thickness);
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
        DispLine(image, p1.y, p1.x, p2.y, p2.x, color, thickness);
    }

    // Close if contour is closed
    if (contour.IsClosed() && contour.Size() >= 3) {
        const auto& p1 = contour.PointAt(contour.Size() - 1);
        const auto& p2 = contour.PointAt(0);
        DispLine(image, p1.y, p1.x, p2.y, p2.x, color, thickness);
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

void DispPoint(QImage& image, double row, double column, const Scalar& color) {
    DrawPixel(image, static_cast<int32_t>(column + 0.5),
              static_cast<int32_t>(row + 0.5), color);
}

void DispPoints(QImage& image, const std::vector<double>& rows,
                const std::vector<double>& cols, const Scalar& color) {
    size_t n = std::min(rows.size(), cols.size());
    for (size_t i = 0; i < n; ++i) {
        DispPoint(image, rows[i], cols[i], color);
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

void DispText(QImage& image, double row, double column, const std::string& text,
              const Scalar& color, int32_t scale) {
    int32_t col = static_cast<int32_t>(column + 0.5);
    int32_t r = static_cast<int32_t>(row + 0.5);
    int32_t charWidth = 6 * scale;

    for (char c : text) {
        DrawChar(image, col, r, c, color, scale);
        col += charWidth;
    }
}

// =============================================================================
// High-Level Drawing Functions
// =============================================================================

void DispMatchResult(QImage& image, double row, double column, double angle,
                     double score, const Scalar& color, int32_t markerSize) {
    // Draw cross at match position
    DispCross(image, row, column, markerSize, angle, color, 2);

    // Draw angle indicator line
    double indicatorLen = markerSize * 1.5;
    DispLine(image, row, column,
             row + indicatorLen * std::sin(angle),
             column + indicatorLen * std::cos(angle),
             color, 2);

    // Display score text if valid
    if (score > 0.0 && score <= 1.0) {
        char scoreText[16];
        std::snprintf(scoreText, sizeof(scoreText), "%.2f", score);
        DispText(image, row + markerSize + 5, column - 10, scoreText, color, 1);
    }
}

void DispEdgeResult(QImage& image, double row, double column,
                    const Scalar& color, int32_t markerSize) {
    DispCross(image, row, column, markerSize, 0.0, color, 1);
}

} // namespace Qi::Vision
