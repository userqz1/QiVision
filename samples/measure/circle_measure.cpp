/**
 * @file circle_measure.cpp
 * @brief Circle measurement sample using Metrology module
 *
 * Demonstrates:
 * - GUI display with zoom/pan
 * - Creating a MetrologyModel with fixed circle parameters
 * - Configuring measurement parameters (measure_length, num_measures, etc.)
 * - Executing measurement and retrieving results
 * - Visualizing results (overlay on image)
 *
 * Usage:
 * 1. Run the program: ./measure_circle [image_dir]
 *    - [image_dir]: Image directory (default: tests/data/matching/image3)
 * 2. Left drag to pan, scroll wheel to zoom, right click to reset view
 * 3. Press any key for next image, 'q' to quit
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Measure/Metrology.h>
#include <QiVision/GUI/Window.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <filesystem>
#include <string>
#include <vector>

using namespace Qi::Vision;
using namespace Qi::Vision::Measure;
using namespace Qi::Vision::GUI;

namespace fs = std::filesystem;

// Draw a line on the image
void DrawLine(uint8_t* data, int width, int height, size_t stride, int channels,
              double x1, double y1, double x2, double y2,
              uint8_t r, uint8_t g, uint8_t b)
{
    auto drawPixel = [&](int x, int y) {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            uint8_t* pixel = data + y * stride + x * channels;
            pixel[0] = r;
            if (channels >= 3) {
                pixel[1] = g;
                pixel[2] = b;
            }
        }
    };

    // Bresenham's line algorithm
    int ix1 = static_cast<int>(x1), iy1 = static_cast<int>(y1);
    int ix2 = static_cast<int>(x2), iy2 = static_cast<int>(y2);
    int dx = std::abs(ix2 - ix1), dy = std::abs(iy2 - iy1);
    int sx = ix1 < ix2 ? 1 : -1, sy = iy1 < iy2 ? 1 : -1;
    int err = dx - dy;

    while (true) {
        drawPixel(ix1, iy1);
        if (ix1 == ix2 && iy1 == iy2) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; ix1 += sx; }
        if (e2 < dx) { err += dx; iy1 += sy; }
    }
}

// Draw edge points on the image
void DrawEdgePoints(QImage& image, const std::vector<Point2d>& points,
                    uint8_t r, uint8_t g, uint8_t b, int radius = 3)
{
    uint8_t* data = static_cast<uint8_t*>(image.Data());
    int width = image.Width();
    int height = image.Height();
    size_t stride = image.Stride();
    int channels = image.Channels();

    for (const auto& pt : points) {
        int cx = static_cast<int>(pt.x);
        int cy = static_cast<int>(pt.y);

        // Draw filled circle
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                if (dx*dx + dy*dy <= radius*radius) {
                    int x = cx + dx;
                    int y = cy + dy;
                    if (x >= 0 && x < width && y >= 0 && y < height) {
                        uint8_t* pixel = data + y * stride + x * channels;
                        pixel[0] = r;
                        if (channels >= 3) {
                            pixel[1] = g;
                            pixel[2] = b;
                        }
                    }
                }
            }
        }
    }
}

// Draw edge points with colors based on weights (green=inlier, red=outlier)
void DrawEdgePointsWithWeights(QImage& image, const std::vector<Point2d>& points,
                                const std::vector<double>& weights, int radius = 4)
{
    uint8_t* data = static_cast<uint8_t*>(image.Data());
    int width = image.Width();
    int height = image.Height();
    size_t stride = image.Stride();
    int channels = image.Channels();

    for (size_t i = 0; i < points.size(); ++i) {
        const auto& pt = points[i];
        int cx = static_cast<int>(pt.x);
        int cy = static_cast<int>(pt.y);

        // Get weight (default to 1.0 if weights not available)
        double w = (i < weights.size()) ? weights[i] : 1.0;

        // Color interpolation: green (inlier) -> yellow -> red (outlier)
        // weight=1.0 -> green, weight=0.5 -> yellow, weight=0.0 -> red
        uint8_t r, g, b;
        if (w >= 0.8) {
            // Inlier: green
            r = 0; g = 255; b = 0;
        } else if (w >= 0.5) {
            // Moderate: yellow
            r = 255; g = 255; b = 0;
        } else {
            // Outlier: red
            r = 255; g = 0; b = 0;
        }

        // Draw filled circle
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                if (dx*dx + dy*dy <= radius*radius) {
                    int x = cx + dx;
                    int y = cy + dy;
                    if (x >= 0 && x < width && y >= 0 && y < height) {
                        uint8_t* pixel = data + y * stride + x * channels;
                        pixel[0] = r;
                        if (channels >= 3) {
                            pixel[1] = g;
                            pixel[2] = b;
                        }
                    }
                }
            }
        }
    }
}

// Draw a circle overlay on the image (for visualization)
QImage DrawCircleOverlay(const QImage& src, double centerX, double centerY, double radius,
                         uint8_t r, uint8_t g, uint8_t b, int thickness = 2)
{
    // Convert to RGB for colored overlay
    QImage result = src;
    if (result.Channels() == 1) {
        // Convert grayscale to RGB
        QImage rgb(result.Width(), result.Height(), PixelType::UInt8, ChannelType::RGB);
        const uint8_t* srcData = static_cast<const uint8_t*>(result.Data());
        uint8_t* dstData = static_cast<uint8_t*>(rgb.Data());
        size_t srcStride = result.Stride();
        size_t dstStride = rgb.Stride();

        for (int y = 0; y < result.Height(); ++y) {
            const uint8_t* srcRow = srcData + y * srcStride;
            uint8_t* dstRow = dstData + y * dstStride;
            for (int x = 0; x < result.Width(); ++x) {
                dstRow[x * 3 + 0] = srcRow[x];  // R
                dstRow[x * 3 + 1] = srcRow[x];  // G
                dstRow[x * 3 + 2] = srcRow[x];  // B
            }
        }
        result = std::move(rgb);
    }

    // Draw circle using parametric method
    int width = result.Width();
    int height = result.Height();
    uint8_t* data = static_cast<uint8_t*>(result.Data());
    size_t stride = result.Stride();
    int channels = result.Channels();

    auto drawPixel = [&](int x, int y) {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            uint8_t* pixel = data + y * stride + x * channels;
            pixel[0] = r;
            if (channels >= 3) {
                pixel[1] = g;
                pixel[2] = b;
            }
        }
    };

    // Draw with thickness
    int numPoints = static_cast<int>(2 * M_PI * radius * 2);
    numPoints = std::max(numPoints, 360);

    for (int i = 0; i < numPoints; ++i) {
        double angle = 2.0 * M_PI * i / numPoints;
        double px = centerX + radius * std::cos(angle);
        double py = centerY + radius * std::sin(angle);

        // Draw thick point
        for (int dy = -thickness/2; dy <= thickness/2; ++dy) {
            for (int dx = -thickness/2; dx <= thickness/2; ++dx) {
                drawPixel(static_cast<int>(px + dx), static_cast<int>(py + dy));
            }
        }
    }

    // Draw center cross
    int cx = static_cast<int>(centerX);
    int cy = static_cast<int>(centerY);
    for (int d = -5; d <= 5; ++d) {
        drawPixel(cx + d, cy);
        drawPixel(cx, cy + d);
    }

    return result;
}

// Draw calipers (projection rectangles) around the circle
void DrawCalipers(QImage& image, double centerX, double centerY, double radius,
                  int numMeasures, double measureLength1, double measureLength2,
                  uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t* data = static_cast<uint8_t*>(image.Data());
    int width = image.Width();
    int height = image.Height();
    size_t stride = image.Stride();
    int channels = image.Channels();

    for (int i = 0; i < numMeasures; ++i) {
        double angle = 2.0 * M_PI * i / numMeasures;

        // Position on circle (center of the rectangle)
        double cx = centerX + radius * std::cos(angle);
        double cy = centerY + radius * std::sin(angle);

        // Direction vectors
        // Radial direction (along measureLength1)
        double radX = std::cos(angle);
        double radY = std::sin(angle);
        // Perpendicular direction (along measureLength2)
        double perpX = -std::sin(angle);
        double perpY = std::cos(angle);

        // Four corners of the rectangle
        // Inner-left, inner-right, outer-right, outer-left
        double x1 = cx - measureLength1 * radX - measureLength2 * perpX;
        double y1 = cy - measureLength1 * radY - measureLength2 * perpY;
        double x2 = cx - measureLength1 * radX + measureLength2 * perpX;
        double y2 = cy - measureLength1 * radY + measureLength2 * perpY;
        double x3 = cx + measureLength1 * radX + measureLength2 * perpX;
        double y3 = cy + measureLength1 * radY + measureLength2 * perpY;
        double x4 = cx + measureLength1 * radX - measureLength2 * perpX;
        double y4 = cy + measureLength1 * radY - measureLength2 * perpY;

        // Draw rectangle (4 edges)
        DrawLine(data, width, height, stride, channels, x1, y1, x2, y2, r, g, b);
        DrawLine(data, width, height, stride, channels, x2, y2, x3, y3, r, g, b);
        DrawLine(data, width, height, stride, channels, x3, y3, x4, y4, r, g, b);
        DrawLine(data, width, height, stride, channels, x4, y4, x1, y1, r, g, b);
    }
}

// Print measurement parameters
void PrintParams(const MetrologyMeasureParams& params) {
    std::cout << "\n=== Measurement Parameters ===\n";
    std::cout << "  measureLength1 (projection length): " << params.measureLength1 << " px\n";
    std::cout << "  measureLength2 (projection width):  " << params.measureLength2 << " px\n";
    std::cout << "  numMeasures (projection count):     " << params.numMeasures << "\n";
    std::cout << "  measureSigma:                       " << params.measureSigma << "\n";
    std::cout << "  measureThreshold:                   ";
    if (params.thresholdMode == ThresholdMode::Auto) {
        std::cout << "auto\n";
    } else {
        std::cout << params.measureThreshold << "\n";
    }
    std::cout << "  numInstances:                       " << params.numInstances << "\n";
    std::cout << "  minScore:                           " << params.minScore << "\n";
}

int main(int argc, char* argv[]) {
    std::cout << "==============================================\n";
    std::cout << "  QiVision Circle Measurement Demo\n";
    std::cout << "==============================================\n\n";

    // Default image directory
    std::string imageDir = "tests/data/matching/image3";

    // Allow override from command line
    if (argc > 1) {
        imageDir = argv[1];
    }

    // Find images in directory
    std::vector<std::string> imagePaths;
    if (fs::exists(imageDir) && fs::is_directory(imageDir)) {
        for (const auto& entry : fs::directory_iterator(imageDir)) {
            std::string ext = entry.path().extension().string();
            // Convert extension to lowercase
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            if (ext == ".bmp" || ext == ".png" || ext == ".jpg" || ext == ".jpeg") {
                imagePaths.push_back(entry.path().string());
            }
        }
        std::sort(imagePaths.begin(), imagePaths.end());
    }

    if (imagePaths.empty()) {
        std::cerr << "No images found in: " << imageDir << "\n";
        return 1;
    }

    std::cout << "Found " << imagePaths.size() << " images in " << imageDir << "\n";

    // Load first image to get dimensions
    QImage firstImage = QImage::FromFile(imagePaths[0]);
    if (!firstImage.IsValid()) {
        std::cerr << "Failed to load: " << imagePaths[0] << "\n";
        return 1;
    }

    int width = firstImage.Width();
    int height = firstImage.Height();
    std::cout << "Image size: " << width << " x " << height << "\n";

    // =========================================================================
    // Configure measurement parameters
    // =========================================================================

    MetrologyMeasureParams params;

    // Key parameters (similar to Halcon)
    params.measureLength1 = 30.0;       // Projection length (search range +/- 30 px)
    params.measureLength2 = 10.0;       // Projection width (for averaging)
    params.numMeasures = 36;            // Number of calipers around the circle
    params.measureSigma = 1.5;          // Gaussian smoothing sigma
    params.measureThreshold = 10.0;     // Edge amplitude threshold (lowered to detect more edges)
    params.measureTransition = EdgeTransition::All;  // Detect all edges
    params.numInstances = 1;            // Find 1 circle
    params.minScore = 0.5;              // Minimum score threshold

    // Threshold mode:
    // - params.SetThreshold(10.0);     // Manual: use specified value
    // - params.SetThreshold("auto");   // Auto: compute per profile region
    params.SetThreshold("auto");

    PrintParams(params);

    // =========================================================================
    // Fixed circle parameters (adjust based on your image)
    // =========================================================================

    double centerCol = 650.0;           // Center X
    double centerRow = 500.0;           // Center Y (user's original setting)
    double radius = 220.0;              // Radius

    std::cout << "\n=== Fixed Circle Parameters ===\n";
    std::cout << "  Center: (" << centerCol << ", " << centerRow << ")\n";
    std::cout << "  Radius: " << radius << " px\n";

    // =========================================================================
    // Create GUI Window
    // =========================================================================

    Window win("Circle Measurement - QiVision", 0, 0);
    win.SetAutoResize(true, 1400, 900);
    win.EnableZoomPan(true);

    // Mouse callback to display coordinates
    std::string currentTitle;
    win.SetMouseCallback([&](const MouseEvent& evt) {
        if (evt.type == MouseEventType::Move) {
            // Update window title with coordinates
            char coordStr[128];
            snprintf(coordStr, sizeof(coordStr), " | X:%.1f Y:%.1f", evt.imageX, evt.imageY);
            win.SetTitle(currentTitle + coordStr);
        }
    });

    std::cout << "\n=== Controls ===\n";
    std::cout << "  Left drag: Pan image\n";
    std::cout << "  Scroll wheel: Zoom in/out\n";
    std::cout << "  Right click: Reset view\n";
    std::cout << "  Mouse move: Show X,Y coordinates in title\n";
    std::cout << "  Any key: Next image\n";
    std::cout << "  'q': Quit\n\n";

    // =========================================================================
    // Process images
    // =========================================================================

    int successCount = 0;
    int totalCount = 0;

    // Process all images
    int maxImages = static_cast<int>(imagePaths.size());

    std::cout << "=== Processing " << maxImages << " images ===\n";

    for (int i = 0; i < maxImages; ++i) {
        const auto& path = imagePaths[i];
        std::string filename = fs::path(path).filename().string();

        std::cout << "\n--- Image " << (i+1) << "/" << maxImages << ": " << filename << " ---\n";

        // Load image and convert to grayscale
        QImage image = QImage::FromFile(path);
        if (!image.IsValid()) {
            std::cerr << "Failed to load: " << filename << "\n";
            continue;
        }
        if (image.Channels() > 1) {
            image = image.ToGray();
        }

        totalCount++;

        // =========================================================================
        // Perform measurement (uses Strongest mode with low threshold internally)
        // =========================================================================

        MetrologyModel model;
        int circleIdx = model.AddCircleMeasure(centerRow, centerCol, radius, params);

        QImage display = image;
        std::string title;

        if (model.Apply(image)) {
            auto edgePoints = model.GetMeasuredPoints(circleIdx);
            auto result = model.GetCircleResult(circleIdx);

            std::cout << "Detected " << edgePoints.size() << " edge points:\n";
            for (size_t j = 0; j < edgePoints.size() && j < 10; ++j) {
                double dist = std::sqrt(std::pow(edgePoints[j].x - centerCol, 2) +
                                       std::pow(edgePoints[j].y - centerRow, 2));
                std::cout << "  [" << j << "] (" << edgePoints[j].x << ", " << edgePoints[j].y
                          << ") dist=" << dist << " (expected " << radius << ")\n";
            }

            if (result.IsValid()) {
                successCount++;
                std::cout << ">>> Measurement Result <<<\n";
                std::cout << "  Center: (" << result.column << ", " << result.row << ")\n";
                std::cout << "  Radius: " << result.radius << " px\n";
                std::cout << "  Points used: " << result.numUsed << "/" << params.numMeasures << "\n";
                std::cout << "  RMS Error: " << result.rmsError << " px\n";
                std::cout << "  Score: " << result.score << "\n";

                title = filename + " - OK (" + std::to_string(edgePoints.size()) + " pts)";
            } else {
                std::cout << "Measurement failed - no valid circle found\n";
                title = filename + " - FAILED (" + std::to_string(edgePoints.size()) + " pts)";
            }

            // Always visualize: green = initial circle, cyan = calipers, yellow = edge points
            display = DrawCircleOverlay(image, centerCol, centerRow, radius,
                                       0, 255, 0, 1);  // Green = initial circle
            // Draw calipers (cyan)
            DrawCalipers(display, centerCol, centerRow, radius,
                        params.numMeasures, params.measureLength1, params.measureLength2,
                        0, 255, 255);
            // Draw edge points (yellow)
            // Get point weights and draw with colors based on weight
            auto pointWeights = model.GetPointWeights(circleIdx);
            DrawEdgePointsWithWeights(display, edgePoints, pointWeights, 4);
            // Draw measured circle (red) - only if valid and within image bounds
            if (result.IsValid() &&
                result.column > -500 && result.column < 2000 &&
                result.row > -500 && result.row < 2000 &&
                result.radius < 1000) {
                display = DrawCircleOverlay(display, result.column, result.row, result.radius,
                                           255, 0, 0, 2);
            }
        } else {
            std::cout << "Apply failed\n";
            title = filename + " - Apply failed";
        }

        // Display result
        currentTitle = title;
        win.SetTitle(title);
        win.DispImage(display);

        // Wait for key
        int key = win.WaitKey(0);
        if (key == 'q' || key == 'Q') {
            std::cout << "Quit requested.\n";
            break;
        }
    }

    // =========================================================================
    // Summary
    // =========================================================================

    std::cout << "\n=== Summary ===\n";
    std::cout << "  Success: " << successCount << " / " << totalCount << "\n";

    std::cout << "\n=== Parameter Tuning Tips ===\n";
    std::cout << "  - If no edges found: decrease measureThreshold\n";
    std::cout << "  - If noisy: increase measureSigma or measureLength2\n";
    std::cout << "  - If missing parts: increase measureLength1 (search range)\n";
    std::cout << "  - For better accuracy: increase numMeasures\n";
    std::cout << "  - Adjust centerRow, centerCol, radius to match your object\n";

    return 0;
}
