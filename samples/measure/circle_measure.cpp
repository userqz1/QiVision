/**
 * @file circle_measure.cpp
 * @brief Circle measurement sample using Metrology module
 *
 * Demonstrates:
 * - GUI display with zoom/pan
 * - Creating a MetrologyModel with fixed circle parameters
 * - Configuring measurement parameters (measure_length, num_measures, etc.)
 * - Executing measurement and retrieving results
 * - Visualizing results using Draw module
 *
 * Usage:
 * 1. Run the program: ./measure_circle [image_dir]
 *    - [image_dir]: Image directory (default: tests/data/matching/image3)
 * 2. Left drag to pan, scroll wheel to zoom, right click to reset view
 * 3. Press any key for next image, 'q' to quit
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/Draw.h>
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

// Draw edge points with colors based on weights (green=inlier, yellow=moderate, red=outlier)
void DrawEdgePointsWithWeights(QImage& image, const std::vector<Point2d>& points,
                                const std::vector<double>& weights, int markerSize = 4)
{
    for (size_t i = 0; i < points.size(); ++i) {
        double w = (i < weights.size()) ? weights[i] : 1.0;

        // Color based on weight
        Color color;
        if (w >= 0.8) {
            color = Color::Green();   // Inlier
        } else if (w >= 0.5) {
            color = Color::Yellow();  // Moderate
        } else {
            color = Color::Red();     // Outlier
        }

        Draw::Cross(image, points[i], markerSize, color, 1);
        Draw::FilledCircle(image, points[i], 2, color);
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
        // Perform measurement
        // =========================================================================

        MetrologyModel model;
        int circleIdx = model.AddCircleMeasure(centerRow, centerCol, radius, params);

        std::string title;

        // Prepare display image (convert to RGB for colored drawing)
        QImage display = Draw::PrepareForDrawing(image);

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

            // =========================================================================
            // Visualization using Draw module
            // =========================================================================

            // 1. Draw initial/reference circle (green, thin)
            Draw::Circle(display, Point2d{centerCol, centerRow}, radius, Color::Green(), 1);
            Draw::Cross(display, Point2d{centerCol, centerRow}, 8, Color::Green(), 1);

            // 2. Draw calipers (cyan) - using new MeasureRects function
            const MetrologyObject* obj = model.GetObject(circleIdx);
            if (obj) {
                auto calipers = obj->GetCalipers();
                Draw::MeasureRects(display, calipers, Color::Cyan(), 1);
            }

            // 3. Draw edge points with weight-based coloring
            auto pointWeights = model.GetPointWeights(circleIdx);
            DrawEdgePointsWithWeights(display, edgePoints, pointWeights, 4);

            // 4. Draw fitted circle result (red, thick) - only if valid
            if (result.IsValid() &&
                result.column > -500 && result.column < 2000 &&
                result.row > -500 && result.row < 2000 &&
                result.radius < 1000) {
                Draw::Circle(display, Point2d{result.column, result.row}, result.radius,
                            Color::Red(), 2);
                Draw::Cross(display, Point2d{result.column, result.row}, 10, Color::Red(), 2);
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
