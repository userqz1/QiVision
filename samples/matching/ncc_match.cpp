/**
 * @file ncc_match.cpp
 * @brief NCCModel template matching demo
 *
 * Demonstrates:
 * - Creating NCC model from template (ROI from first image)
 * - Finding matches in all images in a folder
 * - Drawing results
 */

#include <QiVision/Matching/NCCModel.h>
#include <QiVision/IO/ImageIO.h>
#include <QiVision/Display/Draw.h>
#include <QiVision/GUI/Window.h>
#include <QiVision/Color/ColorConvert.h>
#include <QiVision/Core/Constants.h>

#include <iostream>
#include <chrono>
#include <filesystem>
#include <vector>
#include <algorithm>

namespace fs = std::filesystem;

using namespace Qi::Vision;
using namespace Qi::Vision::Matching;
using namespace Qi::Vision::IO;
using namespace Qi::Vision::GUI;
using namespace Qi::Vision::Color;

int main() {
    std::cout << "=== NCC Model Batch Demo ===\n";

    // 1. Get all images in the folder
    std::string imageFolder = "tests/data/matching/image2";
    std::vector<std::string> imagePaths;

    for (const auto& entry : fs::directory_iterator(imageFolder)) {
        if (entry.is_regular_file()) {
            std::string ext = entry.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            if (ext == ".bmp" || ext == ".png" || ext == ".jpg" || ext == ".jpeg") {
                imagePaths.push_back(entry.path().string());
            }
        }
    }

    std::sort(imagePaths.begin(), imagePaths.end());

    if (imagePaths.empty()) {
        std::cerr << "No images found in " << imageFolder << "\n";
        return 1;
    }

    std::cout << "Found " << imagePaths.size() << " images\n";

    // 2. Load first image for ROI selection
    QImage firstImage;
    try {
        ReadImage(imagePaths[0], firstImage);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load first image: " << e.what() << "\n";
        return 1;
    }
    std::cout << "First image: " << imagePaths[0] << "\n";
    std::cout << "Size: " << firstImage.Width() << " x " << firstImage.Height() << "\n";

    // Convert to grayscale if needed
    QImage firstGray;
    if (firstImage.Channels() > 1) {
        Rgb1ToGray(firstImage, firstGray);
    } else {
        firstGray = firstImage;
    }

    // 3. Interactive ROI selection on first image
    std::cout << "\nDraw template ROI on the first image.\n";
    std::cout << "  - Click and drag to draw rectangle\n";
    std::cout << "  - Release to confirm\n";
    std::cout << "  - Press ESC to cancel\n";

    Window roiWin("Draw Template ROI - First Image");
    roiWin.SetAutoResize(true, 1280, 960);
    roiWin.DispImage(firstImage, ScaleMode::Fit);

    ROIResult roi = roiWin.DrawRectangle();
    if (!roi.valid) {
        std::cerr << "ROI selection cancelled.\n";
        return 1;
    }

    // Convert ROI to integer coordinates
    int32_t tx = static_cast<int32_t>(roi.col1);
    int32_t ty = static_cast<int32_t>(roi.row1);
    int32_t tw = static_cast<int32_t>(roi.col2 - roi.col1);
    int32_t th = static_cast<int32_t>(roi.row2 - roi.row1);

    // Validate ROI
    if (tw <= 0 || th <= 0 || tx + tw > firstGray.Width() || ty + th > firstGray.Height()) {
        std::cerr << "Invalid ROI!\n";
        return 1;
    }

    Rect2i templateRoi(tx, ty, tw, th);
    QImage templateImg = firstGray.SubImage(tx, ty, tw, th);

    std::cout << "Template: " << templateImg.Width() << " x " << templateImg.Height() << "\n";
    std::cout << "Template ROI: (" << tx << ", " << ty << ") " << tw << "x" << th << "\n";

    // 4. Create NCC model
    std::cout << "\nCreating NCC model...\n";

    NCCModel model;
    auto startCreate = std::chrono::high_resolution_clock::now();

    CreateNCCModel(
        templateImg,
        model,
        4,                      // numLevels (0 = auto)
        0.0,                    // angleStart
        DegToRad(360.0),        // angleExtent (full rotation)
        0.0,                    // angleStep (0 = auto)
        "use_polarity"          // metric
    );

    auto endCreate = std::chrono::high_resolution_clock::now();
    double createTime = std::chrono::duration<double, std::milli>(endCreate - startCreate).count();

    if (!model.IsValid()) {
        std::cerr << "Failed to create NCC model\n";
        return 1;
    }

    std::cout << "Model created in " << createTime << " ms\n";

    // Get model params
    int32_t numLevels;
    double angleStart, angleExtent, angleStep;
    std::string metric;
    GetNCCModelParams(model, numLevels, angleStart, angleExtent, angleStep, metric);
    double modelOriginRow = 0.0;
    double modelOriginCol = 0.0;
    GetNCCModelOrigin(model, modelOriginRow, modelOriginCol);
    std::cout << "Model params: levels=" << numLevels
              << ", angles=[" << RadToDeg(angleStart) << ", " << RadToDeg(angleStart + angleExtent) << "] deg"
              << ", step=" << RadToDeg(angleStep) << " deg"
              << ", metric=" << metric << "\n";
    std::cout << "Model origin: (" << modelOriginCol << ", " << modelOriginRow << ")\n";

    // 5. Match all images
    std::cout << "\n=== Matching all " << imagePaths.size() << " images ===\n\n";

    Window win("NCC Match Result");
    win.SetAutoResize(true, 1280, 960);

    double totalSearchTime = 0;
    int totalMatches = 0;

    for (size_t imgIdx = 0; imgIdx < imagePaths.size(); ++imgIdx) {
        const std::string& imagePath = imagePaths[imgIdx];
        std::string filename = fs::path(imagePath).filename().string();

        // Load image
        QImage searchImage;
        try {
            ReadImage(imagePath, searchImage);
        } catch (const std::exception& e) {
            std::cerr << "Failed to load " << filename << ": " << e.what() << "\n";
            continue;
        }

        // Convert to grayscale
        QImage gray;
        if (searchImage.Channels() > 1) {
            Rgb1ToGray(searchImage, gray);
        } else {
            gray = searchImage;
        }

        // Find matches
        std::vector<double> rows, cols, angles, scores;
        auto startSearch = std::chrono::high_resolution_clock::now();

        FindNCCModel(
            gray,
            model,
            0.0,                    // angleStart
            DegToRad(360.0),        // angleExtent (full rotation)
            0.9,                    // minScore
            10,                     // numMatches (0 = all)
            0.5,                    // maxOverlap
            "interpolation",        // subPixel
            0,                      // numLevels (0 = use model levels)
            rows, cols, angles, scores
        );

        auto endSearch = std::chrono::high_resolution_clock::now();
        double searchTime = std::chrono::duration<double, std::milli>(endSearch - startSearch).count();
        totalSearchTime += searchTime;
        totalMatches += static_cast<int>(rows.size());

        // Print results
        std::cout << "[" << (imgIdx + 1) << "/" << imagePaths.size() << "] "
                  << filename << ": "
                  << rows.size() << " match(es), "
                  << searchTime << " ms\n";

        if (rows.empty()) {
            std::vector<double> debugRows, debugCols, debugAngles, debugScores;
            FindNCCModel(
                gray,
                model,
                0.0,
                DegToRad(360.0),
                0.0,                // debug: no threshold
                1,                  // best match only
                0.5,
                "interpolation",
                0,
                debugRows, debugCols, debugAngles, debugScores
            );

            if (!debugRows.empty()) {
                std::cout << "    Best (below threshold): "
                          << "pos=(" << debugCols[0] << ", " << debugRows[0] << ") "
                          << "angle=" << RadToDeg(debugAngles[0]) << " deg "
                          << "score=" << (debugScores[0] * 100) << "%\n";
            }
        }

        for (size_t i = 0; i < rows.size(); ++i) {
            std::cout << "    Match " << (i + 1) << ": "
                      << "pos=(" << cols[i] << ", " << rows[i] << ") "
                      << "angle=" << RadToDeg(angles[i]) << " deg "
                      << "score=" << (scores[i] * 100) << "%\n";
        }

        // Visualize results
        QImage display;
        if (searchImage.Channels() == 1) {
            GrayToRgb(searchImage, display);
        } else {
            display = searchImage.Clone();
        }

        // Draw template ROI on first image only (cyan)
        if (imgIdx == 0) {
            Draw::Rectangle(display, templateRoi, Scalar(0, 255, 255), 2);
        }

        // Draw matches (green cross + box)
        for (size_t i = 0; i < rows.size(); ++i) {
            int32_t cx = static_cast<int32_t>(cols[i]);
            int32_t cy = static_cast<int32_t>(rows[i]);

            // Draw cross at match center
            Draw::Cross(display, Point2d(cols[i], rows[i]), 20, angles[i], Scalar(0, 255, 0), 2);

            // Draw rotated bounding box (account for model origin)
            double dx = (tw * 0.5) - modelOriginCol;
            double dy = (th * 0.5) - modelOriginRow;
            double cosA = std::cos(angles[i]);
            double sinA = std::sin(angles[i]);
            Point2d rectCenter{
                cols[i] + cosA * dx - sinA * dy,
                rows[i] + sinA * dx + cosA * dy
            };
            Draw::RotatedRectangle(display, rectCenter, tw, th, angles[i], Scalar(0, 255, 0), 1);
        }

        // Show result
        std::string title = "NCC Match [" + std::to_string(imgIdx + 1) + "/" +
                           std::to_string(imagePaths.size()) + "] " + filename +
                           " - " + std::to_string(rows.size()) + " matches";
        win.SetTitle(title);
        win.DispImage(display, ScaleMode::Fit);

        // Wait for key (space/enter for next, ESC to quit)
        std::cout << "    Press SPACE/ENTER for next, ESC to quit...\n";
        int key = win.WaitKey(0);
        if (key == 27) {  // ESC
            std::cout << "\nStopped by user.\n";
            break;
        }
    }

    // Summary
    std::cout << "\n=== Summary ===\n";
    std::cout << "Images processed: " << imagePaths.size() << "\n";
    std::cout << "Total matches: " << totalMatches << "\n";
    std::cout << "Total search time: " << totalSearchTime << " ms\n";
    std::cout << "Average search time: " << (totalSearchTime / imagePaths.size()) << " ms/image\n";

    return 0;
}
