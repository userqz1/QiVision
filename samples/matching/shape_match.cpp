/**
 * @file shape_match.cpp
 * @brief Shape matching test - Interactive ROI selection
 *
 * Usage: ./matching_shape_match [image_folder]
 *
 * 1. Load first image as template
 * 2. User draws ROI with mouse
 * 3. Test all images in the folder
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Display/Draw.h>
#include <QiVision/Color/ColorConvert.h>
#include <QiVision/Matching/ShapeModel.h>
#include <QiVision/Matching/MatchTypes.h>
#include <QiVision/IO/ImageIO.h>
#include <QiVision/Platform/Timer.h>
#include <QiVision/Platform/FileIO.h>
#include <QiVision/GUI/Window.h>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <filesystem>

using namespace Qi::Vision;
using namespace Qi::Vision::Matching;
using namespace Qi::Vision::IO;
using namespace Qi::Vision::Platform;
using namespace Qi::Vision::GUI;
namespace fs = std::filesystem;

// Display interval in milliseconds
constexpr int32_t DISPLAY_INTERVAL_MS = 1000;

// Get all image files in a directory
std::vector<std::string> GetImageFiles(const std::string& dir) {
    std::vector<std::string> files;

    for (const auto& entry : fs::directory_iterator(dir)) {
        if (!entry.is_regular_file()) continue;

        std::string filename = entry.path().filename().string();
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" ||
            ext == ".bmp" || ext == ".tif" || ext == ".tiff") {
            files.push_back(filename);
        }
    }

    std::sort(files.begin(), files.end());
    return files;
}

// Draw bounding box for a match result
void DrawMatchBoundingBox(QImage& colorImg, double row, double col, double angle,
                          double modelWidth, double modelHeight) {
    Point2d center{col, row};
    Draw::RotatedRectangle(colorImg, center, modelWidth, modelHeight, angle,
                           Scalar::Cyan(), 2);

    Draw::Cross(colorImg, Point2d{col, row}, 15, angle, Scalar::Yellow(), 2);
}

int main(int argc, char* argv[]) {
    // =========================================================================
    // Configuration - MODIFY THIS PATH TO TEST DIFFERENT FOLDERS
    // =========================================================================
    std::string dataDir = "tests/data/matching/image1/";

    if (argc > 1) {
        dataDir = argv[1];
        if (dataDir.back() != '/') dataDir += '/';
    }

    std::cout << "=== QiVision Shape Matching Test ===" << std::endl;
    std::cout << "Folder: " << dataDir << std::endl;
    std::cout << "Controls: Press 'q' or ESC to quit" << std::endl;
    std::string outputDir = "tests/output/shape_match_debug/";
    fs::create_directories(outputDir);

    // Get all image files
    std::vector<std::string> imageFiles = GetImageFiles(dataDir);
    if (imageFiles.empty()) {
        std::cerr << "No image files found in: " << dataDir << std::endl;
        return 1;
    }
    std::cout << "Found " << imageFiles.size() << " images" << std::endl;

    // Create window
    Window win("Shape Match Test");
    win.SetAutoResize(true);

    Timer timer;

    // Load first image as template
    std::cout << "\n1. Loading template: " << imageFiles[0] << std::endl;
    QImage templateGray;
    ReadImageGray(dataDir + imageFiles[0], templateGray);
    if (templateGray.Empty()) {
        std::cerr << "Failed to load template!" << std::endl;
        return 1;
    }
    std::cout << "   Size: " << templateGray.Width() << " x " << templateGray.Height() << std::endl;

    // Interactive ROI selection
    std::cout << "\n2. Draw ROI on the template (drag mouse, ESC to cancel)..." << std::endl;
    win.SetTitle("Draw ROI - Drag mouse to select region");
    win.DispImage(templateGray);

    ROIResult roiResult = win.DrawRectangle();
    if (!roiResult.valid) {
        std::cerr << "ROI selection cancelled!" << std::endl;
        return 1;
    }

    Rect2i roi;
    roi.x = static_cast<int32_t>(std::min(roiResult.col1, roiResult.col2));
    roi.y = static_cast<int32_t>(std::min(roiResult.row1, roiResult.row2));
    roi.width = static_cast<int32_t>(std::abs(roiResult.col2 - roiResult.col1));
    roi.height = static_cast<int32_t>(std::abs(roiResult.row2 - roiResult.row1));

    std::cout << "   Selected ROI: (" << roi.x << ", " << roi.y
              << ", " << roi.width << ", " << roi.height << ")" << std::endl;

    // Create shape model
    std::cout << "\n3. Creating shape model..." << std::endl;
    timer.Start();

    ShapeModel model;
    CreateShapeModel(
        templateGray, roi, model,
        4,                      // numLevels
        0, RAD(360), 0,         // angleStart, angleExtent, angleStep (auto)
        "auto",                 // optimization
        "use_polarity",         // metric
        "auto", 5              // contrast, minContrast
    );

    if (!model.IsValid()) {
        std::cerr << "Failed to create model!" << std::endl;
        return 1;
    }
    std::cout << "   Model created in " << timer.ElapsedMs() << " ms" << std::endl;

    // Get model info
    QContourArray contours;
    GetShapeModelXLD(model, 1, contours);
    size_t totalPoints = 0;
    for (size_t c = 0; c < contours.Size(); ++c) {
        totalPoints += contours[c].Size();
    }
    std::cout << "   Model points: " << totalPoints << ", contours: " << contours.Size() << std::endl;

    // Display model
    QImage roiVis;
    Color::GrayToRgb(templateGray, roiVis);
    Draw::Rectangle(roiVis, roi.x, roi.y, roi.width, roi.height, Scalar::Green(), 2);
    win.SetTitle("Template with ROI");
    win.DispImage(roiVis);
    win.WaitKey(1000);
    WriteImage(roiVis, outputDir + "template_roi.png");

    QImage contrastImage;
    int32_t numPoints = 0;
    InspectShapeModel(model, 0, contrastImage, numPoints);
    WriteImage(contrastImage, outputDir + "model_contrast_level0.png");

    // Search in all images
    std::cout << "\n4. Searching in " << imageFiles.size() << " images..." << std::endl;

    int successCount = 0;
    double totalTime = 0;

    for (size_t i = 0; i < imageFiles.size(); ++i) {
        QImage searchGray;
        ReadImageGray(dataDir + imageFiles[i], searchGray);
        if (searchGray.Empty()) {
            std::cerr << "   [" << (i+1) << "] Failed to load: " << imageFiles[i] << std::endl;
            continue;
        }

        std::vector<double> rows, cols, angles, scores;

        timer.Reset();
        timer.Start();
        FindShapeModel(searchGray, model,
                       0, RAD(360),           // search angle range
                       0.9, 4, 1.0,           // minScore, numMatches, maxOverlap
                       "least_squares", 0, 0.9,  // subPixel, numLevels, greediness
                       rows, cols, angles, scores);
        double searchTime = timer.ElapsedMs();
        totalTime += searchTime;

        bool success = !rows.empty();
        if (success) successCount++;

        std::cout << "   [" << (i+1) << "/" << imageFiles.size() << "] "
                  << imageFiles[i];
        if (success) {
            std::cout << " - Match: (" << std::fixed << std::setprecision(3)
                      << cols[0] << "," << rows[0] << ") "
                      << "Score=" << std::setprecision(4) << scores[0]
                      << " Angle=" << std::setprecision(3) << DEG(angles[0]) << "deg";
        } else {
            std::cout << " - NO MATCH";
        }
        std::cout << " [" << std::setprecision(1) << searchTime << "ms]" << std::endl;

        // Display result
        QImage colorImg;
        Color::GrayToRgb(searchGray, colorImg);

        std::vector<MatchResult> matches;
        for (size_t j = 0; j < rows.size(); ++j) {
            MatchResult m;
            m.x = cols[j];
            m.y = rows[j];
            m.angle = angles[j];
            m.score = scores[j];
            matches.push_back(m);
        }

        // Draw model contours at match positions (Halcon-style)
        Draw::ShapeMatchingResults(colorImg, model, matches,
                                    Scalar::Green(), Scalar::Red(), 2, 0.5);

        // Draw bounding boxes and center crosses
        for (size_t j = 0; j < rows.size(); ++j) {
            DrawMatchBoundingBox(colorImg, rows[j], cols[j], angles[j], roi.width, roi.height);
        }

        std::string title = imageFiles[i];
        if (success) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2)
                << " - Score=" << scores[0]
                << " Angle=" << std::setprecision(3) << DEG(angles[0]) << "deg";
            title += oss.str();
        } else {
            title += " - NO MATCH";
        }

        win.SetTitle(title);
        win.DispImage(colorImg);

        std::ostringstream outName;
        outName << outputDir << "result_" << std::setw(2) << std::setfill('0') << (i + 1)
                << "_" << imageFiles[i];
        WriteImage(colorImg, outName.str());

        int32_t key = win.WaitKey(DISPLAY_INTERVAL_MS);
        if (key == 'q' || key == 'Q' || key == 27) {
            std::cout << "\n   User quit." << std::endl;
            break;
        }
    }

    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "Matched: " << successCount << "/" << imageFiles.size() << std::endl;
    std::cout << "Avg time: " << std::fixed << std::setprecision(1)
              << (totalTime / imageFiles.size()) << " ms" << std::endl;

    std::cout << "\nPress any key to close..." << std::endl;
    win.WaitKey(0);

    return 0;
}
