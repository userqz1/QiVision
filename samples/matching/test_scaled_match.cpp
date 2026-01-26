/**
 * @file test_scaled_match.cpp
 * @brief Scaled shape matching test - Interactive ROI selection
 *
 * Usage: ./matching_test_scaled_match [image_folder] [scale_min] [scale_max]
 *
 * 1. Load first image as template
 * 2. User draws ROI with mouse
 * 3. Test all images with scaled matching
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Display/Draw.h>
#include <QiVision/Color/ColorConvert.h>
#include <QiVision/Matching/ShapeModel.h>
#include <QiVision/Matching/MatchTypes.h>
#include <QiVision/Segment/Segment.h>
#include <QiVision/Filter/Filter.h>
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
                          double modelWidth, double modelHeight, double scale) {
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    double halfW = modelWidth * scale / 2.0;
    double halfH = modelHeight * scale / 2.0;

    double corners[4][2] = {
        {-halfW, -halfH}, {halfW, -halfH},
        {halfW, halfH}, {-halfW, halfH}
    };
    int32_t px[4], py[4];
    for (int k = 0; k < 4; ++k) {
        px[k] = static_cast<int32_t>(col + cosA * corners[k][0] - sinA * corners[k][1]);
        py[k] = static_cast<int32_t>(row + sinA * corners[k][0] + cosA * corners[k][1]);
    }
    Draw::Line(colorImg, px[0], py[0], px[1], py[1], Scalar::Cyan(), 2);
    Draw::Line(colorImg, px[1], py[1], px[2], py[2], Scalar::Cyan(), 2);
    Draw::Line(colorImg, px[2], py[2], px[3], py[3], Scalar::Cyan(), 2);
    Draw::Line(colorImg, px[3], py[3], px[0], py[0], Scalar::Cyan(), 2);

    Draw::Cross(colorImg, Point2d{col, row}, 15, angle, Scalar::Yellow(), 2);
}

int main(int argc, char* argv[]) {
    // =========================================================================
    // Configuration
    // =========================================================================
    std::string dataDir = "tests/data/halcon_images/rings/";
    double scaleMin = 0.5;
    double scaleMax = 1.5;

    if (argc > 1) {
        dataDir = argv[1];
        if (dataDir.back() != '/') dataDir += '/';
    }
    if (argc > 2) {
        scaleMin = std::atof(argv[2]);
    }
    if (argc > 3) {
        scaleMax = std::atof(argv[3]);
    }

    std::cout << "=== QiVision Scaled Shape Matching Test ===" << std::endl;
    std::cout << "Folder: " << dataDir << std::endl;
    std::cout << "Scale range: [" << scaleMin << ", " << scaleMax << "]" << std::endl;
    std::cout << "Controls: Press 'q' or ESC to quit" << std::endl;
    std::string outputDir = "tests/output/shape_debug/";
    fs::create_directories(outputDir);

    // Get all image files
    std::vector<std::string> imageFiles = GetImageFiles(dataDir);
    if (imageFiles.empty()) {
        std::cerr << "No image files found in: " << dataDir << std::endl;
        return 1;
    }
    std::cout << "Found " << imageFiles.size() << " images" << std::endl;

    // Create window
    Window win("Scaled Shape Match Test");
    win.SetAutoResize(true);

    Timer timer;

    // Load first image as template
    std::cout << "\n1. Loading template: " << imageFiles[0] << std::endl;
    QImage templateImg = QImage::FromFile(dataDir + imageFiles[0]);
    if (!templateImg.IsValid()) {
        std::cerr << "Failed to load template!" << std::endl;
        return 1;
    }
    QImage templateGray = templateImg.ToGray();
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

    // Create scaled shape model (smoothed template + auto params -> manual hysteresis + min size)
    std::cout << "\n3. Creating shape model..." << std::endl;
    timer.Start();

    ShapeModel model;
    SetShapeModelDebugCreateGlobal(true);
    QRegion roiRegion = QRegion::Rectangle(roi.x, roi.y, roi.width, roi.height);

    QImage templateSmooth;
    Filter::GaussFilter(templateGray, templateSmooth, 0.7);

    std::string contrastParam = "auto_contrast_hyst";

    CreateScaledShapeModel(
        templateSmooth, roiRegion, model,
        4,                      // numLevels
        0, RAD(360), RAD(5),    // angleStart, angleExtent, angleStep
        scaleMin, scaleMax, 0.05,  // scale range, scaleStep
        "point_reduction_high", // optimization
        "use_polarity",         // metric
        contrastParam, 10.0  // contrast, minContrast
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

    // Dump model contrast image for inspection
    QImage contrastImage;
    int32_t numPoints = 0;
    InspectShapeModel(model, 0, contrastImage, numPoints);
    WriteImage(contrastImage, outputDir + "model_contrast_level0.png");

    // Search in all images
    std::cout << "\n4. Searching in " << imageFiles.size() << " images with scale ["
              << scaleMin << ", " << scaleMax << "]..." << std::endl;

    int successCount = 0;
    double totalTime = 0;

    for (size_t i = 0; i < imageFiles.size(); ++i) {
        QImage searchImg = QImage::FromFile(dataDir + imageFiles[i]);
        if (!searchImg.IsValid()) {
            std::cerr << "   [" << (i+1) << "] Failed to load: " << imageFiles[i] << std::endl;
            continue;
        }
        QImage searchGray = searchImg.ToGray();

        std::vector<double> rows, cols, angles, scales, scores;

        timer.Reset();
        timer.Start();
        FindScaledShapeModel(searchGray, model,
                             0, RAD(360),           // search angle range
                             scaleMin, scaleMax,    // scale range
                             0.9, 1, 0.5,           // minScore, numMatches, maxOverlap
                             "least_squares", 0, 0.8,  // subPixel, numLevels, greediness
                             rows, cols, angles, scales, scores);
        double searchTime = timer.ElapsedMs();
        totalTime += searchTime;

        bool success = !rows.empty();
        if (success) successCount++;

        std::cout << "   [" << (i+1) << "/" << imageFiles.size() << "] "
                  << imageFiles[i];
        if (success) {
            std::cout << " - Match: (" << std::fixed << std::setprecision(1)
                      << cols[0] << "," << rows[0] << ") "
                      << "Score=" << std::setprecision(3) << scores[0]
                      << " Angle=" << std::setprecision(1) << DEG(angles[0]) << "deg"
                      << " Scale=" << std::setprecision(3) << scales[0];
        } else {
            std::cout << " - NO MATCH";
        }
        std::cout << " [" << std::setprecision(0) << searchTime << "ms]" << std::endl;

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

        // Draw model contours at match positions
        Draw::ShapeMatchingResults(colorImg, model, matches,
                                    Scalar::Green(), Scalar::Red(), 2, 0.5);

        // Draw bounding boxes and center crosses (with scale)
        for (size_t j = 0; j < rows.size(); ++j) {
            DrawMatchBoundingBox(colorImg, rows[j], cols[j], angles[j],
                                 roi.width, roi.height, scales[j]);
        }

        std::string title = imageFiles[i];
        if (success) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2)
                << " - Score=" << scores[0]
                << " Angle=" << std::setprecision(1) << DEG(angles[0]) << "deg"
                << " Scale=" << std::setprecision(2) << scales[0];
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
