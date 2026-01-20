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
#include <QiVision/Core/Draw.h>
#include <QiVision/Matching/ShapeModel.h>
#include <QiVision/Matching/MatchTypes.h>
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
using namespace Qi::Vision::Platform;
using namespace Qi::Vision::GUI;
namespace fs = std::filesystem;

// Display interval in milliseconds
constexpr int32_t DISPLAY_INTERVAL_MS = 1000;

// Convert to grayscale helper
QImage ToGrayscale(const QImage& img) {
    if (img.Channels() == 1) return img;

    QImage gray(img.Width(), img.Height(), PixelType::UInt8, ChannelType::Gray);
    const uint8_t* src = static_cast<const uint8_t*>(img.Data());
    uint8_t* dst = static_cast<uint8_t*>(gray.Data());
    size_t srcStride = img.Stride();
    size_t dstStride = gray.Stride();

    for (int32_t y = 0; y < img.Height(); ++y) {
        const uint8_t* srcRow = src + y * srcStride;
        uint8_t* dstRow = dst + y * dstStride;
        for (int32_t x = 0; x < img.Width(); ++x) {
            uint8_t r = srcRow[x * 3 + 0];
            uint8_t g = srcRow[x * 3 + 1];
            uint8_t b = srcRow[x * 3 + 2];
            dstRow[x] = static_cast<uint8_t>(0.299 * r + 0.587 * g + 0.114 * b);
        }
    }
    return gray;
}

// Convert grayscale to RGB for drawing
QImage ToRGB(const QImage& gray) {
    QImage colorImg(gray.Width(), gray.Height(), PixelType::UInt8, ChannelType::RGB);
    const uint8_t* src = static_cast<const uint8_t*>(gray.Data());
    uint8_t* dst = static_cast<uint8_t*>(colorImg.Data());
    for (int32_t y = 0; y < gray.Height(); ++y) {
        for (int32_t x = 0; x < gray.Width(); ++x) {
            uint8_t v = src[y * gray.Stride() + x];
            size_t dstIdx = y * colorImg.Stride() + x * 3;
            dst[dstIdx + 0] = v;
            dst[dstIdx + 1] = v;
            dst[dstIdx + 2] = v;
        }
    }
    return colorImg;
}

// Get all image files in a directory (excluding result_* files)
std::vector<std::string> GetImageFiles(const std::string& dir) {
    std::vector<std::string> files;

    for (const auto& entry : fs::directory_iterator(dir)) {
        if (!entry.is_regular_file()) continue;

        std::string filename = entry.path().filename().string();

        // Skip result files
        if (filename.rfind("result_", 0) == 0) continue;

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
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    double halfW = modelWidth / 2.0;
    double halfH = modelHeight / 2.0;

    double corners[4][2] = {
        {-halfW, -halfH}, {halfW, -halfH},
        {halfW, halfH}, {-halfW, halfH}
    };
    int32_t px[4], py[4];
    for (int k = 0; k < 4; ++k) {
        px[k] = static_cast<int32_t>(col + cosA * corners[k][0] - sinA * corners[k][1]);
        py[k] = static_cast<int32_t>(row + sinA * corners[k][0] + cosA * corners[k][1]);
    }
    Draw::Line(colorImg, px[0], py[0], px[1], py[1], Color::Cyan(), 2);
    Draw::Line(colorImg, px[1], py[1], px[2], py[2], Color::Cyan(), 2);
    Draw::Line(colorImg, px[2], py[2], px[3], py[3], Color::Cyan(), 2);
    Draw::Line(colorImg, px[3], py[3], px[0], py[0], Color::Cyan(), 2);

    Draw::Cross(colorImg, Point2d{col, row}, 15, angle, Color::Yellow(), 2);
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
    QImage templateImg = QImage::FromFile(dataDir + imageFiles[0]);
    if (!templateImg.IsValid()) {
        std::cerr << "Failed to load template!" << std::endl;
        return 1;
    }
    QImage templateGray = ToGrayscale(templateImg);
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

    ShapeModel model = CreateShapeModel(
        templateGray, roi,
        4,                      // numLevels
        0, RAD(360), 0,         // angleStart, angleExtent, angleStep (auto)
        "auto",                 // optimization
        "use_polarity",         // metric
        "auto", 10              // contrast, minContrast
    );

    if (!model.IsValid()) {
        std::cerr << "Failed to create model!" << std::endl;
        return 1;
    }
    std::cout << "   Model created in " << timer.ElapsedMs() << " ms" << std::endl;

    // Get model info
    QContourArray contours = GetShapeModelXLD(model, 1);
    size_t totalPoints = 0;
    for (size_t c = 0; c < contours.Size(); ++c) {
        totalPoints += contours[c].Size();
    }
    std::cout << "   Model points: " << totalPoints << ", contours: " << contours.Size() << std::endl;

    // Display model
    QImage roiVis = ToRGB(templateGray);
    Draw::Rectangle(roiVis, roi.x, roi.y, roi.width, roi.height, Color::Green(), 2);
    win.SetTitle("Template with ROI");
    win.DispImage(roiVis);
    win.WaitKey(1000);

    // Search in all images
    std::cout << "\n4. Searching in " << imageFiles.size() << " images..." << std::endl;

    int successCount = 0;
    double totalTime = 0;

    for (size_t i = 0; i < imageFiles.size(); ++i) {
        QImage searchImg = QImage::FromFile(dataDir + imageFiles[i]);
        if (!searchImg.IsValid()) {
            std::cerr << "   [" << (i+1) << "] Failed to load: " << imageFiles[i] << std::endl;
            continue;
        }
        QImage searchGray = ToGrayscale(searchImg);

        std::vector<double> rows, cols, angles, scores;

        timer.Reset();
        timer.Start();
        FindShapeModel(searchGray, model,
                       0, RAD(360),           // search angle range
                       0.5, 10, 0.5,          // minScore, numMatches, maxOverlap
                       "least_squares", 0, 0.9,  // subPixel, numLevels, greediness
                       rows, cols, angles, scores);
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
                      << " Angle=" << std::setprecision(1) << DEG(angles[0]) << "deg";
        } else {
            std::cout << " - NO MATCH";
        }
        std::cout << " [" << std::setprecision(1) << searchTime << "ms]" << std::endl;

        // Display result
        QImage colorImg = ToRGB(searchGray);

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
                                    Color::Green(), Color::Red(), 2, 0.5);

        // Draw bounding boxes and center crosses
        for (size_t j = 0; j < rows.size(); ++j) {
            DrawMatchBoundingBox(colorImg, rows[j], cols[j], angles[j], roi.width, roi.height);
        }

        std::string title = imageFiles[i];
        if (success) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2)
                << " - Score=" << scores[0]
                << " Angle=" << std::setprecision(1) << DEG(angles[0]) << "deg";
            title += oss.str();
        } else {
            title += " - NO MATCH";
        }

        win.SetTitle(title);
        win.DispImage(colorImg);

        // Save result
        std::string outPath = dataDir + "result_" + std::to_string(i+1) + ".bmp";
        colorImg.SaveToFile(outPath);

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
