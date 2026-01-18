/**
 * @file interactive_match.cpp
 * @brief Interactive shape matching - user draws ROI to select template
 *
 * Usage:
 * 1. Load first image and display
 * 2. User draws rectangle ROI to select template region
 * 3. Train shape model from selected ROI
 * 4. Search in all images and display results
 *
 * Controls:
 * - Left mouse drag: Draw ROI rectangle
 * - Mouse wheel: Zoom in/out
 * - Left drag (when zoom enabled): Pan
 * - Right click: Reset zoom to 1:1
 * - 'F' key: Fit to window
 * - ESC: Cancel ROI drawing / Quit
 * - 'Q': Quit
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
#include <cstdlib>

using namespace Qi::Vision;
using namespace Qi::Vision::Matching;
using namespace Qi::Vision::Platform;
using namespace Qi::Vision::GUI;

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

// Draw match result on image
void DrawMatchResult(QImage& colorImg, double row, double col, double angle, double score,
                     const std::vector<double>& contourRows, const std::vector<double>& contourCols,
                     double modelWidth, double modelHeight) {
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    // Draw contour (green)
    for (size_t i = 0; i < contourRows.size(); ++i) {
        double px = col + cosA * contourCols[i] - sinA * contourRows[i];
        double py = row + sinA * contourCols[i] + cosA * contourRows[i];
        int32_t ix = static_cast<int32_t>(px);
        int32_t iy = static_cast<int32_t>(py);
        if (ix >= 0 && ix < colorImg.Width() && iy >= 0 && iy < colorImg.Height()) {
            uint8_t* dst = static_cast<uint8_t*>(colorImg.Data());
            size_t idx = iy * colorImg.Stride() + ix * 3;
            dst[idx + 0] = 0;
            dst[idx + 1] = 255;
            dst[idx + 2] = 0;
        }
    }

    // Draw bounding box (red)
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
    Draw::Line(colorImg, px[0], py[0], px[1], py[1], Color::Red(), 2);
    Draw::Line(colorImg, px[1], py[1], px[2], py[2], Color::Red(), 2);
    Draw::Line(colorImg, px[2], py[2], px[3], py[3], Color::Red(), 2);
    Draw::Line(colorImg, px[3], py[3], px[0], py[0], Color::Red(), 2);

    // Draw center cross (yellow)
    Draw::Cross(colorImg, Point2d{col, row}, 15, angle, Color::Yellow(), 2);

    // Draw score text
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << score;
    Draw::Text(colorImg, static_cast<int32_t>(col) + 10,
               static_cast<int32_t>(row) - 10, oss.str(), Color::Cyan());
}

int main(int argc, char* argv[]) {
    std::cout << "=== QiVision Interactive Shape Matching ===" << std::endl;
    std::cout << std::endl;

    // Data directory - ampoules images
    std::string dataDir = "tests/data/halcon_images/ampoules/";
    std::vector<std::string> imageFiles = {
        "ampoules_01.png",
        "ampoules_02.png",
        "ampoules_03.png",
        "ampoules_04.png",
        "ampoules_05.png",
        "ampoules_06.png",
        "ampoules_07.png",
        "ampoules_08.png"
    };

    // Check for command-line ROI specification
    // Usage: ./matching_interactive [x y width height]
    int32_t roiX = -1, roiY = -1, roiW = -1, roiH = -1;
    bool useCommandLineROI = false;

    if (argc == 5) {
        roiX = std::atoi(argv[1]);
        roiY = std::atoi(argv[2]);
        roiW = std::atoi(argv[3]);
        roiH = std::atoi(argv[4]);
        useCommandLineROI = true;
        std::cout << "Using command-line ROI: (" << roiX << ", " << roiY
                  << ", " << roiW << ", " << roiH << ")" << std::endl;
    } else if (argc > 1) {
        std::cout << "Usage: " << argv[0] << " [x y width height]" << std::endl;
        std::cout << "Example: " << argv[0] << " 1065 2720 135 200" << std::endl;
        std::cout << std::endl;
        std::cout << "If no arguments, interactive ROI drawing mode is used." << std::endl;
        return 1;
    }

    if (!useCommandLineROI) {
        std::cout << "Controls:" << std::endl;
        std::cout << "  - Left mouse drag: Draw ROI rectangle" << std::endl;
        std::cout << "  - Mouse wheel: Zoom in/out" << std::endl;
        std::cout << "  - 'F' key: Fit to window" << std::endl;
        std::cout << "  - Right click: Reset zoom to 1:1" << std::endl;
        std::cout << "  - ESC / 'Q': Quit" << std::endl;
        std::cout << std::endl;
    }

    // Create window with auto-resize
    Window win("Interactive Shape Match", 1024, 768);
    win.SetAutoResize(true, 1600, 900);  // Max size limit

    // Load first image as template
    std::cout << "Loading template image: " << imageFiles[0] << std::endl;
    QImage templateImg = QImage::FromFile(dataDir + imageFiles[0]);
    if (!templateImg.IsValid()) {
        std::cerr << "Failed to load template image!" << std::endl;
        return 1;
    }

    QImage templateGray = ToGrayscale(templateImg);
    std::cout << "Image size: " << templateGray.Width() << " x " << templateGray.Height() << std::endl;
    std::cout << std::endl;

    if (!useCommandLineROI) {
        // Enable zoom/pan for large image navigation
        win.EnableZoomPan(true);

        // Display image and prompt user
        win.SetTitle("Draw ROI: Left-drag to select template region (Wheel=Zoom, F=Fit)");
        win.DispImage(ToRGB(templateGray));

        std::cout << "Please draw a rectangle ROI on the image to select the template region." << std::endl;
        std::cout << "Use mouse wheel to zoom, left-drag to pan when zoomed." << std::endl;
        std::cout << "Press 'F' to fit image to window." << std::endl;
        std::cout << std::endl;

        // Draw ROI interactively
        ROIResult roiResult = win.DrawRectangle();

        if (!roiResult.valid) {
            std::cout << "ROI drawing cancelled." << std::endl;
            return 0;
        }

        // Convert ROI to Rect2i (col1, row1 -> x, y)
        roiX = static_cast<int32_t>(roiResult.col1);
        roiY = static_cast<int32_t>(roiResult.row1);
        roiW = static_cast<int32_t>(roiResult.col2 - roiResult.col1);
        roiH = static_cast<int32_t>(roiResult.row2 - roiResult.row1);
    }

    // Clamp to image bounds
    roiX = std::max(0, std::min(roiX, templateGray.Width() - 1));
    roiY = std::max(0, std::min(roiY, templateGray.Height() - 1));
    roiW = std::min(roiW, templateGray.Width() - roiX);
    roiH = std::min(roiH, templateGray.Height() - roiY);

    if (roiW < 10 || roiH < 10) {
        std::cerr << "ROI too small (min 10x10)!" << std::endl;
        return 1;
    }

    Rect2i roi(roiX, roiY, roiW, roiH);
    std::cout << "Selected ROI: (" << roi.x << ", " << roi.y << ", "
              << roi.width << ", " << roi.height << ")" << std::endl;
    std::cout << std::endl;

    // Display template with ROI marked
    QImage roiVis = ToRGB(templateGray);
    Draw::Rectangle(roiVis, roi.x, roi.y, roi.width, roi.height, Color::Green(), 2);
    win.SetTitle("Template ROI - Creating model...");
    win.DispImage(roiVis);

    // Create shape model
    Timer timer;
    timer.Start();

    std::cout << "Creating shape model..." << std::endl;
    ShapeModel model = CreateShapeModel(
        templateGray, roi,
        5,                      // numLevels
        0, RAD(360), 0,         // angleStart, angleExtent, angleStep (0=auto)
        "auto",                 // optimization
        "ignore_local_polarity", // metric
        "auto", 5               // contrast, minContrast
    );

    if (!model.IsValid()) {
        std::cerr << "Failed to create shape model!" << std::endl;
        return 1;
    }

    double createTime = timer.ElapsedMs();
    std::cout << "Model created in " << std::fixed << std::setprecision(1)
              << createTime << " ms" << std::endl;

    // Get model contours for visualization
    std::vector<double> contourRows, contourCols;
    GetShapeModelContours(model, 1, contourRows, contourCols);
    std::cout << "Model has " << contourRows.size() << " edge points" << std::endl;
    std::cout << std::endl;

    // Display model edges
    int32_t scale = 4;
    QImage edgeImg(roi.width * scale, roi.height * scale, PixelType::UInt8, ChannelType::RGB);
    std::memset(edgeImg.Data(), 0, edgeImg.Height() * edgeImg.Stride());

    uint8_t* dst = static_cast<uint8_t*>(edgeImg.Data());
    for (size_t i = 0; i < contourRows.size(); ++i) {
        int32_t px = static_cast<int32_t>((contourCols[i] + roi.width / 2) * scale);
        int32_t py = static_cast<int32_t>((contourRows[i] + roi.height / 2) * scale);
        for (int32_t dy = -1; dy <= 1; ++dy) {
            for (int32_t dx = -1; dx <= 1; ++dx) {
                int32_t x = px + dx;
                int32_t y = py + dy;
                if (x >= 0 && x < edgeImg.Width() && y >= 0 && y < edgeImg.Height()) {
                    size_t idx = y * edgeImg.Stride() + x * 3;
                    dst[idx + 0] = 0;
                    dst[idx + 1] = 255;
                    dst[idx + 2] = 0;
                }
            }
        }
    }

    win.SetTitle("Model Edges - Press any key to start matching");
    win.DispImage(edgeImg);
    win.WaitKey(2000);

    // Search in all images
    std::cout << "Searching in " << imageFiles.size() << " images..." << std::endl;
    std::cout << std::endl;

    int successCount = 0;
    double totalTime = 0;

    for (size_t i = 0; i < imageFiles.size(); ++i) {
        QImage searchImg = QImage::FromFile(dataDir + imageFiles[i]);
        if (!searchImg.IsValid()) {
            std::cerr << "[" << (i+1) << "] Failed to load: " << imageFiles[i] << std::endl;
            continue;
        }
        QImage searchGray = ToGrayscale(searchImg);

        // Find shape model
        std::vector<double> rows, cols, angles, scores;

        timer.Reset();
        timer.Start();
        FindShapeModel(
            searchGray, model,
            0, RAD(360),            // angleStart, angleExtent
            0.9, 10, 0.5,           // minScore, numMatches, maxOverlap
            "least_squares", 0, 0.9, // subPixel, numLevels, greediness
            rows, cols, angles, scores
        );
        double searchTime = timer.ElapsedMs();
        totalTime += searchTime;

        bool success = !rows.empty();
        if (success) successCount++;

        std::cout << "[" << (i+1) << "/" << imageFiles.size() << "] " << imageFiles[i];
        if (success) {
            std::cout << " - Found " << rows.size() << " match(es), Score="
                      << std::fixed << std::setprecision(3) << scores[0]
                      << ", Time=" << std::setprecision(1) << searchTime << "ms";
        } else {
            std::cout << " - NO MATCH, Time=" << std::setprecision(1) << searchTime << "ms";
        }
        std::cout << std::endl;

        // Draw results
        QImage colorImg = ToRGB(searchGray);
        for (size_t j = 0; j < rows.size(); ++j) {
            DrawMatchResult(colorImg, rows[j], cols[j], angles[j], scores[j],
                           contourRows, contourCols, roi.width, roi.height);
        }

        // Build title
        std::ostringstream title;
        title << "[" << (i+1) << "/" << imageFiles.size() << "] " << imageFiles[i];
        if (success) {
            title << " - Score=" << std::fixed << std::setprecision(2) << scores[0]
                  << " (" << rows.size() << " matches)";
        } else {
            title << " - NO MATCH";
        }

        win.SetTitle(title.str());
        win.DispImage(colorImg);

        // Wait for key
        int32_t key = win.WaitKey(1500);
        if (key == 'q' || key == 'Q' || key == 27) {
            std::cout << "\nUser quit." << std::endl;
            break;
        }
    }

    // Summary
    std::cout << std::endl;
    std::cout << "=== Summary ===" << std::endl;
    std::cout << "Matched: " << successCount << "/" << imageFiles.size() << std::endl;
    std::cout << "Average time: " << std::fixed << std::setprecision(1)
              << (totalTime / imageFiles.size()) << " ms" << std::endl;

    std::cout << std::endl;
    std::cout << "Press any key to exit..." << std::endl;
    win.WaitKey(0);

    return 0;
}
