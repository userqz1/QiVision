/**
 * @file 07_shape_match_draw.cpp
 * @brief Demonstrates shape matching with Halcon-style API
 *
 * Shows how to:
 * 1. Load images from files
 * 2. Create shape model using CreateShapeModel()
 * 3. Save/Load model using WriteShapeModel()/ReadShapeModel()
 * 4. Find matches using FindShapeModel()
 * 5. Draw results and save visualization
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/Draw.h>
#include <QiVision/Matching/ShapeModel.h>
#include <QiVision/Matching/MatchTypes.h>
#include <QiVision/Platform/Timer.h>
#include <QiVision/Platform/FileIO.h>

#include <iostream>
#include <cmath>

using namespace Qi::Vision;
using namespace Qi::Vision::Matching;
using namespace Qi::Vision::Platform;

// Convert RGB to grayscale
QImage ToGray(const QImage& color) {
    if (color.Channels() == 1) {
        return color.Clone();
    }

    QImage gray(color.Width(), color.Height(), PixelType::UInt8, ChannelType::Gray);
    const uint8_t* src = static_cast<const uint8_t*>(color.Data());
    uint8_t* dst = static_cast<uint8_t*>(gray.Data());
    size_t srcStride = color.Stride();
    size_t dstStride = gray.Stride();

    for (int32_t y = 0; y < color.Height(); ++y) {
        const uint8_t* srcRow = src + y * srcStride;
        uint8_t* dstRow = dst + y * dstStride;
        for (int32_t x = 0; x < color.Width(); ++x) {
            uint8_t r = srcRow[x * 3 + 0];
            uint8_t g = srcRow[x * 3 + 1];
            uint8_t b = srcRow[x * 3 + 2];
            dstRow[x] = static_cast<uint8_t>(0.299 * r + 0.587 * g + 0.114 * b);
        }
    }
    return gray;
}

int main() {
    std::cout << "=== QiVision Shape Matching Demo (Halcon-style API) ===" << std::endl;

    // Define paths
    std::string dataDir = "tests/data/matching/image1/";
    std::string outputDir = "tests/data/matching/";
    std::string modelFile = "shape_model.qism";  // QiVision Shape Model

    std::vector<std::string> imageFiles = {
        "052640-20210901141310.jpg",
        "052640-20210901141317.jpg",
        "052640-20210901141321.jpg",
        "052640-20210901141324.jpg",
        "052640-20210901141327.jpg"
    };

    ShapeModel model;
    Timer timer;
    Rect2i modelROI;

    // Model parameters
    int32_t numLevels = 4;
    double modelWidth = 0, modelHeight = 0;

    // =========================================================================
    // Part 1: Create or Load Model
    // =========================================================================

    if (FileExists(modelFile)) {
        // Load existing model using Halcon-style API
        std::cout << "\n1. Loading existing model from: " << modelFile << std::endl;
        timer.Start();
        model = ReadShapeModel(modelFile);
        if (!model.IsValid()) {
            std::cerr << "   Failed to load model!" << std::endl;
            return 1;
        }
        std::cout << "   Model loaded in " << timer.ElapsedMs() << " ms" << std::endl;

        // Get model parameters using Halcon-style API
        int32_t outNumLevels;
        double outAngleStart, outAngleExtent, outAngleStep;
        double outScaleMin, outScaleMax, outScaleStep;
        std::string outMetric;
        GetShapeModelParams(model, outNumLevels, outAngleStart, outAngleExtent, outAngleStep,
                           outScaleMin, outScaleMax, outScaleStep, outMetric);
        std::cout << "   Pyramid levels: " << outNumLevels << std::endl;

        // Get contours to estimate model size
        std::vector<double> contourRows, contourCols;
        GetShapeModelContours(model, 1, contourRows, contourCols);
        std::cout << "   Model points: " << contourRows.size() << std::endl;

        // Compute bounding box
        if (!contourRows.empty()) {
            double minR = contourRows[0], maxR = contourRows[0];
            double minC = contourCols[0], maxC = contourCols[0];
            for (size_t i = 1; i < contourRows.size(); ++i) {
                if (contourRows[i] < minR) minR = contourRows[i];
                if (contourRows[i] > maxR) maxR = contourRows[i];
                if (contourCols[i] < minC) minC = contourCols[i];
                if (contourCols[i] > maxC) maxC = contourCols[i];
            }
            modelWidth = maxC - minC;
            modelHeight = maxR - minR;
            std::cout << "   Model size: " << modelWidth << " x " << modelHeight << std::endl;
        }

        modelROI.width = static_cast<int32_t>(modelWidth);
        modelROI.height = static_cast<int32_t>(modelHeight);

    } else {
        // Create new model from first image
        std::cout << "\n1. Creating new model from: " << imageFiles[0] << std::endl;

        QImage templateColor = QImage::FromFile(dataDir + imageFiles[0]);
        if (templateColor.Empty()) {
            std::cerr << "   Failed to load template image!" << std::endl;
            return 1;
        }
        QImage templateGray = ToGray(templateColor);
        std::cout << "   Image size: " << templateGray.Width() << "x" << templateGray.Height() << std::endl;

        // Define ROI - user specified: top-left (180, 100), bottom-right (390, 170)
        modelROI.x = 180;
        modelROI.y = 100;
        modelROI.width = 390 - 180;   // 210
        modelROI.height = 170 - 100;  // 70
        modelWidth = modelROI.width;
        modelHeight = modelROI.height;
        std::cout << "   ROI: (" << modelROI.x << "," << modelROI.y << ") "
                  << modelROI.width << "x" << modelROI.height << std::endl;

        // Create model using Halcon-style API
        timer.Start();
        model = CreateShapeModel(
            templateGray,
            modelROI,
            numLevels,                  // numLevels
            0, RAD(360), 0,             // angleStart, angleExtent, angleStep (0=auto)
            "auto",                     // optimization
            "use_polarity",             // metric
            30, 10                      // contrast, minContrast
        );

        if (!model.IsValid()) {
            std::cerr << "   Failed to create model!" << std::endl;
            return 1;
        }
        double createTime = timer.ElapsedMs();
        std::cout << "   Model created in " << createTime << " ms" << std::endl;

        // Get model info
        std::vector<double> contourRows, contourCols;
        GetShapeModelContours(model, 1, contourRows, contourCols);
        std::cout << "   Model points: " << contourRows.size() << std::endl;

        // Save model using Halcon-style API
        std::cout << "\n2. Saving model to: " << modelFile << std::endl;
        timer.Start();
        WriteShapeModel(model, modelFile);
        std::cout << "   Model saved in " << timer.ElapsedMs() << " ms" << std::endl;

        // Save template visualization with ROI
        QImage templateVis = Draw::PrepareForDrawing(templateGray);
        Draw::Rectangle(templateVis, modelROI, Color::Green(), 2);
        std::string roiPath = outputDir + "output_template_roi.jpg";
        templateVis.SaveToFile(roiPath);
        std::cout << "   Saved: " << roiPath << std::endl;

        // Save model edge points visualization
        QImage modelVis = Draw::PrepareForDrawing(templateGray);
        std::cout << "   Drawing " << contourRows.size() << " model points..." << std::endl;

        // Model points are relative to ROI center, need to offset
        double centerX = modelROI.x + modelROI.width / 2.0;
        double centerY = modelROI.y + modelROI.height / 2.0;

        for (size_t i = 0; i < contourRows.size(); ++i) {
            int32_t px = static_cast<int32_t>(centerX + contourCols[i] + 0.5);
            int32_t py = static_cast<int32_t>(centerY + contourRows[i] + 0.5);
            Draw::Pixel(modelVis, px, py, Color::Green());
            Draw::Pixel(modelVis, px+1, py, Color::Green());
            Draw::Pixel(modelVis, px, py+1, Color::Green());
        }
        Draw::Rectangle(modelVis, modelROI, Color::Red(), 1);
        std::string edgePath = outputDir + "output_model_edges.jpg";
        modelVis.SaveToFile(edgePath);
        std::cout << "   Saved: " << edgePath << " (extracted edge points)" << std::endl;
    }

    // =========================================================================
    // Part 2: Search in Images
    // =========================================================================

    std::cout << "\n3. Searching in images..." << std::endl;

    // Get model contour for visualization
    std::vector<double> modelContourRows, modelContourCols;
    GetShapeModelContours(model, 1, modelContourRows, modelContourCols);
    std::cout << "   Model contour has " << modelContourRows.size() << " points" << std::endl;

    // Search parameters (Halcon-style)
    double searchAngleStart = 0;
    double searchAngleExtent = RAD(360);
    double minScore = 0.5;
    int32_t maxMatches = 5;
    double maxOverlap = 0.5;
    std::string subPixel = "least_squares";
    int32_t searchNumLevels = 0;  // 0 = use model levels
    double greediness = 0.9;

    // Process each image
    for (size_t i = 0; i < imageFiles.size(); ++i) {
        std::cout << "\n   [" << (i+1) << "/" << imageFiles.size() << "] "
                  << imageFiles[i] << std::endl;

        // Load image
        QImage searchColor = QImage::FromFile(dataDir + imageFiles[i]);
        if (searchColor.Empty()) {
            std::cerr << "      Failed to load image!" << std::endl;
            continue;
        }
        QImage searchGray = ToGray(searchColor);

        // Search using Halcon-style API
        std::vector<double> rows, cols, angles, scores;

        timer.Reset();
        timer.Start();
        FindShapeModel(searchGray, model, searchAngleStart, searchAngleExtent,
                       minScore, maxMatches, maxOverlap, subPixel, searchNumLevels, greediness,
                       rows, cols, angles, scores);
        double searchTime = timer.ElapsedMs();
        timer.Stop();

        std::cout << "      Found " << rows.size() << " matches in "
                  << searchTime << " ms" << std::endl;

        // Draw results
        QImage resultImage = Draw::PrepareForDrawing(searchGray);

        for (size_t j = 0; j < rows.size(); ++j) {
            std::cout << "      Match " << (j+1) << ": "
                      << "pos=(" << cols[j] << "," << rows[j] << ") "
                      << "angle=" << DEG(angles[j]) << " deg "
                      << "score=" << scores[j] << std::endl;

            double cosA = std::cos(angles[j]);
            double sinA = std::sin(angles[j]);

            // Draw model contour at match position (green)
            for (size_t k = 0; k < modelContourRows.size(); ++k) {
                double px = cols[j] + cosA * modelContourCols[k] - sinA * modelContourRows[k];
                double py = rows[j] + sinA * modelContourCols[k] + cosA * modelContourRows[k];
                int32_t ix = static_cast<int32_t>(px);
                int32_t iy = static_cast<int32_t>(py);
                if (ix >= 0 && ix < resultImage.Width() && iy >= 0 && iy < resultImage.Height()) {
                    Draw::Pixel(resultImage, ix, iy, Color::Green());
                }
            }

            // Draw bounding box (red)
            double halfW = modelWidth / 2.0;
            double halfH = modelHeight / 2.0;

            // Compute rotated corners
            double corners[4][2] = {
                {-halfW, -halfH}, {halfW, -halfH},
                {halfW, halfH}, {-halfW, halfH}
            };
            int32_t px[4], py[4];
            for (int k = 0; k < 4; ++k) {
                px[k] = static_cast<int32_t>(cols[j] + cosA * corners[k][0] - sinA * corners[k][1]);
                py[k] = static_cast<int32_t>(rows[j] + sinA * corners[k][0] + cosA * corners[k][1]);
            }
            // Draw box edges
            Draw::Line(resultImage, px[0], py[0], px[1], py[1], Color::Red(), 2);
            Draw::Line(resultImage, px[1], py[1], px[2], py[2], Color::Red(), 2);
            Draw::Line(resultImage, px[2], py[2], px[3], py[3], Color::Red(), 2);
            Draw::Line(resultImage, px[3], py[3], px[0], py[0], Color::Red(), 2);

            // Draw center cross
            Draw::Cross(resultImage, cols[j], rows[j], 10, Color::Yellow(), 2);
        }

        // Save result
        std::string outPath = outputDir + "output_match_" + std::to_string(i + 1) + ".jpg";
        if (resultImage.SaveToFile(outPath)) {
            std::cout << "      Saved: " << outPath << std::endl;
        }
    }

    std::cout << "\n=== Demo Complete ===" << std::endl;
    return 0;
}
