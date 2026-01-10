/**
 * @file 08_shape_match_large.cpp
 * @brief Shape matching test for small (640x512) and large (2048x4001) images
 *
 * Tests both image1 (small) and image2 (large) directories
 * Outputs: template ROI, model edges, match results for all images
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/Draw.h>
#include <QiVision/Matching/ShapeModel.h>
#include <QiVision/Platform/Timer.h>
#include <QiVision/Platform/FileIO.h>

#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstring>

using namespace Qi::Vision;
using namespace Qi::Vision::Matching;
using namespace Qi::Vision::Platform;

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
            // Standard luminance conversion for RGB images
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

// Save model visualization (ROI + edges)
void SaveModelVisualization(const QImage& templateGray, const Rect2i& roi,
                            const ShapeModel& model, const std::string& prefix) {
    // Save template with ROI
    QImage roiVis = ToRGB(templateGray);
    Draw::Rectangle(roiVis, roi.x, roi.y, roi.width, roi.height, Color::Green(), 2);
    std::string roiPath = prefix + "_template_roi.bmp";
    roiVis.SaveToFile(roiPath);
    std::cout << "   Saved: " << roiPath << std::endl;

    // Save model edges on black background
    auto modelPoints = model.GetModelPoints(0);
    QImage edgeImg(roi.width, roi.height, PixelType::UInt8, ChannelType::RGB);
    std::memset(edgeImg.Data(), 0, edgeImg.Height() * edgeImg.Stride());

    uint8_t* dst = static_cast<uint8_t*>(edgeImg.Data());
    for (const auto& pt : modelPoints) {
        int32_t px = static_cast<int32_t>(pt.x + roi.width / 2);
        int32_t py = static_cast<int32_t>(pt.y + roi.height / 2);
        if (px >= 0 && px < roi.width && py >= 0 && py < roi.height) {
            size_t idx = py * edgeImg.Stride() + px * 3;
            dst[idx + 0] = 0;
            dst[idx + 1] = 255;
            dst[idx + 2] = 0;
        }
    }
    std::string edgePath = prefix + "_model_edges.bmp";
    edgeImg.SaveToFile(edgePath);
    std::cout << "   Saved: " << edgePath << " (" << modelPoints.size() << " points)" << std::endl;
}

// Draw match result on image
void DrawMatchResult(QImage& colorImg, const MatchResult& m,
                     const std::vector<Point2d>& contour, const ModelStats& stats) {
    // Draw contour (green)
    Draw::MatchResultWithContour(colorImg, m, contour, Color::Green(), 2);

    // Draw bounding box (red)
    double halfW = stats.Width() / 2.0;
    double halfH = stats.Height() / 2.0;
    double cosA = std::cos(m.angle);
    double sinA = std::sin(m.angle);

    double corners[4][2] = {
        {-halfW, -halfH}, {halfW, -halfH},
        {halfW, halfH}, {-halfW, halfH}
    };
    int32_t px[4], py[4];
    for (int k = 0; k < 4; ++k) {
        px[k] = static_cast<int32_t>(m.x + cosA * corners[k][0] - sinA * corners[k][1]);
        py[k] = static_cast<int32_t>(m.y + sinA * corners[k][0] + cosA * corners[k][1]);
    }
    Draw::Line(colorImg, px[0], py[0], px[1], py[1], Color::Red(), 2);
    Draw::Line(colorImg, px[1], py[1], px[2], py[2], Color::Red(), 2);
    Draw::Line(colorImg, px[2], py[2], px[3], py[3], Color::Red(), 2);
    Draw::Line(colorImg, px[3], py[3], px[0], py[0], Color::Red(), 2);

    // Draw center cross (yellow)
    Draw::Cross(colorImg, m.x, m.y, 15, Color::Yellow(), 2);
}

// Test a set of images
void TestImageSet(const std::string& name, const std::string& dataDir,
                  const std::string& outputPrefix,
                  const std::vector<std::string>& imageFiles,
                  const Rect2i& roi, const ModelParams& modelParams,
                  const SearchParams& searchParams) {

    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "  " << name << std::endl;
    std::cout << std::string(70, '=') << std::endl;

    ShapeModel model;
    Timer timer;

    // Create model from first image
    std::cout << "\n1. Creating model from: " << imageFiles[0] << std::endl;

    QImage templateImg = QImage::FromFile(dataDir + imageFiles[0]);
    if (!templateImg.IsValid()) {
        std::cerr << "   Failed to load template!" << std::endl;
        return;
    }

    QImage templateGray = ToGrayscale(templateImg);
    std::cout << "   Image size: " << templateGray.Width() << " x " << templateGray.Height() << std::endl;
    std::cout << "   ROI: (" << roi.x << ", " << roi.y << ", " << roi.width << ", " << roi.height << ")" << std::endl;

    timer.Start();
    if (!model.Create(templateGray, roi, modelParams)) {
        std::cerr << "   Failed to create model!" << std::endl;
        return;
    }
    std::cout << "   Model created in " << timer.ElapsedMs() << " ms" << std::endl;

    auto stats = model.GetStats();
    std::cout << "   Model points: " << stats.numPoints << ", Levels: " << stats.numLevels << std::endl;

    // Save visualization
    SaveModelVisualization(templateGray, roi, model, outputPrefix);

    // Search in all images
    std::cout << "\n2. Searching in " << imageFiles.size() << " images..." << std::endl;

    auto contour = model.GetModelContour();
    int successCount = 0;
    double totalTime = 0;

    for (size_t i = 0; i < imageFiles.size(); ++i) {
        QImage searchImg = QImage::FromFile(dataDir + imageFiles[i]);
        if (!searchImg.IsValid()) {
            std::cerr << "   [" << (i+1) << "] Failed to load: " << imageFiles[i] << std::endl;
            continue;
        }
        QImage searchGray = ToGrayscale(searchImg);

        timer.Reset();
        timer.Start();
        auto matches = model.Find(searchGray, searchParams);
        double searchTime = timer.ElapsedMs();
        totalTime += searchTime;

        bool success = !matches.empty();
        if (success) successCount++;

        std::cout << "   [" << (i+1) << "/" << imageFiles.size() << "] "
                  << imageFiles[i] << " - ";
        if (success) {
            std::cout << "Match: (" << std::fixed << std::setprecision(1)
                      << matches[0].x << "," << matches[0].y << ") "
                      << "Score=" << std::setprecision(3) << matches[0].score
                      << " Time=" << std::setprecision(1) << searchTime << "ms" << std::endl;
        } else {
            std::cout << "NO MATCH (Time=" << searchTime << "ms)" << std::endl;
        }

        // Save result image
        QImage colorImg = ToRGB(searchGray);
        for (const auto& m : matches) {
            DrawMatchResult(colorImg, m, contour, stats);
        }
        std::string outPath = outputPrefix + "_" + std::to_string(i+1) + ".bmp";
        colorImg.SaveToFile(outPath);
    }

    std::cout << "\n   Summary: " << successCount << "/" << imageFiles.size()
              << " matched, Avg time: " << (totalTime / imageFiles.size()) << " ms" << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "=== QiVision Shape Matching - Complete Test ===" << std::endl;

    std::string outputDir = "tests/data/matching/";

    // =========================================================================
    // Test 1: Small Images (640x512)
    // =========================================================================
    {
        std::vector<std::string> smallImages = {
            "052640-20210901141310.jpg",
            "052640-20210901141317.jpg",
            "052640-20210901141321.jpg",
            "052640-20210901141324.jpg",
            "052640-20210901141327.jpg"
        };

        Rect2i smallROI(180, 100, 210, 70);

        ModelParams smallModelParams;
        smallModelParams.SetContrastAutoHysteresis();
        smallModelParams.SetNumLevels(4);

        SearchParams smallSearchParams;
        smallSearchParams.SetAngleRange(0, 2 * M_PI);
        smallSearchParams.SetMinScore(0.5);
        smallSearchParams.SetGreediness(0.9);
        smallSearchParams.SetMaxMatches(5);

        TestImageSet("Small Images (640x512)",
                     "tests/data/matching/image1/",
                     outputDir + "output_small",
                     smallImages, smallROI,
                     smallModelParams, smallSearchParams);
    }

    // =========================================================================
    // Test 2: Large Images (2048x4001)
    // =========================================================================
    {
        std::vector<std::string> largeImages = {
            "2025120119482739.bmp",
            "20251201194802191.bmp",
            "20251201194804137.bmp",
            "20251201194805266.bmp",
            "20251201194806360.bmp",
            "2025120119482935.bmp",
            "20251201194837127.bmp",
            "20251201194840615.bmp",
            "20251201194842271.bmp",
            "20251201194843675.bmp",
            "20251201194844759.bmp"
        };

        Rect2i largeROI(1065, 2720, 135, 200);

        ModelParams largeModelParams;
        largeModelParams.numLevels = 5;
        largeModelParams.angleStart = 0;
        largeModelParams.angleExtent = 2 * M_PI;
        largeModelParams.contrastHigh = 10;
        largeModelParams.contrastLow = 5;
        largeModelParams.metric = MetricMode::IgnoreLocalPolarity;
        largeModelParams.optimization = OptimizationMode::Auto;

        SearchParams largeSearchParams;
        largeSearchParams.angleStart = 0;
        largeSearchParams.angleExtent = 2 * M_PI;
        largeSearchParams.minScore = 0.7;
        largeSearchParams.greediness = 0.9;
        largeSearchParams.maxMatches = 10;

        TestImageSet("Large Images (2048x4001)",
                     "tests/data/matching/image2/",
                     outputDir + "output_large",
                     largeImages, largeROI,
                     largeModelParams, largeSearchParams);
    }

    std::cout << "\n=== All Tests Complete ===" << std::endl;
    return 0;
}
