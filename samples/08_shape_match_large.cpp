/**
 * @file 08_shape_match_large.cpp
 * @brief Shape matching test using Halcon-style API
 *
 * Tests both image1 (small) and image2 (large) directories
 * Outputs: template ROI, model edges, match results for all images
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/Draw.h>
#include <QiVision/Matching/ShapeModel.h>
#include <QiVision/Matching/MatchTypes.h>
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

    // Get model contours using Halcon-style API
    std::vector<double> contourRows, contourCols;
    GetShapeModelContours(model, 1, contourRows, contourCols);

    // Save model edges on black background
    QImage edgeImg(roi.width, roi.height, PixelType::UInt8, ChannelType::RGB);
    std::memset(edgeImg.Data(), 0, edgeImg.Height() * edgeImg.Stride());

    uint8_t* dst = static_cast<uint8_t*>(edgeImg.Data());
    for (size_t i = 0; i < contourRows.size(); ++i) {
        int32_t px = static_cast<int32_t>(contourCols[i] + roi.width / 2);
        int32_t py = static_cast<int32_t>(contourRows[i] + roi.height / 2);
        if (px >= 0 && px < roi.width && py >= 0 && py < roi.height) {
            size_t idx = py * edgeImg.Stride() + px * 3;
            dst[idx + 0] = 0;
            dst[idx + 1] = 255;
            dst[idx + 2] = 0;
        }
    }
    std::string edgePath = prefix + "_model_edges.bmp";
    edgeImg.SaveToFile(edgePath);
    std::cout << "   Saved: " << edgePath << " (" << contourRows.size() << " points)" << std::endl;
}

// Draw match result on image
void DrawMatchResult(QImage& colorImg, double row, double col, double angle, double score,
                     const std::vector<double>& contourRows, const std::vector<double>& contourCols,
                     double modelWidth, double modelHeight) {
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    // Draw contour (green)
    for (size_t i = 0; i < contourRows.size(); ++i) {
        // Transform contour point to image coordinates
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

    // Draw rotated center cross (yellow)
    Draw::Cross(colorImg, Point2d{col, row}, 15, angle, Color::Yellow(), 2);
}

// Test a set of images using Halcon-style API
void TestImageSet(const std::string& name, const std::string& dataDir,
                  const std::string& outputPrefix,
                  const std::vector<std::string>& imageFiles,
                  const Rect2i& roi,
                  int32_t numLevels,
                  double angleStart, double angleExtent, double angleStep,
                  const std::string& optimization, const std::string& metric,
                  const std::string& contrast, double minContrast,
                  double searchAngleStart, double searchAngleExtent,
                  double minScore, int32_t numMatches, double maxOverlap,
                  const std::string& subPixel, int32_t searchNumLevels, double greediness) {

    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "  " << name << std::endl;
    std::cout << std::string(70, '=') << std::endl;

    Timer timer;

    // Create model from first image using Halcon-style API
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
    // Halcon-style: CreateShapeModel with ROI
    ShapeModel model = CreateShapeModel(templateGray, roi, numLevels,
                                        angleStart, angleExtent, angleStep,
                                        optimization, metric, contrast, minContrast);

    if (!model.IsValid()) {
        std::cerr << "   Failed to create model!" << std::endl;
        return;
    }
    std::cout << "   Model created in " << timer.ElapsedMs() << " ms" << std::endl;

    // Get model parameters using Halcon-style API
    int32_t outNumLevels;
    double outAngleStart, outAngleExtent, outAngleStep;
    double outScaleMin, outScaleMax, outScaleStep;
    std::string outMetric;
    GetShapeModelParams(model, outNumLevels, outAngleStart, outAngleExtent, outAngleStep,
                        outScaleMin, outScaleMax, outScaleStep, outMetric);
    std::cout << "   Model levels: " << outNumLevels << std::endl;

    // Save visualization
    SaveModelVisualization(templateGray, roi, model, outputPrefix);

    // Get contours for drawing
    std::vector<double> contourRows, contourCols;
    GetShapeModelContours(model, 1, contourRows, contourCols);

    // Search in all images
    std::cout << "\n2. Searching in " << imageFiles.size() << " images..." << std::endl;

    int successCount = 0;
    double totalTime = 0;

    for (size_t i = 0; i < imageFiles.size(); ++i) {
        QImage searchImg = QImage::FromFile(dataDir + imageFiles[i]);
        if (!searchImg.IsValid()) {
            std::cerr << "   [" << (i+1) << "] Failed to load: " << imageFiles[i] << std::endl;
            continue;
        }
        QImage searchGray = ToGrayscale(searchImg);

        // Halcon-style: FindShapeModel with output parameters
        std::vector<double> rows, cols, angles, scores;

        timer.Reset();
        timer.Start();
        FindShapeModel(searchGray, model, searchAngleStart, searchAngleExtent,
                       minScore, numMatches, maxOverlap, subPixel, searchNumLevels, greediness,
                       rows, cols, angles, scores);
        double searchTime = timer.ElapsedMs();
        totalTime += searchTime;

        bool success = !rows.empty();
        if (success) successCount++;

        std::cout << "   [" << (i+1) << "/" << imageFiles.size() << "] "
                  << imageFiles[i] << std::endl;
        if (success) {
            std::cout << "      Match: (" << std::fixed << std::setprecision(1)
                      << cols[0] << "," << rows[0] << ") "
                      << "Score=" << std::setprecision(3) << scores[0]
                      << " Angle=" << std::setprecision(1) << DEG(angles[0]) << "deg" << std::endl;
        } else {
            std::cout << "      NO MATCH" << std::endl;
        }
        std::cout << "      Time: " << std::setprecision(1) << searchTime << "ms" << std::endl;
        std::cout << "      Found: " << rows.size() << " matches" << std::endl;

        // Save result image
        QImage colorImg = ToRGB(searchGray);
        for (size_t j = 0; j < rows.size(); ++j) {
            DrawMatchResult(colorImg, rows[j], cols[j], angles[j], scores[j],
                           contourRows, contourCols, roi.width, roi.height);
        }
        std::string outPath = outputPrefix + "_" + std::to_string(i+1) + ".bmp";
        colorImg.SaveToFile(outPath);
    }

    std::cout << "\n   Summary: " << successCount << "/" << imageFiles.size()
              << " matched, Avg time: " << (totalTime / imageFiles.size()) << " ms" << std::endl;
}

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    std::cout << "=== QiVision Shape Matching - Halcon-style API Test ===" << std::endl;

    // =========================================================================
    // Test 1: Small Images (640x512) - XLDContour mode
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

        TestImageSet("Small Images (640x512)",
                     "tests/data/matching/image1/",
                     "tests/data/matching/image1_result/output",
                     smallImages, smallROI,
                     4,                      // numLevels
                     0, RAD(360), 0,         // angleStart, angleExtent, angleStep (0=auto)
                     "auto",                 // optimization (storage optimization level)
                     "use_polarity",         // metric
                     "auto", 10,             // contrast (auto), minContrast
                     0, RAD(360),            // search angleStart, angleExtent
                     0.9, 5, 0.5,            // minScore, numMatches, maxOverlap
                     "least_squares", 0, 0.9 // subPixel, numLevels, greediness
        );
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

        TestImageSet("Large Images (2048x4001)",
                     "tests/data/matching/image2/",
                     "tests/data/matching/image2_result/output",
                     largeImages, largeROI,
                     5,                             // numLevels
                     0, RAD(360), 0,                // angleStart, angleExtent, angleStep
                     "auto",                        // optimization (storage optimization)
                     "ignore_local_polarity",       // metric
                     "auto", 5,                     // contrast (auto), minContrast
                     0, RAD(360),                   // search angleStart, angleExtent
                     0.9, 10, 0.5,                  // minScore, numMatches, maxOverlap
                     "least_squares", 0, 0.9        // subPixel, numLevels, greediness
        );
    }

    // =========================================================================
    // Test 3: Image3 (1280x1024) - 66 images
    // =========================================================================
    {
        std::vector<std::string> image3Files = {
            // Template image first
            "202412121422747.bmp",
            // All other images (65 total, excluding 20241021163820218.bmp - wrong format)
            "20241021163730950.bmp",
            // "20241021163820218.bmp",  // EXCLUDED: 1024x1280x32 (wrong size and format)
            "20241021163823205.bmp",
            "20241021163826212.bmp",
            "20241021163829214.bmp",
            "20241021163832216.bmp",
            "20241021163835217.bmp",
            "20241021163838214.bmp",
            "20241021163841216.bmp",
            "20241021163844224.bmp",
            "20241021163847216.bmp",
            "20241021163850215.bmp",
            "20241021163853214.bmp",
            "20241021163856217.bmp",
            "20241212141622357.bmp",
            "20241212141622370.bmp",
            "20241212141718906.bmp",
            "20241212141718921.bmp",
            "20241212141815739.bmp",
            "20241212141828746.bmp",
            "20241212142048783.bmp",
            "20241212142048797.bmp",
            "20241212142157595.bmp",
            "20241212142157608.bmp",
            "20241212142223555.bmp",
            "20241212142235286.bmp",
            "20241212195933476.bmp",
            "2024121220320483.bmp",
            "20241212212921224.bmp",
            "20241212212923225.bmp",
            "20241212213145414.bmp",
            "20241212213147407.bmp",
            "20241212213149405.bmp",
            "20241212213331374.bmp",
            "20241212213333350.bmp",
            "20241212213335372.bmp",
            "20241212213616738.bmp",
            "20241212213829107.bmp",
            "20241212213845786.bmp",
            "20241212214038339.bmp",
            "20241212214256310.bmp",
            "20241212214332142.bmp",
            "20241212214334144.bmp",
            "20241212214336140.bmp",
            "2024121221829183.bmp",
            "2024121221951633.bmp",
            "20241216174441447.bmp",
            "20241216174444376.bmp",
            "20241216174447349.bmp",
            "20241216174611370.bmp",
            "2024121617465344.bmp",
            "20241216174659345.bmp",
            "2024121617468370.bmp",
            "2024121617472365.bmp",
            "2024121617475358.bmp",
            "20241216174814380.bmp",
            "20241216174817366.bmp",
            "20241216174820367.bmp",
            "20241216174911370.bmp",
            "20241216174914373.bmp",
            "20241216174959372.bmp",
            "2024121617498368.bmp",
            "2024121617502355.bmp",
            "2024121617505369.bmp",
            "20241217133454888.bmp",
            "20241217134935725.bmp"
        };

        // ROI: (580, 340) to (650, 400) -> x=580, y=340, width=70, height=60
        Rect2i image3ROI(580, 340, 70, 60);

        TestImageSet("Image3 (1280x1024) - 66 images",
                     "tests/data/matching/image3/",
                     "tests/data/matching/image3_result/output",
                     image3Files, image3ROI,
                     4,                             // numLevels
                     0, RAD(360), 0,                // angleStart, angleExtent, angleStep
                     "auto",                        // optimization
                     "ignore_local_polarity",       // metric
                     "auto", 10,                    // contrast (auto), minContrast
                     0, RAD(360),                   // search angleStart, angleExtent
                     0.9, 10, 0.5,                  // minScore, numMatches, maxOverlap
                     "least_squares", 0, 0.9        // subPixel, numLevels, greediness
        );
    }

    // =========================================================================
    // Test 4: Image4 - Rotated images (10° to 200°, step 10°)
    // =========================================================================
    {
        std::vector<std::string> image4Files = {
            // Template image first
            "rotated_10.png",
            // All rotated images (20 total)
            "rotated_20.png",
            "rotated_30.png",
            "rotated_40.png",
            "rotated_50.png",
            "rotated_60.png",
            "rotated_70.png",
            "rotated_80.png",
            "rotated_90.png",
            "rotated_100.png",
            "rotated_110.png",
            "rotated_120.png",
            "rotated_130.png",
            "rotated_140.png",
            "rotated_150.png",
            "rotated_160.png",
            "rotated_170.png",
            "rotated_180.png",
            "rotated_190.png",
            "rotated_200.png"
        };

        // ROI: (266, 467) to (300, 496) -> x=266, y=467, width=34, height=29
        Rect2i image4ROI(266, 467, 34, 29);

        TestImageSet("Image4 (Rotated) - 20 images",
                     "tests/data/matching/image4/",
                     "tests/data/matching/image4_result/output",
                     image4Files, image4ROI,
                     4,                             // numLevels
                     0, RAD(360), 0,                // angleStart, angleExtent, angleStep
                     "auto",                        // optimization
                     "use_polarity",                // metric - stricter matching
                     "auto", 10,                    // contrast (auto), minContrast
                     0, RAD(360),                   // search angleStart, angleExtent
                     0.9, 10, 0.5,                  // minScore, numMatches, maxOverlap
                     "least_squares", 0, 0.9        // subPixel, numLevels, greediness
        );
    }

    std::cout << "\n=== All Tests Complete ===" << std::endl;
    return 0;
}
