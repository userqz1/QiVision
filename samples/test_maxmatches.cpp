/**
 * @file test_maxmatches.cpp
 * @brief Test maxMatches optimization for candidate reduction
 *
 * Demonstrates:
 * - maxMatches=1 (single target): minimal candidate overhead
 * - maxMatches=10 (multi-target): moderate candidates
 * - maxMatches=0 (unlimited): full candidate exploration
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Matching/ShapeModel.h>
#include <QiVision/Platform/Timer.h>

#include <iostream>
#include <iomanip>

using namespace Qi::Vision;
using namespace Qi::Vision::Matching;
using namespace Qi::Vision::Platform;

int main() {
    std::cout << "=== maxMatches Optimization Test ===" << std::endl;
    std::cout << std::endl;

    std::string dataDir = "tests/data/matching/image2/";
    std::string templateFile = "2025120119482739.bmp";
    std::string searchFile = "20251201194802191.bmp";
    Rect2i roi(1065, 2720, 135, 200);

    // Load and convert images
    QImage templateImg = QImage::FromFile(dataDir + templateFile);
    QImage searchImg = QImage::FromFile(dataDir + searchFile);

    if (!templateImg.IsValid() || !searchImg.IsValid()) {
        std::cerr << "Failed to load images!" << std::endl;
        return 1;
    }

    auto convertToGray = [](QImage& img) {
        if (img.Channels() == 3) {
            QImage gray(img.Width(), img.Height(), PixelType::UInt8, ChannelType::Gray);
            const uint8_t* src = static_cast<const uint8_t*>(img.Data());
            uint8_t* dst = static_cast<uint8_t*>(gray.Data());
            size_t srcStride = img.Stride();
            size_t dstStride = gray.Stride();
            for (int32_t y = 0; y < img.Height(); ++y) {
                const uint8_t* srcRow = src + y * srcStride;
                uint8_t* dstRow = dst + y * dstStride;
                for (int32_t x = 0; x < img.Width(); ++x) {
                    dstRow[x] = srcRow[x * 3];
                }
            }
            img = std::move(gray);
        }
    };

    convertToGray(templateImg);
    convertToGray(searchImg);

    std::cout << "Image size: " << searchImg.Width() << " x " << searchImg.Height() << std::endl;
    std::cout << std::endl;

    // Create model once
    ModelParams modelParams;
    modelParams.numLevels = 5;
    modelParams.contrastHigh = 10;
    modelParams.contrastLow = 5;
    modelParams.metric = MetricMode::IgnoreLocalPolarity;
    modelParams.optimization = OptimizationMode::PointReductionHigh;
    modelParams.pregeneration = true;
    modelParams.angleStart = 0.0;
    modelParams.angleExtent = 2 * M_PI;

    ShapeModel model;
    if (!model.Create(templateImg, roi, modelParams)) {
        std::cerr << "Failed to create model!" << std::endl;
        return 1;
    }

    Timer timer;
    const int NUM_RUNS = 3;

    // Test 1: maxMatches = 1 (single target - minimal candidates)
    {
        std::cout << "========================================" << std::endl;
        std::cout << "Test 1: maxMatches = 1 (Single Target)" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Strategy: Keep minimal candidates (~10-20)" << std::endl;
        std::cout << std::endl;

        SearchParams searchParams;
        searchParams.minScore = 0.5;
        searchParams.subpixelMethod = SubpixelMethod::LeastSquares;
        searchParams.greediness = 0.8;
        searchParams.maxMatches = 1;  // Only need 1 result

        double totalTime = 0.0;
        for (int run = 0; run < NUM_RUNS; ++run) {
            timer.Reset();
            timer.Start();
            auto results = model.Find(searchImg, searchParams);
            totalTime += timer.ElapsedMs();

            if (run == 0 && !results.empty()) {
                std::cout << "  Result: score=" << std::fixed << std::setprecision(6)
                          << results[0].score << " at (" << results[0].x << ", "
                          << results[0].y << ") angle=" << results[0].angle << std::endl;
            }
        }

        double avgTime = totalTime / NUM_RUNS;
        std::cout << "  Average search time: " << avgTime << " ms" << std::endl;
        std::cout << std::endl;
    }

    // Test 2: maxMatches = 10 (multi-target - moderate candidates)
    {
        std::cout << "========================================" << std::endl;
        std::cout << "Test 2: maxMatches = 10 (Multi-Target)" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Strategy: Keep moderate candidates (~50-100)" << std::endl;
        std::cout << std::endl;

        SearchParams searchParams;
        searchParams.minScore = 0.5;
        searchParams.subpixelMethod = SubpixelMethod::LeastSquares;
        searchParams.greediness = 0.8;
        searchParams.maxMatches = 10;  // Need up to 10 results

        double totalTime = 0.0;
        for (int run = 0; run < NUM_RUNS; ++run) {
            timer.Reset();
            timer.Start();
            auto results = model.Find(searchImg, searchParams);
            totalTime += timer.ElapsedMs();

            if (run == 0) {
                std::cout << "  Found " << results.size() << " matches" << std::endl;
                if (!results.empty()) {
                    std::cout << "  Best: score=" << std::fixed << std::setprecision(6)
                              << results[0].score << std::endl;
                }
            }
        }

        double avgTime = totalTime / NUM_RUNS;
        std::cout << "  Average search time: " << avgTime << " ms" << std::endl;
        std::cout << std::endl;
    }

    // Test 3: maxMatches = 0 (unlimited - full exploration)
    {
        std::cout << "========================================" << std::endl;
        std::cout << "Test 3: maxMatches = 0 (Unlimited)" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Strategy: Full candidate exploration (~100-500)" << std::endl;
        std::cout << std::endl;

        SearchParams searchParams;
        searchParams.minScore = 0.5;
        searchParams.subpixelMethod = SubpixelMethod::LeastSquares;
        searchParams.greediness = 0.8;
        searchParams.maxMatches = 0;  // No limit

        double totalTime = 0.0;
        for (int run = 0; run < NUM_RUNS; ++run) {
            timer.Reset();
            timer.Start();
            auto results = model.Find(searchImg, searchParams);
            totalTime += timer.ElapsedMs();

            if (run == 0) {
                std::cout << "  Found " << results.size() << " matches" << std::endl;
                if (!results.empty()) {
                    std::cout << "  Best: score=" << std::fixed << std::setprecision(6)
                              << results[0].score << std::endl;
                }
            }
        }

        double avgTime = totalTime / NUM_RUNS;
        std::cout << "  Average search time: " << avgTime << " ms" << std::endl;
        std::cout << std::endl;
    }

    std::cout << "========================================" << std::endl;
    std::cout << "Summary" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Expected behavior:" << std::endl;
    std::cout << "  - maxMatches=1 should be fastest (minimal candidates)" << std::endl;
    std::cout << "  - maxMatches=10 should be moderate" << std::endl;
    std::cout << "  - maxMatches=0 should be slowest (full exploration)" << std::endl;

    return 0;
}
