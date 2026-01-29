/**
 * @file polar_transform_test.cpp
 * @brief Demo: Detect circle with Metrology, then apply polar transform
 *
 * Workflow:
 * 1. Detect circle using Metrology (caliper measurement)
 * 2. Use detected circle center for polar transform
 * 3. Display original and polar-transformed images
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Display/Draw.h>
#include <QiVision/Color/ColorConvert.h>
#include <QiVision/Measure/Metrology.h>
#include <QiVision/Transform/PolarTransform.h>
#include <QiVision/IO/ImageIO.h>
#include <QiVision/Platform/Timer.h>
#include <QiVision/GUI/Window.h>

#include <iostream>
#include <iomanip>

using namespace Qi::Vision;
using namespace Qi::Vision::Measure;
using namespace Qi::Vision::Transform;
using namespace Qi::Vision::IO;
using namespace Qi::Vision::Platform;
using namespace Qi::Vision::GUI;

int main() {
    // =========================================================================
    // Configuration - TODO: Update these parameters
    // =========================================================================
    std::string imagePath = "tests/data/matching/image3/202412121422747.bmp";

    // Circle initial estimate (TODO: update after viewing image)
    double circleRow = 488;      // Y coordinate of circle center
    double circleCol = 645;      // X coordinate of circle center
    double circleRadius = 220;   // Circle radius

    // Metrology parameters (adjusted for large circle)
    int numCalipers = 30;        // Number of calipers around the circle
    double measureLength1 = 30;  // Caliper length along radius
    double measureLength2 = 10;  // Caliper width perpendicular

    // =========================================================================
    // 1. Load image
    // =========================================================================
    QImage grayImage;
    ReadImageGray(imagePath, grayImage);
    if (grayImage.Empty()) {
        std::cerr << "Failed to load: " << imagePath << std::endl;
        return 1;
    }

    std::cout << "=== Polar Transform Demo ===" << std::endl;
    std::cout << "Image: " << grayImage.Width() << " x " << grayImage.Height() << std::endl;

    // =========================================================================
    // 2. Detect circle using Metrology
    // =========================================================================
    MetrologyModel model;

    MetrologyMeasureParams params;
    params.SetNumMeasures(numCalipers)
          .SetThreshold("auto")
          .SetMeasureSigma(1.0);

    model.AddCircleMeasure(
        circleRow, circleCol, circleRadius,
        measureLength1, measureLength2,
        "all", "all", params
    );

    Timer timer;
    timer.Start();
    model.Apply(grayImage);
    double measureTime = timer.ElapsedMs();

    // Get result
    auto result = model.GetCircleResult(0);

    std::cout << "\n=== Circle Detection ===" << std::endl;
    std::cout << "Initial:  center=(" << circleCol << ", " << circleRow
              << ") r=" << circleRadius << std::endl;

    if (result.numUsed < 5) {
        std::cerr << "Circle detection failed: only " << result.numUsed
                  << " points found" << std::endl;
        std::cerr << "Please adjust circle parameters (row, col, radius)" << std::endl;
        return 1;
    }

    Point2d center(result.column, result.row);
    double radius = result.radius;

    std::cout << "Fitted:   center=(" << std::fixed << std::setprecision(2)
              << center.x << ", " << center.y << ") r=" << radius << std::endl;
    std::cout << "Points:   " << result.numUsed << "/" << numCalipers << std::endl;
    std::cout << "RMS:      " << std::setprecision(3) << result.rmsError << " px" << std::endl;
    std::cout << "Time:     " << std::setprecision(1) << measureTime << " ms" << std::endl;

    // =========================================================================
    // 3. Apply polar transform
    // =========================================================================
    double maxRadius = radius;  // Only the circle area (no margin)

    timer.Start();
    QImage polarImage;
    CartesianToPolar(
        grayImage, polarImage,
        center, maxRadius,
        0, 0,  // auto size
        PolarMode::Linear,
        PolarInterpolation::Bilinear
    );
    double polarTime = timer.ElapsedMs();

    std::cout << "\n=== Polar Transform ===" << std::endl;
    std::cout << "Center:   (" << center.x << ", " << center.y << ")" << std::endl;
    std::cout << "MaxR:     " << maxRadius << " px" << std::endl;
    std::cout << "Output:   " << polarImage.Width() << " x " << polarImage.Height() << std::endl;
    std::cout << "Time:     " << polarTime << " ms" << std::endl;

    // =========================================================================
    // 4. Inverse transform to verify
    // =========================================================================
    timer.Start();
    QImage reconstructed;
    int32_t reconSize = static_cast<int32_t>(maxRadius * 2);
    // For inverse transform, center is relative to output image (center of output)
    Point2d reconCenter(maxRadius, maxRadius);
    PolarToCartesian(
        polarImage, reconstructed,
        reconCenter, maxRadius,
        reconSize, reconSize,
        PolarMode::Linear,
        PolarInterpolation::Bilinear
    );
    double invTime = timer.ElapsedMs();

    std::cout << "\n=== Inverse Transform ===" << std::endl;
    std::cout << "Output:   " << reconstructed.Width() << " x " << reconstructed.Height() << std::endl;
    std::cout << "Time:     " << invTime << " ms" << std::endl;

    // =========================================================================
    // 5. Visualize
    // =========================================================================
    // Draw detection on original
    QImage colorImg;
    Color::GrayToRgb(grayImage, colorImg);
    Draw::MetrologyModelResult(colorImg, model);
    Draw::Circle(colorImg, static_cast<int>(center.x), static_cast<int>(center.y),
                 static_cast<int>(radius), Scalar(0, 255, 0), 2);
    Draw::Cross(colorImg, static_cast<int>(center.x), static_cast<int>(center.y),
                15, Scalar(255, 0, 0), 2);

    // Convert to color for display
    QImage polarColor, reconColor;
    Color::GrayToRgb(polarImage, polarColor);
    Color::GrayToRgb(reconstructed, reconColor);

    // Debug: print image info
    std::cout << "\n=== Image Debug Info ===" << std::endl;
    std::cout << "colorImg:    " << colorImg.Width() << "x" << colorImg.Height()
              << " ch=" << colorImg.Channels() << " stride=" << colorImg.Stride() << std::endl;
    std::cout << "polarColor:  " << polarColor.Width() << "x" << polarColor.Height()
              << " ch=" << polarColor.Channels() << " stride=" << polarColor.Stride() << std::endl;
    std::cout << "reconColor:  " << reconColor.Width() << "x" << reconColor.Height()
              << " ch=" << reconColor.Channels() << " stride=" << reconColor.Stride() << std::endl;

    // Save results
    WriteImage(colorImg, "tests/output/polar_original.png");
    WriteImage(polarColor, "tests/output/polar_transformed.png");
    WriteImage(reconColor, "tests/output/polar_reconstructed.png");

    std::cout << "\n=== Results saved ===" << std::endl;
    std::cout << "  tests/output/polar_original.png" << std::endl;
    std::cout << "  tests/output/polar_transformed.png" << std::endl;
    std::cout << "  tests/output/polar_reconstructed.png" << std::endl;

    // =========================================================================
    // 6. Display
    // =========================================================================
    Window winOrig("Original + Circle Detection");
    winOrig.SetAutoResize(true, 800, 600);
    winOrig.EnablePixelInfo(true);
    winOrig.DispImage(colorImg);

    Window winPolar("Polar (X=angle, Y=radius)");
    winPolar.SetAutoResize(true, 800, 300);
    winPolar.EnablePixelInfo(true);
    winPolar.DispImage(polarColor);

    Window winRecon("Reconstructed");
    winRecon.SetAutoResize(true, 400, 400);
    winRecon.EnablePixelInfo(true);
    winRecon.DispImage(reconColor);

    std::cout << "\nPress any key to close..." << std::endl;
    winOrig.WaitKey(0);

    return 0;
}
