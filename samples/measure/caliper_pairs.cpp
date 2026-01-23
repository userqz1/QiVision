/**
 * @file caliper_pairs.cpp
 * @brief Caliper edge pairs demo - measure width between two edges
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Display/Draw.h>
#include <QiVision/Color/ColorConvert.h>
#include <QiVision/Measure/Caliper.h>
#include <QiVision/Measure/MeasureHandle.h>
#include <QiVision/GUI/Window.h>

#include <iostream>
#include <iomanip>

using namespace Qi::Vision;
using namespace Qi::Vision::Measure;
using namespace Qi::Vision::GUI;

int main(int argc, char* argv[]) {
    std::string imagePath = "tests/data/halcon_images/circle_plate.png";
    if (argc > 1) {
        imagePath = argv[1];
    }

    QImage image = QImage::FromFile(imagePath);
    if (!image.IsValid()) {
        std::cerr << "Failed to load: " << imagePath << std::endl;
        return 1;
    }
    QImage gray = image.ToGray();

    std::cout << "=== Caliper Edge Pairs Demo ===" << std::endl;
    std::cout << "Image: " << image.Width() << " x " << image.Height() << std::endl;

    // Create caliper handle - horizontal across first circle
    // Circle center approx (col=210, row=420), radius approx 63
    // Caliper position: circle center
    // Caliper direction: phi=0 (edge direction vertical, projection direction horizontal)
    // length1=100 (projection direction, horizontal), length2=10 (edge direction, vertical)
    double centerRow = 420;
    double centerCol = 210;
    double phi = 0;           // Edge direction = vertical
    double length1 = 100;     // Projection length (horizontal)
    double length2 = 10;      // Edge length (vertical)

    MeasureRectangle2 handle = GenMeasureRectangle2(centerRow, centerCol, phi, length1, length2);

    std::cout << "\nCaliper position:" << std::endl;
    std::cout << "  Center: (" << centerCol << ", " << centerRow << ")" << std::endl;
    std::cout << "  Phi: " << phi << " rad (edge direction = vertical)" << std::endl;
    std::cout << "  Size: " << length1 << " x " << length2 << " (projection x edge)" << std::endl;

    // Edge pair parameters (Halcon style):
    // First edge: dark->bright (positive) - entering circle
    // Second edge: bright->dark (negative) - leaving circle
    // transition="positive" means: first edge positive, second edge negative
    double sigma = 1.0;
    double threshold = 20.0;
    std::string transition = "positive";  // First edge: dark->bright
    std::string selectMode = "first";     // Select first pair

    std::cout << "\nPair parameters:" << std::endl;
    std::cout << "  Sigma: " << sigma << std::endl;
    std::cout << "  Threshold: " << threshold << std::endl;
    std::cout << "  Transition: " << transition << " (first edge dark->bright)" << std::endl;
    std::cout << "  Select: " << selectMode << std::endl;

    // Measure edge pairs
    auto pairs = MeasurePairs(gray, handle, sigma, threshold, transition, selectMode);

    std::cout << "\n=== Results ===" << std::endl;
    std::cout << "Found " << pairs.size() << " edge pair(s)" << std::endl;

    for (size_t i = 0; i < pairs.size(); ++i) {
        const auto& p = pairs[i];
        std::cout << "\nPair " << (i+1) << ":" << std::endl;
        std::cout << "  First edge:  (" << std::fixed << std::setprecision(2)
                  << p.first.column << ", " << p.first.row << ") amp=" << p.first.amplitude << std::endl;
        std::cout << "  Second edge: (" << p.second.column << ", " << p.second.row << ") amp=" << p.second.amplitude << std::endl;
        std::cout << "  Width: " << std::setprecision(3) << p.width << " px" << std::endl;
        std::cout << "  Center: (" << p.centerColumn << ", " << p.centerRow << ")" << std::endl;
    }

    // Visualization colors
    Scalar cyan(0, 255, 255);    // Caliper tool
    Scalar green(0, 255, 0);     // Edge points (inliers)

    QImage colorImg;
    Color::GrayToRgb(gray, colorImg);

    // Draw caliper tool (rectangle + projection line) - cyan
    Draw::MeasureRect(colorImg, handle, cyan, 1);

    // Draw edge points and center - green
    for (const auto& p : pairs) {
        Point2d pt1{p.first.column, p.first.row};
        Point2d pt2{p.second.column, p.second.row};
        Point2d center{p.centerColumn, p.centerRow};

        // Edge points
        Draw::Cross(colorImg, pt1, 8, 0, green, 2);
        Draw::Cross(colorImg, pt2, 8, 0, green, 2);

        // Pair center
        Draw::Cross(colorImg, center, 8, 0, green, 2);
    }

    // Display
    Window win("Caliper Pairs Demo");
    win.SetAutoResize(true);
    win.EnablePixelInfo(true);
    win.DispImage(colorImg);

    std::cout << "\n[Visualization]" << std::endl;
    std::cout << "  Cyan: Caliper tool + projection line" << std::endl;
    std::cout << "  Green: Edge points + center" << std::endl;
    std::cout << "\nPress any key to close..." << std::endl;

    win.WaitKey(0);

    return 0;
}
