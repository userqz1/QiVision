/**
 * @file blob_analysis.cpp
 * @brief Blob analysis demonstration
 *
 * Demonstrates:
 * - Binary thresholding
 * - Connected component analysis (Connection)
 * - Shape feature extraction (Area, Circularity, etc.)
 * - Shape-based selection (SelectShape)
 * - Region sorting
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/QRegion.h>
#include <QiVision/IO/ImageIO.h>
#include <QiVision/Blob/Blob.h>
#include <QiVision/Segment/Segment.h>
#include <QiVision/Color/ColorConvert.h>
#include <QiVision/Display/Draw.h>
#include <QiVision/GUI/Window.h>

#include <iostream>
#include <iomanip>
#include <string>

using namespace Qi::Vision;

void PrintRegionFeatures(const QRegion& region, int index) {
    int64_t area;
    double row, col;
    Blob::AreaCenter(region, area, row, col);

    double circularity = Blob::Circularity(region);
    double rectangularity = Blob::Rectangularity(region);

    double ra, rb, phi;
    Blob::EllipticAxis(region, ra, rb, phi);

    std::cout << "  [" << index << "] Area=" << area
              << " Center=(" << std::fixed << std::setprecision(1)
              << row << "," << col << ")"
              << " Circ=" << std::setprecision(3) << circularity
              << " Rect=" << rectangularity
              << " Ra/Rb=" << std::setprecision(1) << ra << "/" << rb
              << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "=== QiVision Blob Analysis Demo ===\n\n";

    // Default test image path
    std::string imagePath = "tests/data/halcon_images/rings/rings_01.png";
    if (argc > 1) {
        imagePath = argv[1];
    }

    // Load image
    QImage image;
    try {
        IO::ReadImage(imagePath, image);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load image: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "Loaded: " << imagePath << " (" << image.Width() << "x" << image.Height() << ")\n\n";

    // Convert to grayscale for processing
    QImage gray;
    if (image.Channels() == 3 || image.Channels() == 4) {
        Color::Rgb1ToGray(image, gray);
    } else {
        gray = image;
    }

    // Create window for display
    GUI::Window window("Blob Analysis", image.Width(), image.Height());
    window.EnablePixelInfo(true);

    // Binary threshold
    double lowThresh = 0.0;
    double highThresh = 128.0;

    std::cout << "Controls:\n";
    std::cout << "  Q/A: Adjust low threshold (+/-10)\n";
    std::cout << "  W/S: Adjust high threshold (+/-10)\n";
    std::cout << "  R: Reset thresholds\n";
    std::cout << "  P: Print detailed features\n";
    std::cout << "  ESC: Exit\n\n";

    bool running = true;
    while (running) {
        // Threshold to get binary region
        QRegion binaryRegion = Segment::ThresholdToRegion(gray, lowThresh, highThresh);

        // Find connected components
        std::vector<QRegion> blobs;
        Blob::Connection(binaryRegion, blobs);

        // Filter by area (remove noise)
        std::vector<QRegion> filteredBlobs;
        Blob::SelectShapeArea(blobs, filteredBlobs, 100, 1000000);

        // Sort by area (largest first)
        std::vector<QRegion> sortedBlobs;
        Blob::SortRegion(filteredBlobs, sortedBlobs, Blob::SortMode::Area, false);

        // Select circular blobs (Ra/Rb close to 1)
        std::vector<QRegion> circularBlobs;
        for (const auto& blob : sortedBlobs) {
            double ra, rb, phi;
            Blob::EllipticAxis(blob, ra, rb, phi);
            double ratio = (rb > 0) ? ra / rb : 999;
            if (ratio >= 0.8 && ratio <= 1.25) {  // Nearly circular
                circularBlobs.push_back(blob);
            }
        }

        // Print results
        std::cout << "\r" << std::string(80, ' ') << "\r";
        std::cout << "Threshold: [" << static_cast<int>(lowThresh) << ", " << static_cast<int>(highThresh) << "]"
                  << " | Blobs: " << blobs.size()
                  << " | Filtered: " << sortedBlobs.size()
                  << " | Circular: " << circularBlobs.size() << std::flush;

        // Create display image (convert to RGB for color drawing)
        QImage display;
        if (image.Channels() == 1) {
            Color::GrayToRgb(image, display);
        } else {
            display = image.Clone();
        }

        // Draw all filtered blobs with semi-transparent green overlay
        for (const auto& blob : sortedBlobs) {
            Draw::RegionAlpha(display, blob, Scalar(0, 200, 0), 0.5);
        }

        // Draw blob contours in yellow
        for (const auto& blob : sortedBlobs) {
            Draw::RegionContour(display, blob, Scalar(255, 255, 0), 1);
        }

        // Draw circular blobs with outer circle, inner circle (if has hole), and center
        for (const auto& blob : circularBlobs) {
            // Get blob center
            int64_t area;
            double row, col;
            Blob::AreaCenter(blob, area, row, col);

            // Get outer circle (smallest enclosing circle)
            double outerRow, outerCol, outerRadius;
            Blob::SmallestCircle(blob, outerRow, outerCol, outerRadius);

            // Draw outer circle in red
            Draw::Circle(display, {outerCol, outerRow}, outerRadius, Scalar(255, 0, 0), 2);

            // Check for holes (ring-shaped blob)
            std::vector<QRegion> holes;
            Blob::GetHoles(blob, holes);

            if (!holes.empty()) {
                // For each hole, draw inner circle in blue
                for (const auto& hole : holes) {
                    double holeRow, holeCol, holeRadius;
                    Blob::SmallestCircle(hole, holeRow, holeCol, holeRadius);

                    // Draw inner circle in blue
                    Draw::Circle(display, {holeCol, holeRow}, holeRadius, Scalar(0, 100, 255), 2);

                    // Draw hole center in magenta
                    Draw::Cross(display, {holeCol, holeRow}, 8, 0, Scalar(255, 0, 255), 1);
                }
            }

            // Draw blob center cross in cyan
            Draw::Cross(display, {col, row}, 15, 0, Scalar(0, 255, 255), 2);
        }

        window.DispImage(display);

        // Handle input
        int key = window.WaitKey(50);
        switch (key) {
            case 'q': case 'Q':
                lowThresh = std::min(255.0, lowThresh + 10.0);
                break;
            case 'a': case 'A':
                lowThresh = std::max(0.0, lowThresh - 10.0);
                break;
            case 'w': case 'W':
                highThresh = std::min(255.0, highThresh + 10.0);
                break;
            case 's': case 'S':
                highThresh = std::max(0.0, highThresh - 10.0);
                break;
            case 'r': case 'R':
                lowThresh = 0.0;
                highThresh = 128.0;
                break;
            case 'p': case 'P':
                // Print detailed features
                std::cout << "\n\nDetailed features for " << sortedBlobs.size() << " blobs:\n";
                for (size_t i = 0; i < sortedBlobs.size() && i < 10; ++i) {
                    PrintRegionFeatures(sortedBlobs[i], static_cast<int>(i + 1));
                }
                std::cout << std::endl;
                break;
            case 27:  // ESC
            case 'x': case 'X':
                running = false;
                break;
            case -1:
                if (!window.IsOpen()) {
                    running = false;
                }
                break;
        }
    }

    // Final summary
    std::cout << "\n\n=== Final Analysis ===\n";

    QRegion binaryRegion = Segment::ThresholdToRegion(gray, lowThresh, highThresh);
    std::vector<QRegion> blobs;
    Blob::Connection(binaryRegion, blobs);

    std::vector<QRegion> filteredBlobs;
    Blob::SelectShapeArea(blobs, filteredBlobs, 100, 1000000);

    std::vector<QRegion> sortedBlobs;
    Blob::SortRegion(filteredBlobs, sortedBlobs, Blob::SortMode::Area, false);

    std::cout << "Total blobs found: " << blobs.size() << "\n";
    std::cout << "After area filter: " << sortedBlobs.size() << "\n\n";

    std::cout << "Top 10 blobs by area:\n";
    for (size_t i = 0; i < sortedBlobs.size() && i < 10; ++i) {
        PrintRegionFeatures(sortedBlobs[i], static_cast<int>(i + 1));
    }

    return 0;
}
