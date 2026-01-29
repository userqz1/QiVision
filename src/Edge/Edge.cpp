/**
 * @file Edge.cpp
 * @brief Edge detection operations implementation
 *
 * Wraps Internal layer functions (Canny, Steger) with Halcon-style API
 */

#include <QiVision/Edge/Edge.h>
#include <QiVision/Core/Exception.h>
#include <QiVision/Internal/Canny.h>
#include <QiVision/Internal/Steger.h>
#include <QiVision/Internal/Gradient.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace Qi::Vision::Edge {

// =============================================================================
// Helper Functions
// =============================================================================

namespace {

Internal::CannyGradientOp ParseGradientOp(const std::string& filter) {
    std::string lower = filter;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    if (lower.find("scharr") != std::string::npos) {
        return Internal::CannyGradientOp::Scharr;
    } else if (lower.find("5x5") != std::string::npos || lower.find("sobel5") != std::string::npos) {
        return Internal::CannyGradientOp::Sobel5x5;
    }
    // Default: Sobel 3x3
    return Internal::CannyGradientOp::Sobel;
}

Internal::LineType ParseLineType(const std::string& lightDark) {
    std::string lower = lightDark;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    if (lower == "light" || lower == "bright" || lower == "ridge") {
        return Internal::LineType::Ridge;
    } else if (lower == "dark" || lower == "valley") {
        return Internal::LineType::Valley;
    } else if (lower == "all" || lower == "both") {
        return Internal::LineType::Both;
    }

    // Default: light lines (ridge)
    return Internal::LineType::Ridge;
}

Internal::CannyParams BuildCannyParams(const CannyEdgeParams& params) {
    Internal::CannyParams cp;
    cp.sigma = params.sigma;
    cp.lowThreshold = params.lowThreshold;
    cp.highThreshold = params.highThreshold;
    cp.autoThreshold = params.autoThreshold;
    cp.gradientOp = ParseGradientOp(params.gradientOp);
    cp.subpixelRefinement = params.subPixelRefinement;
    cp.minContourLength = params.minContourLength;
    cp.minContourPoints = params.minContourPoints;
    cp.linkEdges = true;
    return cp;
}

Internal::StegerParams BuildStegerParams(const StegerLineParams& params) {
    Internal::StegerParams sp;
    sp.sigma = params.sigma;
    sp.lowThreshold = params.lowThreshold;
    sp.highThreshold = params.highThreshold;
    sp.lineType = ParseLineType(params.lineType);
    sp.minLength = params.minLength;
    sp.maxGap = params.maxGap;
    sp.maxAngleDiff = params.maxAngleDiff;
    sp.subPixelRefinement = params.subPixelRefinement;
    return sp;
}

void ValidateGrayscaleInput(const QImage& image, const char* funcName) {
    if (image.Empty()) {
        throw InvalidArgumentException(std::string(funcName) + ": Input image is empty");
    }

    if (image.Type() != PixelType::UInt8) {
        throw UnsupportedException(std::string(funcName) + " only supports UInt8 images");
    }

    if (image.Channels() != 1) {
        throw UnsupportedException(std::string(funcName) + " requires single-channel grayscale image");
    }
}

} // anonymous namespace

// =============================================================================
// Canny Edge Detection
// =============================================================================

void EdgesImage(
    const QImage& image,
    QImage& edges,
    const std::string& filter,
    double sigma,
    double lowThreshold,
    double highThreshold
) {
    if (image.Empty()) {
        edges = QImage();
        return;
    }

    ValidateGrayscaleInput(image, "EdgesImage");

    // Build Canny parameters
    Internal::CannyParams params;
    params.sigma = sigma;
    params.lowThreshold = lowThreshold;
    params.highThreshold = highThreshold;
    params.autoThreshold = (lowThreshold >= highThreshold);
    params.gradientOp = ParseGradientOp(filter);
    params.subpixelRefinement = false; // No need for binary output
    params.linkEdges = false;           // No need for linking

    // Detect edges and get binary image
    edges = Internal::DetectEdgesCannyImage(image, params);
}

void EdgesSubPix(
    const QImage& image,
    QContourArray& contours,
    const std::string& filter,
    double sigma,
    double lowThreshold,
    double highThreshold
) {
    if (image.Empty()) {
        contours = QContourArray();
        return;
    }

    ValidateGrayscaleInput(image, "EdgesSubPix");

    // Build Canny parameters
    Internal::CannyParams params;
    params.sigma = sigma;
    params.lowThreshold = lowThreshold;
    params.highThreshold = highThreshold;
    params.autoThreshold = (lowThreshold >= highThreshold);
    params.gradientOp = ParseGradientOp(filter);
    params.subpixelRefinement = true;
    params.linkEdges = true;
    params.minContourLength = 5.0;
    params.minContourPoints = 3;

    // Detect edges
    std::vector<QContour> edgeContours = Internal::DetectEdgesCanny(image, params);

    // Sort by length (longest first)
    std::sort(edgeContours.begin(), edgeContours.end(),
              [](const QContour& a, const QContour& b) {
                  return a.Length() > b.Length();
              });

    // Convert to QContourArray
    contours = QContourArray(std::move(edgeContours));
}

void EdgesSubPixAuto(
    const QImage& image,
    QContourArray& contours,
    const std::string& filter,
    double sigma
) {
    // Use auto-threshold by passing low >= high
    EdgesSubPix(image, contours, filter, sigma, 0.0, 0.0);
}

// =============================================================================
// Steger Line Detection
// =============================================================================

void LinesSubPix(
    const QImage& image,
    QContourArray& contours,
    double sigma,
    double lowThreshold,
    double highThreshold,
    const std::string& lightDark
) {
    if (image.Empty()) {
        contours = QContourArray();
        return;
    }

    ValidateGrayscaleInput(image, "LinesSubPix");

    // Build Steger parameters
    Internal::StegerParams params;
    params.sigma = sigma;
    params.lowThreshold = lowThreshold;
    params.highThreshold = highThreshold;
    params.lineType = ParseLineType(lightDark);
    params.minLength = 5.0;
    params.maxGap = 2.0;
    params.maxAngleDiff = 0.5;
    params.subPixelRefinement = true;

    // Detect lines
    std::vector<QContour> lineContours = Internal::DetectStegerEdges(image, params);

    // Sort by length (longest first)
    std::sort(lineContours.begin(), lineContours.end(),
              [](const QContour& a, const QContour& b) {
                  return a.Length() > b.Length();
              });

    // Convert to QContourArray
    contours = QContourArray(std::move(lineContours));
}

void LinesSubPixAuto(
    const QImage& image,
    QContourArray& contours,
    double sigma,
    const std::string& lightDark
) {
    if (image.Empty()) {
        contours = QContourArray();
        return;
    }

    ValidateGrayscaleInput(image, "LinesSubPixAuto");

    // Estimate thresholds based on image statistics
    double lowThreshold, highThreshold;
    EstimateThresholds(image, sigma, lowThreshold, highThreshold);

    // Scale for Steger (Hessian-based thresholds are different from gradient)
    // Typical Steger thresholds are lower than Canny thresholds
    lowThreshold = std::max(2.0, lowThreshold * 0.15);
    highThreshold = std::max(5.0, highThreshold * 0.3);

    LinesSubPix(image, contours, sigma, lowThreshold, highThreshold, lightDark);
}

// =============================================================================
// Advanced Options
// =============================================================================

void DetectEdges(
    const QImage& image,
    QContourArray& contours,
    const CannyEdgeParams& params
) {
    if (image.Empty()) {
        contours = QContourArray();
        return;
    }

    ValidateGrayscaleInput(image, "DetectEdges");

    Internal::CannyParams internalParams = BuildCannyParams(params);

    std::vector<QContour> edgeContours = Internal::DetectEdgesCanny(image, internalParams);

    // Sort by length
    std::sort(edgeContours.begin(), edgeContours.end(),
              [](const QContour& a, const QContour& b) {
                  return a.Length() > b.Length();
              });

    contours = QContourArray(std::move(edgeContours));
}

void DetectLines(
    const QImage& image,
    QContourArray& contours,
    const StegerLineParams& params
) {
    if (image.Empty()) {
        contours = QContourArray();
        return;
    }

    ValidateGrayscaleInput(image, "DetectLines");

    Internal::StegerParams internalParams = BuildStegerParams(params);

    std::vector<QContour> lineContours = Internal::DetectStegerEdges(image, internalParams);

    // Sort by length
    std::sort(lineContours.begin(), lineContours.end(),
              [](const QContour& a, const QContour& b) {
                  return a.Length() > b.Length();
              });

    contours = QContourArray(std::move(lineContours));
}

// =============================================================================
// Utility Functions
// =============================================================================

double ComputeSigmaForLineWidth(double lineWidthPixels) {
    // Steger's rule of thumb: sigma ~ lineWidth / 2.5
    // For good detection, sigma should be approximately half the line width
    if (lineWidthPixels <= 0) {
        return 1.0;
    }
    return std::max(0.5, lineWidthPixels / 2.5);
}

void EstimateThresholds(
    const QImage& image,
    double sigma,
    double& lowThreshold,
    double& highThreshold
) {
    if (image.Empty() || image.Type() != PixelType::UInt8 || image.Channels() != 1) {
        lowThreshold = 20.0;
        highThreshold = 40.0;
        return;
    }

    int32_t w = image.Width();
    int32_t h = image.Height();

    // Compute gradient magnitude statistics
    // Use simple Sobel for estimation
    double sumMag = 0.0;
    double sumMagSq = 0.0;
    int64_t count = 0;

    for (int32_t y = 1; y < h - 1; ++y) {
        const uint8_t* row0 = static_cast<const uint8_t*>(image.RowPtr(y - 1));
        const uint8_t* row1 = static_cast<const uint8_t*>(image.RowPtr(y));
        const uint8_t* row2 = static_cast<const uint8_t*>(image.RowPtr(y + 1));

        for (int32_t x = 1; x < w - 1; ++x) {
            // Simple Sobel
            int gx = (row0[x + 1] - row0[x - 1]) +
                     2 * (row1[x + 1] - row1[x - 1]) +
                     (row2[x + 1] - row2[x - 1]);

            int gy = (row2[x - 1] - row0[x - 1]) +
                     2 * (row2[x] - row0[x]) +
                     (row2[x + 1] - row0[x + 1]);

            double mag = std::sqrt(gx * gx + gy * gy) / 4.0; // Normalize
            sumMag += mag;
            sumMagSq += mag * mag;
            count++;
        }
    }

    if (count == 0) {
        lowThreshold = 20.0;
        highThreshold = 40.0;
        return;
    }

    double meanMag = sumMag / count;
    double variance = (sumMagSq / count) - (meanMag * meanMag);
    double stdMag = std::sqrt(std::max(0.0, variance));

    // Heuristic: thresholds based on gradient statistics
    // Higher sigma -> lower effective gradients, adjust accordingly
    double sigmaFactor = std::max(1.0, sigma);

    // High threshold: mean + 1.5 * std (capture strong edges)
    highThreshold = std::max(10.0, (meanMag + 1.5 * stdMag) / sigmaFactor);

    // Low threshold: 0.4-0.5 of high threshold (hysteresis ratio)
    lowThreshold = std::max(5.0, highThreshold * 0.4);

    // Clamp to reasonable range
    lowThreshold = std::min(lowThreshold, 100.0);
    highThreshold = std::min(highThreshold, 200.0);
}

} // namespace Qi::Vision::Edge
