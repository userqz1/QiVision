#pragma once

/**
 * @file Edge.h
 * @brief Edge detection operations (Halcon-style API)
 *
 * API Style: void Func(const QImage& in, output..., params...)
 *
 * Halcon reference operators:
 * - edges_image, edges_sub_pix (Canny edge detection)
 * - lines_gauss, lines_facet (Steger subpixel line detection)
 *
 * This module provides public API for edge detection, wrapping
 * Internal layer implementations (Canny, Steger).
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/QContour.h>
#include <QiVision/Core/QContourArray.h>

#include <cstdint>
#include <string>
#include <vector>

namespace Qi::Vision::Edge {

// =============================================================================
// Canny Edge Detection
// =============================================================================

/**
 * @brief Detect edges using Canny algorithm (binary output)
 *
 * Equivalent to Halcon's edges_image operator.
 * Produces a binary edge image where edges are marked with 255.
 *
 * @param image Input grayscale image
 * @param edges Output binary edge image (255 = edge, 0 = non-edge)
 * @param filter Edge filter type: "canny", "canny_sobel", "canny_scharr"
 * @param sigma Gaussian smoothing sigma (larger = less noise, less detail)
 * @param lowThreshold Low threshold for hysteresis (weak edges)
 * @param highThreshold High threshold for hysteresis (strong edges)
 *
 * @code
 * QImage edgeImage;
 * EdgesImage(image, edgeImage, "canny", 1.0, 20.0, 40.0);
 * @endcode
 *
 * @note If lowThreshold >= highThreshold, auto-threshold is used
 */
void EdgesImage(
    const QImage& image,
    QImage& edges,
    const std::string& filter = "canny",
    double sigma = 1.0,
    double lowThreshold = 20.0,
    double highThreshold = 40.0
);

/**
 * @brief Detect edges with subpixel accuracy (contour output)
 *
 * Equivalent to Halcon's edges_sub_pix operator.
 * Returns edge contours with subpixel-accurate positions.
 *
 * @param image Input grayscale image
 * @param contours Output edge contours (XLD format)
 * @param filter Edge filter type: "canny", "canny_sobel", "canny_scharr"
 * @param sigma Gaussian smoothing sigma
 * @param lowThreshold Low threshold for hysteresis
 * @param highThreshold High threshold for hysteresis
 *
 * @code
 * QContourArray contours;
 * EdgesSubPix(image, contours, "canny", 1.0, 20.0, 40.0);
 *
 * // Draw contours
 * for (const auto& contour : contours) {
 *     DispContour(displayImage, contour, DrawColor::Green());
 * }
 * @endcode
 *
 * @note Contours are sorted by length (longest first)
 */
void EdgesSubPix(
    const QImage& image,
    QContourArray& contours,
    const std::string& filter = "canny",
    double sigma = 1.0,
    double lowThreshold = 20.0,
    double highThreshold = 40.0
);

/**
 * @brief Detect edges with auto-computed thresholds
 *
 * Uses Otsu or median-based method to automatically compute thresholds.
 *
 * @param image Input grayscale image
 * @param contours Output edge contours
 * @param filter Edge filter type
 * @param sigma Gaussian smoothing sigma
 *
 * @code
 * QContourArray contours;
 * EdgesSubPixAuto(image, contours, "canny", 1.5);
 * @endcode
 */
void EdgesSubPixAuto(
    const QImage& image,
    QContourArray& contours,
    const std::string& filter = "canny",
    double sigma = 1.0
);

// =============================================================================
// Steger Subpixel Line Detection
// =============================================================================

/**
 * @brief Detect lines/edges with subpixel accuracy using Steger algorithm
 *
 * Equivalent to Halcon's lines_gauss operator.
 * Uses Hessian matrix eigenvalue analysis to detect curvilinear structures
 * (lines, ridges, valleys) with high subpixel accuracy.
 *
 * @param image Input grayscale image
 * @param contours Output line contours (XLD format)
 * @param sigma Gaussian sigma for smoothing (controls line width sensitivity)
 *              Typical: 1.0-2.0 for thin lines, 2.0-5.0 for wide lines
 * @param lowThreshold Low threshold for hysteresis (response strength)
 * @param highThreshold High threshold for hysteresis
 * @param lightDark Line polarity: "light" (bright lines on dark background),
 *                  "dark" (dark lines on bright background), "all" (both)
 *
 * @code
 * // Detect bright lines (e.g., laser lines)
 * QContourArray lines;
 * LinesSubPix(image, lines, 1.5, 3.0, 8.0, "light");
 *
 * // Detect dark lines (e.g., cracks)
 * QContourArray cracks;
 * LinesSubPix(image, cracks, 2.0, 5.0, 15.0, "dark");
 * @endcode
 *
 * @note Subpixel accuracy is typically <0.02 pixels under good conditions
 */
void LinesSubPix(
    const QImage& image,
    QContourArray& contours,
    double sigma = 1.5,
    double lowThreshold = 3.0,
    double highThreshold = 8.0,
    const std::string& lightDark = "light"
);

/**
 * @brief Detect lines with auto-computed thresholds
 *
 * @param image Input grayscale image
 * @param contours Output line contours
 * @param sigma Gaussian sigma
 * @param lightDark Line polarity
 */
void LinesSubPixAuto(
    const QImage& image,
    QContourArray& contours,
    double sigma = 1.5,
    const std::string& lightDark = "light"
);

// =============================================================================
// Advanced Options
// =============================================================================

/**
 * @brief Canny edge detection parameters
 */
struct CannyEdgeParams {
    double sigma = 1.0;             ///< Gaussian smoothing sigma
    double lowThreshold = 20.0;     ///< Low threshold for hysteresis
    double highThreshold = 40.0;    ///< High threshold for hysteresis
    bool autoThreshold = false;     ///< Auto-compute thresholds
    std::string gradientOp = "sobel"; ///< Gradient operator: "sobel", "scharr", "sobel5x5"
    bool subPixelRefinement = true; ///< Enable subpixel refinement
    double minContourLength = 5.0;  ///< Minimum contour length (pixels)
    int32_t minContourPoints = 3;   ///< Minimum points per contour

    /// Create with auto-threshold
    static CannyEdgeParams Auto(double sigma = 1.0) {
        CannyEdgeParams p;
        p.sigma = sigma;
        p.autoThreshold = true;
        return p;
    }

    /// Create with specified thresholds
    static CannyEdgeParams WithThresholds(double low, double high, double sigma = 1.0) {
        CannyEdgeParams p;
        p.sigma = sigma;
        p.lowThreshold = low;
        p.highThreshold = high;
        return p;
    }
};

/**
 * @brief Steger line detection parameters
 */
struct StegerLineParams {
    double sigma = 1.5;             ///< Gaussian sigma (line width sensitivity)
    double lowThreshold = 3.0;      ///< Low threshold for hysteresis
    double highThreshold = 8.0;     ///< High threshold for hysteresis
    std::string lineType = "light"; ///< Line type: "light", "dark", "all"
    double minLength = 5.0;         ///< Minimum contour length
    double maxGap = 2.0;            ///< Maximum gap for edge linking
    double maxAngleDiff = 0.5;      ///< Max angle difference for linking (radians)
    bool subPixelRefinement = true; ///< Enable subpixel refinement

    /// Create for detecting bright lines
    static StegerLineParams Light(double sigma = 1.5, double lowThr = 3.0, double highThr = 8.0) {
        StegerLineParams p;
        p.sigma = sigma;
        p.lowThreshold = lowThr;
        p.highThreshold = highThr;
        p.lineType = "light";
        return p;
    }

    /// Create for detecting dark lines
    static StegerLineParams Dark(double sigma = 1.5, double lowThr = 3.0, double highThr = 8.0) {
        StegerLineParams p;
        p.sigma = sigma;
        p.lowThreshold = lowThr;
        p.highThreshold = highThr;
        p.lineType = "dark";
        return p;
    }
};

/**
 * @brief Detect edges with full parameter control
 *
 * @param image Input grayscale image
 * @param contours Output edge contours
 * @param params Canny edge detection parameters
 */
void DetectEdges(
    const QImage& image,
    QContourArray& contours,
    const CannyEdgeParams& params
);

/**
 * @brief Detect lines with full parameter control
 *
 * @param image Input grayscale image
 * @param contours Output line contours
 * @param params Steger line detection parameters
 */
void DetectLines(
    const QImage& image,
    QContourArray& contours,
    const StegerLineParams& params
);

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Compute recommended sigma based on line width
 *
 * @param lineWidthPixels Expected line width in pixels
 * @return Recommended sigma value
 */
double ComputeSigmaForLineWidth(double lineWidthPixels);

/**
 * @brief Estimate thresholds from image gradient statistics
 *
 * @param image Input image
 * @param sigma Gaussian sigma
 * @param[out] lowThreshold Estimated low threshold
 * @param[out] highThreshold Estimated high threshold
 */
void EstimateThresholds(
    const QImage& image,
    double sigma,
    double& lowThreshold,
    double& highThreshold
);

} // namespace Qi::Vision::Edge
