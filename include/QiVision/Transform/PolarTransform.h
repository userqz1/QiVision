#pragma once

/**
 * @file PolarTransform.h
 * @brief Public API for polar coordinate transformation
 *
 * Provides:
 * - Cartesian to Polar image transformation
 * - Polar to Cartesian image transformation (inverse)
 * - Linear and Semi-Log mapping modes
 *
 * Use cases:
 * - Circular object inspection (defects become horizontal lines)
 * - Ring-shaped ROI analysis
 * - Rotation-invariant feature extraction
 */

#include <QiVision/Core/Types.h>
#include <QiVision/Core/QImage.h>

namespace Qi::Vision::Transform {

/**
 * @brief Polar transformation mapping mode
 */
enum class PolarMode {
    Linear,     ///< Linear radial mapping: output_y proportional to radius
    SemiLog     ///< Semi-log radial mapping: enhanced detail near center
};

/**
 * @brief Interpolation method for transformation
 */
enum class PolarInterpolation {
    Nearest,    ///< Nearest neighbor (fast, pixelated)
    Bilinear,   ///< Bilinear interpolation (default, good quality)
    Bicubic     ///< Bicubic interpolation (best quality, slower)
};

/**
 * @brief Transform image from Cartesian to Polar coordinates
 *
 * Maps a circular region centered at 'center' to a rectangular image where:
 * - X axis = angle [0, 2*pi)
 * - Y axis = radius [0, maxRadius]
 *
 * This is useful for:
 * - Inspecting circular objects (defects become horizontal lines)
 * - Analyzing ring-shaped patterns
 * - Rotation-invariant processing
 *
 * @param src Source image (grayscale)
 * @param dst Output polar image
 * @param center Center point of polar transformation
 * @param maxRadius Maximum radius to include
 * @param dstWidth Output width (angle resolution). 0 = auto
 * @param dstHeight Output height (radial resolution). 0 = auto
 * @param mode Mapping mode (Linear or SemiLog)
 * @param interp Interpolation method
 */
void CartesianToPolar(
    const QImage& src,
    QImage& dst,
    const Point2d& center,
    double maxRadius,
    int32_t dstWidth = 0,
    int32_t dstHeight = 0,
    PolarMode mode = PolarMode::Linear,
    PolarInterpolation interp = PolarInterpolation::Bilinear
);

/**
 * @brief Transform image from Polar back to Cartesian coordinates
 *
 * Inverse of CartesianToPolar. Maps a polar image back to Cartesian space.
 *
 * @param src Source polar image
 * @param dst Output Cartesian image
 * @param center Center point for output image
 * @param maxRadius Maximum radius that was used in forward transform
 * @param dstWidth Output width. 0 = 2 * maxRadius
 * @param dstHeight Output height. 0 = 2 * maxRadius
 * @param mode Mapping mode (must match forward transform)
 * @param interp Interpolation method
 */
void PolarToCartesian(
    const QImage& src,
    QImage& dst,
    const Point2d& center,
    double maxRadius,
    int32_t dstWidth = 0,
    int32_t dstHeight = 0,
    PolarMode mode = PolarMode::Linear,
    PolarInterpolation interp = PolarInterpolation::Bilinear
);

/**
 * @brief General polar warp function
 *
 * Combined interface for both forward and inverse transforms.
 *
 * @param src Source image
 * @param dst Output image
 * @param center Center point for polar transformation
 * @param maxRadius Maximum radius
 * @param dstWidth Output width (0 = auto)
 * @param dstHeight Output height (0 = auto)
 * @param mode Mapping mode
 * @param inverse If true, transform Polar->Cartesian; if false, Cartesian->Polar
 * @param interp Interpolation method
 */
void WarpPolar(
    const QImage& src,
    QImage& dst,
    const Point2d& center,
    double maxRadius,
    int32_t dstWidth = 0,
    int32_t dstHeight = 0,
    PolarMode mode = PolarMode::Linear,
    bool inverse = false,
    PolarInterpolation interp = PolarInterpolation::Bilinear
);

/**
 * @brief Convert a single point from Cartesian to Polar coordinates
 *
 * @param pt Point in Cartesian coordinates
 * @param center Center of polar system
 * @return Point where x = angle (radians, [0, 2*pi)), y = radius
 */
Point2d PointCartesianToPolar(const Point2d& pt, const Point2d& center);

/**
 * @brief Convert a single point from Polar to Cartesian coordinates
 *
 * @param angle Angle in radians
 * @param radius Radius
 * @param center Center of polar system
 * @return Point in Cartesian coordinates
 */
Point2d PointPolarToCartesian(double angle, double radius, const Point2d& center);

} // namespace Qi::Vision::Transform
