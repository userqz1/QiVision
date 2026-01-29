/**
 * @file PolarTransform.cpp
 * @brief Implementation of public polar transformation API
 */

#include <QiVision/Transform/PolarTransform.h>
#include <QiVision/Internal/PolarTransform.h>

namespace Qi::Vision::Transform {

namespace {

// Convert public enum to internal enum
Internal::PolarMode ToInternalMode(PolarMode mode) {
    return mode == PolarMode::SemiLog ? Internal::PolarMode::SemiLog : Internal::PolarMode::Linear;
}

Internal::InterpolationMethod ToInternalInterp(PolarInterpolation interp) {
    switch (interp) {
        case PolarInterpolation::Nearest:
            return Internal::InterpolationMethod::Nearest;
        case PolarInterpolation::Bicubic:
            return Internal::InterpolationMethod::Bicubic;
        case PolarInterpolation::Bilinear:
        default:
            return Internal::InterpolationMethod::Bilinear;
    }
}

} // anonymous namespace

void CartesianToPolar(
    const QImage& src,
    QImage& dst,
    const Point2d& center,
    double maxRadius,
    int32_t dstWidth,
    int32_t dstHeight,
    PolarMode mode,
    PolarInterpolation interp)
{
    dst = Internal::WarpPolar(
        src, center, maxRadius,
        dstWidth, dstHeight,
        ToInternalMode(mode),
        false,  // forward transform
        ToInternalInterp(interp),
        Internal::BorderMode::Constant,
        0.0
    );
}

void PolarToCartesian(
    const QImage& src,
    QImage& dst,
    const Point2d& center,
    double maxRadius,
    int32_t dstWidth,
    int32_t dstHeight,
    PolarMode mode,
    PolarInterpolation interp)
{
    // Default output size = 2 * maxRadius
    if (dstWidth == 0) dstWidth = static_cast<int32_t>(maxRadius * 2);
    if (dstHeight == 0) dstHeight = static_cast<int32_t>(maxRadius * 2);

    dst = Internal::WarpPolar(
        src, center, maxRadius,
        dstWidth, dstHeight,
        ToInternalMode(mode),
        true,  // inverse transform
        ToInternalInterp(interp),
        Internal::BorderMode::Constant,
        0.0
    );
}

void WarpPolar(
    const QImage& src,
    QImage& dst,
    const Point2d& center,
    double maxRadius,
    int32_t dstWidth,
    int32_t dstHeight,
    PolarMode mode,
    bool inverse,
    PolarInterpolation interp)
{
    if (inverse) {
        PolarToCartesian(src, dst, center, maxRadius, dstWidth, dstHeight, mode, interp);
    } else {
        CartesianToPolar(src, dst, center, maxRadius, dstWidth, dstHeight, mode, interp);
    }
}

Point2d PointCartesianToPolar(const Point2d& pt, const Point2d& center) {
    return Internal::CartesianToPolar(pt, center);
}

Point2d PointPolarToCartesian(double angle, double radius, const Point2d& center) {
    return Internal::PolarToCartesian(radius, angle, center);
}

} // namespace Qi::Vision::Transform
