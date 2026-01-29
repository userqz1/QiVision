/**
 * @file AffineTransform.cpp
 * @brief Implementation of public affine transformation API
 */

#include <QiVision/Transform/AffineTransform.h>
#include <QiVision/Internal/AffineTransform.h>
#include <QiVision/Internal/Interpolate.h>

#include <algorithm>
#include <cctype>

namespace Qi::Vision::Transform {

namespace {

// Convert string to interpolation method
Internal::InterpolationMethod ToInternalInterp(const std::string& interp) {
    std::string lower = interp;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (lower == "nearest") {
        return Internal::InterpolationMethod::Nearest;
    } else if (lower == "bicubic") {
        return Internal::InterpolationMethod::Bicubic;
    }
    // Default to bilinear
    return Internal::InterpolationMethod::Bilinear;
}

// Convert string to border mode
Internal::BorderMode ToInternalBorderMode(const std::string& mode) {
    std::string lower = mode;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (lower == "replicate") {
        return Internal::BorderMode::Replicate;
    } else if (lower == "reflect" || lower == "reflect101") {
        return Internal::BorderMode::Reflect101;
    } else if (lower == "wrap") {
        return Internal::BorderMode::Wrap;
    }
    // Default to constant
    return Internal::BorderMode::Constant;
}

} // anonymous namespace

// =============================================================================
// Image Transformation
// =============================================================================

void AffineTransImage(
    const QImage& src,
    QImage& dst,
    const QMatrix& matrix,
    const std::string& interpolation,
    const std::string& borderMode,
    double borderValue)
{
    dst = Internal::WarpAffine(
        src,
        matrix,
        0, 0,  // Auto-calculate size
        ToInternalInterp(interpolation),
        ToInternalBorderMode(borderMode),
        borderValue
    );
}

void AffineTransImage(
    const QImage& src,
    QImage& dst,
    const QMatrix& matrix,
    int32_t dstWidth,
    int32_t dstHeight,
    const std::string& interpolation,
    const std::string& borderMode,
    double borderValue)
{
    dst = Internal::WarpAffine(
        src,
        matrix,
        dstWidth, dstHeight,
        ToInternalInterp(interpolation),
        ToInternalBorderMode(borderMode),
        borderValue
    );
}

void RotateImage(
    const QImage& src,
    QImage& dst,
    double angle,
    const std::string& interpolation)
{
    dst = Internal::RotateImage(
        src,
        angle,
        true,  // resize to fit
        ToInternalInterp(interpolation),
        Internal::BorderMode::Constant,
        0.0
    );
}

void RotateImage(
    const QImage& src,
    QImage& dst,
    double angle,
    double centerRow,
    double centerCol,
    const std::string& interpolation)
{
    dst = Internal::RotateImage(
        src,
        angle,
        centerCol,  // Internal uses x, y order
        centerRow,
        true,  // resize to fit
        ToInternalInterp(interpolation),
        Internal::BorderMode::Constant,
        0.0
    );
}

void ScaleImage(
    const QImage& src,
    QImage& dst,
    double scaleX,
    double scaleY,
    const std::string& interpolation)
{
    dst = Internal::ScaleImageFactor(
        src,
        scaleX,
        scaleY,
        ToInternalInterp(interpolation)
    );
}

void ZoomImageSize(
    const QImage& src,
    QImage& dst,
    int32_t dstWidth,
    int32_t dstHeight,
    const std::string& interpolation)
{
    dst = Internal::ScaleImage(
        src,
        dstWidth,
        dstHeight,
        ToInternalInterp(interpolation)
    );
}

// =============================================================================
// Matrix Creation
// =============================================================================

QMatrix HomMat2dIdentity() {
    return QMatrix::Identity();
}

QMatrix HomMat2dRotate(double phi, double cy, double cx) {
    return QMatrix::Rotation(phi, cx, cy);
}

QMatrix HomMat2dScale(double sy, double sx, double cy, double cx) {
    return QMatrix::Scaling(sx, sy, Point2d{cx, cy});
}

QMatrix HomMat2dTranslate(const QMatrix& homMat2d, double ty, double tx) {
    return QMatrix::Translation(tx, ty) * homMat2d;
}

QMatrix HomMat2dTranslateOnly(double ty, double tx) {
    return QMatrix::Translation(tx, ty);
}

QMatrix HomMat2dCompose(const QMatrix& homMat2d1, const QMatrix& homMat2d2) {
    return homMat2d1 * homMat2d2;
}

QMatrix HomMat2dInvert(const QMatrix& homMat2d) {
    return homMat2d.Inverse();
}

QMatrix HomMat2dRotateLocal(const QMatrix& homMat2d, double phi, double cy, double cx) {
    QMatrix rotation = QMatrix::Rotation(phi, cx, cy);
    return rotation * homMat2d;
}

QMatrix HomMat2dScaleLocal(const QMatrix& homMat2d, double sy, double sx, double cy, double cx) {
    QMatrix scaling = QMatrix::Scaling(sx, sy, Point2d{cx, cy});
    return scaling * homMat2d;
}

// =============================================================================
// Point Transformation
// =============================================================================

Point2d AffineTransPoint2d(const QMatrix& homMat2d, const Point2d& point) {
    return homMat2d.Transform(point);
}

void AffineTransPoint2d(
    const QMatrix& homMat2d,
    double py, double px,
    double& qy, double& qx)
{
    Point2d result = homMat2d.Transform(px, py);
    qx = result.x;
    qy = result.y;
}

std::vector<Point2d> AffineTransPoint2d(
    const QMatrix& homMat2d,
    const std::vector<Point2d>& points)
{
    std::vector<Point2d> result;
    result.reserve(points.size());
    for (const auto& p : points) {
        result.push_back(homMat2d.Transform(p));
    }
    return result;
}

void AffineTransPoint2d(
    const QMatrix& homMat2d,
    const std::vector<double>& py, const std::vector<double>& px,
    std::vector<double>& qy, std::vector<double>& qx)
{
    size_t n = std::min(py.size(), px.size());
    qy.resize(n);
    qx.resize(n);

    for (size_t i = 0; i < n; ++i) {
        Point2d result = homMat2d.Transform(px[i], py[i]);
        qx[i] = result.x;
        qy[i] = result.y;
    }
}

// =============================================================================
// Transform Estimation
// =============================================================================

bool VectorToHomMat2d(
    const std::vector<Point2d>& srcPoints,
    const std::vector<Point2d>& dstPoints,
    QMatrix& homMat2d)
{
    auto result = Internal::EstimateAffine(srcPoints, dstPoints);
    if (result) {
        homMat2d = *result;
        return true;
    }
    return false;
}

bool VectorToRigid(
    const std::vector<Point2d>& srcPoints,
    const std::vector<Point2d>& dstPoints,
    QMatrix& homMat2d)
{
    auto result = Internal::EstimateRigid(srcPoints, dstPoints);
    if (result) {
        homMat2d = *result;
        return true;
    }
    return false;
}

bool VectorToSimilarity(
    const std::vector<Point2d>& srcPoints,
    const std::vector<Point2d>& dstPoints,
    QMatrix& homMat2d)
{
    auto result = Internal::EstimateSimilarity(srcPoints, dstPoints);
    if (result) {
        homMat2d = *result;
        return true;
    }
    return false;
}

// =============================================================================
// Matrix Analysis
// =============================================================================

bool HomMat2dToAffinePar(
    const QMatrix& homMat2d,
    double& ty, double& tx,
    double& phi,
    double& sy, double& sx,
    double& theta)
{
    double shear = 0.0;
    bool success = Internal::DecomposeAffine(homMat2d, tx, ty, phi, sx, sy, shear);
    // Convert shear to angle
    theta = std::atan(shear);
    return success;
}

bool HomMat2dIsRigid(const QMatrix& homMat2d, double tolerance) {
    return Internal::IsRigidTransform(homMat2d, tolerance);
}

bool HomMat2dIsSimilarity(const QMatrix& homMat2d, double tolerance) {
    return Internal::IsSimilarityTransform(homMat2d, tolerance);
}

double HomMat2dDeterminant(const QMatrix& homMat2d) {
    return homMat2d.Determinant();
}

} // namespace Qi::Vision::Transform
