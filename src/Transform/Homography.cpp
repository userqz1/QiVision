/**
 * @file Homography.cpp
 * @brief Implementation of public homography (projective) transformation API
 */

#include <QiVision/Transform/Homography.h>
#include <QiVision/Internal/Homography.h>
#include <QiVision/Internal/Interpolate.h>

#include <algorithm>
#include <cctype>
#include <cstring>

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

// Convert HomMat3d to Internal::Homography
Internal::Homography ToInternalHomography(const HomMat3d& h) {
    return Internal::Homography(h.Data());
}

// Convert Internal::Homography to HomMat3d
HomMat3d FromInternalHomography(const Internal::Homography& h) {
    return HomMat3d(h.Data());
}

} // anonymous namespace

// =============================================================================
// HomMat3d Class Implementation
// =============================================================================

HomMat3d::HomMat3d() {
    std::memset(data_, 0, sizeof(data_));
    data_[0] = data_[4] = data_[8] = 1.0;  // Identity
}

HomMat3d::HomMat3d(double h00, double h01, double h02,
                   double h10, double h11, double h12,
                   double h20, double h21, double h22) {
    data_[0] = h00; data_[1] = h01; data_[2] = h02;
    data_[3] = h10; data_[4] = h11; data_[5] = h12;
    data_[6] = h20; data_[7] = h21; data_[8] = h22;
}

HomMat3d::HomMat3d(const double* data) {
    std::memcpy(data_, data, 9 * sizeof(double));
}

double& HomMat3d::operator()(int row, int col) {
    return data_[row * 3 + col];
}

double HomMat3d::operator()(int row, int col) const {
    return data_[row * 3 + col];
}

double* HomMat3d::Data() {
    return data_;
}

const double* HomMat3d::Data() const {
    return data_;
}

void HomMat3d::GetElements(double (&elements)[9]) const {
    std::memcpy(elements, data_, 9 * sizeof(double));
}

HomMat3d HomMat3d::Identity() {
    return HomMat3d();
}

HomMat3d HomMat3d::FromAffine(const QMatrix& affine) {
    return HomMat3d(
        affine.M00(), affine.M01(), affine.M02(),
        affine.M10(), affine.M11(), affine.M12(),
        0.0, 0.0, 1.0
    );
}

HomMat3d HomMat3d::operator*(const HomMat3d& other) const {
    HomMat3d result;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result.data_[i * 3 + j] = 0;
            for (int k = 0; k < 3; ++k) {
                result.data_[i * 3 + j] += data_[i * 3 + k] * other.data_[k * 3 + j];
            }
        }
    }
    return result;
}

double HomMat3d::Determinant() const {
    return data_[0] * (data_[4] * data_[8] - data_[5] * data_[7])
         - data_[1] * (data_[3] * data_[8] - data_[5] * data_[6])
         + data_[2] * (data_[3] * data_[7] - data_[4] * data_[6]);
}

bool HomMat3d::IsInvertible(double tolerance) const {
    return std::abs(Determinant()) > tolerance;
}

HomMat3d HomMat3d::Inverse() const {
    Internal::Homography internal = ToInternalHomography(*this);
    return FromInternalHomography(internal.Inverse());
}

HomMat3d HomMat3d::Normalized() const {
    Internal::Homography internal = ToInternalHomography(*this);
    return FromInternalHomography(internal.Normalized());
}

bool HomMat3d::IsAffine(double tolerance) const {
    return std::abs(data_[6]) < tolerance && std::abs(data_[7]) < tolerance;
}

QMatrix HomMat3d::ToAffine() const {
    if (!IsAffine()) {
        return QMatrix::Identity();
    }

    // Normalize by h22
    double scale = (std::abs(data_[8]) > 1e-10) ? 1.0 / data_[8] : 1.0;

    return QMatrix(
        data_[0] * scale, data_[1] * scale, data_[2] * scale,
        data_[3] * scale, data_[4] * scale, data_[5] * scale
    );
}

Point2d HomMat3d::Transform(const Point2d& p) const {
    return Transform(p.x, p.y);
}

Point2d HomMat3d::Transform(double x, double y) const {
    double w = data_[6] * x + data_[7] * y + data_[8];

    constexpr double EPSILON = 1e-10;
    if (std::abs(w) < EPSILON) {
        // Point at infinity
        return {std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity()};
    }

    double invW = 1.0 / w;
    return {
        (data_[0] * x + data_[1] * y + data_[2]) * invW,
        (data_[3] * x + data_[4] * y + data_[5]) * invW
    };
}

std::vector<Point2d> HomMat3d::Transform(const std::vector<Point2d>& points) const {
    std::vector<Point2d> result;
    result.reserve(points.size());
    for (const auto& p : points) {
        result.push_back(Transform(p));
    }
    return result;
}

// =============================================================================
// Image Transformation
// =============================================================================

void ProjectiveTransImage(
    const QImage& src,
    QImage& dst,
    const HomMat3d& homMat3d,
    const std::string& interpolation,
    const std::string& borderMode,
    double borderValue)
{
    Internal::Homography internal = ToInternalHomography(homMat3d);
    dst = Internal::WarpPerspective(
        src,
        internal,
        0, 0,  // Auto-calculate size
        ToInternalInterp(interpolation),
        ToInternalBorderMode(borderMode),
        borderValue
    );
}

void ProjectiveTransImage(
    const QImage& src,
    QImage& dst,
    const HomMat3d& homMat3d,
    int32_t dstWidth,
    int32_t dstHeight,
    const std::string& interpolation,
    const std::string& borderMode,
    double borderValue)
{
    Internal::Homography internal = ToInternalHomography(homMat3d);
    dst = Internal::WarpPerspective(
        src,
        internal,
        dstWidth, dstHeight,
        ToInternalInterp(interpolation),
        ToInternalBorderMode(borderMode),
        borderValue
    );
}

// =============================================================================
// Matrix Creation
// =============================================================================

HomMat3d ProjHomMat2dIdentity() {
    return HomMat3d::Identity();
}

HomMat3d HomMat2dToProjHomMat(const QMatrix& homMat2d) {
    return HomMat3d::FromAffine(homMat2d);
}

HomMat3d ProjHomMat2dCompose(const HomMat3d& homMat3d1, const HomMat3d& homMat3d2) {
    return homMat3d1 * homMat3d2;
}

HomMat3d ProjHomMat2dInvert(const HomMat3d& homMat3d) {
    return homMat3d.Inverse();
}

// =============================================================================
// Point Transformation
// =============================================================================

Point2d ProjectiveTransPoint2d(const HomMat3d& homMat3d, const Point2d& point) {
    return homMat3d.Transform(point);
}

void ProjectiveTransPoint2d(
    const HomMat3d& homMat3d,
    double py, double px,
    double& qy, double& qx)
{
    Point2d result = homMat3d.Transform(px, py);
    qx = result.x;
    qy = result.y;
}

std::vector<Point2d> ProjectiveTransPoint2d(
    const HomMat3d& homMat3d,
    const std::vector<Point2d>& points)
{
    return homMat3d.Transform(points);
}

// =============================================================================
// Homography Estimation
// =============================================================================

bool VectorToProjHomMat2d(
    const std::vector<Point2d>& srcPoints,
    const std::vector<Point2d>& dstPoints,
    HomMat3d& homMat3d)
{
    auto result = Internal::EstimateHomography(srcPoints, dstPoints);
    if (result) {
        homMat3d = FromInternalHomography(*result);
        return true;
    }
    return false;
}

bool HomVectorToProjHomMat2d(
    const std::array<Point2d, 4>& srcPoints,
    const std::array<Point2d, 4>& dstPoints,
    HomMat3d& homMat3d)
{
    auto result = Internal::Homography::From4Points(srcPoints, dstPoints);
    if (result) {
        homMat3d = FromInternalHomography(*result);
        return true;
    }
    return false;
}

bool ProjMatchPointsRansac(
    const std::vector<Point2d>& srcPoints,
    const std::vector<Point2d>& dstPoints,
    HomMat3d& homMat3d,
    double distanceThreshold,
    double confidence,
    int32_t maxIterations,
    std::vector<bool>* inlierMask)
{
    auto result = Internal::EstimateHomographyRANSAC(
        srcPoints, dstPoints,
        distanceThreshold, confidence, maxIterations,
        inlierMask
    );

    if (result) {
        homMat3d = FromInternalHomography(*result);
        return true;
    }
    return false;
}

// =============================================================================
// Utility Functions
// =============================================================================

bool RectifyQuadrilateral(
    const std::array<Point2d, 4>& quadPoints,
    double width,
    double height,
    HomMat3d& homMat3d)
{
    auto result = Internal::RectifyQuadrilateral(quadPoints, width, height);
    if (result) {
        homMat3d = FromInternalHomography(*result);
        return true;
    }
    return false;
}

bool RectangleToQuadrilateral(
    double width,
    double height,
    const std::array<Point2d, 4>& quadPoints,
    HomMat3d& homMat3d)
{
    auto result = Internal::RectangleToQuadrilateral(width, height, quadPoints);
    if (result) {
        homMat3d = FromInternalHomography(*result);
        return true;
    }
    return false;
}

bool IsValidHomography(const HomMat3d& homMat3d, int32_t srcWidth, int32_t srcHeight) {
    Internal::Homography internal = ToInternalHomography(homMat3d);
    return Internal::IsValidHomography(internal, srcWidth, srcHeight);
}

double HomographyError(
    const std::vector<Point2d>& srcPoints,
    const std::vector<Point2d>& dstPoints,
    const HomMat3d& homMat3d)
{
    Internal::Homography internal = ToInternalHomography(homMat3d);
    return Internal::ComputeHomographyError(srcPoints, dstPoints, internal);
}

void RefineHomography(
    const std::vector<Point2d>& srcPoints,
    const std::vector<Point2d>& dstPoints,
    HomMat3d& homMat3d,
    int32_t maxIterations)
{
    Internal::Homography internal = ToInternalHomography(homMat3d);
    Internal::Homography refined = Internal::RefineHomographyLM(
        srcPoints, dstPoints, internal, maxIterations
    );
    homMat3d = FromInternalHomography(refined);
}

} // namespace Qi::Vision::Transform
