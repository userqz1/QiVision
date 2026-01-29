#pragma once

/**
 * @file Homography.h
 * @brief Public API for projective (homography) transformation operations
 *
 * Provides:
 * - Perspective image warping
 * - Homography matrix estimation and manipulation
 * - Point transformation with projective geometry
 *
 * Reference Halcon operators:
 * - projective_trans_image, projective_trans_point_2d
 * - vector_to_proj_hom_mat2d, proj_match_points_ransac
 * - hom_mat2d_to_proj_hom_mat, proj_hom_mat2d_compose
 */

#include <QiVision/Core/Types.h>
#include <QiVision/Core/QImage.h>
#include <QiVision/Core/QMatrix.h>

#include <string>
#include <vector>
#include <array>

namespace Qi::Vision::Transform {

// =============================================================================
// HomMat3d: 3x3 Homography Matrix
// =============================================================================

/**
 * @brief 3x3 Homography matrix for projective transformations
 *
 * Represents the projective transformation:
 *   [x']   [h00 h01 h02] [x]
 *   [y'] = [h10 h11 h12] [y]
 *   [w']   [h20 h21 h22] [1]
 *
 * Output coordinates: x_dst = x'/w', y_dst = y'/w'
 */
class HomMat3d {
public:
    /// Default constructor (identity matrix)
    HomMat3d();

    /// Construct from 9 elements (row-major)
    HomMat3d(double h00, double h01, double h02,
             double h10, double h11, double h12,
             double h20, double h21, double h22);

    /// Construct from array (row-major, 9 elements)
    explicit HomMat3d(const double* data);

    // =========================================================================
    // Element Access
    // =========================================================================

    /// Access element at (row, col)
    double& operator()(int row, int col);
    double operator()(int row, int col) const;

    /// Get raw data pointer
    double* Data();
    const double* Data() const;

    /// Get all 9 elements (row-major)
    void GetElements(double (&elements)[9]) const;

    // =========================================================================
    // Static Factory Methods
    // =========================================================================

    /// Identity homography
    static HomMat3d Identity();

    /// Create from affine matrix (embeds QMatrix into 3x3)
    static HomMat3d FromAffine(const QMatrix& affine);

    // =========================================================================
    // Matrix Operations
    // =========================================================================

    /// Compose with another homography: result = this * other
    HomMat3d operator*(const HomMat3d& other) const;

    /// Inverse homography
    HomMat3d Inverse() const;

    /// Check if invertible
    bool IsInvertible(double tolerance = 1e-10) const;

    /// Determinant
    double Determinant() const;

    /// Normalize (make h22 = 1 if nonzero)
    HomMat3d Normalized() const;

    // =========================================================================
    // Type Checking
    // =========================================================================

    /// Check if this is actually an affine transform (h20 = h21 = 0)
    bool IsAffine(double tolerance = 1e-10) const;

    /// Convert to QMatrix if affine (returns identity if not affine)
    QMatrix ToAffine() const;

    // =========================================================================
    // Point Transformation
    // =========================================================================

    /// Transform a single point
    Point2d Transform(const Point2d& p) const;
    Point2d Transform(double x, double y) const;

    /// Transform multiple points
    std::vector<Point2d> Transform(const std::vector<Point2d>& points) const;

private:
    double data_[9];  // Row-major storage
};

// =============================================================================
// Image Transformation
// =============================================================================

/**
 * @brief Apply projective (perspective) transformation to an image
 *
 * Transforms the source image using the given 3x3 homography matrix.
 * Output size is automatically calculated to fit the transformed image.
 *
 * @param src Source image (grayscale or multi-channel)
 * @param dst Output transformed image
 * @param homMat3d 3x3 homography matrix
 * @param interpolation Interpolation method: "nearest", "bilinear" (default), "bicubic"
 * @param borderMode Border handling: "constant" (default), "replicate", "reflect", "wrap"
 * @param borderValue Border value for constant mode
 */
void ProjectiveTransImage(
    const QImage& src,
    QImage& dst,
    const HomMat3d& homMat3d,
    const std::string& interpolation = "bilinear",
    const std::string& borderMode = "constant",
    double borderValue = 0.0
);

/**
 * @brief Apply projective transformation with specified output size
 *
 * @param src Source image
 * @param dst Output transformed image
 * @param homMat3d 3x3 homography matrix
 * @param dstWidth Output width
 * @param dstHeight Output height
 * @param interpolation Interpolation method
 * @param borderMode Border handling mode
 * @param borderValue Border value for constant mode
 */
void ProjectiveTransImage(
    const QImage& src,
    QImage& dst,
    const HomMat3d& homMat3d,
    int32_t dstWidth,
    int32_t dstHeight,
    const std::string& interpolation = "bilinear",
    const std::string& borderMode = "constant",
    double borderValue = 0.0
);

// =============================================================================
// Matrix Creation
// =============================================================================

/**
 * @brief Create identity homography matrix
 *
 * @return Identity HomMat3d
 */
HomMat3d ProjHomMat2dIdentity();

/**
 * @brief Convert affine matrix to projective matrix
 *
 * Embeds a 2D affine matrix into a 3x3 homography matrix.
 *
 * @param homMat2d 2D affine transformation matrix
 * @return HomMat3d with h20 = h21 = 0, h22 = 1
 */
HomMat3d HomMat2dToProjHomMat(const QMatrix& homMat2d);

/**
 * @brief Compose two homography matrices
 *
 * Computes H1 * H2, which applies H2 first, then H1.
 *
 * @param homMat3d1 First homography (applied second)
 * @param homMat3d2 Second homography (applied first)
 * @return Composed HomMat3d
 */
HomMat3d ProjHomMat2dCompose(const HomMat3d& homMat3d1, const HomMat3d& homMat3d2);

/**
 * @brief Invert homography matrix
 *
 * @param homMat3d Input homography matrix
 * @return Inverted HomMat3d (identity if not invertible)
 */
HomMat3d ProjHomMat2dInvert(const HomMat3d& homMat3d);

// =============================================================================
// Point Transformation
// =============================================================================

/**
 * @brief Transform a single point using projective matrix
 *
 * @param homMat3d Homography matrix
 * @param point Input point
 * @return Transformed point
 */
Point2d ProjectiveTransPoint2d(const HomMat3d& homMat3d, const Point2d& point);

/**
 * @brief Transform a single point using projective matrix (row/col version)
 *
 * @param homMat3d Homography matrix
 * @param py Input Y coordinate (row)
 * @param px Input X coordinate (column)
 * @param qy Output Y coordinate (row)
 * @param qx Output X coordinate (column)
 */
void ProjectiveTransPoint2d(
    const HomMat3d& homMat3d,
    double py, double px,
    double& qy, double& qx
);

/**
 * @brief Transform multiple points using projective matrix
 *
 * @param homMat3d Homography matrix
 * @param points Input points
 * @return Transformed points
 */
std::vector<Point2d> ProjectiveTransPoint2d(
    const HomMat3d& homMat3d,
    const std::vector<Point2d>& points
);

// =============================================================================
// Homography Estimation
// =============================================================================

/**
 * @brief Estimate homography from point correspondences
 *
 * Uses Direct Linear Transform (DLT) with point normalization.
 * Requires at least 4 point pairs.
 *
 * @param srcPoints Source points
 * @param dstPoints Destination points
 * @param homMat3d Output homography matrix
 * @return true if estimation succeeded
 */
bool VectorToProjHomMat2d(
    const std::vector<Point2d>& srcPoints,
    const std::vector<Point2d>& dstPoints,
    HomMat3d& homMat3d
);

/**
 * @brief Estimate homography from exactly 4 point correspondences
 *
 * @param srcPoints 4 source points
 * @param dstPoints 4 destination points
 * @param homMat3d Output homography matrix
 * @return true if estimation succeeded
 */
bool HomVectorToProjHomMat2d(
    const std::array<Point2d, 4>& srcPoints,
    const std::array<Point2d, 4>& dstPoints,
    HomMat3d& homMat3d
);

/**
 * @brief Estimate homography with RANSAC for outlier rejection
 *
 * Robust homography estimation that handles outlier correspondences.
 *
 * @param srcPoints Source points (at least 4)
 * @param dstPoints Destination points
 * @param homMat3d Output homography matrix
 * @param distanceThreshold Inlier distance threshold (pixels)
 * @param confidence RANSAC confidence level (0-1)
 * @param maxIterations Maximum RANSAC iterations
 * @param inlierMask Output inlier mask (optional)
 * @return true if estimation succeeded
 */
bool ProjMatchPointsRansac(
    const std::vector<Point2d>& srcPoints,
    const std::vector<Point2d>& dstPoints,
    HomMat3d& homMat3d,
    double distanceThreshold = 3.0,
    double confidence = 0.99,
    int32_t maxIterations = 2000,
    std::vector<bool>* inlierMask = nullptr
);

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Create homography that maps a quadrilateral to a rectangle
 *
 * Useful for rectifying perspective distortion.
 *
 * @param quadPoints 4 corners of quadrilateral (TL, TR, BR, BL order)
 * @param width Target rectangle width
 * @param height Target rectangle height
 * @param homMat3d Output homography matrix
 * @return true if estimation succeeded
 */
bool RectifyQuadrilateral(
    const std::array<Point2d, 4>& quadPoints,
    double width,
    double height,
    HomMat3d& homMat3d
);

/**
 * @brief Create homography that maps a rectangle to a quadrilateral
 *
 * @param width Source rectangle width
 * @param height Source rectangle height
 * @param quadPoints 4 corners of target quadrilateral (TL, TR, BR, BL order)
 * @param homMat3d Output homography matrix
 * @return true if estimation succeeded
 */
bool RectangleToQuadrilateral(
    double width,
    double height,
    const std::array<Point2d, 4>& quadPoints,
    HomMat3d& homMat3d
);

/**
 * @brief Check if homography produces valid (non-degenerate) mapping
 *
 * Validates that:
 * - Matrix is invertible
 * - No points map to infinity
 * - Orientation is preserved (no fold-over)
 *
 * @param homMat3d Homography matrix
 * @param srcWidth Source image width
 * @param srcHeight Source image height
 * @return true if the mapping is valid
 */
bool IsValidHomography(const HomMat3d& homMat3d, int32_t srcWidth, int32_t srcHeight);

/**
 * @brief Compute reprojection error for homography
 *
 * @param srcPoints Source points
 * @param dstPoints Destination points
 * @param homMat3d Homography matrix
 * @return RMS reprojection error (pixels)
 */
double HomographyError(
    const std::vector<Point2d>& srcPoints,
    const std::vector<Point2d>& dstPoints,
    const HomMat3d& homMat3d
);

/**
 * @brief Refine homography using Levenberg-Marquardt optimization
 *
 * @param srcPoints Source points
 * @param dstPoints Destination points
 * @param homMat3d Input/output homography (refined in place)
 * @param maxIterations Maximum optimization iterations
 */
void RefineHomography(
    const std::vector<Point2d>& srcPoints,
    const std::vector<Point2d>& dstPoints,
    HomMat3d& homMat3d,
    int32_t maxIterations = 10
);

} // namespace Qi::Vision::Transform
