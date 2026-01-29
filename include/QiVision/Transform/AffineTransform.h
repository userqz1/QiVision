#pragma once

/**
 * @file AffineTransform.h
 * @brief Public API for affine transformation operations
 *
 * Provides:
 * - Image warping (affine, rotate, scale)
 * - Transform matrix creation and manipulation
 * - Point transformation
 *
 * Reference Halcon operators:
 * - affine_trans_image, rotate_image, zoom_image_size
 * - hom_mat2d_identity, hom_mat2d_rotate, hom_mat2d_scale
 * - hom_mat2d_translate, hom_mat2d_compose, hom_mat2d_invert
 * - affine_trans_point_2d
 */

#include <QiVision/Core/Types.h>
#include <QiVision/Core/QImage.h>
#include <QiVision/Core/QMatrix.h>

#include <string>
#include <vector>

namespace Qi::Vision::Transform {

// =============================================================================
// Image Transformation
// =============================================================================

/**
 * @brief Apply affine transformation to an image
 *
 * Transforms the source image using the given affine transformation matrix.
 * Uses backward mapping with the specified interpolation method.
 *
 * @param src Source image (grayscale or multi-channel)
 * @param dst Output transformed image
 * @param matrix 2D affine transformation matrix (QMatrix)
 * @param interpolation Interpolation method: "nearest", "bilinear" (default), "bicubic"
 * @param borderMode Border handling: "constant" (default), "replicate", "reflect", "wrap"
 * @param borderValue Border value for constant mode
 */
void AffineTransImage(
    const QImage& src,
    QImage& dst,
    const QMatrix& matrix,
    const std::string& interpolation = "bilinear",
    const std::string& borderMode = "constant",
    double borderValue = 0.0
);

/**
 * @brief Apply affine transformation with specified output size
 *
 * @param src Source image
 * @param dst Output transformed image
 * @param matrix 2D affine transformation matrix
 * @param dstWidth Output width
 * @param dstHeight Output height
 * @param interpolation Interpolation method
 * @param borderMode Border handling mode
 * @param borderValue Border value for constant mode
 */
void AffineTransImage(
    const QImage& src,
    QImage& dst,
    const QMatrix& matrix,
    int32_t dstWidth,
    int32_t dstHeight,
    const std::string& interpolation = "bilinear",
    const std::string& borderMode = "constant",
    double borderValue = 0.0
);

/**
 * @brief Rotate image around its center
 *
 * Rotates the image by the specified angle around its center.
 * Output size is automatically calculated to fit the entire rotated image.
 *
 * @param src Source image
 * @param dst Output rotated image
 * @param angle Rotation angle in radians (positive = counter-clockwise)
 * @param interpolation Interpolation method: "nearest", "bilinear" (default), "bicubic"
 */
void RotateImage(
    const QImage& src,
    QImage& dst,
    double angle,
    const std::string& interpolation = "bilinear"
);

/**
 * @brief Rotate image around a specified center point
 *
 * @param src Source image
 * @param dst Output rotated image
 * @param angle Rotation angle in radians
 * @param centerRow Center Y coordinate
 * @param centerCol Center X coordinate
 * @param interpolation Interpolation method
 */
void RotateImage(
    const QImage& src,
    QImage& dst,
    double angle,
    double centerRow,
    double centerCol,
    const std::string& interpolation = "bilinear"
);

/**
 * @brief Scale image by factors
 *
 * Scales the image by the specified horizontal and vertical factors.
 *
 * @param src Source image
 * @param dst Output scaled image
 * @param scaleX Horizontal scale factor (>1 enlarges, <1 shrinks)
 * @param scaleY Vertical scale factor
 * @param interpolation Interpolation method: "nearest", "bilinear" (default), "bicubic"
 */
void ScaleImage(
    const QImage& src,
    QImage& dst,
    double scaleX,
    double scaleY,
    const std::string& interpolation = "bilinear"
);

/**
 * @brief Scale image to specified size
 *
 * @param src Source image
 * @param dst Output scaled image
 * @param dstWidth Target width
 * @param dstHeight Target height
 * @param interpolation Interpolation method
 */
void ZoomImageSize(
    const QImage& src,
    QImage& dst,
    int32_t dstWidth,
    int32_t dstHeight,
    const std::string& interpolation = "bilinear"
);

// =============================================================================
// Matrix Creation (Halcon-style naming)
// =============================================================================

/**
 * @brief Create identity transformation matrix
 *
 * Returns a 2D identity matrix (no transformation).
 *
 * @return Identity QMatrix
 */
QMatrix HomMat2dIdentity();

/**
 * @brief Create rotation transformation matrix
 *
 * Creates a matrix that rotates around a specified center point.
 *
 * @param phi Rotation angle in radians (positive = counter-clockwise)
 * @param cy Center Y coordinate (row)
 * @param cx Center X coordinate (column)
 * @return Rotation QMatrix
 */
QMatrix HomMat2dRotate(double phi, double cy, double cx);

/**
 * @brief Create scaling transformation matrix
 *
 * Creates a matrix that scales around a specified center point.
 *
 * @param sy Scale factor in Y direction
 * @param sx Scale factor in X direction
 * @param cy Center Y coordinate (row)
 * @param cx Center X coordinate (column)
 * @return Scaling QMatrix
 */
QMatrix HomMat2dScale(double sy, double sx, double cy, double cx);

/**
 * @brief Add translation to transformation matrix
 *
 * Creates a new matrix that performs the original transformation
 * followed by a translation.
 *
 * @param homMat2d Input transformation matrix
 * @param ty Translation in Y direction (row)
 * @param tx Translation in X direction (column)
 * @return Translated QMatrix
 */
QMatrix HomMat2dTranslate(const QMatrix& homMat2d, double ty, double tx);

/**
 * @brief Create translation-only matrix
 *
 * @param ty Translation in Y direction
 * @param tx Translation in X direction
 * @return Translation QMatrix
 */
QMatrix HomMat2dTranslateOnly(double ty, double tx);

/**
 * @brief Compose two transformation matrices
 *
 * Computes m1 * m2, which applies m2 first, then m1.
 *
 * @param homMat2d1 First transformation matrix (applied second)
 * @param homMat2d2 Second transformation matrix (applied first)
 * @return Composed QMatrix
 */
QMatrix HomMat2dCompose(const QMatrix& homMat2d1, const QMatrix& homMat2d2);

/**
 * @brief Invert transformation matrix
 *
 * Computes the inverse transformation matrix.
 *
 * @param homMat2d Input transformation matrix
 * @return Inverted QMatrix (identity if not invertible)
 */
QMatrix HomMat2dInvert(const QMatrix& homMat2d);

/**
 * @brief Add rotation to transformation matrix
 *
 * Creates a new matrix that performs the original transformation
 * followed by a rotation around a specified center.
 *
 * @param homMat2d Input transformation matrix
 * @param phi Rotation angle in radians
 * @param cy Center Y coordinate
 * @param cx Center X coordinate
 * @return Rotated QMatrix
 */
QMatrix HomMat2dRotateLocal(const QMatrix& homMat2d, double phi, double cy, double cx);

/**
 * @brief Add scaling to transformation matrix
 *
 * Creates a new matrix that performs the original transformation
 * followed by a scaling around a specified center.
 *
 * @param homMat2d Input transformation matrix
 * @param sy Scale factor in Y direction
 * @param sx Scale factor in X direction
 * @param cy Center Y coordinate
 * @param cx Center X coordinate
 * @return Scaled QMatrix
 */
QMatrix HomMat2dScaleLocal(const QMatrix& homMat2d, double sy, double sx, double cy, double cx);

// =============================================================================
// Point Transformation
// =============================================================================

/**
 * @brief Transform a single point using affine matrix
 *
 * @param homMat2d Transformation matrix
 * @param point Input point
 * @return Transformed point
 */
Point2d AffineTransPoint2d(const QMatrix& homMat2d, const Point2d& point);

/**
 * @brief Transform a single point using affine matrix (row/col version)
 *
 * @param homMat2d Transformation matrix
 * @param py Input Y coordinate (row)
 * @param px Input X coordinate (column)
 * @param qy Output Y coordinate (row)
 * @param qx Output X coordinate (column)
 */
void AffineTransPoint2d(
    const QMatrix& homMat2d,
    double py, double px,
    double& qy, double& qx
);

/**
 * @brief Transform multiple points using affine matrix
 *
 * @param homMat2d Transformation matrix
 * @param points Input points
 * @return Transformed points
 */
std::vector<Point2d> AffineTransPoint2d(
    const QMatrix& homMat2d,
    const std::vector<Point2d>& points
);

/**
 * @brief Transform multiple points using affine matrix (row/col version)
 *
 * @param homMat2d Transformation matrix
 * @param py Input Y coordinates
 * @param px Input X coordinates
 * @param qy Output Y coordinates
 * @param qx Output X coordinates
 */
void AffineTransPoint2d(
    const QMatrix& homMat2d,
    const std::vector<double>& py, const std::vector<double>& px,
    std::vector<double>& qy, std::vector<double>& qx
);

// =============================================================================
// Transform Estimation
// =============================================================================

/**
 * @brief Estimate affine transformation from point correspondences
 *
 * Estimates the best-fit affine transformation that maps source points
 * to destination points using least squares.
 *
 * @param srcPoints Source points (at least 3 required)
 * @param dstPoints Destination points (same count as srcPoints)
 * @param homMat2d Output transformation matrix
 * @return true if estimation succeeded
 */
bool VectorToHomMat2d(
    const std::vector<Point2d>& srcPoints,
    const std::vector<Point2d>& dstPoints,
    QMatrix& homMat2d
);

/**
 * @brief Estimate rigid transformation from point correspondences
 *
 * Estimates the best-fit rigid transformation (rotation + translation only)
 * using Procrustes analysis.
 *
 * @param srcPoints Source points (at least 2 required)
 * @param dstPoints Destination points
 * @param homMat2d Output transformation matrix
 * @return true if estimation succeeded
 */
bool VectorToRigid(
    const std::vector<Point2d>& srcPoints,
    const std::vector<Point2d>& dstPoints,
    QMatrix& homMat2d
);

/**
 * @brief Estimate similarity transformation from point correspondences
 *
 * Estimates the best-fit similarity transformation (rotation + translation + uniform scale).
 *
 * @param srcPoints Source points (at least 2 required)
 * @param dstPoints Destination points
 * @param homMat2d Output transformation matrix
 * @return true if estimation succeeded
 */
bool VectorToSimilarity(
    const std::vector<Point2d>& srcPoints,
    const std::vector<Point2d>& dstPoints,
    QMatrix& homMat2d
);

// =============================================================================
// Matrix Analysis
// =============================================================================

/**
 * @brief Decompose affine matrix into components
 *
 * Decomposes the transformation into translation, rotation, scale, and shear.
 *
 * @param homMat2d Input transformation matrix
 * @param ty Output translation Y
 * @param tx Output translation X
 * @param phi Output rotation angle (radians)
 * @param sy Output scale Y
 * @param sx Output scale X
 * @param theta Output shear angle (radians)
 * @return true if decomposition succeeded
 */
bool HomMat2dToAffinePar(
    const QMatrix& homMat2d,
    double& ty, double& tx,
    double& phi,
    double& sy, double& sx,
    double& theta
);

/**
 * @brief Check if matrix represents a rigid transformation
 *
 * @param homMat2d Input transformation matrix
 * @param tolerance Numerical tolerance
 * @return true if the transform is rigid (rotation + translation only)
 */
bool HomMat2dIsRigid(const QMatrix& homMat2d, double tolerance = 1e-6);

/**
 * @brief Check if matrix represents a similarity transformation
 *
 * @param homMat2d Input transformation matrix
 * @param tolerance Numerical tolerance
 * @return true if the transform is similarity (rigid + uniform scale)
 */
bool HomMat2dIsSimilarity(const QMatrix& homMat2d, double tolerance = 1e-6);

/**
 * @brief Get determinant of transformation matrix
 *
 * @param homMat2d Input transformation matrix
 * @return Determinant of the linear part
 */
double HomMat2dDeterminant(const QMatrix& homMat2d);

} // namespace Qi::Vision::Transform
