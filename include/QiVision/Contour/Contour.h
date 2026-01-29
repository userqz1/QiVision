#pragma once

/**
 * @file Contour.h
 * @brief XLD Contour operations (Halcon-style API)
 *
 * API Style: void Func(const QContourArray& in, QContourArray& out, params...)
 *
 * Halcon reference operators:
 * - smooth_contours_xld, simplify_contours_xld
 * - resample_contours_xld, close_contours_xld
 * - length_xld, area_center_xld
 * - smallest_rectangle1_xld, smallest_rectangle2_xld, smallest_circle_xld
 * - curvature_xld, moments_xld
 * - fit_ellipse_contour_xld, fit_line_contour_xld, fit_circle_contour_xld
 * - select_contours_xld, select_shape_xld
 * - segment_contours_xld, split_contours_xld
 * - gen_contour_region_xld, gen_region_contour_xld
 * - gen_circle_contour_xld, gen_ellipse_contour_xld
 *
 * This module wraps Internal layer contour operations into Halcon-style public API.
 */

#include <QiVision/Core/QContour.h>
#include <QiVision/Core/QContourArray.h>
#include <QiVision/Core/QRegion.h>
#include <QiVision/Core/Types.h>

#include <cstdint>
#include <string>
#include <vector>

namespace Qi::Vision::Contour {

// =============================================================================
// Contour Processing
// =============================================================================

/**
 * @brief Smooth contours using moving average
 *
 * Equivalent to Halcon's smooth_contours_xld operator.
 *
 * @param contours Input contour array
 * @param smoothed Output smoothed contours
 * @param numPoints Number of points for smoothing window (default: 5)
 *
 * @code
 * QContourArray smooth;
 * SmoothContoursXld(contours, smooth, 5);
 * @endcode
 */
void SmoothContoursXld(
    const QContourArray& contours,
    QContourArray& smoothed,
    int32_t numPoints = 5
);

/**
 * @brief Smooth contour using Gaussian filter
 *
 * @param contours Input contour array
 * @param smoothed Output smoothed contours
 * @param sigma Gaussian sigma
 */
void SmoothContoursGaussXld(
    const QContourArray& contours,
    QContourArray& smoothed,
    double sigma = 1.0
);

/**
 * @brief Simplify contours using Douglas-Peucker algorithm
 *
 * Reduces the number of points while preserving shape.
 *
 * @param contours Input contour array
 * @param simplified Output simplified contours
 * @param epsilon Maximum deviation tolerance (pixels)
 *
 * @code
 * QContourArray simple;
 * SimplifyContoursXld(contours, simple, 1.0);  // 1 pixel tolerance
 * @endcode
 */
void SimplifyContoursXld(
    const QContourArray& contours,
    QContourArray& simplified,
    double epsilon = 1.0
);

/**
 * @brief Resample contours with fixed distance between points
 *
 * Creates uniformly distributed points along the contour.
 *
 * @param contours Input contour array
 * @param resampled Output resampled contours
 * @param distance Target distance between consecutive points
 *
 * @code
 * QContourArray uniform;
 * ResampleContoursXld(contours, uniform, 2.0);  // 2 pixel spacing
 * @endcode
 */
void ResampleContoursXld(
    const QContourArray& contours,
    QContourArray& resampled,
    double distance
);

/**
 * @brief Resample contours to fixed number of points
 *
 * @param contours Input contour array
 * @param resampled Output resampled contours
 * @param numPoints Target number of points per contour
 */
void ResampleContoursNumXld(
    const QContourArray& contours,
    QContourArray& resampled,
    int32_t numPoints
);

/**
 * @brief Close open contours
 *
 * Marks open contours as closed without adding duplicate points.
 *
 * @param contours Input contour array
 * @param closed Output closed contours
 *
 * @code
 * QContourArray closedContours;
 * CloseContoursXld(contours, closedContours);
 * @endcode
 */
void CloseContoursXld(
    const QContourArray& contours,
    QContourArray& closed
);

/**
 * @brief Reverse contour direction
 *
 * @param contours Input contour array
 * @param reversed Output reversed contours
 */
void ReverseContoursXld(
    const QContourArray& contours,
    QContourArray& reversed
);

// =============================================================================
// Contour Analysis - Basic Properties
// =============================================================================

/**
 * @brief Compute contour length (arc length)
 *
 * Equivalent to Halcon's length_xld operator.
 *
 * @param contour Input contour
 * @return Contour length in pixels
 *
 * @code
 * double len = LengthXld(contour);
 * @endcode
 */
double LengthXld(const QContour& contour);

/**
 * @brief Compute lengths of multiple contours
 *
 * @param contours Input contour array
 * @return Vector of lengths
 */
std::vector<double> LengthXld(const QContourArray& contours);

/**
 * @brief Compute area and centroid of a closed contour
 *
 * Equivalent to Halcon's area_center_xld operator.
 *
 * @param contour Input contour (should be closed)
 * @param row Output centroid Y coordinate
 * @param column Output centroid X coordinate
 * @return Signed area (positive = CCW, negative = CW)
 *
 * @code
 * double row, col;
 * double area = AreaCenterXld(contour, row, col);
 * @endcode
 */
double AreaCenterXld(const QContour& contour, double& row, double& column);

/**
 * @brief Compute contour perimeter
 *
 * For closed contours, includes the closing segment.
 *
 * @param contour Input contour
 * @return Perimeter length in pixels
 */
double PerimeterXld(const QContour& contour);

// =============================================================================
// Contour Analysis - Bounding Geometry
// =============================================================================

/**
 * @brief Compute axis-aligned bounding box
 *
 * Equivalent to Halcon's smallest_rectangle1_xld operator.
 *
 * @param contour Input contour
 * @param row1 Output top Y
 * @param col1 Output left X
 * @param row2 Output bottom Y
 * @param col2 Output right X
 *
 * @code
 * double r1, c1, r2, c2;
 * SmallestRectangle1Xld(contour, r1, c1, r2, c2);
 * @endcode
 */
void SmallestRectangle1Xld(
    const QContour& contour,
    double& row1, double& col1,
    double& row2, double& col2
);

/**
 * @brief Compute minimum area enclosing rectangle (rotated)
 *
 * Equivalent to Halcon's smallest_rectangle2_xld operator.
 *
 * @param contour Input contour
 * @param row Output center Y
 * @param column Output center X
 * @param phi Output rotation angle (radians)
 * @param length1 Output half-length along major axis
 * @param length2 Output half-length along minor axis
 *
 * @code
 * double row, col, phi, len1, len2;
 * SmallestRectangle2Xld(contour, row, col, phi, len1, len2);
 * @endcode
 */
void SmallestRectangle2Xld(
    const QContour& contour,
    double& row, double& column,
    double& phi,
    double& length1, double& length2
);

/**
 * @brief Compute minimum enclosing circle
 *
 * Equivalent to Halcon's smallest_circle_xld operator.
 *
 * @param contour Input contour
 * @param row Output center Y
 * @param column Output center X
 * @param radius Output radius
 *
 * @code
 * double row, col, radius;
 * SmallestCircleXld(contour, row, col, radius);
 * @endcode
 */
void SmallestCircleXld(
    const QContour& contour,
    double& row, double& column,
    double& radius
);

// =============================================================================
// Contour Analysis - Curvature
// =============================================================================

/**
 * @brief Compute curvature at each point of a contour
 *
 * Equivalent to Halcon's curvature_xld operator.
 * Curvature k = 1/R where R is the radius of the osculating circle.
 *
 * @param contour Input contour
 * @param windowSize Window size for curvature computation (default: 5)
 * @return Vector of curvatures (same size as contour points)
 *
 * @code
 * auto curvatures = CurvatureXld(contour, 5);
 * @endcode
 */
std::vector<double> CurvatureXld(
    const QContour& contour,
    int32_t windowSize = 5
);

/**
 * @brief Compute mean curvature of a contour
 *
 * @param contour Input contour
 * @return Mean absolute curvature
 */
double MeanCurvatureXld(const QContour& contour);

/**
 * @brief Compute maximum curvature of a contour
 *
 * @param contour Input contour
 * @return Maximum absolute curvature
 */
double MaxCurvatureXld(const QContour& contour);

// =============================================================================
// Contour Analysis - Moments
// =============================================================================

/**
 * @brief Compute geometric moments of a contour
 *
 * Equivalent to Halcon's moments_xld operator.
 *
 * @param contour Input contour
 * @param m00 Output zeroth moment (area proxy)
 * @param m10 Output first moment (X)
 * @param m01 Output first moment (Y)
 * @param m20 Output second moment (XX)
 * @param m11 Output second moment (XY)
 * @param m02 Output second moment (YY)
 *
 * @code
 * double m00, m10, m01, m20, m11, m02;
 * MomentsXld(contour, m00, m10, m01, m20, m11, m02);
 * @endcode
 */
void MomentsXld(
    const QContour& contour,
    double& m00, double& m10, double& m01,
    double& m20, double& m11, double& m02
);

/**
 * @brief Compute central moments of a contour
 *
 * Central moments are translation invariant.
 *
 * @param contour Input contour
 * @param mu00 Output mu_00
 * @param mu20 Output mu_20
 * @param mu11 Output mu_11
 * @param mu02 Output mu_02
 */
void CentralMomentsXld(
    const QContour& contour,
    double& mu00, double& mu20,
    double& mu11, double& mu02
);

/**
 * @brief Compute orientation from moments
 *
 * Returns the principal axis angle from the second-order central moments.
 *
 * @param contour Input contour
 * @return Orientation angle in radians [-PI/2, PI/2]
 */
double OrientationXld(const QContour& contour);

// =============================================================================
// Contour Analysis - Shape Descriptors
// =============================================================================

/**
 * @brief Compute circularity of a closed contour
 *
 * Circularity = 4 * PI * Area / Perimeter^2
 * Equals 1.0 for a perfect circle, less for other shapes.
 *
 * @param contour Input contour (should be closed)
 * @return Circularity value in [0, 1]
 */
double CircularityXld(const QContour& contour);

/**
 * @brief Compute convexity of a contour
 *
 * Convexity = Convex hull perimeter / Contour perimeter
 * Equals 1.0 for convex shapes.
 *
 * @param contour Input contour
 * @return Convexity value in [0, 1]
 */
double ConvexityXld(const QContour& contour);

/**
 * @brief Compute solidity of a contour
 *
 * Solidity = Area / Convex hull area
 * Equals 1.0 for convex shapes.
 *
 * @param contour Input contour (should be closed)
 * @return Solidity value in [0, 1]
 */
double SolidityXld(const QContour& contour);

/**
 * @brief Compute eccentricity of a contour
 *
 * Eccentricity = sqrt(1 - (minorAxis/majorAxis)^2)
 * Equals 0 for circle, approaches 1 for elongated shapes.
 *
 * @param contour Input contour
 * @return Eccentricity value in [0, 1)
 */
double EccentricityXld(const QContour& contour);

/**
 * @brief Compute compactness of a contour
 *
 * Compactness = Perimeter^2 / Area
 * Minimum for circle (4*PI).
 *
 * @param contour Input contour (should be closed)
 * @return Compactness value (>= 4*PI)
 */
double CompactnessXld(const QContour& contour);

// =============================================================================
// Contour Fitting
// =============================================================================

/**
 * @brief Fit an ellipse to a contour
 *
 * Equivalent to Halcon's fit_ellipse_contour_xld operator.
 *
 * @param contour Input contour
 * @param row Output ellipse center Y
 * @param column Output ellipse center X
 * @param phi Output ellipse orientation (radians)
 * @param ra Output semi-major axis length
 * @param rb Output semi-minor axis length
 * @return True if fitting succeeded
 *
 * @code
 * double row, col, phi, ra, rb;
 * if (FitEllipseContourXld(contour, row, col, phi, ra, rb)) {
 *     // Use fitted ellipse
 * }
 * @endcode
 */
bool FitEllipseContourXld(
    const QContour& contour,
    double& row, double& column,
    double& phi, double& ra, double& rb
);

/**
 * @brief Fit a line to a contour
 *
 * Equivalent to Halcon's fit_line_contour_xld operator.
 *
 * @param contour Input contour
 * @param rowBegin Output line start Y
 * @param colBegin Output line start X
 * @param rowEnd Output line end Y
 * @param colEnd Output line end X
 * @param row Output line center Y
 * @param column Output line center X
 * @param phi Output line angle (radians)
 * @return True if fitting succeeded
 *
 * @code
 * double rb, cb, re, ce, r, c, phi;
 * if (FitLineContourXld(contour, rb, cb, re, ce, r, c, phi)) {
 *     // Use fitted line
 * }
 * @endcode
 */
bool FitLineContourXld(
    const QContour& contour,
    double& rowBegin, double& colBegin,
    double& rowEnd, double& colEnd,
    double& row, double& column,
    double& phi
);

/**
 * @brief Fit a circle/arc to a contour
 *
 * Equivalent to Halcon's fit_circle_contour_xld operator.
 *
 * @param contour Input contour
 * @param row Output circle center Y
 * @param column Output circle center X
 * @param radius Output circle radius
 * @param startPhi Output arc start angle (radians)
 * @param endPhi Output arc end angle (radians)
 * @param algorithm Fitting algorithm: "algebraic" (fast), "geometric" (accurate)
 * @return True if fitting succeeded
 *
 * @code
 * double row, col, radius, startPhi, endPhi;
 * if (FitCircleContourXld(contour, row, col, radius, startPhi, endPhi, "algebraic")) {
 *     // Use fitted circle
 * }
 * @endcode
 */
bool FitCircleContourXld(
    const QContour& contour,
    double& row, double& column,
    double& radius,
    double& startPhi, double& endPhi,
    const std::string& algorithm = "algebraic"
);

/**
 * @brief Compute convex hull of a contour
 *
 * @param contour Input contour
 * @param hull Output convex hull contour
 */
void ConvexHullXld(const QContour& contour, QContour& hull);

// =============================================================================
// Contour Selection
// =============================================================================

/**
 * @brief Select contours by a feature value range
 *
 * Equivalent to Halcon's select_contours_xld / select_shape_xld operators.
 *
 * @param contours Input contour array
 * @param selected Output selected contours
 * @param feature Feature to select by:
 *        "length", "area", "circularity", "convexity", "solidity",
 *        "compactness", "eccentricity", "elongation", "rectangularity",
 *        "mean_curvature", "max_curvature", "num_points"
 * @param minValue Minimum feature value (inclusive)
 * @param maxValue Maximum feature value (inclusive)
 *
 * @code
 * QContourArray large;
 * SelectContoursXld(contours, large, "length", 100.0, 1e6);  // Length >= 100
 * @endcode
 */
void SelectContoursXld(
    const QContourArray& contours,
    QContourArray& selected,
    const std::string& feature,
    double minValue,
    double maxValue
);

/**
 * @brief Select only closed contours
 *
 * @param contours Input contour array
 * @param closed Output closed contours
 */
void SelectClosedXld(
    const QContourArray& contours,
    QContourArray& closed
);

/**
 * @brief Select only open contours
 *
 * @param contours Input contour array
 * @param open Output open contours
 */
void SelectOpenXld(
    const QContourArray& contours,
    QContourArray& open
);

/**
 * @brief Sort contours by a feature
 *
 * @param contours Input contour array
 * @param sorted Output sorted contours
 * @param feature Feature to sort by
 * @param ascending True for ascending order, false for descending
 */
void SortContoursXld(
    const QContourArray& contours,
    QContourArray& sorted,
    const std::string& feature,
    bool ascending = true
);

/**
 * @brief Select top N contours by a feature
 *
 * @param contours Input contour array
 * @param selected Output selected contours
 * @param feature Feature to rank by
 * @param count Number of contours to select
 * @param largest True to select largest values, false for smallest
 */
void SelectTopContoursXld(
    const QContourArray& contours,
    QContourArray& selected,
    const std::string& feature,
    int32_t count,
    bool largest = true
);

// =============================================================================
// Contour Segmentation
// =============================================================================

/**
 * @brief Segment contour into lines and/or circular arcs
 *
 * Equivalent to Halcon's segment_contours_xld operator.
 *
 * @param contour Input contour
 * @param segments Output segmented contours
 * @param mode Segmentation mode: "lines", "circles", "lines_circles"
 * @param maxLineDev Maximum deviation for line fitting (pixels)
 * @param maxArcDev Maximum deviation for arc fitting (pixels)
 *
 * @code
 * QContourArray segments;
 * SegmentContoursXld(contour, segments, "lines_circles", 2.0, 2.0);
 * @endcode
 */
void SegmentContoursXld(
    const QContour& contour,
    QContourArray& segments,
    const std::string& mode = "lines_circles",
    double maxLineDev = 2.0,
    double maxArcDev = 2.0
);

/**
 * @brief Split contours at high curvature points (corners)
 *
 * Equivalent to Halcon's split_contours_xld operator.
 *
 * @param contours Input contour array
 * @param split Output split contours
 * @param maxAngle Maximum angle change threshold (radians)
 *
 * @code
 * QContourArray split;
 * SplitContoursXld(contours, split, 0.5);  // Split at ~28 degree corners
 * @endcode
 */
void SplitContoursXld(
    const QContourArray& contours,
    QContourArray& split,
    double maxAngle = 0.5
);

/**
 * @brief Detect corner points in a contour
 *
 * @param contour Input contour
 * @param curvatureThreshold Minimum curvature for corner detection
 * @return Indices of corner points
 */
std::vector<int32_t> DetectCornersXld(
    const QContour& contour,
    double curvatureThreshold = 0.1
);

// =============================================================================
// Contour-Region Conversion
// =============================================================================

/**
 * @brief Convert region boundary to contours
 *
 * Equivalent to Halcon's gen_contour_region_xld operator.
 *
 * @param region Input region
 * @param contours Output boundary contours
 * @param mode Boundary mode: "border" (outer only), "border_holes" (include holes)
 *
 * @code
 * QContourArray boundaries;
 * GenContourRegionXld(region, boundaries, "border_holes");
 * @endcode
 */
void GenContourRegionXld(
    const QRegion& region,
    QContourArray& contours,
    const std::string& mode = "border"
);

/**
 * @brief Convert contour to filled region
 *
 * Equivalent to Halcon's gen_region_contour_xld operator.
 *
 * @param contour Input contour (should be closed)
 * @param region Output filled region
 * @param mode Fill mode: "filled" (interior), "margin" (contour line only)
 *
 * @code
 * QRegion filled;
 * GenRegionContourXld(contour, filled, "filled");
 * @endcode
 */
void GenRegionContourXld(
    const QContour& contour,
    QRegion& region,
    const std::string& mode = "filled"
);

/**
 * @brief Convert multiple contours to region
 *
 * @param contours Input contour array
 * @param region Output combined region
 * @param mode Fill mode
 */
void GenRegionContoursXld(
    const QContourArray& contours,
    QRegion& region,
    const std::string& mode = "filled"
);

// =============================================================================
// Contour Generation
// =============================================================================

/**
 * @brief Create contour from a list of points
 *
 * Equivalent to Halcon's gen_contour_polygon_xld operator.
 *
 * @param points Input points
 * @return Generated contour
 *
 * @code
 * std::vector<Point2d> pts = {{10,10}, {100,10}, {100,100}, {10,100}};
 * QContour rect = GenContourPolygonXld(pts);
 * @endcode
 */
QContour GenContourPolygonXld(const std::vector<Point2d>& points);

/**
 * @brief Create contour from coordinate arrays
 *
 * @param rows Y coordinates
 * @param cols X coordinates
 * @return Generated contour
 */
QContour GenContourPolygonXld(
    const std::vector<double>& rows,
    const std::vector<double>& cols
);

/**
 * @brief Generate a circle contour
 *
 * Equivalent to Halcon's gen_circle_contour_xld operator.
 *
 * @param row Center Y
 * @param column Center X
 * @param radius Circle radius
 * @param startAngle Start angle (radians, default: 0)
 * @param endAngle End angle (radians, default: 2*PI)
 * @param resolution Point resolution: "positive" (CCW), "negative" (CW)
 * @param stepAngle Angular step between points (radians, default: 0.01)
 * @return Generated circle/arc contour
 *
 * @code
 * QContour circle = GenCircleContourXld(100, 100, 50);  // Full circle
 * QContour arc = GenCircleContourXld(100, 100, 50, 0, 1.57);  // Quarter arc
 * @endcode
 */
QContour GenCircleContourXld(
    double row, double column,
    double radius,
    double startAngle = 0.0,
    double endAngle = 6.28318,
    const std::string& resolution = "positive",
    double stepAngle = 0.01
);

/**
 * @brief Generate an ellipse contour
 *
 * Equivalent to Halcon's gen_ellipse_contour_xld operator.
 *
 * @param row Center Y
 * @param column Center X
 * @param phi Orientation angle (radians)
 * @param ra Semi-major axis length
 * @param rb Semi-minor axis length
 * @param startAngle Start angle (radians, default: 0)
 * @param endAngle End angle (radians, default: 2*PI)
 * @param stepAngle Angular step between points (radians)
 * @return Generated ellipse/elliptic arc contour
 *
 * @code
 * QContour ellipse = GenEllipseContourXld(100, 100, 0.5, 80, 40);
 * @endcode
 */
QContour GenEllipseContourXld(
    double row, double column,
    double phi,
    double ra, double rb,
    double startAngle = 0.0,
    double endAngle = 6.28318,
    double stepAngle = 0.01
);

/**
 * @brief Generate a rectangle contour (rotated)
 *
 * Equivalent to Halcon's gen_rectangle2_contour_xld operator.
 *
 * @param row Center Y
 * @param column Center X
 * @param phi Orientation angle (radians)
 * @param length1 Half-length along major axis
 * @param length2 Half-length along minor axis
 * @return Generated rectangle contour (closed)
 *
 * @code
 * QContour rect = GenRectangle2ContourXld(100, 100, 0.3, 50, 30);
 * @endcode
 */
QContour GenRectangle2ContourXld(
    double row, double column,
    double phi,
    double length1, double length2
);

/**
 * @brief Generate an axis-aligned rectangle contour
 *
 * @param row1 Top Y
 * @param col1 Left X
 * @param row2 Bottom Y
 * @param col2 Right X
 * @return Generated rectangle contour (closed)
 */
QContour GenRectangle1ContourXld(
    double row1, double col1,
    double row2, double col2
);

/**
 * @brief Generate a line segment contour
 *
 * @param row1 Start Y
 * @param col1 Start X
 * @param row2 End Y
 * @param col2 End X
 * @return Generated line segment contour (open)
 */
QContour GenLineContourXld(
    double row1, double col1,
    double row2, double col2
);

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Get the number of points in a contour
 *
 * @param contour Input contour
 * @return Number of points
 */
int32_t CountPointsXld(const QContour& contour);

/**
 * @brief Get the number of points in each contour
 *
 * @param contours Input contour array
 * @return Vector of point counts
 */
std::vector<int32_t> CountPointsXld(const QContourArray& contours);

/**
 * @brief Get contour points as coordinate arrays
 *
 * @param contour Input contour
 * @param rows Output Y coordinates
 * @param cols Output X coordinates
 */
void GetContourXld(
    const QContour& contour,
    std::vector<double>& rows,
    std::vector<double>& cols
);

/**
 * @brief Check if a point is inside a closed contour
 *
 * @param contour Input contour (should be closed)
 * @param row Point Y coordinate
 * @param column Point X coordinate
 * @return True if point is inside
 */
bool TestPointXld(
    const QContour& contour,
    double row, double column
);

/**
 * @brief Compute distance from a point to a contour
 *
 * @param contour Input contour
 * @param row Point Y coordinate
 * @param column Point X coordinate
 * @return Distance to nearest contour point
 */
double DistancePointXld(
    const QContour& contour,
    double row, double column
);

/**
 * @brief Union of contour arrays
 *
 * @param contours1 First contour array
 * @param contours2 Second contour array
 * @param result Output combined array
 */
void UnionContoursXld(
    const QContourArray& contours1,
    const QContourArray& contours2,
    QContourArray& result
);

/**
 * @brief Get a subset of contours by indices
 *
 * @param contours Input contour array
 * @param selected Output selected contours
 * @param indices Indices to select (0-based)
 */
void SelectObjXld(
    const QContourArray& contours,
    QContourArray& selected,
    const std::vector<int32_t>& indices
);

/**
 * @brief Get a single contour from an array
 *
 * @param contours Input contour array
 * @param index Index (0-based)
 * @return Selected contour
 */
QContour SelectObjXld(
    const QContourArray& contours,
    int32_t index
);

/**
 * @brief Get the number of contours in an array
 *
 * @param contours Input contour array
 * @return Number of contours
 */
int32_t CountObjXld(const QContourArray& contours);

} // namespace Qi::Vision::Contour
