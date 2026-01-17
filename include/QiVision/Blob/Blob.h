#pragma once

/**
 * @file Blob.h
 * @brief Blob analysis and region selection (Halcon-style API)
 *
 * Halcon reference operators:
 * - connection, select_shape, count_obj, select_obj
 * - area_center, smallest_rectangle1/2, smallest_circle
 * - circularity, compactness, convexity, orientation_region
 * - moments_region_2nd, elliptic_axis
 *
 * Precision: sub-pixel features computed from RLE region representation
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/QRegion.h>
#include <QiVision/Core/Types.h>

#include <cstdint>
#include <string>
#include <vector>

namespace Qi::Vision::Blob {

// =============================================================================
// Types
// =============================================================================

/**
 * @brief Shape features for select_shape
 */
enum class ShapeFeature {
    Area,               ///< Region area in pixels
    Row,                ///< Centroid row
    Column,             ///< Centroid column
    Width,              ///< Bounding box width
    Height,             ///< Bounding box height
    Circularity,        ///< 4*pi*area/perimeter^2
    Compactness,        ///< perimeter^2/area
    Convexity,          ///< perimeter_convex/perimeter
    Rectangularity,     ///< area/bbox_area
    Elongation,         ///< major_axis/minor_axis
    Orientation,        ///< Principal axis angle (radians)
    Ra,                 ///< Equivalent ellipse major radius
    Rb,                 ///< Equivalent ellipse minor radius
    Phi,                ///< Equivalent ellipse orientation
    Anisometry,         ///< Ra/Rb
    Bulkiness,          ///< pi*Ra*Rb/area
    StructureFactor,    ///< Anisometry*Bulkiness-1
    OuterRadius,        ///< Smallest enclosing circle radius
    InnerRadius,        ///< Largest inscribed circle radius
    Holes               ///< Number of holes
};

/**
 * @brief Selection operation mode
 */
enum class SelectOperation {
    And,    ///< All features must be in range
    Or      ///< Any feature in range
};

/**
 * @brief Region sorting criteria
 */
enum class SortMode {
    None,               ///< No sorting
    Area,               ///< Sort by area
    Row,                ///< Sort by centroid row
    Column,             ///< Sort by centroid column
    FirstPoint,         ///< Sort by first point
    LastPoint           ///< Sort by last point
};

// =============================================================================
// Connection (Connected Component Analysis)
// =============================================================================

/**
 * @brief Extract connected components from a region
 *
 * Equivalent to Halcon's connection operator.
 *
 * @param region Input region
 * @return Vector of connected regions
 *
 * @code
 * QRegion binaryRegion = ThresholdToRegion(image, 128, 255);
 * auto blobs = Connection(binaryRegion);
 * std::cout << "Found " << blobs.size() << " blobs\n";
 * @endcode
 */
std::vector<QRegion> Connection(const QRegion& region);

/**
 * @brief Extract connected components from binary image
 *
 * @param binaryImage Binary image (non-zero = foreground)
 * @param connectivity 4 or 8 connectivity (default: 8)
 * @return Vector of connected regions
 */
std::vector<QRegion> Connection(const QImage& binaryImage,
                                 Connectivity connectivity = Connectivity::Eight);

/**
 * @brief Count number of objects
 *
 * Equivalent to Halcon's count_obj operator.
 */
inline int32_t CountObj(const std::vector<QRegion>& regions) {
    return static_cast<int32_t>(regions.size());
}

/**
 * @brief Select object by index (1-based like Halcon)
 *
 * Equivalent to Halcon's select_obj operator.
 *
 * @param regions Input regions
 * @param index 1-based index
 * @return Selected region (empty if out of range)
 */
QRegion SelectObj(const std::vector<QRegion>& regions, int32_t index);

// =============================================================================
// Region Features
// =============================================================================

/**
 * @brief Get area and centroid of region
 *
 * Equivalent to Halcon's area_center operator.
 *
 * @param region Input region
 * @param[out] area Area in pixels
 * @param[out] row Centroid row
 * @param[out] column Centroid column
 */
void AreaCenter(const QRegion& region, int64_t& area, double& row, double& column);

/**
 * @brief Get area and centroid for multiple regions
 */
void AreaCenter(const std::vector<QRegion>& regions,
                std::vector<int64_t>& areas,
                std::vector<double>& rows,
                std::vector<double>& columns);

/**
 * @brief Get axis-aligned bounding box
 *
 * Equivalent to Halcon's smallest_rectangle1 operator.
 *
 * @param region Input region
 * @param[out] row1 Top row
 * @param[out] column1 Left column
 * @param[out] row2 Bottom row
 * @param[out] column2 Right column
 */
void SmallestRectangle1(const QRegion& region,
                         int32_t& row1, int32_t& column1,
                         int32_t& row2, int32_t& column2);

/**
 * @brief Get minimum area rotated bounding box
 *
 * Equivalent to Halcon's smallest_rectangle2 operator.
 *
 * @param region Input region
 * @param[out] row Center row
 * @param[out] column Center column
 * @param[out] phi Rotation angle (radians)
 * @param[out] length1 Half-length along major axis
 * @param[out] length2 Half-length along minor axis
 */
void SmallestRectangle2(const QRegion& region,
                         double& row, double& column, double& phi,
                         double& length1, double& length2);

/**
 * @brief Get smallest enclosing circle
 *
 * Equivalent to Halcon's smallest_circle operator.
 *
 * @param region Input region
 * @param[out] row Circle center row
 * @param[out] column Circle center column
 * @param[out] radius Circle radius
 */
void SmallestCircle(const QRegion& region,
                     double& row, double& column, double& radius);

/**
 * @brief Get largest inscribed circle (inner circle)
 *
 * Equivalent to Halcon's inner_circle operator.
 * Uses distance transform to find the point furthest from the boundary.
 *
 * @param region Input region
 * @param[out] row Circle center row
 * @param[out] column Circle center column
 * @param[out] radius Circle radius
 */
void InnerCircle(const QRegion& region,
                  double& row, double& column, double& radius);

/**
 * @brief Compute region contour length (perimeter)
 *
 * Equivalent to Halcon's contlength operator.
 *
 * @param region Input region
 * @return Contour length in pixels
 */
double ContourLength(const QRegion& region);

/**
 * @brief Count number of holes in a region
 *
 * Equivalent to Halcon's connect_and_holes / euler_number.
 *
 * @param region Input region
 * @return Number of holes (0 if no holes)
 */
int32_t CountHoles(const QRegion& region);

/**
 * @brief Get the Euler number of a region
 *
 * Euler number = #connected_components - #holes
 * For a single region without holes, Euler = 1
 *
 * @param region Input region
 * @return Euler number
 */
int32_t EulerNumber(const QRegion& region);

/**
 * @brief Fill holes in a region
 *
 * Equivalent to Halcon's fill_up operator.
 *
 * @param region Input region
 * @return Region with holes filled
 */
QRegion FillUp(const QRegion& region);

/**
 * @brief Get the holes of a region
 *
 * @param region Input region
 * @return Vector of hole regions
 */
std::vector<QRegion> GetHoles(const QRegion& region);

/**
 * @brief Compute region circularity
 *
 * Equivalent to Halcon's circularity operator.
 * Circularity = 4*pi*area/perimeter^2, 1.0 for perfect circle.
 *
 * @param region Input region
 * @return Circularity value [0, 1]
 */
double Circularity(const QRegion& region);

/**
 * @brief Compute region compactness
 *
 * Equivalent to Halcon's compactness operator.
 * Compactness = perimeter^2/area, minimum for circle.
 *
 * @param region Input region
 * @return Compactness value (>= 4*pi)
 */
double Compactness(const QRegion& region);

/**
 * @brief Compute region convexity
 *
 * Equivalent to Halcon's convexity operator.
 *
 * @param region Input region
 * @return Convexity value [0, 1]
 */
double Convexity(const QRegion& region);

/**
 * @brief Compute region rectangularity
 *
 * @param region Input region
 * @return Rectangularity = area/bbox_area [0, 1]
 */
double Rectangularity(const QRegion& region);

/**
 * @brief Compute equivalent ellipse parameters
 *
 * Equivalent to Halcon's elliptic_axis operator.
 *
 * @param region Input region
 * @param[out] ra Major radius (semi-major axis)
 * @param[out] rb Minor radius (semi-minor axis)
 * @param[out] phi Orientation angle (radians)
 */
void EllipticAxis(const QRegion& region, double& ra, double& rb, double& phi);

/**
 * @brief Compute region orientation
 *
 * Equivalent to Halcon's orientation_region operator.
 *
 * @param region Input region
 * @return Orientation angle in radians [-pi/2, pi/2]
 */
double OrientationRegion(const QRegion& region);

/**
 * @brief Compute second-order central moments
 *
 * Equivalent to Halcon's moments_region_2nd operator.
 *
 * @param region Input region
 * @param[out] m11 Mixed moment
 * @param[out] m20 Second moment in X
 * @param[out] m02 Second moment in Y
 * @param[out] ia Major moment of inertia
 * @param[out] ib Minor moment of inertia
 */
void MomentsRegion2nd(const QRegion& region,
                       double& m11, double& m20, double& m02,
                       double& ia, double& ib);

/**
 * @brief Compute eccentricity features
 *
 * Equivalent to Halcon's eccentricity operator.
 *
 * @param region Input region
 * @param[out] anisometry Ra/Rb
 * @param[out] bulkiness pi*Ra*Rb/Area
 * @param[out] structureFactor Anisometry*Bulkiness-1
 */
void Eccentricity(const QRegion& region,
                   double& anisometry, double& bulkiness, double& structureFactor);

// =============================================================================
// Region Selection
// =============================================================================

/**
 * @brief Select regions by shape features
 *
 * Equivalent to Halcon's select_shape operator.
 *
 * @param regions Input regions
 * @param feature Feature to select by
 * @param operation Selection operation (And/Or)
 * @param minValue Minimum feature value
 * @param maxValue Maximum feature value
 * @return Selected regions
 *
 * @code
 * // Select circular blobs with area > 100
 * auto selected = SelectShape(blobs, ShapeFeature::Area,
 *                             SelectOperation::And, 100, 999999);
 * selected = SelectShape(selected, ShapeFeature::Circularity,
 *                        SelectOperation::And, 0.8, 1.0);
 * @endcode
 */
std::vector<QRegion> SelectShape(const std::vector<QRegion>& regions,
                                  ShapeFeature feature,
                                  SelectOperation operation,
                                  double minValue,
                                  double maxValue);

/**
 * @brief Select regions by string feature name (Halcon compatible)
 *
 * @param regions Input regions
 * @param features Feature name: "area", "circularity", "compactness", etc.
 * @param operation "and" or "or"
 * @param minValue Minimum value
 * @param maxValue Maximum value
 * @return Selected regions
 */
std::vector<QRegion> SelectShape(const std::vector<QRegion>& regions,
                                  const std::string& features,
                                  const std::string& operation,
                                  double minValue,
                                  double maxValue);

/**
 * @brief Select regions by area
 *
 * @param regions Input regions
 * @param minArea Minimum area
 * @param maxArea Maximum area
 * @return Selected regions
 */
std::vector<QRegion> SelectShapeArea(const std::vector<QRegion>& regions,
                                      int64_t minArea,
                                      int64_t maxArea);

/**
 * @brief Select regions by circularity
 */
std::vector<QRegion> SelectShapeCircularity(const std::vector<QRegion>& regions,
                                             double minCirc,
                                             double maxCirc);

/**
 * @brief Select regions by rectangularity
 */
std::vector<QRegion> SelectShapeRectangularity(const std::vector<QRegion>& regions,
                                                double minRect,
                                                double maxRect);

/**
 * @brief Select regions by standard deviation of shape features
 *
 * Equivalent to Halcon's select_shape_std operator.
 * Selects regions whose feature value is within N standard deviations of the mean.
 *
 * @param regions Input regions
 * @param feature Feature to analyze
 * @param deviationFactor Number of standard deviations (e.g., 1.0, 2.0)
 * @return Selected regions
 */
std::vector<QRegion> SelectShapeStd(const std::vector<QRegion>& regions,
                                     ShapeFeature feature,
                                     double deviationFactor);

/**
 * @brief Select regions by multiple features
 *
 * @param regions Input regions
 * @param features Vector of features
 * @param operation "and" = all must match, "or" = any must match
 * @param minValues Minimum values for each feature
 * @param maxValues Maximum values for each feature
 * @return Selected regions
 */
std::vector<QRegion> SelectShapeMulti(const std::vector<QRegion>& regions,
                                       const std::vector<ShapeFeature>& features,
                                       SelectOperation operation,
                                       const std::vector<double>& minValues,
                                       const std::vector<double>& maxValues);

/**
 * @brief Select regions by convexity
 */
std::vector<QRegion> SelectShapeConvexity(const std::vector<QRegion>& regions,
                                           double minConvex,
                                           double maxConvex);

/**
 * @brief Select regions by elongation
 */
std::vector<QRegion> SelectShapeElongation(const std::vector<QRegion>& regions,
                                            double minElong,
                                            double maxElong);

/**
 * @brief Select the N largest/smallest regions by area
 *
 * @param regions Input regions
 * @param n Number of regions to select
 * @param largest If true, select largest; if false, select smallest
 * @return Selected regions
 */
std::vector<QRegion> SelectShapeProto(const std::vector<QRegion>& regions,
                                       int32_t n,
                                       bool largest = true);

// =============================================================================
// Region Sorting
// =============================================================================

/**
 * @brief Sort regions by specified criteria
 *
 * Equivalent to Halcon's sort_region operator.
 *
 * @param regions Input regions
 * @param mode Sort mode
 * @param order "true" = ascending, "false" = descending
 * @return Sorted regions
 */
std::vector<QRegion> SortRegion(const std::vector<QRegion>& regions,
                                 SortMode mode,
                                 bool ascending = true);

/**
 * @brief Sort regions by string mode (Halcon compatible)
 */
std::vector<QRegion> SortRegion(const std::vector<QRegion>& regions,
                                 const std::string& sortMode,
                                 const std::string& order,
                                 const std::string& rowOrCol);

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Get single feature value for a region
 *
 * @param region Input region
 * @param feature Feature to compute
 * @return Feature value
 */
double GetRegionFeature(const QRegion& region, ShapeFeature feature);

/**
 * @brief Get feature values for multiple regions
 *
 * @param regions Input regions
 * @param feature Feature to compute
 * @return Vector of feature values
 */
std::vector<double> GetRegionFeatures(const std::vector<QRegion>& regions,
                                       ShapeFeature feature);

/**
 * @brief Parse feature name string to enum
 */
ShapeFeature ParseShapeFeature(const std::string& name);

/**
 * @brief Get feature name string
 */
std::string GetShapeFeatureName(ShapeFeature feature);

} // namespace Qi::Vision::Blob
