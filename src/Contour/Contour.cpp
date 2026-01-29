/**
 * @file Contour.cpp
 * @brief XLD Contour operations implementation
 *
 * Wraps Internal layer contour operations into Halcon-style public API.
 */

#include <QiVision/Contour/Contour.h>

#include <QiVision/Internal/ContourProcess.h>
#include <QiVision/Internal/ContourAnalysis.h>
#include <QiVision/Internal/ContourSelect.h>
#include <QiVision/Internal/ContourSegment.h>
#include <QiVision/Internal/ContourConvert.h>
#include <QiVision/Internal/Fitting.h>
#include <QiVision/Internal/Distance.h>
#include <QiVision/Core/Constants.h>
#include <QiVision/Core/Exception.h>

#include <cmath>
#include <algorithm>

namespace Qi::Vision::Contour {

// =============================================================================
// Contour Processing
// =============================================================================

void SmoothContoursXld(
    const QContourArray& contours,
    QContourArray& smoothed,
    int32_t numPoints)
{
    smoothed.Clear();
    smoothed.Reserve(contours.Size());

    Internal::MovingAverageSmoothParams params;
    params.windowSize = numPoints;

    for (size_t i = 0; i < contours.Size(); ++i) {
        QContour result = Internal::SmoothContourMovingAverage(contours[i], params);
        smoothed.Add(std::move(result));
    }
}

void SmoothContoursGaussXld(
    const QContourArray& contours,
    QContourArray& smoothed,
    double sigma)
{
    smoothed.Clear();
    smoothed.Reserve(contours.Size());

    Internal::GaussianSmoothParams params;
    params.sigma = sigma;

    for (size_t i = 0; i < contours.Size(); ++i) {
        QContour result = Internal::SmoothContourGaussian(contours[i], params);
        smoothed.Add(std::move(result));
    }
}

void SimplifyContoursXld(
    const QContourArray& contours,
    QContourArray& simplified,
    double epsilon)
{
    simplified.Clear();
    simplified.Reserve(contours.Size());

    Internal::DouglasPeuckerParams params;
    params.tolerance = epsilon;

    for (size_t i = 0; i < contours.Size(); ++i) {
        QContour result = Internal::SimplifyContourDouglasPeucker(contours[i], params);
        simplified.Add(std::move(result));
    }
}

void ResampleContoursXld(
    const QContourArray& contours,
    QContourArray& resampled,
    double distance)
{
    resampled.Clear();
    resampled.Reserve(contours.Size());

    Internal::ResampleByDistanceParams params;
    params.distance = distance;

    for (size_t i = 0; i < contours.Size(); ++i) {
        QContour result = Internal::ResampleContourByDistance(contours[i], params);
        resampled.Add(std::move(result));
    }
}

void ResampleContoursNumXld(
    const QContourArray& contours,
    QContourArray& resampled,
    int32_t numPoints)
{
    resampled.Clear();
    resampled.Reserve(contours.Size());

    Internal::ResampleByCountParams params;
    params.count = static_cast<size_t>(numPoints);

    for (size_t i = 0; i < contours.Size(); ++i) {
        QContour result = Internal::ResampleContourByCount(contours[i], params);
        resampled.Add(std::move(result));
    }
}

void CloseContoursXld(
    const QContourArray& contours,
    QContourArray& closed)
{
    closed.Clear();
    closed.Reserve(contours.Size());

    for (size_t i = 0; i < contours.Size(); ++i) {
        QContour result = Internal::CloseContour(contours[i]);
        closed.Add(std::move(result));
    }
}

void ReverseContoursXld(
    const QContourArray& contours,
    QContourArray& reversed)
{
    reversed.Clear();
    reversed.Reserve(contours.Size());

    for (size_t i = 0; i < contours.Size(); ++i) {
        QContour result = Internal::ReverseContour(contours[i]);
        reversed.Add(std::move(result));
    }
}

// =============================================================================
// Contour Analysis - Basic Properties
// =============================================================================

double LengthXld(const QContour& contour)
{
    return Internal::ContourLength(contour);
}

std::vector<double> LengthXld(const QContourArray& contours)
{
    std::vector<double> lengths;
    lengths.reserve(contours.Size());

    for (size_t i = 0; i < contours.Size(); ++i) {
        lengths.push_back(Internal::ContourLength(contours[i]));
    }

    return lengths;
}

double AreaCenterXld(const QContour& contour, double& row, double& column)
{
    auto result = Internal::ContourAreaCenter(contour);
    row = result.centroid.y;
    column = result.centroid.x;
    return result.area;
}

double PerimeterXld(const QContour& contour)
{
    return Internal::ContourPerimeter(contour);
}

// =============================================================================
// Contour Analysis - Bounding Geometry
// =============================================================================

void SmallestRectangle1Xld(
    const QContour& contour,
    double& row1, double& col1,
    double& row2, double& col2)
{
    Rect2d bbox = Internal::ContourBoundingBox(contour);
    row1 = bbox.y;
    col1 = bbox.x;
    row2 = bbox.y + bbox.height;
    col2 = bbox.x + bbox.width;
}

void SmallestRectangle2Xld(
    const QContour& contour,
    double& row, double& column,
    double& phi,
    double& length1, double& length2)
{
    auto result = Internal::ContourMinAreaRect(contour);
    if (result.has_value()) {
        row = result->center.y;
        column = result->center.x;
        phi = result->angle;
        length1 = result->width / 2.0;
        length2 = result->height / 2.0;
    } else {
        // Fallback to AABB
        Rect2d bbox = Internal::ContourBoundingBox(contour);
        row = bbox.y + bbox.height / 2.0;
        column = bbox.x + bbox.width / 2.0;
        phi = 0.0;
        length1 = bbox.width / 2.0;
        length2 = bbox.height / 2.0;
    }
}

void SmallestCircleXld(
    const QContour& contour,
    double& row, double& column,
    double& radius)
{
    auto result = Internal::ContourMinEnclosingCircle(contour);
    if (result.has_value()) {
        row = result->center.y;
        column = result->center.x;
        radius = result->radius;
    } else {
        // Fallback to centroid
        Point2d center = Internal::ContourCentroid(contour);
        row = center.y;
        column = center.x;
        radius = 0.0;
    }
}

// =============================================================================
// Contour Analysis - Curvature
// =============================================================================

std::vector<double> CurvatureXld(
    const QContour& contour,
    int32_t windowSize)
{
    return Internal::ComputeContourCurvature(
        contour,
        Internal::CurvatureMethod::ThreePoint,
        windowSize
    );
}

double MeanCurvatureXld(const QContour& contour)
{
    return Internal::ContourMeanCurvature(contour);
}

double MaxCurvatureXld(const QContour& contour)
{
    return Internal::ContourMaxCurvature(contour);
}

// =============================================================================
// Contour Analysis - Moments
// =============================================================================

void MomentsXld(
    const QContour& contour,
    double& m00, double& m10, double& m01,
    double& m20, double& m11, double& m02)
{
    auto moments = Internal::ContourMoments(contour);
    m00 = moments.m00;
    m10 = moments.m10;
    m01 = moments.m01;
    m20 = moments.m20;
    m11 = moments.m11;
    m02 = moments.m02;
}

void CentralMomentsXld(
    const QContour& contour,
    double& mu00, double& mu20,
    double& mu11, double& mu02)
{
    auto moments = Internal::ContourCentralMoments(contour);
    mu00 = moments.mu00;
    mu20 = moments.mu20;
    mu11 = moments.mu11;
    mu02 = moments.mu02;
}

double OrientationXld(const QContour& contour)
{
    return Internal::ContourOrientation(contour);
}

// =============================================================================
// Contour Analysis - Shape Descriptors
// =============================================================================

double CircularityXld(const QContour& contour)
{
    return Internal::ContourCircularity(contour);
}

double ConvexityXld(const QContour& contour)
{
    return Internal::ContourConvexity(contour);
}

double SolidityXld(const QContour& contour)
{
    return Internal::ContourSolidity(contour);
}

double EccentricityXld(const QContour& contour)
{
    return Internal::ContourEccentricity(contour);
}

double CompactnessXld(const QContour& contour)
{
    return Internal::ContourCompactness(contour);
}

// =============================================================================
// Contour Fitting
// =============================================================================

bool FitEllipseContourXld(
    const QContour& contour,
    double& row, double& column,
    double& phi, double& ra, double& rb)
{
    if (contour.Size() < 5) {
        return false;
    }

    // Extract points
    std::vector<Point2d> points;
    points.reserve(contour.Size());
    for (size_t i = 0; i < contour.Size(); ++i) {
        const auto& pt = contour.GetPoint(i);
        points.push_back({pt.x, pt.y});
    }

    // Fit ellipse
    auto result = Internal::FitEllipseFitzgibbon(points);
    if (!result.success) {
        return false;
    }

    row = result.ellipse.center.y;
    column = result.ellipse.center.x;
    phi = result.ellipse.angle;
    ra = result.ellipse.a;
    rb = result.ellipse.b;

    return true;
}

bool FitLineContourXld(
    const QContour& contour,
    double& rowBegin, double& colBegin,
    double& rowEnd, double& colEnd,
    double& row, double& column,
    double& phi)
{
    if (contour.Size() < 2) {
        return false;
    }

    // Extract points
    std::vector<Point2d> points;
    points.reserve(contour.Size());
    for (size_t i = 0; i < contour.Size(); ++i) {
        const auto& pt = contour.GetPoint(i);
        points.push_back({pt.x, pt.y});
    }

    // Fit line
    auto result = Internal::FitLine(points);
    if (!result.success) {
        return false;
    }

    // Get line endpoints from contour extent
    Point2d first = contour.GetPoint(0);
    Point2d last = contour.GetPoint(contour.Size() - 1);

    // Get line angle and direction
    double lineAngle = result.line.Angle();
    Point2d dir = result.line.Direction();

    // Compute a reference point on the line (closest to origin)
    // For ax + by + c = 0, the closest point to origin is (-a*c, -b*c)
    Point2d refPoint(-result.line.a * result.line.c, -result.line.b * result.line.c);

    // Project endpoints onto the fitted line
    auto projectPoint = [&](const Point2d& pt) -> Point2d {
        double t = (pt.x - refPoint.x) * dir.x + (pt.y - refPoint.y) * dir.y;
        return {refPoint.x + t * dir.x, refPoint.y + t * dir.y};
    };

    Point2d projFirst = projectPoint(first);
    Point2d projLast = projectPoint(last);

    // Compute center point
    Point2d center = {(projFirst.x + projLast.x) / 2.0, (projFirst.y + projLast.y) / 2.0};

    rowBegin = projFirst.y;
    colBegin = projFirst.x;
    rowEnd = projLast.y;
    colEnd = projLast.x;
    row = center.y;
    column = center.x;
    phi = lineAngle;

    return true;
}

bool FitCircleContourXld(
    const QContour& contour,
    double& row, double& column,
    double& radius,
    double& startPhi, double& endPhi,
    const std::string& algorithm)
{
    if (contour.Size() < 3) {
        return false;
    }

    // Extract points
    std::vector<Point2d> points;
    points.reserve(contour.Size());
    for (size_t i = 0; i < contour.Size(); ++i) {
        const auto& pt = contour.GetPoint(i);
        points.push_back({pt.x, pt.y});
    }

    // Fit circle
    Internal::CircleFitResult result;
    if (algorithm == "geometric") {
        result = Internal::FitCircleGeometric(points);
    } else {
        result = Internal::FitCircleAlgebraic(points);
    }

    if (!result.success) {
        return false;
    }

    row = result.circle.center.y;
    column = result.circle.center.x;
    radius = result.circle.radius;

    // Compute arc angles from first and last points
    Point2d first = contour.GetPoint(0);
    Point2d last = contour.GetPoint(contour.Size() - 1);

    startPhi = std::atan2(first.y - result.circle.center.y,
                          first.x - result.circle.center.x);
    endPhi = std::atan2(last.y - result.circle.center.y,
                        last.x - result.circle.center.x);

    return true;
}

void ConvexHullXld(const QContour& contour, QContour& hull)
{
    hull = Internal::ContourConvexHull(contour);
}

// =============================================================================
// Contour Selection
// =============================================================================

void SelectContoursXld(
    const QContourArray& contours,
    QContourArray& selected,
    const std::string& feature,
    double minValue,
    double maxValue)
{
    // Map feature string to enum
    Internal::ContourFeature featureEnum = Internal::StringToContourFeature(feature);

    selected = Internal::SelectContoursByFeature(contours, featureEnum, minValue, maxValue);
}

void SelectClosedXld(
    const QContourArray& contours,
    QContourArray& closed)
{
    closed = Internal::SelectClosedContours(contours);
}

void SelectOpenXld(
    const QContourArray& contours,
    QContourArray& open)
{
    open = Internal::SelectOpenContours(contours);
}

void SortContoursXld(
    const QContourArray& contours,
    QContourArray& sorted,
    const std::string& feature,
    bool ascending)
{
    Internal::ContourFeature featureEnum = Internal::StringToContourFeature(feature);
    sorted = Internal::SortContoursByFeature(contours, featureEnum, ascending);
}

void SelectTopContoursXld(
    const QContourArray& contours,
    QContourArray& selected,
    const std::string& feature,
    int32_t count,
    bool largest)
{
    Internal::ContourFeature featureEnum = Internal::StringToContourFeature(feature);
    selected = Internal::SelectTopContoursByFeature(contours, featureEnum, static_cast<size_t>(count), largest);
}

// =============================================================================
// Contour Segmentation
// =============================================================================

void SegmentContoursXld(
    const QContour& contour,
    QContourArray& segments,
    const std::string& mode,
    double maxLineDev,
    double maxArcDev)
{
    Internal::SegmentParams params;

    if (mode == "lines") {
        params.mode = Internal::SegmentMode::LinesOnly;
    } else if (mode == "circles") {
        params.mode = Internal::SegmentMode::ArcsOnly;
    } else {
        params.mode = Internal::SegmentMode::LinesAndArcs;
    }

    params.maxLineError = maxLineDev;
    params.maxArcError = maxArcDev;

    auto result = Internal::SegmentContour(contour, params);
    segments = Internal::PrimitivesToContours(result, 1.0);
}

void SplitContoursXld(
    const QContourArray& contours,
    QContourArray& split,
    double maxAngle)
{
    split.Clear();

    for (size_t i = 0; i < contours.Size(); ++i) {
        // Detect corner points
        double curvatureThreshold = 1.0 / (10.0 * maxAngle);  // Approximate conversion
        auto corners = Internal::DetectCorners(contours[i], curvatureThreshold);

        if (corners.empty()) {
            split.Add(contours[i]);
        } else {
            auto subContours = Internal::SplitContourAtIndices(contours[i], corners);
            for (size_t j = 0; j < subContours.Size(); ++j) {
                split.Add(subContours[j]);
            }
        }
    }
}

std::vector<int32_t> DetectCornersXld(
    const QContour& contour,
    double curvatureThreshold)
{
    auto corners = Internal::DetectCorners(contour, curvatureThreshold);

    std::vector<int32_t> result;
    result.reserve(corners.size());
    for (size_t idx : corners) {
        result.push_back(static_cast<int32_t>(idx));
    }
    return result;
}

// =============================================================================
// Contour-Region Conversion
// =============================================================================

void GenContourRegionXld(
    const QRegion& region,
    QContourArray& contours,
    const std::string& mode)
{
    Internal::BoundaryMode bmode = Internal::BoundaryMode::Outer;
    if (mode == "border_holes") {
        bmode = Internal::BoundaryMode::Both;
    }

    contours = Internal::RegionToContours(region, bmode);
}

void GenRegionContourXld(
    const QContour& contour,
    QRegion& region,
    const std::string& mode)
{
    Internal::ContourFillMode fmode = Internal::ContourFillMode::Filled;
    if (mode == "margin") {
        fmode = Internal::ContourFillMode::Margin;
    }

    region = Internal::ContourToRegion(contour, fmode);
}

void GenRegionContoursXld(
    const QContourArray& contours,
    QRegion& region,
    const std::string& mode)
{
    Internal::ContourFillMode fmode = Internal::ContourFillMode::Filled;
    if (mode == "margin") {
        fmode = Internal::ContourFillMode::Margin;
    }

    region = Internal::ContoursToRegion(contours, fmode);
}

// =============================================================================
// Contour Generation
// =============================================================================

QContour GenContourPolygonXld(const std::vector<Point2d>& points)
{
    QContour contour;
    contour.Reserve(points.size());

    for (const auto& pt : points) {
        contour.AddPoint(pt.x, pt.y);
    }

    return contour;
}

QContour GenContourPolygonXld(
    const std::vector<double>& rows,
    const std::vector<double>& cols)
{
    if (rows.size() != cols.size()) {
        throw Exception("Rows and columns must have same size");
    }

    QContour contour;
    contour.Reserve(rows.size());

    for (size_t i = 0; i < rows.size(); ++i) {
        contour.AddPoint(cols[i], rows[i]);
    }

    return contour;
}

QContour GenCircleContourXld(
    double row, double column,
    double radius,
    double startAngle,
    double endAngle,
    const std::string& resolution,
    double stepAngle)
{
    QContour contour;

    bool ccw = (resolution == "positive");

    // Normalize angles
    double angleExtent = endAngle - startAngle;
    if (std::abs(angleExtent) > 2.0 * PI) {
        angleExtent = 2.0 * PI;
    }

    int32_t numPoints = static_cast<int32_t>(std::abs(angleExtent) / stepAngle) + 1;
    numPoints = std::max(numPoints, 3);

    contour.Reserve(numPoints);

    double step = angleExtent / (numPoints - 1);
    if (!ccw) {
        step = -step;
    }

    for (int32_t i = 0; i < numPoints; ++i) {
        double angle = startAngle + i * step;
        double x = column + radius * std::cos(angle);
        double y = row + radius * std::sin(angle);
        contour.AddPoint(x, y);
    }

    // Close if full circle
    if (std::abs(angleExtent - 2.0 * PI) < 1e-6) {
        contour.SetClosed(true);
    }

    return contour;
}

QContour GenEllipseContourXld(
    double row, double column,
    double phi,
    double ra, double rb,
    double startAngle,
    double endAngle,
    double stepAngle)
{
    QContour contour;

    // Normalize angles
    double angleExtent = endAngle - startAngle;
    if (std::abs(angleExtent) > 2.0 * PI) {
        angleExtent = 2.0 * PI;
    }

    int32_t numPoints = static_cast<int32_t>(std::abs(angleExtent) / stepAngle) + 1;
    numPoints = std::max(numPoints, 3);

    contour.Reserve(numPoints);

    double cosPhi = std::cos(phi);
    double sinPhi = std::sin(phi);
    double step = angleExtent / (numPoints - 1);

    for (int32_t i = 0; i < numPoints; ++i) {
        double t = startAngle + i * step;

        // Ellipse in local coordinates
        double localX = ra * std::cos(t);
        double localY = rb * std::sin(t);

        // Rotate and translate
        double x = column + localX * cosPhi - localY * sinPhi;
        double y = row + localX * sinPhi + localY * cosPhi;

        contour.AddPoint(x, y);
    }

    // Close if full ellipse
    if (std::abs(angleExtent - 2.0 * PI) < 1e-6) {
        contour.SetClosed(true);
    }

    return contour;
}

QContour GenRectangle2ContourXld(
    double row, double column,
    double phi,
    double length1, double length2)
{
    QContour contour;
    contour.Reserve(4);

    double cosPhi = std::cos(phi);
    double sinPhi = std::sin(phi);

    // Four corners in local coordinates
    double dx1 = length1 * cosPhi;
    double dy1 = length1 * sinPhi;
    double dx2 = length2 * (-sinPhi);
    double dy2 = length2 * cosPhi;

    // Add corners (CCW order)
    contour.AddPoint(column - dx1 - dx2, row - dy1 - dy2);
    contour.AddPoint(column + dx1 - dx2, row + dy1 - dy2);
    contour.AddPoint(column + dx1 + dx2, row + dy1 + dy2);
    contour.AddPoint(column - dx1 + dx2, row - dy1 + dy2);

    contour.SetClosed(true);

    return contour;
}

QContour GenRectangle1ContourXld(
    double row1, double col1,
    double row2, double col2)
{
    QContour contour;
    contour.Reserve(4);

    contour.AddPoint(col1, row1);
    contour.AddPoint(col2, row1);
    contour.AddPoint(col2, row2);
    contour.AddPoint(col1, row2);

    contour.SetClosed(true);

    return contour;
}

QContour GenLineContourXld(
    double row1, double col1,
    double row2, double col2)
{
    QContour contour;
    contour.Reserve(2);

    contour.AddPoint(col1, row1);
    contour.AddPoint(col2, row2);

    contour.SetClosed(false);

    return contour;
}

// =============================================================================
// Utility Functions
// =============================================================================

int32_t CountPointsXld(const QContour& contour)
{
    return static_cast<int32_t>(contour.Size());
}

std::vector<int32_t> CountPointsXld(const QContourArray& contours)
{
    std::vector<int32_t> counts;
    counts.reserve(contours.Size());

    for (size_t i = 0; i < contours.Size(); ++i) {
        counts.push_back(static_cast<int32_t>(contours[i].Size()));
    }

    return counts;
}

void GetContourXld(
    const QContour& contour,
    std::vector<double>& rows,
    std::vector<double>& cols)
{
    size_t n = contour.Size();
    rows.resize(n);
    cols.resize(n);

    for (size_t i = 0; i < n; ++i) {
        const auto& pt = contour.GetPoint(i);
        rows[i] = pt.y;
        cols[i] = pt.x;
    }
}

bool TestPointXld(
    const QContour& contour,
    double row, double column)
{
    return Internal::IsPointInsideContour(contour, {column, row});
}

double DistancePointXld(
    const QContour& contour,
    double row, double column)
{
    // Extract contour points
    std::vector<Point2d> points = contour.GetPoints();
    auto result = Internal::DistancePointToContour({column, row}, points, contour.IsClosed());
    return result.distance;
}

void UnionContoursXld(
    const QContourArray& contours1,
    const QContourArray& contours2,
    QContourArray& result)
{
    result.Clear();
    result.Reserve(contours1.Size() + contours2.Size());

    for (size_t i = 0; i < contours1.Size(); ++i) {
        result.Add(contours1[i]);
    }
    for (size_t i = 0; i < contours2.Size(); ++i) {
        result.Add(contours2[i]);
    }
}

void SelectObjXld(
    const QContourArray& contours,
    QContourArray& selected,
    const std::vector<int32_t>& indices)
{
    selected.Clear();
    selected.Reserve(indices.size());

    for (int32_t idx : indices) {
        if (idx >= 0 && static_cast<size_t>(idx) < contours.Size()) {
            selected.Add(contours[static_cast<size_t>(idx)]);
        }
    }
}

QContour SelectObjXld(
    const QContourArray& contours,
    int32_t index)
{
    if (index < 0 || static_cast<size_t>(index) >= contours.Size()) {
        throw Exception("Index out of range");
    }
    return contours[static_cast<size_t>(index)];
}

int32_t CountObjXld(const QContourArray& contours)
{
    return static_cast<int32_t>(contours.Size());
}

} // namespace Qi::Vision::Contour
