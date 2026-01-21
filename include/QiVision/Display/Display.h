#pragma once

/**
 * @file Display.h
 * @brief Image display and drawing functions
 *
 * Provides image display and drawing primitives for debugging and visualization.
 * All functions use (x, y) coordinate order (OpenCV style).
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/QContour.h>
#include <QiVision/Core/QContourArray.h>
#include <QiVision/Core/Types.h>
#include <QiVision/Display/Draw.h>  // For Scalar struct

#include <string>
#include <vector>

namespace Qi::Vision {

// Forward declarations
namespace Matching {
    struct MatchResult;
}
namespace Measure {
    struct EdgeResult;
    struct PairResult;
}

// =============================================================================
// Image Display Functions
// =============================================================================

/**
 * @brief Display an image (save and open with system viewer)
 * @param image Image to display
 * @param title Window title (used as filename)
 * @return true if successful
 */
bool DispImage(const QImage& image, const std::string& title = "image");

/**
 * @brief Set the output directory for displayed images
 * @param path Directory path
 */
void SetDispOutputDir(const std::string& path);

/**
 * @brief Clean up temporary displayed images
 */
void CleanDispImages();

// =============================================================================
// Drawing Primitives - Lines
// =============================================================================

/**
 * @brief Draw a line segment
 * @param image Image to draw on (modified in place)
 * @param x1 Start x coordinate
 * @param y1 Start y coordinate
 * @param x2 End x coordinate
 * @param y2 End y coordinate
 * @param color Drawing color
 * @param thickness Line thickness (default: 1)
 */
void DispLine(QImage& image, double x1, double y1, double x2, double y2,
              const Scalar& color = Scalar::Green(), int32_t thickness = 1);

/**
 * @brief Draw a line from Line2d
 */
void DispLine(QImage& image, const Line2d& line, double length,
              const Scalar& color = Scalar::Green(), int32_t thickness = 1);

// =============================================================================
// Drawing Primitives - Circles and Arcs
// =============================================================================

/**
 * @brief Draw a circle
 * @param image Image to draw on
 * @param cx Center x coordinate
 * @param cy Center y coordinate
 * @param radius Circle radius
 * @param color Drawing color
 * @param thickness Line thickness (default: 1)
 */
void DispCircle(QImage& image, double cx, double cy, double radius,
                const Scalar& color = Scalar::Green(), int32_t thickness = 1);

/**
 * @brief Draw a circle from Circle2d
 */
void DispCircle(QImage& image, const Circle2d& circle,
                const Scalar& color = Scalar::Green(), int32_t thickness = 1);

/**
 * @brief Draw an ellipse
 * @param image Image to draw on
 * @param cx Center x coordinate
 * @param cy Center y coordinate
 * @param angle Orientation angle (radians)
 * @param radiusX Semi-axis along angle
 * @param radiusY Semi-axis perpendicular to angle
 * @param color Drawing color
 * @param thickness Line thickness
 */
void DispEllipse(QImage& image, double cx, double cy, double angle,
                 double radiusX, double radiusY,
                 const Scalar& color = Scalar::Green(), int32_t thickness = 1);

/**
 * @brief Draw an ellipse from Ellipse2d
 */
void DispEllipse(QImage& image, const Ellipse2d& ellipse,
                 const Scalar& color = Scalar::Green(), int32_t thickness = 1);

// =============================================================================
// Drawing Primitives - Rectangles
// =============================================================================

/**
 * @brief Draw an axis-aligned rectangle
 * @param image Image to draw on
 * @param x1 Top-left x coordinate
 * @param y1 Top-left y coordinate
 * @param x2 Bottom-right x coordinate
 * @param y2 Bottom-right y coordinate
 * @param color Drawing color
 * @param thickness Line thickness
 */
void DispRectangle1(QImage& image, double x1, double y1, double x2, double y2,
                    const Scalar& color = Scalar::Green(), int32_t thickness = 1);

/**
 * @brief Draw an axis-aligned rectangle from Rect2i
 */
void DispRectangle1(QImage& image, const Rect2i& rect,
                    const Scalar& color = Scalar::Green(), int32_t thickness = 1);

/**
 * @brief Draw a rotated rectangle
 * @param image Image to draw on
 * @param cx Center x coordinate
 * @param cy Center y coordinate
 * @param angle Orientation angle (radians)
 * @param halfWidth Half-width along angle
 * @param halfHeight Half-height perpendicular to angle
 * @param color Drawing color
 * @param thickness Line thickness
 */
void DispRectangle2(QImage& image, double cx, double cy, double angle,
                    double halfWidth, double halfHeight,
                    const Scalar& color = Scalar::Green(), int32_t thickness = 1);

// =============================================================================
// Drawing Primitives - Markers
// =============================================================================

/**
 * @brief Draw a cross marker
 * @param image Image to draw on
 * @param cx Center x coordinate
 * @param cy Center y coordinate
 * @param size Cross arm length
 * @param angle Rotation angle (radians, default: 0)
 * @param color Drawing color
 * @param thickness Line thickness
 */
void DispCross(QImage& image, double cx, double cy, int32_t size,
               double angle = 0.0,
               const Scalar& color = Scalar::Green(), int32_t thickness = 1);

/**
 * @brief Draw an arrow
 * @param image Image to draw on
 * @param x1 Start x coordinate
 * @param y1 Start y coordinate
 * @param x2 End x coordinate (arrow head)
 * @param y2 End y coordinate (arrow head)
 * @param headSize Arrow head size
 * @param color Drawing color
 * @param thickness Line thickness
 */
void DispArrow(QImage& image, double x1, double y1, double x2, double y2,
               double headSize = 10.0,
               const Scalar& color = Scalar::Green(), int32_t thickness = 1);

// =============================================================================
// Drawing Primitives - Polygons and Contours
// =============================================================================

/**
 * @brief Draw a polygon
 * @param image Image to draw on
 * @param xs X coordinates
 * @param ys Y coordinates
 * @param color Drawing color
 * @param thickness Line thickness
 */
void DispPolygon(QImage& image, const std::vector<double>& xs,
                 const std::vector<double>& ys,
                 const Scalar& color = Scalar::Green(), int32_t thickness = 1);

/**
 * @brief Draw a contour (XLD)
 * @param image Image to draw on
 * @param contour Contour to draw
 * @param color Drawing color
 * @param thickness Line thickness
 */
void DispContour(QImage& image, const QContour& contour,
                 const Scalar& color = Scalar::Green(), int32_t thickness = 1);

/**
 * @brief Draw multiple contours
 */
void DispContours(QImage& image, const QContourArray& contours,
                  const Scalar& color = Scalar::Green(), int32_t thickness = 1);

// =============================================================================
// Drawing Primitives - Points
// =============================================================================

/**
 * @brief Draw a single point
 * @param image Image to draw on
 * @param x Point x coordinate
 * @param y Point y coordinate
 * @param color Drawing color
 */
void DispPoint(QImage& image, double x, double y,
               const Scalar& color = Scalar::Green());

/**
 * @brief Draw multiple points
 * @param image Image to draw on
 * @param xs X coordinates
 * @param ys Y coordinates
 * @param color Drawing color
 */
void DispPoints(QImage& image, const std::vector<double>& xs,
                const std::vector<double>& ys,
                const Scalar& color = Scalar::Green());

// =============================================================================
// Drawing Primitives - Text
// =============================================================================

/**
 * @brief Draw text
 * @param image Image to draw on
 * @param x Text position x coordinate
 * @param y Text position y coordinate
 * @param text Text string
 * @param color Text color
 * @param scale Font scale (default: 1)
 */
void DispText(QImage& image, double x, double y, const std::string& text,
              const Scalar& color = Scalar::Green(), int32_t scale = 1);

// =============================================================================
// High-Level Drawing Functions
// =============================================================================

/**
 * @brief Draw match result with cross and angle indicator
 * @param image Image to draw on
 * @param x Match x coordinate
 * @param y Match y coordinate
 * @param angle Match angle (radians)
 * @param score Match score
 * @param color Drawing color
 * @param markerSize Cross marker size
 */
void DispMatchResult(QImage& image, double x, double y, double angle,
                     double score = 1.0,
                     const Scalar& color = Scalar::Green(),
                     int32_t markerSize = 20);

/**
 * @brief Draw edge measurement result
 * @param image Image to draw on
 * @param x Edge x coordinate
 * @param y Edge y coordinate
 * @param color Drawing color
 * @param markerSize Cross marker size
 */
void DispEdgeResult(QImage& image, double x, double y,
                    const Scalar& color = Scalar::Green(),
                    int32_t markerSize = 5);

} // namespace Qi::Vision
