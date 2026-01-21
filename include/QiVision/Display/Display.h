#pragma once

/**
 * @file Display.h
 * @brief Image display and drawing functions (Halcon-style API)
 *
 * Provides image display and drawing primitives for debugging and visualization.
 * All functions follow Halcon naming conventions and coordinate order (row, col).
 *
 * Halcon API mapping:
 * - disp_image       -> DispImage (Show)
 * - disp_line        -> DispLine
 * - disp_circle      -> DispCircle
 * - disp_cross       -> DispCross
 * - disp_rectangle1  -> DispRectangle1
 * - disp_rectangle2  -> DispRectangle2
 * - disp_arrow       -> DispArrow
 * - disp_polygon     -> DispPolygon
 * - set_color        -> SetColor
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/QContour.h>
#include <QiVision/Core/QContourArray.h>
#include <QiVision/Core/Types.h>
#include <QiVision/Core/Draw.h>  // For Color struct

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
 *
 * Halcon equivalent: disp_image
 *
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
 *
 * Halcon equivalent: disp_line
 *
 * @param image Image to draw on (modified in place)
 * @param row1 Start row
 * @param col1 Start column
 * @param row2 End row
 * @param col2 End column
 * @param color Drawing color
 * @param thickness Line thickness (default: 1)
 */
void DispLine(QImage& image, double row1, double col1, double row2, double col2,
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
 *
 * Halcon equivalent: disp_circle
 *
 * @param image Image to draw on
 * @param row Center row
 * @param column Center column
 * @param radius Circle radius
 * @param color Drawing color
 * @param thickness Line thickness (default: 1)
 */
void DispCircle(QImage& image, double row, double column, double radius,
                const Scalar& color = Scalar::Green(), int32_t thickness = 1);

/**
 * @brief Draw a circle from Circle2d
 */
void DispCircle(QImage& image, const Circle2d& circle,
                const Scalar& color = Scalar::Green(), int32_t thickness = 1);

/**
 * @brief Draw an ellipse
 *
 * Halcon equivalent: disp_ellipse
 *
 * @param image Image to draw on
 * @param row Center row
 * @param column Center column
 * @param phi Orientation angle (radians)
 * @param ra Semi-axis along phi
 * @param rb Semi-axis perpendicular to phi
 * @param color Drawing color
 * @param thickness Line thickness
 */
void DispEllipse(QImage& image, double row, double column, double phi,
                 double ra, double rb,
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
 *
 * Halcon equivalent: disp_rectangle1
 *
 * @param image Image to draw on
 * @param row1 Top-left row
 * @param col1 Top-left column
 * @param row2 Bottom-right row
 * @param col2 Bottom-right column
 * @param color Drawing color
 * @param thickness Line thickness
 */
void DispRectangle1(QImage& image, double row1, double col1, double row2, double col2,
                    const Scalar& color = Scalar::Green(), int32_t thickness = 1);

/**
 * @brief Draw an axis-aligned rectangle from Rect2i
 */
void DispRectangle1(QImage& image, const Rect2i& rect,
                    const Scalar& color = Scalar::Green(), int32_t thickness = 1);

/**
 * @brief Draw a rotated rectangle
 *
 * Halcon equivalent: disp_rectangle2
 *
 * @param image Image to draw on
 * @param row Center row
 * @param column Center column
 * @param phi Orientation angle (radians)
 * @param length1 Half-length along phi
 * @param length2 Half-length perpendicular to phi
 * @param color Drawing color
 * @param thickness Line thickness
 */
void DispRectangle2(QImage& image, double row, double column, double phi,
                    double length1, double length2,
                    const Scalar& color = Scalar::Green(), int32_t thickness = 1);

// =============================================================================
// Drawing Primitives - Markers
// =============================================================================

/**
 * @brief Draw a cross marker
 *
 * Halcon equivalent: disp_cross
 *
 * @param image Image to draw on
 * @param row Center row
 * @param column Center column
 * @param size Cross arm length
 * @param angle Rotation angle (radians, default: 0)
 * @param color Drawing color
 * @param thickness Line thickness
 */
void DispCross(QImage& image, double row, double column, int32_t size,
               double angle = 0.0,
               const Scalar& color = Scalar::Green(), int32_t thickness = 1);

/**
 * @brief Draw an arrow
 *
 * Halcon equivalent: disp_arrow
 *
 * @param image Image to draw on
 * @param row1 Start row
 * @param col1 Start column
 * @param row2 End row (arrow head)
 * @param col2 End column (arrow head)
 * @param headSize Arrow head size
 * @param color Drawing color
 * @param thickness Line thickness
 */
void DispArrow(QImage& image, double row1, double col1, double row2, double col2,
               double headSize = 10.0,
               const Scalar& color = Scalar::Green(), int32_t thickness = 1);

// =============================================================================
// Drawing Primitives - Polygons and Contours
// =============================================================================

/**
 * @brief Draw a polygon
 *
 * Halcon equivalent: disp_polygon
 *
 * @param image Image to draw on
 * @param rows Row coordinates
 * @param cols Column coordinates
 * @param color Drawing color
 * @param thickness Line thickness
 */
void DispPolygon(QImage& image, const std::vector<double>& rows,
                 const std::vector<double>& cols,
                 const Scalar& color = Scalar::Green(), int32_t thickness = 1);

/**
 * @brief Draw a contour (XLD)
 *
 * Halcon equivalent: disp_xld
 *
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
 *
 * @param image Image to draw on
 * @param row Point row
 * @param column Point column
 * @param color Drawing color
 */
void DispPoint(QImage& image, double row, double column,
               const Scalar& color = Scalar::Green());

/**
 * @brief Draw multiple points
 *
 * @param image Image to draw on
 * @param rows Row coordinates
 * @param cols Column coordinates
 * @param color Drawing color
 */
void DispPoints(QImage& image, const std::vector<double>& rows,
                const std::vector<double>& cols,
                const Scalar& color = Scalar::Green());

// =============================================================================
// Drawing Primitives - Text
// =============================================================================

/**
 * @brief Draw text
 *
 * Halcon equivalent: disp_message / write_string
 *
 * @param image Image to draw on
 * @param row Text position row
 * @param column Text position column
 * @param text Text string
 * @param color Text color
 * @param scale Font scale (default: 1)
 */
void DispText(QImage& image, double row, double column, const std::string& text,
              const Scalar& color = Scalar::Green(), int32_t scale = 1);

// =============================================================================
// High-Level Drawing Functions
// =============================================================================

/**
 * @brief Draw match result with cross and angle indicator
 *
 * @param image Image to draw on
 * @param row Match row
 * @param column Match column
 * @param angle Match angle (radians)
 * @param score Match score
 * @param color Drawing color
 * @param markerSize Cross marker size
 */
void DispMatchResult(QImage& image, double row, double column, double angle,
                     double score = 1.0,
                     const Scalar& color = Scalar::Green(),
                     int32_t markerSize = 20);

/**
 * @brief Draw edge measurement result
 *
 * @param image Image to draw on
 * @param row Edge row
 * @param column Edge column
 * @param color Drawing color
 * @param markerSize Cross marker size
 */
void DispEdgeResult(QImage& image, double row, double column,
                    const Scalar& color = Scalar::Green(),
                    int32_t markerSize = 5);

} // namespace Qi::Vision
