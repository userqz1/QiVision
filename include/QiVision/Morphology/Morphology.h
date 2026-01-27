#pragma once

/**
 * @file Morphology.h
 * @brief Morphological operations (Halcon-style API)
 *
 * This module provides:
 * - Binary morphology on regions (QRegion)
 * - Gray-scale morphology on images (QImage)
 * - Structuring element creation
 *
 * API Style: void Func(const QImage& in, QImage& out, params...)
 *            QRegion Func(const QRegion& in, params...)
 *
 * Halcon reference operators:
 * - dilation1, erosion1, dilation_circle, erosion_circle
 * - opening, closing, opening_circle, closing_circle
 * - gray_dilation, gray_erosion, gray_opening, gray_closing
 * - gray_tophat, gray_bothat
 * - boundary, skeleton, thinning
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/QRegion.h>
#include <QiVision/Core/Types.h>

#include <cstdint>
#include <string>
#include <vector>

namespace Qi::Vision::Morphology {

// =============================================================================
// Structuring Element
// =============================================================================

/// Structuring element shape type
enum class SEShape {
    Rectangle,      ///< Rectangular shape
    Ellipse,        ///< Elliptical shape
    Circle,         ///< Circular shape
    Cross,          ///< Cross (plus sign) shape
    Diamond,        ///< Diamond (rhombus) shape
    Line,           ///< Line segment shape
    Octagon,        ///< Octagonal shape
    Custom          ///< Custom user-defined shape
};

/**
 * @brief Structuring element for morphological operations
 *
 * Wraps Internal::StructElement with a cleaner public API.
 */
class StructuringElement {
public:
    // =========================================================================
    // Constructors
    // =========================================================================

    /// Default constructor (3x3 square)
    StructuringElement();

    /// Copy constructor
    StructuringElement(const StructuringElement& other);

    /// Move constructor
    StructuringElement(StructuringElement&& other) noexcept;

    /// Destructor
    ~StructuringElement();

    /// Copy assignment
    StructuringElement& operator=(const StructuringElement& other);

    /// Move assignment
    StructuringElement& operator=(StructuringElement&& other) noexcept;

    // =========================================================================
    // Factory Methods
    // =========================================================================

    /**
     * @brief Create rectangular structuring element
     *
     * Equivalent to Halcon's gen_rectangle1.
     *
     * @param width Width (must be odd for symmetric origin)
     * @param height Height (must be odd for symmetric origin)
     * @return Rectangular structuring element
     *
     * @code
     * auto se = StructuringElement::Rectangle(5, 3);
     * @endcode
     */
    static StructuringElement Rectangle(int32_t width, int32_t height);

    /**
     * @brief Create square structuring element
     *
     * @param size Side length (must be odd)
     * @return Square structuring element
     */
    static StructuringElement Square(int32_t size);

    /**
     * @brief Create circular structuring element
     *
     * Equivalent to Halcon's gen_circle.
     *
     * @param radius Circle radius
     * @return Circular structuring element
     *
     * @code
     * auto se = StructuringElement::Circle(5);
     * @endcode
     */
    static StructuringElement Circle(int32_t radius);

    /**
     * @brief Create elliptical structuring element
     *
     * Equivalent to Halcon's gen_ellipse.
     *
     * @param radiusX Horizontal radius
     * @param radiusY Vertical radius
     * @return Elliptical structuring element
     */
    static StructuringElement Ellipse(int32_t radiusX, int32_t radiusY);

    /**
     * @brief Create cross-shaped structuring element
     *
     * @param armLength Length of each arm from center
     * @param thickness Arm thickness (default 1)
     * @return Cross-shaped structuring element
     */
    static StructuringElement Cross(int32_t armLength, int32_t thickness = 1);

    /**
     * @brief Create diamond-shaped structuring element
     *
     * @param radius Distance from center to vertex
     * @return Diamond structuring element
     */
    static StructuringElement Diamond(int32_t radius);

    /**
     * @brief Create line structuring element
     *
     * @param length Line length
     * @param angle Line angle in radians (0 = horizontal)
     * @return Line structuring element
     */
    static StructuringElement Line(int32_t length, double angle);

    /**
     * @brief Create structuring element from binary mask
     *
     * @param mask Binary image (non-zero = part of element)
     * @param anchorX X coordinate of anchor (default: center)
     * @param anchorY Y coordinate of anchor (default: center)
     * @return Custom structuring element
     */
    static StructuringElement FromMask(const QImage& mask,
                                        int32_t anchorX = -1,
                                        int32_t anchorY = -1);

    /**
     * @brief Create structuring element from region
     *
     * @param region Input region
     * @param anchorX X coordinate of anchor (default: center)
     * @param anchorY Y coordinate of anchor (default: center)
     * @return Custom structuring element
     */
    static StructuringElement FromRegion(const QRegion& region,
                                          int32_t anchorX = -1,
                                          int32_t anchorY = -1);

    // =========================================================================
    // Properties
    // =========================================================================

    /// Check if structuring element is empty
    bool Empty() const;

    /// Get width
    int32_t Width() const;

    /// Get height
    int32_t Height() const;

    /// Get anchor X coordinate
    int32_t AnchorX() const;

    /// Get anchor Y coordinate
    int32_t AnchorY() const;

    /// Get shape type
    SEShape Shape() const;

    /// Get number of pixels in element
    size_t PixelCount() const;

    // =========================================================================
    // Transformations
    // =========================================================================

    /**
     * @brief Reflect structuring element (180 degree rotation)
     *
     * @return Reflected structuring element
     */
    StructuringElement Reflect() const;

    /**
     * @brief Rotate structuring element
     *
     * @param angle Rotation angle in radians
     * @return Rotated structuring element
     */
    StructuringElement Rotate(double angle) const;

    // =========================================================================
    // Internal Access (for implementation)
    // =========================================================================

    /// Get internal implementation pointer (for internal use only)
    void* GetInternal() const;

private:
    struct Impl;
    Impl* impl_;
};

// =============================================================================
// Binary Morphology (Region Operations)
// =============================================================================

/**
 * @brief Dilate region with structuring element
 *
 * Equivalent to Halcon's dilation1 operator.
 * Expands the region by adding pixels where SE overlaps.
 *
 * @param region Input region
 * @param se Structuring element
 * @return Dilated region
 *
 * @code
 * auto se = StructuringElement::Circle(3);
 * QRegion dilated = Dilation(region, se);
 * @endcode
 */
QRegion Dilation(const QRegion& region, const StructuringElement& se);

/**
 * @brief Dilate region with circular SE
 *
 * Equivalent to Halcon's dilation_circle operator.
 *
 * @param region Input region
 * @param radius Circle radius
 * @return Dilated region
 */
QRegion DilationCircle(const QRegion& region, int32_t radius);

/**
 * @brief Dilate region with rectangular SE
 *
 * Equivalent to Halcon's dilation_rectangle1 operator.
 *
 * @param region Input region
 * @param width SE width
 * @param height SE height
 * @return Dilated region
 */
QRegion DilationRectangle(const QRegion& region, int32_t width, int32_t height);

/**
 * @brief Erode region with structuring element
 *
 * Equivalent to Halcon's erosion1 operator.
 * Shrinks the region by removing boundary pixels.
 *
 * @param region Input region
 * @param se Structuring element
 * @return Eroded region
 *
 * @code
 * auto se = StructuringElement::Circle(3);
 * QRegion eroded = Erosion(region, se);
 * @endcode
 */
QRegion Erosion(const QRegion& region, const StructuringElement& se);

/**
 * @brief Erode region with circular SE
 *
 * Equivalent to Halcon's erosion_circle operator.
 *
 * @param region Input region
 * @param radius Circle radius
 * @return Eroded region
 */
QRegion ErosionCircle(const QRegion& region, int32_t radius);

/**
 * @brief Erode region with rectangular SE
 *
 * Equivalent to Halcon's erosion_rectangle1 operator.
 *
 * @param region Input region
 * @param width SE width
 * @param height SE height
 * @return Eroded region
 */
QRegion ErosionRectangle(const QRegion& region, int32_t width, int32_t height);

/**
 * @brief Opening operation (erosion followed by dilation)
 *
 * Equivalent to Halcon's opening operator.
 * Removes small protrusions and separates weakly connected regions.
 *
 * @param region Input region
 * @param se Structuring element
 * @return Opened region
 *
 * @code
 * auto se = StructuringElement::Circle(5);
 * QRegion opened = Opening(region, se);
 * @endcode
 */
QRegion Opening(const QRegion& region, const StructuringElement& se);

/**
 * @brief Opening with circular SE
 *
 * Equivalent to Halcon's opening_circle operator.
 *
 * @param region Input region
 * @param radius Circle radius
 * @return Opened region
 */
QRegion OpeningCircle(const QRegion& region, int32_t radius);

/**
 * @brief Opening with rectangular SE
 *
 * Equivalent to Halcon's opening_rectangle1 operator.
 *
 * @param region Input region
 * @param width SE width
 * @param height SE height
 * @return Opened region
 */
QRegion OpeningRectangle(const QRegion& region, int32_t width, int32_t height);

/**
 * @brief Closing operation (dilation followed by erosion)
 *
 * Equivalent to Halcon's closing operator.
 * Fills small holes and connects nearby regions.
 *
 * @param region Input region
 * @param se Structuring element
 * @return Closed region
 *
 * @code
 * auto se = StructuringElement::Circle(5);
 * QRegion closed = Closing(region, se);
 * @endcode
 */
QRegion Closing(const QRegion& region, const StructuringElement& se);

/**
 * @brief Closing with circular SE
 *
 * Equivalent to Halcon's closing_circle operator.
 *
 * @param region Input region
 * @param radius Circle radius
 * @return Closed region
 */
QRegion ClosingCircle(const QRegion& region, int32_t radius);

/**
 * @brief Closing with rectangular SE
 *
 * Equivalent to Halcon's closing_rectangle1 operator.
 *
 * @param region Input region
 * @param width SE width
 * @param height SE height
 * @return Closed region
 */
QRegion ClosingRectangle(const QRegion& region, int32_t width, int32_t height);

/**
 * @brief Morphological boundary
 *
 * Equivalent to Halcon's boundary operator.
 * Returns the boundary pixels of the region.
 *
 * @param region Input region
 * @param type Boundary type: "inner", "outer", "both"
 * @return Boundary region
 */
QRegion Boundary(const QRegion& region, const std::string& type = "both");

/**
 * @brief Morphological skeleton
 *
 * Equivalent to Halcon's skeleton operator.
 * Extracts the medial axis of the region.
 *
 * @param region Input region
 * @return Skeleton region
 */
QRegion Skeleton(const QRegion& region);

/**
 * @brief Morphological thinning
 *
 * Iteratively thins the region until stable.
 *
 * @param region Input region
 * @param maxIterations Maximum iterations (0 = until stable)
 * @return Thinned region
 */
QRegion Thinning(const QRegion& region, int32_t maxIterations = 0);

/**
 * @brief Prune skeleton branches
 *
 * Removes short branches from skeleton.
 *
 * @param skeleton Input skeleton
 * @param iterations Number of pruning iterations
 * @return Pruned skeleton
 */
QRegion PruneSkeleton(const QRegion& skeleton, int32_t iterations = 1);

/**
 * @brief Fill holes in region
 *
 * Equivalent to Halcon's fill_up operator.
 *
 * @param region Input region
 * @return Region with holes filled
 */
QRegion FillUp(const QRegion& region);

/**
 * @brief Clear border-touching regions
 *
 * Removes regions that touch the image border.
 *
 * @param region Input region
 * @param bounds Image bounds
 * @return Region without border-touching parts
 */
QRegion ClearBorder(const QRegion& region, const Rect2i& bounds);

// =============================================================================
// Gray Morphology (Image Operations)
// =============================================================================

/**
 * @brief Gray-scale dilation
 *
 * Equivalent to Halcon's gray_dilation operator.
 * For each pixel, takes the maximum value in the SE neighborhood.
 *
 * @param image Input grayscale image
 * @param output Output dilated image
 * @param se Structuring element
 *
 * @code
 * QImage dilated;
 * auto se = StructuringElement::Circle(3);
 * GrayDilation(image, dilated, se);
 * @endcode
 */
void GrayDilation(const QImage& image, QImage& output, const StructuringElement& se);

/**
 * @brief Gray-scale dilation with circular SE
 *
 * @param image Input grayscale image
 * @param output Output dilated image
 * @param radius Circle radius
 */
void GrayDilationCircle(const QImage& image, QImage& output, int32_t radius);

/**
 * @brief Gray-scale dilation with rectangular SE
 *
 * @param image Input grayscale image
 * @param output Output dilated image
 * @param width SE width
 * @param height SE height
 */
void GrayDilationRectangle(const QImage& image, QImage& output, int32_t width, int32_t height);

/**
 * @brief Gray-scale erosion
 *
 * Equivalent to Halcon's gray_erosion operator.
 * For each pixel, takes the minimum value in the SE neighborhood.
 *
 * @param image Input grayscale image
 * @param output Output eroded image
 * @param se Structuring element
 *
 * @code
 * QImage eroded;
 * auto se = StructuringElement::Circle(3);
 * GrayErosion(image, eroded, se);
 * @endcode
 */
void GrayErosion(const QImage& image, QImage& output, const StructuringElement& se);

/**
 * @brief Gray-scale erosion with circular SE
 *
 * @param image Input grayscale image
 * @param output Output eroded image
 * @param radius Circle radius
 */
void GrayErosionCircle(const QImage& image, QImage& output, int32_t radius);

/**
 * @brief Gray-scale erosion with rectangular SE
 *
 * @param image Input grayscale image
 * @param output Output eroded image
 * @param width SE width
 * @param height SE height
 */
void GrayErosionRectangle(const QImage& image, QImage& output, int32_t width, int32_t height);

/**
 * @brief Gray-scale opening
 *
 * Equivalent to Halcon's gray_opening operator.
 * Removes bright spots smaller than SE.
 *
 * @param image Input grayscale image
 * @param output Output opened image
 * @param se Structuring element
 */
void GrayOpening(const QImage& image, QImage& output, const StructuringElement& se);

/**
 * @brief Gray-scale opening with circular SE
 *
 * @param image Input grayscale image
 * @param output Output opened image
 * @param radius Circle radius
 */
void GrayOpeningCircle(const QImage& image, QImage& output, int32_t radius);

/**
 * @brief Gray-scale opening with rectangular SE
 *
 * @param image Input grayscale image
 * @param output Output opened image
 * @param width SE width
 * @param height SE height
 */
void GrayOpeningRectangle(const QImage& image, QImage& output, int32_t width, int32_t height);

/**
 * @brief Gray-scale closing
 *
 * Equivalent to Halcon's gray_closing operator.
 * Fills dark spots smaller than SE.
 *
 * @param image Input grayscale image
 * @param output Output closed image
 * @param se Structuring element
 */
void GrayClosing(const QImage& image, QImage& output, const StructuringElement& se);

/**
 * @brief Gray-scale closing with circular SE
 *
 * @param image Input grayscale image
 * @param output Output closed image
 * @param radius Circle radius
 */
void GrayClosingCircle(const QImage& image, QImage& output, int32_t radius);

/**
 * @brief Gray-scale closing with rectangular SE
 *
 * @param image Input grayscale image
 * @param output Output closed image
 * @param width SE width
 * @param height SE height
 */
void GrayClosingRectangle(const QImage& image, QImage& output, int32_t width, int32_t height);

/**
 * @brief Morphological gradient (dilation - erosion)
 *
 * Highlights edges by showing local contrast.
 *
 * @param image Input grayscale image
 * @param output Output gradient image
 * @param se Structuring element
 */
void GrayGradient(const QImage& image, QImage& output, const StructuringElement& se);

/**
 * @brief Morphological gradient with rectangular SE
 *
 * @param image Input grayscale image
 * @param output Output gradient image
 * @param width SE width
 * @param height SE height
 */
void GrayGradientRectangle(const QImage& image, QImage& output, int32_t width, int32_t height);

/**
 * @brief White top-hat (original - opening)
 *
 * Equivalent to Halcon's gray_tophat operator.
 * Extracts bright features smaller than SE.
 *
 * @param image Input grayscale image
 * @param output Output top-hat image
 * @param se Structuring element
 *
 * @code
 * QImage tophat;
 * auto se = StructuringElement::Circle(10);
 * GrayTopHat(image, tophat, se);  // Extract small bright spots
 * @endcode
 */
void GrayTopHat(const QImage& image, QImage& output, const StructuringElement& se);

/**
 * @brief White top-hat with circular SE
 *
 * @param image Input grayscale image
 * @param output Output top-hat image
 * @param radius Circle radius
 */
void GrayTopHatCircle(const QImage& image, QImage& output, int32_t radius);

/**
 * @brief Black top-hat (closing - original)
 *
 * Equivalent to Halcon's gray_bothat operator.
 * Extracts dark features smaller than SE.
 *
 * @param image Input grayscale image
 * @param output Output black-hat image
 * @param se Structuring element
 */
void GrayBlackHat(const QImage& image, QImage& output, const StructuringElement& se);

/**
 * @brief Black top-hat with circular SE
 *
 * @param image Input grayscale image
 * @param output Output black-hat image
 * @param radius Circle radius
 */
void GrayBlackHatCircle(const QImage& image, QImage& output, int32_t radius);

/**
 * @brief Local range (max - min)
 *
 * Equivalent to Halcon's gray_range_rect operator.
 * Computes local contrast at each pixel.
 *
 * @param image Input grayscale image
 * @param output Output range image
 * @param width Window width
 * @param height Window height
 */
void GrayRange(const QImage& image, QImage& output, int32_t width, int32_t height);

/**
 * @brief Rolling ball background subtraction
 *
 * Uses morphological opening with large SE to estimate background,
 * then subtracts from original.
 *
 * @param image Input grayscale image
 * @param output Output background-corrected image
 * @param radius Ball radius (larger = smoother background)
 *
 * @code
 * QImage corrected;
 * RollingBall(image, corrected, 50);  // Remove uneven illumination
 * @endcode
 */
void RollingBall(const QImage& image, QImage& output, int32_t radius);

/**
 * @brief Gray-scale reconstruction by dilation
 *
 * Equivalent to Halcon's gray_opening_shape with reconstruction.
 * Iteratively apply geodesic dilation until stable.
 *
 * @param marker Marker image (seed, must be <= mask)
 * @param mask Constraint mask (upper bound)
 * @param output Output reconstructed image
 */
void GrayReconstructDilation(const QImage& marker, const QImage& mask, QImage& output);

/**
 * @brief Gray-scale reconstruction by erosion
 *
 * @param marker Marker image (seed, must be >= mask)
 * @param mask Constraint mask (lower bound)
 * @param output Output reconstructed image
 */
void GrayReconstructErosion(const QImage& marker, const QImage& mask, QImage& output);

/**
 * @brief Fill holes in grayscale image
 *
 * Fills dark regions completely surrounded by brighter pixels.
 *
 * @param image Input grayscale image
 * @param output Output image with holes filled
 */
void GrayFillHoles(const QImage& image, QImage& output);

// =============================================================================
// Convenience Functions
// =============================================================================

/**
 * @brief Create common 3x3 cross structuring element
 *
 * For 4-connected operations.
 *
 * @return 3x3 cross element
 */
StructuringElement SE_Cross3();

/**
 * @brief Create common 3x3 square structuring element
 *
 * For 8-connected operations.
 *
 * @return 3x3 square element
 */
StructuringElement SE_Square3();

/**
 * @brief Create common 5x5 disk structuring element
 *
 * @return 5x5 disk element
 */
StructuringElement SE_Disk5();

} // namespace Qi::Vision::Morphology
