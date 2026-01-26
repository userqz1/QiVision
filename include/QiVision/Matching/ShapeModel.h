#pragma once

/**
 * @file ShapeModel.h
 * @brief Shape-based template matching (Halcon-style API)
 *
 * Implements gradient direction-based shape matching algorithm,
 * compatible with Halcon's shape model operators.
 *
 * API naming follows Halcon convention:
 * - CreateShapeModel / CreateScaledShapeModel / CreateAnisoShapeModel
 * - FindShapeModel / FindScaledShapeModel / FindAnisoShapeModel
 * - GetShapeModelContours / GetShapeModelParams
 * - SetShapeModelOrigin / SetShapeModelMetric
 * - WriteShapeModel / ReadShapeModel
 * - ClearShapeModel
 *
 * @see https://www.mvtec.com/doc/halcon/12/en/create_shape_model.html
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/QRegion.h>
#include <QiVision/Core/QContourArray.h>
#include <QiVision/Core/Types.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace Qi::Vision::Matching {

// Forward declaration
namespace Internal {
class ShapeModelImpl;
}

// =============================================================================
// ShapeModel Class (Model Handle)
// =============================================================================

/**
 * @brief Shape model handle (equivalent to Halcon's ModelID)
 *
 * This class holds the model data. Use the free functions to create and
 * manipulate models.
 *
 * @code
 * // Create model
 * ShapeModel model = CreateShapeModel(templateImage, 4, 0, RAD(360), 0,
 *                                     "auto", "use_polarity", 30, 10);
 *
 * // Find matches
 * std::vector<double> rows, cols, angles, scores;
 * FindShapeModel(searchImage, model, 0, RAD(360), 0.7, 0, 0.5,
 *                "least_squares", 0, 0.9, rows, cols, angles, scores);
 * @endcode
 */
class ShapeModel {
public:
    ShapeModel();
    ~ShapeModel();
    ShapeModel(const ShapeModel& other);
    ShapeModel(ShapeModel&& other) noexcept;
    ShapeModel& operator=(const ShapeModel& other);
    ShapeModel& operator=(ShapeModel&& other) noexcept;

    /// Check if model is valid
    bool IsValid() const;

    /// Get internal implementation (for internal use only)
    Internal::ShapeModelImpl* Impl() { return impl_.get(); }
    const Internal::ShapeModelImpl* Impl() const { return impl_.get(); }

private:
    friend class Internal::ShapeModelImpl;
    std::unique_ptr<Internal::ShapeModelImpl> impl_;
};

// =============================================================================
// Model Creation Functions
// =============================================================================

/**
 * @brief Create a shape model for matching (rotation only)
 *
 * Equivalent to Halcon's create_shape_model operator.
 *
 * @param templateImage Template image (grayscale)
 * @param model         [out] Created shape model handle
 * @param numLevels     Number of pyramid levels (0 = auto)
 * @param angleStart    Smallest rotation angle [rad]
 * @param angleExtent   Extent of rotation angles [rad] (0 = all orientations)
 * @param angleStep     Step length of angles [rad] (0 = auto, ~1 degree)
 * @param optimization  Point reduction: "none", "auto", "point_reduction_low",
 *                      "point_reduction_medium", "point_reduction_high"
 * @param metric        Match metric: "use_polarity", "ignore_global_polarity",
 *                      "ignore_local_polarity", "ignore_color_polarity"
 * @param contrast      Threshold for edge extraction (or "auto")
 * @param minContrast   Minimum contrast in search images
 *
 * @note The center of the template is used as the model origin.
 *       Use SetShapeModelOrigin() to change it.
 */
void CreateShapeModel(
    const QImage& templateImage,
    ShapeModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast
);

/**
 * @brief Create a shape model from rectangular ROI
 *
 * @param templateImage Source image
 * @param roi           Rectangular region of interest
 * @param model         [out] Created shape model handle
 * @param numLevels     Number of pyramid levels (0 = auto)
 * @param angleStart    Smallest rotation angle [rad]
 * @param angleExtent   Extent of rotation angles [rad]
 * @param angleStep     Step length of angles [rad] (0 = auto)
 * @param optimization  Point reduction mode
 * @param metric        Match metric
 * @param contrast      Threshold for edge extraction
 * @param minContrast   Minimum contrast in search images
 */
void CreateShapeModel(
    const QImage& templateImage,
    const Rect2i& roi,
    ShapeModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast
);

/**
 * @brief Create a shape model from arbitrary region (QRegion)
 *
 * Supports any shape: circle, ellipse, polygon, union of shapes, etc.
 *
 * @param templateImage Source image
 * @param region        Region of interest (supports any shape)
 * @param model         [out] Created shape model handle
 * @param numLevels     Number of pyramid levels (0 = auto)
 * @param angleStart    Smallest rotation angle [rad]
 * @param angleExtent   Extent of rotation angles [rad]
 * @param angleStep     Step length of angles [rad] (0 = auto)
 * @param optimization  Point reduction mode
 * @param metric        Match metric
 * @param contrast      Threshold for edge extraction
 * @param minContrast   Minimum contrast in search images
 *
 * @code
 * // Create circular ROI
 * QRegion circle = QRegion::Circle(320, 240, 100);
 * ShapeModel model;
 * CreateShapeModel(image, circle, model, 4, 0, RAD(360), 0,
 *                  "auto", "use_polarity", "auto", 10);
 *
 * // Create complex ROI (union of shapes)
 * QRegion roi = QRegion::Circle(100, 100, 50) | QRegion::Rectangle(200, 50, 100, 100);
 * ShapeModel model2;
 * CreateShapeModel(image, roi, model2, ...);
 * @endcode
 */
void CreateShapeModel(
    const QImage& templateImage,
    const QRegion& region,
    ShapeModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast
);

/**
 * @brief Create a shape model with scale search (isotropic)
 *
 * Equivalent to Halcon's create_scaled_shape_model operator.
 *
 * @param templateImage Template image
 * @param model         [out] Created shape model handle
 * @param numLevels     Number of pyramid levels (0 = auto)
 * @param angleStart    Smallest rotation angle [rad]
 * @param angleExtent   Extent of rotation angles [rad]
 * @param angleStep     Step length of angles [rad] (0 = auto)
 * @param scaleMin      Minimum scale factor
 * @param scaleMax      Maximum scale factor
 * @param scaleStep     Scale step (0 = auto)
 * @param optimization  Point reduction mode
 * @param metric        Match metric
 * @param contrast      Threshold for edge extraction
 * @param minContrast   Minimum contrast in search images
 */
void CreateScaledShapeModel(
    const QImage& templateImage,
    ShapeModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    double scaleMin,
    double scaleMax,
    double scaleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast
);

/**
 * @brief Create a shape model with scale search (isotropic) from ROI
 *
 * Equivalent to Halcon's create_scaled_shape_model operator with ROI.
 *
 * @param templateImage Template image
 * @param roi           Rectangular ROI for model creation
 * @param model         [out] Created shape model handle
 * @param numLevels     Number of pyramid levels (0 = auto)
 * @param angleStart    Smallest rotation angle [rad]
 * @param angleExtent   Extent of rotation angles [rad]
 * @param angleStep     Step length of angles [rad] (0 = auto)
 * @param scaleMin      Minimum scale factor
 * @param scaleMax      Maximum scale factor
 * @param scaleStep     Scale step (0 = auto)
 * @param optimization  Point reduction mode
 * @param metric        Match metric
 * @param contrast      Threshold for edge extraction
 * @param minContrast   Minimum contrast in search images
 */
void CreateScaledShapeModel(
    const QImage& templateImage,
    const Rect2i& roi,
    ShapeModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    double scaleMin,
    double scaleMax,
    double scaleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast
);

/**
 * @brief Create a shape model with scale search (isotropic) from QRegion
 *
 * Equivalent to Halcon's create_scaled_shape_model operator with region mask.
 *
 * @param templateImage Template image
 * @param region        Region mask for model creation
 * @param model         [out] Created shape model handle
 * @param numLevels     Number of pyramid levels (0 = auto)
 * @param angleStart    Smallest rotation angle [rad]
 * @param angleExtent   Extent of rotation angles [rad]
 * @param angleStep     Step length of angles [rad] (0 = auto)
 * @param scaleMin      Minimum scale factor
 * @param scaleMax      Maximum scale factor
 * @param scaleStep     Scale step (0 = auto)
 * @param optimization  Point reduction mode
 * @param metric        Match metric
 * @param contrast      Threshold for edge extraction
 * @param minContrast   Minimum contrast in search images
 */
void CreateScaledShapeModel(
    const QImage& templateImage,
    const QRegion& region,
    ShapeModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    double scaleMin,
    double scaleMax,
    double scaleStep,
    const std::string& optimization,
    const std::string& metric,
    const std::string& contrast,
    double minContrast
);

// =============================================================================
// Model Search Functions
// =============================================================================

/**
 * @brief Find instances of a shape model in an image
 *
 * Equivalent to Halcon's find_shape_model operator.
 *
 * @param image         Search image (grayscale)
 * @param model         Shape model handle
 * @param angleStart    Smallest rotation angle [rad]
 * @param angleExtent   Extent of rotation angles [rad]
 * @param minScore      Minimum score threshold [0..1]
 * @param numMatches    Maximum number of matches (0 = all)
 * @param maxOverlap    Maximum overlap between matches [0..1]
 * @param subPixel      Subpixel accuracy: "none", "interpolation", "least_squares"
 * @param numLevels     Number of pyramid levels (0 = use model levels)
 * @param greediness    Search greediness [0..1] (higher = faster but may miss)
 * @param rows          [out] Row coordinates of found instances
 * @param cols          [out] Column coordinates of found instances
 * @param angles        [out] Rotation angles of found instances [rad]
 * @param scores        [out] Match scores of found instances [0..1]
 */
void FindShapeModel(
    const QImage& image,
    const ShapeModel& model,
    double angleStart,
    double angleExtent,
    double minScore,
    int32_t numMatches,
    double maxOverlap,
    const std::string& subPixel,
    int32_t numLevels,
    double greediness,
    std::vector<double>& rows,
    std::vector<double>& cols,
    std::vector<double>& angles,
    std::vector<double>& scores
);

/**
 * @brief Find instances of a scaled shape model
 *
 * Equivalent to Halcon's find_scaled_shape_model operator.
 *
 * @param image         Search image
 * @param model         Shape model handle (created with CreateScaledShapeModel)
 * @param angleStart    Smallest rotation angle [rad]
 * @param angleExtent   Extent of rotation angles [rad]
 * @param scaleMin      Minimum scale factor
 * @param scaleMax      Maximum scale factor
 * @param minScore      Minimum score threshold [0..1]
 * @param numMatches    Maximum number of matches (0 = all)
 * @param maxOverlap    Maximum overlap between matches [0..1]
 * @param subPixel      Subpixel accuracy mode
 * @param numLevels     Number of pyramid levels (0 = use model levels)
 * @param greediness    Search greediness [0..1]
 * @param rows          [out] Row coordinates
 * @param cols          [out] Column coordinates
 * @param angles        [out] Rotation angles [rad]
 * @param scales        [out] Scale factors
 * @param scores        [out] Match scores [0..1]
 */
void FindScaledShapeModel(
    const QImage& image,
    const ShapeModel& model,
    double angleStart,
    double angleExtent,
    double scaleMin,
    double scaleMax,
    double minScore,
    int32_t numMatches,
    double maxOverlap,
    const std::string& subPixel,
    int32_t numLevels,
    double greediness,
    std::vector<double>& rows,
    std::vector<double>& cols,
    std::vector<double>& angles,
    std::vector<double>& scales,
    std::vector<double>& scores
);

// =============================================================================
// Model Property Functions
// =============================================================================

/**
 * @brief Get the contour representation of a shape model
 *
 * Equivalent to Halcon's get_shape_model_contours operator.
 *
 * @param model         Shape model handle
 * @param level         Pyramid level (1 = highest resolution)
 * @param contourRows   [out] Row coordinates of contour points
 * @param contourCols   [out] Column coordinates of contour points
 */
void GetShapeModelContours(
    const ShapeModel& model,
    int32_t level,
    std::vector<double>& contourRows,
    std::vector<double>& contourCols
);

/**
 * @brief Get the XLD contour representation of a shape model
 *
 * Returns the model contours as an XLD contour array, preserving topology
 * (individual contours are kept separate).
 *
 * Equivalent to Halcon's get_shape_model_contours with XLD output.
 *
 * @param model         Shape model handle
 * @param level         Pyramid level (1 = highest resolution)
 * @param contours      [out] QContourArray containing model contours
 */
void GetShapeModelXLD(
    const ShapeModel& model,
    int32_t level,
    QContourArray& contours
);

/**
 * @brief Get the parameters of a shape model
 *
 * Equivalent to Halcon's get_shape_model_params operator.
 *
 * @param model         Shape model handle
 * @param numLevels     [out] Number of pyramid levels
 * @param angleStart    [out] Smallest rotation angle [rad]
 * @param angleExtent   [out] Extent of rotation angles [rad]
 * @param angleStep     [out] Step length of angles [rad]
 * @param scaleMin      [out] Minimum scale factor
 * @param scaleMax      [out] Maximum scale factor
 * @param scaleStep     [out] Scale step
 * @param metric        [out] Match metric string
 */
void GetShapeModelParams(
    const ShapeModel& model,
    int32_t& numLevels,
    double& angleStart,
    double& angleExtent,
    double& angleStep,
    double& scaleMin,
    double& scaleMax,
    double& scaleStep,
    std::string& metric
);

/**
 * @brief Get the origin (reference point) of a shape model
 *
 * Equivalent to Halcon's get_shape_model_origin operator.
 *
 * @param model         Shape model handle
 * @param row           [out] Row coordinate of origin
 * @param col           [out] Column coordinate of origin
 */
void GetShapeModelOrigin(
    const ShapeModel& model,
    double& row,
    double& col
);

/**
 * @brief Set the origin (reference point) of a shape model
 *
 * Equivalent to Halcon's set_shape_model_origin operator.
 *
 * @param model         Shape model handle
 * @param row           Row coordinate of new origin
 * @param col           Column coordinate of new origin
 */
void SetShapeModelOrigin(
    ShapeModel& model,
    double row,
    double col
);

// =============================================================================
// Model I/O Functions
// =============================================================================

/**
 * @brief Write a shape model to file
 *
 * Equivalent to Halcon's write_shape_model operator.
 *
 * @param model         Shape model handle
 * @param filename      Output file path
 */
void WriteShapeModel(
    const ShapeModel& model,
    const std::string& filename
);

/**
 * @brief Read a shape model from file
 *
 * Equivalent to Halcon's read_shape_model operator.
 *
 * @param filename      Input file path
 * @param model         [out] Loaded shape model handle
 */
void ReadShapeModel(
    const std::string& filename,
    ShapeModel& model
);

/**
 * @brief Clear (release) a shape model
 *
 * Equivalent to Halcon's clear_shape_model operator.
 *
 * @param model         Shape model handle to clear
 */
void ClearShapeModel(
    ShapeModel& model
);

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Determine optimal shape model parameters automatically
 *
 * Equivalent to Halcon's determine_shape_model_params operator.
 *
 * @param templateImage Template image
 * @param roi           Region of interest (empty = full image)
 * @param numLevels     [out] Recommended pyramid levels
 * @param angleStart    [out] Recommended angle start
 * @param angleExtent   [out] Recommended angle extent
 * @param angleStep     [out] Recommended angle step
 * @param scaleMin      [out] Recommended scale min
 * @param scaleMax      [out] Recommended scale max
 * @param scaleStep     [out] Recommended scale step
 * @param contrast      [out] Recommended contrast threshold
 * @param minContrast   [out] Recommended min contrast
 */
void DetermineShapeModelParams(
    const QImage& templateImage,
    const Rect2i& roi,
    int32_t& numLevels,
    double& angleStart,
    double& angleExtent,
    double& angleStep,
    double& scaleMin,
    double& scaleMax,
    double& scaleStep,
    double& contrast,
    double& minContrast
);

/**
 * @brief Inspect a shape model (for debugging)
 *
 * Equivalent to Halcon's inspect_shape_model operator.
 *
 * @param model         Shape model handle
 * @param level         Pyramid level to inspect
 * @param contrastImage [out] Contrast/edge image at this level
 * @param numPoints     [out] Number of model points
 */
void InspectShapeModel(
    const ShapeModel& model,
    int32_t level,
    QImage& contrastImage,
    int32_t& numPoints
);

/**
 * @brief Enable/disable model creation debug output
 *
 * Prints auto-contrast statistics and stage point counts.
 */
void SetShapeModelDebugCreate(ShapeModel& model, bool enable);

/**
 * @brief Enable/disable model creation debug output for newly created models
 */
void SetShapeModelDebugCreateGlobal(bool enable);

// =============================================================================
// Helper: Convert degrees to radians
// =============================================================================

/// Convert degrees to radians
inline constexpr double RAD(double degrees) {
    return degrees * 0.017453292519943295;  // PI / 180
}

/// Convert radians to degrees
inline constexpr double DEG(double radians) {
    return radians * 57.29577951308232;     // 180 / PI
}

} // namespace Qi::Vision::Matching
