#pragma once

/**
 * @file NCCModel.h
 * @brief Normalized Cross-Correlation (NCC) template matching (Halcon-style API)
 *
 * Implements gray-value based template matching using normalized cross-correlation,
 * compatible with Halcon's NCC model operators.
 *
 * Key features:
 * - Rotation-invariant matching with angle search
 * - Scale-invariant matching (optional)
 * - Subpixel position accuracy via interpolation
 * - Multi-level pyramid search for speed
 * - Integral image acceleration for fast NCC computation
 *
 * When to use NCC vs ShapeModel:
 * - NCC: Textured objects, gray-value patterns, low-contrast features
 * - ShapeModel: High-contrast edges, geometric shapes, occlusion handling
 *
 * API naming follows Halcon convention:
 * - CreateNCCModel / CreateScaledNCCModel
 * - FindNCCModel / FindScaledNCCModel
 * - GetNCCModelParams / GetNCCModelOrigin / SetNCCModelOrigin
 * - GetNCCModelSize
 * - WriteNCCModel / ReadNCCModel
 * - ClearNCCModel
 *
 * @see https://www.mvtec.com/doc/halcon/12/en/create_ncc_model.html
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/QRegion.h>
#include <QiVision/Core/Types.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace Qi::Vision::Matching {

// Forward declaration
namespace Internal {
class NCCModelImpl;
}

// =============================================================================
// NCCModel Class (Model Handle)
// =============================================================================

/**
 * @brief NCC model handle (equivalent to Halcon's ModelID)
 *
 * This class holds the template data for NCC-based matching.
 * Use the free functions to create and manipulate models.
 *
 * @code
 * // Create model
 * NCCModel model;
 * CreateNCCModel(templateImage, model, 4, 0, RAD(360), 0, "use_polarity");
 *
 * // Find matches
 * std::vector<double> rows, cols, angles, scores;
 * FindNCCModel(searchImage, model, 0, RAD(360), 0.7, 1, 0.5,
 *              "true", 0, rows, cols, angles, scores);
 * @endcode
 */
class NCCModel {
public:
    NCCModel();
    ~NCCModel();
    NCCModel(const NCCModel& other);
    NCCModel(NCCModel&& other) noexcept;
    NCCModel& operator=(const NCCModel& other);
    NCCModel& operator=(NCCModel&& other) noexcept;

    /// Check if model is valid
    bool IsValid() const;

    /// Get internal implementation (for internal use only)
    Internal::NCCModelImpl* Impl() { return impl_.get(); }
    const Internal::NCCModelImpl* Impl() const { return impl_.get(); }

private:
    friend class Internal::NCCModelImpl;
    std::unique_ptr<Internal::NCCModelImpl> impl_;
};

// =============================================================================
// Model Creation Functions
// =============================================================================

/**
 * @brief Create an NCC model for matching (rotation only)
 *
 * Equivalent to Halcon's create_ncc_model operator.
 *
 * @param templateImage Template image (grayscale)
 * @param model         [out] Created NCC model handle
 * @param numLevels     Number of pyramid levels (0 = auto)
 * @param angleStart    Smallest rotation angle [rad]
 * @param angleExtent   Extent of rotation angles [rad] (0 = all orientations)
 * @param angleStep     Step length of angles [rad] (0 = auto, ~1 degree)
 * @param metric        Match metric: "use_polarity", "ignore_global_polarity"
 *
 * @note The center of the template is used as the model origin.
 *       Use SetNCCModelOrigin() to change it.
 */
void CreateNCCModel(
    const QImage& templateImage,
    NCCModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    const std::string& metric
);

/**
 * @brief Create an NCC model from rectangular ROI
 *
 * @param templateImage Source image
 * @param roi           Rectangular region of interest
 * @param model         [out] Created NCC model handle
 * @param numLevels     Number of pyramid levels (0 = auto)
 * @param angleStart    Smallest rotation angle [rad]
 * @param angleExtent   Extent of rotation angles [rad]
 * @param angleStep     Step length of angles [rad] (0 = auto)
 * @param metric        Match metric
 */
void CreateNCCModel(
    const QImage& templateImage,
    const Rect2i& roi,
    NCCModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    const std::string& metric
);

/**
 * @brief Create an NCC model from arbitrary region (QRegion)
 *
 * Supports any shape: circle, ellipse, polygon, union of shapes, etc.
 * Pixels outside the region are masked out during matching.
 *
 * @param templateImage Source image
 * @param region        Region of interest (supports any shape)
 * @param model         [out] Created NCC model handle
 * @param numLevels     Number of pyramid levels (0 = auto)
 * @param angleStart    Smallest rotation angle [rad]
 * @param angleExtent   Extent of rotation angles [rad]
 * @param angleStep     Step length of angles [rad] (0 = auto)
 * @param metric        Match metric
 *
 * @code
 * // Create circular ROI
 * QRegion circle = QRegion::Circle(320, 240, 100);
 * NCCModel model;
 * CreateNCCModel(image, circle, model, 4, 0, RAD(360), 0, "use_polarity");
 * @endcode
 */
void CreateNCCModel(
    const QImage& templateImage,
    const QRegion& region,
    NCCModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    const std::string& metric
);

/**
 * @brief Create an NCC model with scale search (isotropic)
 *
 * Equivalent to Halcon's create_scaled_ncc_model operator.
 *
 * @param templateImage Template image
 * @param model         [out] Created NCC model handle
 * @param numLevels     Number of pyramid levels (0 = auto)
 * @param angleStart    Smallest rotation angle [rad]
 * @param angleExtent   Extent of rotation angles [rad]
 * @param angleStep     Step length of angles [rad] (0 = auto)
 * @param scaleMin      Minimum scale factor
 * @param scaleMax      Maximum scale factor
 * @param scaleStep     Scale step (0 = auto)
 * @param metric        Match metric
 */
void CreateScaledNCCModel(
    const QImage& templateImage,
    NCCModel& model,
    int32_t numLevels,
    double angleStart,
    double angleExtent,
    double angleStep,
    double scaleMin,
    double scaleMax,
    double scaleStep,
    const std::string& metric
);

// =============================================================================
// Model Search Functions
// =============================================================================

/**
 * @brief Find instances of an NCC model in an image
 *
 * Equivalent to Halcon's find_ncc_model operator.
 *
 * @param image         Search image (grayscale)
 * @param model         NCC model handle
 * @param angleStart    Smallest rotation angle [rad]
 * @param angleExtent   Extent of rotation angles [rad]
 * @param minScore      Minimum score threshold [0..1]
 * @param numMatches    Maximum number of matches (0 = all)
 * @param maxOverlap    Maximum overlap between matches [0..1]
 * @param subPixel      Subpixel accuracy: "none", "true", "interpolation"
 * @param numLevels     Number of pyramid levels (0 = use model levels)
 * @param rows          [out] Row coordinates of found instances
 * @param cols          [out] Column coordinates of found instances
 * @param angles        [out] Rotation angles of found instances [rad]
 * @param scores        [out] Match scores of found instances [0..1]
 */
void FindNCCModel(
    const QImage& image,
    const NCCModel& model,
    double angleStart,
    double angleExtent,
    double minScore,
    int32_t numMatches,
    double maxOverlap,
    const std::string& subPixel,
    int32_t numLevels,
    std::vector<double>& rows,
    std::vector<double>& cols,
    std::vector<double>& angles,
    std::vector<double>& scores
);

/**
 * @brief Find instances of a scaled NCC model
 *
 * Equivalent to Halcon's find_scaled_ncc_model operator.
 *
 * @param image         Search image
 * @param model         NCC model handle (created with CreateScaledNCCModel)
 * @param angleStart    Smallest rotation angle [rad]
 * @param angleExtent   Extent of rotation angles [rad]
 * @param scaleMin      Minimum scale factor
 * @param scaleMax      Maximum scale factor
 * @param minScore      Minimum score threshold [0..1]
 * @param numMatches    Maximum number of matches (0 = all)
 * @param maxOverlap    Maximum overlap between matches [0..1]
 * @param subPixel      Subpixel accuracy mode
 * @param numLevels     Number of pyramid levels (0 = use model levels)
 * @param rows          [out] Row coordinates
 * @param cols          [out] Column coordinates
 * @param angles        [out] Rotation angles [rad]
 * @param scales        [out] Scale factors
 * @param scores        [out] Match scores [0..1]
 */
void FindScaledNCCModel(
    const QImage& image,
    const NCCModel& model,
    double angleStart,
    double angleExtent,
    double scaleMin,
    double scaleMax,
    double minScore,
    int32_t numMatches,
    double maxOverlap,
    const std::string& subPixel,
    int32_t numLevels,
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
 * @brief Get the parameters of an NCC model
 *
 * Equivalent to Halcon's get_ncc_model_params operator.
 *
 * @param model         NCC model handle
 * @param numLevels     [out] Number of pyramid levels
 * @param angleStart    [out] Smallest rotation angle [rad]
 * @param angleExtent   [out] Extent of rotation angles [rad]
 * @param angleStep     [out] Step length of angles [rad]
 * @param metric        [out] Match metric string
 */
void GetNCCModelParams(
    const NCCModel& model,
    int32_t& numLevels,
    double& angleStart,
    double& angleExtent,
    double& angleStep,
    std::string& metric
);

/**
 * @brief Get the origin (reference point) of an NCC model
 *
 * Equivalent to Halcon's get_ncc_model_origin operator.
 *
 * @param model         NCC model handle
 * @param row           [out] Row coordinate of origin
 * @param col           [out] Column coordinate of origin
 */
void GetNCCModelOrigin(
    const NCCModel& model,
    double& row,
    double& col
);

/**
 * @brief Set the origin (reference point) of an NCC model
 *
 * Equivalent to Halcon's set_ncc_model_origin operator.
 *
 * @param model         NCC model handle
 * @param row           Row coordinate of new origin
 * @param col           Column coordinate of new origin
 */
void SetNCCModelOrigin(
    NCCModel& model,
    double row,
    double col
);

/**
 * @brief Get the size of an NCC model template
 *
 * @param model         NCC model handle
 * @param width         [out] Template width
 * @param height        [out] Template height
 */
void GetNCCModelSize(
    const NCCModel& model,
    int32_t& width,
    int32_t& height
);

// =============================================================================
// Model I/O Functions
// =============================================================================

/**
 * @brief Write an NCC model to file
 *
 * Equivalent to Halcon's write_ncc_model operator.
 *
 * @param model         NCC model handle
 * @param filename      Output file path
 */
void WriteNCCModel(
    const NCCModel& model,
    const std::string& filename
);

/**
 * @brief Read an NCC model from file
 *
 * Equivalent to Halcon's read_ncc_model operator.
 *
 * @param filename      Input file path
 * @param model         [out] Loaded NCC model handle
 */
void ReadNCCModel(
    const std::string& filename,
    NCCModel& model
);

/**
 * @brief Clear (release) an NCC model
 *
 * Equivalent to Halcon's clear_ncc_model operator.
 *
 * @param model         NCC model handle to clear
 */
void ClearNCCModel(
    NCCModel& model
);

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Determine optimal NCC model parameters automatically
 *
 * Analyzes the template to suggest optimal parameters.
 *
 * @param templateImage Template image
 * @param roi           Region of interest (empty = full image)
 * @param numLevels     [out] Recommended pyramid levels
 * @param angleStep     [out] Recommended angle step
 */
void DetermineNCCModelParams(
    const QImage& templateImage,
    const Rect2i& roi,
    int32_t& numLevels,
    double& angleStep
);

} // namespace Qi::Vision::Matching
