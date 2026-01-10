#pragma once

/**
 * @file ShapeModel.h
 * @brief Shape-based template matching using gradient direction similarity
 *
 * ShapeModel implements the gradient direction-based shape matching algorithm,
 * similar to Halcon's find_shape_model. Key features:
 *
 * - Uses gradient direction (not magnitude) for robustness to lighting changes
 * - Multi-scale pyramid search for efficiency
 * - Subpixel position, angle, and scale refinement
 * - Supports arbitrary rotation and uniform scaling
 *
 * Algorithm Overview:
 * 1. Model Creation: Extract gradient directions from template, build pyramid
 * 2. Coarse Search: Start from top pyramid level, find candidate positions
 * 3. Refinement: Propagate candidates down pyramid, refine at each level
 * 4. Subpixel: Final least-squares refinement for subpixel accuracy
 *
 * Performance: O(N*M*A*S) where N,M=image size, A=angle steps, S=scale steps
 * With pyramid acceleration, typical speedup is 10-100x.
 *
 * @see AnglePyramid.h for gradient pyramid computation
 * @see MatchTypes.h for common types
 */

#include <QiVision/Matching/MatchTypes.h>
#include <QiVision/Core/QImage.h>
#include <QiVision/Core/Types.h>

#include <cstdint>
#include <cstdio>
#include <memory>
#include <vector>
#include <string>

namespace Qi::Vision::Matching {

// =============================================================================
// Timing Structures
// =============================================================================

/**
 * @brief Detailed timing for ShapeModel::Create()
 */
struct ShapeModelCreateTiming {
    double totalMs = 0.0;               ///< Total Create time
    double pyramidBuildMs = 0.0;        ///< AnglePyramid::Build time
    double contrastAutoMs = 0.0;        ///< Auto contrast detection time
    double extractPointsMs = 0.0;       ///< Extract model points time
    double optimizeMs = 0.0;            ///< Model optimization time
    double buildSoAMs = 0.0;            ///< Build SoA data for SIMD

    void Print() const {
        std::printf("[ShapeModel Create Timing] Total: %.2fms\n", totalMs);
        std::printf("  PyramidBuild:   %6.2fms (%4.1f%%)\n", pyramidBuildMs, 100.0 * pyramidBuildMs / totalMs);
        std::printf("  ContrastAuto:   %6.2fms (%4.1f%%)\n", contrastAutoMs, 100.0 * contrastAutoMs / totalMs);
        std::printf("  ExtractPoints:  %6.2fms (%4.1f%%)\n", extractPointsMs, 100.0 * extractPointsMs / totalMs);
        std::printf("  Optimize:       %6.2fms (%4.1f%%)\n", optimizeMs, 100.0 * optimizeMs / totalMs);
        std::printf("  BuildSoA:       %6.2fms (%4.1f%%)\n", buildSoAMs, 100.0 * buildSoAMs / totalMs);
    }
};

/**
 * @brief Detailed timing for ShapeModel::Find()
 */
struct ShapeModelFindTiming {
    double totalMs = 0.0;               ///< Total Find time
    double pyramidBuildMs = 0.0;        ///< AnglePyramid::Build time (for target image)
    double coarseSearchMs = 0.0;        ///< Coarse search at top level
    double pyramidRefineMs = 0.0;       ///< Pyramid refinement through levels
    double subpixelRefineMs = 0.0;      ///< Subpixel refinement
    double nmsMs = 0.0;                 ///< Non-maximum suppression
    int32_t numCoarseCandidates = 0;    ///< Number of coarse candidates found
    int32_t numFinalMatches = 0;        ///< Number of final matches

    void Print() const {
        std::printf("[ShapeModel Find Timing] Total: %.2fms\n", totalMs);
        std::printf("  PyramidBuild:   %6.2fms (%4.1f%%)\n", pyramidBuildMs, 100.0 * pyramidBuildMs / totalMs);
        std::printf("  CoarseSearch:   %6.2fms (%4.1f%%) - %d candidates\n",
                    coarseSearchMs, 100.0 * coarseSearchMs / totalMs, numCoarseCandidates);
        std::printf("  PyramidRefine:  %6.2fms (%4.1f%%)\n", pyramidRefineMs, 100.0 * pyramidRefineMs / totalMs);
        std::printf("  SubpixelRefine: %6.2fms (%4.1f%%)\n", subpixelRefineMs, 100.0 * subpixelRefineMs / totalMs);
        std::printf("  NMS:            %6.2fms (%4.1f%%) - %d matches\n",
                    nmsMs, 100.0 * nmsMs / totalMs, numFinalMatches);
    }
};

/**
 * @brief Parameters for enabling timing in ShapeModel
 */
struct ShapeModelTimingParams {
    bool enableTiming = false;          ///< Enable detailed timing statistics
    bool printTiming = false;           ///< Print timing to stderr after each operation

    ShapeModelTimingParams& SetEnableTiming(bool v) { enableTiming = v; return *this; }
    ShapeModelTimingParams& SetPrintTiming(bool v) { printTiming = v; return *this; }
};

// Forward declarations
namespace Internal {
class ShapeModelImpl;
}

// =============================================================================
// ShapeModel Class
// =============================================================================

/**
 * @brief Shape-based template matching model
 *
 * Usage:
 * @code
 * // Create model from template image
 * ShapeModel model;
 * model.Create(templateImage, ModelParams().SetContrast(30));
 *
 * // Find matches in target image
 * auto results = model.Find(targetImage,
 *     SearchParams().SetMinScore(0.7).SetAngleRange(-0.5, 1.0));
 *
 * // Access results
 * for (const auto& match : results) {
 *     std::cout << "Found at (" << match.x << ", " << match.y
 *               << ") angle=" << match.angle << " score=" << match.score << "\n";
 * }
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

    // =========================================================================
    // Model Creation
    // =========================================================================

    /**
     * @brief Create shape model from template image
     * @param templateImage Grayscale template image
     * @param params Model creation parameters
     * @return true if model created successfully
     *
     * The template's center is used as the model origin.
     * Model points are extracted based on gradient magnitude threshold.
     */
    bool Create(const QImage& templateImage, const ModelParams& params = ModelParams());

    /**
     * @brief Create shape model from template with ROI
     * @param templateImage Source image
     * @param roi Region of interest within the image
     * @param params Model creation parameters
     * @return true if model created successfully
     */
    bool Create(const QImage& templateImage, const Rect2i& roi,
                const ModelParams& params = ModelParams());

    /**
     * @brief Create shape model with custom origin
     * @param templateImage Template image
     * @param origin Model origin point (in template coordinates)
     * @param params Model creation parameters
     * @return true if model created successfully
     */
    bool CreateWithOrigin(const QImage& templateImage, const Point2d& origin,
                          const ModelParams& params = ModelParams());

    /**
     * @brief Clear the model
     */
    void Clear();

    /**
     * @brief Check if model is valid
     */
    bool IsValid() const;

    // =========================================================================
    // Searching
    // =========================================================================

    /**
     * @brief Find all matches in the target image
     * @param image Target grayscale image
     * @param params Search parameters
     * @return Vector of match results, sorted by score (descending)
     */
    std::vector<MatchResult> Find(const QImage& image,
                                   const SearchParams& params = SearchParams()) const;

    /**
     * @brief Find single best match
     * @param image Target image
     * @param params Search parameters
     * @return Best match result, or empty result if not found
     */
    MatchResult FindBest(const QImage& image,
                          const SearchParams& params = SearchParams()) const;

    /**
     * @brief Find matches in ROI
     * @param image Target image
     * @param roi Search region
     * @param params Search parameters
     * @return Vector of match results
     */
    std::vector<MatchResult> FindInROI(const QImage& image, const Rect2i& roi,
                                        const SearchParams& params = SearchParams()) const;

    // =========================================================================
    // Model Properties
    // =========================================================================

    /**
     * @brief Get model statistics
     */
    ModelStats GetStats() const;

    /**
     * @brief Get model points at specified level
     * @param level Pyramid level (0 = finest)
     * @return Vector of model points
     */
    std::vector<ModelPoint> GetModelPoints(int32_t level = 0) const;

    /**
     * @brief Get number of pyramid levels
     */
    int32_t NumLevels() const;

    /**
     * @brief Get model origin
     */
    Point2d GetOrigin() const;

    /**
     * @brief Get model size (width, height)
     */
    Size2i GetSize() const;

    /**
     * @brief Get creation parameters
     */
    const ModelParams& GetParams() const;

    // =========================================================================
    // Visualization
    // =========================================================================

    /**
     * @brief Get model contour for visualization
     * @param level Pyramid level
     * @return Contour points representing the model shape
     */
    std::vector<Point2d> GetModelContour(int32_t level = 0) const;

    /**
     * @brief Transform model contour to match position
     * @param match Match result
     * @return Transformed contour points
     */
    std::vector<Point2d> GetMatchContour(const MatchResult& match) const;

    // =========================================================================
    // Serialization
    // =========================================================================

    /**
     * @brief Save model to file
     * @param filename Output file path
     * @return true if saved successfully
     */
    bool Save(const std::string& filename) const;

    /**
     * @brief Load model from file
     * @param filename Input file path
     * @return true if loaded successfully
     */
    bool Load(const std::string& filename);

    // =========================================================================
    // Advanced
    // =========================================================================

    /**
     * @brief Compute match score at specific position
     * @param image Target image
     * @param x X position
     * @param y Y position
     * @param angle Rotation angle (radians)
     * @param scale Scale factor
     * @return Match score [0, 1]
     *
     * Useful for verification or custom search strategies.
     */
    double ComputeScore(const QImage& image,
                        double x, double y,
                        double angle = 0.0,
                        double scale = 1.0) const;

    /**
     * @brief Refine match position with subpixel accuracy
     * @param image Target image
     * @param match Initial match (will be refined in place)
     * @param method Subpixel refinement method
     * @return true if refinement successful
     */
    bool RefineMatch(const QImage& image, MatchResult& match,
                     SubpixelMethod method = SubpixelMethod::LeastSquares) const;

    // =========================================================================
    // Timing (Performance Profiling)
    // =========================================================================

    /**
     * @brief Enable or disable timing collection
     * @param params Timing parameters
     */
    void SetTimingParams(const ShapeModelTimingParams& params);

    /**
     * @brief Get timing from last Create() call
     * @note Only valid if timing was enabled before Create()
     */
    const ShapeModelCreateTiming& GetCreateTiming() const;

    /**
     * @brief Get timing from last Find() call
     * @note Only valid if timing was enabled before Find()
     */
    const ShapeModelFindTiming& GetFindTiming() const;

private:
    std::unique_ptr<Internal::ShapeModelImpl> impl_;
};

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Create shape model from image file
 * @param filename Image file path
 * @param params Model parameters
 * @return Created model (check IsValid())
 */
ShapeModel CreateShapeModelFromFile(const std::string& filename,
                                     const ModelParams& params = ModelParams());

/**
 * @brief Estimate optimal pyramid levels for image size
 * @param imageWidth Image width
 * @param imageHeight Image height
 * @param modelWidth Model width
 * @param modelHeight Model height
 * @return Recommended number of pyramid levels
 */
int32_t EstimateOptimalLevels(int32_t imageWidth, int32_t imageHeight,
                               int32_t modelWidth, int32_t modelHeight);

/**
 * @brief Estimate optimal angle step based on model size
 * @param modelSize Approximate model size in pixels
 * @return Recommended angle step in radians
 */
double EstimateAngleStep(int32_t modelSize);

} // namespace Qi::Vision::Matching
