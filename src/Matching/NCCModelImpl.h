/**
 * @file NCCModelImpl.h
 * @brief Internal implementation structures for NCCModel
 *
 * This file contains:
 * - NCCLevelModel: Template data for a single pyramid level
 * - RotatedTemplate: Precomputed rotated template cache
 * - NCCModelImpl: Implementation class with all model data
 *
 * NCC Algorithm Overview:
 * 1. Precompute rotated templates at discrete angles
 * 2. Build integral images of search image (sum and squared sum)
 * 3. Multi-level pyramid search: coarse-to-fine
 * 4. NCC score: (sum(T*I) - n*mean(T)*mean(I)) / (n * stddev(T) * stddev(I))
 *
 * Optimizations:
 * - Integral images for O(1) region statistics
 * - Precomputed template statistics (mean, stddev)
 * - Pyramid hierarchy for fast coarse search
 * - SIMD for NCC computation (SSE/AVX2)
 */

#pragma once

#include <QiVision/Matching/NCCModel.h>
#include <QiVision/Matching/MatchTypes.h>
#include <QiVision/Internal/IntegralImage.h>
#include <QiVision/Internal/Pyramid.h>
#include <QiVision/Core/Types.h>
#include <QiVision/Core/Constants.h>

#include <vector>
#include <cmath>
#include <cstdint>

namespace Qi::Vision::Matching {
namespace Internal {

// =============================================================================
// NCCLevelModel: Template data for a single pyramid level
// =============================================================================

/**
 * @brief Template data for a single pyramid level
 *
 * Stores the template image and precomputed statistics for NCC.
 * For masked templates (QRegion ROI), stores pixel mask as well.
 */
struct NCCLevelModel {
    // Template image data
    std::vector<float> data;        ///< Template pixel values (normalized)
    std::vector<uint8_t> mask;      ///< Template mask (255 = valid, 0 = ignore)
    int32_t width = 0;              ///< Template width
    int32_t height = 0;             ///< Template height
    double scale = 1.0;             ///< Scale relative to original

    // Precomputed statistics
    double mean = 0.0;              ///< Mean of template (masked region)
    double stddev = 0.0;            ///< Standard deviation of template
    double sumSq = 0.0;             ///< Sum of (pixel - mean)^2
    int32_t numPixels = 0;          ///< Number of valid pixels

    // Zero-mean template (data[i] - mean)
    std::vector<float> zeroMean;    ///< Zero-mean template for correlation

    /// Check if level is valid
    bool IsValid() const { return width > 0 && height > 0 && !data.empty(); }

    /// Compute statistics from data
    void ComputeStatistics();

    /// Compute statistics with mask
    void ComputeStatisticsWithMask();
};

// =============================================================================
// RotatedTemplate: Precomputed rotated template
// =============================================================================

/**
 * @brief Precomputed rotated template at a specific angle
 *
 * For fast rotation search, we precompute rotated versions of the template
 * at discrete angle steps. This trades memory for speed.
 */
struct RotatedTemplate {
    double angle = 0.0;             ///< Rotation angle (radians)
    std::vector<float> data;        ///< Rotated template data (zero-mean)
    std::vector<uint8_t> mask;      ///< Rotated mask
    int32_t width = 0;              ///< Bounding box width after rotation
    int32_t height = 0;             ///< Bounding box height after rotation
    int32_t offsetX = 0;            ///< Offset to center (for coordinate transform)
    int32_t offsetY = 0;

    // Precomputed statistics for rotated template
    double mean = 0.0;
    double stddev = 0.0;
    int32_t numPixels = 0;

    /// Check if valid
    bool IsValid() const { return width > 0 && height > 0 && !data.empty(); }
};

// =============================================================================
// IntegralImagePyramid: Multi-level integral images
// =============================================================================

/**
 * @brief Integral images at multiple pyramid levels
 *
 * For each pyramid level, stores both sum and squared-sum integral images
 * to enable O(1) computation of region mean and variance.
 */
class IntegralImagePyramid {
public:
    IntegralImagePyramid() = default;

    /**
     * @brief Build integral image pyramid from Gaussian pyramid
     * @param pyramid Gaussian image pyramid
     * @return true if successful
     */
    bool Build(const Qi::Vision::Internal::ImagePyramid& pyramid);

    /**
     * @brief Clear all levels
     */
    void Clear();

    /**
     * @brief Check if pyramid is valid
     */
    bool IsValid() const { return !levels_.empty(); }

    /**
     * @brief Get number of levels
     */
    int32_t NumLevels() const { return static_cast<int32_t>(levels_.size()); }

    /**
     * @brief Get integral image at level
     */
    const Qi::Vision::Internal::IntegralImage& GetLevel(int32_t level) const;

    /**
     * @brief Get image pyramid level dimensions
     */
    int32_t GetWidth(int32_t level) const;
    int32_t GetHeight(int32_t level) const;

private:
    std::vector<Qi::Vision::Internal::IntegralImage> levels_;
    std::vector<int32_t> widths_;   ///< Original image width at each level
    std::vector<int32_t> heights_;  ///< Original image height at each level
};

// =============================================================================
// NCCModelImpl: Implementation Class
// =============================================================================

/**
 * @brief Internal implementation of NCCModel
 *
 * Contains all model data and methods for NCC-based template matching.
 */
class NCCModelImpl {
public:
    NCCModelImpl() = default;
    ~NCCModelImpl() = default;

    // Disable copy (use move or NCCModel wrapper)
    NCCModelImpl(const NCCModelImpl&) = delete;
    NCCModelImpl& operator=(const NCCModelImpl&) = delete;
    NCCModelImpl(NCCModelImpl&&) = default;
    NCCModelImpl& operator=(NCCModelImpl&&) = default;

    // =========================================================================
    // Model Data
    // =========================================================================

    /// Template data at each pyramid level
    std::vector<NCCLevelModel> levels_;

    /// Precomputed rotated templates per level
    /// rotatedTemplates_[level][angleIndex]
    std::vector<std::vector<RotatedTemplate>> rotatedTemplates_;

    /// Model parameters
    ModelParams params_;

    /// Model origin (reference point, relative to template center)
    Point2d origin_;

    /// Template size (original, before rotation)
    Size2i templateSize_;

    /// Whether model is valid
    bool valid_ = false;

    /// Whether using mask (QRegion ROI)
    bool hasMask_ = false;

    /// Metric mode
    MetricMode metric_ = MetricMode::UsePolarity;

    /// Search angle cache (discrete angles to search)
    std::vector<double> searchAngles_;

    // =========================================================================
    // Model Creation (NCCModelCreate.cpp)
    // =========================================================================

    /**
     * @brief Create model from image (full image as template)
     */
    bool CreateModel(const QImage& image);

    /**
     * @brief Create model from rectangular ROI
     */
    bool CreateModel(const QImage& image, const Rect2i& roi);

    /**
     * @brief Create model from QRegion (arbitrary shape)
     */
    bool CreateModel(const QImage& image, const QRegion& region);

    /**
     * @brief Build pyramid levels from template
     */
    void BuildPyramidLevels(const std::vector<float>& templateData,
                            int32_t width, int32_t height,
                            const std::vector<uint8_t>& mask = {});

    /**
     * @brief Precompute rotated templates for all angles
     */
    void PrecomputeRotatedTemplates();

    /**
     * @brief Rotate template by given angle
     */
    RotatedTemplate RotateTemplate(const NCCLevelModel& level, double angle);

    /**
     * @brief Build search angle list
     */
    void BuildSearchAngles();

    // =========================================================================
    // Search Functions (NCCModelSearch.cpp)
    // =========================================================================

    /**
     * @brief Find model instances in image
     */
    std::vector<MatchResult> Find(const QImage& image,
                                   const SearchParams& params) const;

    /**
     * @brief Search at a single pyramid level
     */
    std::vector<MatchResult> SearchLevel(
        const IntegralImagePyramid& integralPyramid,
        const Qi::Vision::Internal::ImagePyramid& imagePyramid,
        int32_t level,
        const std::vector<MatchResult>& candidates,
        const SearchParams& params) const;

    /**
     * @brief Coarse search at top pyramid level
     */
    std::vector<MatchResult> CoarseSearch(
        const IntegralImagePyramid& integralPyramid,
        const Qi::Vision::Internal::ImagePyramid& imagePyramid,
        int32_t level,
        const SearchParams& params) const;

    // =========================================================================
    // Score Computation (NCCModelScore.cpp)
    // =========================================================================

    /**
     * @brief Compute NCC score at given position and angle
     *
     * @param integralImage Integral image for fast statistics
     * @param imageData     Image pixel data
     * @param imageWidth    Image width
     * @param imageHeight   Image height
     * @param x, y          Position (top-left of template window)
     * @param angleIndex    Index into rotatedTemplates_
     * @param level         Pyramid level
     * @return NCC score [-1, 1]
     */
    double ComputeNCCScore(
        const Qi::Vision::Internal::IntegralImage& integralImage,
        const float* imageData,
        int32_t imageWidth,
        int32_t imageHeight,
        int32_t x,
        int32_t y,
        int32_t angleIndex,
        int32_t level) const;

    /**
     * @brief Compute NCC with subpixel refinement
     */
    double ComputeNCCScoreSubpixel(
        const float* imageData,
        int32_t imageWidth,
        int32_t imageHeight,
        double x,
        double y,
        double angle,
        int32_t level) const;

    /**
     * @brief Refine position using parabolic interpolation
     */
    void RefinePosition(
        const Qi::Vision::Internal::IntegralImage& integralImage,
        const float* imageData,
        int32_t imageWidth,
        int32_t imageHeight,
        MatchResult& match,
        int32_t level) const;

    // =========================================================================
    // Utility Functions
    // =========================================================================

    /**
     * @brief Get angle index from angle value
     */
    int32_t GetAngleIndex(double angle) const;

    /**
     * @brief Interpolate between two rotated templates
     */
    double InterpolateAngle(double angle, int32_t& lowerIdx, int32_t& upperIdx,
                            double& weight) const;
};

} // namespace Internal
} // namespace Qi::Vision::Matching
