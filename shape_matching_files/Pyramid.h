#pragma once

/**
 * @file Pyramid.h
 * @brief Image pyramid for multi-scale processing
 *
 * Implements various image pyramids for multi-scale analysis:
 * - Gaussian pyramid: progressive smoothing and downsampling
 * - Laplacian pyramid: band-pass decomposition for reconstruction
 * - Gradient pyramid: gradient magnitude/direction at each scale
 *
 * Used by:
 * - Template matching (ShapeModel): multi-scale search
 * - Image blending: seamless compositing
 * - Scale-space analysis: feature detection
 *
 * Reference:
 * - Burt & Adelson, "The Laplacian Pyramid as a Compact Image Code" (1983)
 */

#include <QiVision/Core/Types.h>
#include <QiVision/Core/QImage.h>

#include <cstdint>
#include <vector>

namespace Qi::Vision::Internal {

// ============================================================================
// Constants
// ============================================================================

/// Maximum number of pyramid levels
constexpr int32_t MAX_PYRAMID_LEVELS = 16;

/// Default pyramid scale factor
constexpr double DEFAULT_SCALE_FACTOR = 0.5;

/// Minimum image dimension for pyramid level
constexpr int32_t MIN_PYRAMID_DIMENSION = 4;

// ============================================================================
// Enumerations
// ============================================================================

/**
 * @brief Pyramid type
 */
enum class PyramidType {
    Gaussian,       ///< Gaussian (low-pass) pyramid
    Laplacian,      ///< Laplacian (band-pass) pyramid
    Gradient        ///< Gradient magnitude pyramid
};

/**
 * @brief Downsampling method
 */
enum class DownsampleMethod {
    Skip,           ///< Take every other pixel (fastest, may alias)
    Average,        ///< Average 2x2 block
    Gaussian        ///< Gaussian blur then subsample (best quality)
};

/**
 * @brief Upsampling method for reconstruction
 */
enum class UpsampleMethod {
    NearestNeighbor,    ///< Nearest neighbor (blocky)
    Bilinear,           ///< Bilinear interpolation
    Bicubic             ///< Bicubic interpolation (best quality)
};

// ============================================================================
// Data Structures
// ============================================================================

/**
 * @brief Parameters for pyramid construction
 */
struct PyramidParams {
    int32_t numLevels = 0;              ///< Number of levels (0 = auto-compute)
    double scaleFactor = 0.5;           ///< Scale factor per level (typically 0.5)
    double sigma = 1.0;                 ///< Gaussian sigma for smoothing
    DownsampleMethod downsample = DownsampleMethod::Gaussian;
    int32_t minDimension = MIN_PYRAMID_DIMENSION;  ///< Stop when dimension < this

    /**
     * @brief Create default params with specified number of levels
     */
    static PyramidParams WithLevels(int32_t levels, double sigma = 1.0) {
        PyramidParams p;
        p.numLevels = levels;
        p.sigma = sigma;
        return p;
    }

    /**
     * @brief Create params for auto-computed levels
     */
    static PyramidParams Auto(double sigma = 1.0, int32_t minDim = MIN_PYRAMID_DIMENSION) {
        PyramidParams p;
        p.numLevels = 0;  // Auto-compute
        p.sigma = sigma;
        p.minDimension = minDim;
        return p;
    }
};

/**
 * @brief A single level of a pyramid
 */
struct PyramidLevel {
    std::vector<float> data;    ///< Image data (float for precision)
    int32_t width = 0;          ///< Level width
    int32_t height = 0;         ///< Level height
    double scale = 1.0;         ///< Scale relative to original (1.0, 0.5, 0.25, ...)
    int32_t level = 0;          ///< Level index (0 = original)

    /// Check if level is valid
    bool IsValid() const { return width > 0 && height > 0 && !data.empty(); }

    /// Get pixel value at (x, y)
    float At(int32_t x, int32_t y) const {
        if (x < 0 || x >= width || y < 0 || y >= height) return 0.0f;
        return data[y * width + x];
    }

    /// Get pixel value with bounds checking
    float AtSafe(int32_t x, int32_t y) const {
        x = std::max(0, std::min(x, width - 1));
        y = std::max(0, std::min(y, height - 1));
        return data[y * width + x];
    }
};

/**
 * @brief Gradient pyramid level (stores magnitude and direction)
 */
struct GradientPyramidLevel {
    std::vector<float> magnitude;   ///< Gradient magnitude
    std::vector<float> direction;   ///< Gradient direction (radians)
    int32_t width = 0;
    int32_t height = 0;
    double scale = 1.0;
    int32_t level = 0;

    bool IsValid() const {
        return width > 0 && height > 0 &&
               !magnitude.empty() && !direction.empty();
    }
};

/**
 * @brief Image pyramid container
 */
class ImagePyramid {
public:
    ImagePyramid() = default;

    /// Number of levels
    int32_t NumLevels() const { return static_cast<int32_t>(levels_.size()); }

    /// Check if empty
    bool Empty() const { return levels_.empty(); }

    /// Get level by index (0 = finest/original)
    const PyramidLevel& GetLevel(int32_t level) const;
    PyramidLevel& GetLevel(int32_t level);

    /// Get level by scale (finds closest match)
    const PyramidLevel& GetLevelByScale(double scale) const;

    /// Original image dimensions
    int32_t OriginalWidth() const { return levels_.empty() ? 0 : levels_[0].width; }
    int32_t OriginalHeight() const { return levels_.empty() ? 0 : levels_[0].height; }

    /// Add a level
    void AddLevel(PyramidLevel&& level) { levels_.push_back(std::move(level)); }

    /// Clear all levels
    void Clear() { levels_.clear(); }

    /// Get all levels
    const std::vector<PyramidLevel>& Levels() const { return levels_; }
    std::vector<PyramidLevel>& Levels() { return levels_; }

private:
    std::vector<PyramidLevel> levels_;
};

/**
 * @brief Gradient pyramid container
 */
class GradientPyramid {
public:
    GradientPyramid() = default;

    int32_t NumLevels() const { return static_cast<int32_t>(levels_.size()); }
    bool Empty() const { return levels_.empty(); }

    const GradientPyramidLevel& GetLevel(int32_t level) const;
    GradientPyramidLevel& GetLevel(int32_t level);

    void AddLevel(GradientPyramidLevel&& level) { levels_.push_back(std::move(level)); }
    void Clear() { levels_.clear(); }

    const std::vector<GradientPyramidLevel>& Levels() const { return levels_; }
    std::vector<GradientPyramidLevel>& Levels() { return levels_; }

private:
    std::vector<GradientPyramidLevel> levels_;
};

// ============================================================================
// Gaussian Pyramid
// ============================================================================

/**
 * @brief Compute number of pyramid levels for given image size
 *
 * @param width Image width
 * @param height Image height
 * @param scaleFactor Scale factor per level
 * @param minDimension Minimum dimension for smallest level
 * @return Number of levels (including level 0)
 */
int32_t ComputeNumLevels(int32_t width, int32_t height,
                         double scaleFactor = 0.5,
                         int32_t minDimension = MIN_PYRAMID_DIMENSION);

/**
 * @brief Build Gaussian pyramid from image
 *
 * @param image Input image (grayscale)
 * @param params Pyramid parameters
 * @return Gaussian pyramid
 */
ImagePyramid BuildGaussianPyramid(const QImage& image,
                                   const PyramidParams& params = PyramidParams());

/**
 * @brief Build Gaussian pyramid from float data
 *
 * @param src Source data
 * @param width Image width
 * @param height Image height
 * @param params Pyramid parameters
 * @return Gaussian pyramid
 */
ImagePyramid BuildGaussianPyramid(const float* src, int32_t width, int32_t height,
                                   const PyramidParams& params = PyramidParams());

/**
 * @brief Downsample image by factor of 2
 *
 * @param src Source data
 * @param srcWidth Source width
 * @param srcHeight Source height
 * @param dst Destination (must be pre-allocated, size = srcWidth/2 * srcHeight/2)
 * @param sigma Gaussian sigma for pre-smoothing
 * @param method Downsampling method
 */
void DownsampleBy2(const float* src, int32_t srcWidth, int32_t srcHeight,
                   float* dst,
                   double sigma = 1.0,
                   DownsampleMethod method = DownsampleMethod::Gaussian);

/**
 * @brief Upsample image by factor of 2
 *
 * @param src Source data
 * @param srcWidth Source width
 * @param srcHeight Source height
 * @param dst Destination (must be pre-allocated, size = srcWidth*2 * srcHeight*2)
 * @param method Upsampling method
 */
void UpsampleBy2(const float* src, int32_t srcWidth, int32_t srcHeight,
                 float* dst,
                 UpsampleMethod method = UpsampleMethod::Bilinear);

// ============================================================================
// Laplacian Pyramid
// ============================================================================

/**
 * @brief Build Laplacian pyramid from image
 *
 * The Laplacian pyramid stores band-pass images (difference between levels).
 * Can be used to reconstruct the original image exactly.
 *
 * @param image Input image
 * @param params Pyramid parameters
 * @return Laplacian pyramid (last level is low-pass residual)
 */
ImagePyramid BuildLaplacianPyramid(const QImage& image,
                                    const PyramidParams& params = PyramidParams());

/**
 * @brief Build Laplacian from Gaussian pyramid
 *
 * @param gaussian Pre-built Gaussian pyramid
 * @return Laplacian pyramid
 */
ImagePyramid GaussianToLaplacian(const ImagePyramid& gaussian);

/**
 * @brief Reconstruct image from Laplacian pyramid
 *
 * @param laplacian Laplacian pyramid
 * @param method Upsampling method
 * @return Reconstructed image (float data)
 */
PyramidLevel ReconstructFromLaplacian(const ImagePyramid& laplacian,
                                       UpsampleMethod method = UpsampleMethod::Bilinear);

/**
 * @brief Blend two images using Laplacian pyramid
 *
 * @param img1 First image
 * @param img2 Second image
 * @param mask Blend mask (0 = img1, 255 = img2)
 * @param numLevels Number of pyramid levels
 * @return Blended image
 */
QImage BlendLaplacian(const QImage& img1, const QImage& img2,
                      const QImage& mask, int32_t numLevels = 4);

// ============================================================================
// Gradient Pyramid
// ============================================================================

/**
 * @brief Build gradient pyramid (magnitude and direction at each scale)
 *
 * @param image Input image
 * @param params Pyramid parameters
 * @return Gradient pyramid
 */
GradientPyramid BuildGradientPyramid(const QImage& image,
                                      const PyramidParams& params = PyramidParams());

/**
 * @brief Build gradient pyramid from Gaussian pyramid
 *
 * @param gaussian Pre-built Gaussian pyramid
 * @return Gradient pyramid
 */
GradientPyramid GaussianToGradient(const ImagePyramid& gaussian);

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Convert pyramid level to QImage
 *
 * @param level Pyramid level
 * @param normalize If true, normalize to [0, 255]
 * @return QImage
 */
QImage PyramidLevelToImage(const PyramidLevel& level, bool normalize = true);

/**
 * @brief Convert QImage to pyramid level
 *
 * @param image Input image
 * @param levelIndex Level index to assign
 * @param scale Scale value to assign
 * @return Pyramid level
 */
PyramidLevel ImageToPyramidLevel(const QImage& image, int32_t levelIndex = 0,
                                  double scale = 1.0);

/**
 * @brief Get pyramid level dimensions
 *
 * @param originalWidth Original image width
 * @param originalHeight Original image height
 * @param level Level index (0 = original)
 * @param scaleFactor Scale factor per level
 * @param[out] levelWidth Level width
 * @param[out] levelHeight Level height
 */
void GetLevelDimensions(int32_t originalWidth, int32_t originalHeight,
                        int32_t level, double scaleFactor,
                        int32_t& levelWidth, int32_t& levelHeight);

/**
 * @brief Convert coordinates from one level to another
 *
 * @param x X coordinate in source level
 * @param y Y coordinate in source level
 * @param srcLevel Source level index
 * @param dstLevel Destination level index
 * @param scaleFactor Scale factor per level
 * @param[out] dstX X coordinate in destination level
 * @param[out] dstY Y coordinate in destination level
 */
void ConvertCoordinates(double x, double y,
                        int32_t srcLevel, int32_t dstLevel,
                        double scaleFactor,
                        double& dstX, double& dstY);

/**
 * @brief Sample pyramid at specific scale using interpolation
 *
 * @param pyramid Gaussian pyramid
 * @param x X coordinate in original image
 * @param y Y coordinate in original image
 * @param scale Desired scale (e.g., 0.75 between levels 0 and 1)
 * @return Interpolated value
 */
float SamplePyramidAtScale(const ImagePyramid& pyramid,
                           double x, double y, double scale);

/**
 * @brief Compute image scale for template matching
 *
 * Given a model trained at a specific size, compute the scale factors
 * needed to search at different target sizes.
 *
 * @param modelWidth Model width
 * @param modelHeight Model height
 * @param minScale Minimum scale to search
 * @param maxScale Maximum scale to search
 * @param scaleStep Scale step
 * @return Vector of scales to search
 */
std::vector<double> ComputeSearchScales(int32_t modelWidth, int32_t modelHeight,
                                         double minScale, double maxScale,
                                         double scaleStep);

} // namespace Qi::Vision::Internal
