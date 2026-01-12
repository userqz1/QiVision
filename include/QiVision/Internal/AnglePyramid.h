#pragma once

/**
 * @file AnglePyramid.h
 * @brief Angle pyramid for shape-based matching
 *
 * Provides:
 * - Precomputed gradient direction images at multiple pyramid levels
 * - Quantized angle bins for fast lookup
 * - Edge point extraction with direction information
 *
 * Used by ShapeModel for efficient template matching.
 *
 * @see Pyramid.h for image pyramid construction
 * @see Gradient.h for gradient computation
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/Types.h>

#include <cstdint>
#include <cstdio>
#include <memory>
#include <vector>

namespace Qi::Vision::Internal {

// =============================================================================
// Constants
// =============================================================================

/// Default number of angle bins (quantization levels)
constexpr int32_t DEFAULT_ANGLE_BINS = 16;

/// Minimum number of angle bins
constexpr int32_t MIN_ANGLE_BINS = 4;

/// Maximum number of angle bins
constexpr int32_t MAX_ANGLE_BINS = 360;

/// Default minimum gradient magnitude for edge points
constexpr double DEFAULT_MIN_CONTRAST = 10.0;

/// Maximum pyramid levels for angle pyramid
constexpr int32_t ANGLE_PYRAMID_MAX_LEVELS = 10;

// =============================================================================
// Edge Point Structure
// =============================================================================

/**
 * @brief Edge point with gradient direction for shape matching
 */
struct EdgePoint {
    double x = 0.0;           ///< X coordinate (column)
    double y = 0.0;           ///< Y coordinate (row)
    double angle = 0.0;       ///< Gradient direction (radians, 0 to 2*PI)
    double magnitude = 0.0;   ///< Gradient magnitude
    int32_t angleBin = 0;     ///< Quantized angle bin index

    EdgePoint() = default;
    EdgePoint(double x_, double y_, double ang, double mag, int32_t bin)
        : x(x_), y(y_), angle(ang), magnitude(mag), angleBin(bin) {}
};

// =============================================================================
// Angle Pyramid Parameters
// =============================================================================

/**
 * @brief Parameters for angle pyramid construction
 */
struct AnglePyramidParams {
    int32_t numLevels = 4;              ///< Number of pyramid levels
    int32_t angleBins = DEFAULT_ANGLE_BINS;  ///< Number of angle quantization bins
    double minContrast = DEFAULT_MIN_CONTRAST;  ///< Minimum gradient magnitude
    double smoothSigma = 0.5;           ///< Gaussian smoothing before gradient
    bool useSubpixel = true;            ///< Use subpixel gradient interpolation
    bool enableTiming = false;          ///< Enable detailed timing statistics
    bool extractEdgePoints = true;      ///< Extract edge points (false for search pyramid)
    bool useNMS = true;                 ///< Apply Non-Maximum Suppression (false for shape matching)

    // Builder pattern
    AnglePyramidParams& SetNumLevels(int32_t n) { numLevels = n; return *this; }
    AnglePyramidParams& SetAngleBins(int32_t n) { angleBins = n; return *this; }
    AnglePyramidParams& SetMinContrast(double v) { minContrast = v; return *this; }
    AnglePyramidParams& SetSmoothSigma(double v) { smoothSigma = v; return *this; }
    AnglePyramidParams& SetSubpixel(bool v) { useSubpixel = v; return *this; }
    AnglePyramidParams& SetEnableTiming(bool v) { enableTiming = v; return *this; }
    AnglePyramidParams& SetExtractEdgePoints(bool v) { extractEdgePoints = v; return *this; }
    AnglePyramidParams& SetUseNMS(bool v) { useNMS = v; return *this; }
};

/**
 * @brief Detailed timing statistics for AnglePyramid::Build
 */
struct AnglePyramidTiming {
    double totalMs = 0.0;               ///< Total build time
    double toFloatMs = 0.0;             ///< Convert to float
    double gaussPyramidMs = 0.0;        ///< Build Gaussian pyramid (blur + downsample)
    double sobelMs = 0.0;               ///< Sobel gradient (Gx, Gy)
    double sqrtMs = 0.0;                ///< sqrt for magnitude
    double atan2Ms = 0.0;               ///< atan2 for direction
    double quantizeMs = 0.0;            ///< Quantize to bins
    double extractEdgeMs = 0.0;         ///< Extract edge points
    double copyMs = 0.0;                ///< Memory copy operations

    void Print() const {
        std::printf("[AnglePyramid Timing] Total: %.2fms\n", totalMs);
        std::printf("  ToFloat:      %6.2fms (%4.1f%%)\n", toFloatMs, 100.0 * toFloatMs / totalMs);
        std::printf("  GaussPyramid: %6.2fms (%4.1f%%)\n", gaussPyramidMs, 100.0 * gaussPyramidMs / totalMs);
        std::printf("  Sobel:        %6.2fms (%4.1f%%)\n", sobelMs, 100.0 * sobelMs / totalMs);
        std::printf("  Sqrt(mag):    %6.2fms (%4.1f%%)\n", sqrtMs, 100.0 * sqrtMs / totalMs);
        std::printf("  Atan2(dir):   %6.2fms (%4.1f%%)\n", atan2Ms, 100.0 * atan2Ms / totalMs);
        std::printf("  Quantize:     %6.2fms (%4.1f%%)\n", quantizeMs, 100.0 * quantizeMs / totalMs);
        std::printf("  ExtractEdge:  %6.2fms (%4.1f%%)\n", extractEdgeMs, 100.0 * extractEdgeMs / totalMs);
        std::printf("  Copy:         %6.2fms (%4.1f%%)\n", copyMs, 100.0 * copyMs / totalMs);
    }
};

// =============================================================================
// Pyramid Level Data
// =============================================================================

/**
 * @brief Data for a single pyramid level
 */
struct PyramidLevelData {
    int32_t level = 0;
    int32_t width = 0;
    int32_t height = 0;
    double scale = 1.0;         ///< Scale factor relative to original (0.5^level)

    QImage gradX;               ///< X gradient (float)
    QImage gradY;               ///< Y gradient (float)
    QImage gradMag;             ///< Gradient magnitude (float)
    QImage gradDir;             ///< Gradient direction in radians (float)
    QImage angleBinImage;       ///< Quantized angle bin indices (int16)

    std::vector<EdgePoint> edgePoints;  ///< Extracted edge points
};

// =============================================================================
// AnglePyramid Class
// =============================================================================

/**
 * @brief Angle pyramid for shape-based template matching
 *
 * @code
 * // Build angle pyramid from image
 * AnglePyramid pyramid;
 * pyramid.Build(image, AnglePyramidParams().SetNumLevels(4).SetMinContrast(20));
 *
 * // Access gradient direction at a level
 * const auto& level = pyramid.GetLevel(2);
 * double angle = pyramid.GetAngleAt(2, x, y);
 *
 * // Get edge points for model creation
 * auto points = pyramid.GetEdgePoints(0, roi);
 * @endcode
 */
class AnglePyramid {
public:
    AnglePyramid();
    ~AnglePyramid();
    AnglePyramid(const AnglePyramid& other);
    AnglePyramid(AnglePyramid&& other) noexcept;
    AnglePyramid& operator=(const AnglePyramid& other);
    AnglePyramid& operator=(AnglePyramid&& other) noexcept;

    // =========================================================================
    // Construction
    // =========================================================================

    /**
     * @brief Build angle pyramid from image
     * @param image Input grayscale image
     * @param params Pyramid parameters
     * @return true if successful
     */
    bool Build(const QImage& image, const AnglePyramidParams& params = AnglePyramidParams());

    /**
     * @brief Clear the pyramid
     */
    void Clear();

    /**
     * @brief Check if pyramid is valid
     */
    bool IsValid() const;

    // =========================================================================
    // Properties
    // =========================================================================

    int32_t NumLevels() const;
    int32_t AngleBins() const;
    int32_t OriginalWidth() const;
    int32_t OriginalHeight() const;
    const AnglePyramidParams& GetParams() const;

    /**
     * @brief Get timing statistics from last Build() call
     * @note Only valid if enableTiming was set in params
     */
    const AnglePyramidTiming& GetTiming() const;

    // =========================================================================
    // Level Access
    // =========================================================================

    /**
     * @brief Get pyramid level data
     * @param level Level index (0 = original resolution)
     */
    const PyramidLevelData& GetLevel(int32_t level) const;

    /**
     * @brief Get dimensions at a level
     */
    int32_t GetWidth(int32_t level) const;
    int32_t GetHeight(int32_t level) const;
    double GetScale(int32_t level) const;

    // =========================================================================
    // Gradient Access
    // =========================================================================

    /**
     * @brief Get gradient angle at position (bilinear interpolated)
     * @param level Pyramid level
     * @param x X coordinate at that level
     * @param y Y coordinate at that level
     * @return Gradient angle in radians [0, 2*PI), or -1 if invalid
     */
    double GetAngleAt(int32_t level, double x, double y) const;

    /**
     * @brief Get gradient magnitude at position
     */
    double GetMagnitudeAt(int32_t level, double x, double y) const;

    /**
     * @brief Get quantized angle bin at position
     * @return Angle bin index [0, angleBins), or -1 if invalid
     */
    int32_t GetAngleBinAt(int32_t level, int32_t x, int32_t y) const;

    /**
     * @brief Get gradient components at position (bilinear interpolated)
     */
    bool GetGradientAt(int32_t level, double x, double y,
                       double& gx, double& gy) const;

    /**
     * @brief Get gradient components at integer position (fast, nearest neighbor)
     * Used for coarse level scoring where subpixel precision is not needed.
     * @return true if position is valid
     */
    bool GetGradientAtFast(int32_t level, int32_t x, int32_t y,
                           float& gx, float& gy) const;

    /**
     * @brief Get raw gradient data pointers for direct access (fastest)
     * @return true if level is valid
     */
    bool GetGradientData(int32_t level, const float*& gxData, const float*& gyData,
                         int32_t& width, int32_t& height, int32_t& stride) const;

    /**
     * @brief Get quantized angle bin data pointer for direct access
     * Used for direction-quantized scoring optimization
     * @param level Pyramid level
     * @param binData Output pointer to bin data (int16_t)
     * @param width Output image width
     * @param height Output image height
     * @param stride Output stride in elements (not bytes)
     * @param numBins Output number of angle bins
     * @return true if level is valid
     */
    bool GetAngleBinData(int32_t level, const int16_t*& binData,
                         int32_t& width, int32_t& height, int32_t& stride,
                         int32_t& numBins) const;

    // =========================================================================
    // Edge Point Extraction
    // =========================================================================

    /**
     * @brief Extract edge points from a level
     * @param level Pyramid level
     * @param roi Optional ROI (empty = full image)
     * @param minContrast Minimum gradient magnitude (0 = use default)
     * @return Vector of edge points
     */
    std::vector<EdgePoint> ExtractEdgePoints(int32_t level,
                                              const Rect2i& roi = Rect2i(),
                                              double minContrast = 0) const;

    /**
     * @brief Get pre-extracted edge points from a level
     */
    const std::vector<EdgePoint>& GetEdgePoints(int32_t level) const;

    // =========================================================================
    // Coordinate Transform
    // =========================================================================

    /**
     * @brief Transform coordinates from original to pyramid level
     */
    Point2d ToLevelCoords(int32_t level, const Point2d& original) const;

    /**
     * @brief Transform coordinates from pyramid level to original
     */
    Point2d ToOriginalCoords(int32_t level, const Point2d& levelCoords) const;

    // =========================================================================
    // Utility
    // =========================================================================

    /**
     * @brief Quantize angle to bin index
     * @param angle Angle in radians
     * @return Bin index [0, angleBins)
     */
    int32_t AngleToBin(double angle) const;

    /**
     * @brief Get center angle of a bin
     * @param bin Bin index
     * @return Center angle in radians
     */
    double BinToAngle(int32_t bin) const;

    /**
     * @brief Compute angle difference considering wrap-around
     * @return Difference in radians [0, PI]
     */
    static double AngleDifference(double angle1, double angle2);

    /**
     * @brief Compute cosine similarity between two angles
     * @return cos(angle1 - angle2) in range [-1, 1]
     */
    static double AngleSimilarity(double angle1, double angle2);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Compute gradient direction image
 * @param gradX X gradient image (float)
 * @param gradY Y gradient image (float)
 * @return Direction image in radians [0, 2*PI) (float)
 */
QImage ComputeGradientDirection(const QImage& gradX, const QImage& gradY);

/**
 * @brief Compute gradient magnitude image
 */
QImage ComputeGradientMagnitude(const QImage& gradX, const QImage& gradY);

/**
 * @brief Quantize gradient direction to angle bins
 * @param gradDir Direction image in radians
 * @param numBins Number of angle bins
 * @return Quantized angle bin image (int16)
 */
QImage QuantizeGradientDirection(const QImage& gradDir, int32_t numBins);

/**
 * @brief Compute optimal number of pyramid levels for image size
 */
int32_t ComputeOptimalPyramidLevels(int32_t width, int32_t height,
                                     int32_t minSize = 16);

} // namespace Qi::Vision::Internal
