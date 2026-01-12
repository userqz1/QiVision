#pragma once

/**
 * @file LinemodPyramid.h
 * @brief LINEMOD-style gradient pyramid for shape-based matching
 *
 * Implements the exact algorithm from:
 * - Hinterstoisser et al. "Gradient Response Maps for Real-Time Detection
 *   of Texture-Less Objects" (TPAMI 2012)
 * - "Multimodal Templates for Real-Time Detection" (ICCV 2011)
 *
 * Key features:
 * - 8-bin orientation quantization (45 degrees per bin)
 * - Bit flag representation (bit i = orientation i present)
 * - 3x3 neighbor voting with threshold for robustness
 * - OR spreading for spatial tolerance
 * - Precomputed SIMILARITY_LUT[8][256] for O(1) scoring
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/Types.h>

#include <array>
#include <cstdint>
#include <vector>

namespace Qi::Vision::Internal {

// =============================================================================
// Constants (from LINEMOD paper)
// =============================================================================

/// Number of orientation bins (8 = 45 degree resolution)
constexpr int32_t LINEMOD_NUM_BINS = 8;

/// Angle per bin in degrees
constexpr double LINEMOD_BIN_ANGLE_DEG = 45.0;

/// Default spreading radius T (paper uses 4, we use 3 for speed/quality balance)
constexpr int32_t LINEMOD_DEFAULT_T = 3;

/// Neighbor voting threshold (paper uses 5 for 3x3 = 9 pixels)
constexpr int32_t LINEMOD_NEIGHBOR_THRESHOLD = 5;

/// Minimum gradient magnitude for feature point
constexpr float LINEMOD_MIN_MAGNITUDE = 10.0f;

/// Maximum response score per feature (paper: 4)
constexpr int32_t LINEMOD_MAX_RESPONSE = 4;

// =============================================================================
// SIMILARITY_LUT - Precomputed lookup table
// =============================================================================

/**
 * @brief Precomputed similarity lookup table
 *
 * LUT[ori][mask] = max similarity between orientation 'ori' and any
 * orientation present in 'mask' (8-bit bitmask)
 *
 * Formula: LUT[i][L] = max_{j in L} |cos((i-j) * 45 degrees)|
 * Scaled to [0, 4] for integer arithmetic
 */
class SimilarityLUT {
public:
    SimilarityLUT();

    /// Get similarity score
    /// @param modelOri Model orientation bin [0, 7]
    /// @param imageMask Image orientation bitmask (bit i = orientation i present)
    /// @return Similarity score [0, 4]
    inline uint8_t Get(int32_t modelOri, uint8_t imageMask) const {
        return lut_[modelOri & 7][imageMask];
    }

    /// Direct array access for SIMD
    const uint8_t* GetRow(int32_t ori) const { return lut_[ori & 7].data(); }

private:
    std::array<std::array<uint8_t, 256>, 8> lut_;
};

/// Global instance of similarity LUT
extern const SimilarityLUT g_SimilarityLUT;

// =============================================================================
// Feature Point (LINEMOD style)
// =============================================================================

/**
 * @brief LINEMOD feature point
 *
 * Compact representation: 6 bytes per feature
 */
struct LinemodFeature {
    int16_t x = 0;          ///< X offset from template center
    int16_t y = 0;          ///< Y offset from template center
    uint8_t ori = 0;        ///< Quantized orientation [0, 7]
    uint8_t reserved = 0;   ///< Padding for alignment

    LinemodFeature() = default;
    LinemodFeature(int16_t x_, int16_t y_, uint8_t ori_)
        : x(x_), y(y_), ori(ori_) {}
};

// =============================================================================
// LINEMOD Pyramid Level
// =============================================================================

/**
 * @brief Data for a single pyramid level
 */
struct LinemodLevelData {
    int32_t width = 0;
    int32_t height = 0;
    int32_t stride = 0;         ///< Row stride in bytes (64-byte aligned)
    double scale = 1.0;         ///< Scale factor (0.5^level)

    QImage gradMag;             ///< Gradient magnitude (float)
    QImage quantized;           ///< Quantized orientations before spreading (uint8, bit flags)
    QImage spread;              ///< Spread orientations (uint8, bit flags after OR)

    /// Response maps for each orientation [8][height][stride]
    std::array<std::vector<uint8_t>, LINEMOD_NUM_BINS> responseMaps;

    /// Extracted features for template
    std::vector<LinemodFeature> features;
};

// =============================================================================
// LINEMOD Pyramid Parameters
// =============================================================================

struct LinemodPyramidParams {
    int32_t numLevels = 4;
    float minMagnitude = LINEMOD_MIN_MAGNITUDE;
    int32_t spreadT = LINEMOD_DEFAULT_T;
    int32_t neighborThreshold = LINEMOD_NEIGHBOR_THRESHOLD;
    double smoothSigma = 1.0;                   ///< Gaussian blur before gradient
    bool extractFeatures = true;                ///< Extract features (for template)

    // Builder pattern
    LinemodPyramidParams& SetNumLevels(int32_t n) { numLevels = n; return *this; }
    LinemodPyramidParams& SetMinMagnitude(float v) { minMagnitude = v; return *this; }
    LinemodPyramidParams& SetSpreadT(int32_t v) { spreadT = v; return *this; }
    LinemodPyramidParams& SetNeighborThreshold(int32_t v) { neighborThreshold = v; return *this; }
    LinemodPyramidParams& SetSmoothSigma(double v) { smoothSigma = v; return *this; }
    LinemodPyramidParams& SetExtractFeatures(bool v) { extractFeatures = v; return *this; }
};

// =============================================================================
// LINEMOD Pyramid Class
// =============================================================================

/**
 * @brief LINEMOD-style gradient pyramid
 *
 * Implements the complete LINEMOD pipeline:
 * 1. Gaussian blur + Sobel gradient
 * 2. Quantize orientations to 8 bins (bit flags)
 * 3. 3x3 neighbor voting for robustness
 * 4. OR spreading for spatial tolerance
 * 5. Build response maps for fast scoring
 *
 * Usage for template creation:
 * @code
 * LinemodPyramid pyramid;
 * pyramid.Build(templateImage, params);
 * auto features = pyramid.ExtractFeatures(0, roi, maxFeatures);
 * @endcode
 *
 * Usage for searching:
 * @code
 * LinemodPyramid searchPyramid;
 * searchPyramid.Build(searchImage, params.SetExtractFeatures(false));
 * double score = searchPyramid.ComputeScore(features, level, x, y);
 * @endcode
 */
class LinemodPyramid {
public:
    LinemodPyramid();
    ~LinemodPyramid();

    // Non-copyable, movable
    LinemodPyramid(const LinemodPyramid&) = delete;
    LinemodPyramid& operator=(const LinemodPyramid&) = delete;
    LinemodPyramid(LinemodPyramid&&) noexcept;
    LinemodPyramid& operator=(LinemodPyramid&&) noexcept;

    // =========================================================================
    // Construction
    // =========================================================================

    /**
     * @brief Build pyramid from image
     * @param image Input grayscale image
     * @param params Build parameters
     * @return true if successful
     */
    bool Build(const QImage& image, const LinemodPyramidParams& params = LinemodPyramidParams());

    /// Clear all data
    void Clear();

    /// Check if valid
    bool IsValid() const { return valid_; }

    // =========================================================================
    // Properties
    // =========================================================================

    int32_t NumLevels() const { return numLevels_; }
    int32_t GetWidth(int32_t level) const;
    int32_t GetHeight(int32_t level) const;
    double GetScale(int32_t level) const;
    const LinemodLevelData& GetLevel(int32_t level) const;

    // =========================================================================
    // Feature Extraction (for template creation)
    // =========================================================================

    /**
     * @brief Extract features from a level
     *
     * Selects scattered features with strong gradients using the paper's
     * feature selection strategy.
     *
     * @param level Pyramid level
     * @param roi Region of interest (empty = full image)
     * @param maxFeatures Maximum number of features to extract
     * @param minDistance Minimum distance between features
     * @return Vector of features (relative to ROI center)
     */
    std::vector<LinemodFeature> ExtractFeatures(
        int32_t level,
        const Rect2i& roi = Rect2i(),
        int32_t maxFeatures = 128,
        float minDistance = 3.0f) const;

    /**
     * @brief Get all candidate features before selection
     */
    const std::vector<LinemodFeature>& GetAllFeatures(int32_t level) const;

    // =========================================================================
    // Scoring (for matching)
    // =========================================================================

    /**
     * @brief Compute match score using SIMILARITY_LUT
     *
     * score = (1/N) * sum_{i} LUT[model[i].ori][spread[pos + model[i].offset]]
     *
     * @param features Model features (with offsets from center)
     * @param level Pyramid level
     * @param x Search X position
     * @param y Search Y position
     * @return Normalized score in [0, 1]
     */
    double ComputeScore(const std::vector<LinemodFeature>& features,
                        int32_t level, int32_t x, int32_t y) const;

    /**
     * @brief Compute score with rotated features
     *
     * @param features Original model features
     * @param level Pyramid level
     * @param x Search X position
     * @param y Search Y position
     * @param angle Rotation angle (radians)
     * @return Normalized score in [0, 1]
     */
    double ComputeScoreRotated(const std::vector<LinemodFeature>& features,
                               int32_t level, int32_t x, int32_t y,
                               double angle) const;

    /**
     * @brief Compute score with pre-rotated features (optimized)
     *
     * This is the fast path - features must already be rotated using
     * RotateFeatures(). No cos/sin computation per position.
     *
     * @param rotatedFeatures Pre-rotated features (integer coordinates)
     * @param level Pyramid level
     * @param x Search X position
     * @param y Search Y position
     * @return Normalized score in [0, 1]
     */
    double ComputeScorePrecomputed(const std::vector<LinemodFeature>& rotatedFeatures,
                                    int32_t level, int32_t x, int32_t y) const;

    /**
     * @brief Compute scores for 8 consecutive X positions (SIMD optimized)
     *
     * AVX2/SSE2 optimized batch scoring. Computes scores for positions:
     * (x+0, y), (x+1, y), ..., (x+7, y) simultaneously.
     *
     * @param rotatedFeatures Pre-rotated features
     * @param level Pyramid level
     * @param x Starting X position (must have at least 8 valid positions after it)
     * @param y Y position
     * @param scoresOut Output array for 8 scores (must be at least 8 elements)
     */
    void ComputeScoresBatch8(const std::vector<LinemodFeature>& rotatedFeatures,
                             int32_t level, int32_t x, int32_t y,
                             double* scoresOut) const;

    /**
     * @brief Compute scores for entire row with stride (highly optimized)
     *
     * Processes all X positions in a row at once with specified stride.
     * Uses SIMD to compute multiple positions in parallel.
     *
     * @param rotatedFeatures Pre-rotated features
     * @param level Pyramid level
     * @param xStart Starting X position
     * @param xEnd Ending X position (inclusive)
     * @param y Y position
     * @param stride Step size between positions
     * @param threshold Minimum score threshold
     * @param xPositionsOut Output: X positions that pass threshold
     * @param scoresOut Output: Scores for positions that pass threshold
     */
    void ComputeScoresRow(const std::vector<LinemodFeature>& rotatedFeatures,
                          int32_t level, int32_t xStart, int32_t xEnd, int32_t y,
                          int32_t stride, double threshold,
                          std::vector<int32_t>& xPositionsOut,
                          std::vector<double>& scoresOut) const;

    /**
     * @brief Get spread image data for direct access
     * @return Pointer to spread bitmask image data
     */
    const uint8_t* GetSpreadData(int32_t level) const;
    int32_t GetSpreadStride(int32_t level) const;

    // =========================================================================
    // Utility
    // =========================================================================

    /// Convert angle (radians) to bin index [0, 7]
    static int32_t AngleToBin(double angle);

    /// Convert bin index to center angle (radians)
    static double BinToAngle(int32_t bin);

    /// Rotate feature coordinates and update orientation
    static LinemodFeature RotateFeature(const LinemodFeature& f, double angle);

    /// Rotate multiple features
    static std::vector<LinemodFeature> RotateFeatures(
        const std::vector<LinemodFeature>& features, double angle);

private:
    std::vector<LinemodLevelData> levels_;
    int32_t numLevels_ = 0;
    LinemodPyramidParams params_;
    bool valid_ = false;

    // Internal methods
    bool BuildLevel(const QImage& image, int32_t levelIdx);
    bool BuildLevelFromQuantized(int32_t levelIdx, const LinemodLevelData& prevLevel);
    void QuantizeOrientations(LinemodLevelData& level);
    void ApplyNeighborVoting(LinemodLevelData& level);
    void ApplyORSpreading(LinemodLevelData& level);
    void BuildResponseMaps(LinemodLevelData& level);
    void ExtractLevelFeatures(LinemodLevelData& level);
};

// =============================================================================
// Inline Implementations
// =============================================================================

inline int32_t LinemodPyramid::AngleToBin(double angle) {
    // Normalize to [0, 2*PI)
    constexpr double PI2 = 6.283185307179586;
    constexpr double INV_PI2 = 1.0 / PI2;
    angle = angle - std::floor(angle * INV_PI2) * PI2;

    // Quantize to [0, 7]
    // Note: We use full 360 degree range, each bin covers 45 degrees
    int32_t bin = static_cast<int32_t>(angle * 8.0 * INV_PI2 + 0.5) % 8;
    if (bin < 0) bin += 8;
    return bin;
}

inline double LinemodPyramid::BinToAngle(int32_t bin) {
    constexpr double BIN_ANGLE = 6.283185307179586 / 8.0;  // 45 degrees
    return (bin & 7) * BIN_ANGLE;
}

} // namespace Qi::Vision::Internal
