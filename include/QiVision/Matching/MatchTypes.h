#pragma once

/**
 * @file MatchTypes.h
 * @brief Common types and structures for template matching
 *
 * Provides:
 * - Match result structures
 * - Search parameters
 * - Common enumerations for matching algorithms
 *
 * Used by:
 * - ShapeModel: gradient-based shape matching
 * - NCCModel: normalized cross-correlation matching
 * - ComponentModel: multi-component matching
 */

#include <QiVision/Core/Types.h>

#include <array>
#include <cstdint>
#include <cmath>
#include <vector>
#include <string>

namespace Qi::Vision::Matching {

// =============================================================================
// Enumerations
// =============================================================================

/**
 * @brief Matching algorithm type
 */
enum class MatchMethod {
    Shape,          ///< Gradient direction-based shape matching
    NCC,            ///< Normalized cross-correlation
    Component,      ///< Multi-component with spatial constraints
    Deformable      ///< Deformable template matching
};

/**
 * @brief Subpixel refinement method
 */
enum class SubpixelMethod {
    None,           ///< No subpixel refinement (integer positions only)
    Parabolic,      ///< Parabolic fitting (fast, ~0.1px accuracy)
    LeastSquares,   ///< Least squares optimization (slower, ~0.02px accuracy)
    Gradient        ///< Gradient descent refinement
};

/**
 * @brief Angle search mode
 */
enum class AngleSearchMode {
    Full,           ///< Full 360 degree search
    Range,          ///< Search within specified range
    Discrete        ///< Search at discrete angle steps
};

/**
 * @brief Scale search mode
 */
enum class ScaleSearchMode {
    Fixed,          ///< Fixed scale (no scale search)
    Uniform,        ///< Uniform scale (same X and Y)
    Anisotropic     ///< Anisotropic scale (different X and Y)
};

/**
 * @brief Polarity for matching
 */
enum class MatchPolarity {
    Same,           ///< Match same polarity (dark on light or light on dark)
    Opposite,       ///< Match opposite polarity
    Any             ///< Match either polarity
};

/**
 * @brief Metric for matching score computation
 */
enum class MatchMetric {
    GradientCosine, ///< Cosine similarity of gradient directions (shape matching)
    NCC,            ///< Normalized cross-correlation
    SAD,            ///< Sum of absolute differences
    SSD             ///< Sum of squared differences
};

// =============================================================================
// Match Result
// =============================================================================

/**
 * @brief Single match result
 *
 * Contains position, orientation, scale and quality metrics
 * for a single template match.
 */
struct MatchResult {
    // Position (subpixel)
    double x = 0.0;             ///< X position (column) in image coordinates
    double y = 0.0;             ///< Y position (row) in image coordinates

    // Orientation
    double angle = 0.0;         ///< Rotation angle in radians

    // Scale
    double scaleX = 1.0;        ///< Scale factor in X
    double scaleY = 1.0;        ///< Scale factor in Y

    // Quality
    double score = 0.0;         ///< Match score [0, 1], higher is better

    // Additional metrics
    double coverage = 0.0;      ///< Percentage of model points matched [0, 1]
    double contrast = 0.0;      ///< Average gradient contrast at match

    // Refinement info
    int32_t pyramidLevel = 0;   ///< Pyramid level where match was found
    bool refined = false;       ///< Whether subpixel refinement was applied

    /**
     * @brief Get transformation matrix (2x3 affine)
     * @return Affine transformation from model to image coordinates
     */
    std::array<double, 6> GetTransform() const {
        double cosA = std::cos(angle);
        double sinA = std::sin(angle);
        return {
            scaleX * cosA, -scaleY * sinA, x,
            scaleX * sinA,  scaleY * cosA, y
        };
    }

    /**
     * @brief Transform a point from model to image coordinates
     */
    Point2d TransformPoint(const Point2d& modelPoint) const {
        double cosA = std::cos(angle);
        double sinA = std::sin(angle);
        return Point2d{
            x + scaleX * (cosA * modelPoint.x - sinA * modelPoint.y),
            y + scaleY * (sinA * modelPoint.x + cosA * modelPoint.y)
        };
    }

    /**
     * @brief Comparison for sorting (by score, descending)
     */
    bool operator<(const MatchResult& other) const {
        return score > other.score;  // Higher score = better match
    }
};

// =============================================================================
// Search Parameters
// =============================================================================

/**
 * @brief Parameters for template search
 */
struct SearchParams {
    // Score threshold
    double minScore = 0.5;          ///< Minimum score to accept [0, 1]

    // Number of results
    int32_t maxMatches = 0;         ///< Maximum matches to return (0 = all above threshold)

    // Angle search
    AngleSearchMode angleMode = AngleSearchMode::Full;
    double angleStart = 0.0;        ///< Start angle (radians)
    double angleExtent = 2.0 * 3.14159265358979323846;  ///< Angle range (radians)
    double angleStep = 0.0;         ///< Angle step (0 = auto-compute)

    // Scale search
    ScaleSearchMode scaleMode = ScaleSearchMode::Fixed;
    double scaleMin = 1.0;          ///< Minimum scale
    double scaleMax = 1.0;          ///< Maximum scale
    double scaleStep = 0.0;         ///< Scale step (0 = auto-compute)

    // Search region
    Rect2i searchROI;               ///< Region of interest (empty = full image)

    // Refinement
    SubpixelMethod subpixelMethod = SubpixelMethod::LeastSquares;

    // Performance
    int32_t numLevels = 0;          ///< Pyramid levels (0 = auto)
    double greediness = 0.9;        ///< Greediness for early termination [0, 1]

    // Polarity
    MatchPolarity polarity = MatchPolarity::Same;

    // Non-maximum suppression (Halcon-compatible)
    double maxOverlap = 0.5;        ///< Maximum overlap ratio for NMS [0, 1] (Halcon default: 0.5)

    // Builder pattern
    SearchParams& SetMinScore(double v) { minScore = v; return *this; }
    SearchParams& SetMaxMatches(int32_t v) { maxMatches = v; return *this; }
    SearchParams& SetAngleRange(double start, double extent) {
        angleMode = AngleSearchMode::Range;
        angleStart = start;
        angleExtent = extent;
        return *this;
    }
    SearchParams& SetScaleRange(double min, double max) {
        scaleMode = ScaleSearchMode::Uniform;
        scaleMin = min;
        scaleMax = max;
        return *this;
    }
    SearchParams& SetROI(const Rect2i& roi) { searchROI = roi; return *this; }
    SearchParams& SetSubpixel(SubpixelMethod method) { subpixelMethod = method; return *this; }
    SearchParams& SetGreediness(double v) { greediness = v; return *this; }
    SearchParams& SetPolarity(MatchPolarity p) { polarity = p; return *this; }
    SearchParams& SetMaxOverlap(double v) { maxOverlap = v; return *this; }
};

// =============================================================================
// Model Creation Parameters (Halcon-compatible)
// =============================================================================

/**
 * @brief Contrast detection mode for model creation
 * Corresponds to Halcon's Contrast parameter
 */
enum class ContrastMode {
    Manual,             ///< Use manually specified threshold
    Auto,               ///< Auto-detect contrast threshold ('auto_contrast')
    AutoHysteresis,     ///< Auto-detect hysteresis thresholds ('auto_contrast_hyst')
    AutoMinSize         ///< Auto with minimum component size filter
};

/**
 * @brief Point reduction optimization level
 * Corresponds to Halcon's Optimization parameter
 */
enum class OptimizationMode {
    None,               ///< Keep all model points ('none')
    PointReductionLow,  ///< Light point reduction ('point_reduction_low')
    PointReductionMedium, ///< Medium reduction ('point_reduction_medium')
    PointReductionHigh, ///< Heavy reduction ('point_reduction_high')
    Auto                ///< Automatic selection ('auto')
};

/**
 * @brief Polarity/Metric mode for matching
 * Corresponds to Halcon's Metric parameter
 */
enum class MetricMode {
    UsePolarity,           ///< Match polarity must match ('use_polarity')
    IgnoreGlobalPolarity,  ///< Ignore global polarity ('ignore_global_polarity')
    IgnoreLocalPolarity,   ///< Ignore local polarity ('ignore_local_polarity')
    IgnoreColorPolarity    ///< Ignore color polarity ('ignore_color_polarity')
};

/**
 * @brief Parameters for model creation
 *
 * Follows Halcon's create_shape_model parameter design:
 * - Contrast: threshold for edge extraction
 * - Optimization: point reduction level
 * - Metric: polarity handling mode
 * - NumLevels: pyramid levels (0 = auto)
 * - AngleStep: angle resolution (0 = auto)
 */
struct ModelParams {
    // =========================================================================
    // Contrast Parameters (edge extraction threshold)
    // =========================================================================
    ContrastMode contrastMode = ContrastMode::Manual;
    double contrastHigh = 30.0;     ///< High threshold (single or hysteresis high)
    double contrastLow = 0.0;       ///< Low threshold for hysteresis (0 = single threshold)
    double contrastMax = 10000.0;   ///< Maximum contrast (filter very strong edges)
    int32_t minComponentSize = 3;   ///< Min connected component size (for AutoMinSize)

    // =========================================================================
    // MinContrast (for search, not creation)
    // =========================================================================
    double minContrast = 0.0;       ///< Minimum contrast at search time (0 = auto)

    // =========================================================================
    // Optimization (point reduction)
    // =========================================================================
    OptimizationMode optimization = OptimizationMode::Auto;
    bool pregeneration = false;     ///< Pre-generate rotated models (memory vs speed)

    // =========================================================================
    // Metric (polarity handling)
    // =========================================================================
    MetricMode metric = MetricMode::UsePolarity;

    // =========================================================================
    // Pyramid Parameters
    // =========================================================================
    int32_t numLevels = 0;          ///< Pyramid levels (0 = auto, typically 4-6)
    int32_t startLevel = 0;         ///< Starting search level (0 = finest)

    // =========================================================================
    // Angle Parameters
    // =========================================================================
    double angleStart = 0.0;        ///< Start angle (radians)
    double angleExtent = 0.0;       ///< Angle extent (0 = all orientations)
    double angleStep = 0.0;         ///< Angle step (0 = auto, typically ~1°)

    // =========================================================================
    // Scale Parameters
    // =========================================================================
    double scaleMin = 1.0;          ///< Minimum scale
    double scaleMax = 1.0;          ///< Maximum scale

    // =========================================================================
    // Builder Pattern Methods
    // =========================================================================

    /// Set manual contrast threshold (single value)
    ModelParams& SetContrast(double threshold) {
        contrastMode = ContrastMode::Manual;
        contrastHigh = threshold;
        contrastLow = 0.0;
        return *this;
    }

    /// Set hysteresis contrast thresholds [low, high]
    ModelParams& SetContrastHysteresis(double low, double high) {
        contrastMode = ContrastMode::Manual;
        contrastHigh = high;
        contrastLow = low;
        return *this;
    }

    /// Set auto contrast detection
    ModelParams& SetContrastAuto() {
        contrastMode = ContrastMode::Auto;
        return *this;
    }

    /// Set auto contrast with hysteresis
    ModelParams& SetContrastAutoHysteresis() {
        contrastMode = ContrastMode::AutoHysteresis;
        return *this;
    }

    /// Set optimization mode
    ModelParams& SetOptimization(OptimizationMode mode) {
        optimization = mode;
        return *this;
    }

    /// Set metric (polarity handling) mode
    ModelParams& SetMetric(MetricMode mode) {
        metric = mode;
        return *this;
    }

    /// Set pyramid levels (0 = auto)
    ModelParams& SetNumLevels(int32_t n) {
        numLevels = n;
        return *this;
    }

    /// Set angle range
    ModelParams& SetAngleRange(double start, double extent) {
        angleStart = start;
        angleExtent = extent;
        return *this;
    }

    /// Set angle step (0 = auto)
    ModelParams& SetAngleStep(double step) {
        angleStep = step;
        return *this;
    }

    /// Set scale range
    ModelParams& SetScaleRange(double min, double max) {
        scaleMin = min;
        scaleMax = max;
        return *this;
    }

    /// Set minimum contrast for search
    ModelParams& SetMinContrast(double mc) {
        minContrast = mc;
        return *this;
    }

    // Legacy compatibility
    MatchPolarity polarity = MatchPolarity::Same;
    ModelParams& SetPolarity(MatchPolarity p) { polarity = p; return *this; }
};

// =============================================================================
// Model Point
// =============================================================================

/**
 * @brief Single point in a shape model
 */
struct ModelPoint {
    double x = 0.0;             ///< X position relative to model origin
    double y = 0.0;             ///< Y position relative to model origin
    double angle = 0.0;         ///< Gradient direction (radians)
    double magnitude = 0.0;     ///< Gradient magnitude
    int32_t angleBin = 0;       ///< Quantized angle bin
    double weight = 1.0;        ///< Point weight for scoring

    // Precomputed for SIMD optimization (avoid trig in inner loop)
    double cosAngle = 1.0;      ///< cos(angle) - precomputed
    double sinAngle = 0.0;      ///< sin(angle) - precomputed

    ModelPoint() = default;
    ModelPoint(double x_, double y_, double ang, double mag, int32_t bin, double w = 1.0)
        : x(x_), y(y_), angle(ang), magnitude(mag), angleBin(bin), weight(w),
          cosAngle(std::cos(ang)), sinAngle(std::sin(ang)) {}
};

// =============================================================================
// Model Statistics
// =============================================================================

/**
 * @brief Statistics about a created model
 */
struct ModelStats {
    int32_t numPoints = 0;          ///< Total number of model points
    int32_t numLevels = 0;          ///< Number of pyramid levels
    double meanContrast = 0.0;      ///< Mean gradient magnitude
    double minContrast = 0.0;       ///< Minimum gradient magnitude
    double maxContrast = 0.0;       ///< Maximum gradient magnitude

    // Bounding box of model points
    double minX = 0.0;
    double maxX = 0.0;
    double minY = 0.0;
    double maxY = 0.0;

    // Points per level
    std::vector<int32_t> pointsPerLevel;

    double Width() const { return maxX - minX; }
    double Height() const { return maxY - minY; }
};

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Non-maximum suppression for match results (distance-based)
 * @param matches Input matches
 * @param minDistance Minimum distance between matches
 * @return Filtered matches
 */
inline std::vector<MatchResult> NonMaxSuppression(
    const std::vector<MatchResult>& matches,
    double minDistance = 10.0)
{
    if (matches.empty()) return {};

    // Matches should already be sorted by score (descending)
    std::vector<MatchResult> result;
    result.reserve(matches.size());

    double minDistSq = minDistance * minDistance;

    for (const auto& match : matches) {
        bool suppress = false;
        for (const auto& kept : result) {
            double dx = match.x - kept.x;
            double dy = match.y - kept.y;
            if (dx * dx + dy * dy < minDistSq) {
                suppress = true;
                break;
            }
        }
        if (!suppress) {
            result.push_back(match);
        }
    }

    return result;
}

/**
 * @brief Non-maximum suppression using overlap ratio (Halcon-compatible)
 * @param matches Input matches (sorted by score descending)
 * @param maxOverlap Maximum overlap ratio [0, 1] (Halcon default: 0.5)
 * @param modelWidth Model bounding box width
 * @param modelHeight Model bounding box height
 * @return Filtered matches
 *
 * Overlap is computed as: intersection_area / min(area1, area2)
 * where area is the model's bounding box transformed by each match.
 */
inline std::vector<MatchResult> NonMaxSuppressionOverlap(
    const std::vector<MatchResult>& matches,
    double maxOverlap,
    double modelWidth,
    double modelHeight)
{
    if (matches.empty()) return {};

    // Matches should already be sorted by score (descending)
    std::vector<MatchResult> result;
    result.reserve(matches.size());

    // For axis-aligned bounding boxes (simplified: ignore rotation for speed)
    // More accurate: use OBB intersection, but much slower
    for (const auto& match : matches) {
        bool suppress = false;

        // Compute match bounding box (account for scale)
        double w1 = modelWidth * match.scaleX;
        double h1 = modelHeight * match.scaleY;
        double area1 = w1 * h1;

        // Half-sizes for overlap computation
        double hw1 = w1 * 0.5;
        double hh1 = h1 * 0.5;

        for (const auto& kept : result) {
            double w2 = modelWidth * kept.scaleX;
            double h2 = modelHeight * kept.scaleY;
            double area2 = w2 * h2;
            double hw2 = w2 * 0.5;
            double hh2 = h2 * 0.5;

            // AABB intersection
            double dx = std::abs(match.x - kept.x);
            double dy = std::abs(match.y - kept.y);

            double overlapX = std::max(0.0, hw1 + hw2 - dx);
            double overlapY = std::max(0.0, hh1 + hh2 - dy);
            double intersectionArea = overlapX * overlapY;

            // Overlap ratio = intersection / min(area1, area2)
            double minArea = std::min(area1, area2);
            double overlapRatio = (minArea > 0.0) ? (intersectionArea / minArea) : 0.0;

            if (overlapRatio > maxOverlap) {
                suppress = true;
                break;
            }
        }

        if (!suppress) {
            result.push_back(match);
        }
    }

    return result;
}

/**
 * @brief Filter matches by score
 */
inline std::vector<MatchResult> FilterByScore(
    const std::vector<MatchResult>& matches,
    double minScore)
{
    std::vector<MatchResult> result;
    result.reserve(matches.size());
    for (const auto& m : matches) {
        if (m.score >= minScore) {
            result.push_back(m);
        }
    }
    return result;
}

} // namespace Qi::Vision::Matching
