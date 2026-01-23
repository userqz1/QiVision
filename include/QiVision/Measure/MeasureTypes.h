#pragma once

/**
 * @file MeasureTypes.h
 * @brief Measure module type definitions
 *
 * Provides:
 * - Edge transition types
 * - Edge and pair result structures
 * - Score and quality metrics
 * - String parsing utilities for Halcon compatibility
 */

#include <QiVision/Core/Types.h>

#include <cstdint>
#include <string>
#include <vector>

namespace Qi::Vision::Measure {

// =============================================================================
// Constants
// =============================================================================

/// Default minimum edge amplitude
constexpr double DEFAULT_MIN_AMPLITUDE = 20.0;

/// Default Gaussian smoothing sigma
constexpr double DEFAULT_SIGMA = 1.0;

/// Default number of perpendicular lines for averaging
constexpr int32_t DEFAULT_NUM_LINES = 10;

/// Default interpolation samples per pixel
constexpr double DEFAULT_SAMPLES_PER_PIXEL = 1.0;

/// Maximum number of edges to return
constexpr int32_t MAX_EDGES = 1000;

// =============================================================================
// Enumerations
// =============================================================================

/**
 * @brief Edge transition type (polarity)
 */
enum class EdgeTransition {
    Positive,       ///< Dark to light (rising edge)
    Negative,       ///< Light to dark (falling edge)
    All             ///< Both transitions
};

/**
 * @brief Edge selection mode
 */
enum class EdgeSelectMode {
    All,            ///< Return all detected edges
    First,          ///< First edge only (along profile direction)
    Last,           ///< Last edge only
    Strongest,      ///< Edge with highest amplitude
    Weakest         ///< Edge with lowest amplitude (above threshold)
};

/**
 * @brief Edge pair selection mode
 */
enum class PairSelectMode {
    All,            ///< All valid pairs
    First,          ///< First pair only
    Last,           ///< Last pair only
    Strongest,      ///< Pair with highest combined amplitude
    Widest,         ///< Pair with largest distance
    Narrowest       ///< Pair with smallest distance
};

/**
 * @brief Interpolation method for profile extraction
 */
enum class ProfileInterpolation {
    Nearest,        ///< Nearest neighbor (fast)
    Bilinear,       ///< Bilinear (default, good balance)
    Bicubic         ///< Bicubic (highest accuracy)
};

/**
 * @brief Score computation method for fuzzy measurement
 */
enum class ScoreMethod {
    Amplitude,      ///< Based on edge amplitude only
    AmplitudeScore, ///< Amplitude normalized by max possible
    Contrast,       ///< Local contrast ratio
    FuzzyScore      ///< Combined fuzzy logic score
};

// =============================================================================
// Result Structures
// =============================================================================

/**
 * @brief Single edge measurement result
 */
struct EdgeResult {
    // Position in image coordinates (subpixel)
    double row = 0.0;           ///< Y coordinate (row)
    double column = 0.0;        ///< X coordinate (column)

    // Position along profile
    double profilePosition = 0.0;   ///< Position along measurement profile [0, length]

    // Edge properties
    double amplitude = 0.0;     ///< Edge amplitude (gradient magnitude)
    EdgeTransition transition = EdgeTransition::Positive; ///< Edge polarity

    // Quality metrics
    double score = 0.0;         ///< Quality score [0, 1] (for fuzzy)
    double confidence = 0.0;    ///< Detection confidence [0, 1]

    // Optional: edge direction
    double angle = 0.0;         ///< Edge normal angle (radians)

    /// Check if result is valid
    bool IsValid() const { return amplitude > 0 && confidence > 0; }

    /// Get position as Point2d
    Point2d Position() const { return {column, row}; }
};

/**
 * @brief Edge pair (width) measurement result (Halcon compatible)
 *
 * Halcon output mapping:
 * - RowEdgeFirst, ColEdgeFirst, AmplitudeFirst -> first.row, first.column, first.amplitude
 * - RowEdgeSecond, ColEdgeSecond, AmplitudeSecond -> second.row, second.column, second.amplitude
 * - IntraDistance -> intraDistance (width between edges of this pair)
 * - InterDistance -> interDistance (distance to next pair)
 */
struct PairResult {
    EdgeResult first;           ///< First edge of pair
    EdgeResult second;          ///< Second edge of pair

    // Pair metrics (Halcon compatible names)
    double intraDistance = 0.0; ///< Distance between edges of this pair (Halcon: IntraDistance)
    double interDistance = 0.0; ///< Distance to next pair (Halcon: InterDistance)
    double centerRow = 0.0;     ///< Center Y coordinate
    double centerColumn = 0.0;  ///< Center X coordinate

    // Legacy alias
    double width = 0.0;         ///< Same as intraDistance (for compatibility)

    // Quality
    double score = 0.0;         ///< Combined pair score [0, 1]
    double symmetry = 0.0;      ///< Amplitude symmetry [0, 1]

    /// Check if result is valid
    bool IsValid() const {
        return first.IsValid() && second.IsValid() && intraDistance > 0;
    }

    /// Get center position
    Point2d Center() const { return {centerColumn, centerRow}; }
};

/**
 * @brief Measurement statistics
 */
struct MeasureStats {
    int32_t numEdgesFound = 0;      ///< Total edges detected
    int32_t numEdgesReturned = 0;   ///< Edges after filtering

    double meanAmplitude = 0.0;     ///< Mean edge amplitude
    double maxAmplitude = 0.0;      ///< Maximum amplitude
    double minAmplitude = 0.0;      ///< Minimum amplitude (of detected)

    double profileContrast = 0.0;   ///< Overall profile contrast
    double signalNoiseRatio = 0.0;  ///< Estimated SNR
};

// =============================================================================
// Conversion Functions
// =============================================================================

/**
 * @brief Convert EdgeTransition to Internal EdgePolarity
 */
inline EdgePolarity ToEdgePolarity(EdgeTransition t) {
    switch (t) {
        case EdgeTransition::Positive: return EdgePolarity::Positive;
        case EdgeTransition::Negative: return EdgePolarity::Negative;
        case EdgeTransition::All:      return EdgePolarity::Both;
    }
    return EdgePolarity::Both;
}

/**
 * @brief Convert Internal EdgePolarity to EdgeTransition
 */
inline EdgeTransition FromEdgePolarity(EdgePolarity p) {
    switch (p) {
        case EdgePolarity::Positive: return EdgeTransition::Positive;
        case EdgePolarity::Negative: return EdgeTransition::Negative;
        case EdgePolarity::Both:     return EdgeTransition::All;
    }
    return EdgeTransition::All;
}

// =============================================================================
// Halcon String Parameter Parsing
// =============================================================================

/**
 * @brief Parse Halcon transition string to EdgeTransition
 * @param transition "positive", "negative", "all" (case-insensitive)
 * @return EdgeTransition enum value
 */
EdgeTransition ParseTransition(const std::string& transition);

/**
 * @brief Parse Halcon select string to EdgeSelectMode
 * @param select "first", "last", "all" (case-insensitive)
 * @return EdgeSelectMode enum value
 */
EdgeSelectMode ParseEdgeSelect(const std::string& select);

/**
 * @brief Parse Halcon select string to PairSelectMode
 * @param select "first", "last", "all" (case-insensitive)
 * @return PairSelectMode enum value
 */
PairSelectMode ParsePairSelect(const std::string& select);

/**
 * @brief Parse Halcon interpolation string
 * @param interpolation "nearest", "bilinear", "bicubic" (case-insensitive)
 * @return ProfileInterpolation enum value
 */
ProfileInterpolation ParseInterpolation(const std::string& interpolation);

/**
 * @brief Convert EdgeTransition to Halcon string
 */
std::string TransitionToString(EdgeTransition t);

/**
 * @brief Convert EdgeSelectMode to Halcon string
 */
std::string EdgeSelectToString(EdgeSelectMode m);

} // namespace Qi::Vision::Measure
