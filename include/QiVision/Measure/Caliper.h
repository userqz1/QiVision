#pragma once

/**
 * @file Caliper.h
 * @brief Caliper measurement functions (Halcon compatible API)
 *
 * Provides:
 * - Edge position measurement (MeasurePos)
 * - Edge pair/width measurement (MeasurePairs)
 * - Fuzzy (robust) variants with scoring
 * - Profile extraction and analysis
 *
 * API Style:
 * - Halcon compatible: direct parameters, no struct wrappers
 * - String parameters: "positive", "negative", "all", "first", "last", etc.
 *
 * Precision targets (standard conditions: contrast>=50, noise sigma<=5):
 * - Position: < 0.03 px (1 sigma)
 * - Width: < 0.05 px (1 sigma)
 *
 * Thread safety:
 * - All functions are thread-safe for const handles
 * - Multiple threads can measure with same handle simultaneously
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Measure/MeasureTypes.h>
#include <QiVision/Measure/MeasureHandle.h>

#include <vector>

namespace Qi::Vision::Measure {

// =============================================================================
// Edge Position Measurement
// =============================================================================

/**
 * @brief Measure edge positions in rectangular region (Halcon compatible)
 *
 * Extracts profile along the measurement direction, detects edges using
 * 1D edge detection with optional Gaussian smoothing.
 *
 * @param image Input image (grayscale)
 * @param handle Measurement handle
 * @param sigma Gaussian smoothing sigma (Halcon: Sigma)
 * @param threshold Edge amplitude threshold (Halcon: Threshold)
 * @param transition "positive", "negative", "all" (Halcon: Transition)
 * @param select "first", "last", "all" (Halcon: Select)
 * @return Vector of edge results (may be empty if no edges found)
 *
 * @note Results are sorted by profile position (ascending)
 * @note Use higher sigma for noisy images
 *
 * Example:
 * @code
 * auto handle = GenMeasureRectangle2(100, 200, 0, 50, 10);
 * auto edges = MeasurePos(image, handle, 1.5, 30.0, "all", "first");
 * for (const auto& e : edges) {
 *     std::cout << "Edge at (" << e.column << ", " << e.row << ")\n";
 * }
 * @endcode
 */
std::vector<EdgeResult> MeasurePos(const QImage& image,
                                    const MeasureRectangle2& handle,
                                    double sigma,
                                    double threshold,
                                    const std::string& transition,
                                    const std::string& select);

/**
 * @brief Measure edge positions along arc (Halcon compatible)
 */
std::vector<EdgeResult> MeasurePos(const QImage& image,
                                    const MeasureArc& handle,
                                    double sigma,
                                    double threshold,
                                    const std::string& transition,
                                    const std::string& select);

/**
 * @brief Measure edge positions along concentric circles (radial)
 */
std::vector<EdgeResult> MeasurePos(const QImage& image,
                                    const MeasureConcentricCircles& handle,
                                    double sigma,
                                    double threshold,
                                    const std::string& transition,
                                    const std::string& select);

// =============================================================================
// Edge Pair (Width) Measurement
// =============================================================================

/**
 * @brief Measure edge pairs (width) in rectangular region (Halcon compatible)
 *
 * Detects pairs of edges for width measurement. The transition parameter
 * controls edge pairing:
 * - "positive": first edge positive (dark->bright), second negative (bright->dark)
 * - "negative": first edge negative (bright->dark), second positive (dark->bright)
 * - "all": any valid edge pair
 *
 * @param image Input image (grayscale)
 * @param handle Measurement handle
 * @param sigma Gaussian smoothing sigma (Halcon: Sigma)
 * @param threshold Edge amplitude threshold (Halcon: Threshold)
 * @param transition "positive", "negative", "all" (Halcon: Transition)
 * @param select "first", "last", "all" (Halcon: Select)
 * @return Vector of pair results with intraDistance and interDistance
 *
 * Example:
 * @code
 * auto handle = GenMeasureRectangle2(100, 200, 0, 100, 10);
 * auto pairs = MeasurePairs(image, handle, 1.0, 25.0, "positive", "first");
 * if (!pairs.empty()) {
 *     std::cout << "Width: " << pairs[0].intraDistance << " pixels\n";
 * }
 * @endcode
 */
std::vector<PairResult> MeasurePairs(const QImage& image,
                                      const MeasureRectangle2& handle,
                                      double sigma,
                                      double threshold,
                                      const std::string& transition,
                                      const std::string& select);

/**
 * @brief Measure edge pairs along arc (Halcon compatible)
 */
std::vector<PairResult> MeasurePairs(const QImage& image,
                                      const MeasureArc& handle,
                                      double sigma,
                                      double threshold,
                                      const std::string& transition,
                                      const std::string& select);

/**
 * @brief Measure edge pairs along concentric circles
 */
std::vector<PairResult> MeasurePairs(const QImage& image,
                                      const MeasureConcentricCircles& handle,
                                      double sigma,
                                      double threshold,
                                      const std::string& transition,
                                      const std::string& select);

// =============================================================================
// Fuzzy (Robust) Measurement
// =============================================================================

/**
 * @brief Fuzzy edge position measurement with scoring (Halcon compatible)
 *
 * Similar to MeasurePos but computes quality scores for each edge.
 * Scores are based on amplitude, local contrast, and consistency.
 * Better for uncertain edge conditions.
 *
 * @param image Input image
 * @param handle Measurement handle
 * @param sigma Gaussian smoothing sigma
 * @param threshold Edge amplitude threshold
 * @param transition "positive", "negative", "all"
 * @param select "first", "last", "all"
 * @param fuzzyThresh Minimum fuzzy score threshold [0, 1], default 0.5
 * @param stats Optional output statistics
 * @return Vector of edge results with scores
 *
 * @note Scores in [0, 1], higher is better
 * @note Low-score edges can be filtered by fuzzyThresh
 */
std::vector<EdgeResult> FuzzyMeasurePos(const QImage& image,
                                         const MeasureRectangle2& handle,
                                         double sigma,
                                         double threshold,
                                         const std::string& transition,
                                         const std::string& select,
                                         double fuzzyThresh = 0.5,
                                         MeasureStats* stats = nullptr);

std::vector<EdgeResult> FuzzyMeasurePos(const QImage& image,
                                         const MeasureArc& handle,
                                         double sigma,
                                         double threshold,
                                         const std::string& transition,
                                         const std::string& select,
                                         double fuzzyThresh = 0.5,
                                         MeasureStats* stats = nullptr);

std::vector<EdgeResult> FuzzyMeasurePos(const QImage& image,
                                         const MeasureConcentricCircles& handle,
                                         double sigma,
                                         double threshold,
                                         const std::string& transition,
                                         const std::string& select,
                                         double fuzzyThresh = 0.5,
                                         MeasureStats* stats = nullptr);

/**
 * @brief Fuzzy edge pair measurement with scoring (Halcon compatible)
 *
 * @param image Input image
 * @param handle Measurement handle
 * @param sigma Gaussian smoothing sigma
 * @param threshold Edge amplitude threshold
 * @param transition "positive", "negative", "all"
 * @param select "first", "last", "all"
 * @param fuzzyThresh Minimum fuzzy score threshold [0, 1], default 0.5
 * @param stats Optional output statistics
 * @return Vector of pair results with scores
 */
std::vector<PairResult> FuzzyMeasurePairs(const QImage& image,
                                           const MeasureRectangle2& handle,
                                           double sigma,
                                           double threshold,
                                           const std::string& transition,
                                           const std::string& select,
                                           double fuzzyThresh = 0.5,
                                           MeasureStats* stats = nullptr);

std::vector<PairResult> FuzzyMeasurePairs(const QImage& image,
                                           const MeasureArc& handle,
                                           double sigma,
                                           double threshold,
                                           const std::string& transition,
                                           const std::string& select,
                                           double fuzzyThresh = 0.5,
                                           MeasureStats* stats = nullptr);

std::vector<PairResult> FuzzyMeasurePairs(const QImage& image,
                                           const MeasureConcentricCircles& handle,
                                           double sigma,
                                           double threshold,
                                           const std::string& transition,
                                           const std::string& select,
                                           double fuzzyThresh = 0.5,
                                           MeasureStats* stats = nullptr);

// =============================================================================
// Profile Extraction (for debugging/visualization)
// =============================================================================

/**
 * @brief Extract measurement profile from image using rectangle handle
 *
 * @param image Input image
 * @param handle Measurement handle
 * @param interp Interpolation method: "nearest", "bilinear", "bicubic"
 * @return Profile data (gray values along profile)
 */
std::vector<double> ExtractMeasureProfile(const QImage& image,
                                           const MeasureRectangle2& handle,
                                           const std::string& interp = "bilinear");

/**
 * @brief Extract measurement profile from image using arc handle
 */
std::vector<double> ExtractMeasureProfile(const QImage& image,
                                           const MeasureArc& handle,
                                           const std::string& interp = "bilinear");

/**
 * @brief Extract measurement profile from image using concentric handle
 */
std::vector<double> ExtractMeasureProfile(const QImage& image,
                                           const MeasureConcentricCircles& handle,
                                           const std::string& interp = "bilinear");

// =============================================================================
// Coordinate Transformation
// =============================================================================

/**
 * @brief Convert profile position to image coordinates for rectangle handle
 *
 * @param handle Measurement handle
 * @param profilePos Position along profile [0, ProfileLength]
 * @return Image coordinates (x=column, y=row)
 */
Point2d ProfileToImage(const MeasureRectangle2& handle, double profilePos);

/**
 * @brief Convert profile position to image coordinates for arc handle
 */
Point2d ProfileToImage(const MeasureArc& handle, double profilePos);

/**
 * @brief Convert profile position to image coordinates for concentric handle
 */
Point2d ProfileToImage(const MeasureConcentricCircles& handle, double profilePos);

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Compute expected number of samples for handle
 */
int32_t GetNumSamples(const MeasureRectangle2& handle);
int32_t GetNumSamples(const MeasureArc& handle);
int32_t GetNumSamples(const MeasureConcentricCircles& handle);

/**
 * @brief Filter edges by selection mode
 */
std::vector<EdgeResult> SelectEdges(const std::vector<EdgeResult>& edges,
                                     EdgeSelectMode mode,
                                     int32_t maxCount = MAX_EDGES);

/**
 * @brief Filter pairs by selection mode
 */
std::vector<PairResult> SelectPairs(const std::vector<PairResult>& pairs,
                                     PairSelectMode mode,
                                     int32_t maxCount = MAX_EDGES);

/**
 * @brief Sort edges by various criteria
 */
enum class EdgeSortBy { Position, Amplitude, Score };
void SortEdges(std::vector<EdgeResult>& edges, EdgeSortBy criterion, bool ascending = true);

/**
 * @brief Sort pairs by various criteria
 */
enum class PairSortBy { Position, Width, Score, Symmetry };
void SortPairs(std::vector<PairResult>& pairs, PairSortBy criterion, bool ascending = true);

} // namespace Qi::Vision::Measure
