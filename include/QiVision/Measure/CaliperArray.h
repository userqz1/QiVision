#pragma once

/**
 * @file CaliperArray.h
 * @brief Multi-caliper array measurement for QiVision
 *
 * Provides:
 * - Array generation along paths (line, arc, circle, contour)
 * - Batch measurement with shared parameters
 * - Aggregated results for geometric fitting
 * - Support for edge position and edge pair measurement
 *
 * Reference Halcon operators:
 * - gen_measure_rectangle2_array (create array of rectangle handles)
 * - measure_pos_multi, measure_pairs_multi (batch measurement)
 * - fuzzy_measure_pos_multi, fuzzy_measure_pairs_multi (robust batch)
 *
 * Usage pattern:
 * 1. Create CaliperArray along a path (line, arc, or contour)
 * 2. Measure all handles in batch
 * 3. Extract Point2d arrays for fitting
 * 4. Fit line/circle/ellipse to measured points
 *
 * Precision targets (standard conditions: contrast>=50, noise sigma<=5):
 * - Individual position: < 0.03 px (1 sigma)
 * - Fitted line angle: < 0.005 degrees (1 sigma) with >=10 calipers
 * - Fitted circle center: < 0.02 px (1 sigma) with >=8 calipers
 *
 * @see MeasureHandle.h for individual handle types
 * @see Caliper.h for single handle measurement
 * @see Internal/Fitting.h for geometric fitting functions
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/QContour.h>
#include <QiVision/Core/Types.h>
#include <QiVision/Measure/MeasureTypes.h>
#include <QiVision/Measure/MeasureHandle.h>
#include <QiVision/Measure/Caliper.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace Qi::Vision::Measure {

// =============================================================================
// Forward Declarations
// =============================================================================

class CaliperArray;

// =============================================================================
// Constants
// =============================================================================

/// Default number of calipers in an array
constexpr int32_t DEFAULT_CALIPER_COUNT = 10;

/// Minimum number of calipers
constexpr int32_t MIN_CALIPER_COUNT = 2;

/// Maximum number of calipers
constexpr int32_t MAX_CALIPER_COUNT = 1000;

/// Default caliper spacing (pixels)
constexpr double DEFAULT_CALIPER_SPACING = 10.0;

/// Minimum caliper spacing (pixels)
constexpr double MIN_CALIPER_SPACING = 1.0;

// =============================================================================
// Path Types for Array Generation
// =============================================================================

/**
 * @brief Path type enumeration
 */
enum class PathType {
    Line,           ///< Straight line segment
    Arc,            ///< Circular arc
    Circle,         ///< Full circle
    Contour         ///< XLD contour (arbitrary path)
};

// =============================================================================
// Array Generation Parameters
// =============================================================================

/**
 * @brief Parameters for caliper array generation
 */
struct CaliperArrayParams {
    // Handle geometry
    double profileLength = 50.0;    ///< Profile length per caliper (pixels)
    double handleWidth = 10.0;      ///< Handle width for averaging (pixels)

    // Array distribution
    int32_t caliperCount = DEFAULT_CALIPER_COUNT;   ///< Number of calipers

    // Alternatively, specify by spacing (if caliperCount <= 0)
    double caliperSpacing = DEFAULT_CALIPER_SPACING; ///< Spacing between calipers

    // Profile direction relative to path
    bool profilePerpendicular = true;   ///< true: perpendicular to path

    // Sampling parameters
    int32_t numLines = DEFAULT_NUM_LINES;
    double samplesPerPixel = DEFAULT_SAMPLES_PER_PIXEL;

    // Edge offset from path
    double pathOffset = 0.0;        ///< Offset from path centerline

    // Path coverage
    double startRatio = 0.0;        ///< Start position along path [0, 1]
    double endRatio = 1.0;          ///< End position along path [0, 1]

    // Builder pattern
    CaliperArrayParams& SetProfileLength(double v) { profileLength = v; return *this; }
    CaliperArrayParams& SetHandleWidth(double v) { handleWidth = v; return *this; }
    CaliperArrayParams& SetCaliperCount(int32_t n) { caliperCount = n; return *this; }
    CaliperArrayParams& SetCaliperSpacing(double v) {
        caliperSpacing = v;
        caliperCount = 0;
        return *this;
    }
    CaliperArrayParams& SetProfilePerpendicular(bool v) { profilePerpendicular = v; return *this; }
    CaliperArrayParams& SetNumLines(int32_t n) { numLines = n; return *this; }
    CaliperArrayParams& SetSamplesPerPixel(double v) { samplesPerPixel = v; return *this; }
    CaliperArrayParams& SetPathOffset(double v) { pathOffset = v; return *this; }
    CaliperArrayParams& SetCoverage(double start, double end) {
        startRatio = start;
        endRatio = end;
        return *this;
    }
};

// =============================================================================
// Array Measurement Results
// =============================================================================

/**
 * @brief Result for a single caliper in the array
 */
struct CaliperResult {
    int32_t caliperIndex = -1;      ///< Index in the array

    // Path information
    double pathPosition = 0.0;      ///< Position along path
    double pathRatio = 0.0;         ///< Ratio along path [0, 1]
    Point2d pathPoint;              ///< Point on path
    double pathAngle = 0.0;         ///< Path tangent angle

    // Measurement results
    std::vector<EdgeResult> edges;
    std::vector<PairResult> pairs;

    // Selected result
    bool hasResult = false;
    Point2d resultPoint;
    double resultScore = 0.0;

    // For pair measurement
    double width = 0.0;
    Point2d firstEdge;
    Point2d secondEdge;
};

/**
 * @brief Aggregated results from caliper array measurement
 */
struct CaliperArrayResult {
    // Array information
    int32_t numCalipers = 0;
    int32_t numValid = 0;
    int32_t numInvalid = 0;

    // Per-caliper results
    std::vector<CaliperResult> results;

    // Aggregated edge points
    std::vector<Point2d> allEdgePoints;
    std::vector<Point2d> firstEdgePoints;
    std::vector<Point2d> centerPoints;
    std::vector<Point2d> firstPairEdges;
    std::vector<Point2d> secondPairEdges;

    // Width statistics
    std::vector<double> widths;
    double meanWidth = 0.0;
    double stdWidth = 0.0;
    double minWidth = 0.0;
    double maxWidth = 0.0;

    // Quality metrics
    double validRatio = 0.0;
    double meanScore = 0.0;

    // Validity mask
    std::vector<bool> validMask;

    /// Check if enough results for fitting
    bool CanFitLine() const { return numValid >= 2; }
    bool CanFitCircle() const { return numValid >= 3; }
    bool CanFitEllipse() const { return numValid >= 5; }

    /// Get points for fitting
    std::vector<Point2d> GetPointsForLineFit() const { return firstEdgePoints; }
    std::vector<Point2d> GetPointsForCurveFit() const { return firstEdgePoints; }
};

/**
 * @brief Statistics for array measurement
 */
struct CaliperArrayStats {
    int32_t totalEdgesFound = 0;
    int32_t totalPairsFound = 0;

    double meanAmplitude = 0.0;
    double minAmplitude = 0.0;
    double maxAmplitude = 0.0;

    double measurementTime = 0.0;
    double avgTimePerCaliper = 0.0;
};

// =============================================================================
// CaliperArray Class
// =============================================================================

/**
 * @brief Caliper array for multi-point measurement along a path
 *
 * @code
 * // Create array along a line
 * CaliperArray array;
 * array.CreateAlongLine({100, 100}, {500, 100},
 *     CaliperArrayParams().SetCaliperCount(20).SetProfileLength(30));
 *
 * // Measure edges
 * auto result = array.MeasurePos(image, 1.5, 30.0, "all", "first");
 *
 * // Fit line to measured points
 * if (result.CanFitLine()) {
 *     auto lineResult = FitLine(result.GetPointsForLineFit());
 * }
 * @endcode
 */
class CaliperArray {
public:
    CaliperArray();
    ~CaliperArray();
    CaliperArray(const CaliperArray& other);
    CaliperArray(CaliperArray&& other) noexcept;
    CaliperArray& operator=(const CaliperArray& other);
    CaliperArray& operator=(CaliperArray&& other) noexcept;

    // =========================================================================
    // Array Creation
    // =========================================================================

    /**
     * @brief Create caliper array along a line segment (struct params)
     */
    bool CreateAlongLine(const Point2d& p1, const Point2d& p2,
                         const CaliperArrayParams& params);

    /**
     * @brief Create caliper array along a line segment (direct params - Halcon style)
     *
     * @param p1 Start point
     * @param p2 End point
     * @param caliperCount Number of calipers along the line
     * @param profileLength Profile length per caliper (pixels)
     * @param handleWidth Handle width for averaging (pixels)
     */
    bool CreateAlongLine(const Point2d& p1, const Point2d& p2,
                         int32_t caliperCount,
                         double profileLength = 50.0,
                         double handleWidth = 10.0);

    bool CreateAlongLine(const Segment2d& segment,
                         const CaliperArrayParams& params);

    bool CreateAlongLine(const Segment2d& segment,
                         int32_t caliperCount,
                         double profileLength = 50.0,
                         double handleWidth = 10.0);

    /**
     * @brief Create caliper array along an arc (struct params)
     */
    bool CreateAlongArc(const Point2d& center, double radius,
                        double startAngle, double sweepAngle,
                        const CaliperArrayParams& params);

    /**
     * @brief Create caliper array along an arc (direct params - Halcon style)
     *
     * @param center Arc center
     * @param radius Arc radius
     * @param startAngle Start angle (radians)
     * @param sweepAngle Sweep angle (radians)
     * @param caliperCount Number of calipers
     * @param profileLength Profile length per caliper
     * @param handleWidth Handle width
     */
    bool CreateAlongArc(const Point2d& center, double radius,
                        double startAngle, double sweepAngle,
                        int32_t caliperCount,
                        double profileLength = 50.0,
                        double handleWidth = 10.0);

    bool CreateAlongArc(const Arc2d& arc, const CaliperArrayParams& params);

    bool CreateAlongArc(const Arc2d& arc,
                        int32_t caliperCount,
                        double profileLength = 50.0,
                        double handleWidth = 10.0);

    /**
     * @brief Create caliper array along a full circle (struct params)
     */
    bool CreateAlongCircle(const Point2d& center, double radius,
                           const CaliperArrayParams& params);

    /**
     * @brief Create caliper array along a full circle (direct params - Halcon style)
     *
     * @param center Circle center
     * @param radius Circle radius
     * @param caliperCount Number of calipers
     * @param profileLength Profile length per caliper
     * @param handleWidth Handle width
     */
    bool CreateAlongCircle(const Point2d& center, double radius,
                           int32_t caliperCount,
                           double profileLength = 50.0,
                           double handleWidth = 10.0);

    bool CreateAlongCircle(const Circle2d& circle, const CaliperArrayParams& params);

    bool CreateAlongCircle(const Circle2d& circle,
                           int32_t caliperCount,
                           double profileLength = 50.0,
                           double handleWidth = 10.0);

    /**
     * @brief Create caliper array along a contour (struct params)
     */
    bool CreateAlongContour(const QContour& contour, const CaliperArrayParams& params);

    /**
     * @brief Create caliper array along a contour (direct params - Halcon style)
     *
     * @param contour XLD contour
     * @param caliperCount Number of calipers (0 = auto from spacing)
     * @param profileLength Profile length per caliper
     * @param handleWidth Handle width
     */
    bool CreateAlongContour(const QContour& contour,
                            int32_t caliperCount,
                            double profileLength = 50.0,
                            double handleWidth = 10.0);

    /**
     * @brief Clear the array
     */
    void Clear();

    // =========================================================================
    // Array Properties
    // =========================================================================

    bool IsValid() const;
    int32_t Size() const;
    PathType GetPathType() const;
    double GetPathLength() const;
    const CaliperArrayParams& GetParams() const;
    const MeasureRectangle2& GetHandle(int32_t index) const;
    const std::vector<MeasureRectangle2>& GetHandles() const;
    double GetPathPosition(int32_t index) const;
    double GetPathRatio(int32_t index) const;
    Point2d GetPathPoint(int32_t index) const;

    // =========================================================================
    // Edge Position Measurement (Halcon compatible API)
    // =========================================================================

    /**
     * @brief Measure edge positions in all calipers
     *
     * @param image Input image (grayscale)
     * @param sigma Gaussian smoothing sigma
     * @param threshold Edge amplitude threshold
     * @param transition "positive", "negative", "all"
     * @param select "first", "last", "all"
     * @param stats Optional output statistics
     */
    CaliperArrayResult MeasurePos(const QImage& image,
                                   double sigma,
                                   double threshold,
                                   const std::string& transition,
                                   const std::string& select,
                                   CaliperArrayStats* stats = nullptr) const;

    /**
     * @brief Measure edge positions with fuzzy scoring
     */
    CaliperArrayResult FuzzyMeasurePos(const QImage& image,
                                        double sigma,
                                        double threshold,
                                        const std::string& transition,
                                        const std::string& select,
                                        double fuzzyThresh = 0.5,
                                        CaliperArrayStats* stats = nullptr) const;

    // =========================================================================
    // Edge Pair (Width) Measurement (Halcon compatible API)
    // =========================================================================

    /**
     * @brief Measure edge pairs (widths) in all calipers
     *
     * @param image Input image (grayscale)
     * @param sigma Gaussian smoothing sigma
     * @param threshold Edge amplitude threshold
     * @param transition "positive", "negative", "all"
     * @param select "first", "last", "all"
     * @param stats Optional output statistics
     */
    CaliperArrayResult MeasurePairs(const QImage& image,
                                     double sigma,
                                     double threshold,
                                     const std::string& transition,
                                     const std::string& select,
                                     CaliperArrayStats* stats = nullptr) const;

    /**
     * @brief Measure edge pairs with fuzzy scoring
     */
    CaliperArrayResult FuzzyMeasurePairs(const QImage& image,
                                          double sigma,
                                          double threshold,
                                          const std::string& transition,
                                          const std::string& select,
                                          double fuzzyThresh = 0.5,
                                          CaliperArrayStats* stats = nullptr) const;

    // =========================================================================
    // Convenience Methods
    // =========================================================================

    /**
     * @brief Measure and return first edge points only
     */
    std::vector<Point2d> MeasureFirstEdges(const QImage& image,
                                            double sigma,
                                            double threshold,
                                            const std::string& transition,
                                            const std::string& select) const;

    /**
     * @brief Measure and return pair center points only
     */
    std::vector<Point2d> MeasurePairCenters(const QImage& image,
                                             double sigma,
                                             double threshold,
                                             const std::string& transition,
                                             const std::string& select) const;

    /**
     * @brief Measure widths and return statistics
     */
    std::vector<double> MeasureWidths(const QImage& image,
                                       double sigma,
                                       double threshold,
                                       const std::string& transition,
                                       const std::string& select,
                                       double& meanWidth,
                                       double& stdWidth) const;

    // =========================================================================
    // Visualization
    // =========================================================================

    std::vector<RotatedRect2d> GetHandleRects() const;
    std::vector<Point2d> GetPathPoints(int32_t numPoints = 0) const;
    std::vector<Point2d> GetCaliperCenters() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

// =============================================================================
// Factory Functions
// =============================================================================

CaliperArray CreateCaliperArrayLine(const Point2d& p1, const Point2d& p2,
                                     int32_t caliperCount = DEFAULT_CALIPER_COUNT,
                                     double profileLength = 50.0,
                                     double handleWidth = 10.0);

CaliperArray CreateCaliperArrayLine(const Segment2d& segment,
                                     int32_t caliperCount = DEFAULT_CALIPER_COUNT,
                                     double profileLength = 50.0,
                                     double handleWidth = 10.0);

CaliperArray CreateCaliperArrayArc(const Point2d& center, double radius,
                                    double startAngle, double sweepAngle,
                                    int32_t caliperCount = DEFAULT_CALIPER_COUNT,
                                    double profileLength = 50.0,
                                    double handleWidth = 10.0);

CaliperArray CreateCaliperArrayArc(const Arc2d& arc,
                                    int32_t caliperCount = DEFAULT_CALIPER_COUNT,
                                    double profileLength = 50.0,
                                    double handleWidth = 10.0);

CaliperArray CreateCaliperArrayCircle(const Point2d& center, double radius,
                                       int32_t caliperCount = DEFAULT_CALIPER_COUNT,
                                       double profileLength = 50.0,
                                       double handleWidth = 10.0);

CaliperArray CreateCaliperArrayCircle(const Circle2d& circle,
                                       int32_t caliperCount = DEFAULT_CALIPER_COUNT,
                                       double profileLength = 50.0,
                                       double handleWidth = 10.0);

CaliperArray CreateCaliperArrayContour(const QContour& contour,
                                        int32_t caliperCount = 0,
                                        double profileLength = 50.0,
                                        double handleWidth = 10.0);

// =============================================================================
// Measurement + Fitting Convenience Functions
// =============================================================================

/**
 * @brief Measure edges along a line and fit a line to results
 */
std::optional<Line2d> MeasureAndFitLine(const QImage& image,
                                         const Point2d& p1, const Point2d& p2,
                                         int32_t caliperCount = 10,
                                         double sigma = 1.0,
                                         double threshold = 20.0,
                                         const std::string& transition = "all",
                                         const std::string& select = "first",
                                         std::vector<Point2d>* measuredPoints = nullptr);

/**
 * @brief Measure edges along a circle and fit a circle to results
 */
std::optional<Circle2d> MeasureAndFitCircle(const QImage& image,
                                             const Point2d& approxCenter,
                                             double approxRadius,
                                             int32_t caliperCount = 24,
                                             double sigma = 1.0,
                                             double threshold = 20.0,
                                             const std::string& transition = "all",
                                             const std::string& select = "first",
                                             std::vector<Point2d>* measuredPoints = nullptr);

// =============================================================================
// Width Measurement Functions
// =============================================================================

/**
 * @brief Measure widths along a line and compute statistics
 */
bool MeasureWidthsAlongLine(const QImage& image,
                             const Point2d& p1, const Point2d& p2,
                             int32_t caliperCount,
                             double sigma,
                             double threshold,
                             const std::string& transition,
                             const std::string& select,
                             double& meanWidth,
                             double& stdWidth,
                             std::vector<double>* widths = nullptr);

/**
 * @brief Measure widths along an arc and compute statistics
 */
bool MeasureWidthsAlongArc(const QImage& image,
                            const Arc2d& arc,
                            int32_t caliperCount,
                            double sigma,
                            double threshold,
                            const std::string& transition,
                            const std::string& select,
                            double& meanWidth,
                            double& stdWidth,
                            std::vector<double>* widths = nullptr);

} // namespace Qi::Vision::Measure
