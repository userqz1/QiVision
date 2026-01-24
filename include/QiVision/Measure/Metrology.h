#pragma once

/**
 * @file Metrology.h
 * @brief Metrology (geometric measurement) module
 *
 * Provides a unified framework for measuring multiple geometric objects
 * in an image. Similar to Halcon's Metrology module.
 *
 * Features:
 * - Add multiple measurement objects (lines, circles, ellipses, rectangles)
 * - Automatic caliper placement along object contours
 * - Fit geometric primitives from edge measurements
 * - Support for alignment/transformation
 *
 * Halcon equivalents:
 * - create_metrology_model
 * - add_metrology_object_line_measure
 * - add_metrology_object_circle_measure
 * - add_metrology_object_ellipse_measure
 * - add_metrology_object_rectangle2_measure
 * - apply_metrology_model
 * - get_metrology_object_result
 * - align_metrology_model
 */

#include <QiVision/Core/QContour.h>
#include <QiVision/Core/QImage.h>
#include <QiVision/Core/Types.h>
#include <QiVision/Measure/MeasureHandle.h>
#include <QiVision/Measure/MeasureTypes.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace Qi::Vision::Measure {

// =============================================================================
// Metrology Parameter Flags (OpenCV-style vector<int> params)
// =============================================================================

/**
 * @brief Metrology measurement parameter flags
 *
 * @deprecated Use MetrologyMeasureParams struct instead.
 *
 * Legacy usage (deprecated):
 *   model.AddCircleMeasure(200, 200, 50, 20, 5, "all", "all",
 *       {METROLOGY_NUM_MEASURES, 20, METROLOGY_MIN_SCORE, 70});
 *
 * New usage (recommended):
 *   model.AddCircleMeasure(200, 200, 50, 20, 5, "all", "all",
 *       MetrologyMeasureParams().SetNumMeasures(20).SetMinScore(0.7));
 */
enum MetrologyParamFlag {
    METROLOGY_NUM_INSTANCES = 1,      ///< Number of instances to find (int)
    METROLOGY_MEASURE_SIGMA = 2,      ///< Gaussian sigma * 100 (e.g., 150 = 1.5)
    METROLOGY_MEASURE_THRESHOLD = 3,  ///< Edge threshold (int)
    METROLOGY_NUM_MEASURES = 4,       ///< Number of calipers (int)
    METROLOGY_MIN_SCORE = 5,          ///< Min score * 100 (e.g., 70 = 0.7)
    METROLOGY_FIT_METHOD = 6,         ///< 0=RANSAC, 1=Huber, 2=Tukey
    METROLOGY_DISTANCE_THRESHOLD = 7, ///< Outlier threshold * 100 (e.g., 350 = 3.5)
    METROLOGY_MAX_ITERATIONS = 8,     ///< Max RANSAC iterations (-1 = unlimited)
    METROLOGY_RAND_SEED = 9,          ///< Random seed for RANSAC
    METROLOGY_THRESHOLD_MODE = 10     ///< 0=Manual, 1=Auto
};

// =============================================================================
// Metrology Types
// =============================================================================

/**
 * @brief Type of metrology object
 */
enum class MetrologyObjectType {
    Line,           ///< Straight line
    Circle,         ///< Circle (full or partial)
    Ellipse,        ///< Ellipse
    Rectangle2      ///< Rotated rectangle
};

/**
 * @brief Threshold mode for edge detection
 */
enum class ThresholdMode {
    Manual,         ///< Use user-specified threshold value
    Auto            ///< Automatically compute threshold per profile region
};

/**
 * @brief Fitting method for geometric primitives
 *
 * Halcon uses RANSAC by default with distance_threshold=3.5
 */
enum class MetrologyFitMethod {
    RANSAC,         ///< RANSAC (Halcon compatible) - outliers completely excluded
    Huber,          ///< Huber M-estimator (IRLS) - outliers soft-weighted
    Tukey           ///< Tukey biweight (IRLS) - outliers strongly suppressed
};

/**
 * @brief Parameters for metrology measurement
 */
struct MetrologyMeasureParams {
    int32_t numInstances = 1;           ///< Number of instances to find
    double measureLength1 = 20.0;       ///< Half-length of caliper along profile
    double measureLength2 = 5.0;        ///< Half-width of caliper perpendicular
    double measureSigma = 1.0;          ///< Gaussian smoothing sigma
    double measureThreshold = 30.0;     ///< Edge amplitude threshold (used when thresholdMode is Manual)
    ThresholdMode thresholdMode = ThresholdMode::Manual;  ///< Threshold mode
    EdgeTransition measureTransition = EdgeTransition::All;  ///< Edge polarity
    EdgeSelectMode measureSelect = EdgeSelectMode::All;      ///< Edge selection
    int32_t numMeasures = 10;           ///< Number of calipers per object
    double minScore = 0.7;              ///< Minimum score threshold (Halcon default: 0.7)

    // Fitting parameters (Halcon compatible)
    MetrologyFitMethod fitMethod = MetrologyFitMethod::RANSAC;  ///< Fitting method (default: RANSAC for Halcon compatibility)
    double distanceThreshold = 3.5;     ///< Outlier distance threshold in pixels (Halcon default: 3.5)
    int32_t maxIterations = -1;         ///< Max RANSAC iterations (-1 = unlimited, Halcon default)
    int32_t randSeed = 42;              ///< Random seed for RANSAC (Halcon default: 42)

    // Builder pattern
    MetrologyMeasureParams& SetNumInstances(int32_t n) { numInstances = n; return *this; }
    MetrologyMeasureParams& SetMeasureLength(double l1, double l2) {
        measureLength1 = l1; measureLength2 = l2; return *this;
    }
    MetrologyMeasureParams& SetMeasureSigma(double s) { measureSigma = s; return *this; }

    /// Set threshold with numeric value (Manual mode)
    MetrologyMeasureParams& SetThreshold(double t) {
        measureThreshold = t;
        thresholdMode = ThresholdMode::Manual;
        return *this;
    }

    /// Set threshold with string ("auto" for automatic mode)
    MetrologyMeasureParams& SetThreshold(const std::string& mode) {
        if (mode == "auto" || mode == "Auto" || mode == "AUTO") {
            thresholdMode = ThresholdMode::Auto;
        }
        return *this;
    }

    /// @deprecated Use SetThreshold instead
    MetrologyMeasureParams& SetMeasureThreshold(double t) { return SetThreshold(t); }

    MetrologyMeasureParams& SetMeasureTransition(EdgeTransition t) { measureTransition = t; return *this; }
    MetrologyMeasureParams& SetMeasureSelect(EdgeSelectMode m) { measureSelect = m; return *this; }
    MetrologyMeasureParams& SetNumMeasures(int32_t n) { numMeasures = n; return *this; }
    MetrologyMeasureParams& SetMinScore(double s) { minScore = s; return *this; }

    /// Set fitting method
    MetrologyMeasureParams& SetFitMethod(MetrologyFitMethod m) { fitMethod = m; return *this; }

    /// Set fitting method with string ("ransac", "huber", "tukey")
    MetrologyMeasureParams& SetFitMethod(const std::string& method) {
        if (method == "ransac" || method == "RANSAC") {
            fitMethod = MetrologyFitMethod::RANSAC;
        } else if (method == "huber" || method == "Huber") {
            fitMethod = MetrologyFitMethod::Huber;
        } else if (method == "tukey" || method == "Tukey") {
            fitMethod = MetrologyFitMethod::Tukey;
        }
        return *this;
    }

    /// Set outlier distance threshold (pixels)
    MetrologyMeasureParams& SetDistanceThreshold(double t) { distanceThreshold = t; return *this; }

    /// Set max RANSAC iterations (-1 for unlimited)
    MetrologyMeasureParams& SetMaxIterations(int32_t n) { maxIterations = n; return *this; }

    /// Set random seed for RANSAC
    MetrologyMeasureParams& SetRandSeed(int32_t s) { randSeed = s; return *this; }
};

/**
 * @brief Result for a line measurement
 */
struct MetrologyLineResult {
    // Fitted line parameters (Hesse normal form: row*nr + col*nc = d)
    double row1 = 0.0;          ///< Start point row
    double col1 = 0.0;          ///< Start point column
    double row2 = 0.0;          ///< End point row
    double col2 = 0.0;          ///< End point column

    // Line equation
    double nr = 0.0;            ///< Normal row component
    double nc = 0.0;            ///< Normal column component
    double dist = 0.0;          ///< Distance from origin

    // Quality metrics
    double score = 0.0;         ///< Fit quality [0, 1]
    int32_t numUsed = 0;        ///< Number of edges used in fit
    double rmsError = 0.0;      ///< RMS fitting error

    bool IsValid() const { return numUsed >= 2 && score > 0; }
};

/**
 * @brief Result for a circle measurement
 */
struct MetrologyCircleResult {
    double row = 0.0;           ///< Center row
    double column = 0.0;        ///< Center column
    double radius = 0.0;        ///< Radius

    // Arc parameters (for partial circles)
    double startAngle = 0.0;    ///< Start angle (radians)
    double endAngle = 0.0;      ///< End angle (radians)

    // Quality metrics
    double score = 0.0;         ///< Fit quality [0, 1]
    int32_t numUsed = 0;        ///< Number of edges used in fit
    double rmsError = 0.0;      ///< RMS fitting error

    bool IsValid() const { return numUsed >= 3 && radius > 0 && score > 0; }
    Point2d Center() const { return {column, row}; }
};

/**
 * @brief Result for an ellipse measurement
 */
struct MetrologyEllipseResult {
    double row = 0.0;           ///< Center row
    double column = 0.0;        ///< Center column
    double phi = 0.0;           ///< Orientation angle (radians)
    double ra = 0.0;            ///< Semi-major axis
    double rb = 0.0;            ///< Semi-minor axis

    // Quality metrics
    double score = 0.0;
    int32_t numUsed = 0;
    double rmsError = 0.0;

    bool IsValid() const { return numUsed >= 5 && ra > 0 && rb > 0 && score > 0; }
};

/**
 * @brief Result for a rectangle measurement
 */
struct MetrologyRectangle2Result {
    double row = 0.0;           ///< Center row
    double column = 0.0;        ///< Center column
    double phi = 0.0;           ///< Orientation angle (radians)
    double length1 = 0.0;       ///< Half-length along phi
    double length2 = 0.0;       ///< Half-length perpendicular to phi

    // Quality metrics
    double score = 0.0;
    int32_t numUsed = 0;
    double rmsError = 0.0;

    bool IsValid() const { return numUsed >= 4 && length1 > 0 && length2 > 0 && score > 0; }
};

// =============================================================================
// Forward Declaration
// =============================================================================

class MetrologyModel;
class MetrologyObject;

// =============================================================================
// MetrologyObject Base Class
// =============================================================================

/**
 * @brief Base class for metrology measurement objects
 */
class MetrologyObject {
public:
    virtual ~MetrologyObject() = default;

    /// Get object type
    virtual MetrologyObjectType Type() const = 0;

    /// Get object index
    int32_t Index() const { return index_; }

    // =========================================================================
    // Parameter Getters
    // =========================================================================

    /// Get half-length of caliper along profile direction
    double GetMeasureLength1() const { return params_.measureLength1; }

    /// Get half-width of caliper perpendicular to profile direction
    double GetMeasureLength2() const { return params_.measureLength2; }

    /// Get number of calipers
    int32_t GetNumMeasures() const { return params_.numMeasures; }

    /// Get Gaussian smoothing sigma
    double GetMeasureSigma() const { return params_.measureSigma; }

    /// Get edge amplitude threshold
    double GetMeasureThreshold() const { return params_.measureThreshold; }

    /// Get threshold mode
    ThresholdMode GetThresholdMode() const { return params_.thresholdMode; }

    /// Get edge transition filter
    EdgeTransition GetMeasureTransition() const { return params_.measureTransition; }

    /// Get edge selection mode
    EdgeSelectMode GetMeasureSelect() const { return params_.measureSelect; }

    /// Get minimum score threshold
    double GetMinScore() const { return params_.minScore; }

    /// Get fitting method
    MetrologyFitMethod GetFitMethod() const { return params_.fitMethod; }

    /// Get outlier distance threshold
    double GetDistanceThreshold() const { return params_.distanceThreshold; }

    /// Get max RANSAC iterations
    int32_t GetMaxIterations() const { return params_.maxIterations; }

    /// Get random seed for RANSAC
    int32_t GetRandSeed() const { return params_.randSeed; }

    // =========================================================================
    // Parameter Setters
    // =========================================================================

    /// Set caliper dimensions
    void SetMeasureLength(double length1, double length2) {
        params_.measureLength1 = length1;
        params_.measureLength2 = length2;
    }

    /// Set number of calipers
    void SetNumMeasures(int32_t numMeasures) { params_.numMeasures = numMeasures; }

    /// Set Gaussian smoothing sigma
    void SetMeasureSigma(double sigma) { params_.measureSigma = sigma; }

    /// Set edge amplitude threshold (manual mode)
    void SetMeasureThreshold(double threshold) {
        params_.measureThreshold = threshold;
        params_.thresholdMode = ThresholdMode::Manual;
    }

    /// Set threshold mode ("manual" or "auto")
    void SetThresholdMode(const std::string& mode) {
        if (mode == "auto" || mode == "Auto" || mode == "AUTO") {
            params_.thresholdMode = ThresholdMode::Auto;
        } else {
            params_.thresholdMode = ThresholdMode::Manual;
        }
    }

    /// Set edge transition filter ("positive", "negative", "all")
    void SetMeasureTransition(const std::string& transition) {
        if (transition == "positive" || transition == "Positive") {
            params_.measureTransition = EdgeTransition::Positive;
        } else if (transition == "negative" || transition == "Negative") {
            params_.measureTransition = EdgeTransition::Negative;
        } else {
            params_.measureTransition = EdgeTransition::All;
        }
    }

    /// Set edge selection mode ("first", "last", "best", "all")
    void SetMeasureSelect(const std::string& select) {
        if (select == "first" || select == "First") {
            params_.measureSelect = EdgeSelectMode::First;
        } else if (select == "last" || select == "Last") {
            params_.measureSelect = EdgeSelectMode::Last;
        } else if (select == "best" || select == "Best" || select == "strongest" || select == "Strongest") {
            params_.measureSelect = EdgeSelectMode::Strongest;
        } else {
            params_.measureSelect = EdgeSelectMode::All;
        }
    }

    /// Set minimum score threshold
    void SetMinScore(double minScore) { params_.minScore = minScore; }

    /// Set fitting method ("ransac", "huber", "tukey")
    void SetFitMethod(const std::string& method) {
        if (method == "ransac" || method == "RANSAC") {
            params_.fitMethod = MetrologyFitMethod::RANSAC;
        } else if (method == "huber" || method == "Huber") {
            params_.fitMethod = MetrologyFitMethod::Huber;
        } else if (method == "tukey" || method == "Tukey") {
            params_.fitMethod = MetrologyFitMethod::Tukey;
        }
    }

    /// Set outlier distance threshold (pixels)
    void SetDistanceThreshold(double threshold) { params_.distanceThreshold = threshold; }

    /// Set max RANSAC iterations (-1 for unlimited)
    void SetMaxIterations(int32_t maxIterations) { params_.maxIterations = maxIterations; }

    /// Set random seed for RANSAC
    void SetRandSeed(int32_t seed) { params_.randSeed = seed; }

    // =========================================================================
    // Virtual Methods
    // =========================================================================

    /// Get caliper handles for this object
    virtual std::vector<MeasureRectangle2> GetCalipers() const = 0;

    /// Get the reference geometry contour
    virtual QContour GetContour() const = 0;

    /// Check if this object has a geometric center (circles, ellipses, rectangles have center; lines don't)
    virtual bool HasCenter() const = 0;

    /// Get geometric center (only valid when HasCenter() returns true)
    virtual Point2d GetCenter() const = 0;

    /// Apply transformation (for alignment)
    virtual void Transform(double rowOffset, double colOffset, double phi = 0.0) = 0;

protected:
    int32_t index_ = -1;
    MetrologyMeasureParams params_;

    /// Internal access to params (for derived classes and MetrologyModel)
    const MetrologyMeasureParams& Params() const { return params_; }

    friend class MetrologyModel;
};

// =============================================================================
// Metrology Object Implementations
// =============================================================================

/**
 * @brief Line measurement object
 */
class MetrologyObjectLine : public MetrologyObject {
public:
    /**
     * @brief Construct line measurement object
     * @param row1 Start row
     * @param col1 Start column
     * @param row2 End row
     * @param col2 End column
     * @param measureLength1 Half-length of caliper along profile direction (default: 20.0)
     * @param measureLength2 Half-width of caliper perpendicular to profile (default: 5.0)
     * @param numMeasures Number of calipers along the line (default: 10)
     */
    MetrologyObjectLine(double row1, double col1, double row2, double col2,
                        double measureLength1 = 20.0, double measureLength2 = 5.0,
                        int32_t numMeasures = 10);

    MetrologyObjectType Type() const override { return MetrologyObjectType::Line; }
    std::vector<MeasureRectangle2> GetCalipers() const override;
    QContour GetContour() const override;
    bool HasCenter() const override { return false; }
    Point2d GetCenter() const override { return {0, 0}; }
    void Transform(double rowOffset, double colOffset, double phi = 0.0) override;

    // Line-specific accessors
    double Row1() const { return row1_; }
    double Col1() const { return col1_; }
    double Row2() const { return row2_; }
    double Col2() const { return col2_; }
    double Length() const;
    double Angle() const;

private:
    double row1_, col1_, row2_, col2_;
};

/**
 * @brief Circle measurement object
 */
class MetrologyObjectCircle : public MetrologyObject {
public:
    /**
     * @brief Construct circle measurement object (full circle)
     * @param row Center row
     * @param column Center column
     * @param radius Radius
     * @param measureLength1 Half-length of caliper along profile direction (default: 20.0)
     * @param measureLength2 Half-width of caliper perpendicular to profile (default: 5.0)
     * @param numMeasures Number of calipers around the circle (default: 20)
     */
    MetrologyObjectCircle(double row, double column, double radius,
                          double measureLength1 = 20.0, double measureLength2 = 5.0,
                          int32_t numMeasures = 20);

    /**
     * @brief Construct arc measurement object (partial circle)
     * @param row Center row
     * @param column Center column
     * @param radius Radius
     * @param angleStart Start angle (radians)
     * @param angleEnd End angle (radians)
     * @param measureLength1 Half-length of caliper along profile direction (default: 20.0)
     * @param measureLength2 Half-width of caliper perpendicular to profile (default: 5.0)
     * @param numMeasures Number of calipers along the arc (default: 20)
     */
    MetrologyObjectCircle(double row, double column, double radius,
                          double angleStart, double angleEnd,
                          double measureLength1 = 20.0, double measureLength2 = 5.0,
                          int32_t numMeasures = 20);

    MetrologyObjectType Type() const override { return MetrologyObjectType::Circle; }
    std::vector<MeasureRectangle2> GetCalipers() const override;
    QContour GetContour() const override;
    bool HasCenter() const override { return true; }
    Point2d GetCenter() const override { return {column_, row_}; }
    void Transform(double rowOffset, double colOffset, double phi = 0.0) override;

    // Circle-specific accessors
    double Row() const { return row_; }
    double Column() const { return column_; }
    double Radius() const { return radius_; }
    double AngleStart() const { return angleStart_; }
    double AngleEnd() const { return angleEnd_; }
    bool IsFullCircle() const;

private:
    double row_, column_, radius_;
    double angleStart_ = 0.0;
    double angleEnd_ = 6.283185307;  // 2*PI
};

/**
 * @brief Ellipse measurement object
 */
class MetrologyObjectEllipse : public MetrologyObject {
public:
    /**
     * @brief Construct ellipse measurement object
     * @param row Center row
     * @param column Center column
     * @param phi Orientation angle (radians)
     * @param ra Semi-major axis
     * @param rb Semi-minor axis
     * @param measureLength1 Half-length of caliper along profile direction (default: 20.0)
     * @param measureLength2 Half-width of caliper perpendicular to profile (default: 5.0)
     * @param numMeasures Number of calipers around the ellipse (default: 20)
     */
    MetrologyObjectEllipse(double row, double column, double phi,
                            double ra, double rb,
                            double measureLength1 = 20.0, double measureLength2 = 5.0,
                            int32_t numMeasures = 20);

    MetrologyObjectType Type() const override { return MetrologyObjectType::Ellipse; }
    std::vector<MeasureRectangle2> GetCalipers() const override;
    QContour GetContour() const override;
    bool HasCenter() const override { return true; }
    Point2d GetCenter() const override { return {column_, row_}; }
    void Transform(double rowOffset, double colOffset, double phi = 0.0) override;

    // Ellipse-specific accessors
    double Row() const { return row_; }
    double Column() const { return column_; }
    double Phi() const { return phi_; }
    double Ra() const { return ra_; }
    double Rb() const { return rb_; }

private:
    double row_, column_, phi_, ra_, rb_;
};

/**
 * @brief Rectangle measurement object
 */
class MetrologyObjectRectangle2 : public MetrologyObject {
public:
    /**
     * @brief Construct rectangle measurement object
     * @param row Center row
     * @param column Center column
     * @param phi Orientation angle (radians)
     * @param length1 Half-length along phi
     * @param length2 Half-length perpendicular to phi
     * @param measureLength1 Half-length of caliper along profile direction (default: 20.0)
     * @param measureLength2 Half-width of caliper perpendicular to profile (default: 5.0)
     * @param numMeasuresPerSide Number of calipers per side (default: 5, total = 4 * numMeasuresPerSide)
     */
    MetrologyObjectRectangle2(double row, double column, double phi,
                               double length1, double length2,
                               double measureLength1 = 20.0, double measureLength2 = 5.0,
                               int32_t numMeasuresPerSide = 5);

    MetrologyObjectType Type() const override { return MetrologyObjectType::Rectangle2; }
    std::vector<MeasureRectangle2> GetCalipers() const override;
    QContour GetContour() const override;
    bool HasCenter() const override { return true; }
    Point2d GetCenter() const override { return {column_, row_}; }
    void Transform(double rowOffset, double colOffset, double phi = 0.0) override;

    // Rectangle-specific accessors
    double Row() const { return row_; }
    double Column() const { return column_; }
    double Phi() const { return phi_; }
    double Length1() const { return length1_; }
    double Length2() const { return length2_; }

private:
    double row_, column_, phi_, length1_, length2_;
};

// =============================================================================
// MetrologyModel
// =============================================================================

/**
 * @brief Metrology model for geometric measurement
 *
 * A MetrologyModel manages multiple measurement objects and performs
 * coordinated measurement across all objects.
 *
 * Usage:
 * @code
 * // Create model
 * MetrologyModel model;
 *
 * // Add objects
 * int lineIdx = model.AddLineMeasure(100, 100, 200, 100);
 * int circleIdx = model.AddCircleMeasure(150, 150, 50);
 *
 * // Apply measurement
 * model.Apply(image);
 *
 * // Get results
 * auto lineResult = model.GetLineResult(lineIdx);
 * auto circleResult = model.GetCircleResult(circleIdx);
 * @endcode
 */
class MetrologyModel {
public:
    MetrologyModel();
    ~MetrologyModel();

    // Non-copyable, movable
    MetrologyModel(const MetrologyModel&) = delete;
    MetrologyModel& operator=(const MetrologyModel&) = delete;
    MetrologyModel(MetrologyModel&&) noexcept;
    MetrologyModel& operator=(MetrologyModel&&) noexcept;

    // =========================================================================
    // Object Management
    // =========================================================================

    /**
     * @brief Add a line measurement object
     *
     * @param row1 Start row
     * @param col1 Start column
     * @param row2 End row
     * @param col2 End column
     * @param measureLength1 Half-length of caliper along profile
     * @param measureLength2 Half-width of caliper perpendicular
     * @param transition "positive", "negative", "all"
     * @param select "first", "last", "all"
     * @param params Measurement parameters (numMeasures, sigma, threshold, fitting method, etc.)
     * @return Object index
     */
    int32_t AddLineMeasure(double row1, double col1, double row2, double col2,
                           double measureLength1, double measureLength2,
                           const std::string& transition = "all",
                           const std::string& select = "all",
                           const MetrologyMeasureParams& params = {});

    /**
     * @brief Add a circle measurement object
     *
     * @param row Center row
     * @param column Center column
     * @param radius Radius
     * @param measureLength1 Half-length of caliper along profile
     * @param measureLength2 Half-width of caliper perpendicular
     * @param transition "positive", "negative", "all"
     * @param select "first", "last", "all"
     * @param params Measurement parameters (numMeasures, sigma, threshold, fitting method, etc.)
     * @return Object index
     */
    int32_t AddCircleMeasure(double row, double column, double radius,
                              double measureLength1, double measureLength2,
                              const std::string& transition = "all",
                              const std::string& select = "all",
                              const MetrologyMeasureParams& params = {});

    /**
     * @brief Add an arc measurement object
     *
     * @param row Center row
     * @param column Center column
     * @param radius Radius
     * @param angleStart Start angle (radians)
     * @param angleEnd End angle (radians)
     * @param measureLength1 Half-length of caliper along profile
     * @param measureLength2 Half-width of caliper perpendicular
     * @param transition "positive", "negative", "all"
     * @param select "first", "last", "all"
     * @param params Measurement parameters (numMeasures, sigma, threshold, fitting method, etc.)
     * @return Object index
     */
    int32_t AddArcMeasure(double row, double column, double radius,
                           double angleStart, double angleEnd,
                           double measureLength1, double measureLength2,
                           const std::string& transition = "all",
                           const std::string& select = "all",
                           const MetrologyMeasureParams& params = {});

    /**
     * @brief Add an ellipse measurement object
     *
     * @param row Center row
     * @param column Center column
     * @param phi Orientation angle (radians)
     * @param ra Semi-major axis
     * @param rb Semi-minor axis
     * @param measureLength1 Half-length of caliper along profile
     * @param measureLength2 Half-width of caliper perpendicular
     * @param transition "positive", "negative", "all"
     * @param select "first", "last", "all"
     * @param params Measurement parameters (numMeasures, sigma, threshold, fitting method, etc.)
     * @return Object index
     */
    int32_t AddEllipseMeasure(double row, double column, double phi,
                               double ra, double rb,
                               double measureLength1, double measureLength2,
                               const std::string& transition = "all",
                               const std::string& select = "all",
                               const MetrologyMeasureParams& params = {});

    /**
     * @brief Add a rectangle measurement object
     *
     * @param row Center row
     * @param column Center column
     * @param phi Orientation angle (radians)
     * @param length1 Half-length along phi
     * @param length2 Half-length perpendicular to phi
     * @param measureLength1 Half-length of caliper along profile
     * @param measureLength2 Half-width of caliper perpendicular
     * @param transition "positive", "negative", "all"
     * @param select "first", "last", "all"
     * @param params Measurement parameters (numMeasures, sigma, threshold, fitting method, etc.)
     * @return Object index
     */
    int32_t AddRectangle2Measure(double row, double column, double phi,
                                  double length1, double length2,
                                  double measureLength1, double measureLength2,
                                  const std::string& transition = "all",
                                  const std::string& select = "all",
                                  const MetrologyMeasureParams& params = {});

    /**
     * @brief Clear a specific object
     * @param index Object index
     */
    void ClearObject(int32_t index);

    /**
     * @brief Clear all objects
     */
    void ClearAll();

    /**
     * @brief Get number of objects
     */
    int32_t NumObjects() const;

    /**
     * @brief Get object by index
     */
    const MetrologyObject* GetObject(int32_t index) const;

    // =========================================================================
    // Measurement
    // =========================================================================

    /**
     * @brief Apply measurement to image (Halcon: apply_metrology_model)
     * @param image Input image
     * @return true if measurement succeeded
     */
    bool Apply(const QImage& image);

    /**
     * @brief Get line measurement result (Halcon: get_metrology_object_result)
     * @param index Object index
     * @param instanceIndex Instance index (for multiple instances)
     * @return Measurement result
     */
    MetrologyLineResult GetLineResult(int32_t index, int32_t instanceIndex = 0) const;

    /**
     * @brief Get circle measurement result
     */
    MetrologyCircleResult GetCircleResult(int32_t index, int32_t instanceIndex = 0) const;

    /**
     * @brief Get ellipse measurement result
     */
    MetrologyEllipseResult GetEllipseResult(int32_t index, int32_t instanceIndex = 0) const;

    /**
     * @brief Get rectangle measurement result
     */
    MetrologyRectangle2Result GetRectangle2Result(int32_t index, int32_t instanceIndex = 0) const;

    /**
     * @brief Get result contour for visualization
     * @param index Object index
     * @param instanceIndex Instance index
     * @return Fitted contour
     */
    QContour GetResultContour(int32_t index, int32_t instanceIndex = 0) const;

    /**
     * @brief Get measured edge points
     * @param index Object index
     * @return Vector of measured edge positions
     */
    std::vector<Point2d> GetMeasuredPoints(int32_t index) const;

    /**
     * @brief Get point weights from robust fitting (Huber/Tukey)
     * @param index Object index
     * @return Vector of weights [0, 1] for each measured point
     *         Weight < 1 indicates potential outlier (lower weight = more outlier-like)
     */
    std::vector<double> GetPointWeights(int32_t index) const;

    // =========================================================================
    // Alignment
    // =========================================================================

    /**
     * @brief Align model to new position (Halcon: align_metrology_model)
     * @param row Row offset
     * @param column Column offset
     * @param phi Rotation angle (radians)
     */
    void Align(double row, double column, double phi = 0.0);

    /**
     * @brief Reset to original position
     */
    void ResetAlignment();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// =============================================================================
// Factory Functions (Halcon compatible naming)
// =============================================================================

/**
 * @brief Create a metrology model (Halcon: create_metrology_model)
 */
inline MetrologyModel CreateMetrologyModel() {
    return MetrologyModel();
}

/**
 * @brief Clear a metrology model (Halcon: clear_metrology_model)
 */
inline void ClearMetrologyModel(MetrologyModel& model) {
    model.ClearAll();
}

} // namespace Qi::Vision::Measure
