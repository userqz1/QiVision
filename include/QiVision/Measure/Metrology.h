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
    double minScore = 0.5;              ///< Minimum score threshold

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

    /// Get/set measurement parameters
    const MetrologyMeasureParams& Params() const { return params_; }
    void SetParams(const MetrologyMeasureParams& params) { params_ = params; }

    /// Get caliper handles for this object
    virtual std::vector<MeasureRectangle2> GetCalipers() const = 0;

    /// Get the reference geometry contour
    virtual QContour GetContour() const = 0;

    /// Apply transformation (for alignment)
    virtual void Transform(double rowOffset, double colOffset, double phi = 0.0) = 0;

protected:
    int32_t index_ = -1;
    MetrologyMeasureParams params_;

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
     * @param params Measurement parameters
     */
    MetrologyObjectLine(double row1, double col1, double row2, double col2,
                        const MetrologyMeasureParams& params = MetrologyMeasureParams());

    MetrologyObjectType Type() const override { return MetrologyObjectType::Line; }
    std::vector<MeasureRectangle2> GetCalipers() const override;
    QContour GetContour() const override;
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
     * @brief Construct circle measurement object
     * @param row Center row
     * @param column Center column
     * @param radius Radius
     * @param params Measurement parameters
     */
    MetrologyObjectCircle(double row, double column, double radius,
                          const MetrologyMeasureParams& params = MetrologyMeasureParams());

    /**
     * @brief Construct arc measurement object
     * @param row Center row
     * @param column Center column
     * @param radius Radius
     * @param angleStart Start angle (radians)
     * @param angleEnd End angle (radians)
     * @param params Measurement parameters
     */
    MetrologyObjectCircle(double row, double column, double radius,
                          double angleStart, double angleEnd,
                          const MetrologyMeasureParams& params = MetrologyMeasureParams());

    MetrologyObjectType Type() const override { return MetrologyObjectType::Circle; }
    std::vector<MeasureRectangle2> GetCalipers() const override;
    QContour GetContour() const override;
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
     * @param params Measurement parameters
     */
    MetrologyObjectEllipse(double row, double column, double phi,
                            double ra, double rb,
                            const MetrologyMeasureParams& params = MetrologyMeasureParams());

    MetrologyObjectType Type() const override { return MetrologyObjectType::Ellipse; }
    std::vector<MeasureRectangle2> GetCalipers() const override;
    QContour GetContour() const override;
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
     * @param params Measurement parameters
     */
    MetrologyObjectRectangle2(double row, double column, double phi,
                               double length1, double length2,
                               const MetrologyMeasureParams& params = MetrologyMeasureParams());

    MetrologyObjectType Type() const override { return MetrologyObjectType::Rectangle2; }
    std::vector<MeasureRectangle2> GetCalipers() const override;
    QContour GetContour() const override;
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
     * @brief Add a line measurement object (Halcon: add_metrology_object_line_measure)
     * @param row1 Start row
     * @param col1 Start column
     * @param row2 End row
     * @param col2 End column
     * @param params Measurement parameters
     * @return Object index
     */
    int32_t AddLineMeasure(double row1, double col1, double row2, double col2,
                           const MetrologyMeasureParams& params = MetrologyMeasureParams());

    /**
     * @brief Add a circle measurement object (Halcon: add_metrology_object_circle_measure)
     * @param row Center row
     * @param column Center column
     * @param radius Radius
     * @param params Measurement parameters
     * @return Object index
     */
    int32_t AddCircleMeasure(double row, double column, double radius,
                              const MetrologyMeasureParams& params = MetrologyMeasureParams());

    /**
     * @brief Add an arc measurement object
     * @param row Center row
     * @param column Center column
     * @param radius Radius
     * @param angleStart Start angle (radians)
     * @param angleEnd End angle (radians)
     * @param params Measurement parameters
     * @return Object index
     */
    int32_t AddArcMeasure(double row, double column, double radius,
                           double angleStart, double angleEnd,
                           const MetrologyMeasureParams& params = MetrologyMeasureParams());

    /**
     * @brief Add an ellipse measurement object (Halcon: add_metrology_object_ellipse_measure)
     * @param row Center row
     * @param column Center column
     * @param phi Orientation angle (radians)
     * @param ra Semi-major axis
     * @param rb Semi-minor axis
     * @param params Measurement parameters
     * @return Object index
     */
    int32_t AddEllipseMeasure(double row, double column, double phi,
                               double ra, double rb,
                               const MetrologyMeasureParams& params = MetrologyMeasureParams());

    /**
     * @brief Add a rectangle measurement object (Halcon: add_metrology_object_rectangle2_measure)
     * @param row Center row
     * @param column Center column
     * @param phi Orientation angle (radians)
     * @param length1 Half-length along phi
     * @param length2 Half-length perpendicular to phi
     * @param params Measurement parameters
     * @return Object index
     */
    int32_t AddRectangle2Measure(double row, double column, double phi,
                                  double length1, double length2,
                                  const MetrologyMeasureParams& params = MetrologyMeasureParams());

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

    // =========================================================================
    // Model Parameters
    // =========================================================================

    /**
     * @brief Set default measurement parameters for new objects
     */
    void SetDefaultParams(const MetrologyMeasureParams& params);

    /**
     * @brief Get default measurement parameters
     */
    const MetrologyMeasureParams& DefaultParams() const;

    /**
     * @brief Set object-specific parameters
     */
    void SetObjectParams(int32_t index, const MetrologyMeasureParams& params);

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
