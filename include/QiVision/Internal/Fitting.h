#pragma once

/**
 * @file Fitting.h
 * @brief Geometric fitting algorithms for QiVision
 *
 * This module provides:
 * - Line fitting: least squares, weighted, robust (Huber/Tukey), RANSAC
 * - Circle fitting: algebraic (Kasa), geometric (Levenberg-Marquardt), RANSAC
 * - Ellipse fitting: Fitzgibbon (direct), geometric, RANSAC
 * - Generic RANSAC framework
 * - Robust weight functions and scale estimators
 * - Residual computation and statistics
 *
 * Used by:
 * - Measure module (caliper, metrology)
 * - Edge module (contour fitting)
 * - Calib module (calibration)
 *
 * Precision targets (standard conditions: contrast>=50, noise sigma<=5):
 * - Line angle: < 0.005 degrees (1 sigma)
 * - Circle center/radius: < 0.02 px (1 sigma)
 *
 * Design principles:
 * - All functions are pure (no global state)
 * - Results include residual statistics for quality assessment
 * - RANSAC is templated for extensibility
 * - Robust estimators use breakdown point analysis
 */

#include <QiVision/Core/Types.h>
#include <QiVision/Internal/Matrix.h>
#include <QiVision/Internal/Solver.h>

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

namespace Qi::Vision::Internal {

// =============================================================================
// Constants
// =============================================================================

/// Maximum RANSAC iterations
constexpr int RANSAC_MAX_ITERATIONS = 1000;

/// Default RANSAC inlier threshold (pixels)
constexpr double RANSAC_DEFAULT_THRESHOLD = 1.0;

/// Default RANSAC confidence level
constexpr double RANSAC_DEFAULT_CONFIDENCE = 0.99;

/// Minimum points required for line fitting
constexpr int LINE_FIT_MIN_POINTS = 2;

/// Minimum points required for circle fitting
constexpr int CIRCLE_FIT_MIN_POINTS = 3;

/// Minimum points required for ellipse fitting
constexpr int ELLIPSE_FIT_MIN_POINTS = 5;

/// Convergence tolerance for geometric (iterative) fitting
constexpr double GEOMETRIC_FIT_TOLERANCE = 1e-8;

/// Maximum iterations for geometric fitting
constexpr int GEOMETRIC_FIT_MAX_ITERATIONS = 100;

/// Huber robust estimator tuning constant (95% efficiency for Gaussian)
constexpr double HUBER_K = 1.345;

/// Tukey biweight tuning constant (95% efficiency for Gaussian)
constexpr double TUKEY_C = 4.685;

/// Tolerance for collinearity check
constexpr double COLLINEAR_TOLERANCE = 1e-10;

// =============================================================================
// Enumerations
// =============================================================================

/**
 * @brief General fitting method enumeration
 */
enum class FitMethod {
    LeastSquares,   ///< Standard L2 least squares
    Weighted,       ///< Weighted least squares
    Huber,          ///< Robust M-estimator (Huber weight)
    Tukey,          ///< Robust M-estimator (Tukey biweight)
    Geometric,      ///< Geometric (orthogonal distance) fitting
    GeoHuber,       ///< Geometric with Huber weighting
    GeoTukey,       ///< Geometric with Tukey weighting
    RANSAC,         ///< Random Sample Consensus
    LMedS           ///< Least Median of Squares
};

/**
 * @brief Circle fitting method enumeration
 */
enum class CircleFitMethod {
    Algebraic,      ///< Kasa algebraic least squares (fast, biased for small arcs)
    AlgebraicHuber, ///< Algebraic with Huber weighting
    AlgebraicTukey, ///< Algebraic with Tukey weighting
    Geometric,      ///< Geometric (orthogonal distance) via Levenberg-Marquardt
    GeoHuber,       ///< Geometric with Huber weighting
    GeoTukey,       ///< Geometric with Tukey weighting
    RANSAC          ///< RANSAC-based robust fitting
};

/**
 * @brief Ellipse fitting method enumeration
 */
enum class EllipseFitMethod {
    Fitzgibbon,     ///< Direct least squares (Fitzgibbon 1999)
    FocalPoints,    ///< Focal point parameterization
    Taubin,         ///< Taubin method (better noise handling)
    Geometric,      ///< Geometric fitting via iteration
    RANSAC          ///< RANSAC-based robust fitting
};

// =============================================================================
// Result Structures
// =============================================================================

/**
 * @brief Base class for all fitting results
 */
struct FitResultBase {
    bool success = false;           ///< Whether fitting succeeded

    int numPoints = 0;              ///< Total number of input points
    int numInliers = 0;             ///< Number of inliers (for RANSAC)

    double residualMean = 0.0;      ///< Mean of absolute residuals
    double residualStd = 0.0;       ///< Standard deviation of residuals
    double residualMax = 0.0;       ///< Maximum absolute residual
    double residualRMS = 0.0;       ///< Root mean square of residuals

    std::vector<double> residuals;  ///< Per-point residuals (optional)
    std::vector<bool> inlierMask;   ///< Inlier mask (optional, for RANSAC)
    std::vector<double> weights;    ///< Per-point weights (optional, for Huber/Tukey)
};

/**
 * @brief Line fitting result
 */
struct LineFitResult : public FitResultBase {
    Line2d line;                    ///< Fitted line (ax + by + c = 0, normalized)

    /// Get line angle in radians
    double Angle() const { return line.Angle(); }

    /// Get line angle in degrees
    double AngleDegrees() const { return line.Angle() * 180.0 / M_PI; }
};

/**
 * @brief Circle fitting result
 */
struct CircleFitResult : public FitResultBase {
    Circle2d circle;                ///< Fitted circle (center, radius)

    /// Get center
    Point2d Center() const { return circle.center; }

    /// Get radius
    double Radius() const { return circle.radius; }
};

/**
 * @brief Ellipse fitting result
 */
struct EllipseFitResult : public FitResultBase {
    Ellipse2d ellipse;              ///< Fitted ellipse (center, axes, angle)

    /// Get center
    Point2d Center() const { return ellipse.center; }

    /// Get semi-major axis
    double SemiMajor() const { return ellipse.a; }

    /// Get semi-minor axis
    double SemiMinor() const { return ellipse.b; }

    /// Get rotation angle in radians
    double Angle() const { return ellipse.angle; }
};

// =============================================================================
// Parameter Structures
// =============================================================================

/**
 * @brief Common fitting parameters
 */
struct FitParams {
    bool computeResiduals = false;      ///< Whether to compute per-point residuals
    bool computeInlierMask = false;     ///< Whether to compute inlier mask
    double outlierThreshold = 3.0;      ///< Outlier threshold (in sigma units)

    FitParams() = default;

    FitParams& SetComputeResiduals(bool value) {
        computeResiduals = value;
        return *this;
    }

    FitParams& SetComputeInlierMask(bool value) {
        computeInlierMask = value;
        return *this;
    }

    FitParams& SetOutlierThreshold(double value) {
        outlierThreshold = value;
        return *this;
    }
};

/**
 * @brief RANSAC parameters
 */
struct RansacParams {
    double threshold = RANSAC_DEFAULT_THRESHOLD;    ///< Inlier threshold (pixels)
    double confidence = RANSAC_DEFAULT_CONFIDENCE;  ///< Confidence level [0, 1)
    int maxIterations = RANSAC_MAX_ITERATIONS;      ///< Maximum iterations
    int minInliers = 0;                             ///< Minimum required inliers (0 = auto)

    RansacParams() = default;

    RansacParams& SetThreshold(double value) {
        threshold = value;
        return *this;
    }

    RansacParams& SetConfidence(double value) {
        confidence = value;
        return *this;
    }

    RansacParams& SetMaxIterations(int value) {
        maxIterations = value;
        return *this;
    }

    RansacParams& SetMinInliers(int value) {
        minInliers = value;
        return *this;
    }

    /**
     * @brief Compute adaptive iteration count based on inlier ratio
     * @param inlierRatio Estimated ratio of inliers [0, 1]
     * @param sampleSize Number of points in minimal sample
     * @return Recommended number of iterations
     */
    int AdaptiveIterations(double inlierRatio, int sampleSize) const {
        if (inlierRatio <= 0.0 || inlierRatio >= 1.0) {
            return maxIterations;
        }

        // k = log(1 - confidence) / log(1 - w^n)
        // where w = inlier ratio, n = sample size
        double w_n = std::pow(inlierRatio, sampleSize);
        if (w_n >= 1.0) return 1;

        double k = std::log(1.0 - confidence) / std::log(1.0 - w_n);
        return std::min(static_cast<int>(std::ceil(k)), maxIterations);
    }
};

/**
 * @brief Weighted fitting parameters
 */
struct WeightParams {
    std::vector<double> weights;        ///< Per-point weights (must match point count)

    WeightParams() = default;

    explicit WeightParams(const std::vector<double>& w) : weights(w) {}

    WeightParams(std::initializer_list<double> w) : weights(w) {}
};

/**
 * @brief Geometric (iterative) fitting parameters
 */
struct GeometricFitParams {
    double tolerance = GEOMETRIC_FIT_TOLERANCE;     ///< Convergence tolerance
    int maxIterations = GEOMETRIC_FIT_MAX_ITERATIONS; ///< Maximum iterations
    bool useInitialGuess = false;                   ///< Use provided initial guess

    GeometricFitParams() = default;

    GeometricFitParams& SetTolerance(double value) {
        tolerance = value;
        return *this;
    }

    GeometricFitParams& SetMaxIterations(int value) {
        maxIterations = value;
        return *this;
    }

    GeometricFitParams& SetUseInitialGuess(bool value) {
        useInitialGuess = value;
        return *this;
    }
};

// =============================================================================
// Line Fitting Functions
// =============================================================================

/**
 * @brief Fit line using standard least squares
 *
 * Minimizes sum of squared orthogonal distances to line.
 * Uses Total Least Squares (TLS) approach via SVD.
 *
 * @param points Input points (at least 2)
 * @param params Fitting parameters
 * @return LineFitResult with fitted line and statistics
 *
 * Complexity: O(n) for centroid + O(n) for covariance + O(1) for eigendecomp
 */
LineFitResult FitLine(const std::vector<Point2d>& points,
                      const FitParams& params = FitParams());

/**
 * @brief Fit line using specified method
 *
 * @param points Input points
 * @param method Fitting method to use
 * @param params Fitting parameters
 * @return LineFitResult
 */
LineFitResult FitLine(const std::vector<Point2d>& points,
                      FitMethod method,
                      const FitParams& params = FitParams());

/**
 * @brief Fit line using weighted least squares
 *
 * Higher weights give more influence to corresponding points.
 * Weights should be non-negative.
 *
 * @param points Input points
 * @param weights Per-point weights (same size as points)
 * @param params Fitting parameters
 * @return LineFitResult
 */
LineFitResult FitLineWeighted(const std::vector<Point2d>& points,
                               const std::vector<double>& weights,
                               const FitParams& params = FitParams());

/**
 * @brief Fit line using Huber robust estimator
 *
 * Uses iteratively reweighted least squares (IRLS) with Huber weight function.
 * More robust to outliers than standard least squares.
 *
 * @param points Input points
 * @param sigma Scale estimate (if <= 0, computed automatically via MAD)
 * @param params Fitting parameters
 * @return LineFitResult
 */
LineFitResult FitLineHuber(const std::vector<Point2d>& points,
                            double sigma = 0.0,
                            const FitParams& params = FitParams());

/**
 * @brief Fit line using Tukey biweight robust estimator
 *
 * Uses IRLS with Tukey biweight function.
 * Completely rejects outliers beyond threshold.
 *
 * @param points Input points
 * @param sigma Scale estimate (if <= 0, computed automatically via MAD)
 * @param params Fitting parameters
 * @return LineFitResult
 */
LineFitResult FitLineTukey(const std::vector<Point2d>& points,
                            double sigma = 0.0,
                            const FitParams& params = FitParams());

/**
 * @brief Fit line using RANSAC
 *
 * Robust fitting that can handle high outlier ratios (up to 50%).
 *
 * @param points Input points
 * @param ransacParams RANSAC parameters
 * @param params Fitting parameters
 * @return LineFitResult with inlier mask
 */
LineFitResult FitLineRANSAC(const std::vector<Point2d>& points,
                             const RansacParams& ransacParams = RansacParams(),
                             const FitParams& params = FitParams());

// =============================================================================
// Circle Fitting Functions
// =============================================================================

/**
 * @brief Fit circle using algebraic (Kasa) method
 *
 * Fast closed-form solution, but slightly biased for small arcs.
 * Minimizes algebraic distance: (x-cx)^2 + (y-cy)^2 - r^2
 *
 * @param points Input points (at least 3)
 * @param params Fitting parameters
 * @return CircleFitResult
 *
 * Complexity: O(n)
 */
CircleFitResult FitCircleAlgebraic(const std::vector<Point2d>& points,
                                    const FitParams& params = FitParams());

/**
 * @brief Fit circle using geometric (Levenberg-Marquardt) method
 *
 * Minimizes sum of squared orthogonal distances.
 * More accurate than algebraic, especially for small arcs.
 *
 * @param points Input points
 * @param geoParams Geometric fitting parameters
 * @param params Fitting parameters
 * @return CircleFitResult
 *
 * Complexity: O(n * iterations)
 */
CircleFitResult FitCircleGeometric(const std::vector<Point2d>& points,
                                    const GeometricFitParams& geoParams = GeometricFitParams(),
                                    const FitParams& params = FitParams());

/**
 * @brief Fit circle using specified method
 *
 * @param points Input points
 * @param method Circle fitting method
 * @param params Fitting parameters
 * @return CircleFitResult
 */
CircleFitResult FitCircle(const std::vector<Point2d>& points,
                           CircleFitMethod method,
                           const FitParams& params = FitParams());

/**
 * @brief Fit circle using weighted algebraic method
 *
 * @param points Input points
 * @param weights Per-point weights
 * @param params Fitting parameters
 * @return CircleFitResult
 */
CircleFitResult FitCircleWeighted(const std::vector<Point2d>& points,
                                   const std::vector<double>& weights,
                                   const FitParams& params = FitParams());

/**
 * @brief Fit circle using Huber robust estimator
 *
 * @param points Input points
 * @param geometric If true, use geometric fitting; if false, use algebraic
 * @param sigma Scale estimate (if <= 0, computed automatically)
 * @param params Fitting parameters
 * @return CircleFitResult
 */
CircleFitResult FitCircleHuber(const std::vector<Point2d>& points,
                                bool geometric = true,
                                double sigma = 0.0,
                                const FitParams& params = FitParams());

/**
 * @brief Fit circle using Tukey biweight robust estimator
 *
 * @param points Input points
 * @param geometric If true, use geometric fitting; if false, use algebraic
 * @param sigma Scale estimate (if <= 0, computed automatically)
 * @param params Fitting parameters
 * @return CircleFitResult
 */
CircleFitResult FitCircleTukey(const std::vector<Point2d>& points,
                                bool geometric = true,
                                double sigma = 0.0,
                                const FitParams& params = FitParams());

/**
 * @brief Fit circle using RANSAC
 *
 * @param points Input points
 * @param ransacParams RANSAC parameters
 * @param params Fitting parameters
 * @return CircleFitResult with inlier mask
 */
CircleFitResult FitCircleRANSAC(const std::vector<Point2d>& points,
                                 const RansacParams& ransacParams = RansacParams(),
                                 const FitParams& params = FitParams());

/**
 * @brief Fit circle exactly through 3 points
 *
 * @param p1 First point
 * @param p2 Second point
 * @param p3 Third point
 * @return Circle if points are not collinear, nullopt otherwise
 */
std::optional<Circle2d> FitCircleExact3Points(const Point2d& p1,
                                               const Point2d& p2,
                                               const Point2d& p3);

// =============================================================================
// Ellipse Fitting Functions
// =============================================================================

/**
 * @brief Fit ellipse using Fitzgibbon direct method
 *
 * Direct least squares fitting (Fitzgibbon et al. 1999).
 * Guarantees ellipse constraint (discriminant < 0).
 *
 * @param points Input points (at least 5)
 * @param params Fitting parameters
 * @return EllipseFitResult
 *
 * Complexity: O(n) + O(1) for 6x6 generalized eigenvalue problem
 */
EllipseFitResult FitEllipseFitzgibbon(const std::vector<Point2d>& points,
                                       const FitParams& params = FitParams());

/**
 * @brief Fit ellipse using geometric (Levenberg-Marquardt) method
 *
 * Minimizes sum of squared orthogonal distances.
 * More accurate but slower than Fitzgibbon.
 *
 * @param points Input points
 * @param geoParams Geometric fitting parameters
 * @param params Fitting parameters
 * @return EllipseFitResult
 */
EllipseFitResult FitEllipseGeometric(const std::vector<Point2d>& points,
                                      const GeometricFitParams& geoParams = GeometricFitParams(),
                                      const FitParams& params = FitParams());

/**
 * @brief Fit ellipse using specified method
 *
 * @param points Input points
 * @param method Ellipse fitting method
 * @param params Fitting parameters
 * @return EllipseFitResult
 */
EllipseFitResult FitEllipse(const std::vector<Point2d>& points,
                             EllipseFitMethod method,
                             const FitParams& params = FitParams());

/**
 * @brief Fit ellipse using Huber robust estimator
 *
 * Uses IRLS with Huber weight function on geometric distances.
 *
 * @param points Input points
 * @param sigma Scale estimate (if <= 0, computed automatically via MAD)
 * @param params Fitting parameters
 * @return EllipseFitResult with weights
 */
EllipseFitResult FitEllipseHuber(const std::vector<Point2d>& points,
                                  double sigma = 0.0,
                                  const FitParams& params = FitParams());

/**
 * @brief Fit ellipse using Tukey biweight robust estimator
 *
 * Uses IRLS with Tukey biweight function.
 * Completely rejects outliers beyond threshold.
 *
 * @param points Input points
 * @param sigma Scale estimate (if <= 0, computed automatically via MAD)
 * @param params Fitting parameters
 * @return EllipseFitResult with weights
 */
EllipseFitResult FitEllipseTukey(const std::vector<Point2d>& points,
                                  double sigma = 0.0,
                                  const FitParams& params = FitParams());

/**
 * @brief Fit ellipse using RANSAC
 *
 * @param points Input points
 * @param ransacParams RANSAC parameters
 * @param params Fitting parameters
 * @return EllipseFitResult with inlier mask
 */
EllipseFitResult FitEllipseRANSAC(const std::vector<Point2d>& points,
                                   const RansacParams& ransacParams = RansacParams(),
                                   const FitParams& params = FitParams());

// =============================================================================
// Rectangle Fitting Functions
// =============================================================================

/**
 * @brief Rectangle fitting result
 */
struct RectangleFitResult : public FitResultBase {
    RotatedRect2d rect;               ///< Fitted rectangle (center, size, angle)

    /// Get center
    Point2d Center() const { return rect.center; }

    /// Get half-length along major axis
    double Length1() const { return rect.width / 2.0; }

    /// Get half-length along minor axis
    double Length2() const { return rect.height / 2.0; }

    /// Get rotation angle in radians
    double Angle() const { return rect.angle; }

    /// Per-side fit results (4 lines)
    std::array<LineFitResult, 4> sideResults;
};

/**
 * @brief Fit rectangle from edge points using robust line fitting
 *
 * Algorithm:
 * 1. Segment points into 4 groups by initial rectangle estimate
 * 2. Fit 4 lines using robust method (Huber/Tukey)
 * 3. Compute rectangle from line intersections
 * 4. Optionally refine by iterating
 *
 * @param points Input edge points
 * @param initialRect Initial rectangle estimate (center, size, angle)
 * @param method Robust fitting method (Huber or Tukey)
 * @param params Fitting parameters
 * @return RectangleFitResult with fitted rectangle
 */
RectangleFitResult FitRectangle(const std::vector<Point2d>& points,
                                 const RotatedRect2d& initialRect,
                                 FitMethod method = FitMethod::Huber,
                                 const FitParams& params = FitParams());

/**
 * @brief Fit rectangle with iterative refinement
 *
 * Iterates segmentation and fitting until convergence.
 *
 * @param points Input edge points
 * @param initialRect Initial rectangle estimate
 * @param maxIterations Maximum refinement iterations
 * @param convergenceThreshold Stop when parameters change less than this
 * @param params Fitting parameters
 * @return RectangleFitResult
 */
RectangleFitResult FitRectangleIterative(const std::vector<Point2d>& points,
                                          const RotatedRect2d& initialRect,
                                          int maxIterations = 10,
                                          double convergenceThreshold = 0.01,
                                          const FitParams& params = FitParams());

/**
 * @brief Segment points into 4 groups by rectangle side
 *
 * Assigns each point to the nearest side of the rectangle.
 *
 * @param points Input edge points
 * @param rect Reference rectangle
 * @return Array of 4 point vectors (top, right, bottom, left sides)
 */
std::array<std::vector<Point2d>, 4> SegmentPointsByRectangleSide(
    const std::vector<Point2d>& points,
    const RotatedRect2d& rect);

/**
 * @brief Compute rectangle from 4 fitted lines
 *
 * Computes the best-fit rectangle from 4 lines by finding intersections
 * and computing center, dimensions, and orientation.
 *
 * @param lines 4 fitted lines (top, right, bottom, left)
 * @return Rectangle parameters, or nullopt if lines don't form valid rectangle
 */
std::optional<RotatedRect2d> RectangleFromLines(const std::array<Line2d, 4>& lines);

// =============================================================================
// Generic RANSAC Framework
// =============================================================================

/**
 * @brief RANSAC model interface
 *
 * Template parameter ModelType should be the geometric model (Line2d, Circle2d, etc.)
 */
template<typename ModelType>
struct RansacModel {
    /// Minimum number of points to fit model
    int minSampleSize = 0;

    /// Function to fit model from minimal sample
    /// Returns nullopt if fitting fails
    std::function<std::optional<ModelType>(const std::vector<Point2d>&)> fitMinimal;

    /// Function to fit model from all inliers (refinement)
    std::function<std::optional<ModelType>(const std::vector<Point2d>&)> fitAll;

    /// Function to compute distance from point to model
    std::function<double(const ModelType&, const Point2d&)> distance;
};

/**
 * @brief RANSAC result
 */
template<typename ModelType>
struct RansacResult {
    bool success = false;           ///< Whether RANSAC found a valid model
    ModelType model;                ///< Best model found
    int numInliers = 0;             ///< Number of inliers for best model
    int numIterations = 0;          ///< Actual iterations performed
    std::vector<bool> inlierMask;   ///< Inlier mask for all points
    std::vector<double> residuals;  ///< Per-point distances to final model
};

/**
 * @brief Generic RANSAC algorithm
 *
 * @tparam ModelType Geometric model type
 * @param points Input points
 * @param modelFitter Model fitting interface
 * @param params RANSAC parameters
 * @return RansacResult with best model and inlier information
 *
 * @note Uses adaptive iteration count if confidence < 1.0
 */
template<typename ModelType>
RansacResult<ModelType> RANSAC(const std::vector<Point2d>& points,
                                const RansacModel<ModelType>& modelFitter,
                                const RansacParams& params = RansacParams());

// =============================================================================
// Robust Weight Functions
// =============================================================================

/**
 * @brief Huber weight function
 *
 * w(r) = 1           if |r| <= k
 *      = k / |r|     otherwise
 *
 * @param residual Standardized residual (residual / sigma)
 * @param k Tuning constant (default HUBER_K = 1.345)
 * @return Weight in [0, 1]
 */
inline double HuberWeight(double residual, double k = HUBER_K) {
    double absR = std::abs(residual);
    if (absR <= k) {
        return 1.0;
    }
    return k / absR;
}

/**
 * @brief Tukey biweight function
 *
 * w(r) = (1 - (r/c)^2)^2   if |r| <= c
 *      = 0                  otherwise
 *
 * @param residual Standardized residual (residual / sigma)
 * @param c Tuning constant (default TUKEY_C = 4.685)
 * @return Weight in [0, 1]
 */
inline double TukeyWeight(double residual, double c = TUKEY_C) {
    double absR = std::abs(residual);
    if (absR > c) {
        return 0.0;
    }
    double t = residual / c;
    double w = 1.0 - t * t;
    return w * w;
}

/**
 * @brief Robust scale estimator using Median Absolute Deviation (MAD)
 *
 * MAD = median(|x - median(x)|)
 * sigma = 1.4826 * MAD (consistent for Gaussian)
 *
 * @param residuals Residual values
 * @return Robust scale estimate
 */
double RobustScaleMAD(const std::vector<double>& residuals);

/**
 * @brief Robust scale estimator using Interquartile Range (IQR)
 *
 * IQR = Q3 - Q1
 * sigma = IQR / 1.349 (consistent for Gaussian)
 *
 * @param residuals Residual values
 * @return Robust scale estimate
 */
double RobustScaleIQR(const std::vector<double>& residuals);

// =============================================================================
// Residual Computation Functions
// =============================================================================

/**
 * @brief Compute residuals (signed distances) from points to line
 *
 * @param points Input points
 * @param line Fitted line
 * @return Vector of signed distances (positive = above line normal direction)
 */
std::vector<double> ComputeLineResiduals(const std::vector<Point2d>& points,
                                          const Line2d& line);

/**
 * @brief Compute residuals (signed distances) from points to circle
 *
 * @param points Input points
 * @param circle Fitted circle
 * @return Vector of signed distances (positive = outside circle)
 */
std::vector<double> ComputeCircleResiduals(const std::vector<Point2d>& points,
                                            const Circle2d& circle);

/**
 * @brief Compute residuals (approximate distances) from points to ellipse
 *
 * @param points Input points
 * @param ellipse Fitted ellipse
 * @return Vector of approximate signed distances (positive = outside ellipse)
 */
std::vector<double> ComputeEllipseResiduals(const std::vector<Point2d>& points,
                                             const Ellipse2d& ellipse);

/**
 * @brief Compute residual statistics
 *
 * @param residuals Input residuals
 * @param mean Output: mean of absolute residuals
 * @param stdDev Output: standard deviation
 * @param maxAbs Output: maximum absolute value
 * @param rms Output: root mean square
 */
void ComputeResidualStats(const std::vector<double>& residuals,
                           double& mean,
                           double& stdDev,
                           double& maxAbs,
                           double& rms);

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Check if points are approximately collinear
 *
 * @param points Input points
 * @param tolerance Maximum perpendicular distance to best-fit line
 * @return true if all points lie approximately on a line
 */
bool ArePointsCollinear(const std::vector<Point2d>& points,
                         double tolerance = COLLINEAR_TOLERANCE);

/**
 * @brief Check if 3 points are collinear
 *
 * Uses cross product to check area of triangle.
 *
 * @param p1 First point
 * @param p2 Second point
 * @param p3 Third point
 * @param tolerance Area threshold (triangle area = 0.5 * |cross|)
 * @return true if points are collinear
 */
inline bool AreCollinear(const Point2d& p1, const Point2d& p2, const Point2d& p3,
                          double tolerance = COLLINEAR_TOLERANCE) {
    // Cross product of (p2 - p1) and (p3 - p1)
    double cross = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
    return std::abs(cross) < tolerance;
}

/**
 * @brief Compute centroid of points
 *
 * @param points Input points
 * @return Centroid (average position)
 */
Point2d ComputeCentroid(const std::vector<Point2d>& points);

/**
 * @brief Compute weighted centroid of points
 *
 * @param points Input points
 * @param weights Per-point weights
 * @return Weighted centroid
 */
Point2d ComputeWeightedCentroid(const std::vector<Point2d>& points,
                                 const std::vector<double>& weights);

/**
 * @brief Normalize points for numerical stability
 *
 * Translates centroid to origin and scales so RMS distance is sqrt(2).
 * Returns normalization matrix T such that p' = T * [p; 1]
 *
 * @param points Input points
 * @return Pair of (normalized points, 3x3 normalization matrix)
 */
std::pair<std::vector<Point2d>, Mat33> NormalizePoints(const std::vector<Point2d>& points);

/**
 * @brief Denormalize line from normalized coordinates
 *
 * @param normalizedLine Line in normalized coordinates
 * @param T Normalization matrix (from NormalizePoints)
 * @return Line in original coordinates
 */
Line2d DenormalizeLine(const Line2d& normalizedLine, const Mat33& T);

/**
 * @brief Denormalize circle from normalized coordinates
 *
 * @param normalizedCircle Circle in normalized coordinates
 * @param T Normalization matrix (from NormalizePoints)
 * @return Circle in original coordinates
 */
Circle2d DenormalizeCircle(const Circle2d& normalizedCircle, const Mat33& T);

/**
 * @brief Denormalize ellipse from normalized coordinates
 *
 * @param normalizedEllipse Ellipse in normalized coordinates
 * @param T Normalization matrix (from NormalizePoints)
 * @return Ellipse in original coordinates
 */
Ellipse2d DenormalizeEllipse(const Ellipse2d& normalizedEllipse, const Mat33& T);

// =============================================================================
// Template Implementation: RANSAC
// =============================================================================

template<typename ModelType>
RansacResult<ModelType> RANSAC(const std::vector<Point2d>& points,
                                const RansacModel<ModelType>& modelFitter,
                                const RansacParams& params) {
    RansacResult<ModelType> result;
    result.success = false;
    result.numIterations = 0;

    const int n = static_cast<int>(points.size());
    const int sampleSize = modelFitter.minSampleSize;

    // Check minimum points
    if (n < sampleSize) {
        return result;
    }

    // Initialize
    int bestInliers = 0;
    ModelType bestModel{};
    std::vector<int> sampleIndices(sampleSize);
    std::vector<Point2d> sample(sampleSize);
    std::vector<bool> bestInlierMask(n, false);

    // Adaptive iteration count
    int maxIter = params.maxIterations;
    int minInliers = params.minInliers > 0 ? params.minInliers : sampleSize;

    // Random number generation (simple LCG for reproducibility)
    uint32_t seed = 12345;
    auto randNext = [&seed]() -> uint32_t {
        seed = seed * 1103515245u + 12345u;
        return seed;
    };

    for (int iter = 0; iter < maxIter; ++iter) {
        result.numIterations = iter + 1;

        // Random sampling without replacement
        for (int i = 0; i < sampleSize; ++i) {
            bool unique = false;
            while (!unique) {
                sampleIndices[i] = randNext() % n;
                unique = true;
                for (int j = 0; j < i; ++j) {
                    if (sampleIndices[i] == sampleIndices[j]) {
                        unique = false;
                        break;
                    }
                }
            }
            sample[i] = points[sampleIndices[i]];
        }

        // Fit model from minimal sample
        auto modelOpt = modelFitter.fitMinimal(sample);
        if (!modelOpt.has_value()) {
            continue;
        }

        ModelType model = modelOpt.value();

        // Count inliers
        std::vector<bool> inlierMask(n, false);
        int numInliers = 0;
        for (int i = 0; i < n; ++i) {
            double dist = std::abs(modelFitter.distance(model, points[i]));
            if (dist <= params.threshold) {
                inlierMask[i] = true;
                ++numInliers;
            }
        }

        // Update best model
        if (numInliers > bestInliers) {
            bestInliers = numInliers;
            bestModel = model;
            bestInlierMask = inlierMask;

            // Update adaptive iteration count
            if (params.confidence < 1.0 && numInliers > 0) {
                double inlierRatio = static_cast<double>(numInliers) / n;
                int adaptiveIter = params.AdaptiveIterations(inlierRatio, sampleSize);
                maxIter = std::min(maxIter, adaptiveIter);
            }
        }
    }

    // Check minimum inliers
    if (bestInliers < minInliers) {
        return result;
    }

    // Refine model using all inliers
    if (modelFitter.fitAll) {
        std::vector<Point2d> inlierPoints;
        inlierPoints.reserve(bestInliers);
        for (int i = 0; i < n; ++i) {
            if (bestInlierMask[i]) {
                inlierPoints.push_back(points[i]);
            }
        }

        auto refinedOpt = modelFitter.fitAll(inlierPoints);
        if (refinedOpt.has_value()) {
            bestModel = refinedOpt.value();

            // Recompute inliers with refined model
            bestInliers = 0;
            for (int i = 0; i < n; ++i) {
                double dist = std::abs(modelFitter.distance(bestModel, points[i]));
                if (dist <= params.threshold) {
                    bestInlierMask[i] = true;
                    ++bestInliers;
                } else {
                    bestInlierMask[i] = false;
                }
            }
        }
    }

    // Compute residuals
    result.residuals.resize(n);
    for (int i = 0; i < n; ++i) {
        result.residuals[i] = modelFitter.distance(bestModel, points[i]);
    }

    result.success = true;
    result.model = bestModel;
    result.numInliers = bestInliers;
    result.inlierMask = bestInlierMask;

    return result;
}

} // namespace Qi::Vision::Internal
