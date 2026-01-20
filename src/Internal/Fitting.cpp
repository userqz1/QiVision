/**
 * @file Fitting.cpp
 * @brief Implementation of geometric fitting algorithms
 */

#include <QiVision/Internal/Fitting.h>
#include <QiVision/Internal/Eigen.h>
#include <QiVision/Platform/Random.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace Qi::Vision::Internal {

// =============================================================================
// Helper Functions (anonymous namespace)
// =============================================================================

namespace {

/// Compute median of a vector (modifies input)
double Median(std::vector<double>& values) {
    if (values.empty()) return 0.0;

    size_t n = values.size();
    size_t mid = n / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());

    if (n % 2 == 0) {
        double upper = values[mid];
        std::nth_element(values.begin(), values.begin() + mid - 1, values.end());
        return (values[mid - 1] + upper) / 2.0;
    }
    return values[mid];
}

/// Compute percentile of a vector
double Percentile(std::vector<double> values, double p) {
    if (values.empty()) return 0.0;

    std::sort(values.begin(), values.end());
    double idx = p * (values.size() - 1);
    size_t lower = static_cast<size_t>(idx);
    size_t upper = std::min(lower + 1, values.size() - 1);
    double frac = idx - lower;
    return values[lower] * (1.0 - frac) + values[upper] * frac;
}

/// Fill result statistics from residuals
void FillResidualStats(FitResultBase& result, const std::vector<double>& residuals,
                       const FitParams& params) {
    result.numPoints = static_cast<int>(residuals.size());

    if (residuals.empty()) {
        return;
    }

    // Compute statistics
    double sumAbs = 0.0;
    double sumSq = 0.0;
    double maxAbs = 0.0;

    for (double r : residuals) {
        double absR = std::abs(r);
        sumAbs += absR;
        sumSq += r * r;
        maxAbs = std::max(maxAbs, absR);
    }

    int n = static_cast<int>(residuals.size());
    result.residualMean = sumAbs / n;
    result.residualRMS = std::sqrt(sumSq / n);
    result.residualMax = maxAbs;

    // Standard deviation
    double sumSqDev = 0.0;
    for (double r : residuals) {
        double absR = std::abs(r);
        double dev = absR - result.residualMean;
        sumSqDev += dev * dev;
    }
    result.residualStd = std::sqrt(sumSqDev / n);

    if (params.computeResiduals) {
        result.residuals = residuals;
    }
}

} // anonymous namespace

// =============================================================================
// Robust Scale Estimators
// =============================================================================

double RobustScaleMAD(const std::vector<double>& residuals) {
    if (residuals.empty()) return 0.0;

    // Compute median
    std::vector<double> values = residuals;
    for (auto& v : values) v = std::abs(v);
    double med = Median(values);

    // Compute MAD (Median Absolute Deviation)
    for (auto& v : values) v = std::abs(v - med);
    double mad = Median(values);

    // Scale factor for consistency with Gaussian (MAD / 0.6745)
    return mad / 0.6745;
}

double RobustScaleIQR(const std::vector<double>& residuals) {
    if (residuals.size() < 4) {
        return RobustScaleMAD(residuals);
    }

    std::vector<double> absRes;
    absRes.reserve(residuals.size());
    for (double r : residuals) {
        absRes.push_back(std::abs(r));
    }

    double q1 = Percentile(absRes, 0.25);
    double q3 = Percentile(absRes, 0.75);
    double iqr = q3 - q1;

    // Scale factor for consistency with Gaussian
    return iqr / 1.349;
}

// =============================================================================
// Utility Functions
// =============================================================================

Point2d ComputeCentroid(const std::vector<Point2d>& points) {
    if (points.empty()) {
        return Point2d(0.0, 0.0);
    }

    double sx = 0.0, sy = 0.0;
    for (const auto& p : points) {
        sx += p.x;
        sy += p.y;
    }

    int n = static_cast<int>(points.size());
    return Point2d(sx / n, sy / n);
}

Point2d ComputeWeightedCentroid(const std::vector<Point2d>& points,
                                 const std::vector<double>& weights) {
    if (points.empty() || weights.size() != points.size()) {
        return ComputeCentroid(points);
    }

    double sx = 0.0, sy = 0.0, sw = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
        double w = weights[i];
        sx += w * points[i].x;
        sy += w * points[i].y;
        sw += w;
    }

    if (sw < 1e-15) {
        return ComputeCentroid(points);
    }

    return Point2d(sx / sw, sy / sw);
}

bool ArePointsCollinear(const std::vector<Point2d>& points, double tolerance) {
    if (points.size() < 3) {
        return true;  // 2 or fewer points are always collinear
    }

    // Fit line and check maximum residual
    auto result = FitLine(points);
    if (!result.success) {
        return true;  // Degenerate case
    }

    return result.residualMax < tolerance;
}

std::pair<std::vector<Point2d>, Mat33> NormalizePoints(const std::vector<Point2d>& points) {
    int n = static_cast<int>(points.size());

    if (n == 0) {
        return {{}, Mat33::Identity()};
    }

    // Compute centroid
    Point2d centroid = ComputeCentroid(points);

    // Compute RMS distance from centroid
    double sumSqDist = 0.0;
    for (const auto& p : points) {
        double dx = p.x - centroid.x;
        double dy = p.y - centroid.y;
        sumSqDist += dx * dx + dy * dy;
    }

    double rmsDist = std::sqrt(sumSqDist / n);
    double scale = (rmsDist > 1e-15) ? std::sqrt(2.0) / rmsDist : 1.0;

    // Build normalization matrix: T = [[s, 0, -s*cx], [0, s, -s*cy], [0, 0, 1]]
    Mat33 T;
    T(0, 0) = scale;  T(0, 1) = 0.0;    T(0, 2) = -scale * centroid.x;
    T(1, 0) = 0.0;    T(1, 1) = scale;  T(1, 2) = -scale * centroid.y;
    T(2, 0) = 0.0;    T(2, 1) = 0.0;    T(2, 2) = 1.0;

    // Normalize points
    std::vector<Point2d> normalized;
    normalized.reserve(n);
    for (const auto& p : points) {
        normalized.emplace_back(
            scale * (p.x - centroid.x),
            scale * (p.y - centroid.y)
        );
    }

    return {normalized, T};
}

Line2d DenormalizeLine(const Line2d& normalizedLine, const Mat33& T) {
    // Line l' = [a', b', c'] in normalized coords
    // Original line l = T^T * l'
    // l = [a*s, b*s, c - a*s*cx - b*s*cy]

    double scale = T(0, 0);  // s
    double tx = -T(0, 2) / scale;  // cx
    double ty = -T(1, 2) / scale;  // cy

    double a = normalizedLine.a * scale;
    double b = normalizedLine.b * scale;
    double c = normalizedLine.c - normalizedLine.a * scale * tx - normalizedLine.b * scale * ty;

    // Normalize
    double norm = std::sqrt(a * a + b * b);
    if (norm > 1e-15) {
        a /= norm;
        b /= norm;
        c /= norm;
    }

    return Line2d(a, b, c);
}

Circle2d DenormalizeCircle(const Circle2d& normalizedCircle, const Mat33& T) {
    double scale = T(0, 0);
    double tx = -T(0, 2) / scale;
    double ty = -T(1, 2) / scale;

    double cx = normalizedCircle.center.x / scale + tx;
    double cy = normalizedCircle.center.y / scale + ty;
    double r = normalizedCircle.radius / scale;

    return Circle2d(Point2d(cx, cy), r);
}

Ellipse2d DenormalizeEllipse(const Ellipse2d& normalizedEllipse, const Mat33& T) {
    double scale = T(0, 0);
    double tx = -T(0, 2) / scale;
    double ty = -T(1, 2) / scale;

    double cx = normalizedEllipse.center.x / scale + tx;
    double cy = normalizedEllipse.center.y / scale + ty;
    double a = normalizedEllipse.a / scale;
    double b = normalizedEllipse.b / scale;

    return Ellipse2d(Point2d(cx, cy), a, b, normalizedEllipse.angle);
}

// =============================================================================
// Residual Computation
// =============================================================================

std::vector<double> ComputeLineResiduals(const std::vector<Point2d>& points,
                                          const Line2d& line) {
    std::vector<double> residuals;
    residuals.reserve(points.size());

    double norm = std::sqrt(line.a * line.a + line.b * line.b);
    if (norm < 1e-15) norm = 1.0;

    for (const auto& p : points) {
        double dist = (line.a * p.x + line.b * p.y + line.c) / norm;
        residuals.push_back(dist);
    }

    return residuals;
}

std::vector<double> ComputeCircleResiduals(const std::vector<Point2d>& points,
                                            const Circle2d& circle) {
    std::vector<double> residuals;
    residuals.reserve(points.size());

    for (const auto& p : points) {
        double dx = p.x - circle.center.x;
        double dy = p.y - circle.center.y;
        double dist = std::sqrt(dx * dx + dy * dy) - circle.radius;
        residuals.push_back(dist);
    }

    return residuals;
}

std::vector<double> ComputeEllipseResiduals(const std::vector<Point2d>& points,
                                             const Ellipse2d& ellipse) {
    std::vector<double> residuals;
    residuals.reserve(points.size());

    double cosA = std::cos(-ellipse.angle);
    double sinA = std::sin(-ellipse.angle);

    for (const auto& p : points) {
        // Transform point to ellipse-centered coordinates
        double dx = p.x - ellipse.center.x;
        double dy = p.y - ellipse.center.y;

        // Rotate to align with ellipse axes
        double x = dx * cosA - dy * sinA;
        double y = dx * sinA + dy * cosA;

        // Algebraic distance (approximate)
        double a2 = ellipse.a * ellipse.a;
        double b2 = ellipse.b * ellipse.b;
        double val = (x * x) / a2 + (y * y) / b2 - 1.0;

        // Approximate geometric distance
        double dist = val * std::sqrt(a2 * b2) / std::sqrt(b2 * x * x + a2 * y * y + 1e-15);
        residuals.push_back(dist);
    }

    return residuals;
}

void ComputeResidualStats(const std::vector<double>& residuals,
                           double& mean, double& stdDev, double& maxAbs, double& rms) {
    mean = stdDev = maxAbs = rms = 0.0;

    if (residuals.empty()) return;

    int n = static_cast<int>(residuals.size());
    double sumAbs = 0.0, sumSq = 0.0;

    for (double r : residuals) {
        double absR = std::abs(r);
        sumAbs += absR;
        sumSq += r * r;
        maxAbs = std::max(maxAbs, absR);
    }

    mean = sumAbs / n;
    rms = std::sqrt(sumSq / n);

    double sumSqDev = 0.0;
    for (double r : residuals) {
        double dev = std::abs(r) - mean;
        sumSqDev += dev * dev;
    }
    stdDev = std::sqrt(sumSqDev / n);
}

// =============================================================================
// Line Fitting Implementation
// =============================================================================

LineFitResult FitLine(const std::vector<Point2d>& points, const FitParams& params) {
    LineFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < LINE_FIT_MIN_POINTS) {
        return result;
    }

    // Compute centroid
    Point2d centroid = ComputeCentroid(points);

    // Build covariance matrix
    double sxx = 0.0, sxy = 0.0, syy = 0.0;
    for (const auto& p : points) {
        double dx = p.x - centroid.x;
        double dy = p.y - centroid.y;
        sxx += dx * dx;
        sxy += dx * dy;
        syy += dy * dy;
    }

    // Eigendecomposition of 2x2 covariance matrix
    // The eigenvector for smaller eigenvalue gives line normal
    // For M = [[sxx, sxy], [sxy, syy]]
    // eigenvalues: lambda = (trace +/- sqrt(trace^2 - 4*det)) / 2

    double trace = sxx + syy;
    double det = sxx * syy - sxy * sxy;
    double disc = trace * trace - 4.0 * det;

    if (disc < 0) disc = 0;
    double sqrtDisc = std::sqrt(disc);

    double lambda1 = (trace + sqrtDisc) / 2.0;
    double lambda2 = (trace - sqrtDisc) / 2.0;

    // Eigenvector for smaller eigenvalue (lambda2)
    double a, b;
    if (std::abs(sxy) > 1e-15) {
        // Normal direction from smaller eigenvalue
        a = lambda2 - syy;
        b = sxy;
    } else if (sxx < syy) {
        a = 1.0;
        b = 0.0;
    } else {
        a = 0.0;
        b = 1.0;
    }

    // Normalize
    double norm = std::sqrt(a * a + b * b);
    if (norm < 1e-15) {
        return result;
    }
    a /= norm;
    b /= norm;

    // Compute c: ax + by + c = 0 passes through centroid
    double c = -(a * centroid.x + b * centroid.y);

    result.line = Line2d(a, b, c);
    result.success = true;

    // Compute residuals and statistics
    auto residuals = ComputeLineResiduals(points, result.line);
    FillResidualStats(result, residuals, params);
    result.numInliers = result.numPoints;

    return result;
}

LineFitResult FitLine(const std::vector<Point2d>& points, FitMethod method,
                      const FitParams& params) {
    switch (method) {
        case FitMethod::LeastSquares:
            return FitLine(points, params);
        case FitMethod::Huber:
            return FitLineHuber(points, 0.0, params);
        case FitMethod::Tukey:
            return FitLineTukey(points, 0.0, params);
        case FitMethod::RANSAC:
            return FitLineRANSAC(points, RansacParams(), params);
        default:
            return FitLine(points, params);
    }
}

LineFitResult FitLineWeighted(const std::vector<Point2d>& points,
                               const std::vector<double>& weights,
                               const FitParams& params) {
    LineFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < LINE_FIT_MIN_POINTS || weights.size() != points.size()) {
        return result;
    }

    // Compute weighted centroid
    Point2d centroid = ComputeWeightedCentroid(points, weights);

    // Build weighted covariance matrix
    double sxx = 0.0, sxy = 0.0, syy = 0.0, sw = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
        double w = weights[i];
        double dx = points[i].x - centroid.x;
        double dy = points[i].y - centroid.y;
        sxx += w * dx * dx;
        sxy += w * dx * dy;
        syy += w * dy * dy;
        sw += w;
    }

    if (sw < 1e-15) {
        return FitLine(points, params);
    }

    // Normalize
    sxx /= sw;
    sxy /= sw;
    syy /= sw;

    // Eigendecomposition
    double trace = sxx + syy;
    double det = sxx * syy - sxy * sxy;
    double disc = trace * trace - 4.0 * det;
    if (disc < 0) disc = 0;

    double lambda2 = (trace - std::sqrt(disc)) / 2.0;

    double a, b;
    if (std::abs(sxy) > 1e-15) {
        a = lambda2 - syy;
        b = sxy;
    } else if (sxx < syy) {
        a = 1.0;
        b = 0.0;
    } else {
        a = 0.0;
        b = 1.0;
    }

    double norm = std::sqrt(a * a + b * b);
    if (norm < 1e-15) return result;
    a /= norm;
    b /= norm;

    double c = -(a * centroid.x + b * centroid.y);

    result.line = Line2d(a, b, c);
    result.success = true;

    auto residuals = ComputeLineResiduals(points, result.line);
    FillResidualStats(result, residuals, params);
    result.numInliers = result.numPoints;

    return result;
}

LineFitResult FitLineHuber(const std::vector<Point2d>& points, double sigma,
                            const FitParams& params) {
    LineFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < LINE_FIT_MIN_POINTS) {
        return result;
    }

    // Initial fit
    result = FitLine(points, params);
    if (!result.success) return result;

    // IRLS iterations
    const int maxIter = 20;
    const double tol = 1e-6;

    std::vector<double> weights(points.size(), 1.0);

    for (int iter = 0; iter < maxIter; ++iter) {
        // Compute residuals
        auto residuals = ComputeLineResiduals(points, result.line);

        // Estimate scale if not provided
        double scale = sigma;
        if (scale <= 0) {
            scale = RobustScaleMAD(residuals);
            if (scale < 1e-10) scale = 1.0;
        }

        // Compute weights
        double maxWeightChange = 0.0;
        for (size_t i = 0; i < points.size(); ++i) {
            double r = residuals[i] / scale;
            double newWeight = HuberWeight(r);
            maxWeightChange = std::max(maxWeightChange, std::abs(newWeight - weights[i]));
            weights[i] = newWeight;
        }

        // Refit with weights
        result = FitLineWeighted(points, weights, params);
        if (!result.success) return result;

        // Check convergence
        if (maxWeightChange < tol) break;
    }

    // Final residuals
    auto residuals = ComputeLineResiduals(points, result.line);
    FillResidualStats(result, residuals, params);
    result.numInliers = result.numPoints;

    return result;
}

LineFitResult FitLineTukey(const std::vector<Point2d>& points, double sigma,
                            const FitParams& params) {
    LineFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < LINE_FIT_MIN_POINTS) {
        return result;
    }

    // Initial fit
    result = FitLine(points, params);
    if (!result.success) return result;

    // IRLS iterations
    const int maxIter = 20;
    const double tol = 1e-6;

    std::vector<double> weights(points.size(), 1.0);

    for (int iter = 0; iter < maxIter; ++iter) {
        auto residuals = ComputeLineResiduals(points, result.line);

        double scale = sigma;
        if (scale <= 0) {
            scale = RobustScaleMAD(residuals);
            if (scale < 1e-10) scale = 1.0;
        }

        double maxWeightChange = 0.0;
        int numNonZero = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            double r = residuals[i] / scale;
            double newWeight = TukeyWeight(r);
            maxWeightChange = std::max(maxWeightChange, std::abs(newWeight - weights[i]));
            weights[i] = newWeight;
            if (newWeight > 0) ++numNonZero;
        }

        // Need at least MIN_POINTS non-zero weights
        if (numNonZero < LINE_FIT_MIN_POINTS) {
            break;
        }

        result = FitLineWeighted(points, weights, params);
        if (!result.success) return result;

        if (maxWeightChange < tol) break;
    }

    auto residuals = ComputeLineResiduals(points, result.line);
    FillResidualStats(result, residuals, params);

    // Count inliers (non-zero weights)
    result.numInliers = 0;
    double scale = RobustScaleMAD(residuals);
    if (scale < 1e-10) scale = 1.0;
    for (size_t i = 0; i < points.size(); ++i) {
        if (TukeyWeight(residuals[i] / scale) > 0) {
            ++result.numInliers;
        }
    }

    return result;
}

LineFitResult FitLineRANSAC(const std::vector<Point2d>& points,
                             const RansacParams& ransacParams,
                             const FitParams& params) {
    LineFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < LINE_FIT_MIN_POINTS) {
        return result;
    }

    // Define RANSAC model for line
    RansacModel<Line2d> lineModel;
    lineModel.minSampleSize = 2;

    lineModel.fitMinimal = [](const std::vector<Point2d>& pts) -> std::optional<Line2d> {
        if (pts.size() < 2) return std::nullopt;

        const Point2d& p1 = pts[0];
        const Point2d& p2 = pts[1];

        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double len = std::sqrt(dx * dx + dy * dy);

        if (len < 1e-15) return std::nullopt;

        // Normal direction: (-dy, dx)
        double a = -dy / len;
        double b = dx / len;
        double c = -(a * p1.x + b * p1.y);

        return Line2d(a, b, c);
    };

    lineModel.fitAll = [](const std::vector<Point2d>& pts) -> std::optional<Line2d> {
        if (pts.size() < 2) return std::nullopt;
        auto res = FitLine(pts);
        if (!res.success) return std::nullopt;
        return res.line;
    };

    lineModel.distance = [](const Line2d& line, const Point2d& p) -> double {
        double norm = std::sqrt(line.a * line.a + line.b * line.b);
        if (norm < 1e-15) return 0.0;
        return (line.a * p.x + line.b * p.y + line.c) / norm;
    };

    // Run RANSAC
    auto ransacResult = RANSAC<Line2d>(points, lineModel, ransacParams);

    if (!ransacResult.success) {
        return result;
    }

    result.line = ransacResult.model;
    result.success = true;
    result.numInliers = ransacResult.numInliers;

    if (params.computeInlierMask) {
        result.inlierMask = ransacResult.inlierMask;
    }

    auto residuals = ComputeLineResiduals(points, result.line);
    FillResidualStats(result, residuals, params);

    return result;
}

// =============================================================================
// Circle Fitting Implementation
// =============================================================================

CircleFitResult FitCircleAlgebraic(const std::vector<Point2d>& points,
                                    const FitParams& params) {
    CircleFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < CIRCLE_FIT_MIN_POINTS) {
        return result;
    }

    // Kasa method: (x - a)^2 + (y - b)^2 = r^2
    // Linearize: x^2 + y^2 + Ax + By + C = 0
    // where A = -2a, B = -2b, C = a^2 + b^2 - r^2

    int n = static_cast<int>(points.size());

    // Build normal equations: M^T M x = M^T b
    // where M = [x, y, 1], b = -(x^2 + y^2)

    double sxx = 0.0, sxy = 0.0, sx = 0.0;
    double syy = 0.0, sy = 0.0;
    double s1 = static_cast<double>(n);
    double bx = 0.0, by = 0.0, b1 = 0.0;

    for (const auto& p : points) {
        double x2 = p.x * p.x;
        double y2 = p.y * p.y;
        double rhs = -(x2 + y2);

        sxx += x2;
        sxy += p.x * p.y;
        sx += p.x;
        syy += y2;
        sy += p.y;

        bx += p.x * rhs;
        by += p.y * rhs;
        b1 += rhs;
    }

    // Solve 3x3 system using Cramer's rule or LU
    // [sxx sxy sx ] [A]   [bx]
    // [sxy syy sy ] [B] = [by]
    // [sx  sy  s1 ] [C]   [b1]

    Mat33 M;
    M(0, 0) = sxx; M(0, 1) = sxy; M(0, 2) = sx;
    M(1, 0) = sxy; M(1, 1) = syy; M(1, 2) = sy;
    M(2, 0) = sx;  M(2, 1) = sy;  M(2, 2) = s1;

    Vec3 rhs{bx, by, b1};
    Vec3 sol = Solve3x3(M, rhs);

    double A = sol[0];
    double B = sol[1];
    double C = sol[2];

    // Extract circle parameters
    double cx = -A / 2.0;
    double cy = -B / 2.0;
    double r2 = cx * cx + cy * cy - C;

    if (r2 <= 0) {
        return result;  // Invalid circle
    }

    result.circle = Circle2d(Point2d(cx, cy), std::sqrt(r2));
    result.success = true;

    auto residuals = ComputeCircleResiduals(points, result.circle);
    FillResidualStats(result, residuals, params);
    result.numInliers = result.numPoints;

    return result;
}

CircleFitResult FitCircleGeometric(const std::vector<Point2d>& points,
                                    const GeometricFitParams& geoParams,
                                    const FitParams& params) {
    CircleFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < CIRCLE_FIT_MIN_POINTS) {
        return result;
    }

    // Initial estimate from algebraic fitting
    auto algebraicResult = FitCircleAlgebraic(points, FitParams());
    if (!algebraicResult.success) {
        return result;
    }

    double cx = algebraicResult.circle.center.x;
    double cy = algebraicResult.circle.center.y;
    double r = algebraicResult.circle.radius;

    int n = static_cast<int>(points.size());

    // Gauss-Newton iterations
    for (int iter = 0; iter < geoParams.maxIterations; ++iter) {
        // Build Jacobian J and residual vector
        MatX J(n, 3);
        VecX residual(n);

        for (int i = 0; i < n; ++i) {
            double dx = points[i].x - cx;
            double dy = points[i].y - cy;
            double d = std::sqrt(dx * dx + dy * dy);

            if (d < 1e-15) d = 1e-15;

            // Residual: d_i - r
            residual[i] = d - r;

            // Jacobian: dr/d(cx) = -dx/d, dr/d(cy) = -dy/d, dr/d(r) = -1
            J(i, 0) = -dx / d;
            J(i, 1) = -dy / d;
            J(i, 2) = -1.0;
        }

        // Solve J^T J delta = -J^T r
        VecX delta = SolveLeastSquares(J, residual);

        // Update parameters
        cx -= delta[0];
        cy -= delta[1];
        r -= delta[2];

        // Check convergence
        if (delta.Norm() < geoParams.tolerance) {
            break;
        }
    }

    if (r <= 0) {
        return result;
    }

    result.circle = Circle2d(Point2d(cx, cy), r);
    result.success = true;

    auto residuals = ComputeCircleResiduals(points, result.circle);
    FillResidualStats(result, residuals, params);
    result.numInliers = result.numPoints;

    return result;
}

CircleFitResult FitCircle(const std::vector<Point2d>& points,
                           CircleFitMethod method,
                           const FitParams& params) {
    switch (method) {
        case CircleFitMethod::Algebraic:
            return FitCircleAlgebraic(points, params);
        case CircleFitMethod::Geometric:
            return FitCircleGeometric(points, GeometricFitParams(), params);
        case CircleFitMethod::AlgebraicHuber:
        case CircleFitMethod::GeoHuber:
            return FitCircleHuber(points, method == CircleFitMethod::GeoHuber, 0.0, params);
        case CircleFitMethod::AlgebraicTukey:
        case CircleFitMethod::GeoTukey:
            return FitCircleTukey(points, method == CircleFitMethod::GeoTukey, 0.0, params);
        case CircleFitMethod::RANSAC:
            return FitCircleRANSAC(points, RansacParams(), params);
        default:
            return FitCircleAlgebraic(points, params);
    }
}

CircleFitResult FitCircleWeighted(const std::vector<Point2d>& points,
                                   const std::vector<double>& weights,
                                   const FitParams& params) {
    CircleFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < CIRCLE_FIT_MIN_POINTS || weights.size() != points.size()) {
        return result;
    }

    // Weighted Kasa method
    double sxx = 0.0, sxy = 0.0, sx = 0.0;
    double syy = 0.0, sy = 0.0, sw = 0.0;
    double bx = 0.0, by = 0.0, b1 = 0.0;

    for (size_t i = 0; i < points.size(); ++i) {
        double w = weights[i];
        double x = points[i].x;
        double y = points[i].y;
        double x2 = x * x;
        double y2 = y * y;
        double rhs = -(x2 + y2);

        sxx += w * x2;
        sxy += w * x * y;
        sx += w * x;
        syy += w * y2;
        sy += w * y;
        sw += w;

        bx += w * x * rhs;
        by += w * y * rhs;
        b1 += w * rhs;
    }

    if (sw < 1e-15) {
        return FitCircleAlgebraic(points, params);
    }

    Mat33 M;
    M(0, 0) = sxx; M(0, 1) = sxy; M(0, 2) = sx;
    M(1, 0) = sxy; M(1, 1) = syy; M(1, 2) = sy;
    M(2, 0) = sx;  M(2, 1) = sy;  M(2, 2) = sw;

    Vec3 rhs{bx, by, b1};
    Vec3 sol = Solve3x3(M, rhs);

    double cx = -sol[0] / 2.0;
    double cy = -sol[1] / 2.0;
    double r2 = cx * cx + cy * cy - sol[2];

    if (r2 <= 0) return result;

    result.circle = Circle2d(Point2d(cx, cy), std::sqrt(r2));
    result.success = true;

    auto residuals = ComputeCircleResiduals(points, result.circle);
    FillResidualStats(result, residuals, params);
    result.numInliers = result.numPoints;

    return result;
}

CircleFitResult FitCircleHuber(const std::vector<Point2d>& points,
                                bool geometric, double sigma,
                                const FitParams& params) {
    CircleFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < CIRCLE_FIT_MIN_POINTS) {
        return result;
    }

    // Initial fit
    result = geometric ? FitCircleGeometric(points, GeometricFitParams(), params)
                       : FitCircleAlgebraic(points, params);
    if (!result.success) return result;

    const int maxIter = 20;
    const double tol = 1e-6;
    std::vector<double> weights(points.size(), 1.0);

    for (int iter = 0; iter < maxIter; ++iter) {
        auto residuals = ComputeCircleResiduals(points, result.circle);

        double scale = sigma;
        if (scale <= 0) {
            scale = RobustScaleMAD(residuals);
            if (scale < 1e-10) scale = 1.0;
        }

        double maxWeightChange = 0.0;
        for (size_t i = 0; i < points.size(); ++i) {
            double r = residuals[i] / scale;
            double newWeight = HuberWeight(r);
            maxWeightChange = std::max(maxWeightChange, std::abs(newWeight - weights[i]));
            weights[i] = newWeight;
        }

        result = FitCircleWeighted(points, weights, params);
        if (!result.success) return result;

        if (geometric) {
            // Refine with geometric fitting
            auto geoResult = FitCircleGeometric(points, GeometricFitParams(), params);
            if (geoResult.success) {
                result = geoResult;
            }
        }

        if (maxWeightChange < tol) break;
    }

    auto residuals = ComputeCircleResiduals(points, result.circle);
    FillResidualStats(result, residuals, params);
    result.weights = weights;  // Save weights for outlier visualization
    result.numInliers = result.numPoints;

    return result;
}

CircleFitResult FitCircleTukey(const std::vector<Point2d>& points,
                                bool geometric, double sigma,
                                const FitParams& params) {
    CircleFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < CIRCLE_FIT_MIN_POINTS) {
        return result;
    }

    result = geometric ? FitCircleGeometric(points, GeometricFitParams(), params)
                       : FitCircleAlgebraic(points, params);
    if (!result.success) return result;

    const int maxIter = 20;
    const double tol = 1e-6;
    std::vector<double> weights(points.size(), 1.0);

    for (int iter = 0; iter < maxIter; ++iter) {
        auto residuals = ComputeCircleResiduals(points, result.circle);

        double scale = sigma;
        if (scale <= 0) {
            scale = RobustScaleMAD(residuals);
            if (scale < 1e-10) scale = 1.0;
        }

        double maxWeightChange = 0.0;
        int numNonZero = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            double r = residuals[i] / scale;
            double newWeight = TukeyWeight(r);
            maxWeightChange = std::max(maxWeightChange, std::abs(newWeight - weights[i]));
            weights[i] = newWeight;
            if (newWeight > 0) ++numNonZero;
        }

        if (numNonZero < CIRCLE_FIT_MIN_POINTS) break;

        result = FitCircleWeighted(points, weights, params);
        if (!result.success) return result;

        if (maxWeightChange < tol) break;
    }

    auto residuals = ComputeCircleResiduals(points, result.circle);
    FillResidualStats(result, residuals, params);
    result.weights = weights;  // Save weights for outlier visualization

    double scale = RobustScaleMAD(residuals);
    if (scale < 1e-10) scale = 1.0;
    result.numInliers = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        if (TukeyWeight(residuals[i] / scale) > 0) ++result.numInliers;
    }

    return result;
}

CircleFitResult FitCircleRANSAC(const std::vector<Point2d>& points,
                                 const RansacParams& ransacParams,
                                 const FitParams& params) {
    CircleFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < CIRCLE_FIT_MIN_POINTS) {
        return result;
    }

    RansacModel<Circle2d> circleModel;
    circleModel.minSampleSize = 3;

    circleModel.fitMinimal = [](const std::vector<Point2d>& pts) -> std::optional<Circle2d> {
        if (pts.size() < 3) return std::nullopt;
        return FitCircleExact3Points(pts[0], pts[1], pts[2]);
    };

    circleModel.fitAll = [](const std::vector<Point2d>& pts) -> std::optional<Circle2d> {
        if (pts.size() < 3) return std::nullopt;
        auto res = FitCircleAlgebraic(pts);
        if (!res.success) return std::nullopt;
        return res.circle;
    };

    circleModel.distance = [](const Circle2d& c, const Point2d& p) -> double {
        double dx = p.x - c.center.x;
        double dy = p.y - c.center.y;
        return std::sqrt(dx * dx + dy * dy) - c.radius;
    };

    auto ransacResult = RANSAC<Circle2d>(points, circleModel, ransacParams);

    if (!ransacResult.success) return result;

    result.circle = ransacResult.model;
    result.success = true;
    result.numInliers = ransacResult.numInliers;

    if (params.computeInlierMask) {
        result.inlierMask = ransacResult.inlierMask;
    }

    auto residuals = ComputeCircleResiduals(points, result.circle);
    FillResidualStats(result, residuals, params);

    return result;
}

std::optional<Circle2d> FitCircleExact3Points(const Point2d& p1,
                                               const Point2d& p2,
                                               const Point2d& p3) {
    // Check collinearity
    if (AreCollinear(p1, p2, p3)) {
        return std::nullopt;
    }

    // Perpendicular bisector method
    // Midpoints
    double mx1 = (p1.x + p2.x) / 2.0;
    double my1 = (p1.y + p2.y) / 2.0;
    double mx2 = (p2.x + p3.x) / 2.0;
    double my2 = (p2.y + p3.y) / 2.0;

    // Direction vectors of edges
    double dx1 = p2.x - p1.x;
    double dy1 = p2.y - p1.y;
    double dx2 = p3.x - p2.x;
    double dy2 = p3.y - p2.y;

    // Perpendicular directions
    double px1 = -dy1, py1 = dx1;
    double px2 = -dy2, py2 = dx2;

    // Intersection of perpendicular bisectors
    // (mx1, my1) + t * (px1, py1) = (mx2, my2) + s * (px2, py2)
    double denom = px1 * py2 - py1 * px2;
    if (std::abs(denom) < 1e-15) {
        return std::nullopt;
    }

    double t = ((mx2 - mx1) * py2 - (my2 - my1) * px2) / denom;

    double cx = mx1 + t * px1;
    double cy = my1 + t * py1;
    double r = std::sqrt((cx - p1.x) * (cx - p1.x) + (cy - p1.y) * (cy - p1.y));

    return Circle2d(Point2d(cx, cy), r);
}

// =============================================================================
// Ellipse Fitting Implementation
// =============================================================================

EllipseFitResult FitEllipseFitzgibbon(const std::vector<Point2d>& points,
                                       const FitParams& params) {
    EllipseFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < ELLIPSE_FIT_MIN_POINTS) {
        return result;
    }

    int n = static_cast<int>(points.size());

    // Normalize points for numerical stability
    auto [normPoints, T] = NormalizePoints(points);

    // Build design matrix D = [x^2, xy, y^2, x, y, 1]
    MatX D(n, 6);
    for (int i = 0; i < n; ++i) {
        double x = normPoints[i].x;
        double y = normPoints[i].y;
        D(i, 0) = x * x;
        D(i, 1) = x * y;
        D(i, 2) = y * y;
        D(i, 3) = x;
        D(i, 4) = y;
        D(i, 5) = 1.0;
    }

    // Scatter matrix S = D^T D
    MatX S = D.Transpose() * D;

    // Constraint matrix C for ellipse: 4ac - b^2 > 0
    // Using constraint b^2 - 4ac = -1 (normalized)
    // C = [0 0 -2; 0 1 0; -2 0 0; 0 0 0; 0 0 0; 0 0 0] extended to 6x6
    Mat33 C1, C2;
    C1(0, 0) = 0;  C1(0, 1) = 0;  C1(0, 2) = -2;
    C1(1, 0) = 0;  C1(1, 1) = 1;  C1(1, 2) = 0;
    C1(2, 0) = -2; C1(2, 1) = 0;  C1(2, 2) = 0;

    // Partition S into blocks
    // S = [S1  S2]  where S1 is 3x3 (quadratic terms)
    //     [S2' S3]        S3 is 3x3 (linear terms)
    Mat33 S1, S3;
    MatX S2(3, 3);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            S1(i, j) = S(i, j);
            S3(i, j) = S(i + 3, j + 3);
            S2(i, j) = S(i, j + 3);
        }
    }

    // Reduced problem: M * a1 = lambda * C1 * a1
    // where M = S1 - S2 * S3^{-1} * S2^T

    // Solve S3 * X = S2^T for X
    Mat33 S3inv;
    if (!S3.IsInvertible()) {
        return result;
    }
    S3inv = S3.Inverse();

    MatX S2T = S2.Transpose();
    MatX temp(3, 3);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            temp(i, j) = 0;
            for (int k = 0; k < 3; ++k) {
                temp(i, j) += S3inv(i, k) * S2T(k, j);
            }
        }
    }

    Mat33 M;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            M(i, j) = S1(i, j);
            for (int k = 0; k < 3; ++k) {
                M(i, j) -= S2(i, k) * temp(k, j);
            }
        }
    }

    // Solve C1^{-1} * M * a1 = lambda * a1
    // C1^{-1} = [0 0 -0.5; 0 1 0; -0.5 0 0]
    Mat33 C1inv;
    C1inv(0, 0) = 0;    C1inv(0, 1) = 0;  C1inv(0, 2) = -0.5;
    C1inv(1, 0) = 0;    C1inv(1, 1) = 1;  C1inv(1, 2) = 0;
    C1inv(2, 0) = -0.5; C1inv(2, 1) = 0;  C1inv(2, 2) = 0;

    Mat33 MC = C1inv * M;

    // Find eigenvalues using characteristic polynomial
    // For 3x3 matrix, use analytic formula
    double a11 = MC(0, 0), a12 = MC(0, 1), a13 = MC(0, 2);
    double a21 = MC(1, 0), a22 = MC(1, 1), a23 = MC(1, 2);
    double a31 = MC(2, 0), a32 = MC(2, 1), a33 = MC(2, 2);

    double tr = a11 + a22 + a33;
    double q = tr / 3.0;

    double p1 = (a11 - q) * (a11 - q) + (a22 - q) * (a22 - q) + (a33 - q) * (a33 - q);
    p1 += 2.0 * (a12 * a12 + a13 * a13 + a23 * a23);
    double p = std::sqrt(p1 / 6.0);

    if (p < 1e-15) {
        return result;
    }

    Mat33 B;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            B(i, j) = (MC(i, j) - (i == j ? q : 0)) / p;
        }
    }

    double detB = B.Determinant();
    double r = detB / 2.0;
    r = std::max(-1.0, std::min(1.0, r));
    double phi = std::acos(r) / 3.0;

    // Eigenvalues
    double eig1 = q + 2.0 * p * std::cos(phi);
    double eig2 = q + 2.0 * p * std::cos(phi + 2.0 * M_PI / 3.0);
    double eig3 = q + 2.0 * p * std::cos(phi + 4.0 * M_PI / 3.0);

    // Find the smallest positive eigenvalue (for ellipse constraint)
    double targetEig = std::numeric_limits<double>::max();
    if (eig1 > 0 && eig1 < targetEig) targetEig = eig1;
    if (eig2 > 0 && eig2 < targetEig) targetEig = eig2;
    if (eig3 > 0 && eig3 < targetEig) targetEig = eig3;

    if (targetEig == std::numeric_limits<double>::max()) {
        return result;
    }

    // Find eigenvector for targetEig
    // (MC - targetEig * I) * v = 0
    Mat33 A = MC;
    A(0, 0) -= targetEig;
    A(1, 1) -= targetEig;
    A(2, 2) -= targetEig;

    // Use SVD to find null space
    // Simple approach: find row with smallest norm, compute cross product of other two
    Vec3 r0{A(0, 0), A(0, 1), A(0, 2)};
    Vec3 r1{A(1, 0), A(1, 1), A(1, 2)};
    Vec3 r2{A(2, 0), A(2, 1), A(2, 2)};

    double n0 = r0.NormSquared();
    double n1 = r1.NormSquared();
    double n2 = r2.NormSquared();

    Vec3 a1;
    if (n0 <= n1 && n0 <= n2) {
        a1 = Cross(r1, r2);
    } else if (n1 <= n0 && n1 <= n2) {
        a1 = Cross(r0, r2);
    } else {
        a1 = Cross(r0, r1);
    }

    double norm = a1.Norm();
    if (norm < 1e-15) {
        return result;
    }
    a1 = a1 / norm;

    // Compute a2 = -S3^{-1} * S2^T * a1
    Vec3 a2;
    for (int i = 0; i < 3; ++i) {
        a2[i] = 0;
        for (int j = 0; j < 3; ++j) {
            a2[i] -= temp(i, j) * a1[j];
        }
    }

    // Full conic coefficients [A, B, C, D, E, F]
    double A_coef = a1[0];
    double B_coef = a1[1];
    double C_coef = a1[2];
    double D_coef = a2[0];
    double E_coef = a2[1];
    double F_coef = a2[2];

    // Extract ellipse parameters from conic Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0
    // Check discriminant: B^2 - 4AC < 0 for ellipse
    double disc = B_coef * B_coef - 4.0 * A_coef * C_coef;
    if (disc >= 0) {
        return result;  // Not an ellipse
    }

    // Center: solve dF/dx = 0, dF/dy = 0
    // 2Ax + By + D = 0
    // Bx + 2Cy + E = 0
    double det = 4.0 * A_coef * C_coef - B_coef * B_coef;
    double cx = (B_coef * E_coef - 2.0 * C_coef * D_coef) / det;
    double cy = (B_coef * D_coef - 2.0 * A_coef * E_coef) / det;

    // Value at center
    double F_center = A_coef * cx * cx + B_coef * cx * cy + C_coef * cy * cy
                     + D_coef * cx + E_coef * cy + F_coef;

    // Semi-axes and angle
    // Principal axis angle: tan(2*theta) = B / (A - C)
    double theta = 0.5 * std::atan2(B_coef, A_coef - C_coef);

    double cosT = std::cos(theta);
    double sinT = std::sin(theta);

    // Rotated coefficients
    double Ap = A_coef * cosT * cosT + B_coef * cosT * sinT + C_coef * sinT * sinT;
    double Cp = A_coef * sinT * sinT - B_coef * cosT * sinT + C_coef * cosT * cosT;

    if (Ap * F_center >= 0 || Cp * F_center >= 0) {
        return result;  // Invalid ellipse
    }

    double a = std::sqrt(-F_center / Ap);  // semi-axis along rotated x
    double b = std::sqrt(-F_center / Cp);  // semi-axis along rotated y

    // Ensure a >= b (a is semi-major)
    if (a < b) {
        std::swap(a, b);
        theta += M_PI / 2.0;
    }

    // Normalize angle to [-pi/2, pi/2]
    while (theta > M_PI / 2.0) theta -= M_PI;
    while (theta < -M_PI / 2.0) theta += M_PI;

    // Denormalize
    Ellipse2d normEllipse(Point2d(cx, cy), a, b, theta);
    result.ellipse = DenormalizeEllipse(normEllipse, T);
    result.success = true;

    auto residuals = ComputeEllipseResiduals(points, result.ellipse);
    FillResidualStats(result, residuals, params);
    result.numInliers = result.numPoints;

    return result;
}

EllipseFitResult FitEllipseGeometric(const std::vector<Point2d>& points,
                                      const GeometricFitParams& geoParams,
                                      const FitParams& params) {
    EllipseFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < ELLIPSE_FIT_MIN_POINTS) {
        return result;
    }

    // Initial estimate from Fitzgibbon
    auto fitzResult = FitEllipseFitzgibbon(points, FitParams());
    if (!fitzResult.success) {
        return result;
    }

    // For now, just return Fitzgibbon result
    // Full geometric fitting would require Levenberg-Marquardt iteration
    result = fitzResult;

    auto residuals = ComputeEllipseResiduals(points, result.ellipse);
    FillResidualStats(result, residuals, params);

    return result;
}

EllipseFitResult FitEllipse(const std::vector<Point2d>& points,
                             EllipseFitMethod method,
                             const FitParams& params) {
    switch (method) {
        case EllipseFitMethod::Fitzgibbon:
            return FitEllipseFitzgibbon(points, params);
        case EllipseFitMethod::Geometric:
            return FitEllipseGeometric(points, GeometricFitParams(), params);
        case EllipseFitMethod::RANSAC:
            return FitEllipseRANSAC(points, RansacParams(), params);
        default:
            return FitEllipseFitzgibbon(points, params);
    }
}

EllipseFitResult FitEllipseRANSAC(const std::vector<Point2d>& points,
                                   const RansacParams& ransacParams,
                                   const FitParams& params) {
    EllipseFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < ELLIPSE_FIT_MIN_POINTS) {
        return result;
    }

    RansacModel<Ellipse2d> ellipseModel;
    ellipseModel.minSampleSize = 5;

    ellipseModel.fitMinimal = [](const std::vector<Point2d>& pts) -> std::optional<Ellipse2d> {
        if (pts.size() < 5) return std::nullopt;
        auto res = FitEllipseFitzgibbon(pts);
        if (!res.success) return std::nullopt;
        return res.ellipse;
    };

    ellipseModel.fitAll = [](const std::vector<Point2d>& pts) -> std::optional<Ellipse2d> {
        if (pts.size() < 5) return std::nullopt;
        auto res = FitEllipseFitzgibbon(pts);
        if (!res.success) return std::nullopt;
        return res.ellipse;
    };

    ellipseModel.distance = [](const Ellipse2d& e, const Point2d& p) -> double {
        double cosA = std::cos(-e.angle);
        double sinA = std::sin(-e.angle);
        double dx = p.x - e.center.x;
        double dy = p.y - e.center.y;
        double x = dx * cosA - dy * sinA;
        double y = dx * sinA + dy * cosA;

        double a2 = e.a * e.a;
        double b2 = e.b * e.b;
        double val = (x * x) / a2 + (y * y) / b2 - 1.0;

        return val * std::sqrt(a2 * b2) / std::sqrt(b2 * x * x + a2 * y * y + 1e-15);
    };

    auto ransacResult = RANSAC<Ellipse2d>(points, ellipseModel, ransacParams);

    if (!ransacResult.success) return result;

    result.ellipse = ransacResult.model;
    result.success = true;
    result.numInliers = ransacResult.numInliers;

    if (params.computeInlierMask) {
        result.inlierMask = ransacResult.inlierMask;
    }

    auto residuals = ComputeEllipseResiduals(points, result.ellipse);
    FillResidualStats(result, residuals, params);

    return result;
}

// =============================================================================
// Robust Ellipse Fitting (IRLS)
// =============================================================================

// Helper: Weighted Ellipse Fitzgibbon fitting
static EllipseFitResult FitEllipseWeightedInternal(const std::vector<Point2d>& points,
                                                    const std::vector<double>& weights,
                                                    const FitParams& params) {
    EllipseFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < ELLIPSE_FIT_MIN_POINTS) {
        return result;
    }

    // Compute weighted centroid for normalization
    double sumW = 0.0;
    double cx = 0.0, cy = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
        double w = weights[i];
        sumW += w;
        cx += w * points[i].x;
        cy += w * points[i].y;
    }
    if (sumW < 1e-15) {
        return result;
    }
    cx /= sumW;
    cy /= sumW;

    // Build weighted design matrix D and constraint matrix C
    // Fitzgibbon: minimize x^T D^T D x subject to x^T C x = 1
    // where D is [x^2, xy, y^2, x, y, 1] and C enforces 4ac - b^2 = 1
    Mat<6, 6> S;  // Scatter matrix
    S.Zero();

    for (size_t i = 0; i < points.size(); ++i) {
        double w = weights[i];
        if (w < 1e-10) continue;

        double sqrtW = std::sqrt(w);
        double x = points[i].x - cx;
        double y = points[i].y - cy;

        // Design vector [x^2, xy, y^2, x, y, 1]
        Vec<6> d;
        d[0] = x * x;
        d[1] = x * y;
        d[2] = y * y;
        d[3] = x;
        d[4] = y;
        d[5] = 1.0;

        // Add weighted outer product to scatter matrix
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < 6; ++c) {
                S(r, c) += w * d[r] * d[c];
            }
        }
    }

    // Partition S into blocks: S = [S1 S2; S2^T S3]
    // S1: 3x3 (quadratic terms), S3: 3x3 (linear terms)
    Mat<3, 3> S1, S2, S3;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            S1(r, c) = S(r, c);
            S2(r, c) = S(r, c + 3);
            S3(r, c) = S(r + 3, c + 3);
        }
    }

    // Constraint matrix C1 = [0 0 2; 0 -1 0; 2 0 0]
    Mat<3, 3> C1;
    C1.Zero();
    C1(0, 2) = 2.0;
    C1(1, 1) = -1.0;
    C1(2, 0) = 2.0;

    // Solve generalized eigenvalue problem: S1 a1 = lambda C1 a1
    // where a1 relates to S3 via: a2 = -S3^-1 S2^T a1

    // Check if S3 is invertible by computing determinant
    double det3 = S3(0,0) * (S3(1,1)*S3(2,2) - S3(1,2)*S3(2,1))
                - S3(0,1) * (S3(1,0)*S3(2,2) - S3(1,2)*S3(2,0))
                + S3(0,2) * (S3(1,0)*S3(2,1) - S3(1,1)*S3(2,0));
    if (std::abs(det3) < 1e-15) {
        // Fallback to standard fitting
        return FitEllipseFitzgibbon(points, params);
    }

    Mat<3, 3> S3inv = S3.Inverse();
    Mat<3, 3> T = S1 - S2 * S3inv * S2.Transpose();

    // Solve T a1 = lambda C1 a1
    // Equivalent to C1^-1 T a1 = lambda a1
    Mat<3, 3> C1inv;
    C1inv.Zero();
    C1inv(0, 2) = 0.5;
    C1inv(1, 1) = -1.0;
    C1inv(2, 0) = 0.5;

    Mat<3, 3> M = C1inv * T;

    // Find eigenvalues and eigenvectors using EigenSymmetric3x3
    // Convert Mat<3,3> to Mat33 (same type)
    auto eigenResult = EigenSymmetric3x3(M);
    if (!eigenResult.valid) {
        return FitEllipseFitzgibbon(points, params);
    }

    // Find eigenvector with positive eigenvalue satisfying ellipse constraint
    // Eigenvectors: v1, v2, v3 with eigenvalues lambda1, lambda2, lambda3
    Vec<3> a1;
    bool found = false;

    // Check each eigenvector
    Vec<3> eigenvectors[3] = {eigenResult.v1, eigenResult.v2, eigenResult.v3};
    for (int i = 0; i < 3; ++i) {
        Vec<3> v = eigenvectors[i];

        // Check constraint: 4*a*c - b^2 > 0 (ellipse condition)
        double cond = 4.0 * v[0] * v[2] - v[1] * v[1];
        if (cond > 0) {
            a1 = v;
            found = true;
            break;
        }
    }

    if (!found) {
        return FitEllipseFitzgibbon(points, params);
    }

    // Compute a2 = -S3^-1 S2^T a1
    Vec<3> a2 = -(S3inv * S2.Transpose()) * a1;

    // Full coefficients [A, B, C, D, E, F] for Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0
    double A = a1[0];
    double B = a1[1];
    double C = a1[2];
    double D = a2[0];
    double E = a2[1];
    double F = a2[2];

    // Transform back from centered coordinates
    // Original equation in (x-cx, y-cy): A(x-cx)^2 + B(x-cx)(y-cy) + C(y-cy)^2 + D(x-cx) + E(y-cy) + F = 0
    // Expand and collect terms
    double A_orig = A;
    double B_orig = B;
    double C_orig = C;
    double D_orig = -2.0 * A * cx - B * cy + D;
    double E_orig = -2.0 * C * cy - B * cx + E;
    double F_orig = A * cx * cx + B * cx * cy + C * cy * cy - D * cx - E * cy + F;

    // Convert to standard ellipse parameters
    // Center: solve gradient = 0
    double denom = 4.0 * A_orig * C_orig - B_orig * B_orig;
    if (std::abs(denom) < 1e-15) {
        return FitEllipseFitzgibbon(points, params);
    }

    double ecx = (B_orig * E_orig - 2.0 * C_orig * D_orig) / denom;
    double ecy = (B_orig * D_orig - 2.0 * A_orig * E_orig) / denom;

    // Angle: theta = 0.5 * atan2(B, A - C)
    double theta = 0.5 * std::atan2(B_orig, A_orig - C_orig);

    // Semi-axes from eigenvalues of quadratic form
    double cosT = std::cos(theta);
    double sinT = std::sin(theta);

    double A_rot = A_orig * cosT * cosT + B_orig * cosT * sinT + C_orig * sinT * sinT;
    double C_rot = A_orig * sinT * sinT - B_orig * cosT * sinT + C_orig * cosT * cosT;

    // F at center
    double F_center = A_orig * ecx * ecx + B_orig * ecx * ecy + C_orig * ecy * ecy +
                      D_orig * ecx + E_orig * ecy + F_orig;

    if (F_center > 0 || A_rot < 1e-15 || C_rot < 1e-15) {
        return FitEllipseFitzgibbon(points, params);
    }

    double a = std::sqrt(-F_center / A_rot);
    double b = std::sqrt(-F_center / C_rot);

    // Ensure a >= b
    if (a < b) {
        std::swap(a, b);
        theta += M_PI / 2.0;
    }

    // Normalize angle to [-pi/2, pi/2]
    while (theta > M_PI / 2.0) theta -= M_PI;
    while (theta < -M_PI / 2.0) theta += M_PI;

    result.ellipse.center = {ecx, ecy};
    result.ellipse.a = a;
    result.ellipse.b = b;
    result.ellipse.angle = theta;
    result.success = true;

    auto residuals = ComputeEllipseResiduals(points, result.ellipse);
    FillResidualStats(result, residuals, params);

    return result;
}

EllipseFitResult FitEllipseHuber(const std::vector<Point2d>& points,
                                  double sigma,
                                  const FitParams& params) {
    EllipseFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < ELLIPSE_FIT_MIN_POINTS) {
        return result;
    }

    // Initial fit using Fitzgibbon
    result = FitEllipseFitzgibbon(points, params);
    if (!result.success) return result;

    const int maxIter = 20;
    const double tol = 1e-6;
    std::vector<double> weights(points.size(), 1.0);

    for (int iter = 0; iter < maxIter; ++iter) {
        auto residuals = ComputeEllipseResiduals(points, result.ellipse);

        double scale = sigma;
        if (scale <= 0) {
            scale = RobustScaleMAD(residuals);
            if (scale < 1e-10) scale = 1.0;
        }

        double maxWeightChange = 0.0;
        for (size_t i = 0; i < points.size(); ++i) {
            double r = residuals[i] / scale;
            double newWeight = HuberWeight(r);
            maxWeightChange = std::max(maxWeightChange, std::abs(newWeight - weights[i]));
            weights[i] = newWeight;
        }

        auto newResult = FitEllipseWeightedInternal(points, weights, params);
        if (!newResult.success) {
            // Keep previous result
            break;
        }
        result = newResult;

        if (maxWeightChange < tol) break;
    }

    auto residuals = ComputeEllipseResiduals(points, result.ellipse);
    FillResidualStats(result, residuals, params);
    result.weights = weights;
    result.numInliers = result.numPoints;

    return result;
}

EllipseFitResult FitEllipseTukey(const std::vector<Point2d>& points,
                                  double sigma,
                                  const FitParams& params) {
    EllipseFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < ELLIPSE_FIT_MIN_POINTS) {
        return result;
    }

    // Initial fit using Fitzgibbon
    result = FitEllipseFitzgibbon(points, params);
    if (!result.success) return result;

    const int maxIter = 20;
    const double tol = 1e-6;
    std::vector<double> weights(points.size(), 1.0);

    for (int iter = 0; iter < maxIter; ++iter) {
        auto residuals = ComputeEllipseResiduals(points, result.ellipse);

        double scale = sigma;
        if (scale <= 0) {
            scale = RobustScaleMAD(residuals);
            if (scale < 1e-10) scale = 1.0;
        }

        double maxWeightChange = 0.0;
        int numNonZero = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            double r = residuals[i] / scale;
            double newWeight = TukeyWeight(r);
            maxWeightChange = std::max(maxWeightChange, std::abs(newWeight - weights[i]));
            weights[i] = newWeight;
            if (newWeight > 0) ++numNonZero;
        }

        if (numNonZero < ELLIPSE_FIT_MIN_POINTS) break;

        auto newResult = FitEllipseWeightedInternal(points, weights, params);
        if (!newResult.success) {
            break;
        }
        result = newResult;

        if (maxWeightChange < tol) break;
    }

    auto residuals = ComputeEllipseResiduals(points, result.ellipse);
    FillResidualStats(result, residuals, params);
    result.weights = weights;

    double scale = RobustScaleMAD(residuals);
    if (scale < 1e-10) scale = 1.0;
    result.numInliers = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        if (TukeyWeight(residuals[i] / scale) > 0) ++result.numInliers;
    }

    return result;
}

// =============================================================================
// Rectangle Fitting
// =============================================================================

std::array<std::vector<Point2d>, 4> SegmentPointsByRectangleSide(
    const std::vector<Point2d>& points,
    const RotatedRect2d& rect) {

    std::array<std::vector<Point2d>, 4> segments;  // top, right, bottom, left

    double cosPhi = std::cos(rect.angle);
    double sinPhi = std::sin(rect.angle);

    // Half-dimensions
    double halfW = rect.width / 2.0;   // along major axis (phi direction)
    double halfH = rect.height / 2.0;  // perpendicular to phi

    // Rectangle corners in local frame: (+halfW, +halfH), (+halfW, -halfH), etc.
    // Sides: top (y=+halfH), right (x=+halfW), bottom (y=-halfH), left (x=-halfW)

    for (const auto& p : points) {
        // Transform to local (rectangle-aligned) coordinates
        double dx = p.x - rect.center.x;
        double dy = p.y - rect.center.y;
        double localX = dx * cosPhi + dy * sinPhi;   // along phi
        double localY = -dx * sinPhi + dy * cosPhi;  // perpendicular to phi

        // Compute distance to each side
        double distTop = std::abs(localY - halfH);
        double distBottom = std::abs(localY + halfH);
        double distRight = std::abs(localX - halfW);
        double distLeft = std::abs(localX + halfW);

        // Also check if point is within the side's extent
        bool withinHorizontal = (localX >= -halfW - halfW * 0.5) && (localX <= halfW + halfW * 0.5);
        bool withinVertical = (localY >= -halfH - halfH * 0.5) && (localY <= halfH + halfH * 0.5);

        // Find minimum distance and assign to corresponding side
        double minDist = std::min({distTop, distBottom, distRight, distLeft});

        if (minDist == distTop && withinHorizontal) {
            segments[0].push_back(p);  // top
        } else if (minDist == distRight && withinVertical) {
            segments[1].push_back(p);  // right
        } else if (minDist == distBottom && withinHorizontal) {
            segments[2].push_back(p);  // bottom
        } else if (minDist == distLeft && withinVertical) {
            segments[3].push_back(p);  // left
        } else {
            // Point is near a corner - assign to nearest side
            if (minDist == distTop) segments[0].push_back(p);
            else if (minDist == distRight) segments[1].push_back(p);
            else if (minDist == distBottom) segments[2].push_back(p);
            else segments[3].push_back(p);
        }
    }

    return segments;
}

std::optional<RotatedRect2d> RectangleFromLines(const std::array<Line2d, 4>& lines) {
    // lines: [top, right, bottom, left]
    // Intersections: top-right, right-bottom, bottom-left, left-top

    auto intersect = [](const Line2d& l1, const Line2d& l2) -> std::optional<Point2d> {
        // l1: a1*x + b1*y + c1 = 0
        // l2: a2*x + b2*y + c2 = 0
        double det = l1.a * l2.b - l2.a * l1.b;
        if (std::abs(det) < 1e-10) return std::nullopt;

        double x = (l1.b * l2.c - l2.b * l1.c) / det;
        double y = (l2.a * l1.c - l1.a * l2.c) / det;
        return Point2d{x, y};
    };

    // Four corners
    auto p0 = intersect(lines[0], lines[1]);  // top-right
    auto p1 = intersect(lines[1], lines[2]);  // right-bottom
    auto p2 = intersect(lines[2], lines[3]);  // bottom-left
    auto p3 = intersect(lines[3], lines[0]);  // left-top

    if (!p0 || !p1 || !p2 || !p3) {
        return std::nullopt;
    }

    // Center is average of diagonals' midpoints
    Point2d center;
    center.x = (p0->x + p1->x + p2->x + p3->x) / 4.0;
    center.y = (p0->y + p1->y + p2->y + p3->y) / 4.0;

    // Compute side lengths
    double side01 = std::hypot(p1->x - p0->x, p1->y - p0->y);  // right side
    double side12 = std::hypot(p2->x - p1->x, p2->y - p1->y);  // bottom side
    double side23 = std::hypot(p3->x - p2->x, p3->y - p2->y);  // left side
    double side30 = std::hypot(p0->x - p3->x, p0->y - p3->y);  // top side

    // Average width and height
    double width = (side30 + side12) / 2.0;   // top and bottom (along phi)
    double height = (side01 + side23) / 2.0;  // right and left (perpendicular)

    // Angle from top line (line[0])
    // Line normal is (a, b), line direction is (-b, a)
    double angle = std::atan2(-lines[0].b, lines[0].a);  // direction of top line

    RotatedRect2d result;
    result.center = center;
    result.width = width;
    result.height = height;
    result.angle = angle;

    return result;
}

RectangleFitResult FitRectangle(const std::vector<Point2d>& points,
                                 const RotatedRect2d& initialRect,
                                 FitMethod method,
                                 const FitParams& params) {
    RectangleFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < 4) {
        return result;
    }

    // Segment points by side
    auto segments = SegmentPointsByRectangleSide(points, initialRect);

    // Check minimum points per side
    for (int i = 0; i < 4; ++i) {
        if (segments[i].size() < 2) {
            // Not enough points on this side - use initial estimate
            result.rect = initialRect;
            result.success = true;
            result.numInliers = static_cast<int>(points.size());
            return result;
        }
    }

    // Fit lines to each side
    std::array<Line2d, 4> lines;
    for (int i = 0; i < 4; ++i) {
        LineFitResult lineResult;

        if (method == FitMethod::Huber) {
            lineResult = FitLineHuber(segments[i], 0.0, params);
        } else if (method == FitMethod::Tukey) {
            lineResult = FitLineTukey(segments[i], 0.0, params);
        } else {
            lineResult = FitLine(segments[i], params);
        }

        if (!lineResult.success) {
            // Fallback to basic fitting
            lineResult = FitLine(segments[i], params);
        }

        if (!lineResult.success) {
            result.rect = initialRect;
            result.success = true;
            result.numInliers = static_cast<int>(points.size());
            return result;
        }

        lines[i] = lineResult.line;
        result.sideResults[i] = lineResult;
    }

    // Compute rectangle from lines
    auto rectOpt = RectangleFromLines(lines);
    if (!rectOpt) {
        result.rect = initialRect;
        result.success = true;
        result.numInliers = static_cast<int>(points.size());
        return result;
    }

    result.rect = rectOpt.value();
    result.success = true;

    // Compute residuals (distance to nearest side)
    std::vector<double> residuals(points.size());
    double cosPhi = std::cos(result.rect.angle);
    double sinPhi = std::sin(result.rect.angle);
    double halfW = result.rect.width / 2.0;
    double halfH = result.rect.height / 2.0;

    for (size_t i = 0; i < points.size(); ++i) {
        double dx = points[i].x - result.rect.center.x;
        double dy = points[i].y - result.rect.center.y;
        double localX = dx * cosPhi + dy * sinPhi;
        double localY = -dx * sinPhi + dy * cosPhi;

        double distX = std::max(0.0, std::abs(localX) - halfW);
        double distY = std::max(0.0, std::abs(localY) - halfH);
        residuals[i] = std::sqrt(distX * distX + distY * distY);
    }

    FillResidualStats(result, residuals, params);
    result.numInliers = result.numPoints;

    return result;
}

RectangleFitResult FitRectangleIterative(const std::vector<Point2d>& points,
                                          const RotatedRect2d& initialRect,
                                          int maxIterations,
                                          double convergenceThreshold,
                                          const FitParams& params) {
    RectangleFitResult result;
    result.success = false;
    result.numPoints = static_cast<int>(points.size());

    if (points.size() < 4) {
        return result;
    }

    RotatedRect2d currentRect = initialRect;

    for (int iter = 0; iter < maxIterations; ++iter) {
        result = FitRectangle(points, currentRect, FitMethod::Huber, params);
        if (!result.success) {
            break;
        }

        // Check convergence
        double dx = result.rect.center.x - currentRect.center.x;
        double dy = result.rect.center.y - currentRect.center.y;
        double dw = result.rect.width - currentRect.width;
        double dh = result.rect.height - currentRect.height;
        double da = result.rect.angle - currentRect.angle;

        double change = std::sqrt(dx*dx + dy*dy + dw*dw + dh*dh + da*da * 100.0);

        if (change < convergenceThreshold) {
            break;
        }

        currentRect = result.rect;
    }

    return result;
}

} // namespace Qi::Vision::Internal
