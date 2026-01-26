/**
 * @file ShapeModelCreate.cpp
 * @brief Model creation functions for ShapeModel
 *
 * Contains:
 * - CreateModel
 * - ExtractModelPoints
 * - OptimizeModel
 * - BuildCosLUT
 * - BuildAngleCache
 * - ComputeModelBounds
 * - ComputeRotatedBounds
 * - LevelModel methods
 */

#include "ShapeModelImpl.h"
#include <QiVision/Internal/Pyramid.h>
#include <QiVision/Internal/Canny.h>
#include <QiVision/Internal/ContourProcess.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <queue>
#include <unordered_map>

namespace Qi::Vision::Matching {

// =============================================================================
// XLD Contour Tracing Helpers (Halcon-style)
// =============================================================================

namespace {

double GetResampleSpacing(OptimizationMode mode) {
    switch (mode) {
        case OptimizationMode::PointReductionHigh:
            return 1.5;
        case OptimizationMode::PointReductionMedium:
            return 1.3;
        case OptimizationMode::PointReductionLow:
            return 1.1;
        case OptimizationMode::None:
        case OptimizationMode::Auto:
        default:
            return 1.0;
    }
}

/// Contour segment for XLD processing
struct XLDContourSegment {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> angles;      // Gradient direction at each point
    std::vector<double> magnitudes;  // Gradient magnitude at each point
    std::vector<int32_t> angleBins;  // Quantized angle bins
    bool isClosed = false;

    XLDContourSegment() = default;
    XLDContourSegment(const XLDContourSegment& other)
        : x(other.x),
          y(other.y),
          angles(other.angles),
          magnitudes(other.magnitudes),
          angleBins(other.angleBins),
          isClosed(other.isClosed) {}
    XLDContourSegment& operator=(const XLDContourSegment& other) {
        if (this != &other) {
            x = other.x;
            y = other.y;
            angles = other.angles;
            magnitudes = other.magnitudes;
            angleBins = other.angleBins;
            isClosed = other.isClosed;
        }
        return *this;
    }
    XLDContourSegment(XLDContourSegment&& other) noexcept = default;
    XLDContourSegment& operator=(XLDContourSegment&& other) noexcept = default;

    size_t Size() const { return x.size(); }
    bool Empty() const { return x.empty(); }

    double ComputeLength() const {
        if (Size() < 2) return 0.0;
        double len = 0.0;
        for (size_t i = 1; i < Size(); ++i) {
            double dx = x[i] - x[i-1];
            double dy = y[i] - y[i-1];
            len += std::sqrt(dx*dx + dy*dy);
        }
        return len;
    }

    void Reserve(size_t n) {
        x.reserve(n);
        y.reserve(n);
        angles.reserve(n);
        magnitudes.reserve(n);
        angleBins.reserve(n);
    }

    void PushBack(double px, double py, double ang, double mag, int32_t bin) {
        x.push_back(px);
        y.push_back(py);
        angles.push_back(ang);
        magnitudes.push_back(mag);
        angleBins.push_back(bin);
    }
};

// Intentionally keep all contours by default to preserve inner geometry.

/// Interpolate angle using shortest path
inline double InterpolateAngleXLD(double a1, double a2, double t) {
    double diff = a2 - a1;
    if (diff > PI) diff -= 2.0 * PI;
    if (diff < -PI) diff += 2.0 * PI;
    double result = a1 + t * diff;
    if (result < 0) result += 2.0 * PI;
    if (result >= 2.0 * PI) result -= 2.0 * PI;
    return result;
}

/**
 * @brief Filter edge points by connected component size (8-connectivity)
 *
 * HALCON-style min_size filtering: removes small isolated edge groups.
 * The minSize parameter is scaled per pyramid level (divided by 2 each level).
 *
 * @param edgePoints Edge points to filter (post-hysteresis)
 * @param width Image width at this level
 * @param height Image height at this level
 * @param minSize Minimum component size (point count)
 * @return Filtered edge points
 */
std::vector<Qi::Vision::Internal::EdgePoint> FilterByComponentSize(
    const std::vector<Qi::Vision::Internal::EdgePoint>& edgePoints,
    int32_t width, int32_t height,
    int32_t minSize)
{
    if (edgePoints.empty() || minSize <= 1) {
        return edgePoints;
    }

    // Build spatial hash for fast neighbor lookup
    std::unordered_map<int64_t, std::vector<size_t>> pixelMap;
    auto toKey = [width](int32_t x, int32_t y) -> int64_t {
        return static_cast<int64_t>(y) * width + x;
    };

    for (size_t i = 0; i < edgePoints.size(); ++i) {
        int32_t px = static_cast<int32_t>(std::round(edgePoints[i].x));
        int32_t py = static_cast<int32_t>(std::round(edgePoints[i].y));
        if (px >= 0 && px < width && py >= 0 && py < height) {
            pixelMap[toKey(px, py)].push_back(i);
        }
    }

    // Union-Find for connected components
    std::vector<int32_t> parent(edgePoints.size());
    std::vector<int32_t> rank(edgePoints.size(), 0);
    for (size_t i = 0; i < edgePoints.size(); ++i) {
        parent[i] = static_cast<int32_t>(i);
    }

    std::function<int32_t(int32_t)> find = [&](int32_t x) -> int32_t {
        if (parent[x] != x) {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    };

    auto unite = [&](int32_t x, int32_t y) {
        int32_t px = find(x);
        int32_t py = find(y);
        if (px == py) return;
        if (rank[px] < rank[py]) std::swap(px, py);
        parent[py] = px;
        if (rank[px] == rank[py]) rank[px]++;
    };

    // 8-neighbor offsets
    static const int32_t dx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    static const int32_t dy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    // Connect 8-neighbors
    for (size_t i = 0; i < edgePoints.size(); ++i) {
        int32_t px = static_cast<int32_t>(std::round(edgePoints[i].x));
        int32_t py = static_cast<int32_t>(std::round(edgePoints[i].y));

        for (int32_t d = 0; d < 8; ++d) {
            int32_t nx = px + dx8[d];
            int32_t ny = py + dy8[d];
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

            auto it = pixelMap.find(toKey(nx, ny));
            if (it != pixelMap.end()) {
                for (size_t j : it->second) {
                    unite(static_cast<int32_t>(i), static_cast<int32_t>(j));
                }
            }
        }
    }

    // Count component sizes
    std::unordered_map<int32_t, int32_t> componentSize;
    for (size_t i = 0; i < edgePoints.size(); ++i) {
        componentSize[find(static_cast<int32_t>(i))]++;
    }

    // Filter by size
    std::vector<Qi::Vision::Internal::EdgePoint> result;
    result.reserve(edgePoints.size());
    for (size_t i = 0; i < edgePoints.size(); ++i) {
        int32_t root = find(static_cast<int32_t>(i));
        if (componentSize[root] >= minSize) {
            result.push_back(edgePoints[i]);
        }
    }

    return result;
}

/**
 * @brief Trace edge points into ordered contour segments using 8-neighbor connectivity
 *
 * Halcon-style contour tracing: follows edge direction (perpendicular to gradient)
 *
 * @param edgePoints Filtered edge points from pyramid
 * @param width Image width at this level
 * @param height Image height at this level
 * @param minContourPoints Minimum points per contour (filter short segments)
 * @return Vector of ordered contour segments
 */
std::vector<XLDContourSegment> TraceContoursXLD(
    const std::vector<Qi::Vision::Internal::EdgePoint>& edgePoints,
    int32_t width, int32_t height,
    int32_t minContourPoints = 4)
{
    if (edgePoints.empty()) return {};

    // Build edge map: pixel coord -> edge point index
    std::unordered_map<int64_t, int32_t> edgeMap;
    edgeMap.reserve(edgePoints.size());

    for (size_t i = 0; i < edgePoints.size(); ++i) {
        int32_t px = static_cast<int32_t>(std::round(edgePoints[i].x));
        int32_t py = static_cast<int32_t>(std::round(edgePoints[i].y));
        if (px >= 0 && px < width && py >= 0 && py < height) {
            int64_t key = static_cast<int64_t>(py) * width + px;
            // If multiple points at same pixel, keep strongest
            auto it = edgeMap.find(key);
            if (it == edgeMap.end() || edgePoints[i].magnitude > edgePoints[it->second].magnitude) {
                edgeMap[key] = static_cast<int32_t>(i);
            }
        }
    }

    // 8-neighbor offsets (ordered for smooth traversal)
    static const int32_t dx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    static const int32_t dy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    constexpr double MAX_DIR_DIFF = PI * 0.17;  // ~30 degrees
    constexpr double MIN_MAG_RATIO = 0.4;       // Neighbor must be reasonably strong

    std::vector<bool> visited(edgePoints.size(), false);
    std::vector<XLDContourSegment> contours;

    // Sort by magnitude descending (start from strongest edges)
    std::vector<int32_t> sortedIndices(edgePoints.size());
    for (size_t i = 0; i < edgePoints.size(); ++i) {
        sortedIndices[i] = static_cast<int32_t>(i);
    }
    std::sort(sortedIndices.begin(), sortedIndices.end(),
        [&edgePoints](int32_t a, int32_t b) {
            return edgePoints[a].magnitude > edgePoints[b].magnitude;
        });

    for (int32_t seedIdx : sortedIndices) {
        if (visited[seedIdx]) continue;

        XLDContourSegment contour;
        contour.Reserve(100);

        // === Forward tracing ===
        int32_t currentIdx = seedIdx;
        while (currentIdx >= 0 && !visited[currentIdx]) {
            visited[currentIdx] = true;
            const auto& ep = edgePoints[currentIdx];
            contour.PushBack(ep.x, ep.y, ep.angle, ep.magnitude, ep.angleBin);

            // Find best next neighbor (along edge direction)
            int32_t px = static_cast<int32_t>(std::round(ep.x));
            int32_t py = static_cast<int32_t>(std::round(ep.y));

            // Edge direction = perpendicular to gradient
            double edgeDir = ep.angle + PI * 0.5;

            int32_t nextIdx = -1;
            double bestScore = -1.0;

            for (int32_t d = 0; d < 8; ++d) {
                int32_t nx = px + dx8[d];
                int32_t ny = py + dy8[d];

                if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

                int64_t key = static_cast<int64_t>(ny) * width + nx;
                auto it = edgeMap.find(key);
                if (it == edgeMap.end()) continue;

                int32_t neighborIdx = it->second;
                if (visited[neighborIdx]) continue;

                // Score: direction consistency + magnitude
                double neighborEdgeDir = edgePoints[neighborIdx].angle + PI * 0.5;
                double dirDiff = std::abs(edgeDir - neighborEdgeDir);
                if (dirDiff > PI) dirDiff = 2.0 * PI - dirDiff;

                if (dirDiff > MAX_DIR_DIFF) {
                    continue;
                }

                if (edgePoints[neighborIdx].magnitude < ep.magnitude * MIN_MAG_RATIO) {
                    continue;
                }

                double score = (1.0 - dirDiff / PI) + edgePoints[neighborIdx].magnitude * 0.001;

                if (score > bestScore) {
                    bestScore = score;
                    nextIdx = neighborIdx;
                }
            }

            currentIdx = nextIdx;
        }

        // === Backward tracing from seed ===
        std::vector<double> backX, backY, backAngles, backMags;
        std::vector<int32_t> backBins;

        currentIdx = seedIdx;
        while (true) {
            const auto& ep = edgePoints[currentIdx];
            int32_t px = static_cast<int32_t>(std::round(ep.x));
            int32_t py = static_cast<int32_t>(std::round(ep.y));

            // Opposite edge direction
            double edgeDir = ep.angle - PI * 0.5;

            int32_t prevIdx = -1;
            double bestScore = -1.0;

            for (int32_t d = 0; d < 8; ++d) {
                int32_t nx = px + dx8[d];
                int32_t ny = py + dy8[d];

                if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

                int64_t key = static_cast<int64_t>(ny) * width + nx;
                auto it = edgeMap.find(key);
                if (it == edgeMap.end()) continue;

                int32_t neighborIdx = it->second;
                if (visited[neighborIdx]) continue;

                double neighborEdgeDir = edgePoints[neighborIdx].angle - PI * 0.5;
                double dirDiff = std::abs(edgeDir - neighborEdgeDir);
                if (dirDiff > PI) dirDiff = 2.0 * PI - dirDiff;

                if (dirDiff > MAX_DIR_DIFF) {
                    continue;
                }

                if (edgePoints[neighborIdx].magnitude < ep.magnitude * MIN_MAG_RATIO) {
                    continue;
                }

                double score = (1.0 - dirDiff / PI) + edgePoints[neighborIdx].magnitude * 0.001;

                if (score > bestScore) {
                    bestScore = score;
                    prevIdx = neighborIdx;
                }
            }

            if (prevIdx < 0) break;

            visited[prevIdx] = true;
            const auto& prev = edgePoints[prevIdx];
            backX.push_back(prev.x);
            backY.push_back(prev.y);
            backAngles.push_back(prev.angle);
            backMags.push_back(prev.magnitude);
            backBins.push_back(prev.angleBin);

            currentIdx = prevIdx;
        }

        // Merge backward points (reversed) + forward points
        if (!backX.empty()) {
            XLDContourSegment merged;
            merged.Reserve(backX.size() + contour.Size());

            // Add backward in reverse
            for (size_t i = backX.size(); i > 0; --i) {
                merged.PushBack(backX[i-1], backY[i-1], backAngles[i-1], backMags[i-1], backBins[i-1]);
            }
            // Add forward
            for (size_t i = 0; i < contour.Size(); ++i) {
                merged.PushBack(contour.x[i], contour.y[i], contour.angles[i],
                               contour.magnitudes[i], contour.angleBins[i]);
            }

            contour = std::move(merged);
        }

        // Check closed contour
        if (contour.Size() >= 4) {
            double dx = contour.x.front() - contour.x.back();
            double dy = contour.y.front() - contour.y.back();
            if (dx*dx + dy*dy <= 2.25) {
                contour.isClosed = true;
            }
        }

        // Filter short contours
        if (static_cast<int32_t>(contour.Size()) >= minContourPoints) {
            contours.push_back(std::move(contour));
        }
    }

    return contours;
}

/**
 * @brief Resample contour along arc-length with constant spacing (Halcon-style)
 *
 * @param contour Input contour segment
 * @param spacing Target spacing between points (default 1.0 pixel)
 * @return Resampled contour segment
 */
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#endif
XLDContourSegment ResampleContourXLD(const XLDContourSegment& contour, double spacing = 1.0) {
    if (contour.Size() < 2) return contour;

    // Compute cumulative arc length
    std::vector<double> arcLen(contour.Size());
    arcLen[0] = 0.0;
    for (size_t i = 1; i < contour.Size(); ++i) {
        double dx = contour.x[i] - contour.x[i-1];
        double dy = contour.y[i] - contour.y[i-1];
        arcLen[i] = arcLen[i-1] + std::sqrt(dx*dx + dy*dy);
    }

    double totalLen = arcLen.back();
    if (totalLen < spacing) return contour;

    // Determine sample count
    int32_t numSamples = static_cast<int32_t>(std::floor(totalLen / spacing)) + 1;

    XLDContourSegment result;
    result.Reserve(numSamples);
    result.isClosed = contour.isClosed;

    size_t segIdx = 0;

    for (int32_t k = 0; k < numSamples; ++k) {
        double targetLen = k * spacing;

        // Find segment containing this arc length
        while (segIdx + 1 < contour.Size() && arcLen[segIdx + 1] < targetLen) {
            ++segIdx;
        }

        if (segIdx + 1 >= contour.Size()) {
            // At end
            result.PushBack(contour.x.back(), contour.y.back(),
                           contour.angles.back(), contour.magnitudes.back(), contour.angleBins.back());
        } else {
            // Interpolate within segment
            double segLen = arcLen[segIdx + 1] - arcLen[segIdx];
            double t = (segLen > 1e-10) ? (targetLen - arcLen[segIdx]) / segLen : 0.0;
            t = std::max(0.0, std::min(1.0, t));

            double x = contour.x[segIdx] + t * (contour.x[segIdx + 1] - contour.x[segIdx]);
            double y = contour.y[segIdx] + t * (contour.y[segIdx + 1] - contour.y[segIdx]);
            double mag = contour.magnitudes[segIdx] + t * (contour.magnitudes[segIdx + 1] - contour.magnitudes[segIdx]);
            double angle = InterpolateAngleXLD(contour.angles[segIdx], contour.angles[segIdx + 1], t);

            // Angle bin: use nearest
            int32_t bin = (t < 0.5) ? contour.angleBins[segIdx] : contour.angleBins[segIdx + 1];

            result.PushBack(x, y, angle, mag, bin);
        }
    }

    return result;
}
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

} // anonymous namespace

namespace Internal {

// =============================================================================
// LevelModel Implementation
// =============================================================================

void LevelModel::BuildSoA() {
    // Regenerate gridPoints from points if empty (e.g., after loading from file)
    if (gridPoints.empty() && !points.empty()) {
        RegenerateGridPoints();
    }

    // Build SoA for Block 1 (subpixel points)
    BuildSoAForPoints(points, soaX, soaY, soaCosAngle, soaSinAngle, soaWeight, soaAngleBin);

    // Build SoA for Block 2 (grid points)
    BuildSoAForPoints(gridPoints, gridSoaX, gridSoaY, gridSoaCosAngle, gridSoaSinAngle, gridSoaWeight, gridSoaAngleBin);
}

void LevelModel::RegenerateGridPoints() {
    std::set<std::pair<int32_t, int32_t>> uniqueGridCoords;
    gridPoints.clear();
    gridPoints.reserve(points.size());

    for (const auto& pt : points) {
        int32_t gx = static_cast<int32_t>(std::round(pt.x));
        int32_t gy = static_cast<int32_t>(std::round(pt.y));

        auto key = std::make_pair(gx, gy);
        if (uniqueGridCoords.find(key) == uniqueGridCoords.end()) {
            uniqueGridCoords.insert(key);
            gridPoints.emplace_back(static_cast<double>(gx), static_cast<double>(gy),
                                   pt.angle, pt.magnitude, pt.angleBin, pt.weight);
        }
    }

    // Sort by Y then X
    std::sort(gridPoints.begin(), gridPoints.end(),
        [](const ModelPoint& a, const ModelPoint& b) {
            if (static_cast<int32_t>(a.y) != static_cast<int32_t>(b.y))
                return a.y < b.y;
            return a.x < b.x;
        });
}

void LevelModel::BuildSoAForPoints(const std::vector<ModelPoint>& pts,
                                    std::vector<float>& x, std::vector<float>& y,
                                    std::vector<float>& cosA, std::vector<float>& sinA,
                                    std::vector<float>& w, std::vector<int16_t>& bins) {
    const size_t n = pts.size();
    const size_t paddedN = (n + 7) & ~7;  // Pad to multiple of 8 for AVX2

    x.resize(paddedN, 0.0f);
    y.resize(paddedN, 0.0f);
    cosA.resize(paddedN, 1.0f);
    sinA.resize(paddedN, 0.0f);
    w.resize(paddedN, 0.0f);
    bins.resize(paddedN, 0);

    for (size_t i = 0; i < n; ++i) {
        x[i] = static_cast<float>(pts[i].x);
        y[i] = static_cast<float>(pts[i].y);
        cosA[i] = static_cast<float>(pts[i].cosAngle);
        sinA[i] = static_cast<float>(pts[i].sinAngle);
        w[i] = static_cast<float>(pts[i].weight);
        bins[i] = static_cast<int16_t>(pts[i].angleBin);
    }
}

// =============================================================================
// Helpers for scaled model cache
// =============================================================================

static LevelModel ScaleLevelModel(const LevelModel& src, double scale) {
    LevelModel dst = src;
    dst.points.clear();
    dst.gridPoints.clear();

    dst.width = static_cast<int32_t>(std::round(src.width * scale));
    dst.height = static_cast<int32_t>(std::round(src.height * scale));
    dst.scale = src.scale * scale;

    dst.points.reserve(src.points.size());
    for (const auto& pt : src.points) {
        ModelPoint p = pt;
        p.x *= scale;
        p.y *= scale;
        dst.points.push_back(p);
    }

    // Re-generate grid points for scaled model (integer grid)
    dst.RegenerateGridPoints();
    dst.BuildSoA();
    return dst;
}

static void ComputeBoundsForLevels(const std::vector<LevelModel>& levels,
                                   double& minX, double& maxX,
                                   double& minY, double& maxY) {
    if (levels.empty() || levels[0].points.empty()) {
        minX = maxX = minY = maxY = 0.0;
        return;
    }

    minX = minY = std::numeric_limits<double>::max();
    maxX = maxY = std::numeric_limits<double>::lowest();

    for (const auto& pt : levels[0].points) {
        minX = std::min(minX, pt.x);
        maxX = std::max(maxX, pt.x);
        minY = std::min(minY, pt.y);
        maxY = std::max(maxY, pt.y);
    }
}

static double ComputeMinCoverageForLevels(const std::vector<LevelModel>& levels) {
    if (levels.empty() || levels[0].points.empty()) {
        return 0.7;
    }

    size_t numModelPoints = levels[0].points.size();
    if (numModelPoints < 200) {
        double val = 0.7 + 0.15 * std::max(0.0, (200.0 - static_cast<double>(numModelPoints)) / 150.0);
        return std::min(0.85, val);
    }
    return 0.7;
}

static void BuildSearchAngleCacheForLevels(const std::vector<LevelModel>& levels,
                                           const Size2i& templateSize,
                                           double angleStart, double angleExtent, double angleStep,
                                           std::vector<SearchAngleData>& outCache,
                                           double& outStep) {
    outCache.clear();

    // Auto-compute angle step if not specified (Halcon: AngleStep = atan(1/R_max))
    if (angleStep <= 0) {
        int32_t modelSize = std::max(templateSize.width, templateSize.height);
        angleStep = EstimateAngleStep(modelSize);
    }
    outStep = angleStep;

    int32_t numAngles = static_cast<int32_t>(std::ceil(angleExtent / angleStep)) + 1;
    outCache.resize(numAngles);

    const size_t numLevels = levels.size();

    for (int32_t i = 0; i < numAngles; ++i) {
        SearchAngleData& data = outCache[i];
        data.angle = angleStart + i * angleStep;
        data.cosA = static_cast<float>(std::cos(data.angle));
        data.sinA = static_cast<float>(std::sin(data.angle));

        data.levelBounds.resize(numLevels);
        for (size_t level = 0; level < numLevels; ++level) {
            const auto& levelModel = levels[level];
            if (levelModel.points.empty()) {
                data.levelBounds[level] = {0, 0, 0, 0};
                continue;
            }

            double minX = std::numeric_limits<double>::max();
            double maxX = std::numeric_limits<double>::lowest();
            double minY = std::numeric_limits<double>::max();
            double maxY = std::numeric_limits<double>::lowest();

            const double cosA = data.cosA;
            const double sinA = data.sinA;

            for (const auto& pt : levelModel.points) {
                double rx = cosA * pt.x - sinA * pt.y;
                double ry = sinA * pt.x + cosA * pt.y;
                minX = std::min(minX, rx);
                maxX = std::max(maxX, rx);
                minY = std::min(minY, ry);
                maxY = std::max(maxY, ry);
            }

            data.levelBounds[level].minX = static_cast<int32_t>(std::floor(minX));
            data.levelBounds[level].maxX = static_cast<int32_t>(std::ceil(maxX));
            data.levelBounds[level].minY = static_cast<int32_t>(std::floor(minY));
            data.levelBounds[level].maxY = static_cast<int32_t>(std::ceil(maxY));
        }
    }
}

// =============================================================================
// ShapeModelImpl::CreateModel
// =============================================================================

bool ShapeModelImpl::CreateModel(const QImage& image, const Rect2i& roi, const Point2d& origin) {
    // Validate and fix contrast parameters (HALCON-style hard checks)
    if (!params_.ValidateAndFixContrast()) {
        if (timingParams_.debugCreateModel) {
            std::printf("[CreateModel] Warning: Contrast parameters were invalid and auto-fixed.\n");
            std::printf("  contrastLow=%.1f, contrastHigh=%.1f, minContrast=%.1f, minComponentSize=%d\n",
                        params_.contrastLow, params_.contrastHigh,
                        params_.minContrast, params_.minComponentSize);
        }
    }

    // Reset timing
    createTiming_ = ShapeModelCreateTiming();
    auto tTotal = std::chrono::high_resolution_clock::now();
    auto tStep = tTotal;

    auto elapsedMs = [](auto start) {
        return std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - start).count();
    };

    // Build angle pyramid for template
    AnglePyramidParams pyramidParams;

    // Compute template dimensions
    int32_t templateWidth = roi.width > 0 ? roi.width : image.Width();
    int32_t templateHeight = roi.height > 0 ? roi.height : image.Height();
    int32_t minTemplateDim = std::min(templateWidth, templateHeight);

    // Compute maximum valid pyramid levels based on template size
    // Rule: minimum 12 pixels at coarsest level for reliable matching
    // (tested: 15px works, 8px too sparse)
    constexpr int32_t MIN_LEVEL_SIZE = 12;
    int32_t maxValidLevels = 1;
    int32_t dim = minTemplateDim;
    while (dim >= MIN_LEVEL_SIZE * 2 && maxValidLevels < 6) {
        dim /= 2;
        maxValidLevels++;
    }

    // Auto pyramid levels if not specified (Halcon: 'auto')
    if (params_.numLevels <= 0) {
        pyramidParams.numLevels = maxValidLevels;
    } else {
        // User specified levels - clamp to valid range
        pyramidParams.numLevels = std::min(params_.numLevels, maxValidLevels);
        if (params_.numLevels > maxValidLevels) {
            // Note: numLevels was clamped from user value to maxValidLevels
            // This happens when template is too small for requested levels
        }
    }
    pyramidParams.smoothSigma = 0.5;

    // Enable NMS for single-pixel edge extraction (Halcon-style)
    // Template edges should be single-pixel wide for accurate contour tracing
    pyramidParams.useNMS = true;

    // For contrast auto-detection, use a very low initial threshold to get all gradients
    bool needAutoContrast = (params_.contrastMode == ContrastMode::Auto ||
                             params_.contrastMode == ContrastMode::AutoHysteresis ||
                             params_.contrastMode == ContrastMode::AutoMinSize);

    if (needAutoContrast) {
        pyramidParams.minContrast = 1.0;  // Capture all gradients for analysis
    } else {
        // Manual mode: use contrastLow if hysteresis, otherwise contrastHigh
        pyramidParams.minContrast = (params_.contrastLow > 0) ? params_.contrastLow : params_.contrastHigh;
    }

    // Extract ROI if specified
    QImage templateImg;
    if (roi.width > 0 && roi.height > 0) {
        templateImg = image.SubImage(roi.x, roi.y, roi.width, roi.height);
        templateSize_ = Size2i{roi.width, roi.height};
    } else {
        templateImg = image;
        templateSize_ = Size2i{image.Width(), image.Height()};
    }

    if (templateImg.Empty()) {
        return false;
    }

    // Build pyramid (with timing)
    tStep = std::chrono::high_resolution_clock::now();
    AnglePyramid pyramid;
    if (!pyramid.Build(templateImg, pyramidParams)) {
        return false;
    }
    if (timingParams_.enableTiming) {
        createTiming_.pyramidBuildMs = elapsedMs(tStep);
    }

    // Auto-detect contrast threshold if requested
    tStep = std::chrono::high_resolution_clock::now();
    if (needAutoContrast) {
        const auto& edgePoints = pyramid.GetEdgePoints(0);
        if (!edgePoints.empty()) {
            // Collect magnitudes and sort (ascending for Otsu)
            std::vector<double> magnitudes;
            magnitudes.reserve(edgePoints.size());
            for (const auto& ep : edgePoints) {
                magnitudes.push_back(ep.magnitude);
            }
            std::sort(magnitudes.begin(), magnitudes.end());

            double minMag = magnitudes.front();
            double maxMag = magnitudes.back();

            // Estimate target point count based on template area
            int32_t templateArea = templateSize_.width * templateSize_.height;
            int32_t targetPoints;
            if (templateArea < 2500) {
                targetPoints = std::min(300, static_cast<int32_t>(magnitudes.size() / 4));
            } else if (templateArea < 10000) {
                targetPoints = std::min(800, static_cast<int32_t>(magnitudes.size() / 4));
            } else if (templateArea < 40000) {
                targetPoints = std::min(1500, static_cast<int32_t>(magnitudes.size() / 5));
            } else {
                targetPoints = std::min(2500, static_cast<int32_t>(magnitudes.size() / 6));
            }

            // Helper: Compute Otsu threshold
            auto computeOtsu = [&]() -> double {
                const int32_t numBins = 256;
                double range = maxMag - minMag;
                if (range < 1e-6) return minMag;

                // Build histogram
                std::vector<int32_t> hist(numBins, 0);
                for (double val : magnitudes) {
                    int32_t bin = static_cast<int32_t>((val - minMag) / range * (numBins - 1));
                    bin = std::clamp(bin, 0, numBins - 1);
                    hist[bin]++;
                }

                // Otsu algorithm
                int32_t total = static_cast<int32_t>(magnitudes.size());
                double sumAll = 0.0;
                for (int32_t i = 0; i < numBins; ++i) {
                    sumAll += i * hist[i];
                }

                double sumBg = 0.0;
                int32_t weightBg = 0;
                double maxVariance = 0.0;
                int32_t bestThreshold = 0;

                for (int32_t t = 0; t < numBins; ++t) {
                    weightBg += hist[t];
                    if (weightBg == 0) continue;

                    int32_t weightFg = total - weightBg;
                    if (weightFg == 0) break;

                    sumBg += t * hist[t];
                    double meanBg = sumBg / weightBg;
                    double meanFg = (sumAll - sumBg) / weightFg;

                    double variance = static_cast<double>(weightBg) * weightFg *
                                      (meanBg - meanFg) * (meanBg - meanFg);

                    if (variance > maxVariance) {
                        maxVariance = variance;
                        bestThreshold = t;
                    }
                }

                return minMag + (bestThreshold + 0.5) * range / numBins;
            };

            if (params_.contrastMode == ContrastMode::Auto) {
                size_t targetIdx = (magnitudes.size() > static_cast<size_t>(targetPoints))
                    ? magnitudes.size() - targetPoints : 0;
                double percentileThreshold = magnitudes[targetIdx];
                double otsuThreshold = computeOtsu();

                params_.contrastHigh = std::max(percentileThreshold, otsuThreshold) * 0.6 +
                                       std::min(percentileThreshold, otsuThreshold) * 0.4;
                params_.contrastHigh = std::clamp(params_.contrastHigh, 5.0, maxMag * 0.9);
                params_.contrastLow = 0.0;
            }
            else if (params_.contrastMode == ContrastMode::AutoHysteresis) {
                double otsuThreshold = computeOtsu();
                int32_t effectiveTargetPoints = std::max(80, targetPoints / 2);
                size_t targetIdx = (magnitudes.size() > static_cast<size_t>(effectiveTargetPoints))
                    ? magnitudes.size() - effectiveTargetPoints : 0;
                double percentileHigh = magnitudes[targetIdx];

                params_.contrastHigh = std::max(otsuThreshold, percentileHigh);
                params_.contrastHigh = std::clamp(params_.contrastHigh, 8.0, maxMag * 0.85);

                size_t lowIdx = magnitudes.size() / 2;
                double medianMag = magnitudes[lowIdx];

                params_.contrastLow = std::max(medianMag * 0.6, params_.contrastHigh * 0.4);
                params_.contrastLow = std::clamp(params_.contrastLow, 5.0, params_.contrastHigh * 0.7);
                params_.minComponentSize = std::max(params_.minComponentSize, 8);
            }
            else if (params_.contrastMode == ContrastMode::AutoMinSize) {
                double otsuThreshold = computeOtsu();
                params_.contrastHigh = std::clamp(otsuThreshold * 0.8, 5.0, maxMag * 0.9);
                params_.contrastLow = 0.0;
            }
        } else {
            params_.contrastHigh = 10.0;
            params_.contrastLow = 0.0;
        }
    }
    if (timingParams_.enableTiming) {
        createTiming_.contrastAutoMs = elapsedMs(tStep);
    }

    // Set origin - HALCON uses domain centroid as default reference point
    // For rectangular ROI, centroid equals bbox center
    origin_ = origin;
    if (origin_.x == 0 && origin_.y == 0) {
        origin_.x = templateSize_.width / 2.0;
        origin_.y = templateSize_.height / 2.0;
    }

    // Extract model points from pyramid using XLD contour extraction (Halcon-style)
    // XLD flow: Sobel → NMS → Threshold → 8-neighbor tracing → Resample (spacing ≈ 1px)
    tStep = std::chrono::high_resolution_clock::now();
    ExtractModelPointsXLD(templateImg, pyramid);
    if (timingParams_.enableTiming) {
        createTiming_.extractPointsMs = elapsedMs(tStep);
    }

    if (levels_.empty() || levels_[0].points.empty()) {
        return false;
    }

    // Apply optimization (point reduction) based on mode
    tStep = std::chrono::high_resolution_clock::now();
    if (params_.optimization != OptimizationMode::None) {
        OptimizeModel();
    }
    if (timingParams_.enableTiming) {
        createTiming_.optimizeMs = elapsedMs(tStep);
    }

    // Compute model bounding box for search constraints
    ComputeModelBounds();

    // Compute dynamic coverage threshold based on model complexity
    // Simple models (few points) need higher coverage to avoid false matches
    // Complex models (many points) can use lower coverage
    // This matches Halcon's automatic internal handling
    if (!levels_.empty() && !levels_[0].points.empty()) {
        size_t numModelPoints = levels_[0].points.size();
        if (numModelPoints < 200) {
            // Scale from 0.85 (50 points) to 0.7 (200 points)
            minCoverage_ = 0.7 + 0.15 * std::max(0.0, (200.0 - static_cast<double>(numModelPoints)) / 150.0);
            minCoverage_ = std::min(0.85, minCoverage_);
        } else {
            minCoverage_ = 0.7;
        }
    }

    // Build SoA data for SIMD optimization
    tStep = std::chrono::high_resolution_clock::now();
    for (auto& level : levels_) {
        level.BuildSoA();
    }

    // Build cosine lookup table for direction-quantized scoring
    const int16_t* binData;
    int32_t w, h, s, numBins;
    if (pyramid.GetAngleBinData(0, binData, w, h, s, numBins)) {
        BuildCosLUT(numBins);
    } else {
        BuildCosLUT(64);
    }

    // Build pregenerated search angle cache (Halcon pregeneration strategy)
    // This precomputes cos/sin and rotated bounds for all search angles,
    // avoiding expensive computation during search
    double angleExtent = params_.angleExtent;
    if (angleExtent <= 0) {
        angleExtent = 2.0 * PI;  // Full rotation range
    }
    BuildSearchAngleCache(params_.angleStart, angleExtent, params_.angleStep);

    if (timingParams_.enableTiming) {
        createTiming_.buildSoAMs = elapsedMs(tStep);
        createTiming_.totalMs = elapsedMs(tTotal);

        if (timingParams_.printTiming) {
            createTiming_.Print();
        }
    }

    valid_ = true;
    BuildScaledModels();
    return true;
}

// =============================================================================
// ShapeModelImpl::CreateModel (QRegion version)
// =============================================================================

bool ShapeModelImpl::CreateModel(const QImage& image, const QRegion& region, const Point2d& origin) {
    // Handle empty region as full image
    if (region.Empty()) {
        return CreateModel(image, Rect2i{}, origin);
    }

    // Validate and fix contrast parameters (HALCON-style hard checks)
    if (!params_.ValidateAndFixContrast()) {
        if (timingParams_.debugCreateModel) {
            std::printf("[CreateModel] Warning: Contrast parameters were invalid and auto-fixed.\n");
            std::printf("  contrastLow=%.1f, contrastHigh=%.1f, minContrast=%.1f, minComponentSize=%d\n",
                        params_.contrastLow, params_.contrastHigh,
                        params_.minContrast, params_.minComponentSize);
        }
    }

    // Reset timing
    createTiming_ = ShapeModelCreateTiming();
    auto tTotal = std::chrono::high_resolution_clock::now();
    auto tStep = tTotal;

    auto elapsedMs = [](auto start) {
        return std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - start).count();
    };

    // Get bounding box of region for template extraction
    Rect2i bbox = region.BoundingBox();

    // Compute template dimensions from bounding box
    int32_t templateWidth = bbox.width;
    int32_t templateHeight = bbox.height;
    int32_t minTemplateDim = std::min(templateWidth, templateHeight);

    // Build angle pyramid parameters
    AnglePyramidParams pyramidParams;

    // Compute maximum valid pyramid levels based on template size
    constexpr int32_t MIN_LEVEL_SIZE = 12;
    int32_t maxValidLevels = 1;
    int32_t dim = minTemplateDim;
    while (dim >= MIN_LEVEL_SIZE * 2 && maxValidLevels < 6) {
        dim /= 2;
        maxValidLevels++;
    }

    // Auto pyramid levels if not specified
    if (params_.numLevels <= 0) {
        pyramidParams.numLevels = maxValidLevels;
    } else {
        pyramidParams.numLevels = std::min(params_.numLevels, maxValidLevels);
    }
    pyramidParams.smoothSigma = 0.5;
    pyramidParams.useNMS = true;  // Single-pixel edges for accurate contour tracing

    // Contrast handling
    bool needAutoContrast = (params_.contrastMode == ContrastMode::Auto ||
                             params_.contrastMode == ContrastMode::AutoHysteresis ||
                             params_.contrastMode == ContrastMode::AutoMinSize);

    if (needAutoContrast) {
        pyramidParams.minContrast = 1.0;
    } else {
        pyramidParams.minContrast = (params_.contrastLow > 0) ? params_.contrastLow : params_.contrastHigh;
    }

    // Extract template from bounding box
    QImage templateImg = image.SubImage(bbox.x, bbox.y, bbox.width, bbox.height);
    templateSize_ = Size2i{bbox.width, bbox.height};

    if (templateImg.Empty()) {
        return false;
    }

    // Build pyramid
    tStep = std::chrono::high_resolution_clock::now();
    AnglePyramid pyramid;
    if (!pyramid.Build(templateImg, pyramidParams)) {
        return false;
    }
    if (timingParams_.enableTiming) {
        createTiming_.pyramidBuildMs = elapsedMs(tStep);
    }

    // Store actual levels used
    params_.numLevels = pyramid.NumLevels();

    // Translate region to template-local coordinates (must be done before origin calculation)
    QRegion localRegion = region.Translate(-bbox.x, -bbox.y);

    // Auto-detect contrast threshold for ROI if requested (use NMS edge points within region)
    tStep = std::chrono::high_resolution_clock::now();
    if (needAutoContrast) {
        const auto& edgePoints = pyramid.GetEdgePoints(0);
        if (!edgePoints.empty()) {
            std::vector<double> magnitudes;
            magnitudes.reserve(edgePoints.size());

            for (const auto& ep : edgePoints) {
                int32_t px = static_cast<int32_t>(std::lround(ep.x));
                int32_t py = static_cast<int32_t>(std::lround(ep.y));
                if (localRegion.Contains(px, py)) {
                    magnitudes.push_back(ep.magnitude);
                }
            }

            if (!magnitudes.empty()) {
                std::sort(magnitudes.begin(), magnitudes.end());

                double minMag = magnitudes.front();
                double maxMag = magnitudes.back();

                int32_t templateArea = templateSize_.width * templateSize_.height;
                int32_t targetPoints;
                if (templateArea < 2500) {
                    targetPoints = std::min(300, static_cast<int32_t>(magnitudes.size() / 4));
                } else if (templateArea < 10000) {
                    targetPoints = std::min(800, static_cast<int32_t>(magnitudes.size() / 4));
                } else if (templateArea < 40000) {
                    targetPoints = std::min(1500, static_cast<int32_t>(magnitudes.size() / 5));
                } else {
                    targetPoints = std::min(2500, static_cast<int32_t>(magnitudes.size() / 6));
                }

                auto computeOtsu = [&]() -> double {
                    const int32_t numBins = 256;
                    double range = maxMag - minMag;
                    if (range < 1e-6) return minMag;

                    std::vector<int32_t> hist(numBins, 0);
                    for (double val : magnitudes) {
                        int32_t bin = static_cast<int32_t>((val - minMag) / range * (numBins - 1));
                        bin = std::clamp(bin, 0, numBins - 1);
                        hist[bin]++;
                    }

                    int32_t total = static_cast<int32_t>(magnitudes.size());
                    double sumAll = 0.0;
                    for (int32_t i = 0; i < numBins; ++i) {
                        sumAll += i * hist[i];
                    }

                    double sumBg = 0.0;
                    int32_t weightBg = 0;
                    double maxVariance = 0.0;
                    int32_t bestThreshold = 0;

                    for (int32_t t = 0; t < numBins; ++t) {
                        weightBg += hist[t];
                        if (weightBg == 0) continue;

                        int32_t weightFg = total - weightBg;
                        if (weightFg == 0) break;

                        sumBg += t * hist[t];
                        double meanBg = sumBg / weightBg;
                        double meanFg = (sumAll - sumBg) / weightFg;

                        double variance = static_cast<double>(weightBg) * weightFg *
                                          (meanBg - meanFg) * (meanBg - meanFg);

                        if (variance > maxVariance) {
                            maxVariance = variance;
                            bestThreshold = t;
                        }
                    }

                    return minMag + (bestThreshold + 0.5) * range / numBins;
                };

                if (params_.contrastMode == ContrastMode::Auto) {
                    size_t targetIdx = (magnitudes.size() > static_cast<size_t>(targetPoints))
                        ? magnitudes.size() - targetPoints : 0;
                    double percentileThreshold = magnitudes[targetIdx];
                    double otsuThreshold = computeOtsu();

                    params_.contrastHigh = std::max(percentileThreshold, otsuThreshold) * 0.6 +
                                           std::min(percentileThreshold, otsuThreshold) * 0.4;
                    params_.contrastHigh = std::clamp(params_.contrastHigh, 5.0, maxMag * 0.9);
                    params_.contrastLow = 0.0;
                }
                else if (params_.contrastMode == ContrastMode::AutoHysteresis) {
                    double otsuThreshold = computeOtsu();
                    int32_t effectiveTargetPoints = std::max(80, targetPoints / 2);
                    size_t targetIdx = (magnitudes.size() > static_cast<size_t>(effectiveTargetPoints))
                        ? magnitudes.size() - effectiveTargetPoints : 0;
                    double percentileHigh = magnitudes[targetIdx];

                    params_.contrastHigh = std::max(otsuThreshold, percentileHigh);
                    params_.contrastHigh = std::clamp(params_.contrastHigh, 8.0, maxMag * 0.85);

                    size_t lowIdx = magnitudes.size() / 2;
                    double medianMag = magnitudes[lowIdx];

                    params_.contrastLow = std::max(medianMag * 0.6, params_.contrastHigh * 0.4);
                    params_.contrastLow = std::clamp(params_.contrastLow, 5.0, params_.contrastHigh * 0.7);
                    params_.minComponentSize = std::max(params_.minComponentSize, 8);
                }
                else if (params_.contrastMode == ContrastMode::AutoMinSize) {
                    double otsuThreshold = computeOtsu();
                    params_.contrastHigh = std::clamp(otsuThreshold * 0.8, 5.0, maxMag * 0.9);
                    params_.contrastLow = 0.0;
                }

                if (timingParams_.debugCreateModel) {
                    size_t p50 = magnitudes.size() / 2;
                    size_t p90 = magnitudes.size() * 9 / 10;
                    size_t p95 = magnitudes.size() * 95 / 100;
                    std::printf("[CreateModel][ROI] mag p50=%.2f p90=%.2f p95=%.2f max=%.2f -> contrast=[%.2f, %.2f]\n",
                                magnitudes[p50], magnitudes[p90], magnitudes[p95], maxMag,
                                params_.contrastLow, params_.contrastHigh);
                    std::fflush(stdout);
                }
            } else {
                params_.contrastHigh = 10.0;
                params_.contrastLow = 0.0;
            }
        } else {
            params_.contrastHigh = 10.0;
            params_.contrastLow = 0.0;
        }
    }
    if (timingParams_.enableTiming) {
        createTiming_.contrastAutoMs = elapsedMs(tStep);
    }

    // Set origin - HALCON uses domain centroid as default reference point
    origin_ = origin;
    if (origin_.x == 0 && origin_.y == 0) {
        // Default to domain centroid (center of gravity), not bbox center
        // This matches HALCON's create_shape_model behavior
        Point2d centroid = localRegion.Centroid();
        origin_.x = centroid.x;
        origin_.y = centroid.y;
    }

    // Extract model points using QRegion mask

    tStep = std::chrono::high_resolution_clock::now();
    ExtractModelPointsXLDWithRegion(templateImg, pyramid, localRegion);
    if (timingParams_.enableTiming) {
        createTiming_.extractPointsMs = elapsedMs(tStep);
    }

    if (levels_.empty() || levels_[0].points.empty()) {
        return false;
    }

    // Apply optimization
    tStep = std::chrono::high_resolution_clock::now();
    if (params_.optimization != OptimizationMode::None) {
        OptimizeModel();
    }
    if (timingParams_.enableTiming) {
        createTiming_.optimizeMs = elapsedMs(tStep);
    }

    // Compute model bounding box
    ComputeModelBounds();

    // Compute dynamic coverage threshold
    if (!levels_.empty() && !levels_[0].points.empty()) {
        size_t numModelPoints = levels_[0].points.size();
        if (numModelPoints < 200) {
            minCoverage_ = 0.7 + 0.15 * std::max(0.0, (200.0 - static_cast<double>(numModelPoints)) / 150.0);
            minCoverage_ = std::min(0.85, minCoverage_);
        } else {
            minCoverage_ = 0.7;
        }
    }

    // Build SoA data
    tStep = std::chrono::high_resolution_clock::now();
    for (auto& level : levels_) {
        level.BuildSoA();
    }

    // Build lookup tables
    const int16_t* binData;
    int32_t w, h, s, numBins;
    if (pyramid.GetAngleBinData(0, binData, w, h, s, numBins)) {
        BuildCosLUT(numBins);
    } else {
        BuildCosLUT(64);
    }

    // Build angle cache
    double angleExtent = params_.angleExtent;
    if (angleExtent <= 0) {
        angleExtent = 2.0 * PI;
    }
    BuildSearchAngleCache(params_.angleStart, angleExtent, params_.angleStep);

    if (timingParams_.enableTiming) {
        createTiming_.buildSoAMs = elapsedMs(tStep);
        createTiming_.totalMs = elapsedMs(tTotal);

        if (timingParams_.printTiming) {
            createTiming_.Print();
        }
    }

    valid_ = true;
    BuildScaledModels();
    return true;
}

// =============================================================================
// ShapeModelImpl::ExtractModelPointsXLD (Halcon-style XLD contour extraction)
// =============================================================================

void ShapeModelImpl::ExtractModelPointsXLD(const QImage& templateImg, const AnglePyramid& pyramid) {
    (void)templateImg;

    levels_.clear();
    levels_.resize(pyramid.NumLevels());

    // Halcon XLD flow (based on actual Halcon analysis):
    // 1. Filter edge points using hysteresis thresholding
    // 2. Trace into ordered contours using 8-connectivity
    // 3. Resample with constant spacing (~1px)
    // 4. NO maxPoints limit - let natural contour structure determine point count
    //
    // Halcon actual results (135x200 ROI, 5 levels):
    //   Level 1: 4541 points, spacing=1.14px
    //   Level 2: 958 points,  spacing=1.09px
    //   Level 3: 302 points,  spacing=1.12px
    //   Level 4: 154 points,  spacing=1.09px
    //   Level 5: 52 points,   spacing=1.11px

    const double RESAMPLE_SPACING = GetResampleSpacing(params_.optimization);
    const int32_t MIN_CONTOUR_POINTS = 4;

    double contrastHigh = params_.contrastHigh;
    double contrastLow = (params_.contrastLow > 0) ? params_.contrastLow : contrastHigh;
    double contrastMax = params_.contrastMax;
    bool useHysteresis = (params_.contrastLow > 0 && params_.contrastLow < params_.contrastHigh);

    for (int32_t level = 0; level < pyramid.NumLevels(); ++level) {
        const auto& levelData = pyramid.GetLevel(level);
        auto& levelModel = levels_[level];

        levelModel.width = levelData.width;
        levelModel.height = levelData.height;
        levelModel.scale = levelData.scale;

        double levelOriginX = origin_.x * levelData.scale;
        double levelOriginY = origin_.y * levelData.scale;

        const auto& edgePoints = pyramid.GetEdgePoints(level);

        // Compute level thresholds with fixed floors (HALCON-compatible)
        constexpr double FLOOR_LOW = 1.0;
        constexpr double FLOOR_HIGH = 2.0;
        double levelContrastHigh = std::max(FLOOR_HIGH, contrastHigh * levelData.scale);
        double levelContrastLow = std::max(FLOOR_LOW, contrastLow * levelData.scale);
        double levelContrastMax = contrastMax * levelData.scale;

        // Ensure High >= Low (HALCON requirement)
        if (levelContrastHigh < levelContrastLow) {
            levelContrastHigh = levelContrastLow;
        }

        // Debug: pre-hysteresis point count
        size_t preHysteresisCount = 0;
        for (const auto& ep : edgePoints) {
            if (ep.magnitude >= levelContrastLow && ep.magnitude <= levelContrastMax) {
                preHysteresisCount++;
            }
        }

        // Step 1: Filter edge points using hysteresis thresholding
        std::vector<Qi::Vision::Internal::EdgePoint> filteredPoints;
        filteredPoints.reserve(edgePoints.size());

        if (useHysteresis) {
            // Build edge map for BFS
            const double gridSize = 1.5;
            const double gridSizeSq = gridSize * gridSize;

            std::vector<int32_t> strongIndices;
            std::vector<int32_t> weakIndices;

            for (size_t i = 0; i < edgePoints.size(); ++i) {
                const auto& ep = edgePoints[i];
                if (ep.magnitude > levelContrastMax) continue;

                if (ep.magnitude >= levelContrastHigh) {
                    strongIndices.push_back(static_cast<int32_t>(i));
                } else if (ep.magnitude >= levelContrastLow) {
                    weakIndices.push_back(static_cast<int32_t>(i));
                }
            }

            std::vector<int8_t> keepFlag(edgePoints.size(), 0);
            for (int32_t idx : strongIndices) {
                keepFlag[idx] = 1;
            }

            // Spatial hash for weak points
            std::unordered_map<int64_t, std::vector<int32_t>> weakGrid;
            auto toGridKey = [gridSize](double x, double y) -> int64_t {
                int32_t gx = static_cast<int32_t>(std::floor(x / gridSize));
                int32_t gy = static_cast<int32_t>(std::floor(y / gridSize));
                return (static_cast<int64_t>(gx) << 32) | static_cast<uint32_t>(gy);
            };

            for (int32_t idx : weakIndices) {
                int64_t key = toGridKey(edgePoints[idx].x, edgePoints[idx].y);
                weakGrid[key].push_back(idx);
            }

            // BFS propagation
            std::queue<int32_t> bfsQueue;
            for (int32_t idx : strongIndices) {
                bfsQueue.push(idx);
            }

            while (!bfsQueue.empty()) {
                int32_t currentIdx = bfsQueue.front();
                bfsQueue.pop();

                const auto& currentPt = edgePoints[currentIdx];
                int32_t gx = static_cast<int32_t>(std::floor(currentPt.x / gridSize));
                int32_t gy = static_cast<int32_t>(std::floor(currentPt.y / gridSize));

                for (int32_t dy = -1; dy <= 1; ++dy) {
                    for (int32_t dx = -1; dx <= 1; ++dx) {
                        int64_t neighborKey = (static_cast<int64_t>(gx + dx) << 32) |
                                               static_cast<uint32_t>(gy + dy);

                        auto it = weakGrid.find(neighborKey);
                        if (it == weakGrid.end()) continue;

                        for (int32_t weakIdx : it->second) {
                            if (keepFlag[weakIdx] != 0) continue;

                            double ddx = edgePoints[weakIdx].x - currentPt.x;
                            double ddy = edgePoints[weakIdx].y - currentPt.y;
                            if (ddx * ddx + ddy * ddy <= gridSizeSq) {
                                keepFlag[weakIdx] = 1;
                                bfsQueue.push(weakIdx);
                            }
                        }
                    }
                }
            }

            for (size_t i = 0; i < edgePoints.size(); ++i) {
                if (keepFlag[i] == 1) {
                    filteredPoints.push_back(edgePoints[i]);
                }
            }
        } else {
            // Simple threshold
            for (const auto& ep : edgePoints) {
                if (ep.magnitude >= levelContrastHigh && ep.magnitude <= levelContrastMax) {
                    filteredPoints.push_back(ep);
                }
            }
        }

        // Debug: post-hysteresis point count
        size_t postHysteresisCount = filteredPoints.size();

        // Step 1.5: Component size filtering (HALCON min_size)
        // minSize is scaled per level: minSizeLevel = ceil(minSize * scale)
        if (params_.minComponentSize > 1) {
            int32_t levelMinSize = static_cast<int32_t>(std::ceil(
                static_cast<double>(params_.minComponentSize) * levelData.scale));
            levelMinSize = std::max(1, levelMinSize);

            filteredPoints = FilterByComponentSize(
                filteredPoints, levelData.width, levelData.height, levelMinSize);
        }

        // Debug: post-minSize point count
        size_t postMinSizeCount = filteredPoints.size();

        // Debug output (HALCON inspect_shape_model style)
        if (timingParams_.debugCreateModel) {
            std::printf("[CreateModel] Level %d (scale=%.3f): ", level, levelData.scale);
            std::printf("thresholds=[%.1f, %.1f], ",
                        levelContrastLow, levelContrastHigh);
            std::printf("points: pre-hyst=%zu -> post-hyst=%zu -> post-minSize=%zu\n",
                        preHysteresisCount, postHysteresisCount, postMinSizeCount);
        }

    // Step 2: Trace into ordered contours
        auto contours = TraceContoursXLD(filteredPoints, levelData.width, levelData.height, MIN_CONTOUR_POINTS);

        // Step 3: Resample and collect points WITH contour topology
        std::vector<ModelPoint> allPoints;
        allPoints.reserve(filteredPoints.size());

        std::vector<int32_t> contourStarts;
        std::vector<bool> contourClosed;
        contourStarts.reserve(contours.size() + 1);
        contourClosed.reserve(contours.size());

        for (const auto& contour : contours) {
            auto resampled = ResampleContourXLD(contour, RESAMPLE_SPACING);

            // Skip short contours after resampling
            if (static_cast<int32_t>(resampled.Size()) < MIN_CONTOUR_POINTS) continue;

            // Record contour start index
            contourStarts.push_back(static_cast<int32_t>(allPoints.size()));
            contourClosed.push_back(resampled.isClosed);

            // Convert to ModelPoints (relative to origin)
            for (size_t i = 0; i < resampled.Size(); ++i) {
                double relX = resampled.x[i] - levelOriginX;
                double relY = resampled.y[i] - levelOriginY;
                allPoints.emplace_back(relX, relY, resampled.angles[i],
                                       resampled.magnitudes[i], resampled.angleBins[i], 1.0);
            }
        }

        // Add sentinel value for easy iteration
        contourStarts.push_back(static_cast<int32_t>(allPoints.size()));

        // NO maxPoints limit - let natural contour structure determine count
        // This matches Halcon behavior where point counts naturally reduce with pyramid

        levelModel.points = std::move(allPoints);
        levelModel.contourStarts = std::move(contourStarts);
        levelModel.contourClosed = std::move(contourClosed);

        // Generate grid points (unique integer coordinates)
        std::set<std::pair<int32_t, int32_t>> uniqueGridCoords;
        std::vector<ModelPoint> gridPts;
        gridPts.reserve(levelModel.points.size());

        for (const auto& pt : levelModel.points) {
            int32_t gx = static_cast<int32_t>(std::round(pt.x));
            int32_t gy = static_cast<int32_t>(std::round(pt.y));

            auto key = std::make_pair(gx, gy);
            if (uniqueGridCoords.find(key) == uniqueGridCoords.end()) {
                uniqueGridCoords.insert(key);
                gridPts.emplace_back(static_cast<double>(gx), static_cast<double>(gy),
                                    pt.angle, pt.magnitude, pt.angleBin, pt.weight);
            }
        }

        std::sort(gridPts.begin(), gridPts.end(),
            [](const ModelPoint& a, const ModelPoint& b) {
                if (static_cast<int32_t>(a.y) != static_cast<int32_t>(b.y))
                    return a.y < b.y;
                return a.x < b.x;
            });

        levelModel.gridPoints = std::move(gridPts);
    }
}

// =============================================================================
// ShapeModelImpl::ExtractModelPointsXLDWithRegion (with QRegion mask)
// =============================================================================

void ShapeModelImpl::ExtractModelPointsXLDWithRegion(const QImage& templateImg, const AnglePyramid& pyramid,
                                                      const QRegion& region) {
    (void)templateImg;

    levels_.clear();
    levels_.resize(pyramid.NumLevels());

    const double RESAMPLE_SPACING = GetResampleSpacing(params_.optimization);
    const int32_t MIN_CONTOUR_POINTS = 4;

    double contrastHigh = params_.contrastHigh;
    double contrastLow = (params_.contrastLow > 0) ? params_.contrastLow : contrastHigh;
    double contrastMax = params_.contrastMax;
    bool useHysteresis = (params_.contrastLow > 0 && params_.contrastLow < params_.contrastHigh);

    for (int32_t level = 0; level < pyramid.NumLevels(); ++level) {
        const auto& levelData = pyramid.GetLevel(level);
        auto& levelModel = levels_[level];

        levelModel.width = levelData.width;
        levelModel.height = levelData.height;
        levelModel.scale = levelData.scale;

        double levelOriginX = origin_.x * levelData.scale;
        double levelOriginY = origin_.y * levelData.scale;

        // Scale region for this pyramid level
        QRegion scaledRegion = region.Scale(levelData.scale, levelData.scale);

        // Extract edge points with NMS from full level, then filter by region
        const auto& fullEdgePoints = pyramid.GetEdgePoints(level);
        std::vector<Qi::Vision::Internal::EdgePoint> allEdgePoints;
        allEdgePoints.reserve(fullEdgePoints.size());
        for (const auto& ep : fullEdgePoints) {
            int32_t px = static_cast<int32_t>(std::lround(ep.x));
            int32_t py = static_cast<int32_t>(std::lround(ep.y));
            if (scaledRegion.Contains(px, py)) {
                allEdgePoints.push_back(ep);
            }
        }

        // Compute level thresholds with fixed floors (HALCON-compatible)
        constexpr double FLOOR_LOW = 1.0;
        constexpr double FLOOR_HIGH = 2.0;
        double levelContrastHigh = std::max(FLOOR_HIGH, contrastHigh * levelData.scale);
        double levelContrastLow = std::max(FLOOR_LOW, contrastLow * levelData.scale);
        double levelContrastMax = contrastMax * levelData.scale;

        // Ensure High >= Low (HALCON requirement)
        if (levelContrastHigh < levelContrastLow) {
            levelContrastHigh = levelContrastLow;
        }

        // Local adaptive thresholds (for complex backgrounds)
        // Build per-tile high thresholds from ROI edge magnitudes
        const int32_t tileSize = std::max(16, std::min(32, std::min(levelData.width, levelData.height) / 4));
        const int32_t tilesX = (levelData.width + tileSize - 1) / tileSize;
        const int32_t tilesY = (levelData.height + tileSize - 1) / tileSize;
        std::vector<std::vector<float>> tileMags(static_cast<size_t>(tilesX * tilesY));
        tileMags.shrink_to_fit();  // ensure contiguous storage ownership

        for (const auto& ep : allEdgePoints) {
            int32_t tx = std::clamp(static_cast<int32_t>(ep.x) / tileSize, 0, tilesX - 1);
            int32_t ty = std::clamp(static_cast<int32_t>(ep.y) / tileSize, 0, tilesY - 1);
            tileMags[ty * tilesX + tx].push_back(static_cast<float>(ep.magnitude));
        }

        std::vector<float> tileHigh(static_cast<size_t>(tilesX * tilesY),
                                    static_cast<float>(levelContrastHigh));
        for (int32_t ty = 0; ty < tilesY; ++ty) {
            for (int32_t tx = 0; tx < tilesX; ++tx) {
                auto& mags = tileMags[ty * tilesX + tx];
                if (mags.size() < 20) {
                    continue;
                }
                std::sort(mags.begin(), mags.end());
                size_t idx = mags.size() * 85 / 100;  // p85
                idx = std::min(idx, mags.size() - 1);
                float localHigh = mags[idx];
                tileHigh[ty * tilesX + tx] = std::max(tileHigh[ty * tilesX + tx], localHigh);
            }
        }

        // Filter edge points by low threshold
        std::vector<Qi::Vision::Internal::EdgePoint> edgePoints;
        edgePoints.reserve(allEdgePoints.size());
        for (const auto& ep : allEdgePoints) {
            int32_t tx = std::clamp(static_cast<int32_t>(ep.x) / tileSize, 0, tilesX - 1);
            int32_t ty = std::clamp(static_cast<int32_t>(ep.y) / tileSize, 0, tilesY - 1);
            float localHigh = tileHigh[ty * tilesX + tx];
            float localLow = std::max(static_cast<float>(levelContrastLow), localHigh * 0.5f);
            if (ep.magnitude >= localLow) {
                edgePoints.push_back(ep);
            }
        }

        // Debug: pre-hysteresis point count
        size_t preHysteresisCount = 0;
        for (const auto& ep : edgePoints) {
            if (ep.magnitude >= levelContrastLow && ep.magnitude <= levelContrastMax) {
                preHysteresisCount++;
            }
        }

        // Step 1: Filter edge points using hysteresis thresholding
        std::vector<Qi::Vision::Internal::EdgePoint> filteredPoints;
        filteredPoints.reserve(edgePoints.size());

        if (useHysteresis) {
            const double gridSize = 1.5;
            const double gridSizeSq = gridSize * gridSize;

            std::vector<int32_t> strongIndices;
            std::vector<int32_t> weakIndices;

            for (size_t i = 0; i < edgePoints.size(); ++i) {
                const auto& ep = edgePoints[i];
                int32_t tx = std::clamp(static_cast<int32_t>(ep.x) / tileSize, 0, tilesX - 1);
                int32_t ty = std::clamp(static_cast<int32_t>(ep.y) / tileSize, 0, tilesY - 1);
                float localHigh = tileHigh[ty * tilesX + tx];
                float localLow = std::max(static_cast<float>(levelContrastLow), localHigh * 0.5f);
                if (ep.magnitude > levelContrastMax) continue;

                if (ep.magnitude >= localHigh) {
                    strongIndices.push_back(static_cast<int32_t>(i));
                } else if (ep.magnitude >= localLow) {
                    weakIndices.push_back(static_cast<int32_t>(i));
                }
            }

            std::vector<int8_t> keepFlag(edgePoints.size(), 0);
            for (int32_t idx : strongIndices) {
                keepFlag[idx] = 1;
            }

            std::unordered_map<int64_t, std::vector<int32_t>> weakGrid;
            auto toGridKey = [gridSize](double x, double y) -> int64_t {
                int32_t gx = static_cast<int32_t>(std::floor(x / gridSize));
                int32_t gy = static_cast<int32_t>(std::floor(y / gridSize));
                return (static_cast<int64_t>(gx) << 32) | static_cast<uint32_t>(gy);
            };

            for (int32_t idx : weakIndices) {
                int64_t key = toGridKey(edgePoints[idx].x, edgePoints[idx].y);
                weakGrid[key].push_back(idx);
            }

            std::queue<int32_t> bfsQueue;
            for (int32_t idx : strongIndices) {
                bfsQueue.push(idx);
            }

            while (!bfsQueue.empty()) {
                int32_t currentIdx = bfsQueue.front();
                bfsQueue.pop();

                const auto& currentPt = edgePoints[currentIdx];
                int32_t gx = static_cast<int32_t>(std::floor(currentPt.x / gridSize));
                int32_t gy = static_cast<int32_t>(std::floor(currentPt.y / gridSize));

                for (int32_t dy = -1; dy <= 1; ++dy) {
                    for (int32_t dx = -1; dx <= 1; ++dx) {
                        int64_t neighborKey = (static_cast<int64_t>(gx + dx) << 32) |
                                               static_cast<uint32_t>(gy + dy);

                        auto it = weakGrid.find(neighborKey);
                        if (it == weakGrid.end()) continue;

                        for (int32_t weakIdx : it->second) {
                            if (keepFlag[weakIdx] != 0) continue;

                            double ddx = edgePoints[weakIdx].x - currentPt.x;
                            double ddy = edgePoints[weakIdx].y - currentPt.y;
                            if (ddx * ddx + ddy * ddy <= gridSizeSq) {
                                keepFlag[weakIdx] = 1;
                                bfsQueue.push(weakIdx);
                            }
                        }
                    }
                }
            }

            for (size_t i = 0; i < edgePoints.size(); ++i) {
                if (keepFlag[i] == 1) {
                    filteredPoints.push_back(edgePoints[i]);
                }
            }
        } else {
            for (const auto& ep : edgePoints) {
                if (ep.magnitude >= levelContrastHigh && ep.magnitude <= levelContrastMax) {
                    filteredPoints.push_back(ep);
                }
            }
        }

        // Debug: post-hysteresis point count
        size_t postHysteresisCount = filteredPoints.size();

        // Step 1.5: Component size filtering (HALCON min_size)
        // minSize is scaled per level: minSizeLevel = ceil(minSize * scale)
        if (params_.minComponentSize > 1) {
            int32_t levelMinSize = static_cast<int32_t>(std::ceil(
                static_cast<double>(params_.minComponentSize) * levelData.scale));
            levelMinSize = std::max(1, levelMinSize);

            filteredPoints = FilterByComponentSize(
                filteredPoints, levelData.width, levelData.height, levelMinSize);
        }

        // Debug: post-minSize point count
        size_t postMinSizeCount = filteredPoints.size();

        // Debug output (HALCON inspect_shape_model style)
        if (timingParams_.debugCreateModel) {
            std::printf("[CreateModel] Level %d (scale=%.3f): ", level, levelData.scale);
            std::printf("thresholds=[%.1f, %.1f], ",
                        levelContrastLow, levelContrastHigh);
            std::printf("points: pre-hyst=%zu -> post-hyst=%zu -> post-minSize=%zu\n",
                        preHysteresisCount, postHysteresisCount, postMinSizeCount);
        }

        // Step 2: Trace into ordered contours
        auto contours = TraceContoursXLD(filteredPoints, levelData.width, levelData.height, MIN_CONTOUR_POINTS);

        // Step 3: Resample and collect points WITH contour topology
        std::vector<ModelPoint> allPoints;
        allPoints.reserve(filteredPoints.size());

        std::vector<int32_t> contourStarts;
        std::vector<bool> contourClosed;
        contourStarts.reserve(contours.size() + 1);
        contourClosed.reserve(contours.size());

        for (const auto& contour : contours) {
            auto resampled = ResampleContourXLD(contour, RESAMPLE_SPACING);

            if (static_cast<int32_t>(resampled.Size()) < MIN_CONTOUR_POINTS) continue;

            // Record contour start index
            contourStarts.push_back(static_cast<int32_t>(allPoints.size()));
            contourClosed.push_back(resampled.isClosed);

            for (size_t i = 0; i < resampled.Size(); ++i) {
                double relX = resampled.x[i] - levelOriginX;
                double relY = resampled.y[i] - levelOriginY;
                allPoints.emplace_back(relX, relY, resampled.angles[i],
                                       resampled.magnitudes[i], resampled.angleBins[i], 1.0);
            }
        }

        // Add sentinel value for easy iteration
        contourStarts.push_back(static_cast<int32_t>(allPoints.size()));

        levelModel.points = std::move(allPoints);
        levelModel.contourStarts = std::move(contourStarts);
        levelModel.contourClosed = std::move(contourClosed);

        // Generate grid points
        std::set<std::pair<int32_t, int32_t>> uniqueGridCoords;
        std::vector<ModelPoint> gridPts;
        gridPts.reserve(levelModel.points.size());

        for (const auto& pt : levelModel.points) {
            int32_t gx = static_cast<int32_t>(std::round(pt.x));
            int32_t gy = static_cast<int32_t>(std::round(pt.y));

            auto key = std::make_pair(gx, gy);
            if (uniqueGridCoords.find(key) == uniqueGridCoords.end()) {
                uniqueGridCoords.insert(key);
                gridPts.emplace_back(static_cast<double>(gx), static_cast<double>(gy),
                                    pt.angle, pt.magnitude, pt.angleBin, pt.weight);
            }
        }

        std::sort(gridPts.begin(), gridPts.end(),
            [](const ModelPoint& a, const ModelPoint& b) {
                if (static_cast<int32_t>(a.y) != static_cast<int32_t>(b.y))
                    return a.y < b.y;
                return a.x < b.x;
            });

        levelModel.gridPoints = std::move(gridPts);
    }
}

// =============================================================================
// ShapeModelImpl::OptimizeModel
// =============================================================================

void ShapeModelImpl::OptimizeModel() {
    // Optimization controls storage/search optimization, not edge extraction
    // XLD contour extraction is always used (Halcon-style)
    double minSpacing = 1.0;
    switch (params_.optimization) {
        case OptimizationMode::None:
            minSpacing = 0.0;  // Keep all points
            break;
        case OptimizationMode::PointReductionLow:
            minSpacing = 2.0;
            break;
        case OptimizationMode::PointReductionMedium:
            minSpacing = 3.0;
            break;
        case OptimizationMode::PointReductionHigh:
            minSpacing = 4.0;
            break;
        case OptimizationMode::Auto:
        default:
            // Auto: adaptive spacing based on template size
            int32_t templateDim = std::max(templateSize_.width, templateSize_.height);
            if (templateDim <= 100) {
                minSpacing = 2.0;
            } else if (templateDim <= 300) {
                minSpacing = 2.5;
            } else {
                minSpacing = 3.0;
            }
            break;
    }

    if (minSpacing > 0.5) {
        for (auto& level : levels_) {
            if (level.points.empty()) continue;

            // Check if we have valid contour topology
            bool hasValidTopology = !level.contourStarts.empty() &&
                                    level.contourStarts.size() > 1;

            if (hasValidTopology) {
                // Process each contour separately to preserve topology
                std::vector<ModelPoint> filteredAll;
                std::vector<int32_t> newContourStarts;
                std::vector<bool> newContourClosed;

                filteredAll.reserve(level.points.size());
                newContourStarts.reserve(level.contourStarts.size());
                newContourClosed.reserve(level.contourClosed.size());

                size_t numContours = level.contourStarts.size() - 1;
                for (size_t c = 0; c < numContours; ++c) {
                    int32_t startIdx = level.contourStarts[c];
                    int32_t endIdx = level.contourStarts[c + 1];

                    if (endIdx <= startIdx) continue;

                    // Filter within this contour, preserving order
                    // Use distance-based filtering along the contour path
                    std::vector<ModelPoint> filtered;
                    filtered.reserve(endIdx - startIdx);

                    double accumulatedDist = 0.0;
                    filtered.push_back(level.points[startIdx]);  // Always keep first point

                    for (int32_t i = startIdx + 1; i < endIdx; ++i) {
                        double dx = level.points[i].x - level.points[i-1].x;
                        double dy = level.points[i].y - level.points[i-1].y;
                        accumulatedDist += std::sqrt(dx*dx + dy*dy);

                        if (accumulatedDist >= minSpacing) {
                            filtered.push_back(level.points[i]);
                            accumulatedDist = 0.0;
                        }
                    }

                    // Always keep last point if contour is open
                    bool isClosed = (c < level.contourClosed.size()) ? level.contourClosed[c] : false;
                    if (filtered.size() >= 2 && !isClosed) {
                        const auto& lastPt = level.points[endIdx - 1];
                        if (filtered.back().x != lastPt.x || filtered.back().y != lastPt.y) {
                            filtered.push_back(lastPt);
                        }
                    }

                    // Only keep contour if it has enough points
                    if (filtered.size() >= 2) {
                        newContourStarts.push_back(static_cast<int32_t>(filteredAll.size()));
                        newContourClosed.push_back(isClosed);

                        for (auto& pt : filtered) {
                            filteredAll.push_back(std::move(pt));
                        }
                    }
                }

                // Add sentinel
                newContourStarts.push_back(static_cast<int32_t>(filteredAll.size()));

                level.points = std::move(filteredAll);
                level.contourStarts = std::move(newContourStarts);
                level.contourClosed = std::move(newContourClosed);
            } else {
                // No topology info - use original global filtering (for legacy models)
                double minDistSq = minSpacing * minSpacing;
                std::vector<ModelPoint> filtered;
                filtered.reserve(level.points.size());

                std::sort(level.points.begin(), level.points.end(),
                    [](const ModelPoint& a, const ModelPoint& b) {
                        return a.magnitude > b.magnitude;
                    });

                for (const auto& pt : level.points) {
                    bool tooClose = false;
                    for (const auto& kept : filtered) {
                        double dx = pt.x - kept.x;
                        double dy = pt.y - kept.y;
                        if (dx * dx + dy * dy < minDistSq) {
                            tooClose = true;
                            break;
                        }
                    }
                    if (!tooClose) {
                        filtered.push_back(pt);
                    }
                }

                level.points = std::move(filtered);
                // Clear invalid topology
                level.contourStarts.clear();
                level.contourClosed.clear();
            }
        }
    }

    // Normalize weights
    for (auto& level : levels_) {
        if (level.points.empty()) continue;

        double totalWeight = 0.0;
        for (const auto& pt : level.points) {
            totalWeight += pt.weight;
        }
        if (totalWeight > 0) {
            for (auto& pt : level.points) {
                pt.weight /= totalWeight;
            }
        }
    }
}

// =============================================================================
// ShapeModelImpl::BuildCosLUT
// =============================================================================

void ShapeModelImpl::BuildCosLUT(int32_t numBins) {
    numAngleBins_ = numBins;
    cosLUT_.resize(numBins);

    const double step = 2.0 * PI / numBins;
    for (int32_t i = 0; i < numBins; ++i) {
        cosLUT_[i] = static_cast<float>(std::fabs(std::cos(i * step)));
    }
}

// =============================================================================
// ShapeModelImpl::BuildAngleCache
// =============================================================================

void ShapeModelImpl::BuildAngleCache(double angleStart, double angleExtent, double angleStep) {
    angleCache_.clear();

    if (angleStep <= 0) {
        int32_t modelSize = std::max(templateSize_.width, templateSize_.height);
        angleStep = EstimateAngleStep(modelSize);
    }

    int32_t numAngles = static_cast<int32_t>(std::ceil(angleExtent / angleStep)) + 1;
    angleCache_.resize(levels_.size());

    for (size_t level = 0; level < levels_.size(); ++level) {
        angleCache_[level].resize(numAngles);

        for (int32_t i = 0; i < numAngles; ++i) {
            double angle = angleStart + i * angleStep;
            angleCache_[level][i].angle = angle;
            angleCache_[level][i].cosA = std::cos(angle);
            angleCache_[level][i].sinA = std::sin(angle);
        }
    }
}

// =============================================================================
// ShapeModelImpl::BuildSearchAngleCache (Halcon pregeneration strategy)
// =============================================================================

void ShapeModelImpl::BuildSearchAngleCache(double angleStart, double angleExtent, double angleStep) {
    searchAngleCache_.clear();

    // Store search parameters
    searchAngleStart_ = angleStart;
    searchAngleExtent_ = angleExtent;

    // Auto-compute angle step if not specified (Halcon: AngleStep = atan(1/R_max))
    if (angleStep <= 0) {
        int32_t modelSize = std::max(templateSize_.width, templateSize_.height);
        angleStep = EstimateAngleStep(modelSize);
    }
    searchAngleStep_ = angleStep;

    // Calculate number of angles
    int32_t numAngles = static_cast<int32_t>(std::ceil(angleExtent / angleStep)) + 1;
    searchAngleCache_.resize(numAngles);

    const size_t numLevels = levels_.size();

    // Precompute all angle data
    for (int32_t i = 0; i < numAngles; ++i) {
        SearchAngleData& data = searchAngleCache_[i];
        data.angle = angleStart + i * angleStep;
        data.cosA = static_cast<float>(std::cos(data.angle));
        data.sinA = static_cast<float>(std::sin(data.angle));

        // Precompute bounds for each pyramid level
        data.levelBounds.resize(numLevels);

        for (size_t level = 0; level < numLevels; ++level) {
            const auto& levelModel = levels_[level];
            if (levelModel.points.empty()) {
                data.levelBounds[level] = {0, 0, 0, 0};
                continue;
            }

            // Compute rotated bounds for this level
            double minX = std::numeric_limits<double>::max();
            double maxX = std::numeric_limits<double>::lowest();
            double minY = std::numeric_limits<double>::max();
            double maxY = std::numeric_limits<double>::lowest();

            const double cosA = data.cosA;
            const double sinA = data.sinA;

            for (const auto& pt : levelModel.points) {
                double rx = cosA * pt.x - sinA * pt.y;
                double ry = sinA * pt.x + cosA * pt.y;
                minX = std::min(minX, rx);
                maxX = std::max(maxX, rx);
                minY = std::min(minY, ry);
                maxY = std::max(maxY, ry);
            }

            // Store as integer bounds (floor/ceil for safety margin)
            data.levelBounds[level].minX = static_cast<int32_t>(std::floor(minX));
            data.levelBounds[level].maxX = static_cast<int32_t>(std::ceil(maxX));
            data.levelBounds[level].minY = static_cast<int32_t>(std::floor(minY));
            data.levelBounds[level].maxY = static_cast<int32_t>(std::ceil(maxY));
        }
    }
}

// =============================================================================
// ShapeModelImpl::BuildScaledModels
// =============================================================================

void ShapeModelImpl::BuildScaledModels() {
    scaledModels_.clear();

    if (levels_.empty() || !valid_) {
        return;
    }

    double scaleMin = params_.scaleMin;
    double scaleMax = params_.scaleMax;
    double scaleStep = params_.scaleStep;

    if (scaleMin <= 0 || scaleMax <= 0 || scaleMin > scaleMax) {
        return;
    }

    if (scaleStep <= 0.0) {
        scaleStep = 0.02;
        if (scaleMax > scaleMin) {
            scaleStep = std::max(0.01, (scaleMax - scaleMin) / 10.0);
        }
        params_.scaleStep = scaleStep;
    }

    constexpr double SCALE_TOLERANCE = 1e-6;
    for (double scale = scaleMin; scale <= scaleMax + SCALE_TOLERANCE; scale += scaleStep) {
        ScaledModelData sm;
        sm.scale = scale;

        sm.levels.reserve(levels_.size());
        for (const auto& level : levels_) {
            sm.levels.push_back(ScaleLevelModel(level, scale));
        }

        sm.templateSize.width = static_cast<int32_t>(std::round(templateSize_.width * scale));
        sm.templateSize.height = static_cast<int32_t>(std::round(templateSize_.height * scale));

        ComputeBoundsForLevels(sm.levels, sm.modelMinX, sm.modelMaxX, sm.modelMinY, sm.modelMaxY);
        sm.minCoverage = ComputeMinCoverageForLevels(sm.levels);

        double angleExtent = params_.angleExtent;
        if (angleExtent <= 0) {
            angleExtent = 2.0 * PI;
        }
        BuildSearchAngleCacheForLevels(sm.levels, sm.templateSize,
                                       params_.angleStart, angleExtent, params_.angleStep,
                                       sm.searchAngleCache, sm.searchAngleStep);
        sm.searchAngleStart = params_.angleStart;
        sm.searchAngleExtent = angleExtent;

        scaledModels_.push_back(std::move(sm));
    }
}

const ShapeModelImpl::ScaledModelData* ShapeModelImpl::GetScaledModelData(double scale) const {
    if (scaledModels_.empty()) {
        return nullptr;
    }
    const double tol = 1e-6;
    for (const auto& sm : scaledModels_) {
        if (std::abs(sm.scale - scale) <= tol) {
            return &sm;
        }
    }
    return nullptr;
}

// =============================================================================
// ShapeModelImpl::ComputeModelBounds
// =============================================================================

void ShapeModelImpl::ComputeModelBounds() {
    if (levels_.empty() || levels_[0].points.empty()) {
        modelMinX_ = modelMaxX_ = modelMinY_ = modelMaxY_ = 0;
        return;
    }

    modelMinX_ = modelMinY_ = std::numeric_limits<double>::max();
    modelMaxX_ = modelMaxY_ = std::numeric_limits<double>::lowest();

    for (const auto& pt : levels_[0].points) {
        modelMinX_ = std::min(modelMinX_, pt.x);
        modelMaxX_ = std::max(modelMaxX_, pt.x);
        modelMinY_ = std::min(modelMinY_, pt.y);
        modelMaxY_ = std::max(modelMaxY_, pt.y);
    }
}

// =============================================================================
// ShapeModelImpl::ComputeRotatedBounds
// =============================================================================

void ShapeModelImpl::ComputeRotatedBounds(const std::vector<ModelPoint>& points, double angle,
                                          double& minX, double& maxX, double& minY, double& maxY) {
    if (points.empty()) {
        minX = maxX = minY = maxY = 0;
        return;
    }

    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    minX = minY = std::numeric_limits<double>::max();
    maxX = maxY = std::numeric_limits<double>::lowest();

    for (const auto& pt : points) {
        double rx = cosA * pt.x - sinA * pt.y;
        double ry = sinA * pt.x + cosA * pt.y;
        minX = std::min(minX, rx);
        maxX = std::max(maxX, rx);
        minY = std::min(minY, ry);
        maxY = std::max(maxY, ry);
    }
}

} // namespace Internal
} // namespace Qi::Vision::Matching
