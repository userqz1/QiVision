/**
 * @file ShapeModelImpl.h
 * @brief Internal implementation structures for ShapeModel
 *
 * This file contains:
 * - LevelModel: Model data for a single pyramid level
 * - AngleData, RotatedPoint: Precomputed data structures
 * - FastCosTable: Fast cosine lookup table
 * - ShapeModelImpl: Implementation class
 */

#pragma once

#include <QiVision/Matching/ShapeModel.h>
#include <QiVision/Matching/MatchTypes.h>
#include <QiVision/Internal/AnglePyramid.h>
#include <QiVision/Internal/ResponseMap.h>
#include <QiVision/Internal/LinemodPyramid.h>
#include <QiVision/Core/Types.h>
#include <QiVision/Core/Constants.h>

#include <vector>
#include <set>
#include <cmath>
#include <cstdint>

#ifdef _OPENMP
#include <omp.h>
#endif

// SIMD intrinsics
#if defined(__AVX2__)
#include <immintrin.h>
#define HAVE_AVX2 1
#elif defined(__SSE4_1__)
#include <smmintrin.h>
#define HAVE_SSE4 1
#endif

namespace Qi::Vision::Matching {
namespace Internal {

// Import Internal types
using Qi::Vision::Internal::AnglePyramid;
using Qi::Vision::Internal::AnglePyramidParams;
using Qi::Vision::Internal::PyramidLevelData;
using Qi::Vision::Internal::EdgePoint;
using Qi::Vision::Internal::ResponseMap;
using Qi::Vision::Internal::RotatedResponseModel;
using Qi::Vision::Internal::LinemodPyramid;
using Qi::Vision::Internal::LinemodPyramidParams;
using Qi::Vision::Internal::LinemodFeature;
using Qi::Vision::Internal::LinemodLevelData;

// =============================================================================
// LevelModel: Model data for a single pyramid level
// =============================================================================

/**
 * @brief Model data for a single pyramid level
 *
 * Halcon-style dual block storage:
 * - Block 1 (points/soaX,Y...): Subpixel edge points for high-precision matching
 * - Block 2 (gridPoints/gridSoaX,Y...): Integer grid samples for fast coarse search
 *
 * Usage strategy:
 * - Coarse search (top pyramid levels): Use gridPoints (faster, integer positions)
 * - Fine refinement (level 0): Use points (subpixel accuracy)
 */
struct LevelModel {
    // Block 1: Subpixel edge points (from edge detection, irregular positions)
    std::vector<ModelPoint> points;

    // Block 2: Integer grid sample points (regular positions, faster for coarse search)
    std::vector<ModelPoint> gridPoints;

    int32_t width = 0;
    int32_t height = 0;
    double scale = 1.0;

    // SoA for Block 1 (subpixel points)
    std::vector<float> soaX;
    std::vector<float> soaY;
    std::vector<float> soaCosAngle;
    std::vector<float> soaSinAngle;
    std::vector<float> soaWeight;
    std::vector<int16_t> soaAngleBin;

    // SoA for Block 2 (grid points)
    std::vector<float> gridSoaX;
    std::vector<float> gridSoaY;
    std::vector<float> gridSoaCosAngle;
    std::vector<float> gridSoaSinAngle;
    std::vector<float> gridSoaWeight;
    std::vector<int16_t> gridSoaAngleBin;

    void BuildSoA();
    void RegenerateGridPoints();

private:
    static void BuildSoAForPoints(const std::vector<ModelPoint>& pts,
                                   std::vector<float>& x, std::vector<float>& y,
                                   std::vector<float>& cosA, std::vector<float>& sinA,
                                   std::vector<float>& w, std::vector<int16_t>& bins);
};

// =============================================================================
// Supporting Structures
// =============================================================================

/**
 * @brief Precomputed data for angle search
 */
struct AngleData {
    double angle = 0.0;
    double cosA = 1.0;
    double sinA = 0.0;
};

/**
 * @brief Precomputed rotated model point for fast score computation
 */
struct RotatedPoint {
    double offsetX;      // cosA * pt.x - sinA * pt.y
    double offsetY;      // sinA * pt.x + cosA * pt.y
    double expectedAngle; // pt.angle + angle
    double weight;
};

/**
 * @brief Precomputed search angle data with rotated bounds per level
 *
 * Halcon pregeneration strategy: Pre-compute all rotation-dependent data
 * at model creation time to avoid expensive computation during search.
 *
 * Key optimizations:
 * - cos/sin computed once per angle (not per search position)
 * - Rotated bounds computed once per angle per level (not per search position)
 * - Search region calculation becomes O(1) lookup
 */
struct SearchAngleData {
    double angle = 0.0;           ///< Rotation angle (radians)
    float cosA = 1.0f;            ///< cos(angle)
    float sinA = 0.0f;            ///< sin(angle)

    /// Precomputed rotated model bounds for each pyramid level
    struct LevelBounds {
        int32_t minX = 0, maxX = 0;  ///< Integer bounds for fast region computation
        int32_t minY = 0, maxY = 0;
    };
    std::vector<LevelBounds> levelBounds;  ///< Bounds[level]
};

/**
 * @brief Fast cosine lookup table for angle difference computation
 * Optimized with O(1) angle normalization instead of while loops
 */
class FastCosTable {
public:
    static constexpr int TABLE_SIZE = 2048;
    static constexpr int TABLE_MASK = TABLE_SIZE - 1;
    static constexpr double TABLE_SCALE = TABLE_SIZE / (2.0 * PI);
    static constexpr double INV_2PI = 1.0 / (2.0 * PI);
    static constexpr double INV_PI = 1.0 / PI;

    FastCosTable();

    // Fast cosine with angle in radians - O(1) normalization
    float FastCos(double angle) const;

    // Fast abs(cos) for symmetric similarity - O(1) normalization
    float FastAbsCos(double angleDiff) const;

private:
    float table_[TABLE_SIZE];
};

// Global cosine lookup table (defined in ShapeModelScore.cpp)
extern const FastCosTable g_cosTable;

// =============================================================================
// ShapeModelImpl: Implementation Class
// =============================================================================

class ShapeModelImpl {
public:
    // Model data
    std::vector<LevelModel> levels_;
    ModelParams params_;
    Point2d origin_;
    Size2i templateSize_;
    bool valid_ = false;

    // Timing configuration and results
    ShapeModelTimingParams timingParams_;
    ShapeModelCreateTiming createTiming_;
    mutable ShapeModelFindTiming findTiming_;  // mutable for const Find()

    // Model bounding box (cached)
    double modelMinX_ = 0, modelMaxX_ = 0;
    double modelMinY_ = 0, modelMaxY_ = 0;

    // Precomputed angle data for common angles
    std::vector<std::vector<AngleData>> angleCache_;

    // Direction quantization lookup table for fast scoring
    std::vector<float> cosLUT_;
    int32_t numAngleBins_ = 0;

    // LINEMOD mode data
    std::vector<std::vector<LinemodFeature>> linemodFeatures_;  ///< Features per level

    // Pregenerated search data (Halcon pregeneration strategy)
    std::vector<SearchAngleData> searchAngleCache_;  ///< Precomputed angle data for search
    double searchAngleStart_ = 0.0;                  ///< Search angle range start
    double searchAngleExtent_ = 2.0 * PI;            ///< Search angle range extent
    double searchAngleStep_ = 0.0;                   ///< Search angle step (0 = auto)

    // Dynamic coverage threshold (computed from model complexity)
    // Simple models (few points) need higher coverage to avoid false matches
    double minCoverage_ = 0.7;                       ///< Minimum coverage for valid match

    // ==========================================================================
    // Model Creation (ShapeModelCreate.cpp)
    // ==========================================================================

    bool CreateModel(const QImage& image, const Rect2i& roi, const Point2d& origin);
    bool CreateModel(const QImage& image, const QRegion& region, const Point2d& origin);
    bool CreateModelLinemod(const QImage& image, const Rect2i& roi, const Point2d& origin);
    void ExtractModelPointsXLD(const QImage& templateImg, const AnglePyramid& pyramid);
    void ExtractModelPointsXLDWithRegion(const QImage& templateImg, const AnglePyramid& pyramid,
                                          const QRegion& region);
    void OptimizeModel();
    void BuildCosLUT(int32_t numBins);
    void BuildAngleCache(double angleStart, double angleExtent, double angleStep);
    void BuildSearchAngleCache(double angleStart, double angleExtent, double angleStep);
    void ComputeModelBounds();
    static void ComputeRotatedBounds(const std::vector<ModelPoint>& points, double angle,
                                     double& minX, double& maxX, double& minY, double& maxY);

    // ==========================================================================
    // Search Functions (ShapeModelSearch.cpp)
    // ==========================================================================

    std::vector<MatchResult> SearchPyramid(const AnglePyramid& targetPyramid,
                                            const SearchParams& params) const;

    std::vector<MatchResult> SearchPyramidWithResponseMap(
        const AnglePyramid& targetPyramid,
        const ResponseMap& responseMap,
        const SearchParams& params) const;

    std::vector<MatchResult> SearchPyramidLinemod(
        const LinemodPyramid& targetPyramid,
        const SearchParams& params) const;

    std::vector<MatchResult> SearchLevel(const AnglePyramid& targetPyramid,
                                          int32_t level,
                                          const std::vector<MatchResult>& candidates,
                                          const SearchParams& params,
                                          int32_t positionRadius = -1,
                                          double angleRadiusDeg = -1) const;

    // ==========================================================================
    // Score Computation (ShapeModelScore.cpp)
    // ==========================================================================

    double ComputeScoreAtPosition(const AnglePyramid& pyramid, int32_t level,
                                   double x, double y, double angle, double scale,
                                   double greediness, double* outCoverage = nullptr,
                                   bool useGridPoints = false) const;

    double ComputeScoreAtPositionFast(const AnglePyramid& pyramid, int32_t level,
                                       int32_t x, int32_t y, double angle, double scale,
                                       double* outCoverage = nullptr) const;

    double ComputeScoreBilinearScalar(const AnglePyramid& pyramid, int32_t level,
                                       double x, double y, double angle, double scale,
                                       double greediness, double* outCoverage = nullptr,
                                       bool useGridPoints = false) const;

    double ComputeScoreBilinearSSE(const AnglePyramid& pyramid, int32_t level,
                                    double x, double y, double angle, double scale,
                                    double greediness, double* outCoverage = nullptr,
                                    bool useGridPoints = false) const;

    double ComputeScoreWithSinCos(const AnglePyramid& pyramid, int32_t level,
                                   double x, double y, float cosR, float sinR, double scale,
                                   double greediness, double* outCoverage = nullptr,
                                   bool useGridPoints = false) const;

    /// Fast score using nearest-neighbor interpolation (4x less memory access)
    double ComputeScoreNearestNeighbor(const AnglePyramid& pyramid, int32_t level,
                                        int32_t x, int32_t y, float cosR, float sinR,
                                        double greediness, double* outCoverage = nullptr) const;

    double ComputeScoreQuantized(const AnglePyramid& pyramid, int32_t level,
                                  double x, double y, float cosR, float sinR, int32_t rotationBin,
                                  double greediness, double* outCoverage = nullptr,
                                  bool useGridPoints = false) const;

    void RefinePosition(const AnglePyramid& pyramid, MatchResult& match,
                        SubpixelMethod method) const;
};

} // namespace Internal
} // namespace Qi::Vision::Matching
