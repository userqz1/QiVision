/**
 * @file Caliper.cpp
 * @brief Implementation of caliper measurement functions (Halcon compatible API)
 */

#include <QiVision/Measure/Caliper.h>
#include <QiVision/Internal/Edge1D.h>
#include <QiVision/Internal/Profiler.h>
#include <QiVision/Internal/SubPixel.h>
#include <QiVision/Internal/Interpolate.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <numeric>

namespace Qi::Vision::Measure {

namespace {
    constexpr double PI = 3.14159265358979323846;

    // Convert ProfileInterpolation to Internal InterpolationMethod
    Internal::InterpolationMethod ToInternalInterp(ProfileInterpolation interp) {
        switch (interp) {
            case ProfileInterpolation::Nearest:
                return Internal::InterpolationMethod::Nearest;
            case ProfileInterpolation::Bilinear:
                return Internal::InterpolationMethod::Bilinear;
            case ProfileInterpolation::Bicubic:
                return Internal::InterpolationMethod::Bicubic;
        }
        return Internal::InterpolationMethod::Bilinear;
    }

    // Convert Internal EdgePolarity to EdgeTransition
    EdgeTransition ToEdgeTransition(Internal::EdgePolarity polarity) {
        switch (polarity) {
            case Internal::EdgePolarity::Positive:
                return EdgeTransition::Positive;
            case Internal::EdgePolarity::Negative:
                return EdgeTransition::Negative;
            case Internal::EdgePolarity::Both:
                return EdgeTransition::All;
        }
        return EdgeTransition::All;
    }

    // Compute fuzzy score for an edge
    double ComputeFuzzyScore(double amplitude, double maxAmplitude,
                             double fuzzyLow, double fuzzyHigh) {
        if (maxAmplitude < 1e-6) return 0.0;
        double ratio = amplitude / maxAmplitude;

        if (ratio < fuzzyLow) return 0.0;
        if (ratio >= fuzzyHigh) return 1.0;

        // Linear interpolation between fuzzyLow and fuzzyHigh
        return (ratio - fuzzyLow) / (fuzzyHigh - fuzzyLow);
    }

    // Extract profile implementation
    std::vector<double> ExtractProfileImpl(const QImage& image,
                                            const MeasureRectangle2& handle,
                                            ProfileInterpolation interp) {
        if (image.Empty() || !handle.IsValid()) {
            return {};
        }

        // Build RectProfileParams
        Internal::RectProfileParams params;
        params.centerX = handle.Column();
        params.centerY = handle.Row();
        params.length = handle.ProfileLength();  // 2 * Length1
        params.width = 2.0 * handle.Length2();   // Full width
        params.angle = handle.ProfileAngle();
        params.numLines = handle.NumLines();
        params.samplesPerPixel = handle.SamplesPerPixel();
        params.interp = ToInternalInterp(interp);
        params.method = Internal::ProfileMethod::Average;

        // Extract profile using Profiler
        auto profile = Internal::ExtractRectProfile(image, params);
        return profile.data;
    }

    std::vector<double> ExtractProfileImpl(const QImage& image,
                                            const MeasureArc& handle,
                                            ProfileInterpolation interp) {
        if (image.Empty() || !handle.IsValid()) {
            return {};
        }

        // Build ArcProfileParams
        Internal::ArcProfileParams params;
        params.centerX = handle.CenterCol();
        params.centerY = handle.CenterRow();
        params.radius = handle.Radius();
        params.startAngle = handle.AngleStart();
        params.endAngle = handle.AngleEnd();
        params.width = handle.AnnulusRadius() * 2.0;  // Full width
        params.numLines = handle.NumLines();
        params.samplesPerPixel = handle.SamplesPerPixel();
        params.interp = ToInternalInterp(interp);
        params.method = Internal::ProfileMethod::Average;

        auto profile = Internal::ExtractArcProfile(image, params);
        return profile.data;
    }

    std::vector<double> ExtractProfileImpl(const QImage& image,
                                            const MeasureConcentricCircles& handle,
                                            ProfileInterpolation interp) {
        if (image.Empty() || !handle.IsValid()) {
            return {};
        }

        // Build AnnularProfileParams
        Internal::AnnularProfileParams params;
        params.centerX = handle.CenterCol();
        params.centerY = handle.CenterRow();
        params.innerRadius = handle.InnerRadius();
        params.outerRadius = handle.OuterRadius();
        params.angle = handle.Angle();
        params.angularWidth = handle.AngularWidth();
        params.numLines = handle.NumLines();
        params.samplesPerPixel = handle.SamplesPerPixel();
        params.interp = ToInternalInterp(interp);
        params.method = Internal::ProfileMethod::Average;

        auto profile = Internal::ExtractAnnularProfile(image, params);
        return profile.data;
    }
}

// =============================================================================
// Profile Extraction (Public API)
// =============================================================================

std::vector<double> ExtractMeasureProfile(const QImage& image,
                                           const MeasureRectangle2& handle,
                                           const std::string& interp) {
    return ExtractProfileImpl(image, handle, ParseInterpolation(interp));
}

std::vector<double> ExtractMeasureProfile(const QImage& image,
                                           const MeasureArc& handle,
                                           const std::string& interp) {
    return ExtractProfileImpl(image, handle, ParseInterpolation(interp));
}

std::vector<double> ExtractMeasureProfile(const QImage& image,
                                           const MeasureConcentricCircles& handle,
                                           const std::string& interp) {
    return ExtractProfileImpl(image, handle, ParseInterpolation(interp));
}

// =============================================================================
// Coordinate Transformation
// =============================================================================

Point2d ProfileToImage(const MeasureRectangle2& handle, double profilePos) {
    double t = profilePos / handle.ProfileLength();
    t = std::clamp(t, 0.0, 1.0);

    double profileAngle = handle.ProfileAngle();
    double halfLen = handle.Length1();  // Length1 is half-length

    double startX = handle.Column() - halfLen * std::cos(profileAngle);
    double startY = handle.Row() - halfLen * std::sin(profileAngle);

    return Point2d{
        startX + profilePos * std::cos(profileAngle),
        startY + profilePos * std::sin(profileAngle)
    };
}

Point2d ProfileToImage(const MeasureArc& handle, double profilePos) {
    double angle = handle.ProfilePosToAngle(profilePos);
    return Point2d{
        handle.CenterCol() + handle.Radius() * std::cos(angle),
        handle.CenterRow() + handle.Radius() * std::sin(angle)
    };
}

Point2d ProfileToImage(const MeasureConcentricCircles& handle, double profilePos) {
    double radius = handle.ProfilePosToRadius(profilePos);
    return Point2d{
        handle.CenterCol() + radius * std::cos(handle.Angle()),
        handle.CenterRow() + radius * std::sin(handle.Angle())
    };
}

// =============================================================================
// Sample Count Helpers
// =============================================================================

int32_t GetNumSamples(const MeasureRectangle2& handle) {
    return static_cast<int32_t>(std::ceil(handle.ProfileLength() * handle.SamplesPerPixel())) + 1;
}

int32_t GetNumSamples(const MeasureArc& handle) {
    return static_cast<int32_t>(std::ceil(handle.ProfileLength() * handle.SamplesPerPixel())) + 1;
}

int32_t GetNumSamples(const MeasureConcentricCircles& handle) {
    return static_cast<int32_t>(std::ceil(handle.ProfileLength() * handle.SamplesPerPixel())) + 1;
}

// =============================================================================
// Edge Detection Core
// =============================================================================

namespace {
    // Common edge detection implementation
    template<typename HandleT>
    std::vector<EdgeResult> MeasurePosImpl(const QImage& image,
                                            const HandleT& handle,
                                            double sigma,
                                            double threshold,
                                            EdgeTransition transition,
                                            EdgeSelectMode selectMode) {
        std::vector<EdgeResult> results;

        if (image.Empty() || !handle.IsValid()) {
            return results;
        }

        // Extract profile
        auto profile = ExtractProfileImpl(image, handle, ProfileInterpolation::Bilinear);
        if (profile.size() < 3) {
            return results;
        }

        // Detect edges using Edge1D
        auto edges1D = Internal::DetectEdges1D(
            profile.data(),
            profile.size(),
            threshold,
            ToEdgePolarity(transition),
            sigma
        );

        if (edges1D.empty()) {
            return results;
        }

        // Convert 1D results to EdgeResult
        double profileLength = handle.ProfileLength();
        double stepSize = profileLength / (profile.size() - 1);

        for (const auto& edge : edges1D) {
            EdgeResult result;

            // Profile position
            result.profilePosition = edge.position * stepSize;

            // Convert to image coordinates
            Point2d imgPos = ProfileToImage(handle, result.profilePosition);
            result.column = imgPos.x;
            result.row = imgPos.y;

            // Edge properties
            result.amplitude = std::abs(edge.amplitude);
            result.transition = ToEdgeTransition(edge.polarity);

            // Angle is the profile direction
            result.angle = handle.ProfileAngle();

            // Confidence based on amplitude
            result.confidence = std::min(1.0, result.amplitude / 255.0);
            result.score = result.confidence;

            results.push_back(result);
        }

        // Apply selection mode
        results = SelectEdges(results, selectMode, MAX_EDGES);

        return results;
    }

    // Pair detection implementation
    template<typename HandleT>
    std::vector<PairResult> MeasurePairsImpl(const QImage& image,
                                              const HandleT& handle,
                                              double sigma,
                                              double threshold,
                                              EdgeTransition firstTransition,
                                              EdgeTransition secondTransition,
                                              PairSelectMode selectMode) {
        std::vector<PairResult> results;

        if (image.Empty() || !handle.IsValid()) {
            return results;
        }

        // Extract profile
        auto profile = ExtractProfileImpl(image, handle, ProfileInterpolation::Bilinear);
        if (profile.size() < 3) {
            return results;
        }

        // Detect all edges (with Both polarity to get all)
        auto allEdges = Internal::DetectEdges1D(
            profile.data(),
            profile.size(),
            threshold,
            Internal::EdgePolarity::Both,
            sigma
        );

        if (allEdges.size() < 2) {
            return results;
        }

        // Separate edges by polarity
        std::vector<Internal::Edge1DResult> firstEdges, secondEdges;

        for (const auto& edge : allEdges) {
            EdgeTransition trans = ToEdgeTransition(edge.polarity);

            bool matchFirst = (firstTransition == EdgeTransition::All) ||
                              (firstTransition == trans);
            bool matchSecond = (secondTransition == EdgeTransition::All) ||
                               (secondTransition == trans);

            if (matchFirst) firstEdges.push_back(edge);
            if (matchSecond) secondEdges.push_back(edge);
        }

        // Find pairs
        double profileLength = handle.ProfileLength();
        double stepSize = profileLength / (profile.size() - 1);

        for (const auto& first : firstEdges) {
            for (const auto& second : secondEdges) {
                // Second edge must come after first
                if (second.position <= first.position) continue;

                double width = (second.position - first.position) * stepSize;

                PairResult pair;

                // First edge
                pair.first.profilePosition = first.position * stepSize;
                Point2d p1 = ProfileToImage(handle, pair.first.profilePosition);
                pair.first.column = p1.x;
                pair.first.row = p1.y;
                pair.first.amplitude = std::abs(first.amplitude);
                pair.first.transition = ToEdgeTransition(first.polarity);
                pair.first.confidence = std::min(1.0, pair.first.amplitude / 255.0);
                pair.first.angle = handle.ProfileAngle();

                // Second edge
                pair.second.profilePosition = second.position * stepSize;
                Point2d p2 = ProfileToImage(handle, pair.second.profilePosition);
                pair.second.column = p2.x;
                pair.second.row = p2.y;
                pair.second.amplitude = std::abs(second.amplitude);
                pair.second.transition = ToEdgeTransition(second.polarity);
                pair.second.confidence = std::min(1.0, pair.second.amplitude / 255.0);
                pair.second.angle = handle.ProfileAngle();

                // Pair properties (Halcon compatible)
                pair.intraDistance = width;  // Distance within this pair
                pair.width = width;          // Legacy alias
                pair.centerColumn = (pair.first.column + pair.second.column) / 2.0;
                pair.centerRow = (pair.first.row + pair.second.row) / 2.0;

                // Symmetry: ratio of amplitudes (1.0 = perfectly symmetric)
                double minAmp = std::min(pair.first.amplitude, pair.second.amplitude);
                double maxAmp = std::max(pair.first.amplitude, pair.second.amplitude);
                pair.symmetry = (maxAmp > 0) ? minAmp / maxAmp : 0.0;

                // Score: combination of amplitude and symmetry
                pair.score = (pair.first.confidence + pair.second.confidence) / 2.0 * pair.symmetry;

                results.push_back(pair);
            }
        }

        // Apply selection mode
        results = SelectPairs(results, selectMode, MAX_EDGES);

        return results;
    }

    // Fuzzy measurement implementation
    template<typename HandleT>
    std::vector<EdgeResult> FuzzyMeasurePosImpl(const QImage& image,
                                                 const HandleT& handle,
                                                 double sigma,
                                                 double threshold,
                                                 EdgeTransition transition,
                                                 EdgeSelectMode selectMode,
                                                 double fuzzyThresh,
                                                 MeasureStats* stats) {
        std::vector<EdgeResult> results;

        if (image.Empty() || !handle.IsValid()) {
            if (stats) *stats = MeasureStats{};
            return results;
        }

        // Extract profile
        auto profile = ExtractProfileImpl(image, handle, ProfileInterpolation::Bilinear);
        if (profile.size() < 3) {
            if (stats) *stats = MeasureStats{};
            return results;
        }

        // Compute profile statistics for adaptive threshold
        double profileMin = *std::min_element(profile.begin(), profile.end());
        double profileMax = *std::max_element(profile.begin(), profile.end());
        double contrast = profileMax - profileMin;

        // Detect edges with lower threshold for more candidates
        constexpr double FUZZY_LOW = 0.3;
        constexpr double FUZZY_HIGH = 0.8;
        double effectiveThreshold = std::max(threshold * FUZZY_LOW, contrast * FUZZY_LOW);

        auto edges1D = Internal::DetectEdges1D(
            profile.data(),
            profile.size(),
            effectiveThreshold,
            ToEdgePolarity(transition),
            sigma
        );

        if (edges1D.empty()) {
            if (stats) {
                stats->numEdgesFound = 0;
                stats->numEdgesReturned = 0;
                stats->profileContrast = contrast;
            }
            return results;
        }

        // Find max amplitude for fuzzy scoring
        double maxAmplitude = 0.0;
        for (const auto& e : edges1D) {
            maxAmplitude = std::max(maxAmplitude, std::abs(e.amplitude));
        }

        // Convert to EdgeResult with fuzzy scores
        double profileLength = handle.ProfileLength();
        double stepSize = profileLength / (profile.size() - 1);

        for (const auto& edge : edges1D) {
            double score = ComputeFuzzyScore(std::abs(edge.amplitude), maxAmplitude,
                                             FUZZY_LOW, FUZZY_HIGH);

            // Filter by minimum score
            if (score < fuzzyThresh) continue;

            EdgeResult result;

            result.profilePosition = edge.position * stepSize;
            Point2d imgPos = ProfileToImage(handle, result.profilePosition);
            result.column = imgPos.x;
            result.row = imgPos.y;

            result.amplitude = std::abs(edge.amplitude);
            result.transition = ToEdgeTransition(edge.polarity);
            result.angle = handle.ProfileAngle();
            result.score = score;
            result.confidence = score;

            results.push_back(result);
        }

        // Update statistics
        if (stats) {
            stats->numEdgesFound = static_cast<int32_t>(edges1D.size());
            stats->numEdgesReturned = static_cast<int32_t>(results.size());
            stats->profileContrast = contrast;
            stats->maxAmplitude = maxAmplitude;

            if (!results.empty()) {
                double sumAmp = 0.0;
                double minAmp = results[0].amplitude;
                for (const auto& r : results) {
                    sumAmp += r.amplitude;
                    minAmp = std::min(minAmp, r.amplitude);
                }
                stats->meanAmplitude = sumAmp / results.size();
                stats->minAmplitude = minAmp;
            }

            // Estimate SNR
            if (contrast > 0) {
                stats->signalNoiseRatio = maxAmplitude / (contrast * 0.1 + 1.0);
            }
        }

        // Apply selection mode
        results = SelectEdges(results, selectMode, MAX_EDGES);

        return results;
    }

    // Fuzzy pair measurement implementation
    template<typename HandleT>
    std::vector<PairResult> FuzzyMeasurePairsImpl(const QImage& image,
                                                   const HandleT& handle,
                                                   double sigma,
                                                   double threshold,
                                                   EdgeTransition transition,
                                                   EdgeSelectMode selectMode,
                                                   double fuzzyThresh,
                                                   MeasureStats* stats) {
        // First get fuzzy edges
        MeasureStats localStats;
        auto edges = FuzzyMeasurePosImpl(image, handle, sigma, threshold,
                                          EdgeTransition::All, EdgeSelectMode::All,
                                          fuzzyThresh, &localStats);

        if (stats) *stats = localStats;

        std::vector<PairResult> results;
        if (edges.size() < 2) {
            return results;
        }

        // Determine first/second transition based on transition parameter
        EdgeTransition firstTrans = EdgeTransition::Positive;
        EdgeTransition secondTrans = EdgeTransition::Negative;
        if (transition == EdgeTransition::Negative) {
            firstTrans = EdgeTransition::Negative;
            secondTrans = EdgeTransition::Positive;
        } else if (transition == EdgeTransition::All) {
            firstTrans = EdgeTransition::All;
            secondTrans = EdgeTransition::All;
        }

        // Separate by transition type
        std::vector<EdgeResult> firstEdges, secondEdges;

        for (const auto& edge : edges) {
            bool matchFirst = (firstTrans == EdgeTransition::All) ||
                              (firstTrans == edge.transition);
            bool matchSecond = (secondTrans == EdgeTransition::All) ||
                               (secondTrans == edge.transition);

            if (matchFirst) firstEdges.push_back(edge);
            if (matchSecond) secondEdges.push_back(edge);
        }

        // Find pairs
        for (const auto& first : firstEdges) {
            for (const auto& second : secondEdges) {
                if (second.profilePosition <= first.profilePosition) continue;

                double width = second.profilePosition - first.profilePosition;

                PairResult pair;
                pair.first = first;
                pair.second = second;
                pair.intraDistance = width;  // Halcon compatible
                pair.width = width;          // Legacy alias
                pair.centerColumn = (first.column + second.column) / 2.0;
                pair.centerRow = (first.row + second.row) / 2.0;

                double minAmp = std::min(first.amplitude, second.amplitude);
                double maxAmp = std::max(first.amplitude, second.amplitude);
                pair.symmetry = (maxAmp > 0) ? minAmp / maxAmp : 0.0;

                // Combined score
                pair.score = (first.score + second.score) / 2.0 * pair.symmetry;

                results.push_back(pair);
            }
        }

        // Sort by score descending
        std::sort(results.begin(), results.end(),
                  [](const PairResult& a, const PairResult& b) {
                      return a.score > b.score;
                  });

        // Apply selection mode
        results = SelectPairs(results, ParsePairSelect(
            selectMode == EdgeSelectMode::First ? "first" :
            selectMode == EdgeSelectMode::Last ? "last" : "all"), MAX_EDGES);

        return results;
    }
}

// =============================================================================
// MeasurePos Implementations
// =============================================================================

std::vector<EdgeResult> MeasurePos(const QImage& image,
                                    const MeasureRectangle2& handle,
                                    double sigma,
                                    double threshold,
                                    const std::string& transition,
                                    const std::string& select) {
    return MeasurePosImpl(image, handle, sigma, threshold,
                          ParseTransition(transition), ParseEdgeSelect(select));
}

std::vector<EdgeResult> MeasurePos(const QImage& image,
                                    const MeasureArc& handle,
                                    double sigma,
                                    double threshold,
                                    const std::string& transition,
                                    const std::string& select) {
    return MeasurePosImpl(image, handle, sigma, threshold,
                          ParseTransition(transition), ParseEdgeSelect(select));
}

std::vector<EdgeResult> MeasurePos(const QImage& image,
                                    const MeasureConcentricCircles& handle,
                                    double sigma,
                                    double threshold,
                                    const std::string& transition,
                                    const std::string& select) {
    return MeasurePosImpl(image, handle, sigma, threshold,
                          ParseTransition(transition), ParseEdgeSelect(select));
}

// =============================================================================
// MeasurePairs Implementations
// =============================================================================

std::vector<PairResult> MeasurePairs(const QImage& image,
                                      const MeasureRectangle2& handle,
                                      double sigma,
                                      double threshold,
                                      const std::string& transition,
                                      const std::string& select) {
    // Parse transition for pair: interpret as first/second transition
    EdgeTransition trans = ParseTransition(transition);
    EdgeTransition firstTrans, secondTrans;

    if (trans == EdgeTransition::All) {
        firstTrans = EdgeTransition::Positive;
        secondTrans = EdgeTransition::Negative;
    } else if (trans == EdgeTransition::Positive) {
        firstTrans = EdgeTransition::Positive;
        secondTrans = EdgeTransition::Negative;
    } else {
        firstTrans = EdgeTransition::Negative;
        secondTrans = EdgeTransition::Positive;
    }

    return MeasurePairsImpl(image, handle, sigma, threshold,
                            firstTrans, secondTrans, ParsePairSelect(select));
}

std::vector<PairResult> MeasurePairs(const QImage& image,
                                      const MeasureArc& handle,
                                      double sigma,
                                      double threshold,
                                      const std::string& transition,
                                      const std::string& select) {
    EdgeTransition trans = ParseTransition(transition);
    EdgeTransition firstTrans, secondTrans;

    if (trans == EdgeTransition::All) {
        firstTrans = EdgeTransition::Positive;
        secondTrans = EdgeTransition::Negative;
    } else if (trans == EdgeTransition::Positive) {
        firstTrans = EdgeTransition::Positive;
        secondTrans = EdgeTransition::Negative;
    } else {
        firstTrans = EdgeTransition::Negative;
        secondTrans = EdgeTransition::Positive;
    }

    return MeasurePairsImpl(image, handle, sigma, threshold,
                            firstTrans, secondTrans, ParsePairSelect(select));
}

std::vector<PairResult> MeasurePairs(const QImage& image,
                                      const MeasureConcentricCircles& handle,
                                      double sigma,
                                      double threshold,
                                      const std::string& transition,
                                      const std::string& select) {
    EdgeTransition trans = ParseTransition(transition);
    EdgeTransition firstTrans, secondTrans;

    if (trans == EdgeTransition::All) {
        firstTrans = EdgeTransition::Positive;
        secondTrans = EdgeTransition::Negative;
    } else if (trans == EdgeTransition::Positive) {
        firstTrans = EdgeTransition::Positive;
        secondTrans = EdgeTransition::Negative;
    } else {
        firstTrans = EdgeTransition::Negative;
        secondTrans = EdgeTransition::Positive;
    }

    return MeasurePairsImpl(image, handle, sigma, threshold,
                            firstTrans, secondTrans, ParsePairSelect(select));
}

// =============================================================================
// FuzzyMeasurePos Implementations
// =============================================================================

std::vector<EdgeResult> FuzzyMeasurePos(const QImage& image,
                                         const MeasureRectangle2& handle,
                                         double sigma,
                                         double threshold,
                                         const std::string& transition,
                                         const std::string& select,
                                         double fuzzyThresh,
                                         MeasureStats* stats) {
    return FuzzyMeasurePosImpl(image, handle, sigma, threshold,
                                ParseTransition(transition), ParseEdgeSelect(select),
                                fuzzyThresh, stats);
}

std::vector<EdgeResult> FuzzyMeasurePos(const QImage& image,
                                         const MeasureArc& handle,
                                         double sigma,
                                         double threshold,
                                         const std::string& transition,
                                         const std::string& select,
                                         double fuzzyThresh,
                                         MeasureStats* stats) {
    return FuzzyMeasurePosImpl(image, handle, sigma, threshold,
                                ParseTransition(transition), ParseEdgeSelect(select),
                                fuzzyThresh, stats);
}

std::vector<EdgeResult> FuzzyMeasurePos(const QImage& image,
                                         const MeasureConcentricCircles& handle,
                                         double sigma,
                                         double threshold,
                                         const std::string& transition,
                                         const std::string& select,
                                         double fuzzyThresh,
                                         MeasureStats* stats) {
    return FuzzyMeasurePosImpl(image, handle, sigma, threshold,
                                ParseTransition(transition), ParseEdgeSelect(select),
                                fuzzyThresh, stats);
}

// =============================================================================
// FuzzyMeasurePairs Implementations
// =============================================================================

std::vector<PairResult> FuzzyMeasurePairs(const QImage& image,
                                           const MeasureRectangle2& handle,
                                           double sigma,
                                           double threshold,
                                           const std::string& transition,
                                           const std::string& select,
                                           double fuzzyThresh,
                                           MeasureStats* stats) {
    return FuzzyMeasurePairsImpl(image, handle, sigma, threshold,
                                  ParseTransition(transition), ParseEdgeSelect(select),
                                  fuzzyThresh, stats);
}

std::vector<PairResult> FuzzyMeasurePairs(const QImage& image,
                                           const MeasureArc& handle,
                                           double sigma,
                                           double threshold,
                                           const std::string& transition,
                                           const std::string& select,
                                           double fuzzyThresh,
                                           MeasureStats* stats) {
    return FuzzyMeasurePairsImpl(image, handle, sigma, threshold,
                                  ParseTransition(transition), ParseEdgeSelect(select),
                                  fuzzyThresh, stats);
}

std::vector<PairResult> FuzzyMeasurePairs(const QImage& image,
                                           const MeasureConcentricCircles& handle,
                                           double sigma,
                                           double threshold,
                                           const std::string& transition,
                                           const std::string& select,
                                           double fuzzyThresh,
                                           MeasureStats* stats) {
    return FuzzyMeasurePairsImpl(image, handle, sigma, threshold,
                                  ParseTransition(transition), ParseEdgeSelect(select),
                                  fuzzyThresh, stats);
}

// =============================================================================
// Selection and Sorting
// =============================================================================

std::vector<EdgeResult> SelectEdges(const std::vector<EdgeResult>& edges,
                                     EdgeSelectMode mode, int32_t maxCount) {
    if (edges.empty()) return {};

    std::vector<EdgeResult> result;

    switch (mode) {
        case EdgeSelectMode::All:
            result = edges;
            break;

        case EdgeSelectMode::First:
            result.push_back(edges.front());
            break;

        case EdgeSelectMode::Last:
            result.push_back(edges.back());
            break;

        case EdgeSelectMode::Strongest: {
            auto it = std::max_element(edges.begin(), edges.end(),
                [](const EdgeResult& a, const EdgeResult& b) {
                    return a.amplitude < b.amplitude;
                });
            result.push_back(*it);
            break;
        }

        case EdgeSelectMode::Weakest: {
            auto it = std::min_element(edges.begin(), edges.end(),
                [](const EdgeResult& a, const EdgeResult& b) {
                    return a.amplitude < b.amplitude;
                });
            result.push_back(*it);
            break;
        }
    }

    // Limit count
    if (result.size() > static_cast<size_t>(maxCount)) {
        result.resize(maxCount);
    }

    return result;
}

std::vector<PairResult> SelectPairs(const std::vector<PairResult>& pairs,
                                     PairSelectMode mode, int32_t maxCount) {
    if (pairs.empty()) return {};

    std::vector<PairResult> result;

    switch (mode) {
        case PairSelectMode::All:
            result = pairs;
            break;

        case PairSelectMode::First:
            result.push_back(pairs.front());
            break;

        case PairSelectMode::Last:
            result.push_back(pairs.back());
            break;

        case PairSelectMode::Strongest: {
            auto it = std::max_element(pairs.begin(), pairs.end(),
                [](const PairResult& a, const PairResult& b) {
                    return (a.first.amplitude + a.second.amplitude) <
                           (b.first.amplitude + b.second.amplitude);
                });
            result.push_back(*it);
            break;
        }

        case PairSelectMode::Widest: {
            auto it = std::max_element(pairs.begin(), pairs.end(),
                [](const PairResult& a, const PairResult& b) {
                    return a.width < b.width;
                });
            result.push_back(*it);
            break;
        }

        case PairSelectMode::Narrowest: {
            auto it = std::min_element(pairs.begin(), pairs.end(),
                [](const PairResult& a, const PairResult& b) {
                    return a.width < b.width;
                });
            result.push_back(*it);
            break;
        }
    }

    // Limit count
    if (result.size() > static_cast<size_t>(maxCount)) {
        result.resize(maxCount);
    }

    return result;
}

void SortEdges(std::vector<EdgeResult>& edges, EdgeSortBy criterion, bool ascending) {
    auto compare = [criterion, ascending](const EdgeResult& a, const EdgeResult& b) {
        double va = 0.0, vb = 0.0;
        switch (criterion) {
            case EdgeSortBy::Position:
                va = a.profilePosition;
                vb = b.profilePosition;
                break;
            case EdgeSortBy::Amplitude:
                va = a.amplitude;
                vb = b.amplitude;
                break;
            case EdgeSortBy::Score:
                va = a.score;
                vb = b.score;
                break;
        }
        return ascending ? (va < vb) : (va > vb);
    };

    std::sort(edges.begin(), edges.end(), compare);
}

void SortPairs(std::vector<PairResult>& pairs, PairSortBy criterion, bool ascending) {
    auto compare = [criterion, ascending](const PairResult& a, const PairResult& b) {
        double va = 0.0, vb = 0.0;
        switch (criterion) {
            case PairSortBy::Position:
                va = a.first.profilePosition;
                vb = b.first.profilePosition;
                break;
            case PairSortBy::Width:
                va = a.width;
                vb = b.width;
                break;
            case PairSortBy::Score:
                va = a.score;
                vb = b.score;
                break;
            case PairSortBy::Symmetry:
                va = a.symmetry;
                vb = b.symmetry;
                break;
        }
        return ascending ? (va < vb) : (va > vb);
    };

    std::sort(pairs.begin(), pairs.end(), compare);
}

// =============================================================================
// String Parameter Parsing (Halcon compatible)
// =============================================================================

EdgeTransition ParseTransition(const std::string& transition) {
    std::string lower;
    lower.reserve(transition.size());
    for (char c : transition) {
        lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }

    if (lower == "positive") return EdgeTransition::Positive;
    if (lower == "negative") return EdgeTransition::Negative;
    if (lower == "all")      return EdgeTransition::All;

    // Default
    return EdgeTransition::All;
}

EdgeSelectMode ParseEdgeSelect(const std::string& select) {
    std::string lower;
    lower.reserve(select.size());
    for (char c : select) {
        lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }

    if (lower == "first")     return EdgeSelectMode::First;
    if (lower == "last")      return EdgeSelectMode::Last;
    if (lower == "all")       return EdgeSelectMode::All;
    if (lower == "strongest") return EdgeSelectMode::Strongest;
    if (lower == "weakest")   return EdgeSelectMode::Weakest;

    // Default
    return EdgeSelectMode::All;
}

PairSelectMode ParsePairSelect(const std::string& select) {
    std::string lower;
    lower.reserve(select.size());
    for (char c : select) {
        lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }

    if (lower == "first")     return PairSelectMode::First;
    if (lower == "last")      return PairSelectMode::Last;
    if (lower == "all")       return PairSelectMode::All;
    if (lower == "strongest") return PairSelectMode::Strongest;
    if (lower == "widest")    return PairSelectMode::Widest;
    if (lower == "narrowest") return PairSelectMode::Narrowest;

    // Default
    return PairSelectMode::All;
}

ProfileInterpolation ParseInterpolation(const std::string& interpolation) {
    std::string lower;
    lower.reserve(interpolation.size());
    for (char c : interpolation) {
        lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }

    if (lower == "nearest")  return ProfileInterpolation::Nearest;
    if (lower == "bilinear") return ProfileInterpolation::Bilinear;
    if (lower == "bicubic")  return ProfileInterpolation::Bicubic;

    // Default
    return ProfileInterpolation::Bilinear;
}

std::string TransitionToString(EdgeTransition t) {
    switch (t) {
        case EdgeTransition::Positive: return "positive";
        case EdgeTransition::Negative: return "negative";
        case EdgeTransition::All:      return "all";
    }
    return "all";
}

std::string EdgeSelectToString(EdgeSelectMode m) {
    switch (m) {
        case EdgeSelectMode::First:     return "first";
        case EdgeSelectMode::Last:      return "last";
        case EdgeSelectMode::All:       return "all";
        case EdgeSelectMode::Strongest: return "strongest";
        case EdgeSelectMode::Weakest:   return "weakest";
    }
    return "all";
}

} // namespace Qi::Vision::Measure
