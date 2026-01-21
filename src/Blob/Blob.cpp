/**
 * @file Blob.cpp
 * @brief Blob analysis implementation
 */

#include <QiVision/Blob/Blob.h>
#include <QiVision/Internal/ConnectedComponent.h>
#include <QiVision/Internal/RegionFeatures.h>
#include <QiVision/Internal/RLEOps.h>
#include <QiVision/Core/Exception.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_map>

namespace Qi::Vision::Blob {

// =============================================================================
// Connection
// =============================================================================

void Connection(const QRegion& region, std::vector<QRegion>& regions) {
    regions.clear();
    if (region.Empty()) return;

    // Get runs and find connected components
    const auto& runs = region.Runs();
    if (runs.empty()) return;

    // Build adjacency using RLE
    // Two runs are connected if they are in adjacent rows and overlap in columns
    std::vector<int32_t> labels(runs.size(), -1);
    std::vector<int32_t> parent(runs.size());
    for (size_t i = 0; i < runs.size(); ++i) {
        parent[i] = static_cast<int32_t>(i);
    }

    // Union-Find helpers
    auto find = [&parent](int32_t x) -> int32_t {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    };

    auto unite = [&parent, &find](int32_t x, int32_t y) {
        int32_t px = find(x);
        int32_t py = find(y);
        if (px != py) {
            parent[px] = py;
        }
    };

    // Build index for fast lookup by row
    std::unordered_map<int32_t, std::vector<size_t>> rowIndex;
    for (size_t i = 0; i < runs.size(); ++i) {
        rowIndex[runs[i].row].push_back(i);
    }

    // Connect adjacent runs (8-connectivity)
    for (size_t i = 0; i < runs.size(); ++i) {
        const auto& run = runs[i];

        // Check previous row
        auto it = rowIndex.find(run.row - 1);
        if (it != rowIndex.end()) {
            for (size_t j : it->second) {
                const auto& other = runs[j];
                // Check overlap (8-connectivity: overlap or adjacent by 1)
                if (other.colEnd >= run.colBegin - 1 && other.colBegin <= run.colEnd + 1) {
                    unite(static_cast<int32_t>(i), static_cast<int32_t>(j));
                }
            }
        }
    }

    // Collect components
    std::unordered_map<int32_t, std::vector<QRegion::Run>> componentRuns;
    for (size_t i = 0; i < runs.size(); ++i) {
        int32_t root = find(static_cast<int32_t>(i));
        componentRuns[root].push_back(runs[i]);
    }

    // Create result regions
    regions.reserve(componentRuns.size());
    for (auto& [label, compRuns] : componentRuns) {
        regions.emplace_back(std::move(compRuns));
    }
}

void Connection(const QImage& binaryImage,
                std::vector<QRegion>& regions,
                Connectivity connectivity) {
    regions.clear();
    if (binaryImage.Empty()) return;

    int32_t numLabels = 0;
    QImage labels = Internal::LabelConnectedComponents(binaryImage, connectivity, numLabels);

    if (numLabels == 0) return;

    regions.reserve(numLabels);

    for (int32_t label = 1; label <= numLabels; ++label) {
        QImage component = Internal::ExtractComponent(labels, label);
        QRegion region = Internal::NonZeroToRegion(component);
        if (!region.Empty()) {
            regions.push_back(std::move(region));
        }
    }
}

QRegion SelectObj(const std::vector<QRegion>& regions, int32_t index) {
    if (index < 1 || index > static_cast<int32_t>(regions.size())) {
        return QRegion();
    }
    return regions[index - 1];
}

// =============================================================================
// Region Features
// =============================================================================

void AreaCenter(const QRegion& region, int64_t& area, double& row, double& column) {
    auto features = Internal::ComputeBasicFeatures(region);
    area = features.area;
    row = features.centroidY;
    column = features.centroidX;
}

void AreaCenter(const std::vector<QRegion>& regions,
                std::vector<int64_t>& areas,
                std::vector<double>& rows,
                std::vector<double>& columns) {
    areas.resize(regions.size());
    rows.resize(regions.size());
    columns.resize(regions.size());

    for (size_t i = 0; i < regions.size(); ++i) {
        AreaCenter(regions[i], areas[i], rows[i], columns[i]);
    }
}

void SmallestRectangle1(const QRegion& region,
                         int32_t& row1, int32_t& column1,
                         int32_t& row2, int32_t& column2) {
    Rect2i bbox = Internal::ComputeBoundingBox(region);
    row1 = bbox.y;
    column1 = bbox.x;
    row2 = bbox.y + bbox.height - 1;
    column2 = bbox.x + bbox.width - 1;
}

void SmallestRectangle2(const QRegion& region,
                         double& row, double& column, double& phi,
                         double& length1, double& length2) {
    auto rect = Internal::ComputeMinAreaRect(region);
    row = rect.center.y;
    column = rect.center.x;
    phi = rect.angle;
    length1 = rect.width / 2.0;
    length2 = rect.height / 2.0;
}

void SmallestCircle(const QRegion& region,
                     double& row, double& column, double& radius) {
    auto circle = Internal::ComputeMinEnclosingCircle(region);
    row = circle.center.y;
    column = circle.center.x;
    radius = circle.radius;
}

double Circularity(const QRegion& region) {
    auto features = Internal::ComputeShapeFeatures(region);
    return features.circularity;
}

double Compactness(const QRegion& region) {
    auto features = Internal::ComputeShapeFeatures(region);
    return features.compactness;
}

double Convexity(const QRegion& region) {
    return Internal::ComputeConvexity(region);
}

double Rectangularity(const QRegion& region) {
    auto features = Internal::ComputeShapeFeatures(region);
    return features.rectangularity;
}

void EllipticAxis(const QRegion& region, double& ra, double& rb, double& phi) {
    auto ellipse = Internal::ComputeEllipseFeatures(region);
    ra = ellipse.majorAxis;
    rb = ellipse.minorAxis;
    phi = ellipse.angle;
}

double OrientationRegion(const QRegion& region) {
    return Internal::ComputeOrientation(region);
}

void MomentsRegion2nd(const QRegion& region,
                       double& m11, double& m20, double& m02,
                       double& ia, double& ib) {
    auto moments = Internal::ComputeMoments(region);
    m11 = moments.mu11;
    m20 = moments.mu20;
    m02 = moments.mu02;

    // Compute principal moments of inertia
    double trace = m20 + m02;
    double det = m20 * m02 - m11 * m11;
    double disc = std::sqrt(std::max(0.0, trace * trace / 4.0 - det));
    ia = trace / 2.0 + disc;
    ib = trace / 2.0 - disc;
}

void Eccentricity(const QRegion& region,
                   double& anisometry, double& bulkiness, double& structureFactor) {
    auto ellipse = Internal::ComputeEllipseFeatures(region);
    auto basic = Internal::ComputeBasicFeatures(region);

    double ra = ellipse.majorAxis;
    double rb = ellipse.minorAxis;

    anisometry = (rb > 0) ? ra / rb : 0.0;
    bulkiness = (basic.area > 0) ? (M_PI * ra * rb / basic.area) : 0.0;
    structureFactor = anisometry * bulkiness - 1.0;
}

// =============================================================================
// Region Selection
// =============================================================================

double GetRegionFeature(const QRegion& region, ShapeFeature feature) {
    switch (feature) {
        case ShapeFeature::Area: {
            return static_cast<double>(Internal::ComputeArea(region));
        }
        case ShapeFeature::Row:
        case ShapeFeature::Column: {
            auto centroid = Internal::ComputeRegionCentroid(region);
            return (feature == ShapeFeature::Row) ? centroid.y : centroid.x;
        }
        case ShapeFeature::Width:
        case ShapeFeature::Height: {
            auto bbox = Internal::ComputeBoundingBox(region);
            return (feature == ShapeFeature::Width) ? bbox.width : bbox.height;
        }
        case ShapeFeature::Circularity:
            return Circularity(region);
        case ShapeFeature::Compactness:
            return Compactness(region);
        case ShapeFeature::Convexity:
            return Convexity(region);
        case ShapeFeature::Rectangularity:
            return Rectangularity(region);
        case ShapeFeature::Elongation: {
            auto ellipse = Internal::ComputeEllipseFeatures(region);
            return (ellipse.minorAxis > 0) ? ellipse.majorAxis / ellipse.minorAxis : 1.0;
        }
        case ShapeFeature::Orientation:
            return OrientationRegion(region);
        case ShapeFeature::Ra:
        case ShapeFeature::Rb:
        case ShapeFeature::Phi: {
            auto ellipse = Internal::ComputeEllipseFeatures(region);
            if (feature == ShapeFeature::Ra) return ellipse.majorAxis;
            if (feature == ShapeFeature::Rb) return ellipse.minorAxis;
            return ellipse.angle;
        }
        case ShapeFeature::Anisometry:
        case ShapeFeature::Bulkiness:
        case ShapeFeature::StructureFactor: {
            double ani, bulk, sf;
            Eccentricity(region, ani, bulk, sf);
            if (feature == ShapeFeature::Anisometry) return ani;
            if (feature == ShapeFeature::Bulkiness) return bulk;
            return sf;
        }
        case ShapeFeature::OuterRadius: {
            auto circle = Internal::ComputeMinEnclosingCircle(region);
            return circle.radius;
        }
        case ShapeFeature::InnerRadius: {
            double row, col, radius;
            InnerCircle(region, row, col, radius);
            return radius;
        }
        case ShapeFeature::Holes:
            return static_cast<double>(CountHoles(region));
        default:
            return 0.0;
    }
}

std::vector<double> GetRegionFeatures(const std::vector<QRegion>& regions,
                                       ShapeFeature feature) {
    std::vector<double> result;
    result.reserve(regions.size());
    for (const auto& region : regions) {
        result.push_back(GetRegionFeature(region, feature));
    }
    return result;
}

void SelectShape(const std::vector<QRegion>& regions,
                 std::vector<QRegion>& selected,
                 ShapeFeature feature,
                 SelectOperation /*operation*/,
                 double minValue,
                 double maxValue) {
    selected.clear();
    selected.reserve(regions.size());

    for (const auto& region : regions) {
        double value = GetRegionFeature(region, feature);
        if (value >= minValue && value <= maxValue) {
            selected.push_back(region);
        }
    }
}

ShapeFeature ParseShapeFeature(const std::string& name) {
    std::string lower = name;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    if (lower == "area") return ShapeFeature::Area;
    if (lower == "row") return ShapeFeature::Row;
    if (lower == "column" || lower == "col") return ShapeFeature::Column;
    if (lower == "width") return ShapeFeature::Width;
    if (lower == "height") return ShapeFeature::Height;
    if (lower == "circularity") return ShapeFeature::Circularity;
    if (lower == "compactness") return ShapeFeature::Compactness;
    if (lower == "convexity") return ShapeFeature::Convexity;
    if (lower == "rectangularity") return ShapeFeature::Rectangularity;
    if (lower == "elongation") return ShapeFeature::Elongation;
    if (lower == "orientation" || lower == "phi") return ShapeFeature::Orientation;
    if (lower == "ra") return ShapeFeature::Ra;
    if (lower == "rb") return ShapeFeature::Rb;
    if (lower == "anisometry") return ShapeFeature::Anisometry;
    if (lower == "bulkiness") return ShapeFeature::Bulkiness;
    if (lower == "structure_factor") return ShapeFeature::StructureFactor;
    if (lower == "outer_radius") return ShapeFeature::OuterRadius;
    if (lower == "inner_radius") return ShapeFeature::InnerRadius;
    if (lower == "holes") return ShapeFeature::Holes;

    return ShapeFeature::Area; // default
}

std::string GetShapeFeatureName(ShapeFeature feature) {
    switch (feature) {
        case ShapeFeature::Area: return "area";
        case ShapeFeature::Row: return "row";
        case ShapeFeature::Column: return "column";
        case ShapeFeature::Width: return "width";
        case ShapeFeature::Height: return "height";
        case ShapeFeature::Circularity: return "circularity";
        case ShapeFeature::Compactness: return "compactness";
        case ShapeFeature::Convexity: return "convexity";
        case ShapeFeature::Rectangularity: return "rectangularity";
        case ShapeFeature::Elongation: return "elongation";
        case ShapeFeature::Orientation: return "orientation";
        case ShapeFeature::Ra: return "ra";
        case ShapeFeature::Rb: return "rb";
        case ShapeFeature::Phi: return "phi";
        case ShapeFeature::Anisometry: return "anisometry";
        case ShapeFeature::Bulkiness: return "bulkiness";
        case ShapeFeature::StructureFactor: return "structure_factor";
        case ShapeFeature::OuterRadius: return "outer_radius";
        case ShapeFeature::InnerRadius: return "inner_radius";
        case ShapeFeature::Holes: return "holes";
        default: return "unknown";
    }
}

void SelectShape(const std::vector<QRegion>& regions,
                 std::vector<QRegion>& selected,
                 const std::string& features,
                 const std::string& operation,
                 double minValue,
                 double maxValue) {
    ShapeFeature feature = ParseShapeFeature(features);
    SelectOperation op = SelectOperation::And;
    std::string lowerOp = operation;
    std::transform(lowerOp.begin(), lowerOp.end(), lowerOp.begin(), ::tolower);
    if (lowerOp == "or") {
        op = SelectOperation::Or;
    }
    SelectShape(regions, selected, feature, op, minValue, maxValue);
}

void SelectShapeArea(const std::vector<QRegion>& regions,
                     std::vector<QRegion>& selected,
                     int64_t minArea,
                     int64_t maxArea) {
    SelectShape(regions, selected, ShapeFeature::Area, SelectOperation::And,
                static_cast<double>(minArea), static_cast<double>(maxArea));
}

void SelectShapeCircularity(const std::vector<QRegion>& regions,
                            std::vector<QRegion>& selected,
                            double minCirc,
                            double maxCirc) {
    SelectShape(regions, selected, ShapeFeature::Circularity, SelectOperation::And, minCirc, maxCirc);
}

void SelectShapeRectangularity(const std::vector<QRegion>& regions,
                               std::vector<QRegion>& selected,
                               double minRect,
                               double maxRect) {
    SelectShape(regions, selected, ShapeFeature::Rectangularity, SelectOperation::And, minRect, maxRect);
}

// =============================================================================
// Region Sorting
// =============================================================================

void SortRegion(const std::vector<QRegion>& regions,
                std::vector<QRegion>& sorted,
                SortMode mode,
                bool ascending) {
    sorted.clear();
    if (regions.empty() || mode == SortMode::None) {
        sorted = regions;
        return;
    }

    std::vector<std::pair<double, size_t>> sortKeys;
    sortKeys.reserve(regions.size());

    for (size_t i = 0; i < regions.size(); ++i) {
        double key = 0.0;
        switch (mode) {
            case SortMode::Area:
                key = static_cast<double>(Internal::ComputeArea(regions[i]));
                break;
            case SortMode::Row:
            case SortMode::Column: {
                auto centroid = Internal::ComputeRegionCentroid(regions[i]);
                key = (mode == SortMode::Row) ? centroid.y : centroid.x;
                break;
            }
            case SortMode::FirstPoint: {
                const auto& runs = regions[i].Runs();
                if (!runs.empty()) {
                    key = runs.front().row * 100000 + runs.front().colBegin;
                }
                break;
            }
            case SortMode::LastPoint: {
                const auto& runs = regions[i].Runs();
                if (!runs.empty()) {
                    key = runs.back().row * 100000 + runs.back().colEnd;
                }
                break;
            }
            default:
                break;
        }
        sortKeys.emplace_back(key, i);
    }

    if (ascending) {
        std::sort(sortKeys.begin(), sortKeys.end());
    } else {
        std::sort(sortKeys.begin(), sortKeys.end(), std::greater<>());
    }

    sorted.reserve(regions.size());
    for (const auto& [key, idx] : sortKeys) {
        sorted.push_back(regions[idx]);
    }
}

void SortRegion(const std::vector<QRegion>& regions,
                std::vector<QRegion>& sorted,
                const std::string& sortMode,
                const std::string& order,
                const std::string& /*rowOrCol*/) {
    std::string lowerMode = sortMode;
    std::transform(lowerMode.begin(), lowerMode.end(), lowerMode.begin(), ::tolower);

    SortMode mode = SortMode::None;
    if (lowerMode == "area") mode = SortMode::Area;
    else if (lowerMode == "first_point") mode = SortMode::FirstPoint;
    else if (lowerMode == "last_point") mode = SortMode::LastPoint;
    else if (lowerMode == "character" || lowerMode == "row") mode = SortMode::Row;
    else if (lowerMode == "column") mode = SortMode::Column;

    std::string lowerOrder = order;
    std::transform(lowerOrder.begin(), lowerOrder.end(), lowerOrder.begin(), ::tolower);
    bool ascending = (lowerOrder != "false" && lowerOrder != "descending");

    SortRegion(regions, sorted, mode, ascending);
}

// =============================================================================
// New Region Features
// =============================================================================

void InnerCircle(const QRegion& region,
                  double& row, double& column, double& radius) {
    if (region.Empty()) {
        row = column = radius = 0.0;
        return;
    }

    // Get bounding box
    Rect2i bbox = Internal::ComputeBoundingBox(region);
    if (bbox.width <= 0 || bbox.height <= 0) {
        row = column = radius = 0.0;
        return;
    }

    // Convert region to binary image
    QImage binary(bbox.width, bbox.height, PixelType::UInt8, ChannelType::Gray);
    std::memset(binary.Data(), 0, binary.Height() * binary.Stride());

    uint8_t* data = static_cast<uint8_t*>(binary.Data());
    size_t stride = binary.Stride();

    for (const auto& run : region.Runs()) {
        int32_t y = run.row - bbox.y;
        if (y < 0 || y >= bbox.height) continue;
        int32_t x0 = std::max(0, run.colBegin - bbox.x);
        int32_t x1 = std::min(bbox.width - 1, run.colEnd - bbox.x);
        for (int32_t x = x0; x <= x1; ++x) {
            data[y * stride + x] = 255;
        }
    }

    // Compute distance transform (simple L2 approximation using chamfer 3-4)
    std::vector<float> dist(bbox.width * bbox.height, 0.0f);

    // Initialize: 0 for foreground, infinity for background
    const float INF = 1e9f;
    for (int32_t y = 0; y < bbox.height; ++y) {
        for (int32_t x = 0; x < bbox.width; ++x) {
            dist[y * bbox.width + x] = (data[y * stride + x] > 0) ? INF : 0.0f;
        }
    }

    // Forward pass
    for (int32_t y = 0; y < bbox.height; ++y) {
        for (int32_t x = 0; x < bbox.width; ++x) {
            float& d = dist[y * bbox.width + x];
            if (x > 0) d = std::min(d, dist[y * bbox.width + (x-1)] + 1.0f);
            if (y > 0) d = std::min(d, dist[(y-1) * bbox.width + x] + 1.0f);
            if (x > 0 && y > 0) d = std::min(d, dist[(y-1) * bbox.width + (x-1)] + 1.414f);
            if (x < bbox.width-1 && y > 0) d = std::min(d, dist[(y-1) * bbox.width + (x+1)] + 1.414f);
        }
    }

    // Backward pass
    for (int32_t y = bbox.height - 1; y >= 0; --y) {
        for (int32_t x = bbox.width - 1; x >= 0; --x) {
            float& d = dist[y * bbox.width + x];
            if (x < bbox.width-1) d = std::min(d, dist[y * bbox.width + (x+1)] + 1.0f);
            if (y < bbox.height-1) d = std::min(d, dist[(y+1) * bbox.width + x] + 1.0f);
            if (x < bbox.width-1 && y < bbox.height-1)
                d = std::min(d, dist[(y+1) * bbox.width + (x+1)] + 1.414f);
            if (x > 0 && y < bbox.height-1)
                d = std::min(d, dist[(y+1) * bbox.width + (x-1)] + 1.414f);
        }
    }

    // Find maximum distance (center of inscribed circle)
    float maxDist = 0.0f;
    int32_t maxX = 0, maxY = 0;
    for (int32_t y = 0; y < bbox.height; ++y) {
        for (int32_t x = 0; x < bbox.width; ++x) {
            if (dist[y * bbox.width + x] > maxDist) {
                maxDist = dist[y * bbox.width + x];
                maxX = x;
                maxY = y;
            }
        }
    }

    row = maxY + bbox.y;
    column = maxX + bbox.x;
    radius = maxDist;
}

double ContourLength(const QRegion& region) {
    if (region.Empty()) return 0.0;
    auto features = Internal::ComputeBasicFeatures(region);
    return features.perimeter;
}

int32_t CountHoles(const QRegion& region) {
    if (region.Empty()) return 0;

    // Fill the region and count the difference
    QRegion filled;
    FillUp(region, filled);
    int64_t filledArea = Internal::ComputeArea(filled);
    int64_t originalArea = Internal::ComputeArea(region);

    if (filledArea == originalArea) return 0;

    // Get holes and count connected components
    std::vector<QRegion> holes;
    GetHoles(region, holes);
    return static_cast<int32_t>(holes.size());
}

int32_t EulerNumber(const QRegion& region) {
    // Euler number = 1 - number_of_holes (for a single connected region)
    return 1 - CountHoles(region);
}

void FillUp(const QRegion& region, QRegion& filled) {
    if (region.Empty()) {
        filled = region;
        return;
    }

    // Get bounding box
    Rect2i bbox = Internal::ComputeBoundingBox(region);

    // Create binary image with padding
    int32_t padW = bbox.width + 2;
    int32_t padH = bbox.height + 2;

    QImage binary(padW, padH, PixelType::UInt8, ChannelType::Gray);
    std::memset(binary.Data(), 0, binary.Height() * binary.Stride());

    uint8_t* data = static_cast<uint8_t*>(binary.Data());
    size_t stride = binary.Stride();

    // Fill region (with offset for padding)
    for (const auto& run : region.Runs()) {
        int32_t y = run.row - bbox.y + 1;
        if (y < 0 || y >= padH) continue;
        int32_t x0 = std::max(0, run.colBegin - bbox.x + 1);
        int32_t x1 = std::min(padW - 1, run.colEnd - bbox.x + 1);
        for (int32_t x = x0; x <= x1; ++x) {
            data[y * stride + x] = 255;
        }
    }

    // Flood fill from corners (mark exterior as 128)
    std::vector<std::pair<int32_t, int32_t>> stack;
    stack.push_back({0, 0});

    while (!stack.empty()) {
        auto [x, y] = stack.back();
        stack.pop_back();

        if (x < 0 || x >= padW || y < 0 || y >= padH) continue;
        if (data[y * stride + x] != 0) continue;

        data[y * stride + x] = 128;
        stack.push_back({x + 1, y});
        stack.push_back({x - 1, y});
        stack.push_back({x, y + 1});
        stack.push_back({x, y - 1});
    }

    // Anything still 0 is a hole - fill it
    for (int32_t y = 1; y < padH - 1; ++y) {
        for (int32_t x = 1; x < padW - 1; ++x) {
            if (data[y * stride + x] == 0) {
                data[y * stride + x] = 255;  // Fill hole
            }
        }
    }

    // Convert back to region
    std::vector<QRegion::Run> runs;
    for (int32_t y = 1; y < padH - 1; ++y) {
        int32_t runStart = -1;
        for (int32_t x = 1; x < padW - 1; ++x) {
            if (data[y * stride + x] == 255) {
                if (runStart < 0) runStart = x;
            } else {
                if (runStart >= 0) {
                    runs.push_back({y - 1 + bbox.y, runStart - 1 + bbox.x, x - 2 + bbox.x});
                    runStart = -1;
                }
            }
        }
        if (runStart >= 0) {
            runs.push_back({y - 1 + bbox.y, runStart - 1 + bbox.x, padW - 3 + bbox.x});
        }
    }

    filled = QRegion(std::move(runs));
}

void GetHoles(const QRegion& region, std::vector<QRegion>& holes) {
    holes.clear();
    if (region.Empty()) return;

    QRegion filled;
    FillUp(region, filled);
    int64_t filledArea = Internal::ComputeArea(filled);
    int64_t originalArea = Internal::ComputeArea(region);

    if (filledArea == originalArea) return;

    // Compute difference: filled - original
    QRegion holeRegion = filled.Difference(region);
    if (holeRegion.Empty()) return;

    // Find connected components of holes
    Connection(holeRegion, holes);
}

// =============================================================================
// Additional Selection Functions
// =============================================================================

void SelectShapeStd(const std::vector<QRegion>& regions,
                    std::vector<QRegion>& selected,
                    ShapeFeature feature,
                    double deviationFactor) {
    selected.clear();
    if (regions.empty() || deviationFactor <= 0) return;

    // Compute feature values
    std::vector<double> values = GetRegionFeatures(regions, feature);

    // Compute mean
    double sum = 0.0;
    for (double v : values) sum += v;
    double mean = sum / values.size();

    // Compute standard deviation
    double sqSum = 0.0;
    for (double v : values) sqSum += (v - mean) * (v - mean);
    double stdDev = std::sqrt(sqSum / values.size());

    // Select regions within range
    double minVal = mean - deviationFactor * stdDev;
    double maxVal = mean + deviationFactor * stdDev;

    for (size_t i = 0; i < regions.size(); ++i) {
        if (values[i] >= minVal && values[i] <= maxVal) {
            selected.push_back(regions[i]);
        }
    }
}

void SelectShapeMulti(const std::vector<QRegion>& regions,
                      std::vector<QRegion>& selected,
                      const std::vector<ShapeFeature>& features,
                      SelectOperation operation,
                      const std::vector<double>& minValues,
                      const std::vector<double>& maxValues) {
    selected.clear();
    if (regions.empty() || features.empty()) return;
    if (features.size() != minValues.size() || features.size() != maxValues.size()) {
        return;
    }

    for (const auto& region : regions) {
        bool matches = (operation == SelectOperation::And);

        for (size_t i = 0; i < features.size(); ++i) {
            double value = GetRegionFeature(region, features[i]);
            bool inRange = (value >= minValues[i] && value <= maxValues[i]);

            if (operation == SelectOperation::And) {
                if (!inRange) {
                    matches = false;
                    break;
                }
            } else {  // Or
                if (inRange) {
                    matches = true;
                    break;
                }
            }
        }

        if (matches) {
            selected.push_back(region);
        }
    }
}

void SelectShapeConvexity(const std::vector<QRegion>& regions,
                          std::vector<QRegion>& selected,
                          double minConvex,
                          double maxConvex) {
    SelectShape(regions, selected, ShapeFeature::Convexity, SelectOperation::And, minConvex, maxConvex);
}

void SelectShapeElongation(const std::vector<QRegion>& regions,
                           std::vector<QRegion>& selected,
                           double minElong,
                           double maxElong) {
    SelectShape(regions, selected, ShapeFeature::Elongation, SelectOperation::And, minElong, maxElong);
}

void SelectShapeProto(const std::vector<QRegion>& regions,
                      std::vector<QRegion>& selected,
                      int32_t n,
                      bool largest) {
    selected.clear();
    if (regions.empty() || n <= 0) return;
    if (n >= static_cast<int32_t>(regions.size())) {
        selected = regions;
        return;
    }

    // Get areas and sort
    std::vector<std::pair<int64_t, size_t>> areaIdx;
    areaIdx.reserve(regions.size());
    for (size_t i = 0; i < regions.size(); ++i) {
        areaIdx.emplace_back(Internal::ComputeArea(regions[i]), i);
    }

    if (largest) {
        std::partial_sort(areaIdx.begin(), areaIdx.begin() + n, areaIdx.end(),
                         [](const auto& a, const auto& b) { return a.first > b.first; });
    } else {
        std::partial_sort(areaIdx.begin(), areaIdx.begin() + n, areaIdx.end(),
                         [](const auto& a, const auto& b) { return a.first < b.first; });
    }

    selected.reserve(n);
    for (int32_t i = 0; i < n; ++i) {
        selected.push_back(regions[areaIdx[i].second]);
    }
}

} // namespace Qi::Vision::Blob
