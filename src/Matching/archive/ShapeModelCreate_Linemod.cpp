/**
 * @file ShapeModelCreate_Linemod.cpp
 * @brief ARCHIVED: LINEMOD-based model creation (8-bin quantization)
 *
 * This file contains the LINEMOD model creation code that was archived in favor of XLD.
 * LINEMOD uses 8-bin orientation quantization which causes accuracy issues
 * at bin boundaries (e.g., 155°, 337°).
 *
 * Archived: January 2025
 * Reason: XLD (continuous gradient) provides better accuracy for all angles
 *
 * @note To restore this functionality:
 * 1. Copy CreateModelLinemod back to ShapeModelCreate.cpp
 * 2. Add the declaration to ShapeModelImpl.h
 * 3. Add LINEMOD data members (linemodFeatures_, linemodAngleTemplates_, etc.)
 * 4. Update ShapeModel.cpp to call CreateModelLinemod when useLinemod=true
 */

#if 0  // ARCHIVED CODE - NOT COMPILED

#include "ShapeModelImpl.h"
#include <cstring>

namespace Qi::Vision::Matching {
namespace Internal {

// =============================================================================
// ShapeModelImpl::CreateModelLinemod
// =============================================================================

bool ShapeModelImpl::CreateModelLinemod(const QImage& image, const Rect2i& roi, const Point2d& origin) {
    // LINEMOD-style model creation (paper-accurate implementation)
    //
    // Uses:
    // - 8-bin orientation quantization (bit flags)
    // - 3x3 neighbor voting with threshold
    // - OR spreading for robustness
    // - SIMILARITY_LUT[8][256] for O(1) scoring

    if (image.Empty() || image.Channels() != 1) {
        return false;
    }

    // Extract template region
    Rect2i actualRoi = roi;
    if (actualRoi.width <= 0 || actualRoi.height <= 0) {
        actualRoi = Rect2i(0, 0, image.Width(), image.Height());
    }

    // Clamp ROI
    actualRoi.x = std::max(0, actualRoi.x);
    actualRoi.y = std::max(0, actualRoi.y);
    actualRoi.width = std::min(actualRoi.width, image.Width() - actualRoi.x);
    actualRoi.height = std::min(actualRoi.height, image.Height() - actualRoi.y);

    if (actualRoi.width < 8 || actualRoi.height < 8) {
        return false;
    }

    templateSize_ = Size2i{actualRoi.width, actualRoi.height};

    // Set origin
    if (origin.x != 0 || origin.y != 0) {
        origin_ = origin;
    } else {
        origin_ = Point2d{actualRoi.width / 2.0, actualRoi.height / 2.0};
    }

    // Extract template image
    QImage templateImg(actualRoi.width, actualRoi.height,
                       image.Type(), image.GetChannelType());
    {
        const uint8_t* src = static_cast<const uint8_t*>(image.Data());
        uint8_t* dst = static_cast<uint8_t*>(templateImg.Data());
        const size_t srcStride = image.Stride();
        const size_t dstStride = templateImg.Stride();
        const size_t rowBytes = actualRoi.width * (image.Type() == PixelType::Float32 ? 4 : 1);

        for (int32_t y = 0; y < actualRoi.height; ++y) {
            std::memcpy(dst + y * dstStride,
                       src + (actualRoi.y + y) * srcStride + actualRoi.x * (rowBytes / actualRoi.width),
                       rowBytes);
        }
    }

    // Build LINEMOD pyramid for template
    // Compute optimal levels based on template size (not fixed!)
    // Rule: top level template should be ~15-30 pixels in smallest dimension
    LinemodPyramidParams pyramidParams;
    if (params_.numLevels > 0) {
        pyramidParams.numLevels = params_.numLevels;
    } else {
        int32_t minDim = std::min(actualRoi.width, actualRoi.height);
        int32_t levels = 1;
        while (minDim > 30 && levels < 6) {  // Stop when top level ~30px
            minDim /= 2;
            levels++;
        }
        pyramidParams.numLevels = levels;
    }
    pyramidParams.minMagnitude = static_cast<float>(params_.contrastHigh);
    pyramidParams.smoothSigma = 1.0;
    pyramidParams.spreadT = 4;
    pyramidParams.neighborThreshold = 5;
    pyramidParams.extractFeatures = true;

    LinemodPyramid pyramid;
    if (!pyramid.Build(templateImg, pyramidParams)) {
        return false;
    }

    // Extract features for each level
    int32_t numLevels = pyramid.NumLevels();
    linemodFeatures_.resize(numLevels);
    levels_.resize(numLevels);

    int32_t maxFeaturesLevel0 = 256;  // Max features at finest level
    float minDistance = 2.0f;

    for (int32_t level = 0; level < numLevels; ++level) {
        // Extract features (relative to template center)
        // Use more features at each level for better robustness
        int32_t maxFeatures = maxFeaturesLevel0 / (1 << level);
        maxFeatures = std::max(64, maxFeatures);  // Minimum 64 features per level

        linemodFeatures_[level] = pyramid.ExtractFeatures(
            level, Rect2i(), maxFeatures, minDistance);

        // Also store in LevelModel for compatibility
        auto& levelModel = levels_[level];
        levelModel.width = pyramid.GetWidth(level);
        levelModel.height = pyramid.GetHeight(level);
        levelModel.scale = pyramid.GetScale(level);

        // Convert LinemodFeature to ModelPoint for compatibility
        levelModel.points.clear();
        levelModel.points.reserve(linemodFeatures_[level].size());

        for (const auto& f : linemodFeatures_[level]) {
            // Convert 8-bin orientation back to radians
            double angle = LinemodPyramid::BinToAngle(f.ori);

            levelModel.points.emplace_back(
                static_cast<double>(f.x),
                static_cast<double>(f.y),
                angle,
                1.0,  // magnitude (not used in LINEMOD)
                static_cast<int32_t>(f.ori),
                1.0   // weight
            );
        }

        levelModel.BuildSoA();
    }

    ComputeModelBounds();

    // =========================================================================
    // Pre-generate coarse angle templates for hierarchical angle search
    // This solves the 8-bin quantization error problem for large rotations
    // =========================================================================

    // Coarse angle step (degrees) - balance between accuracy and memory
    constexpr double COARSE_ANGLE_STEP_DEG = 10.0;
    constexpr double COARSE_ANGLE_STEP_RAD = COARSE_ANGLE_STEP_DEG * PI / 180.0;

    // Calculate number of angle templates based on angleExtent
    double angleExtent = params_.angleExtent;
    if (angleExtent <= 0) {
        angleExtent = 2.0 * PI;  // Full rotation
    }

    int32_t numAngleTemplates = static_cast<int32_t>(std::ceil(angleExtent / COARSE_ANGLE_STEP_RAD));
    numAngleTemplates = std::max(1, numAngleTemplates);

    // Store template angles
    linemodTemplateAngles_.clear();
    linemodTemplateAngles_.reserve(numAngleTemplates);

    linemodAngleTemplates_.clear();
    linemodAngleTemplates_.resize(numAngleTemplates);

    // Template image center for rotation
    double templateCenterX = templateImg.Width() / 2.0;
    double templateCenterY = templateImg.Height() / 2.0;
    (void)templateCenterX;  // Unused but kept for reference
    (void)templateCenterY;

    // Generate templates for each coarse angle
    for (int32_t ai = 0; ai < numAngleTemplates; ++ai) {
        double angle = params_.angleStart + ai * COARSE_ANGLE_STEP_RAD;
        linemodTemplateAngles_.push_back(angle);

        // Pre-generate rotated features using mathematical rotation
        // This avoids image cropping issues and maintains feature consistency
        linemodAngleTemplates_[ai].resize(numLevels);
        for (int32_t level = 0; level < numLevels; ++level) {
            linemodAngleTemplates_[ai][level] =
                LinemodPyramid::RotateFeatures(linemodFeatures_[level], angle);
        }
    }

    valid_ = true;

    return true;
}

} // namespace Internal
} // namespace Qi::Vision::Matching

#endif  // ARCHIVED CODE
