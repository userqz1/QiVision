/**
 * @file NCCModelCreate.cpp
 * @brief NCCModel creation implementation
 *
 * Contains:
 * - Model creation from image/ROI/region
 * - Pyramid level building
 * - Rotated template precomputation
 * - Template statistics computation
 */

#include "NCCModelImpl.h"

#include <QiVision/Internal/Pyramid.h>
#include <QiVision/Internal/Interpolate.h>
#include <QiVision/Internal/AffineTransform.h>
#include <QiVision/Core/Constants.h>

#include <cmath>
#include <algorithm>
#include <numeric>

namespace Qi::Vision::Matching {
namespace Internal {

// =============================================================================
// NCCLevelModel Implementation
// =============================================================================

void NCCLevelModel::ComputeStatistics()
{
    if (data.empty()) {
        mean = 0.0;
        stddev = 0.0;
        sumSq = 0.0;
        numPixels = 0;
        return;
    }

    numPixels = static_cast<int32_t>(data.size());

    // Compute mean
    double sum = 0.0;
    for (float v : data) {
        sum += v;
    }
    mean = sum / numPixels;

    // Compute standard deviation and zero-mean template
    zeroMean.resize(data.size());
    sumSq = 0.0;
    for (size_t i = 0; i < data.size(); ++i) {
        double diff = data[i] - mean;
        zeroMean[i] = static_cast<float>(diff);
        sumSq += diff * diff;
    }

    stddev = (numPixels > 1) ? std::sqrt(sumSq / numPixels) : 0.0;
}

void NCCLevelModel::ComputeStatisticsWithMask()
{
    if (data.empty() || mask.empty()) {
        ComputeStatistics();
        return;
    }

    // Count valid pixels and compute mean
    numPixels = 0;
    double sum = 0.0;
    for (size_t i = 0; i < data.size(); ++i) {
        if (mask[i] > 0) {
            sum += data[i];
            numPixels++;
        }
    }

    if (numPixels == 0) {
        mean = 0.0;
        stddev = 0.0;
        sumSq = 0.0;
        return;
    }

    mean = sum / numPixels;

    // Compute standard deviation and zero-mean template
    zeroMean.resize(data.size());
    sumSq = 0.0;
    for (size_t i = 0; i < data.size(); ++i) {
        if (mask[i] > 0) {
            double diff = data[i] - mean;
            zeroMean[i] = static_cast<float>(diff);
            sumSq += diff * diff;
        } else {
            zeroMean[i] = 0.0f;
        }
    }

    stddev = (numPixels > 1) ? std::sqrt(sumSq / numPixels) : 0.0;
}

// =============================================================================
// IntegralImagePyramid Implementation
// =============================================================================

bool IntegralImagePyramid::Build(const Qi::Vision::Internal::ImagePyramid& pyramid)
{
    Clear();

    if (pyramid.Empty()) {
        return false;
    }

    int32_t numLevels = pyramid.NumLevels();
    levels_.resize(numLevels);
    widths_.resize(numLevels);
    heights_.resize(numLevels);

    for (int32_t i = 0; i < numLevels; ++i) {
        const auto& level = pyramid.GetLevel(i);

        // Create QImage from pyramid level data for integral computation
        QImage levelImage(level.width, level.height, PixelType::UInt8, ChannelType::Gray);
        uint8_t* dst = static_cast<uint8_t*>(levelImage.Data());
        for (int32_t y = 0; y < level.height; ++y) {
            for (int32_t x = 0; x < level.width; ++x) {
                float val = level.data[y * level.width + x];
                dst[y * levelImage.Stride() + x] = static_cast<uint8_t>(
                    std::clamp(val, 0.0f, 255.0f));
            }
        }

        if (!levels_[i].Compute(levelImage, true)) {
            Clear();
            return false;
        }

        widths_[i] = level.width;
        heights_[i] = level.height;
    }

    return true;
}

void IntegralImagePyramid::Clear()
{
    levels_.clear();
    widths_.clear();
    heights_.clear();
}

const Qi::Vision::Internal::IntegralImage& IntegralImagePyramid::GetLevel(int32_t level) const
{
    static Qi::Vision::Internal::IntegralImage empty;
    if (level < 0 || level >= static_cast<int32_t>(levels_.size())) {
        return empty;
    }
    return levels_[level];
}

int32_t IntegralImagePyramid::GetWidth(int32_t level) const
{
    if (level < 0 || level >= static_cast<int32_t>(widths_.size())) {
        return 0;
    }
    return widths_[level];
}

int32_t IntegralImagePyramid::GetHeight(int32_t level) const
{
    if (level < 0 || level >= static_cast<int32_t>(heights_.size())) {
        return 0;
    }
    return heights_[level];
}

// =============================================================================
// NCCModelImpl - Model Creation
// =============================================================================

bool NCCModelImpl::CreateModel(const QImage& image)
{
    if (!image.IsValid()) {
        return false;
    }

    // Use full image as template
    templateSize_.width = image.Width();
    templateSize_.height = image.Height();
    origin_.x = templateSize_.width * 0.5;
    origin_.y = templateSize_.height * 0.5;
    hasMask_ = false;

    // Convert image to float
    std::vector<float> templateData(templateSize_.width * templateSize_.height);
    const uint8_t* src = static_cast<const uint8_t*>(image.Data());
    int32_t stride = image.Stride();

    for (int32_t y = 0; y < templateSize_.height; ++y) {
        for (int32_t x = 0; x < templateSize_.width; ++x) {
            templateData[y * templateSize_.width + x] =
                static_cast<float>(src[y * stride + x]);
        }
    }

    // Build pyramid levels
    BuildPyramidLevels(templateData, templateSize_.width, templateSize_.height);

    // Precompute rotated templates
    PrecomputeRotatedTemplates();

    valid_ = !levels_.empty();
    return valid_;
}

bool NCCModelImpl::CreateModel(const QImage& image, const Rect2i& roi)
{
    if (!image.IsValid()) {
        return false;
    }

    // Validate ROI
    int32_t x1 = std::max(0, roi.x);
    int32_t y1 = std::max(0, roi.y);
    int32_t x2 = std::min(image.Width(), roi.x + roi.width);
    int32_t y2 = std::min(image.Height(), roi.y + roi.height);

    if (x2 <= x1 || y2 <= y1) {
        return false;
    }

    templateSize_.width = x2 - x1;
    templateSize_.height = y2 - y1;
    origin_.x = templateSize_.width * 0.5;
    origin_.y = templateSize_.height * 0.5;
    hasMask_ = false;

    // Extract ROI to float
    std::vector<float> templateData(templateSize_.width * templateSize_.height);
    const uint8_t* src = static_cast<const uint8_t*>(image.Data());
    int32_t stride = image.Stride();

    for (int32_t y = 0; y < templateSize_.height; ++y) {
        for (int32_t x = 0; x < templateSize_.width; ++x) {
            templateData[y * templateSize_.width + x] =
                static_cast<float>(src[(y + y1) * stride + (x + x1)]);
        }
    }

    // Build pyramid levels
    BuildPyramidLevels(templateData, templateSize_.width, templateSize_.height);

    // Precompute rotated templates
    PrecomputeRotatedTemplates();

    valid_ = !levels_.empty();
    return valid_;
}

bool NCCModelImpl::CreateModel(const QImage& image, const QRegion& region)
{
    if (!image.IsValid() || region.Empty()) {
        return false;
    }

    // Get bounding box
    auto bbox = region.BoundingBox();
    int32_t x1 = std::max(0, bbox.x);
    int32_t y1 = std::max(0, bbox.y);
    int32_t x2 = std::min(image.Width(), bbox.x + bbox.width);
    int32_t y2 = std::min(image.Height(), bbox.y + bbox.height);

    if (x2 <= x1 || y2 <= y1) {
        return false;
    }

    templateSize_.width = x2 - x1;
    templateSize_.height = y2 - y1;
    origin_.x = templateSize_.width * 0.5;
    origin_.y = templateSize_.height * 0.5;
    hasMask_ = true;

    // Extract ROI and mask
    std::vector<float> templateData(templateSize_.width * templateSize_.height, 0.0f);
    std::vector<uint8_t> mask(templateSize_.width * templateSize_.height, 0);

    const uint8_t* src = static_cast<const uint8_t*>(image.Data());
    int32_t stride = image.Stride();

    for (int32_t y = 0; y < templateSize_.height; ++y) {
        for (int32_t x = 0; x < templateSize_.width; ++x) {
            int32_t imgX = x + x1;
            int32_t imgY = y + y1;

            if (region.Contains(imgX, imgY)) {
                templateData[y * templateSize_.width + x] =
                    static_cast<float>(src[imgY * stride + imgX]);
                mask[y * templateSize_.width + x] = 255;
            }
        }
    }

    // Build pyramid levels with mask
    BuildPyramidLevels(templateData, templateSize_.width, templateSize_.height, mask);

    // Precompute rotated templates
    PrecomputeRotatedTemplates();

    valid_ = !levels_.empty();
    return valid_;
}

void NCCModelImpl::BuildPyramidLevels(const std::vector<float>& templateData,
                                       int32_t width, int32_t height,
                                       const std::vector<uint8_t>& mask)
{
    levels_.clear();

    // Determine number of levels
    int32_t numLevels = params_.numLevels;
    if (numLevels <= 0) {
        numLevels = Qi::Vision::Internal::ComputeNumLevels(width, height, 0.5, 8);
    }

    // Build Gaussian pyramid for template
    Qi::Vision::Internal::PyramidParams pyramidParams;
    pyramidParams.numLevels = numLevels;
    pyramidParams.sigma = 1.0;

    auto pyramid = Qi::Vision::Internal::BuildGaussianPyramid(
        templateData.data(), width, height, pyramidParams);

    // Create level models
    levels_.resize(pyramid.NumLevels());

    for (int32_t i = 0; i < pyramid.NumLevels(); ++i) {
        const auto& pyrLevel = pyramid.GetLevel(i);

        auto& level = levels_[i];
        level.width = pyrLevel.width;
        level.height = pyrLevel.height;
        level.scale = pyrLevel.scale;
        level.data = pyrLevel.data;

        // Downsample mask if present
        if (!mask.empty()) {
            if (i == 0) {
                level.mask = mask;
            } else {
                // Simple nearest-neighbor downsample for mask
                int32_t prevWidth = levels_[i-1].width;
                int32_t prevHeight = levels_[i-1].height;
                const auto& prevMask = levels_[i-1].mask;

                level.mask.resize(level.width * level.height);
                for (int32_t y = 0; y < level.height; ++y) {
                    for (int32_t x = 0; x < level.width; ++x) {
                        int32_t srcX = std::min(x * 2, prevWidth - 1);
                        int32_t srcY = std::min(y * 2, prevHeight - 1);
                        level.mask[y * level.width + x] = prevMask[srcY * prevWidth + srcX];
                    }
                }
            }
            level.ComputeStatisticsWithMask();
        } else {
            level.ComputeStatistics();
        }
    }
}

void NCCModelImpl::PrecomputeRotatedTemplates()
{
    // Build search angle list
    BuildSearchAngles();

    if (searchAngles_.empty() || levels_.empty()) {
        return;
    }

    rotatedTemplates_.resize(levels_.size());

    for (size_t levelIdx = 0; levelIdx < levels_.size(); ++levelIdx) {
        const auto& level = levels_[levelIdx];

        rotatedTemplates_[levelIdx].resize(searchAngles_.size());

        for (size_t angleIdx = 0; angleIdx < searchAngles_.size(); ++angleIdx) {
            rotatedTemplates_[levelIdx][angleIdx] =
                RotateTemplate(level, searchAngles_[angleIdx]);
        }
    }
}

RotatedTemplate NCCModelImpl::RotateTemplate(const NCCLevelModel& level, double angle)
{
    RotatedTemplate result;
    result.angle = angle;

    // For angle == 0, just copy the template
    if (std::abs(angle) < 1e-6) {
        result.width = level.width;
        result.height = level.height;
        result.offsetX = 0;
        result.offsetY = 0;
        result.data = level.zeroMean;
        result.mask = level.mask;
        result.mean = level.mean;
        result.stddev = level.stddev;
        result.numPixels = level.numPixels;
        return result;
    }

    // Compute rotated bounding box
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    double cx = level.width * 0.5;
    double cy = level.height * 0.5;

    // Transform corners
    double corners[4][2] = {
        {-cx, -cy},
        { cx - 1, -cy},
        { cx - 1,  cy - 1},
        {-cx,  cy - 1}
    };

    double minX = 1e9, maxX = -1e9;
    double minY = 1e9, maxY = -1e9;

    for (int i = 0; i < 4; ++i) {
        double rx = cosA * corners[i][0] - sinA * corners[i][1];
        double ry = sinA * corners[i][0] + cosA * corners[i][1];
        minX = std::min(minX, rx);
        maxX = std::max(maxX, rx);
        minY = std::min(minY, ry);
        maxY = std::max(maxY, ry);
    }

    result.width = static_cast<int32_t>(std::ceil(maxX - minX)) + 1;
    result.height = static_cast<int32_t>(std::ceil(maxY - minY)) + 1;
    result.offsetX = static_cast<int32_t>(-minX);
    result.offsetY = static_cast<int32_t>(-minY);

    // Allocate rotated template
    result.data.resize(result.width * result.height, 0.0f);
    if (hasMask_) {
        result.mask.resize(result.width * result.height, 0);
    }

    // Rotate using inverse mapping
    double invCosA = cosA;   // cos(-angle) = cos(angle)
    double invSinA = -sinA;  // sin(-angle) = -sin(angle)

    double rcx = result.width * 0.5;
    double rcy = result.height * 0.5;

    double sum = 0.0;
    int32_t count = 0;

    for (int32_t y = 0; y < result.height; ++y) {
        for (int32_t x = 0; x < result.width; ++x) {
            // Map to source coordinates
            double dx = x - rcx;
            double dy = y - rcy;
            double srcX = invCosA * dx - invSinA * dy + cx;
            double srcY = invSinA * dx + invCosA * dy + cy;

            // Check bounds
            if (srcX >= 0 && srcX < level.width - 1 &&
                srcY >= 0 && srcY < level.height - 1) {

                // Bilinear interpolation
                int32_t x0 = static_cast<int32_t>(srcX);
                int32_t y0 = static_cast<int32_t>(srcY);
                double fx = srcX - x0;
                double fy = srcY - y0;

                double v00 = level.zeroMean[y0 * level.width + x0];
                double v10 = level.zeroMean[y0 * level.width + x0 + 1];
                double v01 = level.zeroMean[(y0 + 1) * level.width + x0];
                double v11 = level.zeroMean[(y0 + 1) * level.width + x0 + 1];

                double val = v00 * (1 - fx) * (1 - fy) +
                             v10 * fx * (1 - fy) +
                             v01 * (1 - fx) * fy +
                             v11 * fx * fy;

                result.data[y * result.width + x] = static_cast<float>(val);

                // Handle mask
                if (hasMask_) {
                    uint8_t m = level.mask[y0 * level.width + x0];
                    result.mask[y * result.width + x] = m;
                    if (m > 0) {
                        sum += val * val;
                        count++;
                    }
                } else {
                    sum += val * val;
                    count++;
                }
            }
        }
    }

    result.numPixels = count;
    result.mean = 0.0;  // Already zero-mean
    result.stddev = (count > 0) ? std::sqrt(sum / count) : 0.0;

    return result;
}

void NCCModelImpl::BuildSearchAngles()
{
    searchAngles_.clear();

    double angleStart = params_.angleStart;
    double angleExtent = params_.angleExtent;
    double angleStep = params_.angleStep;

    // Handle default values
    if (std::abs(angleExtent) < 1e-6) {
        angleExtent = 2.0 * PI;
    }

    if (angleStep <= 0) {
        // Auto-compute: ~1 pixel arc at template edge
        double radius = std::max(templateSize_.width, templateSize_.height) * 0.5;
        if (radius < 10) radius = 10;
        angleStep = std::atan(1.0 / radius);

        // Clamp to reasonable range
        constexpr double MIN_ANGLE_STEP = 0.005;  // ~0.3 degrees
        constexpr double MAX_ANGLE_STEP = 0.1;    // ~5.7 degrees
        angleStep = std::clamp(angleStep, MIN_ANGLE_STEP, MAX_ANGLE_STEP);
    }

    // Generate angle list
    int32_t numAngles = static_cast<int32_t>(std::ceil(std::abs(angleExtent) / angleStep)) + 1;
    searchAngles_.reserve(numAngles);

    for (int32_t i = 0; i < numAngles; ++i) {
        double angle = angleStart + i * angleStep;
        if (angleExtent >= 0) {
            if (angle > angleStart + angleExtent) break;
        } else {
            if (angle < angleStart + angleExtent) break;
        }
        searchAngles_.push_back(angle);
    }

    // Store the actual step used
    if (searchAngles_.size() > 1) {
        params_.angleStep = searchAngles_[1] - searchAngles_[0];
    }
}

} // namespace Internal
} // namespace Qi::Vision::Matching
