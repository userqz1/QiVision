/**
 * @file Filter.cpp
 * @brief Image filtering operations implementation
 *
 * Wraps Internal layer functions with Halcon-style API
 */

#include <QiVision/Filter/Filter.h>
#include <QiVision/Core/Exception.h>
#include <QiVision/Internal/Convolution.h>
#include <QiVision/Internal/Gradient.h>
#include <QiVision/Internal/Gaussian.h>
#include <QiVision/Internal/MorphGray.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>

namespace Qi::Vision::Filter {

// Use Internal types
using Internal::BorderMode;

// =============================================================================
// Helper Functions
// =============================================================================

namespace {

BorderMode ParseBorderMode(const std::string& mode) {
    std::string lower = mode;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    if (lower == "reflect" || lower == "reflect101" || lower == "mirrored") {
        return BorderMode::Reflect101;
    } else if (lower == "replicate" || lower == "continued") {
        return BorderMode::Replicate;
    } else if (lower == "constant" || lower == "zero") {
        return BorderMode::Constant;
    } else if (lower == "wrap" || lower == "cyclic") {
        return BorderMode::Wrap;
    }

    return BorderMode::Reflect101;
}

int32_t ParseKernelSize(const std::string& size) {
    if (size == "3x3" || size == "3") return 3;
    if (size == "5x5" || size == "5") return 5;
    if (size == "7x7" || size == "7") return 7;
    if (size == "9x9" || size == "9") return 9;
    if (size == "11x11" || size == "11") return 11;
    return 3;  // Default
}

inline double Clamp(double val, double minVal, double maxVal) {
    return std::max(minVal, std::min(maxVal, val));
}

inline uint8_t ClampU8(double val) {
    return static_cast<uint8_t>(Clamp(val, 0.0, 255.0));
}

} // anonymous namespace

// =============================================================================
// Utility Functions
// =============================================================================

int32_t OptimalKernelSize(double sigma) {
    // Halcon convention: kernel size = 2 * ceil(3 * sigma) + 1
    int32_t size = 2 * static_cast<int32_t>(std::ceil(3.0 * sigma)) + 1;
    return std::max(3, size);
}

std::vector<double> GenGaussKernel(double sigma, int32_t size) {
    if (size <= 0) {
        size = OptimalKernelSize(sigma);
    }

    // Ensure odd size
    if (size % 2 == 0) size++;

    std::vector<double> kernel(size);
    int32_t center = size / 2;
    double sum = 0.0;
    double sigma2 = 2.0 * sigma * sigma;

    for (int32_t i = 0; i < size; ++i) {
        double x = i - center;
        kernel[i] = std::exp(-x * x / sigma2);
        sum += kernel[i];
    }

    // Normalize
    for (double& k : kernel) {
        k /= sum;
    }

    return kernel;
}

std::vector<double> GenGaussDerivKernel(double sigma, int32_t order, int32_t size) {
    if (size <= 0) {
        size = OptimalKernelSize(sigma);
    }

    if (size % 2 == 0) size++;

    std::vector<double> kernel(size);
    int32_t center = size / 2;
    double sigma2 = sigma * sigma;
    double sigma4 = sigma2 * sigma2;

    for (int32_t i = 0; i < size; ++i) {
        double x = i - center;

        if (order == 1) {
            // First derivative: -x / sigma^2 * G(x)
            kernel[i] = -x / sigma2 * std::exp(-x * x / (2.0 * sigma2));
        } else if (order == 2) {
            // Second derivative: (x^2 - sigma^2) / sigma^4 * G(x)
            kernel[i] = (x * x - sigma2) / sigma4 * std::exp(-x * x / (2.0 * sigma2));
        } else {
            kernel[i] = std::exp(-x * x / (2.0 * sigma2));
        }
    }

    return kernel;
}

// =============================================================================
// Smoothing Filters
// =============================================================================

QImage GaussFilter(const QImage& image, double sigma) {
    return GaussFilter(image, sigma, sigma, "reflect");
}

QImage GaussFilter(const QImage& image, double sigmaX, double sigmaY,
                    const std::string& borderMode) {
    if (image.Empty()) return QImage();

    if (image.Type() != PixelType::UInt8) {
        throw UnsupportedException("GaussFilter only supports UInt8 images");
    }

    BorderMode border = ParseBorderMode(borderMode);

    // Generate Gaussian kernels
    auto kernelX = GenGaussKernel(sigmaX);
    auto kernelY = GenGaussKernel(sigmaY);

    int32_t w = image.Width();
    int32_t h = image.Height();
    int channels = image.Channels();

    QImage result(w, h, image.Type(), image.GetChannelType());

    // Process each channel
    std::vector<float> temp(w * h);
    std::vector<float> src(w * h);

    for (int c = 0; c < channels; ++c) {
        // Extract channel to float
        for (int32_t y = 0; y < h; ++y) {
            const uint8_t* row = static_cast<const uint8_t*>(image.RowPtr(y));
            for (int32_t x = 0; x < w; ++x) {
                src[y * w + x] = static_cast<float>(row[x * channels + c]);
            }
        }

        // Apply separable convolution
        Internal::ConvolveSeparable<float, float>(
            src.data(), temp.data(), w, h,
            kernelX.data(), static_cast<int32_t>(kernelX.size()),
            kernelY.data(), static_cast<int32_t>(kernelY.size()),
            border);

        // Write back to result
        for (int32_t y = 0; y < h; ++y) {
            uint8_t* row = static_cast<uint8_t*>(result.RowPtr(y));
            for (int32_t x = 0; x < w; ++x) {
                row[x * channels + c] = ClampU8(temp[y * w + x]);
            }
        }
    }

    return result;
}

QImage GaussImage(const QImage& image, const std::string& size) {
    int32_t kernelSize = ParseKernelSize(size);
    double sigma = kernelSize / 6.0;  // Approximate sigma from size
    return GaussFilter(image, sigma);
}

QImage MeanImage(const QImage& image, int32_t width, int32_t height,
                  const std::string& borderMode) {
    if (image.Empty()) return QImage();

    if (image.Type() != PixelType::UInt8) {
        throw UnsupportedException("MeanImage only supports UInt8 images");
    }

    // Create uniform kernel
    std::vector<double> kernelX(width, 1.0 / width);
    std::vector<double> kernelY(height, 1.0 / height);

    BorderMode border = ParseBorderMode(borderMode);

    int32_t w = image.Width();
    int32_t h = image.Height();
    int channels = image.Channels();

    QImage result(w, h, image.Type(), image.GetChannelType());

    std::vector<float> temp(w * h);
    std::vector<float> src(w * h);

    for (int c = 0; c < channels; ++c) {
        for (int32_t y = 0; y < h; ++y) {
            const uint8_t* row = static_cast<const uint8_t*>(image.RowPtr(y));
            for (int32_t x = 0; x < w; ++x) {
                src[y * w + x] = static_cast<float>(row[x * channels + c]);
            }
        }

        Internal::ConvolveSeparable<float, float>(
            src.data(), temp.data(), w, h,
            kernelX.data(), width,
            kernelY.data(), height,
            border);

        for (int32_t y = 0; y < h; ++y) {
            uint8_t* row = static_cast<uint8_t*>(result.RowPtr(y));
            for (int32_t x = 0; x < w; ++x) {
                row[x * channels + c] = ClampU8(temp[y * w + x]);
            }
        }
    }

    return result;
}

QImage MeanImage(const QImage& image, int32_t size, const std::string& borderMode) {
    return MeanImage(image, size, size, borderMode);
}

QImage MedianImage(const QImage& image, const std::string& maskType,
                    int32_t radius, const std::string& marginMode) {
    if (image.Empty()) return QImage();

    if (image.Type() != PixelType::UInt8) {
        throw UnsupportedException("MedianImage only supports UInt8 images");
    }

    int32_t size = 2 * radius + 1;
    return MedianRect(image, size, size);
}

QImage MedianRect(const QImage& image, int32_t width, int32_t height) {
    if (image.Empty()) return QImage();

    if (image.Type() != PixelType::UInt8) {
        throw UnsupportedException("MedianRect only supports UInt8 images");
    }

    int32_t w = image.Width();
    int32_t h = image.Height();
    int channels = image.Channels();
    int32_t halfW = width / 2;
    int32_t halfH = height / 2;

    QImage result(w, h, image.Type(), image.GetChannelType());
    std::vector<uint8_t> neighborhood(width * height);

    for (int c = 0; c < channels; ++c) {
        for (int32_t y = 0; y < h; ++y) {
            uint8_t* dstRow = static_cast<uint8_t*>(result.RowPtr(y));

            for (int32_t x = 0; x < w; ++x) {
                // Collect neighborhood
                int count = 0;
                for (int32_t ky = -halfH; ky <= halfH; ++ky) {
                    int32_t sy = std::max(0, std::min(h - 1, y + ky));
                    const uint8_t* srcRow = static_cast<const uint8_t*>(image.RowPtr(sy));

                    for (int32_t kx = -halfW; kx <= halfW; ++kx) {
                        int32_t sx = std::max(0, std::min(w - 1, x + kx));
                        neighborhood[count++] = srcRow[sx * channels + c];
                    }
                }

                // Find median (partial sort)
                std::nth_element(neighborhood.begin(), neighborhood.begin() + count / 2,
                                neighborhood.begin() + count);
                dstRow[x * channels + c] = neighborhood[count / 2];
            }
        }
    }

    return result;
}

QImage BilateralFilter(const QImage& image, double sigmaSpatial, double sigmaIntensity) {
    int32_t size = OptimalKernelSize(sigmaSpatial);
    return BilateralFilter(image, size, sigmaSpatial, sigmaIntensity);
}

QImage BilateralFilter(const QImage& image, int32_t size,
                        double sigmaSpatial, double sigmaIntensity) {
    if (image.Empty()) return QImage();

    if (image.Type() != PixelType::UInt8) {
        throw UnsupportedException("BilateralFilter only supports UInt8 images");
    }

    int32_t w = image.Width();
    int32_t h = image.Height();
    int channels = image.Channels();
    int32_t halfSize = size / 2;

    double spatialCoeff = -0.5 / (sigmaSpatial * sigmaSpatial);
    double intensityCoeff = -0.5 / (sigmaIntensity * sigmaIntensity);

    QImage result(w, h, image.Type(), image.GetChannelType());

    // Precompute spatial weights
    std::vector<double> spatialWeight(size * size);
    for (int ky = 0; ky < size; ++ky) {
        for (int kx = 0; kx < size; ++kx) {
            double dy = ky - halfSize;
            double dx = kx - halfSize;
            spatialWeight[ky * size + kx] = std::exp((dx * dx + dy * dy) * spatialCoeff);
        }
    }

    for (int32_t y = 0; y < h; ++y) {
        uint8_t* dstRow = static_cast<uint8_t*>(result.RowPtr(y));

        for (int32_t x = 0; x < w; ++x) {
            for (int c = 0; c < channels; ++c) {
                const uint8_t* centerRow = static_cast<const uint8_t*>(image.RowPtr(y));
                double centerVal = centerRow[x * channels + c];

                double sum = 0.0;
                double weightSum = 0.0;

                for (int32_t ky = -halfSize; ky <= halfSize; ++ky) {
                    int32_t sy = std::max(0, std::min(h - 1, y + ky));
                    const uint8_t* srcRow = static_cast<const uint8_t*>(image.RowPtr(sy));

                    for (int32_t kx = -halfSize; kx <= halfSize; ++kx) {
                        int32_t sx = std::max(0, std::min(w - 1, x + kx));
                        double pixelVal = srcRow[sx * channels + c];

                        double intensityDiff = pixelVal - centerVal;
                        double intensityWeight = std::exp(intensityDiff * intensityDiff * intensityCoeff);

                        int kidx = (ky + halfSize) * size + (kx + halfSize);
                        double weight = spatialWeight[kidx] * intensityWeight;

                        sum += pixelVal * weight;
                        weightSum += weight;
                    }
                }

                dstRow[x * channels + c] = ClampU8(sum / weightSum);
            }
        }
    }

    return result;
}

QImage BinomialFilter(const QImage& image, int32_t width, int32_t height,
                       const std::string& borderMode) {
    // Binomial coefficients for different sizes
    auto getBinomial = [](int32_t n) -> std::vector<double> {
        std::vector<double> coeffs(n);
        double sum = 0;
        for (int i = 0; i < n; ++i) {
            coeffs[i] = std::tgamma(n) / (std::tgamma(i + 1) * std::tgamma(n - i));
            sum += coeffs[i];
        }
        for (double& c : coeffs) c /= sum;
        return coeffs;
    };

    auto kernelX = getBinomial(width);
    auto kernelY = getBinomial(height);

    BorderMode border = ParseBorderMode(borderMode);

    int32_t w = image.Width();
    int32_t h = image.Height();
    int channels = image.Channels();

    QImage result(w, h, image.Type(), image.GetChannelType());

    std::vector<float> temp(w * h);
    std::vector<float> src(w * h);

    for (int c = 0; c < channels; ++c) {
        for (int32_t y = 0; y < h; ++y) {
            const uint8_t* row = static_cast<const uint8_t*>(image.RowPtr(y));
            for (int32_t x = 0; x < w; ++x) {
                src[y * w + x] = static_cast<float>(row[x * channels + c]);
            }
        }

        Internal::ConvolveSeparable<float, float>(
            src.data(), temp.data(), w, h,
            kernelX.data(), width,
            kernelY.data(), height,
            border);

        for (int32_t y = 0; y < h; ++y) {
            uint8_t* row = static_cast<uint8_t*>(result.RowPtr(y));
            for (int32_t x = 0; x < w; ++x) {
                row[x * channels + c] = ClampU8(temp[y * w + x]);
            }
        }
    }

    return result;
}

// =============================================================================
// Derivative Filters
// =============================================================================

QImage SobelAmp(const QImage& image, const std::string& filterType, int32_t size) {
    if (image.Empty()) return QImage();

    if (image.Type() != PixelType::UInt8 || image.Channels() != 1) {
        throw UnsupportedException("SobelAmp requires single-channel UInt8 image");
    }

    int32_t w = image.Width();
    int32_t h = image.Height();

    // Get Sobel kernels
    auto deriv = Internal::SobelDerivativeKernel(size);
    auto smooth = Internal::SobelSmoothingKernel(size);

    std::vector<float> src(w * h);
    std::vector<float> gx(w * h);
    std::vector<float> gy(w * h);

    // Convert to float
    for (int32_t y = 0; y < h; ++y) {
        const uint8_t* row = static_cast<const uint8_t*>(image.RowPtr(y));
        for (int32_t x = 0; x < w; ++x) {
            src[y * w + x] = static_cast<float>(row[x]);
        }
    }

    // Compute Gx and Gy using separable convolution
    Internal::ConvolveSeparable<float, float>(
        src.data(), gx.data(), w, h,
        deriv.data(), static_cast<int32_t>(deriv.size()),
        smooth.data(), static_cast<int32_t>(smooth.size()),
        BorderMode::Reflect101);

    Internal::ConvolveSeparable<float, float>(
        src.data(), gy.data(), w, h,
        smooth.data(), static_cast<int32_t>(smooth.size()),
        deriv.data(), static_cast<int32_t>(deriv.size()),
        BorderMode::Reflect101);

    // Create result
    QImage result(w, h, PixelType::UInt8, ChannelType::Gray);

    std::string lowerType = filterType;
    std::transform(lowerType.begin(), lowerType.end(), lowerType.begin(), ::tolower);

    bool useSqrt = (lowerType == "sum_sqrt" || lowerType == "sqrt");

    for (int32_t y = 0; y < h; ++y) {
        uint8_t* row = static_cast<uint8_t*>(result.RowPtr(y));
        for (int32_t x = 0; x < w; ++x) {
            float gxVal = gx[y * w + x];
            float gyVal = gy[y * w + x];

            double mag;
            if (useSqrt) {
                mag = std::sqrt(gxVal * gxVal + gyVal * gyVal);
            } else {
                mag = std::abs(gxVal) + std::abs(gyVal);
            }

            row[x] = ClampU8(mag);
        }
    }

    return result;
}

QImage SobelDir(const QImage& image, const std::string& dirType, int32_t size) {
    if (image.Empty()) return QImage();

    if (image.Type() != PixelType::UInt8 || image.Channels() != 1) {
        throw UnsupportedException("SobelDir requires single-channel UInt8 image");
    }

    int32_t w = image.Width();
    int32_t h = image.Height();

    auto deriv = Internal::SobelDerivativeKernel(size);
    auto smooth = Internal::SobelSmoothingKernel(size);

    std::vector<float> src(w * h);
    std::vector<float> gx(w * h);
    std::vector<float> gy(w * h);

    for (int32_t y = 0; y < h; ++y) {
        const uint8_t* row = static_cast<const uint8_t*>(image.RowPtr(y));
        for (int32_t x = 0; x < w; ++x) {
            src[y * w + x] = static_cast<float>(row[x]);
        }
    }

    Internal::ConvolveSeparable<float, float>(
        src.data(), gx.data(), w, h,
        deriv.data(), static_cast<int32_t>(deriv.size()),
        smooth.data(), static_cast<int32_t>(smooth.size()),
        BorderMode::Reflect101);

    Internal::ConvolveSeparable<float, float>(
        src.data(), gy.data(), w, h,
        smooth.data(), static_cast<int32_t>(smooth.size()),
        deriv.data(), static_cast<int32_t>(deriv.size()),
        BorderMode::Reflect101);

    // Return as float image with direction in radians
    QImage result(w, h, PixelType::Float32, ChannelType::Gray);

    for (int32_t y = 0; y < h; ++y) {
        float* row = static_cast<float*>(result.RowPtr(y));
        for (int32_t x = 0; x < w; ++x) {
            row[x] = static_cast<float>(std::atan2(gy[y * w + x], gx[y * w + x]));
        }
    }

    return result;
}

QImage DerivateGauss(const QImage& image, double sigma, const std::string& component) {
    if (image.Empty()) return QImage();

    if (image.Type() != PixelType::UInt8 || image.Channels() != 1) {
        throw UnsupportedException("DerivateGauss requires single-channel UInt8 image");
    }

    int32_t w = image.Width();
    int32_t h = image.Height();

    std::string lowerComp = component;
    std::transform(lowerComp.begin(), lowerComp.end(), lowerComp.begin(), ::tolower);

    std::vector<double> gaussKernel = GenGaussKernel(sigma);
    std::vector<double> derivKernel = GenGaussDerivKernel(sigma, 1);
    std::vector<double> deriv2Kernel = GenGaussDerivKernel(sigma, 2);

    std::vector<float> src(w * h);
    std::vector<float> dst(w * h);

    for (int32_t y = 0; y < h; ++y) {
        const uint8_t* row = static_cast<const uint8_t*>(image.RowPtr(y));
        for (int32_t x = 0; x < w; ++x) {
            src[y * w + x] = static_cast<float>(row[x]);
        }
    }

    if (lowerComp == "x") {
        Internal::ConvolveSeparable<float, float>(
            src.data(), dst.data(), w, h,
            derivKernel.data(), static_cast<int32_t>(derivKernel.size()),
            gaussKernel.data(), static_cast<int32_t>(gaussKernel.size()),
            BorderMode::Reflect101);
    } else if (lowerComp == "y") {
        Internal::ConvolveSeparable<float, float>(
            src.data(), dst.data(), w, h,
            gaussKernel.data(), static_cast<int32_t>(gaussKernel.size()),
            derivKernel.data(), static_cast<int32_t>(derivKernel.size()),
            BorderMode::Reflect101);
    } else if (lowerComp == "xx") {
        Internal::ConvolveSeparable<float, float>(
            src.data(), dst.data(), w, h,
            deriv2Kernel.data(), static_cast<int32_t>(deriv2Kernel.size()),
            gaussKernel.data(), static_cast<int32_t>(gaussKernel.size()),
            BorderMode::Reflect101);
    } else if (lowerComp == "yy") {
        Internal::ConvolveSeparable<float, float>(
            src.data(), dst.data(), w, h,
            gaussKernel.data(), static_cast<int32_t>(gaussKernel.size()),
            deriv2Kernel.data(), static_cast<int32_t>(deriv2Kernel.size()),
            BorderMode::Reflect101);
    } else if (lowerComp == "xy") {
        Internal::ConvolveSeparable<float, float>(
            src.data(), dst.data(), w, h,
            derivKernel.data(), static_cast<int32_t>(derivKernel.size()),
            derivKernel.data(), static_cast<int32_t>(derivKernel.size()),
            BorderMode::Reflect101);
    } else {
        // Default: gradient magnitude
        return GradientMagnitude(image, sigma);
    }

    QImage result(w, h, PixelType::Float32, ChannelType::Gray);
    std::memcpy(result.Data(), dst.data(), w * h * sizeof(float));

    return result;
}

QImage GradientMagnitude(const QImage& image, double sigma) {
    if (image.Empty()) return QImage();

    QImage gx = DerivateGauss(image, sigma, "x");
    QImage gy = DerivateGauss(image, sigma, "y");

    int32_t w = image.Width();
    int32_t h = image.Height();

    QImage result(w, h, PixelType::Float32, ChannelType::Gray);

    for (int32_t y = 0; y < h; ++y) {
        const float* gxRow = static_cast<const float*>(gx.RowPtr(y));
        const float* gyRow = static_cast<const float*>(gy.RowPtr(y));
        float* dstRow = static_cast<float*>(result.RowPtr(y));

        for (int32_t x = 0; x < w; ++x) {
            dstRow[x] = std::sqrt(gxRow[x] * gxRow[x] + gyRow[x] * gyRow[x]);
        }
    }

    return result;
}

QImage GradientDirection(const QImage& image, double sigma) {
    if (image.Empty()) return QImage();

    QImage gx = DerivateGauss(image, sigma, "x");
    QImage gy = DerivateGauss(image, sigma, "y");

    int32_t w = image.Width();
    int32_t h = image.Height();

    QImage result(w, h, PixelType::Float32, ChannelType::Gray);

    for (int32_t y = 0; y < h; ++y) {
        const float* gxRow = static_cast<const float*>(gx.RowPtr(y));
        const float* gyRow = static_cast<const float*>(gy.RowPtr(y));
        float* dstRow = static_cast<float*>(result.RowPtr(y));

        for (int32_t x = 0; x < w; ++x) {
            dstRow[x] = std::atan2(gyRow[x], gxRow[x]);
        }
    }

    return result;
}

QImage Laplace(const QImage& image, const std::string& filterType) {
    if (image.Empty()) return QImage();

    if (image.Type() != PixelType::UInt8 || image.Channels() != 1) {
        throw UnsupportedException("Laplace requires single-channel UInt8 image");
    }

    // Laplacian kernels
    std::vector<double> kernel;

    std::string lower = filterType;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    if (lower == "n4" || lower == "4") {
        kernel = {0, 1, 0, 1, -4, 1, 0, 1, 0};
    } else if (lower == "n8" || lower == "8") {
        kernel = {1, 1, 1, 1, -8, 1, 1, 1, 1};
    } else {
        // Default 3x3
        kernel = {0, 1, 0, 1, -4, 1, 0, 1, 0};
    }

    return ConvolImage(image, kernel, 3, 3, false, "reflect");
}

QImage LaplacianOfGaussian(const QImage& image, double sigma) {
    if (image.Empty()) return QImage();

    // LoG = Gxx + Gyy
    QImage gxx = DerivateGauss(image, sigma, "xx");
    QImage gyy = DerivateGauss(image, sigma, "yy");

    int32_t w = image.Width();
    int32_t h = image.Height();

    QImage result(w, h, PixelType::Float32, ChannelType::Gray);

    for (int32_t y = 0; y < h; ++y) {
        const float* gxxRow = static_cast<const float*>(gxx.RowPtr(y));
        const float* gyyRow = static_cast<const float*>(gyy.RowPtr(y));
        float* dstRow = static_cast<float*>(result.RowPtr(y));

        for (int32_t x = 0; x < w; ++x) {
            dstRow[x] = gxxRow[x] + gyyRow[x];
        }
    }

    return result;
}

// =============================================================================
// Frequency Domain Filters
// =============================================================================

QImage HighpassImage(const QImage& image, int32_t width, int32_t height) {
    if (image.Empty()) return QImage();

    // Highpass = Original - Lowpass
    QImage lowpass = MeanImage(image, width, height);

    int32_t w = image.Width();
    int32_t h = image.Height();
    int channels = image.Channels();

    QImage result(w, h, image.Type(), image.GetChannelType());

    for (int32_t y = 0; y < h; ++y) {
        const uint8_t* srcRow = static_cast<const uint8_t*>(image.RowPtr(y));
        const uint8_t* lpRow = static_cast<const uint8_t*>(lowpass.RowPtr(y));
        uint8_t* dstRow = static_cast<uint8_t*>(result.RowPtr(y));

        for (int32_t x = 0; x < w * channels; ++x) {
            int diff = srcRow[x] - lpRow[x] + 128;  // Add offset to keep in range
            dstRow[x] = ClampU8(diff);
        }
    }

    return result;
}

QImage LowpassImage(const QImage& image, int32_t width, int32_t height) {
    return MeanImage(image, width, height);
}

// =============================================================================
// Enhancement Filters
// =============================================================================

QImage EmphasizeImage(const QImage& image, int32_t width, int32_t height, double factor) {
    if (image.Empty()) return QImage();

    QImage lowpass = MeanImage(image, width, height);

    int32_t w = image.Width();
    int32_t h = image.Height();
    int channels = image.Channels();

    QImage result(w, h, image.Type(), image.GetChannelType());

    for (int32_t y = 0; y < h; ++y) {
        const uint8_t* srcRow = static_cast<const uint8_t*>(image.RowPtr(y));
        const uint8_t* lpRow = static_cast<const uint8_t*>(lowpass.RowPtr(y));
        uint8_t* dstRow = static_cast<uint8_t*>(result.RowPtr(y));

        for (int32_t x = 0; x < w * channels; ++x) {
            double detail = srcRow[x] - lpRow[x];
            double enhanced = srcRow[x] + factor * detail;
            dstRow[x] = ClampU8(enhanced);
        }
    }

    return result;
}

QImage UnsharpMask(const QImage& image, double sigma, double amount, double threshold) {
    if (image.Empty()) return QImage();

    QImage blurred = GaussFilter(image, sigma);

    int32_t w = image.Width();
    int32_t h = image.Height();
    int channels = image.Channels();

    QImage result(w, h, image.Type(), image.GetChannelType());

    for (int32_t y = 0; y < h; ++y) {
        const uint8_t* srcRow = static_cast<const uint8_t*>(image.RowPtr(y));
        const uint8_t* blurRow = static_cast<const uint8_t*>(blurred.RowPtr(y));
        uint8_t* dstRow = static_cast<uint8_t*>(result.RowPtr(y));

        for (int32_t x = 0; x < w * channels; ++x) {
            double diff = srcRow[x] - blurRow[x];

            if (std::abs(diff) >= threshold) {
                double sharpened = srcRow[x] + amount * diff;
                dstRow[x] = ClampU8(sharpened);
            } else {
                dstRow[x] = srcRow[x];
            }
        }
    }

    return result;
}

QImage ShockFilter(const QImage& image, int32_t iterations, double dt) {
    if (image.Empty()) return QImage();

    QImage current = image.Clone();

    for (int32_t iter = 0; iter < iterations; ++iter) {
        QImage lap = Laplace(current, "n4");

        int32_t w = current.Width();
        int32_t h = current.Height();

        for (int32_t y = 0; y < h; ++y) {
            const uint8_t* lapRow = static_cast<const uint8_t*>(lap.RowPtr(y));
            uint8_t* curRow = static_cast<uint8_t*>(current.RowPtr(y));

            for (int32_t x = 0; x < w; ++x) {
                double lapVal = lapRow[x] - 128;  // Remove offset
                double update = dt * (lapVal < 0 ? 1.0 : -1.0);
                curRow[x] = ClampU8(curRow[x] + update);
            }
        }
    }

    return current;
}

// =============================================================================
// Anisotropic Diffusion
// =============================================================================

QImage AnisoDiff(const QImage& image, const std::string& mode,
                  double contrast, double theta, int32_t iterations) {
    if (image.Empty()) return QImage();

    if (image.Type() != PixelType::UInt8 || image.Channels() != 1) {
        throw UnsupportedException("AnisoDiff requires single-channel UInt8 image");
    }

    int32_t w = image.Width();
    int32_t h = image.Height();

    std::string lowerMode = mode;
    std::transform(lowerMode.begin(), lowerMode.end(), lowerMode.begin(), ::tolower);
    bool usePM1 = (lowerMode == "pm1");

    double k2 = contrast * contrast;

    // Work with float
    std::vector<float> current(w * h);
    std::vector<float> next(w * h);

    for (int32_t y = 0; y < h; ++y) {
        const uint8_t* row = static_cast<const uint8_t*>(image.RowPtr(y));
        for (int32_t x = 0; x < w; ++x) {
            current[y * w + x] = static_cast<float>(row[x]);
        }
    }

    for (int32_t iter = 0; iter < iterations; ++iter) {
        for (int32_t y = 0; y < h; ++y) {
            for (int32_t x = 0; x < w; ++x) {
                float center = current[y * w + x];

                // Compute gradients (4-connected)
                float gN = (y > 0) ? current[(y-1) * w + x] - center : 0;
                float gS = (y < h-1) ? current[(y+1) * w + x] - center : 0;
                float gE = (x < w-1) ? current[y * w + x + 1] - center : 0;
                float gW = (x > 0) ? current[y * w + x - 1] - center : 0;

                // Compute diffusion coefficients
                auto diffCoeff = [&](float g) -> float {
                    double g2 = g * g;
                    if (usePM1) {
                        // PM1: c(g) = exp(-g^2/k^2)
                        return static_cast<float>(std::exp(-g2 / k2));
                    } else {
                        // PM2: c(g) = 1 / (1 + g^2/k^2)
                        return static_cast<float>(1.0 / (1.0 + g2 / k2));
                    }
                };

                float cN = diffCoeff(gN);
                float cS = diffCoeff(gS);
                float cE = diffCoeff(gE);
                float cW = diffCoeff(gW);

                // Update
                float update = theta * (cN * gN + cS * gS + cE * gE + cW * gW);
                next[y * w + x] = center + update;
            }
        }

        std::swap(current, next);
    }

    // Convert back to image
    QImage result(w, h, PixelType::UInt8, ChannelType::Gray);

    for (int32_t y = 0; y < h; ++y) {
        uint8_t* row = static_cast<uint8_t*>(result.RowPtr(y));
        for (int32_t x = 0; x < w; ++x) {
            row[x] = ClampU8(current[y * w + x]);
        }
    }

    return result;
}

// =============================================================================
// Custom Convolution
// =============================================================================

QImage ConvolImage(const QImage& image,
                    const std::vector<double>& kernel,
                    int32_t kernelWidth, int32_t kernelHeight,
                    bool normalize,
                    const std::string& borderMode) {
    if (image.Empty()) return QImage();

    if (image.Type() != PixelType::UInt8) {
        throw UnsupportedException("ConvolImage only supports UInt8 images");
    }

    int32_t w = image.Width();
    int32_t h = image.Height();
    int channels = image.Channels();

    BorderMode border = ParseBorderMode(borderMode);

    std::vector<double> normKernel = kernel;
    if (normalize) {
        double sum = std::accumulate(normKernel.begin(), normKernel.end(), 0.0);
        if (sum != 0) {
            for (double& k : normKernel) k /= sum;
        }
    }

    QImage result(w, h, image.Type(), image.GetChannelType());

    std::vector<float> src(w * h);
    std::vector<float> dst(w * h);

    for (int c = 0; c < channels; ++c) {
        for (int32_t y = 0; y < h; ++y) {
            const uint8_t* row = static_cast<const uint8_t*>(image.RowPtr(y));
            for (int32_t x = 0; x < w; ++x) {
                src[y * w + x] = static_cast<float>(row[x * channels + c]);
            }
        }

        Internal::Convolve2D<float, float>(
            src.data(), dst.data(), w, h,
            normKernel.data(), kernelWidth, kernelHeight,
            border);

        for (int32_t y = 0; y < h; ++y) {
            uint8_t* row = static_cast<uint8_t*>(result.RowPtr(y));
            for (int32_t x = 0; x < w; ++x) {
                row[x * channels + c] = ClampU8(dst[y * w + x]);
            }
        }
    }

    return result;
}

QImage ConvolSeparable(const QImage& image,
                        const std::vector<double>& kernelX,
                        const std::vector<double>& kernelY,
                        const std::string& borderMode) {
    if (image.Empty()) return QImage();

    if (image.Type() != PixelType::UInt8) {
        throw UnsupportedException("ConvolSeparable only supports UInt8 images");
    }

    int32_t w = image.Width();
    int32_t h = image.Height();
    int channels = image.Channels();

    BorderMode border = ParseBorderMode(borderMode);

    QImage result(w, h, image.Type(), image.GetChannelType());

    std::vector<float> src(w * h);
    std::vector<float> dst(w * h);

    for (int c = 0; c < channels; ++c) {
        for (int32_t y = 0; y < h; ++y) {
            const uint8_t* row = static_cast<const uint8_t*>(image.RowPtr(y));
            for (int32_t x = 0; x < w; ++x) {
                src[y * w + x] = static_cast<float>(row[x * channels + c]);
            }
        }

        Internal::ConvolveSeparable<float, float>(
            src.data(), dst.data(), w, h,
            kernelX.data(), static_cast<int32_t>(kernelX.size()),
            kernelY.data(), static_cast<int32_t>(kernelY.size()),
            border);

        for (int32_t y = 0; y < h; ++y) {
            uint8_t* row = static_cast<uint8_t*>(result.RowPtr(y));
            for (int32_t x = 0; x < w; ++x) {
                row[x * channels + c] = ClampU8(dst[y * w + x]);
            }
        }
    }

    return result;
}

// =============================================================================
// Rank Filters
// =============================================================================

QImage RankImage(const QImage& image, int32_t width, int32_t height, int32_t rank) {
    if (image.Empty()) return QImage();

    if (image.Type() != PixelType::UInt8 || image.Channels() != 1) {
        throw UnsupportedException("RankImage requires single-channel UInt8 image");
    }

    int32_t w = image.Width();
    int32_t h = image.Height();
    int32_t halfW = width / 2;
    int32_t halfH = height / 2;

    QImage result(w, h, PixelType::UInt8, ChannelType::Gray);
    std::vector<uint8_t> neighborhood(width * height);

    for (int32_t y = 0; y < h; ++y) {
        uint8_t* dstRow = static_cast<uint8_t*>(result.RowPtr(y));

        for (int32_t x = 0; x < w; ++x) {
            int count = 0;
            for (int32_t ky = -halfH; ky <= halfH; ++ky) {
                int32_t sy = std::max(0, std::min(h - 1, y + ky));
                const uint8_t* srcRow = static_cast<const uint8_t*>(image.RowPtr(sy));

                for (int32_t kx = -halfW; kx <= halfW; ++kx) {
                    int32_t sx = std::max(0, std::min(w - 1, x + kx));
                    neighborhood[count++] = srcRow[sx];
                }
            }

            int32_t actualRank = std::min(rank, count - 1);
            std::nth_element(neighborhood.begin(), neighborhood.begin() + actualRank,
                            neighborhood.begin() + count);
            dstRow[x] = neighborhood[actualRank];
        }
    }

    return result;
}

QImage MinImage(const QImage& image, int32_t width, int32_t height) {
    return RankImage(image, width, height, 0);
}

QImage MaxImage(const QImage& image, int32_t width, int32_t height) {
    return RankImage(image, width, height, width * height - 1);
}

// =============================================================================
// Texture Filters
// =============================================================================

QImage StdDevImage(const QImage& image, int32_t width, int32_t height) {
    if (image.Empty()) return QImage();

    QImage variance = VarianceImage(image, width, height);

    int32_t w = variance.Width();
    int32_t h = variance.Height();

    QImage result(w, h, PixelType::Float32, ChannelType::Gray);

    for (int32_t y = 0; y < h; ++y) {
        const float* varRow = static_cast<const float*>(variance.RowPtr(y));
        float* dstRow = static_cast<float*>(result.RowPtr(y));

        for (int32_t x = 0; x < w; ++x) {
            dstRow[x] = std::sqrt(varRow[x]);
        }
    }

    return result;
}

QImage VarianceImage(const QImage& image, int32_t width, int32_t height) {
    if (image.Empty()) return QImage();

    if (image.Type() != PixelType::UInt8 || image.Channels() != 1) {
        throw UnsupportedException("VarianceImage requires single-channel UInt8 image");
    }

    int32_t w = image.Width();
    int32_t h = image.Height();
    int32_t halfW = width / 2;
    int32_t halfH = height / 2;

    QImage result(w, h, PixelType::Float32, ChannelType::Gray);

    for (int32_t y = 0; y < h; ++y) {
        float* dstRow = static_cast<float*>(result.RowPtr(y));

        for (int32_t x = 0; x < w; ++x) {
            double sum = 0;
            double sumSq = 0;
            int count = 0;

            for (int32_t ky = -halfH; ky <= halfH; ++ky) {
                int32_t sy = std::max(0, std::min(h - 1, y + ky));
                const uint8_t* srcRow = static_cast<const uint8_t*>(image.RowPtr(sy));

                for (int32_t kx = -halfW; kx <= halfW; ++kx) {
                    int32_t sx = std::max(0, std::min(w - 1, x + kx));
                    double val = srcRow[sx];
                    sum += val;
                    sumSq += val * val;
                    count++;
                }
            }

            double mean = sum / count;
            double variance = (sumSq / count) - (mean * mean);
            dstRow[x] = static_cast<float>(std::max(0.0, variance));
        }
    }

    return result;
}

QImage EntropyImage(const QImage& image, int32_t width, int32_t height, int32_t numBins) {
    if (image.Empty()) return QImage();

    if (image.Type() != PixelType::UInt8 || image.Channels() != 1) {
        throw UnsupportedException("EntropyImage requires single-channel UInt8 image");
    }

    int32_t w = image.Width();
    int32_t h = image.Height();
    int32_t halfW = width / 2;
    int32_t halfH = height / 2;

    QImage result(w, h, PixelType::Float32, ChannelType::Gray);
    std::vector<int> histogram(numBins);

    for (int32_t y = 0; y < h; ++y) {
        float* dstRow = static_cast<float*>(result.RowPtr(y));

        for (int32_t x = 0; x < w; ++x) {
            std::fill(histogram.begin(), histogram.end(), 0);
            int count = 0;

            for (int32_t ky = -halfH; ky <= halfH; ++ky) {
                int32_t sy = std::max(0, std::min(h - 1, y + ky));
                const uint8_t* srcRow = static_cast<const uint8_t*>(image.RowPtr(sy));

                for (int32_t kx = -halfW; kx <= halfW; ++kx) {
                    int32_t sx = std::max(0, std::min(w - 1, x + kx));
                    int bin = srcRow[sx] * numBins / 256;
                    histogram[bin]++;
                    count++;
                }
            }

            double entropy = 0;
            for (int i = 0; i < numBins; ++i) {
                if (histogram[i] > 0) {
                    double p = static_cast<double>(histogram[i]) / count;
                    entropy -= p * std::log2(p);
                }
            }

            dstRow[x] = static_cast<float>(entropy);
        }
    }

    return result;
}

} // namespace Qi::Vision::Filter
