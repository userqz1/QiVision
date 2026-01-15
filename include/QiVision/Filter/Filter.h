#pragma once

/**
 * @file Filter.h
 * @brief Image filtering operations (Halcon-style API)
 *
 * Halcon reference operators:
 * - gauss_filter, mean_image, median_image
 * - bilateral_filter, binomial_filter
 * - sobel_amp, sobel_dir, derivate_gauss
 * - laplacian_of_gaussian, highpass_image, lowpass_image
 * - aniso_diff, emphasize_image
 *
 * Border handling: Reflect101 (default), Replicate, Constant
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/Types.h>

#include <cstdint>
#include <string>
#include <vector>

namespace Qi::Vision::Filter {

// =============================================================================
// Smoothing Filters
// =============================================================================

/**
 * @brief Apply Gaussian filter
 *
 * Equivalent to Halcon's gauss_filter / gauss_image operator.
 *
 * @param image Input image
 * @param sigma Gaussian sigma (standard deviation)
 * @return Filtered image
 *
 * @code
 * QImage smooth = GaussFilter(image, 1.5);  // sigma = 1.5
 * @endcode
 */
QImage GaussFilter(const QImage& image, double sigma);

/**
 * @brief Apply Gaussian filter with separate X/Y sigmas
 *
 * @param image Input image
 * @param sigmaX Sigma in X direction
 * @param sigmaY Sigma in Y direction
 * @param borderMode Border handling mode: "reflect", "replicate", "constant"
 * @return Filtered image
 */
QImage GaussFilter(const QImage& image, double sigmaX, double sigmaY,
                    const std::string& borderMode = "reflect");

/**
 * @brief Apply Gaussian filter with specified kernel size
 *
 * @param image Input image
 * @param size Filter size: "3x3", "5x5", "7x7", "9x9", "11x11"
 * @return Filtered image
 *
 * @code
 * QImage smooth = GaussImage(image, "5x5");
 * @endcode
 */
QImage GaussImage(const QImage& image, const std::string& size);

/**
 * @brief Apply mean (box) filter
 *
 * Equivalent to Halcon's mean_image operator.
 *
 * @param image Input image
 * @param width Kernel width
 * @param height Kernel height
 * @param borderMode Border handling mode
 * @return Filtered image
 */
QImage MeanImage(const QImage& image, int32_t width, int32_t height,
                  const std::string& borderMode = "reflect");

/**
 * @brief Apply mean filter with square kernel
 */
QImage MeanImage(const QImage& image, int32_t size,
                  const std::string& borderMode = "reflect");

/**
 * @brief Apply median filter
 *
 * Equivalent to Halcon's median_image operator.
 *
 * @param image Input image
 * @param maskType Mask type: "circle", "square", "rhombus"
 * @param radius Mask radius
 * @param marginMode Border handling: "mirrored", "cyclic", "continued"
 * @return Filtered image
 *
 * @code
 * QImage denoised = MedianImage(image, "circle", 2, "mirrored");
 * @endcode
 */
QImage MedianImage(const QImage& image, const std::string& maskType,
                    int32_t radius, const std::string& marginMode = "mirrored");

/**
 * @brief Apply median filter with rectangular mask
 */
QImage MedianRect(const QImage& image, int32_t width, int32_t height);

/**
 * @brief Apply bilateral filter (edge-preserving smoothing)
 *
 * Equivalent to Halcon's bilateral_filter operator.
 *
 * @param image Input image
 * @param sigmaSpatial Spatial sigma (distance weight)
 * @param sigmaIntensity Intensity sigma (color similarity weight)
 * @return Filtered image
 *
 * @code
 * QImage smooth = BilateralFilter(image, 5.0, 30.0);
 * @endcode
 */
QImage BilateralFilter(const QImage& image, double sigmaSpatial, double sigmaIntensity);

/**
 * @brief Apply bilateral filter with kernel size
 *
 * @param image Input image
 * @param size Kernel size
 * @param sigmaSpatial Spatial sigma
 * @param sigmaIntensity Intensity sigma
 * @return Filtered image
 */
QImage BilateralFilter(const QImage& image, int32_t size,
                        double sigmaSpatial, double sigmaIntensity);

/**
 * @brief Apply binomial filter (approximation of Gaussian)
 *
 * Equivalent to Halcon's binomial_filter operator.
 * Faster than Gaussian, uses binomial coefficients.
 *
 * @param image Input image
 * @param width Kernel width (odd, typically 3, 5, 7)
 * @param height Kernel height (odd)
 * @param borderMode Border handling
 * @return Filtered image
 */
QImage BinomialFilter(const QImage& image, int32_t width, int32_t height,
                       const std::string& borderMode = "reflect");

// =============================================================================
// Derivative Filters
// =============================================================================

/**
 * @brief Compute Sobel amplitude (gradient magnitude)
 *
 * Equivalent to Halcon's sobel_amp operator.
 *
 * @param image Input image
 * @param filterType Filter type: "sum_abs" (|Gx|+|Gy|), "sum_sqrt" (sqrt(Gx^2+Gy^2))
 * @param size Kernel size: 3, 5, 7
 * @return Gradient magnitude image
 *
 * @code
 * QImage edges = SobelAmp(image, "sum_abs", 3);
 * @endcode
 */
QImage SobelAmp(const QImage& image, const std::string& filterType = "sum_abs",
                 int32_t size = 3);

/**
 * @brief Compute Sobel direction (gradient angle)
 *
 * Equivalent to Halcon's sobel_dir operator.
 *
 * @param image Input image
 * @param dirType Direction type: "gradient" (edge normal), "tangent" (edge direction)
 * @param size Kernel size
 * @return Direction image (angle in radians)
 */
QImage SobelDir(const QImage& image, const std::string& dirType = "gradient",
                 int32_t size = 3);

/**
 * @brief Compute Gaussian derivative
 *
 * Equivalent to Halcon's derivate_gauss operator.
 *
 * @param image Input image
 * @param sigma Gaussian sigma
 * @param component Derivative component: "x", "y", "xx", "xy", "yy", "gradient"
 * @return Derivative image
 *
 * @code
 * QImage gx = DerivateGauss(image, 1.5, "x");      // First derivative in X
 * QImage gxx = DerivateGauss(image, 1.5, "xx");    // Second derivative in X
 * QImage gxy = DerivateGauss(image, 1.5, "xy");    // Mixed second derivative
 * @endcode
 */
QImage DerivateGauss(const QImage& image, double sigma, const std::string& component);

/**
 * @brief Compute gradient magnitude using Gaussian derivatives
 *
 * @param image Input image
 * @param sigma Gaussian sigma
 * @return Gradient magnitude image
 */
QImage GradientMagnitude(const QImage& image, double sigma);

/**
 * @brief Compute gradient direction using Gaussian derivatives
 *
 * @param image Input image
 * @param sigma Gaussian sigma
 * @return Gradient direction image (radians)
 */
QImage GradientDirection(const QImage& image, double sigma);

/**
 * @brief Apply Laplacian filter
 *
 * @param image Input image
 * @param filterType Filter type: "3x3", "5x5", "n4" (4-connected), "n8" (8-connected)
 * @return Laplacian image
 */
QImage Laplace(const QImage& image, const std::string& filterType = "3x3");

/**
 * @brief Apply Laplacian of Gaussian (LoG) filter
 *
 * Equivalent to Halcon's laplace_of_gauss operator.
 *
 * @param image Input image
 * @param sigma Gaussian sigma
 * @return LoG filtered image
 *
 * @code
 * QImage log = LaplacianOfGaussian(image, 2.0);
 * @endcode
 */
QImage LaplacianOfGaussian(const QImage& image, double sigma);

// =============================================================================
// Frequency Domain Filters
// =============================================================================

/**
 * @brief Apply highpass filter
 *
 * Equivalent to Halcon's highpass_image operator.
 *
 * @param image Input image
 * @param width Kernel width
 * @param height Kernel height
 * @return Highpass filtered image
 */
QImage HighpassImage(const QImage& image, int32_t width, int32_t height);

/**
 * @brief Apply lowpass filter
 *
 * Equivalent to Halcon's lowpass_image operator.
 * Same as MeanImage but with different naming convention.
 *
 * @param image Input image
 * @param width Kernel width
 * @param height Kernel height
 * @return Lowpass filtered image
 */
QImage LowpassImage(const QImage& image, int32_t width, int32_t height);

// =============================================================================
// Enhancement Filters
// =============================================================================

/**
 * @brief Enhance image (sharpen + contrast)
 *
 * Equivalent to Halcon's emphasize operator.
 *
 * @param image Input image
 * @param width Kernel width
 * @param height Kernel height
 * @param factor Enhancement factor (1.0 = no change, >1 = more enhancement)
 * @return Enhanced image
 *
 * @code
 * QImage sharp = EmphasizeImage(image, 7, 7, 1.5);
 * @endcode
 */
QImage EmphasizeImage(const QImage& image, int32_t width, int32_t height, double factor);

/**
 * @brief Apply unsharp mask (sharpening)
 *
 * @param image Input image
 * @param sigma Gaussian sigma for blur
 * @param amount Sharpening amount (1.0 = standard)
 * @param threshold Threshold for detail (0 = sharpen everything)
 * @return Sharpened image
 */
QImage UnsharpMask(const QImage& image, double sigma, double amount = 1.0,
                    double threshold = 0.0);

/**
 * @brief Apply shock filter (edge-enhancing diffusion)
 *
 * @param image Input image
 * @param iterations Number of iterations
 * @param dt Time step
 * @return Filtered image
 */
QImage ShockFilter(const QImage& image, int32_t iterations, double dt = 0.1);

// =============================================================================
// Anisotropic Diffusion
// =============================================================================

/**
 * @brief Apply anisotropic diffusion (Perona-Malik)
 *
 * Equivalent to Halcon's aniso_diff operator.
 * Edge-preserving smoothing using nonlinear diffusion.
 *
 * @param image Input image
 * @param mode Diffusion mode: "pm1" (favors high-contrast edges),
 *             "pm2" (favors wide regions)
 * @param contrast Contrast parameter (edge threshold)
 * @param theta Diffusion coefficient (0-0.25)
 * @param iterations Number of iterations
 * @return Filtered image
 *
 * @code
 * QImage smooth = AnisoDiff(image, "pm1", 30.0, 0.25, 10);
 * @endcode
 */
QImage AnisoDiff(const QImage& image, const std::string& mode,
                  double contrast, double theta, int32_t iterations);

// =============================================================================
// Custom Convolution
// =============================================================================

/**
 * @brief Apply custom convolution kernel
 *
 * Equivalent to Halcon's convol_image operator.
 *
 * @param image Input image
 * @param kernel Convolution kernel (row-major, size = kernelWidth * kernelHeight)
 * @param kernelWidth Kernel width (must be odd)
 * @param kernelHeight Kernel height (must be odd)
 * @param normalize If true, normalize kernel to sum to 1
 * @param borderMode Border handling
 * @return Convolved image
 */
QImage ConvolImage(const QImage& image,
                    const std::vector<double>& kernel,
                    int32_t kernelWidth, int32_t kernelHeight,
                    bool normalize = false,
                    const std::string& borderMode = "reflect");

/**
 * @brief Apply separable convolution
 *
 * @param image Input image
 * @param kernelX Horizontal kernel
 * @param kernelY Vertical kernel
 * @param borderMode Border handling
 * @return Convolved image
 */
QImage ConvolSeparable(const QImage& image,
                        const std::vector<double>& kernelX,
                        const std::vector<double>& kernelY,
                        const std::string& borderMode = "reflect");

// =============================================================================
// Rank Filters
// =============================================================================

/**
 * @brief Apply rank filter
 *
 * @param image Input image
 * @param width Kernel width
 * @param height Kernel height
 * @param rank Rank (0 = minimum, width*height-1 = maximum)
 * @return Filtered image
 */
QImage RankImage(const QImage& image, int32_t width, int32_t height, int32_t rank);

/**
 * @brief Apply minimum filter (gray erosion)
 */
QImage MinImage(const QImage& image, int32_t width, int32_t height);

/**
 * @brief Apply maximum filter (gray dilation)
 */
QImage MaxImage(const QImage& image, int32_t width, int32_t height);

// =============================================================================
// Texture Filters
// =============================================================================

/**
 * @brief Compute local standard deviation
 *
 * @param image Input image
 * @param width Window width
 * @param height Window height
 * @return Standard deviation image
 */
QImage StdDevImage(const QImage& image, int32_t width, int32_t height);

/**
 * @brief Compute local variance
 *
 * @param image Input image
 * @param width Window width
 * @param height Window height
 * @return Variance image
 */
QImage VarianceImage(const QImage& image, int32_t width, int32_t height);

/**
 * @brief Compute local entropy
 *
 * @param image Input image
 * @param width Window width
 * @param height Window height
 * @param numBins Number of histogram bins
 * @return Entropy image
 */
QImage EntropyImage(const QImage& image, int32_t width, int32_t height,
                     int32_t numBins = 256);

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Generate Gaussian kernel
 *
 * @param sigma Gaussian sigma
 * @param size Kernel size (0 = auto-compute from sigma)
 * @return 1D Gaussian kernel
 */
std::vector<double> GenGaussKernel(double sigma, int32_t size = 0);

/**
 * @brief Generate Gaussian derivative kernel
 *
 * @param sigma Gaussian sigma
 * @param order Derivative order (1 or 2)
 * @param size Kernel size (0 = auto)
 * @return 1D derivative kernel
 */
std::vector<double> GenGaussDerivKernel(double sigma, int32_t order, int32_t size = 0);

/**
 * @brief Compute optimal kernel size for sigma
 *
 * @param sigma Gaussian sigma
 * @return Recommended kernel size (odd number)
 */
int32_t OptimalKernelSize(double sigma);

} // namespace Qi::Vision::Filter
