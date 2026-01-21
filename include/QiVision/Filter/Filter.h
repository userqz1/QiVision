#pragma once

/**
 * @file Filter.h
 * @brief Image filtering operations (Halcon-style API)
 *
 * API Style: void Func(const QImage& in, QImage& out, params...)
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
 * @param output Output filtered image
 * @param sigma Gaussian sigma (standard deviation)
 *
 * @code
 * QImage smooth;
 * GaussFilter(image, smooth, 1.5);  // sigma = 1.5
 * @endcode
 */
void GaussFilter(const QImage& image, QImage& output, double sigma);

/**
 * @brief Apply Gaussian filter with separate X/Y sigmas
 *
 * @param image Input image
 * @param output Output filtered image
 * @param sigmaX Sigma in X direction
 * @param sigmaY Sigma in Y direction
 * @param borderMode Border handling mode: "reflect", "replicate", "constant"
 */
void GaussFilter(const QImage& image, QImage& output, double sigmaX, double sigmaY,
                  const std::string& borderMode = "reflect");

/**
 * @brief Apply Gaussian filter with specified kernel size
 *
 * @param image Input image
 * @param output Output filtered image
 * @param size Filter size: "3x3", "5x5", "7x7", "9x9", "11x11"
 *
 * @code
 * QImage smooth;
 * GaussImage(image, smooth, "5x5");
 * @endcode
 */
void GaussImage(const QImage& image, QImage& output, const std::string& size);

/**
 * @brief Apply mean (box) filter
 *
 * Equivalent to Halcon's mean_image operator.
 *
 * @param image Input image
 * @param output Output filtered image
 * @param width Kernel width
 * @param height Kernel height
 * @param borderMode Border handling mode
 */
void MeanImage(const QImage& image, QImage& output, int32_t width, int32_t height,
                const std::string& borderMode = "reflect");

/**
 * @brief Apply mean filter with square kernel
 */
void MeanImage(const QImage& image, QImage& output, int32_t size,
                const std::string& borderMode = "reflect");

/**
 * @brief Apply median filter
 *
 * Equivalent to Halcon's median_image operator.
 *
 * @param image Input image
 * @param output Output filtered image
 * @param maskType Mask type: "circle", "square", "rhombus"
 * @param radius Mask radius
 * @param marginMode Border handling: "mirrored", "cyclic", "continued"
 *
 * @code
 * QImage denoised;
 * MedianImage(image, denoised, "circle", 2, "mirrored");
 * @endcode
 */
void MedianImage(const QImage& image, QImage& output, const std::string& maskType,
                  int32_t radius, const std::string& marginMode = "mirrored");

/**
 * @brief Apply median filter with rectangular mask
 */
void MedianRect(const QImage& image, QImage& output, int32_t width, int32_t height);

/**
 * @brief Apply bilateral filter (edge-preserving smoothing)
 *
 * Equivalent to Halcon's bilateral_filter operator.
 *
 * @param image Input image
 * @param output Output filtered image
 * @param sigmaSpatial Spatial sigma (distance weight)
 * @param sigmaIntensity Intensity sigma (color similarity weight)
 *
 * @code
 * QImage smooth;
 * BilateralFilter(image, smooth, 5.0, 30.0);
 * @endcode
 */
void BilateralFilter(const QImage& image, QImage& output, double sigmaSpatial, double sigmaIntensity);

/**
 * @brief Apply bilateral filter with kernel size
 *
 * @param image Input image
 * @param output Output filtered image
 * @param size Kernel size
 * @param sigmaSpatial Spatial sigma
 * @param sigmaIntensity Intensity sigma
 */
void BilateralFilter(const QImage& image, QImage& output, int32_t size,
                      double sigmaSpatial, double sigmaIntensity);

/**
 * @brief Apply binomial filter (approximation of Gaussian)
 *
 * Equivalent to Halcon's binomial_filter operator.
 * Faster than Gaussian, uses binomial coefficients.
 *
 * @param image Input image
 * @param output Output filtered image
 * @param width Kernel width (odd, typically 3, 5, 7)
 * @param height Kernel height (odd)
 * @param borderMode Border handling
 */
void BinomialFilter(const QImage& image, QImage& output, int32_t width, int32_t height,
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
 * @param output Output gradient magnitude image
 * @param filterType Filter type: "sum_abs" (|Gx|+|Gy|), "sum_sqrt" (sqrt(Gx^2+Gy^2))
 * @param size Kernel size: 3, 5, 7
 *
 * @code
 * QImage edges;
 * SobelAmp(image, edges, "sum_abs", 3);
 * @endcode
 */
void SobelAmp(const QImage& image, QImage& output, const std::string& filterType = "sum_abs",
               int32_t size = 3);

/**
 * @brief Compute Sobel direction (gradient angle)
 *
 * Equivalent to Halcon's sobel_dir operator.
 *
 * @param image Input image
 * @param output Output direction image (angle in radians)
 * @param dirType Direction type: "gradient" (edge normal), "tangent" (edge direction)
 * @param size Kernel size
 */
void SobelDir(const QImage& image, QImage& output, const std::string& dirType = "gradient",
               int32_t size = 3);

/**
 * @brief Compute Prewitt amplitude (gradient magnitude)
 *
 * Equivalent to Halcon's prewitt_amp operator.
 * Uses uniform smoothing kernel [1,1,1]/3 instead of Sobel's [1,2,1]/4.
 *
 * @param image Input image
 * @param output Output gradient magnitude image
 * @param filterType Filter type: "sum_abs" (|Gx|+|Gy|), "sum_sqrt" (sqrt(Gx^2+Gy^2))
 *
 * @code
 * QImage edges;
 * PrewittAmp(image, edges, "sum_abs");
 * @endcode
 */
void PrewittAmp(const QImage& image, QImage& output, const std::string& filterType = "sum_abs");

/**
 * @brief Compute Roberts cross gradient (2x2 diagonal derivative)
 *
 * Equivalent to Halcon's roberts operator.
 * Uses 2x2 kernels: [[1,0],[0,-1]] and [[0,1],[-1,0]]
 *
 * @param image Input image
 * @param output Output gradient magnitude image
 * @param filterType Filter type: "sum_abs" (|Gx|+|Gy|), "sum_sqrt" (sqrt(Gx^2+Gy^2))
 *
 * @code
 * QImage edges;
 * RobertsAmp(image, edges, "sum_abs");
 * @endcode
 */
void RobertsAmp(const QImage& image, QImage& output, const std::string& filterType = "sum_abs");

/**
 * @brief Compute Gaussian derivative
 *
 * Equivalent to Halcon's derivate_gauss operator.
 *
 * @param image Input image
 * @param output Output derivative image
 * @param sigma Gaussian sigma
 * @param component Derivative component: "x", "y", "xx", "xy", "yy", "gradient"
 *
 * @code
 * QImage gx;
 * DerivateGauss(image, gx, 1.5, "x");      // First derivative in X
 * QImage gxx;
 * DerivateGauss(image, gxx, 1.5, "xx");    // Second derivative in X
 * @endcode
 */
void DerivateGauss(const QImage& image, QImage& output, double sigma, const std::string& component);

/**
 * @brief Compute gradient magnitude using Gaussian derivatives
 *
 * @param image Input image
 * @param output Output gradient magnitude image
 * @param sigma Gaussian sigma
 */
void GradientMagnitude(const QImage& image, QImage& output, double sigma);

/**
 * @brief Compute gradient direction using Gaussian derivatives
 *
 * @param image Input image
 * @param output Output gradient direction image (radians)
 * @param sigma Gaussian sigma
 */
void GradientDirection(const QImage& image, QImage& output, double sigma);

/**
 * @brief Apply Laplacian filter
 *
 * @param image Input image
 * @param output Output Laplacian image
 * @param filterType Filter type: "3x3", "5x5", "n4" (4-connected), "n8" (8-connected)
 */
void Laplace(const QImage& image, QImage& output, const std::string& filterType = "3x3");

/**
 * @brief Apply Laplacian of Gaussian (LoG) filter
 *
 * Equivalent to Halcon's laplace_of_gauss operator.
 *
 * @param image Input image
 * @param output Output LoG filtered image
 * @param sigma Gaussian sigma
 *
 * @code
 * QImage log;
 * LaplacianOfGaussian(image, log, 2.0);
 * @endcode
 */
void LaplacianOfGaussian(const QImage& image, QImage& output, double sigma);

// =============================================================================
// Frequency Domain Filters
// =============================================================================

/**
 * @brief Apply highpass filter
 *
 * Equivalent to Halcon's highpass_image operator.
 *
 * @param image Input image
 * @param output Output highpass filtered image
 * @param width Kernel width
 * @param height Kernel height
 */
void HighpassImage(const QImage& image, QImage& output, int32_t width, int32_t height);

/**
 * @brief Apply lowpass filter
 *
 * Equivalent to Halcon's lowpass_image operator.
 * Same as MeanImage but with different naming convention.
 *
 * @param image Input image
 * @param output Output lowpass filtered image
 * @param width Kernel width
 * @param height Kernel height
 */
void LowpassImage(const QImage& image, QImage& output, int32_t width, int32_t height);

// =============================================================================
// Enhancement Filters
// =============================================================================

/**
 * @brief Enhance image (sharpen + contrast)
 *
 * Equivalent to Halcon's emphasize operator.
 *
 * @param image Input image
 * @param output Output enhanced image
 * @param width Kernel width
 * @param height Kernel height
 * @param factor Enhancement factor (1.0 = no change, >1 = more enhancement)
 *
 * @code
 * QImage sharp;
 * EmphasizeImage(image, sharp, 7, 7, 1.5);
 * @endcode
 */
void EmphasizeImage(const QImage& image, QImage& output, int32_t width, int32_t height, double factor);

/**
 * @brief Apply unsharp mask (sharpening)
 *
 * @param image Input image
 * @param output Output sharpened image
 * @param sigma Gaussian sigma for blur
 * @param amount Sharpening amount (1.0 = standard)
 * @param threshold Threshold for detail (0 = sharpen everything)
 */
void UnsharpMask(const QImage& image, QImage& output, double sigma, double amount = 1.0,
                  double threshold = 0.0);

/**
 * @brief Apply shock filter (edge-enhancing diffusion)
 *
 * @param image Input image
 * @param output Output filtered image
 * @param iterations Number of iterations
 * @param dt Time step
 */
void ShockFilter(const QImage& image, QImage& output, int32_t iterations, double dt = 0.1);

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
 * @param output Output filtered image
 * @param mode Diffusion mode: "pm1" (favors high-contrast edges),
 *             "pm2" (favors wide regions)
 * @param contrast Contrast parameter (edge threshold)
 * @param theta Diffusion coefficient (0-0.25)
 * @param iterations Number of iterations
 *
 * @code
 * QImage smooth;
 * AnisoDiff(image, smooth, "pm1", 30.0, 0.25, 10);
 * @endcode
 */
void AnisoDiff(const QImage& image, QImage& output, const std::string& mode,
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
 * @param output Output convolved image
 * @param kernel Convolution kernel (row-major, size = kernelWidth * kernelHeight)
 * @param kernelWidth Kernel width (must be odd)
 * @param kernelHeight Kernel height (must be odd)
 * @param normalize If true, normalize kernel to sum to 1
 * @param borderMode Border handling
 */
void ConvolImage(const QImage& image, QImage& output,
                  const std::vector<double>& kernel,
                  int32_t kernelWidth, int32_t kernelHeight,
                  bool normalize = false,
                  const std::string& borderMode = "reflect");

/**
 * @brief Apply separable convolution
 *
 * @param image Input image
 * @param output Output convolved image
 * @param kernelX Horizontal kernel
 * @param kernelY Vertical kernel
 * @param borderMode Border handling
 */
void ConvolSeparable(const QImage& image, QImage& output,
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
 * @param output Output filtered image
 * @param width Kernel width
 * @param height Kernel height
 * @param rank Rank (0 = minimum, width*height-1 = maximum)
 */
void RankImage(const QImage& image, QImage& output, int32_t width, int32_t height, int32_t rank);

/**
 * @brief Apply minimum filter (gray erosion)
 */
void MinImage(const QImage& image, QImage& output, int32_t width, int32_t height);

/**
 * @brief Apply maximum filter (gray dilation)
 */
void MaxImage(const QImage& image, QImage& output, int32_t width, int32_t height);

// =============================================================================
// Texture Filters
// =============================================================================

/**
 * @brief Compute local standard deviation
 *
 * @param image Input image
 * @param output Output standard deviation image
 * @param width Window width
 * @param height Window height
 */
void StdDevImage(const QImage& image, QImage& output, int32_t width, int32_t height);

/**
 * @brief Compute local variance
 *
 * @param image Input image
 * @param output Output variance image
 * @param width Window width
 * @param height Window height
 */
void VarianceImage(const QImage& image, QImage& output, int32_t width, int32_t height);

/**
 * @brief Compute local entropy
 *
 * @param image Input image
 * @param output Output entropy image
 * @param width Window width
 * @param height Window height
 * @param numBins Number of histogram bins
 */
void EntropyImage(const QImage& image, QImage& output, int32_t width, int32_t height,
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
