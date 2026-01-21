#pragma once

/**
 * @file ImageIO.h
 * @brief Image I/O operations (Halcon-style API)
 *
 * Halcon reference operators:
 * - read_image, write_image
 * - read_sequence, write_sequence
 *
 * Supported formats: PNG, JPG/JPEG, BMP, TIFF, PGM/PPM, RAW
 * Supported types: UInt8, UInt16, Float32
 * Supported channels: Gray, RGB, RGBA
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/Types.h>

#include <cstdint>
#include <string>
#include <vector>

namespace Qi::Vision::IO {

// =============================================================================
// Image Format Enumeration
// =============================================================================

/**
 * @brief Supported image file formats
 */
enum class ImageFormat {
    Auto,       ///< Auto-detect from extension
    PNG,        ///< PNG (lossless, supports 8/16-bit, alpha)
    JPEG,       ///< JPEG (lossy, 8-bit only)
    BMP,        ///< BMP (uncompressed, 8-bit)
    TIFF,       ///< TIFF (supports 8/16-bit, float)
    PGM,        ///< PGM/PNM (portable graymap)
    PPM,        ///< PPM (portable pixmap, RGB)
    RAW         ///< Raw binary data (requires dimensions)
};

// =============================================================================
// Parameter Structures
// =============================================================================

/**
 * @brief JPEG/PNG compression parameters
 */
struct CompressionParams {
    int32_t jpegQuality = 95;       ///< JPEG quality [0-100]
    int32_t pngCompression = 6;     ///< PNG compression level [0-9]
    bool tiffCompression = true;    ///< Enable TIFF LZW compression

    CompressionParams& SetJpegQuality(int32_t q) { jpegQuality = q; return *this; }
    CompressionParams& SetPngCompression(int32_t c) { pngCompression = c; return *this; }
    CompressionParams& SetTiffCompression(bool c) { tiffCompression = c; return *this; }
};

/**
 * @brief Image metadata structure
 */
struct ImageMetadata {
    int32_t width = 0;
    int32_t height = 0;
    int32_t channels = 0;
    int32_t bitsPerChannel = 0;
    PixelType pixelType = PixelType::UInt8;
    ChannelType channelType = ChannelType::Gray;

    // Optional metadata
    double dpiX = 0.0;              ///< Horizontal DPI (0 = unknown)
    double dpiY = 0.0;              ///< Vertical DPI (0 = unknown)
    std::string colorProfile;       ///< Color profile name (if available)
    std::string description;        ///< Image description
    std::string software;           ///< Creating software
    std::string dateTime;           ///< Creation date/time
};

/**
 * @brief Raw image read parameters
 */
struct RawReadParams {
    int32_t width = 0;              ///< Image width (required)
    int32_t height = 0;             ///< Image height (required)
    PixelType pixelType = PixelType::UInt8;
    ChannelType channelType = ChannelType::Gray;
    int32_t headerBytes = 0;        ///< Skip header bytes
    bool bigEndian = false;         ///< Byte order for >8-bit

    RawReadParams& SetSize(int32_t w, int32_t h) { width = w; height = h; return *this; }
    RawReadParams& SetPixelType(PixelType t) { pixelType = t; return *this; }
    RawReadParams& SetChannelType(ChannelType t) { channelType = t; return *this; }
    RawReadParams& SetHeader(int32_t bytes) { headerBytes = bytes; return *this; }
    RawReadParams& SetBigEndian(bool be) { bigEndian = be; return *this; }
};

// =============================================================================
// Image Read Functions
// =============================================================================

/**
 * @brief Read image from file
 *
 * Equivalent to Halcon's read_image operator.
 *
 * @param filename Input file path
 * @param[out] image Loaded image
 * @throws IOException if file cannot be read
 *
 * @code
 * QImage img;
 * ReadImage("test.png", img);
 * @endcode
 */
void ReadImage(const std::string& filename, QImage& image);

/**
 * @brief Read image with format hint
 *
 * @param filename Input file path
 * @param[out] image Loaded image
 * @param format Force specific format (Auto = detect from extension)
 */
void ReadImage(const std::string& filename, QImage& image, ImageFormat format);

/**
 * @brief Read raw binary image data
 *
 * @param filename Input file path
 * @param[out] image Loaded image
 * @param params Raw read parameters (width, height, type required)
 */
void ReadImageRaw(const std::string& filename, QImage& image, const RawReadParams& params);

/**
 * @brief Read image metadata without loading full image
 *
 * Useful for getting dimensions before loading large images.
 *
 * @param filename Input file path
 * @param metadata [out] Image metadata
 * @return true if metadata was successfully read
 */
bool ReadImageMetadata(const std::string& filename, ImageMetadata& metadata);

/**
 * @brief Read image and convert to specified type
 *
 * @param filename Input file path
 * @param[out] image Loaded and converted image
 * @param targetType Target pixel type for conversion
 */
void ReadImageAs(const std::string& filename, QImage& image, PixelType targetType);

/**
 * @brief Read image and convert to grayscale
 *
 * @param filename Input file path
 * @param[out] image Grayscale image
 */
void ReadImageGray(const std::string& filename, QImage& image);

// =============================================================================
// Image Write Functions
// =============================================================================

/**
 * @brief Write image to file
 *
 * Equivalent to Halcon's write_image operator.
 *
 * @param image Image to write
 * @param filename Output file path
 * @return true if successful
 *
 * @code
 * WriteImage(img, "output.png");
 * WriteImage(img, "output.jpg");
 * @endcode
 */
bool WriteImage(const QImage& image, const std::string& filename);

/**
 * @brief Write image with format and compression parameters
 *
 * @param image Image to write
 * @param filename Output file path
 * @param format Output format (Auto = detect from extension)
 * @param params Compression parameters
 * @return true if successful
 */
bool WriteImage(const QImage& image, const std::string& filename,
                ImageFormat format, const CompressionParams& params = CompressionParams());

/**
 * @brief Write image as raw binary data
 *
 * @param image Image to write
 * @param filename Output file path
 * @param bigEndian Byte order for >8-bit
 * @return true if successful
 */
bool WriteImageRaw(const QImage& image, const std::string& filename,
                   bool bigEndian = false);

// =============================================================================
// Image Sequence Functions
// =============================================================================

/**
 * @brief Read image sequence (multiple files)
 *
 * Equivalent to Halcon's read_sequence operator.
 *
 * @param pattern File pattern with printf-style placeholder (e.g., "img_%03d.png")
 * @param[out] images Vector of loaded images
 * @param startIndex Starting index
 * @param endIndex Ending index (inclusive)
 * @param step Index step (default 1)
 *
 * @code
 * std::vector<QImage> images;
 * ReadSequence("frame_%04d.png", images, 0, 99);
 * @endcode
 */
void ReadSequence(const std::string& pattern,
                  std::vector<QImage>& images,
                  int32_t startIndex,
                  int32_t endIndex,
                  int32_t step = 1);

/**
 * @brief Read all images from directory
 *
 * @param directory Directory path
 * @param[out] images Vector of loaded images
 * @param extensions Filter by extensions (empty = all supported)
 */
void ReadDirectory(const std::string& directory,
                   std::vector<QImage>& images,
                   const std::vector<std::string>& extensions = {});

/**
 * @brief Write image sequence
 *
 * @param images Vector of images to write
 * @param pattern File pattern with printf-style placeholder
 * @param startIndex Starting index
 * @param params Compression parameters
 * @return Number of images successfully written
 */
int32_t WriteSequence(const std::vector<QImage>& images,
                       const std::string& pattern,
                       int32_t startIndex = 0,
                       const CompressionParams& params = CompressionParams());

// =============================================================================
// Format Utility Functions
// =============================================================================

/**
 * @brief Get image format from filename extension
 *
 * @param filename File path
 * @return Detected format (Auto if unknown)
 */
ImageFormat GetFormatFromFilename(const std::string& filename);

/**
 * @brief Get file extension for format
 *
 * @param format Image format
 * @return Extension string (e.g., ".png")
 */
std::string GetExtensionForFormat(ImageFormat format);

/**
 * @brief Check if format supports 16-bit depth
 */
bool FormatSupports16Bit(ImageFormat format);

/**
 * @brief Check if format supports alpha channel
 */
bool FormatSupportsAlpha(ImageFormat format);

/**
 * @brief Check if format is lossless
 */
bool FormatIsLossless(ImageFormat format);

/**
 * @brief List supported file extensions
 *
 * @return Vector of supported extensions (e.g., {".png", ".jpg", ".bmp"})
 */
std::vector<std::string> GetSupportedExtensions();

/**
 * @brief Check if file is a supported image format
 */
bool IsSupportedImageFile(const std::string& filename);

} // namespace Qi::Vision::IO
