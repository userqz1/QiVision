/**
 * @file ImageIO.cpp
 * @brief Image I/O operations implementation
 */

#include <QiVision/IO/ImageIO.h>
#include <QiVision/Core/Exception.h>
#include <QiVision/Platform/FileIO.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <cstdio>

// For directory listing
#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#else
#include <dirent.h>
#include <sys/stat.h>
#endif

// Use stb_image for reading (already included in QImage.cpp with IMPLEMENTATION)
#include <stb/stb_image.h>
#include <stb/stb_image_write.h>

namespace Qi::Vision::IO {

// =============================================================================
// Helper Functions
// =============================================================================

namespace {

std::string ToLower(const std::string& str) {
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);
    return result;
}

std::string GetExtension(const std::string& filename) {
    auto pos = filename.rfind('.');
    if (pos == std::string::npos) return "";
    return ToLower(filename.substr(pos));
}

// Format file pattern with index
std::string FormatFilename(const std::string& pattern, int32_t index) {
    char buffer[1024];
    std::snprintf(buffer, sizeof(buffer), pattern.c_str(), index);
    return std::string(buffer);
}

// Get number of channels from ChannelType
int32_t GetChannelCount(ChannelType type) {
    switch (type) {
        case ChannelType::Gray: return 1;
        case ChannelType::RGB:
        case ChannelType::BGR: return 3;
        case ChannelType::RGBA:
        case ChannelType::BGRA: return 4;
        default: return 1;
    }
}

// Get bytes per pixel
size_t GetBytesPerPixel(PixelType type, ChannelType channels) {
    size_t channelSize = 1;
    switch (type) {
        case PixelType::UInt8: channelSize = 1; break;
        case PixelType::UInt16:
        case PixelType::Int16: channelSize = 2; break;
        case PixelType::Float32: channelSize = 4; break;
    }
    return channelSize * GetChannelCount(channels);
}

// List files in a directory
std::vector<std::string> ListDirectoryFiles(const std::string& directory) {
    std::vector<std::string> files;

#if defined(_WIN32) || defined(_WIN64)
    std::string searchPath = directory + "\\*";
    WIN32_FIND_DATAA fd;
    HANDLE hFind = FindFirstFileA(searchPath.c_str(), &fd);
    if (hFind != INVALID_HANDLE_VALUE) {
        do {
            if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
                files.push_back(fd.cFileName);
            }
        } while (FindNextFileA(hFind, &fd));
        FindClose(hFind);
    }
#else
    DIR* dir = opendir(directory.c_str());
    if (dir) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            if (entry->d_type == DT_REG) {
                files.push_back(entry->d_name);
            } else if (entry->d_type == DT_UNKNOWN) {
                // Check with stat for filesystems that don't support d_type
                std::string fullPath = directory + "/" + entry->d_name;
                struct stat st;
                if (stat(fullPath.c_str(), &st) == 0 && S_ISREG(st.st_mode)) {
                    files.push_back(entry->d_name);
                }
            }
        }
        closedir(dir);
    }
#endif

    return files;
}

} // anonymous namespace

// =============================================================================
// Format Utility Functions
// =============================================================================

ImageFormat GetFormatFromFilename(const std::string& filename) {
    std::string ext = GetExtension(filename);

    if (ext == ".png") return ImageFormat::PNG;
    if (ext == ".jpg" || ext == ".jpeg") return ImageFormat::JPEG;
    if (ext == ".bmp") return ImageFormat::BMP;
    if (ext == ".tif" || ext == ".tiff") return ImageFormat::TIFF;
    if (ext == ".pgm" || ext == ".ppm" || ext == ".pnm") return ImageFormat::PGM;
    if (ext == ".raw" || ext == ".bin") return ImageFormat::RAW;

    return ImageFormat::Auto;
}

std::string GetExtensionForFormat(ImageFormat format) {
    switch (format) {
        case ImageFormat::PNG: return ".png";
        case ImageFormat::JPEG: return ".jpg";
        case ImageFormat::BMP: return ".bmp";
        case ImageFormat::TIFF: return ".tiff";
        case ImageFormat::PGM: return ".pgm";
        case ImageFormat::PPM: return ".ppm";
        case ImageFormat::RAW: return ".raw";
        default: return ".png";
    }
}

bool FormatSupports16Bit(ImageFormat format) {
    return format == ImageFormat::PNG ||
           format == ImageFormat::TIFF ||
           format == ImageFormat::PGM ||
           format == ImageFormat::PPM ||
           format == ImageFormat::RAW;
}

bool FormatSupportsAlpha(ImageFormat format) {
    return format == ImageFormat::PNG ||
           format == ImageFormat::TIFF ||
           format == ImageFormat::RAW;
}

bool FormatIsLossless(ImageFormat format) {
    return format != ImageFormat::JPEG;
}

std::vector<std::string> GetSupportedExtensions() {
    return {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff",
            ".pgm", ".ppm", ".pnm", ".raw", ".bin"};
}

bool IsSupportedImageFile(const std::string& filename) {
    std::string ext = GetExtension(filename);
    auto supported = GetSupportedExtensions();
    return std::find(supported.begin(), supported.end(), ext) != supported.end();
}

// =============================================================================
// Image Read Functions
// =============================================================================

void ReadImage(const std::string& filename, QImage& image) {
    ReadImage(filename, image, ImageFormat::Auto);
}

void ReadImage(const std::string& filename, QImage& image, ImageFormat /*format*/) {
    // Use QImage::FromFile which already handles stb_image
    // This wraps it with Halcon-style naming
    try {
        image = QImage::FromFile(filename);
    } catch (const std::exception& e) {
        throw IOException("Failed to read image: " + filename + " - " + e.what());
    }
}

void ReadImageRaw(const std::string& filename, QImage& image, const RawReadParams& params) {
    ReadImageRaw(filename, image, params.width, params.height,
                 params.pixelType, params.channelType,
                 params.headerBytes, params.bigEndian);
}

void ReadImageRaw(const std::string& filename, QImage& image,
                  int32_t width, int32_t height,
                  PixelType pixelType,
                  ChannelType channelType,
                  int32_t headerBytes,
                  bool bigEndian) {
    if (width <= 0 || height <= 0) {
        throw InvalidArgumentException("RAW read requires width and height");
    }

    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        throw IOException("Cannot open file: " + filename);
    }

    // Skip header
    if (headerBytes > 0) {
        file.seekg(headerBytes);
    }

    // Calculate buffer size
    size_t bytesPerPixel = GetBytesPerPixel(pixelType, channelType);
    size_t totalBytes = width * height * bytesPerPixel;

    // Read data
    std::vector<uint8_t> buffer(totalBytes);
    file.read(reinterpret_cast<char*>(buffer.data()), totalBytes);

    if (!file) {
        throw IOException("Failed to read raw data from: " + filename);
    }

    // Handle endianness for 16-bit data
    if (bigEndian && (pixelType == PixelType::UInt16 ||
                      pixelType == PixelType::Int16)) {
        uint16_t* data16 = reinterpret_cast<uint16_t*>(buffer.data());
        size_t count = totalBytes / 2;
        for (size_t i = 0; i < count; ++i) {
            uint16_t val = data16[i];
            data16[i] = ((val & 0xFF) << 8) | ((val >> 8) & 0xFF);
        }
    }

    // Create image from data
    image = QImage::FromData(buffer.data(), width, height, pixelType, channelType);
}

bool ReadImageMetadata(const std::string& filename, ImageMetadata& metadata) {
    int w, h, channels;

    if (stbi_info(filename.c_str(), &w, &h, &channels) == 0) {
        return false;
    }

    metadata.width = w;
    metadata.height = h;
    metadata.channels = channels;
    metadata.bitsPerChannel = 8;  // stb_image info doesn't provide bit depth
    metadata.pixelType = PixelType::UInt8;

    switch (channels) {
        case 1: metadata.channelType = ChannelType::Gray; break;
        case 3: metadata.channelType = ChannelType::RGB; break;
        case 4: metadata.channelType = ChannelType::RGBA; break;
        default: metadata.channelType = ChannelType::Gray; break;
    }

    // Check for 16-bit PNG
    if (stbi_is_16_bit(filename.c_str())) {
        metadata.bitsPerChannel = 16;
        metadata.pixelType = PixelType::UInt16;
    }

    return true;
}

void ReadImageAs(const std::string& filename, QImage& image, PixelType targetType) {
    ReadImage(filename, image);

    if (image.Empty()) {
        return;
    }

    // Convert to target type if needed
    if (image.Type() != targetType) {
        image = image.ConvertTo(targetType);
    }
}

void ReadImageGray(const std::string& filename, QImage& image) {
    ReadImage(filename, image);

    if (image.Empty()) {
        return;
    }

    if (image.GetChannelType() != ChannelType::Gray) {
        image = image.ToGray();
    }
}

// =============================================================================
// Image Write Functions
// =============================================================================

bool WriteImage(const QImage& image, const std::string& filename) {
    return WriteImage(image, filename, ImageFormat::Auto, std::vector<int>{});
}

bool WriteImage(const QImage& image, const std::string& filename,
                ImageFormat format, const CompressionParams& params) {
    // Convert CompressionParams to vector<int> and delegate
    std::vector<int> vecParams;
    vecParams.push_back(QIWRITE_JPEG_QUALITY);
    vecParams.push_back(params.jpegQuality);
    vecParams.push_back(QIWRITE_PNG_COMPRESSION);
    vecParams.push_back(params.pngCompression);
    vecParams.push_back(QIWRITE_TIFF_COMPRESSION);
    vecParams.push_back(params.tiffCompression ? 1 : 0);
    return WriteImage(image, filename, format, vecParams);
}

bool WriteImage(const QImage& image, const std::string& filename,
                ImageFormat format, const std::vector<int>& params) {
    // Parse vector<int> params into local variables
    int32_t jpegQuality = 95;
    int32_t pngCompression = 6;
    bool tiffCompression = true;

    for (size_t i = 0; i + 1 < params.size(); i += 2) {
        int key = params[i];
        int value = params[i + 1];
        switch (key) {
            case QIWRITE_JPEG_QUALITY:
                jpegQuality = value;
                break;
            case QIWRITE_PNG_COMPRESSION:
                pngCompression = value;
                break;
            case QIWRITE_TIFF_COMPRESSION:
                tiffCompression = (value != 0);
                break;
        }
    }

    (void)pngCompression;    // Reserved for future use
    (void)tiffCompression;   // Reserved for future use
    if (image.Empty()) {
        return false;
    }

    // Determine format
    if (format == ImageFormat::Auto) {
        format = GetFormatFromFilename(filename);
        if (format == ImageFormat::Auto) {
            format = ImageFormat::PNG;  // Default to PNG
        }
    }

    // For now, use QImage's SaveToFile for basic formats
    // TODO: Add 16-bit and compression control support
    if (image.Type() == PixelType::UInt8) {
        int channels = image.Channels();
        int32_t w = image.Width();
        int32_t h = image.Height();

        // Create contiguous buffer
        std::vector<uint8_t> buffer(w * h * channels);
        size_t srcStride = w * channels;

        for (int32_t y = 0; y < h; ++y) {
            std::memcpy(buffer.data() + y * srcStride, image.RowPtr(y), srcStride);
        }

        switch (format) {
            case ImageFormat::PNG:
                return stbi_write_png(filename.c_str(), w, h, channels,
                                      buffer.data(), static_cast<int>(srcStride)) != 0;
            case ImageFormat::JPEG:
                return stbi_write_jpg(filename.c_str(), w, h, channels,
                                      buffer.data(), jpegQuality) != 0;
            case ImageFormat::BMP:
                return stbi_write_bmp(filename.c_str(), w, h, channels,
                                      buffer.data()) != 0;
            default:
                // Default to PNG
                return stbi_write_png(filename.c_str(), w, h, channels,
                                      buffer.data(), static_cast<int>(srcStride)) != 0;
        }
    }

    // For non-UInt8 types, try RAW format
    if (format == ImageFormat::RAW) {
        return WriteImageRaw(image, filename, false);
    }

    return false;
}

bool WriteImageRaw(const QImage& image, const std::string& filename, bool bigEndian) {
    if (image.Empty()) {
        return false;
    }

    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        return false;
    }

    int32_t w = image.Width();
    int32_t h = image.Height();
    size_t bytesPerPixel = GetBytesPerPixel(image.Type(), image.GetChannelType());
    size_t rowBytes = w * bytesPerPixel;

    for (int32_t y = 0; y < h; ++y) {
        const uint8_t* row = static_cast<const uint8_t*>(image.RowPtr(y));

        if (bigEndian && (image.Type() == PixelType::UInt16 ||
                          image.Type() == PixelType::Int16)) {
            // Swap bytes for big-endian output
            std::vector<uint8_t> swapped(rowBytes);
            const uint16_t* src16 = reinterpret_cast<const uint16_t*>(row);
            uint16_t* dst16 = reinterpret_cast<uint16_t*>(swapped.data());
            size_t count = rowBytes / 2;
            for (size_t i = 0; i < count; ++i) {
                uint16_t val = src16[i];
                dst16[i] = ((val & 0xFF) << 8) | ((val >> 8) & 0xFF);
            }
            file.write(reinterpret_cast<const char*>(swapped.data()), rowBytes);
        } else {
            file.write(reinterpret_cast<const char*>(row), rowBytes);
        }
    }

    return file.good();
}

// =============================================================================
// Image Sequence Functions
// =============================================================================

void ReadSequence(const std::string& pattern,
                  std::vector<QImage>& images,
                  int32_t startIndex,
                  int32_t endIndex,
                  int32_t step) {
    images.clear();

    if (step <= 0) step = 1;

    for (int32_t i = startIndex; i <= endIndex; i += step) {
        std::string filename = FormatFilename(pattern, i);
        try {
            QImage img;
            ReadImage(filename, img);
            if (!img.Empty()) {
                images.push_back(std::move(img));
            }
        } catch (const std::exception&) {
            // Skip files that can't be read
            continue;
        }
    }
}

void ReadDirectory(const std::string& directory,
                   std::vector<QImage>& images,
                   const std::vector<std::string>& extensions) {
    images.clear();

    // Get list of files in directory
    auto files = ListDirectoryFiles(directory);

    // Sort files for consistent ordering
    std::sort(files.begin(), files.end());

    // Filter by extension if specified
    std::vector<std::string> exts = extensions;
    if (exts.empty()) {
        exts = GetSupportedExtensions();
    }

    for (const auto& file : files) {
        std::string ext = GetExtension(file);
        bool match = exts.empty() ||
                     std::find(exts.begin(), exts.end(), ext) != exts.end();

        if (match) {
            std::string fullPath = directory + "/" + file;
            try {
                QImage img;
                ReadImage(fullPath, img);
                if (!img.Empty()) {
                    images.push_back(std::move(img));
                }
            } catch (const std::exception&) {
                continue;
            }
        }
    }
}

int32_t WriteSequence(const std::vector<QImage>& images,
                       const std::string& pattern,
                       int32_t startIndex,
                       const CompressionParams& params) {
    int32_t count = 0;
    int32_t index = startIndex;

    ImageFormat format = GetFormatFromFilename(pattern);

    for (const auto& img : images) {
        std::string filename = FormatFilename(pattern, index);
        if (WriteImage(img, filename, format, params)) {
            ++count;
        }
        ++index;
    }

    return count;
}

} // namespace Qi::Vision::IO
