#pragma once

/**
 * @file Window.h
 * @brief Lightweight GUI window for image display and debugging
 *
 * Cross-platform window implementation:
 * - Linux: X11 (Xlib)
 * - Windows: Win32 GDI
 *
 * Features:
 * - Display images with auto-scaling
 * - Wait for key press
 * - Adjustable window size
 */

#include <QiVision/Core/QImage.h>
#include <string>
#include <memory>

namespace Qi::Vision::GUI {

/**
 * @brief Scale mode for image display
 */
enum class ScaleMode {
    None,       ///< No scaling (1:1 pixel)
    Fit,        ///< Fit image to window (maintain aspect ratio)
    Fill,       ///< Fill window (may crop)
    Stretch     ///< Stretch to window size (ignore aspect ratio)
};

/**
 * @brief Lightweight window for image display
 *
 * Example usage:
 * @code
 * Window win("Debug", 800, 600);
 * win.DispImage(image);
 * win.WaitKey();  // Wait for any key
 *
 * // Or with timeout
 * while (win.WaitKey(30) != 'q') {
 *     win.DispImage(processedImage);
 * }
 * @endcode
 */
class Window {
public:
    /**
     * @brief Create a window
     * @param title Window title
     * @param width Initial window width (0 = auto from first image)
     * @param height Initial window height (0 = auto from first image)
     */
    Window(const std::string& title = "QiVision", int32_t width = 0, int32_t height = 0);

    /**
     * @brief Destructor - closes window
     */
    ~Window();

    // Non-copyable
    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    // Movable
    Window(Window&& other) noexcept;
    Window& operator=(Window&& other) noexcept;

    /**
     * @brief Display an image in the window
     * @param image Image to display (grayscale or RGB)
     * @param scaleMode How to scale the image
     */
    void DispImage(const QImage& image, ScaleMode scaleMode = ScaleMode::Fit);

    /**
     * @brief Wait for a key press
     * @param timeoutMs Timeout in milliseconds (0 = wait forever, -1 = no wait)
     * @return Key code pressed, or -1 if timeout/closed
     */
    int32_t WaitKey(int32_t timeoutMs = 0);

    /**
     * @brief Check if window is still open
     */
    bool IsOpen() const;

    /**
     * @brief Close the window
     */
    void Close();

    /**
     * @brief Set window title
     */
    void SetTitle(const std::string& title);

    /**
     * @brief Resize window
     */
    void Resize(int32_t width, int32_t height);

    /**
     * @brief Get current window size
     */
    void GetSize(int32_t& width, int32_t& height) const;

    /**
     * @brief Move window to position
     */
    void Move(int32_t x, int32_t y);

    /**
     * @brief Enable/disable auto-resize mode
     * @param enable If true, window resizes to fit each image (with max size limit)
     * @param maxWidth Maximum window width (0 = screen width)
     * @param maxHeight Maximum window height (0 = screen height)
     */
    void SetAutoResize(bool enable, int32_t maxWidth = 0, int32_t maxHeight = 0);

    /**
     * @brief Check if auto-resize is enabled
     */
    bool IsAutoResize() const;

    // =========================================================================
    // Static convenience functions
    // =========================================================================

    /**
     * @brief Quick display - create window, show image, wait for key
     * @param image Image to display
     * @param title Window title
     * @return Key code pressed
     */
    static int32_t ShowImage(const QImage& image, const std::string& title = "QiVision");

    /**
     * @brief Quick display with timeout
     */
    static int32_t ShowImage(const QImage& image, const std::string& title, int32_t timeoutMs);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

// =============================================================================
// Convenience functions (Halcon-style)
// =============================================================================

/**
 * @brief Display image in a window (creates window if needed)
 * @param image Image to display
 * @param windowName Window name (creates new if not exists)
 */
void DispImage(const QImage& image, const std::string& windowName = "QiVision");

/**
 * @brief Wait for key press on any window
 * @param timeoutMs Timeout in milliseconds (0 = forever)
 * @return Key code, or -1 on timeout
 */
int32_t WaitKey(int32_t timeoutMs = 0);

/**
 * @brief Close a named window
 */
void CloseWindow(const std::string& windowName);

/**
 * @brief Close all windows
 */
void CloseAllWindows();

} // namespace Qi::Vision::GUI
