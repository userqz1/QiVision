/**
 * @file Window.cpp
 * @brief Cross-platform window implementation
 *
 * Supported platforms:
 * - Linux: X11 (Xlib)
 * - Windows: Win32 GDI
 * - macOS: Cocoa (AppKit) - requires .mm compilation
 * - Android: Stub (GUI requires Java layer)
 * - iOS: Stub (GUI requires Swift/ObjC layer)
 */

#include <QiVision/GUI/Window.h>

#include <algorithm>
#include <unordered_map>
#include <mutex>
#include <cstring>

// =============================================================================
// Platform detection
// =============================================================================

#if defined(_WIN32) || defined(_WIN64)
    #define QIVISION_PLATFORM_WINDOWS
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #include <windows.h>

#elif defined(__APPLE__)
    #include <TargetConditionals.h>
    #if TARGET_OS_IPHONE || TARGET_IPHONE_SIMULATOR
        #define QIVISION_PLATFORM_IOS
        // iOS: GUI requires Swift/ObjC layer, use stub
    #else
        #define QIVISION_PLATFORM_MACOS
        #ifdef QIVISION_HAS_COCOA
            // macOS with Cocoa support (requires .mm compilation)
            #import <Cocoa/Cocoa.h>
        #endif
    #endif

#elif defined(__ANDROID__)
    #define QIVISION_PLATFORM_ANDROID
    // Android: GUI requires Java layer (Activity/View), use stub
    // Native code can use ANativeWindow but cannot create windows

#elif defined(__linux__)
    #define QIVISION_PLATFORM_LINUX
    #ifdef QIVISION_HAS_X11
        #include <X11/Xlib.h>
        #include <X11/Xutil.h>
        #include <X11/keysym.h>
        #include <unistd.h>
        #include <sys/time.h>
        // X11 defines 'None' as a macro, which conflicts with ScaleMode::None
        #ifdef None
            #undef None
        #endif
    #endif
#endif

namespace Qi::Vision::GUI {

// =============================================================================
// Global window manager for convenience functions
// =============================================================================

static std::unordered_map<std::string, std::unique_ptr<Window>> g_windows;
static std::mutex g_windowsMutex;

// =============================================================================
// Platform-specific implementation
// =============================================================================

#if defined(QIVISION_PLATFORM_LINUX) && defined(QIVISION_HAS_X11)

class Window::Impl {
public:
    Display* display_ = nullptr;
    ::Window window_ = 0;
    GC gc_ = nullptr;
    XImage* ximage_ = nullptr;
    Atom wmDeleteMessage_ = 0;

    int32_t width_ = 0;
    int32_t height_ = 0;
    bool isOpen_ = false;
    std::string title_;

    // Image buffer for display
    std::vector<uint8_t> buffer_;
    int32_t imageWidth_ = 0;
    int32_t imageHeight_ = 0;

    // Auto-resize settings
    bool autoResize_ = false;
    int32_t maxWidth_ = 0;
    int32_t maxHeight_ = 0;
    int32_t screenWidth_ = 0;
    int32_t screenHeight_ = 0;

    Impl(const std::string& title, int32_t width, int32_t height)
        : width_(width), height_(height), title_(title) {

        display_ = XOpenDisplay(nullptr);
        if (!display_) {
            return;
        }

        int screen = DefaultScreen(display_);

        // Get screen size for auto-resize limits
        screenWidth_ = DisplayWidth(display_, screen);
        screenHeight_ = DisplayHeight(display_, screen);

        // If size not specified, use reasonable default
        if (width_ <= 0) width_ = 800;
        if (height_ <= 0) height_ = 600;

        window_ = XCreateSimpleWindow(
            display_,
            RootWindow(display_, screen),
            100, 100,
            width_, height_,
            1,
            BlackPixel(display_, screen),
            WhitePixel(display_, screen)
        );

        if (!window_) {
            XCloseDisplay(display_);
            display_ = nullptr;
            return;
        }

        // Set window title
        XStoreName(display_, window_, title_.c_str());

        // Select input events
        XSelectInput(display_, window_,
                     ExposureMask | KeyPressMask | StructureNotifyMask);

        // Handle window close button
        wmDeleteMessage_ = XInternAtom(display_, "WM_DELETE_WINDOW", False);
        XSetWMProtocols(display_, window_, &wmDeleteMessage_, 1);

        // Create graphics context
        gc_ = XCreateGC(display_, window_, 0, nullptr);

        // Show window
        XMapWindow(display_, window_);
        XFlush(display_);

        isOpen_ = true;
    }

    ~Impl() {
        Close();
    }

    void Close() {
        if (ximage_) {
            // Don't destroy data, we own it
            ximage_->data = nullptr;
            XDestroyImage(ximage_);
            ximage_ = nullptr;
        }
        if (gc_) {
            XFreeGC(display_, gc_);
            gc_ = nullptr;
        }
        if (window_) {
            XDestroyWindow(display_, window_);
            window_ = 0;
        }
        if (display_) {
            XCloseDisplay(display_);
            display_ = nullptr;
        }
        isOpen_ = false;
    }

    void Show(const QImage& image, ScaleMode scaleMode) {
        if (!isOpen_ || !display_) return;

        int32_t srcWidth = image.Width();
        int32_t srcHeight = image.Height();

        // Auto-resize window to fit image
        if (autoResize_) {
            int32_t maxW = (maxWidth_ > 0) ? maxWidth_ : (screenWidth_ - 100);
            int32_t maxH = (maxHeight_ > 0) ? maxHeight_ : (screenHeight_ - 100);

            int32_t newWidth = srcWidth;
            int32_t newHeight = srcHeight;

            // Scale down if image is larger than max size
            if (newWidth > maxW || newHeight > maxH) {
                double scaleW = static_cast<double>(maxW) / srcWidth;
                double scaleH = static_cast<double>(maxH) / srcHeight;
                double scale = std::min(scaleW, scaleH);
                newWidth = static_cast<int32_t>(srcWidth * scale);
                newHeight = static_cast<int32_t>(srcHeight * scale);
            }

            // Resize window if size changed
            if (newWidth != width_ || newHeight != height_) {
                width_ = newWidth;
                height_ = newHeight;
                XResizeWindow(display_, window_, width_, height_);
                XFlush(display_);
            }
        }

        int32_t dstWidth = width_;
        int32_t dstHeight = height_;

        // Calculate display size based on scale mode
        double scaleX = 1.0, scaleY = 1.0;

        switch (scaleMode) {
            case ScaleMode::None:
                dstWidth = srcWidth;
                dstHeight = srcHeight;
                break;

            case ScaleMode::Fit: {
                double scaleW = static_cast<double>(width_) / srcWidth;
                double scaleH = static_cast<double>(height_) / srcHeight;
                double scale = std::min(scaleW, scaleH);
                dstWidth = static_cast<int32_t>(srcWidth * scale);
                dstHeight = static_cast<int32_t>(srcHeight * scale);
                scaleX = scaleY = scale;
                break;
            }

            case ScaleMode::Fill: {
                double scaleW = static_cast<double>(width_) / srcWidth;
                double scaleH = static_cast<double>(height_) / srcHeight;
                double scale = std::max(scaleW, scaleH);
                scaleX = scaleY = scale;
                dstWidth = width_;
                dstHeight = height_;
                break;
            }

            case ScaleMode::Stretch:
                scaleX = static_cast<double>(width_) / srcWidth;
                scaleY = static_cast<double>(height_) / srcHeight;
                dstWidth = width_;
                dstHeight = height_;
                break;
        }

        imageWidth_ = dstWidth;
        imageHeight_ = dstHeight;

        const uint8_t* srcData = static_cast<const uint8_t*>(image.Data());
        size_t srcStride = image.Stride();
        int channels = image.Channels();

        // Create/update XImage
        if (ximage_) {
            ximage_->data = nullptr;
            XDestroyImage(ximage_);
            ximage_ = nullptr;
        }

        int screen = DefaultScreen(display_);
        Visual* visual = DefaultVisual(display_, screen);
        int depth = DefaultDepth(display_, screen);

        // First create XImage to get the proper bytes_per_line
        ximage_ = XCreateImage(
            display_, visual, depth, ZPixmap, 0,
            nullptr,  // Don't set data yet
            dstWidth, dstHeight, 32, 0  // Let X11 decide bytes_per_line
        );

        if (!ximage_) return;

        // Use X11's calculated bytes_per_line for proper alignment
        int xStride = ximage_->bytes_per_line;
        buffer_.resize(xStride * dstHeight);
        std::memset(buffer_.data(), 0, buffer_.size());

        // Scale with area averaging (box filter) for downscaling
        // This prevents thin lines from disappearing when shrinking
        bool useAreaAverage = (scaleX < 1.0 || scaleY < 1.0);

        for (int32_t dy = 0; dy < dstHeight; ++dy) {
            for (int32_t dx = 0; dx < dstWidth; ++dx) {
                uint8_t* dst = &buffer_[dy * xStride + dx * 4];

                if (useAreaAverage) {
                    // Calculate the source region that maps to this destination pixel
                    double srcX0 = dx / scaleX;
                    double srcY0 = dy / scaleY;
                    double srcX1 = (dx + 1) / scaleX;
                    double srcY1 = (dy + 1) / scaleY;

                    int32_t ix0 = static_cast<int32_t>(srcX0);
                    int32_t iy0 = static_cast<int32_t>(srcY0);
                    int32_t ix1 = std::min(static_cast<int32_t>(srcX1) + 1, srcWidth);
                    int32_t iy1 = std::min(static_cast<int32_t>(srcY1) + 1, srcHeight);

                    // Average all pixels in the source region
                    double sumR = 0, sumG = 0, sumB = 0;
                    int count = 0;

                    for (int32_t sy = iy0; sy < iy1; ++sy) {
                        for (int32_t sx = ix0; sx < ix1; ++sx) {
                            if (channels == 1) {
                                uint8_t gray = srcData[sy * srcStride + sx];
                                sumR += gray;
                                sumG += gray;
                                sumB += gray;
                            } else if (channels == 3) {
                                const uint8_t* src = &srcData[sy * srcStride + sx * 3];
                                sumR += src[0];
                                sumG += src[1];
                                sumB += src[2];
                            }
                            count++;
                        }
                    }

                    if (count > 0) {
                        dst[0] = static_cast<uint8_t>(sumB / count);  // B
                        dst[1] = static_cast<uint8_t>(sumG / count);  // G
                        dst[2] = static_cast<uint8_t>(sumR / count);  // R
                        dst[3] = 255;
                    }
                } else {
                    // Nearest neighbor for upscaling
                    int32_t sx = static_cast<int32_t>(dx / scaleX);
                    int32_t sy = static_cast<int32_t>(dy / scaleY);
                    if (sx >= srcWidth) sx = srcWidth - 1;
                    if (sy >= srcHeight) sy = srcHeight - 1;

                    if (channels == 1) {
                        uint8_t gray = srcData[sy * srcStride + sx];
                        dst[0] = gray;  // B
                        dst[1] = gray;  // G
                        dst[2] = gray;  // R
                        dst[3] = 255;   // A
                    } else if (channels == 3) {
                        const uint8_t* src = &srcData[sy * srcStride + sx * 3];
                        dst[0] = src[2];  // B
                        dst[1] = src[1];  // G
                        dst[2] = src[0];  // R
                        dst[3] = 255;     // A
                    }
                }
            }
        }

        ximage_->data = reinterpret_cast<char*>(buffer_.data());

        // Clear window and draw image centered
        XClearWindow(display_, window_);

        int offsetX = (width_ - dstWidth) / 2;
        int offsetY = (height_ - dstHeight) / 2;
        if (offsetX < 0) offsetX = 0;
        if (offsetY < 0) offsetY = 0;

        XPutImage(display_, window_, gc_, ximage_,
                  0, 0, offsetX, offsetY, dstWidth, dstHeight);
        XFlush(display_);
    }

    int32_t WaitKey(int32_t timeoutMs) {
        if (!isOpen_ || !display_) return -1;

        // Calculate end time
        struct timeval startTime, currentTime;
        gettimeofday(&startTime, nullptr);

        while (isOpen_) {
            // Check for events
            while (XPending(display_)) {
                XEvent event;
                XNextEvent(display_, &event);

                switch (event.type) {
                    case Expose:
                        // Redraw
                        if (ximage_) {
                            int offsetX = (width_ - imageWidth_) / 2;
                            int offsetY = (height_ - imageHeight_) / 2;
                            if (offsetX < 0) offsetX = 0;
                            if (offsetY < 0) offsetY = 0;
                            XPutImage(display_, window_, gc_, ximage_,
                                      0, 0, offsetX, offsetY, imageWidth_, imageHeight_);
                            XFlush(display_);
                        }
                        break;

                    case KeyPress: {
                        KeySym keysym = XLookupKeysym(&event.xkey, 0);
                        // Convert to ASCII if possible
                        if (keysym >= XK_space && keysym <= XK_asciitilde) {
                            return static_cast<int32_t>(keysym);
                        }
                        // Return special keys as negative
                        return static_cast<int32_t>(keysym & 0xFFFF);
                    }

                    case ConfigureNotify:
                        width_ = event.xconfigure.width;
                        height_ = event.xconfigure.height;
                        break;

                    case ClientMessage:
                        if (static_cast<Atom>(event.xclient.data.l[0]) == wmDeleteMessage_) {
                            isOpen_ = false;
                            return -1;
                        }
                        break;
                }
            }

            // Check timeout
            if (timeoutMs > 0) {
                gettimeofday(&currentTime, nullptr);
                long elapsedMs = (currentTime.tv_sec - startTime.tv_sec) * 1000 +
                                 (currentTime.tv_usec - startTime.tv_usec) / 1000;
                if (elapsedMs >= timeoutMs) {
                    return -1;
                }
            } else if (timeoutMs < 0) {
                // No wait mode
                return -1;
            }

            // Small sleep to avoid busy waiting
            usleep(10000);  // 10ms
        }

        return -1;
    }

    void SetTitle(const std::string& title) {
        title_ = title;
        if (display_ && window_) {
            XStoreName(display_, window_, title_.c_str());
            XFlush(display_);
        }
    }

    void Resize(int32_t width, int32_t height) {
        width_ = width;
        height_ = height;
        if (display_ && window_) {
            XResizeWindow(display_, window_, width, height);
            XFlush(display_);
        }
    }

    void Move(int32_t x, int32_t y) {
        if (display_ && window_) {
            XMoveWindow(display_, window_, x, y);
            XFlush(display_);
        }
    }

    void SetAutoResize(bool enable, int32_t maxWidth, int32_t maxHeight) {
        autoResize_ = enable;
        maxWidth_ = maxWidth;
        maxHeight_ = maxHeight;
    }

    bool IsAutoResize() const {
        return autoResize_;
    }
};

#endif // QIVISION_PLATFORM_LINUX && QIVISION_HAS_X11

#ifdef QIVISION_PLATFORM_WINDOWS

class Window::Impl {
public:
    HWND hwnd_ = nullptr;
    HDC hdc_ = nullptr;
    HBITMAP hbitmap_ = nullptr;
    HDC memDC_ = nullptr;

    int32_t width_ = 0;
    int32_t height_ = 0;
    bool isOpen_ = false;
    std::string title_;

    std::vector<uint8_t> buffer_;
    int32_t imageWidth_ = 0;
    int32_t imageHeight_ = 0;

    // Auto-resize settings
    bool autoResize_ = false;
    int32_t maxWidth_ = 0;
    int32_t maxHeight_ = 0;
    int32_t screenWidth_ = 0;
    int32_t screenHeight_ = 0;

    static LRESULT CALLBACK WindowProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
        Impl* impl = reinterpret_cast<Impl*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));

        switch (msg) {
            case WM_DESTROY:
                if (impl) impl->isOpen_ = false;
                return 0;

            case WM_SIZE:
                if (impl) {
                    impl->width_ = LOWORD(lParam);
                    impl->height_ = HIWORD(lParam);
                }
                return 0;

            case WM_PAINT: {
                PAINTSTRUCT ps;
                HDC hdc = BeginPaint(hwnd, &ps);
                if (impl && impl->memDC_) {
                    int offsetX = (impl->width_ - impl->imageWidth_) / 2;
                    int offsetY = (impl->height_ - impl->imageHeight_) / 2;
                    if (offsetX < 0) offsetX = 0;
                    if (offsetY < 0) offsetY = 0;

                    // Clear background
                    RECT rect;
                    GetClientRect(hwnd, &rect);
                    FillRect(hdc, &rect, (HBRUSH)GetStockObject(BLACK_BRUSH));

                    // Draw image
                    BitBlt(hdc, offsetX, offsetY, impl->imageWidth_, impl->imageHeight_,
                           impl->memDC_, 0, 0, SRCCOPY);
                }
                EndPaint(hwnd, &ps);
                return 0;
            }
        }
        return DefWindowProc(hwnd, msg, wParam, lParam);
    }

    Impl(const std::string& title, int32_t width, int32_t height)
        : width_(width), height_(height), title_(title) {

        // Get screen size for auto-resize limits
        screenWidth_ = GetSystemMetrics(SM_CXSCREEN);
        screenHeight_ = GetSystemMetrics(SM_CYSCREEN);

        if (width_ <= 0) width_ = 800;
        if (height_ <= 0) height_ = 600;

        // Register window class
        static bool classRegistered = false;
        static const char* className = "QiVisionWindow";

        if (!classRegistered) {
            WNDCLASSA wc = {};
            wc.lpfnWndProc = WindowProc;
            wc.hInstance = GetModuleHandle(nullptr);
            wc.lpszClassName = className;
            wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
            wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
            RegisterClassA(&wc);
            classRegistered = true;
        }

        // Adjust window size to account for borders
        RECT rect = {0, 0, width_, height_};
        AdjustWindowRect(&rect, WS_OVERLAPPEDWINDOW, FALSE);

        hwnd_ = CreateWindowA(
            className, title_.c_str(),
            WS_OVERLAPPEDWINDOW,
            CW_USEDEFAULT, CW_USEDEFAULT,
            rect.right - rect.left, rect.bottom - rect.top,
            nullptr, nullptr, GetModuleHandle(nullptr), nullptr
        );

        if (!hwnd_) return;

        SetWindowLongPtr(hwnd_, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));

        hdc_ = GetDC(hwnd_);
        memDC_ = CreateCompatibleDC(hdc_);

        ShowWindow(hwnd_, SW_SHOW);
        UpdateWindow(hwnd_);

        isOpen_ = true;
    }

    ~Impl() {
        Close();
    }

    void Close() {
        if (hbitmap_) {
            DeleteObject(hbitmap_);
            hbitmap_ = nullptr;
        }
        if (memDC_) {
            DeleteDC(memDC_);
            memDC_ = nullptr;
        }
        if (hdc_ && hwnd_) {
            ReleaseDC(hwnd_, hdc_);
            hdc_ = nullptr;
        }
        if (hwnd_) {
            DestroyWindow(hwnd_);
            hwnd_ = nullptr;
        }
        isOpen_ = false;
    }

    void Show(const QImage& image, ScaleMode scaleMode) {
        if (!isOpen_ || !hwnd_) return;

        int32_t srcWidth = image.Width();
        int32_t srcHeight = image.Height();

        // Auto-resize window to fit image
        if (autoResize_) {
            int32_t maxW = (maxWidth_ > 0) ? maxWidth_ : (screenWidth_ - 100);
            int32_t maxH = (maxHeight_ > 0) ? maxHeight_ : (screenHeight_ - 100);

            int32_t newWidth = srcWidth;
            int32_t newHeight = srcHeight;

            // Scale down if image is larger than max size
            if (newWidth > maxW || newHeight > maxH) {
                double scaleW = static_cast<double>(maxW) / srcWidth;
                double scaleH = static_cast<double>(maxH) / srcHeight;
                double scale = std::min(scaleW, scaleH);
                newWidth = static_cast<int32_t>(srcWidth * scale);
                newHeight = static_cast<int32_t>(srcHeight * scale);
            }

            // Resize window if size changed
            if (newWidth != width_ || newHeight != height_) {
                width_ = newWidth;
                height_ = newHeight;
                RECT rect = {0, 0, width_, height_};
                AdjustWindowRect(&rect, WS_OVERLAPPEDWINDOW, FALSE);
                SetWindowPos(hwnd_, nullptr, 0, 0,
                             rect.right - rect.left, rect.bottom - rect.top,
                             SWP_NOMOVE | SWP_NOZORDER);
            }
        }

        int32_t dstWidth = width_;
        int32_t dstHeight = height_;

        double scaleX = 1.0, scaleY = 1.0;

        switch (scaleMode) {
            case ScaleMode::None:
                dstWidth = srcWidth;
                dstHeight = srcHeight;
                break;

            case ScaleMode::Fit: {
                double scaleW = static_cast<double>(width_) / srcWidth;
                double scaleH = static_cast<double>(height_) / srcHeight;
                double scale = std::min(scaleW, scaleH);
                dstWidth = static_cast<int32_t>(srcWidth * scale);
                dstHeight = static_cast<int32_t>(srcHeight * scale);
                scaleX = scaleY = scale;
                break;
            }

            case ScaleMode::Fill: {
                double scaleW = static_cast<double>(width_) / srcWidth;
                double scaleH = static_cast<double>(height_) / srcHeight;
                double scale = std::max(scaleW, scaleH);
                scaleX = scaleY = scale;
                dstWidth = width_;
                dstHeight = height_;
                break;
            }

            case ScaleMode::Stretch:
                scaleX = static_cast<double>(width_) / srcWidth;
                scaleY = static_cast<double>(height_) / srcHeight;
                dstWidth = width_;
                dstHeight = height_;
                break;
        }

        imageWidth_ = dstWidth;
        imageHeight_ = dstHeight;

        // Create DIB
        BITMAPINFO bmi = {};
        bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        bmi.bmiHeader.biWidth = dstWidth;
        bmi.bmiHeader.biHeight = -dstHeight;  // Top-down
        bmi.bmiHeader.biPlanes = 1;
        bmi.bmiHeader.biBitCount = 32;
        bmi.bmiHeader.biCompression = BI_RGB;

        void* bits = nullptr;
        HBITMAP newBitmap = CreateDIBSection(hdc_, &bmi, DIB_RGB_COLORS, &bits, nullptr, 0);
        if (!newBitmap) return;

        // Scale and convert to BGRA
        const uint8_t* srcData = static_cast<const uint8_t*>(image.Data());
        size_t srcStride = image.Stride();
        int channels = image.Channels();
        uint8_t* dstData = static_cast<uint8_t*>(bits);

        for (int32_t dy = 0; dy < dstHeight; ++dy) {
            int32_t sy = static_cast<int32_t>(dy / scaleY);
            if (sy >= srcHeight) sy = srcHeight - 1;

            for (int32_t dx = 0; dx < dstWidth; ++dx) {
                int32_t sx = static_cast<int32_t>(dx / scaleX);
                if (sx >= srcWidth) sx = srcWidth - 1;

                uint8_t* dst = &dstData[(dy * dstWidth + dx) * 4];

                if (channels == 1) {
                    uint8_t gray = srcData[sy * srcStride + sx];
                    dst[0] = gray;  // B
                    dst[1] = gray;  // G
                    dst[2] = gray;  // R
                    dst[3] = 255;   // A
                } else if (channels == 3) {
                    const uint8_t* src = &srcData[sy * srcStride + sx * 3];
                    dst[0] = src[2];  // B
                    dst[1] = src[1];  // G
                    dst[2] = src[0];  // R
                    dst[3] = 255;     // A
                }
            }
        }

        // Replace bitmap
        if (hbitmap_) DeleteObject(hbitmap_);
        hbitmap_ = newBitmap;
        SelectObject(memDC_, hbitmap_);

        // Redraw
        InvalidateRect(hwnd_, nullptr, TRUE);
        UpdateWindow(hwnd_);
    }

    int32_t WaitKey(int32_t timeoutMs) {
        if (!isOpen_ || !hwnd_) return -1;

        DWORD startTime = GetTickCount();
        MSG msg;

        while (isOpen_) {
            // Process messages
            while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
                if (msg.message == WM_QUIT) {
                    isOpen_ = false;
                    return -1;
                }

                if (msg.message == WM_KEYDOWN && msg.hwnd == hwnd_) {
                    // Convert virtual key to character
                    int key = static_cast<int>(msg.wParam);
                    if (key >= 'A' && key <= 'Z') {
                        // Check if shift is pressed
                        if (!(GetKeyState(VK_SHIFT) & 0x8000)) {
                            key = key - 'A' + 'a';  // Convert to lowercase
                        }
                    }
                    return key;
                }

                TranslateMessage(&msg);
                DispatchMessage(&msg);
            }

            // Check timeout
            if (timeoutMs > 0) {
                DWORD elapsed = GetTickCount() - startTime;
                if (elapsed >= static_cast<DWORD>(timeoutMs)) {
                    return -1;
                }
            } else if (timeoutMs < 0) {
                return -1;
            }

            Sleep(10);  // Avoid busy waiting
        }

        return -1;
    }

    void SetTitle(const std::string& title) {
        title_ = title;
        if (hwnd_) {
            SetWindowTextA(hwnd_, title_.c_str());
        }
    }

    void Resize(int32_t width, int32_t height) {
        width_ = width;
        height_ = height;
        if (hwnd_) {
            RECT rect = {0, 0, width, height};
            AdjustWindowRect(&rect, WS_OVERLAPPEDWINDOW, FALSE);
            SetWindowPos(hwnd_, nullptr, 0, 0,
                         rect.right - rect.left, rect.bottom - rect.top,
                         SWP_NOMOVE | SWP_NOZORDER);
        }
    }

    void Move(int32_t x, int32_t y) {
        if (hwnd_) {
            SetWindowPos(hwnd_, nullptr, x, y, 0, 0, SWP_NOSIZE | SWP_NOZORDER);
        }
    }

    void SetAutoResize(bool enable, int32_t maxWidth, int32_t maxHeight) {
        autoResize_ = enable;
        maxWidth_ = maxWidth;
        maxHeight_ = maxHeight;
    }

    bool IsAutoResize() const {
        return autoResize_;
    }
};

#endif // QIVISION_PLATFORM_WINDOWS

// =============================================================================
// Stub implementation (no GUI available)
// =============================================================================

#if !defined(QIVISION_PLATFORM_WINDOWS) && !(defined(QIVISION_PLATFORM_LINUX) && defined(QIVISION_HAS_X11))

class Window::Impl {
public:
    int32_t width_ = 0;
    int32_t height_ = 0;
    bool isOpen_ = false;
    std::string title_;
    bool autoResize_ = false;

    Impl(const std::string& title, int32_t width, int32_t height)
        : width_(width > 0 ? width : 800)
        , height_(height > 0 ? height : 600)
        , title_(title)
        , isOpen_(true) {
        // Stub: No actual window created
    }

    ~Impl() = default;

    void Close() { isOpen_ = false; }

    void Show(const QImage& /*image*/, ScaleMode /*scaleMode*/) {
        // Stub: No-op
    }

    int32_t WaitKey(int32_t /*timeoutMs*/) {
        // Stub: Return immediately
        return -1;
    }

    void SetTitle(const std::string& title) { title_ = title; }
    void Resize(int32_t width, int32_t height) { width_ = width; height_ = height; }
    void Move(int32_t /*x*/, int32_t /*y*/) {}
    void SetAutoResize(bool enable, int32_t /*maxWidth*/, int32_t /*maxHeight*/) { autoResize_ = enable; }
    bool IsAutoResize() const { return autoResize_; }
};

#endif // Stub implementation

// =============================================================================
// Window class implementation
// =============================================================================

Window::Window(const std::string& title, int32_t width, int32_t height)
    : impl_(std::make_unique<Impl>(title, width, height)) {}

Window::~Window() = default;

Window::Window(Window&& other) noexcept = default;
Window& Window::operator=(Window&& other) noexcept = default;

void Window::DispImage(const QImage& image, ScaleMode scaleMode) {
    if (impl_) impl_->Show(image, scaleMode);
}

int32_t Window::WaitKey(int32_t timeoutMs) {
    return impl_ ? impl_->WaitKey(timeoutMs) : -1;
}

bool Window::IsOpen() const {
    return impl_ && impl_->isOpen_;
}

void Window::Close() {
    if (impl_) impl_->Close();
}

void Window::SetTitle(const std::string& title) {
    if (impl_) impl_->SetTitle(title);
}

void Window::Resize(int32_t width, int32_t height) {
    if (impl_) impl_->Resize(width, height);
}

void Window::GetSize(int32_t& width, int32_t& height) const {
    if (impl_) {
        width = impl_->width_;
        height = impl_->height_;
    } else {
        width = height = 0;
    }
}

void Window::Move(int32_t x, int32_t y) {
    if (impl_) impl_->Move(x, y);
}

void Window::SetAutoResize(bool enable, int32_t maxWidth, int32_t maxHeight) {
    if (impl_) impl_->SetAutoResize(enable, maxWidth, maxHeight);
}

bool Window::IsAutoResize() const {
    return impl_ ? impl_->IsAutoResize() : false;
}

int32_t Window::ShowImage(const QImage& image, const std::string& title) {
    Window win(title, image.Width(), image.Height());
    win.DispImage(image);
    return win.WaitKey();
}

int32_t Window::ShowImage(const QImage& image, const std::string& title, int32_t timeoutMs) {
    Window win(title, image.Width(), image.Height());
    win.DispImage(image);
    return win.WaitKey(timeoutMs);
}

// =============================================================================
// Convenience functions
// =============================================================================

void DispImage(const QImage& image, const std::string& windowName) {
    std::lock_guard<std::mutex> lock(g_windowsMutex);

    auto it = g_windows.find(windowName);
    if (it == g_windows.end()) {
        auto win = std::make_unique<Window>(windowName, image.Width(), image.Height());
        it = g_windows.emplace(windowName, std::move(win)).first;
    }
    it->second->DispImage(image);
}

int32_t WaitKey(int32_t timeoutMs) {
    std::lock_guard<std::mutex> lock(g_windowsMutex);

    // Wait on first available window
    for (auto& [name, win] : g_windows) {
        if (win && win->IsOpen()) {
            return win->WaitKey(timeoutMs);
        }
    }
    return -1;
}

void CloseWindow(const std::string& windowName) {
    std::lock_guard<std::mutex> lock(g_windowsMutex);
    g_windows.erase(windowName);
}

void CloseAllWindows() {
    std::lock_guard<std::mutex> lock(g_windowsMutex);
    g_windows.clear();
}

} // namespace Qi::Vision::GUI
