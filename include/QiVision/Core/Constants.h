#pragma once

/**
 * @file Constants.h
 * @brief Mathematical and precision constants for QiVision
 */

#include <cmath>
#include <cstdint>
#include <limits>

namespace Qi::Vision {

// =============================================================================
// Mathematical Constants
// =============================================================================

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;
constexpr double HALF_PI = PI / 2.0;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;

// =============================================================================
// Precision Constants
// =============================================================================

/// Tolerance for floating point comparison
constexpr double EPSILON = 1e-9;

/// Tolerance for geometric comparisons (pixels)
constexpr double PIXEL_EPSILON = 1e-6;

/// Tolerance for angle comparisons (radians)
constexpr double ANGLE_EPSILON = 1e-6;

// =============================================================================
// Algorithm Limits
// =============================================================================

/// Maximum pyramid levels
constexpr int MAX_PYRAMID_LEVELS = 10;

/// Maximum image dimension (supports line scan cameras)
constexpr int32_t MAX_IMAGE_DIMENSION = 65536;

/// Memory alignment for SIMD (AVX512)
constexpr size_t MEMORY_ALIGNMENT = 64;

// =============================================================================
// Default Parameters (defined after Types.h is included)
// =============================================================================

// Note: DEFAULT_BORDER_TYPE and DEFAULT_INTERPOLATION are defined in Types.h
// to avoid circular dependency

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Check if two doubles are approximately equal
 */
inline bool ApproxEqual(double a, double b, double epsilon = EPSILON) {
    return std::abs(a - b) <= epsilon;
}

/**
 * @brief Check if a double is approximately zero
 */
inline bool ApproxZero(double a, double epsilon = EPSILON) {
    return std::abs(a) <= epsilon;
}

/**
 * @brief Clamp value to range
 */
template<typename T>
inline T Clamp(T value, T minVal, T maxVal) {
    return value < minVal ? minVal : (value > maxVal ? maxVal : value);
}

/**
 * @brief Convert degrees to radians
 */
inline double DegToRad(double degrees) {
    return degrees * DEG_TO_RAD;
}

/**
 * @brief Convert radians to degrees
 */
inline double RadToDeg(double radians) {
    return radians * RAD_TO_DEG;
}

/**
 * @brief Normalize angle to [-PI, PI]
 */
inline double NormalizeAngle(double angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

/**
 * @brief Square of a value
 */
template<typename T>
inline T Square(T x) {
    return x * x;
}

} // namespace Qi::Vision
