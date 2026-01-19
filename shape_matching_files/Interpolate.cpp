/**
 * @file Interpolate.cpp
 * @brief Interpolation function implementations
 */

#include <QiVision/Internal/Interpolate.h>

namespace Qi::Vision::Internal {

int32_t HandleBorder(int32_t coord, int32_t size, BorderMode mode) {
    if (size <= 0) return 0;

    // If coordinate is within bounds, return as-is
    if (coord >= 0 && coord < size) {
        return coord;
    }

    switch (mode) {
        case BorderMode::Constant:
            // Should not reach here - caller handles this case
            return (coord < 0) ? 0 : size - 1;

        case BorderMode::Replicate:
            // Clamp to edge
            if (coord < 0) return 0;
            if (coord >= size) return size - 1;
            return coord;

        case BorderMode::Reflect:
            // Mirror including edge: cba|abcd|dcb
            // For size=4: -1 -> 0, -2 -> 1, 4 -> 3, 5 -> 2
            {
                int32_t period = 2 * size;
                coord = coord % period;
                if (coord < 0) coord += period;
                if (coord >= size) {
                    coord = period - 1 - coord;
                }
                return coord;
            }

        case BorderMode::Reflect101:
            // Mirror excluding edge: dcb|abcd|cba
            // For size=4: -1 -> 1, -2 -> 2, 4 -> 2, 5 -> 1
            {
                if (size == 1) return 0;
                int32_t period = 2 * (size - 1);
                coord = coord % period;
                if (coord < 0) coord += period;
                if (coord >= size) {
                    coord = period - coord;
                }
                return coord;
            }

        case BorderMode::Wrap:
            // Wrap around: bcd|abcd|abc
            {
                coord = coord % size;
                if (coord < 0) coord += size;
                return coord;
            }

        default:
            return (coord < 0) ? 0 : size - 1;
    }
}

} // namespace Qi::Vision::Internal
