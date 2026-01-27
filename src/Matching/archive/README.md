# ShapeModel Archive - Legacy Methods

This directory contains archived implementations of ShapeModel matching algorithms
that were replaced by the optimized LINEMOD method.

## Archived Methods

### 1. AnglePyramid-based Matching (Auto/XLDContour modes)

**Files:**
- `ShapeModelCreate_Legacy.cpp` - Traditional edge-based model creation
- `ShapeModelSearch_Legacy.cpp` - Gradient-direction-based pyramid search

**Algorithm:**
- Uses AnglePyramid with continuous gradient angles
- Point extraction via edge detection + contrast thresholding
- Scoring via gradient direction similarity (cosine)
- Supports hysteresis thresholding for connected edge chains

**Performance:**
- Large images (1536x2048): ~180-200ms per search
- Small images (640x480): ~13ms per search

**Why Archived:**
- LINEMOD provides 3x faster search with similar accuracy
- The quantized orientation approach is more robust to noise
- OR spreading provides better spatial tolerance

### 2. ResponseMap Method

**Location:** `ShapeModelSearch_Legacy.cpp` (SearchPyramidWithResponseMap)

**Algorithm:**
- Precomputes response maps for all orientations at each pyramid level
- Direct LUT lookup for score computation
- Designed for scenarios with many rotations to test

**Why Archived:**
- Higher memory usage (stores all orientation responses)
- LINEMOD achieves similar speed without the memory overhead

## Usage Notes

These implementations are preserved for:
1. Research and comparison purposes
2. Potential future hybrid approaches
3. Documentation of the algorithm evolution

If you need to restore any of these methods, you can copy the relevant code
back to the main source files and add the appropriate conditional paths.

## Current LINEMOD Method - Known Limitations

### 1. Large Rotation Angle Score Degradation
LINEMOD uses 8-bin orientation quantization (45° per bin). Images with rotations
far from bin centers (0°, 45°, 90°, etc.) may have reduced scores:
- Up to 30% score reduction for extreme angles (e.g., 155°, 337°)
- Mitigated by lowering coarse threshold to 0.6
- Some extreme rotations may still fail to match

### 2. WSL2 Performance Warmup
In WSL2 environments, first runs are ~40ms slower due to:
- Memory allocator cache warmup
- OS page cache population
- CPU branch prediction warmup

Cold run: ~100-110ms, Warm run: ~60-70ms

See `/TROUBLESHOOTING.md` for detailed solutions.

## Timeline

- Original implementation: 2024
- LINEMOD optimization: January 2025
- Archived: January 2025
- Known limitations documented: January 2025

## References

- Halcon `create_shape_model` documentation
- Steger et al., "Robust Template Matching" (2003)
- Original LINEMOD: Hinterstoisser et al. (2011)
