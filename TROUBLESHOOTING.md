# QiVision Troubleshooting Guide

## Shape Matching (LINEMOD)

### Issue: Performance varies between cold and warm runs

**Symptoms:**
- First run of LINEMOD search takes ~100ms
- Subsequent runs take ~60-70ms (after system warmup)
- Running other tests first, then LINEMOD shows faster times

**Root Cause:**
This is a system-level warmup effect, particularly noticeable in WSL2 environments:
- Memory allocator cache needs to be warmed
- OS page cache needs to be populated
- CPU branch prediction/instruction cache warm-up
- Memory pages mapped but not yet physically allocated

**Benchmark Impact:**
- ~40ms difference between cold and warm runs
- Cold run: ~100-110ms
- Warm run: ~60-70ms

**Solutions:**
1. Run a warmup iteration before measuring performance
2. Average multiple runs for accurate benchmarks
3. Run tests in sequence to ensure warm state
4. For production, the system naturally warms up after initial processing

**Example benchmark code:**
```cpp
// Warmup run (discard result)
model.Find(searchImage, params);

// Actual measurement
Timer timer;
timer.Start();
auto results = model.Find(searchImage, params);
double searchTime = timer.ElapsedMs();
```

---

### Issue: Large rotation angles have lower matching scores

**Symptoms:**
- Images rotated near 0°, 90°, 180°, 270° match well
- Images rotated to intermediate angles (e.g., 155°, 337°) may fail to match
- Score drops below threshold despite correct template presence

**Root Cause:**
LINEMOD uses 8-bin orientation quantization (45° per bin). When the template and search image have orientations that fall into different bins, the score degrades:
- 8-bin quantization: 0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°
- OR spreading mitigates this but doesn't completely solve it
- Large rotations (>45° from nearest bin center) suffer most

**Impact:**
- Up to 30% score reduction for extreme angles
- Some matches may fall below the coarse search threshold

**Solutions:**
1. Lower `minScore` parameter (e.g., 0.5 instead of 0.7)
2. The coarse threshold is automatically adjusted to 0.6 to allow more candidates
3. For rotation-heavy applications, consider using finer angle quantization (16-bin)
4. Post-filter results by verifying with grayscale correlation if needed

**Known Limitations:**
- Small templates with large rotations (>120° from reference) may not match
- This is an inherent trade-off in LINEMOD's speed vs rotation invariance

---

### Issue: Small templates have fewer matching features

**Symptoms:**
- Small templates (< 100x100 pixels) have fewer model points
- Lower matching robustness compared to larger templates
- May need lower contrast thresholds

**Solutions:**
1. Use lower `contrastHigh` and `contrastLow` values (e.g., 10 and 5)
2. Ensure ROI captures the full object with some margin
3. Consider using `MetricMode::IgnoreLocalPolarity` for more features
4. Pyramid levels are automatically adjusted based on template size

---

## Build Issues

### Issue: Compilation errors with SIMD

**Symptoms:**
- Errors about undefined AVX/SSE intrinsics
- Crash on older CPUs without AVX2 support

**Solutions:**
1. Ensure `-mavx2` flag is set in CMakeLists.txt
2. For older CPUs, compile without AVX2 by setting `USE_AVX2=OFF`
3. The code includes scalar fallbacks for unsupported SIMD

---

## Test Image Issues

### Issue: Test images not found

**Symptoms:**
- "Failed to load template!" error
- File not found errors

**Solutions:**
1. Ensure test images are in `tests/data/matching/image1/` and `image2/`
2. Run tests from the project root directory
3. Check file paths are correct (case-sensitive on Linux)

---

## Performance Tuning

### Recommended parameters for different scenarios:

| Scenario | minScore | greediness | contrastHigh | contrastLow |
|----------|----------|------------|--------------|-------------|
| High contrast objects | 0.7 | 0.9 | 30 | 15 |
| Low contrast objects | 0.5 | 0.8 | 10 | 5 |
| Noisy images | 0.6 | 0.85 | 20 | 10 |
| Large rotations expected | 0.5 | 0.8 | 15 | 8 |

### Pyramid levels:
- Auto-calculated based on template size
- Smaller templates → fewer levels
- Manual override: set `numLevels` in `ModelParams`
