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

---

## 性能优化验证规则

**任何性能优化必须同时验证精度和时间：**

1. **优化前基准测试** - 记录精度（匹配成功率）和时间
2. **优化后验证** - 运行 `./build/bin/samples/matching_shape_match`
3. **验证标准** - 精度不能下降，时间不能变长
4. **不达标时处理流程**:
   - 先分析原因（测量波动？实现错误？方向错误？）
   - 多次测试确认（排除偶然因素）
   - 尝试改进方案（修复实现问题）
   - 确认方向不可行后再回滚并记录
5. **记录失败** - 仅在确认方向不可行时记录，避免因偶然因素误判

---

## 失败的优化尝试（已确认不可行）

### ❌ L1-norm 替代 L2-norm 计算梯度幅值 (2025-01-19)

**尝试内容:**
将梯度幅值计算从 `sqrt(gx² + gy²)` (L2-norm) 改为 `|gx| + |gy|` (L1-norm)，理论上可节省 sqrt 计算开销。

**预期收益:**
- L1-norm 比 L2-norm 快约 40%
- 对边缘检测"应该足够"

**实际结果:**

| 测试集 | L1-norm | L2-norm (原始) | 差异 |
|--------|---------|----------------|------|
| Small Images | 4/5, 9.7ms | 5/5, 7.8ms | ❌ 精度下降，时间变长 |
| Large Images | 11/11, 216.4ms | 11/11, 212.5ms | ❌ 时间变长 |
| Image3 | 66/66, 33.9ms | 66/66, 33.1ms | ❌ 时间变长 |
| Image4 | 20/20, 36.8ms | 20/20, 35.1ms | ❌ 时间变长 |

**失败原因分析:**
1. **阈值失效**: L1-norm 的值域范围与 L2-norm 不同 (L1 ≥ L2)，导致固定阈值判断偏移
2. **边缘点选取偏差**: 梯度幅值分布改变，影响边缘点提取质量
3. **候选点增加**: 边缘质量下降导致粗搜索产生更多误候选，反而拖慢整体速度
4. **精度损失**: 部分弱边缘被错误过滤，导致匹配失败

**教训:**
- 不能孤立优化单个计算步骤，需考虑对整个流水线的影响
- 优化后必须立即验证**精度**和**时间**，不达标立即回滚
- 改变数学公式（如范数类型）会影响下游所有依赖该值的逻辑

---

### ❌ Ring Buffer 替代 2-pass Separable 高斯下采样 (2026-01-19)

**尝试内容:**
将 GaussPyramid 的 2-pass 分离高斯滤波改为 5 行 ring buffer 版本，理论上可减少内存占用从 O(width*height) 到 O(width*5)。

**预期收益:**
- 内存占用降低 800x (2048x4001 图像: 16.4 MB → 20 KB)
- 更好的 cache 局部性
- 减少内存带宽需求

**实际结果 (两次尝试):**

| 版本 | Small | Large | 问题 |
|------|-------|-------|------|
| Separable (基线) | 7.6 ms | 147.4 ms | - |
| Ring v1 (错误: 用 gather) | 6.1 ms | 152.4 ms | gather 太慢 |
| Ring v2 (正确: 连续 load) | 8.3 ms | 154.8 ms | 仍慢 5% |

**第一次失败 - 实现错误:**
水平 pass 用了 `_mm256_i32gather_ps` 做 stride=2 访问，这是灾难性的错误。
Gather 指令把本应连续的 load 变成了每个 lane 随机取值，开销极大。

**第二次失败 - 正确实现仍慢的原因:**
1. **循环融合破坏了 streaming**: Separable 是两个大循环，像 memcpy 一样流水线跑满；
   Ring buffer 每个 dy 都要交替做 read 5 rows + write 2 rows + rotate pointers，
   更像小型调度器，指令开销和前端压力更大
2. **Small 也变慢说明不是带宽问题**: 如果是 DRAM/L3 瓶颈，Small 应该不受影响
3. **函数调用开销**: 每个 dy 调用两次 computeHorizontalRow lambda

**什么时候 ring buffer 可能赢:**
- 图像 > 8000x8000 (hBlur 超过 L3 容量)
- 多线程场景下 cache 严重竞争
- 当 Gauss 真正成为带宽瓶颈时

**结论:**
Ring buffer 代码保留但不使用。Separable 是当前最优解：
- 性能更好 (稳定快 5%)
- 代码更简单
- 维护成本更低

---

### ⏸️ 内存对齐优化 hBlur 缓冲区 (2026-01-19) - 低优先级

**尝试内容:**
将 Pyramid.cpp 中 `FusedGaussianDownsample2x_Separable` 的 `std::vector<float> hBlur` 替换为 `Platform::AllocateAligned<float>()` (32 字节对齐)。

**预期收益:**
- 减少跨 cache line 访问
- 可安全使用 aligned load/store 指令

**实际结果:**

| 测试集 | 基线 | 对齐版本 | 差异 |
|--------|------|----------|------|
| Large Images | 144.4 ms | 153.3 ms | ❌ +6.2% 变慢 |

**本次失败原因:**
1. **只对齐内存，未改用 aligned 指令**: 仍用 `_mm256_loadu_ps`，收益上限有限
2. **未做"标量前导到对齐边界"**: 无法保证每行起点 32B 对齐
3. **自定义 allocator 开销**: `Platform::AllocateAligned` 可能引入额外开销
4. **主要瓶颈不在这里**: 当前瓶颈是带宽和指令结构，不是 cache line 跨越

**对齐优化的正确理解:**

| 说法 | 更准确的表述 |
|------|-------------|
| "loadu 不会从对齐获益" | ❌ loadu 在地址对齐时走类似内部路径，代价接近 |
| "对齐对 streaming 没帮助" | ❌ 跨 cache line 时仍有 1-3% 收益空间 |
| "现代 CPU 对齐=非对齐" | ⚠️ 仅当不跨 64B cache line 时成立 |

**对齐真正有价值的场景:**
- 32B 向量经常跨 64B cache line 边界（起始地址随机偏移时）
- 同一循环多数组读写，跨线放大 L1D/L2 端口压力
- 大量小数组临时缓冲，分配器行首对齐差

**正确的做法（如果要优化）:**
1. 用 32B 对齐分配缓冲区
2. 内层循环用"标量前导到对齐边界，再用 aligned load"
3. 用 `_mm256_load_ps` 替代 `_mm256_loadu_ps`

**结论:**
对齐优化**不是错误方向**，但对当前 streaming 带宽型工作负载**收益很小**（预期 1-3%）。
优先级降为 **P3**，排在 rcp 替代 div、边界分离、减少中间读写之后。
当 profiling 显示 L1D 端口或跨线 load 有明显 stall 时再考虑。

---

## 成功的优化（可参考）

### ✅ 消除 Copy 阶段 + OpenMP chunk size 优化 (2025-01-19)

**优化内容:**
1. 创建 `FusedSobelGradMagBinDirect` - 直接写入 QImage（带 stride），消除中间 buffer 和 Copy 阶段
2. 添加 `schedule(static, 64)` 减少 OpenMP 调度开销

**优化前后对比 (1 线程):**

| 测试集 | 优化前 | 优化后 | 提升 |
|--------|--------|--------|------|
| Small Images | 9.3 ms | 7.6 ms | -18% |
| Large Images | 198.7 ms | 147.4 ms | **-26%** |
| Medium | 31.1 ms | 28.9 ms | -7% |
| Rotated | 33.0 ms | 35.3 ms | +7% |

**关键发现:**
- Large 图像提升最显著（内存操作减少效果明显）
- 所有测试 100% 精度保持
- 4 线程仍比 1 线程慢（内存带宽瓶颈）

**技术细节:**
- 每层 BuildLevelFused 从 2 个 parallel for 减少到 1 个
- 消除了 5 个临时 buffer（gxBuffer, gyBuffer, magBuffer, binBuffer, dirBuffer）的分配和拷贝
- schedule(static, 64) 增大 chunk 减少调度开销

---

### ✅ 融合 ToFloat + Copy 阶段 (2026-01-19)

**优化内容:**
将 AnglePyramid::Build 中的两步操作融合为一步：
1. 原流程: uint8 → float QImage (有 stride) → 连续 float vector (Copy)
2. 新流程: uint8 → 连续 float vector (直接)

**优化前后对比:**

| 测试集 | 优化前 | 优化后 | 提升 |
|--------|--------|--------|------|
| Small Images (640x512) | 6.8 ms | 5.8 ms | **-14.7%** |
| Large Images (2048x4001) | 162.8 ms | 133.0 ms | **-18.3%** |
| Copy 阶段占比 | 3-18% | **0%** | 完全消除 |

**关键发现:**
- 消除了中间 float QImage 分配（对于 2048x4001 图像约 32MB）
- 减少一次完整的内存遍历
- Large 图像提升更显著（内存带宽敏感）
- 所有测试 100% 精度保持

**技术细节:**
- 直接将 uint8 源数据转换到 `std::vector<float>`
- AVX2 优化: 8 个 uint8 → 8 个 float 一次完成
- 输出直接写入连续内存（无 stride）

---

### ✅ rcp_ps + Newton-Raphson 替代 div_ps (2026-01-19)

**优化内容:**
将 `fast_quantize_bin_avx2` 和 `atan2_avx2` 中的 `_mm256_div_ps` 替换为 `_mm256_rcp_ps` + Newton-Raphson 迭代。

**技术原理:**
```cpp
// 快速倒数: rcp_ps 约 12 位精度，Newton-Raphson 迭代提升到 ~23 位
__m256 rcp = _mm256_rcp_ps(x);
rcp = rcp * (2 - x * rcp);  // Newton-Raphson

// 快速除法: a / b = a * rcp(b)
__m256 result = _mm256_mul_ps(a, rcp);
```

**优化前后对比:**

| 测试集 | 优化前 | 优化后 | 提升 |
|--------|--------|--------|------|
| Small Images (640x512) | 7.2 ms | 6.3 ms | **-12.5%** |
| Large Images (2048x4001) | ~147 ms | 144.4 ms | -1.8% |
| Image3 (1280x1024) | 33.1 ms | 32.2 ms | -2.7% |
| Image4 (Rotated) | 35.1 ms | 37.0 ms | +5.4% |

**关键发现:**
- Small 图像提升最显著（除法在总计算中占比较高）
- Large 图像提升较小（内存带宽仍是主要瓶颈）
- 所有测试 100% 精度保持
- rcp + NR 比 div 快约 2-4x，但实际提升取决于除法在总时间中的占比

**修改的函数:**
- `fast_quantize_bin_avx2()` - 计算 t = min/max 比值
- `atan2_avx2()` - 计算 t = num/den 比值
