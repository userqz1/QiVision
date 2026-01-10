# ShapeModel 性能优化记录

> 最后更新: 2025-01-10
> 测试环境: 小图 640×512, 大图 2048×4001, 4 金字塔层, 360° 旋转搜索

## 详细性能分析 (2025-01-10)

### 测试配置
- **小图**: 640×512, ROI 210×70, ~446 模型点
- **大图**: 2048×4001, ROI 135×200, ~540 模型点
- **搜索参数**: minScore=0.5, greediness=0.9, 360° 旋转, LeastSquares 亚像素

### Create() 时间分解

| 步骤 | 小图 (640×512) | 大图 (2048×4001) | 说明 |
|------|---------------|-----------------|------|
| **总计** | **48.54ms** | **217.74ms** | |
| PyramidBuild | 41.32ms (85.1%) | 206.97ms (95.1%) | **主要耗时** |
| ContrastAuto | 0.98ms (2.0%) | 3.30ms (1.5%) | Otsu + 百分位数 |
| ExtractPoints | 5.89ms (12.1%) | 7.18ms (3.3%) | 边缘点提取 |
| Optimize | 0.33ms (0.7%) | 0.26ms (0.1%) | 点数优化 |
| BuildSoA | 0.01ms | 0.01ms | SIMD 数据结构 |

### Find() 时间分解

| 步骤 | 小图 (640×512) | 大图 (2048×4001) | 说明 |
|------|---------------|-----------------|------|
| **总计** | **74.39ms** | **535.98ms** | |
| **PyramidBuild** | **64.85ms (87.2%)** | **495.37ms (92.4%)** | **最大瓶颈！** |
| CoarseSearch | 8.87ms (11.9%) | 38.52ms (7.2%) | 顶层粗搜索 |
| PyramidRefine | 0.40ms (0.5%) | 1.02ms (0.2%) | 金字塔精细化 |
| SubpixelRefine | 0.27ms (0.4%) | 1.07ms (0.2%) | LeastSquares |
| NMS | <0.01ms | <0.01ms | 非极大抑制 |

### AnglePyramid::Build 细节 (Find 内部)

| 步骤 | 小图 | 大图 | 说明 |
|------|------|------|------|
| **总计** | **88.71ms** | **592.43ms** | |
| **GaussPyramid** | 13.76ms (15.5%) | **328.50ms (55.4%)** | **大图最大瓶颈** |
| ExtractEdge | 22.62ms (25.5%) | 96.53ms (16.3%) | 边缘点提取 |
| Sobel | 9.16ms (10.3%) | 35.34ms (6.0%) | 梯度计算 |
| Sqrt(mag) | 9.93ms (11.2%) | 14.27ms (2.4%) | 幅值计算 |
| Atan2(dir) | 9.28ms (10.5%) | 17.76ms (3.0%) | 方向计算 |
| Quantize | 9.07ms (10.2%) | 18.01ms (3.0%) | 角度量化 |
| ToFloat | 1.75ms (2.0%) | 8.42ms (1.4%) | 类型转换 |
| Copy | 10.41ms (11.7%) | 19.12ms (3.2%) | 内存拷贝 |

### 关键发现

1. **AnglePyramid::Build 占 Find() 总时间的 87-92%**
   - 搜索算法本身 (CoarseSearch + PyramidRefine) 只占 7-12%
   - 搜索算法优化空间有限

2. **GaussPyramid 是大图的最大瓶颈 (55.4%)**
   - 高斯模糊 + 下采样占用大量时间
   - 小图比例较低 (15.5%)，但大图占比显著

3. **ExtractEdge 是小图的主要耗时 (25.5%)**
   - 边缘点收集和筛选

### 优化建议

| 优先级 | 方向 | 预期收益 | 说明 |
|--------|------|----------|------|
| **高** | GaussPyramid SIMD 优化 | 2-3× | 高斯卷积向量化 |
| **高** | 下采样优化 | 1.5-2× | 使用 SIMD 或更快算法 |
| 中 | ExtractEdge 优化 | 1.3-1.5× | 减少内存分配 |
| 低 | CoarseSearch 优化 | <1.2× | 已经很快 |

---

## 基准性能 (旧数据，仅供参考)

| 图像 | 搜索时间 | 匹配结果 |
|------|----------|----------|
| Image 1 (模板位置) | 66 ms | score=1.0 |
| Image 2 | 137 ms | score=0.98 |
| Image 3 | 219 ms | score=0.98 |
| Image 4 | 306 ms | score=0.98 |
| Image 5 | 399 ms | score=0.98 |

**当前实现**: `SearchPyramid` + OpenMP 并行化 + 双线性插值 + **早期终止 (greediness)**

**关键性能因素**:
- 早期终止: 坏位置只检查 ~10-20 点 (vs 全部 450 点)
- OpenMP: 按角度并行化
- 双线性插值: 精度优先，性能可接受

---

## 已测试的优化方案

### 方案 1: 预计算旋转 + AVX2 Gather

**思路**: 预计算所有 720 个旋转角度的模型偏移，用 AVX2 gather 指令并行加载梯度

**实现**:
```cpp
struct PrecomputedRotation {
    float angle, cosA, sinA;
    std::vector<int16_t> offsetX, offsetY;
    std::vector<float> rotatedCos, rotatedSin;
    float minX, maxX, minY, maxY;
};

// AVX2 gather 加载梯度
__m256 vGx = _mm256_mask_i32gather_ps(..., gxData, vIdx, vInBounds, 4);
__m256 vGy = _mm256_mask_i32gather_ps(..., gyData, vIdx, vInBounds, 4);
```

**结果**: ❌ **2× 更慢**

**原因**:
1. 预计算 720 个旋转的开销大，但每个旋转只用一次
2. AVX2 gather 对分散内存访问效率低（cache miss 严重）
3. gather 指令本身延迟高（~20 cycles）

---

### 方案 2: ResponseMap (O(1) 查表)

**思路**: 使用 LINE-MOD 风格的响应图，将梯度方向量化为 8 个 bin，O(1) 查表代替双线性插值

**实现**:
```cpp
// 构建响应图: 8 bins × W × H
ResponseMap responseMap;
responseMap.Build(targetPyramid, spreadRadius=2);

// O(1) 评分
double score = responseMap.ComputeScore(rotatedModel, level, x, y);
```

**结果**: ❌ **2.5× 更慢** (238-1121 ms)

**原因**:
1. ResponseMap 构建开销: 8 × W × H bytes per level
2. 粗搜索节省的时间被构建开销抵消
3. 精细化阶段仍需双线性插值（占主要时间）
4. 对于 640×512 图像，顶层图像已经很小 (~80×64)，O(1) 优势不明显

---

### 方案 3: ResponseMap + OpenMP

**思路**: 在 ResponseMap 基础上添加 OpenMP 并行化

**结果**: ❌ **2× 更慢** (237-1121 ms)

**原因**: 同方案 2，构建开销仍是瓶颈

---

### 方案 4: 增大粗搜索步长

**思路**: 将粗搜索步长从 2 改为 4

**结果**: ❌ **漏检**

**原因**: 步长过大导致正确位置被跳过

---

### 方案 5: 降低覆盖率阈值

**思路**: 将覆盖率阈值从 70% 降低到 50%

**结果**: ❌ **漏检 + 误检增多**

---

### 方案 6: 标量加载 + SIMD 批量计算 (ComputeScoreAtPositionFastSIMD)

**思路**: 用标量方式加载梯度到对齐缓冲区，然后用 AVX2 批量计算相似度

**实现**:
```cpp
// 标量加载梯度
alignas(32) float gxBuf[8], gyBuf[8];
for (int j = 0; j < 8; ++j) {
    gxBuf[j] = gxData[iy * stride + ix];
    gyBuf[j] = gyData[iy * stride + ix];
}

// AVX2 批量计算
__m256 vGx = _mm256_load_ps(gxBuf);
__m256 vGy = _mm256_load_ps(gyBuf);
__m256 vMagSq = _mm256_add_ps(_mm256_mul_ps(vGx, vGx), _mm256_mul_ps(vGy, vGy));
// ... 向量化计算相似度
```

**结果**: ❌ **6× 更慢** (532-2590 ms)

**原因**:
1. **缺少早期终止**: `ComputeScoreAtPositionFast` 不支持 greediness 早期终止
2. 原始 `ComputeScoreAtPosition` 有早期终止逻辑，大多数坏位置在检查 ~10 个点后即被拒绝
3. SIMD 版本必须检查全部 450 个点，无论位置好坏
4. 早期终止带来的 6× 加速完全抵消 SIMD 计算的收益

**关键教训**: **早期终止 > SIMD 优化**

---

### 方案 7: 标量加载 + SIMD 计算 + 周期性早期终止 ✅

**思路**: 结合方案 6 的 SIMD 计算和原版的早期终止机制

**实现**:
```cpp
// ComputeScoreBilinearAVX2:
// 1. 直接指针访问 (避免 GetGradientAt 函数调用)
// 2. 标量加载梯度到对齐缓冲区 (处理非连续内存)
// 3. SIMD 批量计算相似度 (向量化点积和累加)
// 4. 每 16 点检查一次早期终止 (避免频繁 hadd)

for (size_t i = 0; i < n8; i += 8) {
    // SIMD 坐标变换
    // 标量加载 + 双线性插值
    // SIMD 相似度计算

    // 周期性早期终止 (每 16 点)
    if (checkEarlyExit && ((i & 8) != 0)) {
        // 水平求和检查
        if ((currentScore / currentWeight) * numPoints < threshold * 0.8f) {
            return 0.0;
        }
    }
}
```

**结果**: ✅ **性能与基准持平** (68-401ms vs 66-399ms)

**优势**:
1. 保留早期终止机制 (关键!)
2. 直接指针访问减少函数调用开销
3. SIMD 加速相似度计算
4. 周期性检查 (每 16 点) 平衡了检查开销和退出速度

---

## 瓶颈分析 (2025-01-10 更新)

通过详细计时分析发现:

| 阶段 | 小图耗时占比 | 大图耗时占比 | 说明 |
|------|-------------|-------------|------|
| **AnglePyramid::Build** | **87.2%** | **92.4%** | **真正的瓶颈！** |
| └─ GaussPyramid | 15.5% | 55.4% | 高斯模糊+下采样 |
| └─ ExtractEdge | 25.5% | 16.3% | 边缘点提取 |
| └─ Sobel/Sqrt/Atan2 | ~30% | ~10% | 梯度计算 |
| 粗搜索 (顶层) | 11.9% | 7.2% | OpenMP 并行化 |
| 金字塔精细化 | 0.5% | 0.2% | 已经非常快 |
| 亚像素精化 | 0.4% | 0.2% | LeastSquares |

**重要结论**:
1. **瓶颈不在搜索算法，而在 AnglePyramid::Build** (87-92%)
2. **GaussPyramid 是大图的主要瓶颈** (55.4%)，应优先优化
3. 搜索算法 (CoarseSearch + PyramidRefine) 只占 7-12%，优化空间有限
4. 之前认为双线性插值是瓶颈的结论已过时

---

## 正确的 SIMD 优化方向

### 核心思路
- ❌ 错误: 用 gather 并行加载分散的梯度数据
- ✅ 正确: 标量加载 → SIMD 批量计算相似度

### 推荐方案 1: 批量计算相似度 (预期 2-3× 加速)

```cpp
// 标量加载 8 个点的梯度到连续缓冲区
alignas(32) float gxBuf[8], gyBuf[8];
for (int j = 0; j < 8; ++j) {
    int32_t ix = cx + (int32_t)(cosR * px - sinR * py);
    int32_t iy = cy + (int32_t)(sinR * px + cosR * py);
    gxBuf[j] = gxData[iy * stride + ix];
    gyBuf[j] = gyData[iy * stride + ix];
}

// SIMD 批量计算相似度
__m256 vGx = _mm256_loadu_ps(gxBuf);
__m256 vGy = _mm256_loadu_ps(gyBuf);
__m256 vMagSq = _mm256_add_ps(_mm256_mul_ps(vGx, vGx), _mm256_mul_ps(vGy, vGy));
// ... 后续计算
```

### 推荐方案 2: ResponseMap + SIMD 累加 (预期 5-8× 加速)

仅适用于**更大的图像** (>2K×2K)，此时 ResponseMap 构建成本可摊销

```cpp
// Response Map 将随机访问转为查表
// 然后用 SIMD 批量累加响应值
__m256i vResponses = _mm256_loadu_si256((__m256i*)responses);
vTotalScore = _mm256_add_epi16(vTotalScore, vResponses);
```

---

## 结论与建议 (2025-01-10 更新)

### 当前状态
- **小图 (640×512)**: Create 48ms, Find 74ms
- **大图 (2048×4001)**: Create 218ms, Find 536ms
- OpenMP 并行化已实现
- 代码已清理，移除无效优化代码

### 未来优化方向 (基于新的瓶颈分析)

| 优先级 | 方案 | 预期收益 | 说明 |
|--------|------|----------|------|
| **最高** | **GaussPyramid SIMD 优化** | **2-3×** | 大图瓶颈 (55.4%)，高斯卷积向量化 |
| **高** | **下采样算法优化** | **1.5-2×** | 使用更快的下采样方法 |
| 中 | ExtractEdge 优化 | 1.3-1.5× | 减少内存分配，小图瓶颈 (25.5%) |
| 中 | Sobel SIMD 优化 | 1.2-1.5× | 梯度计算向量化 |
| 低 | 搜索算法优化 | <1.2× | 已经只占 7-12% |

### 不推荐的方案
- ❌ AVX2 gather (延迟高，cache miss)
- ❌ 预计算所有旋转 (一次性使用，开销大)
- ❌ 增大搜索步长 (漏检风险)
- ❌ **无早期终止的 SIMD** (必须检查所有点，6× 更慢)
- ❌ **继续优化搜索算法** (已经只占 7-12%，ROI 有限)

### 最重要的教训

1. **早期终止 (greediness) 比 SIMD 优化更关键！**
   - 当前的 `ComputeScoreAtPosition` 通过早期终止，对于大多数坏位置只检查 ~10-20 个点
   - 任何 SIMD 优化方案都必须保留早期终止能力

2. **瓶颈已转移到 AnglePyramid::Build** (2025-01-10 发现)
   - 搜索算法优化收益有限 (<12%)
   - **GaussPyramid 是大图真正的瓶颈** (55.4%)
   - 优化重点应转移到图像金字塔构建

---

## 相关代码位置

- 主搜索: `src/Matching/ShapeModel.cpp` → `SearchPyramid()`
- 评分函数: `src/Matching/ShapeModel.cpp` → `ComputeScoreAtPosition()`
- ResponseMap: `src/Internal/ResponseMap.cpp` (已实现，默认禁用)
- ResponseMap 搜索: `SearchPyramidWithResponseMap()` (已实现，默认禁用)
- **计时结构**: `include/QiVision/Matching/ShapeModel.h` → `ShapeModelCreateTiming`, `ShapeModelFindTiming`
- **AnglePyramid 计时**: `include/QiVision/Internal/AnglePyramid.h` → `AnglePyramidTiming`
- **性能测试**: `samples/09_performance_benchmark.cpp`
