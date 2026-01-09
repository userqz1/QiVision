# ShapeModel 性能优化记录

> 记录日期: 2025-01-09
> 测试环境: 640×512 图像, 450 模型点, 3 金字塔层, 360° 旋转搜索

## 基准性能

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

## 瓶颈分析

通过 profiling 发现:

| 阶段 | 耗时占比 | 说明 |
|------|----------|------|
| 粗搜索 (顶层) | ~15% | 图像小，计算量有限 |
| **金字塔精细化** | **~70%** | 双线性插值是瓶颈 |
| 亚像素精化 | ~15% | 最后的精确定位 |

**结论**: 瓶颈不在粗搜索，而在金字塔精细化阶段的**双线性插值**

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

## 结论与建议

### 当前状态
- 86-407 ms 对于工业应用已经可以接受
- OpenMP 并行化已实现
- 代码已清理，移除无效优化代码

### 未来优化方向

| 优先级 | 方案 | 预期收益 | 实现难度 |
|--------|------|----------|----------|
| 高 | 方案1: 标量加载 + SIMD 计算 | 2-3× | 中 |
| 中 | 两阶段角度搜索 (粗6° + 细1°) | 1.5-2× | 低 |
| 低 | 方案2: ResponseMap + SIMD | 5-8× | 高 (仅大图有效) |

### 不推荐的方案
- ❌ AVX2 gather (延迟高，cache miss)
- ❌ 预计算所有旋转 (一次性使用，开销大)
- ❌ 增大搜索步长 (漏检风险)
- ❌ **无早期终止的 SIMD** (必须检查所有点，6× 更慢)

### 最重要的教训
**早期终止 (greediness) 比 SIMD 优化更关键！**

当前的 `ComputeScoreAtPosition` 通过早期终止，对于大多数坏位置只检查 ~10-20 个点而非全部 450 个点。这带来约 20-40× 的加速，远超任何 SIMD 优化能提供的 2-8× 加速。

任何 SIMD 优化方案都必须保留早期终止能力，否则会适得其反。

---

## 相关代码位置

- 主搜索: `src/Matching/ShapeModel.cpp` → `SearchPyramid()`
- 评分函数: `src/Matching/ShapeModel.cpp` → `ComputeScoreAtPosition()`
- ResponseMap: `src/Internal/ResponseMap.cpp` (已实现，默认禁用)
- ResponseMap 搜索: `SearchPyramidWithResponseMap()` (已实现，默认禁用)
