# QiVision API 重命名映射表

本文档记录 API 重命名的完整映射，用于追踪旧接口到新接口的对应关系。

## 命名规范

参考 Halcon API 命名风格：`<动作><修饰词><模块名称>`

## 1. Matching 模块

### 1.1 ShapeModel 类

| 原函数名 | 新函数名 | 说明 | 状态 |
|----------|----------|------|------|
| `ShapeModel::Create(image, params)` | `CreateShapeModel()` 或 `ShapeModel::CreateModel()` | 创建只旋转模型 | 待重命名 |
| `ShapeModel::Create(image, roi, params)` | `CreateShapeModelRoi()` 或 `ShapeModel::CreateModel()` | 创建只旋转模型(带ROI) | 待重命名 |
| `ShapeModel::CreateWithOrigin()` | `CreateShapeModelOrigin()` 或 `ShapeModel::CreateModelOrigin()` | 创建带自定义原点模型 | 待重命名 |
| `ShapeModel::Find(image, params)` | `FindShapeModel()` 或 `ShapeModel::FindModel()` | 查找只旋转匹配 | 待重命名 |
| `ShapeModel::FindBest(image, params)` | `FindShapeModelBest()` 或 `ShapeModel::FindBest()` | 查找最佳匹配 | 保留 |
| `ShapeModel::FindInROI()` | `FindShapeModelRoi()` 或 `ShapeModel::FindModelRoi()` | ROI内查找 | 待重命名 |
| `ShapeModel::Save(filename)` | `WriteShapeModel()` 或 `ShapeModel::WriteModel()` | 保存模型 | 待重命名 |
| `ShapeModel::Load(filename)` | `ReadShapeModel()` 或 `ShapeModel::ReadModel()` | 加载模型 | 待重命名 |
| `ShapeModel::GetStats()` | `GetShapeModelParams()` 或 `ShapeModel::GetParams()` | 获取模型参数 | 待重命名 |
| `ShapeModel::GetModelPoints()` | `GetShapeModelContours()` 或 `ShapeModel::GetContours()` | 获取轮廓点 | 待重命名 |
| `ShapeModel::GetModelContour()` | `GetShapeModelContour()` | 获取显示轮廓 | 保留 |
| `ShapeModel::GetMatchContour()` | `GetShapeModelMatchContour()` | 获取匹配轮廓 | 保留 |
| `ShapeModel::Clear()` | `ClearShapeModel()` 或 `ShapeModel::Clear()` | 清除模型 | 保留 |
| `ShapeModel::ComputeScore()` | `ShapeModel::ComputeScore()` | 计算匹配分数 | 保留 |
| `ShapeModel::RefineMatch()` | `ShapeModel::RefineMatch()` | 亚像素精化 | 保留 |

### 1.2 未来扩展 (Scaled 和 Aniso 模型)

| Halcon 函数 | QiVision 函数 | 说明 | 状态 |
|-------------|---------------|------|------|
| `create_scaled_shape_model` | `CreateScaledShapeModel()` | 各向同性缩放模型 | 未实现 |
| `create_aniso_shape_model` | `CreateAnisoShapeModel()` | 各向异性缩放模型 | 未实现 |
| `create_shape_model_xld` | `CreateShapeModelXld()` | 从轮廓创建 | 未实现 |
| `find_scaled_shape_model` | `FindScaledShapeModel()` | 查找带缩放 | 未实现 |
| `find_aniso_shape_model` | `FindAnisoShapeModel()` | 查找各向异性 | 未实现 |
| `find_shape_models` | `FindShapeModels()` | 多模型查找 | 未实现 |
| `set_shape_model_origin` | `SetShapeModelOrigin()` | 设置模型原点 | 未实现 |
| `determine_shape_model_params` | `DetermineShapeModelParams()` | 自动参数推荐 | 未实现 |

## 2. Measure 模块

### 2.1 Caliper 函数 (已符合规范)

| 当前函数名 | 对应 Halcon 函数 | 说明 | 状态 |
|------------|------------------|------|------|
| `MeasurePos()` | `measure_pos` | 测量边缘位置 | ✅ 已符合 |
| `MeasurePairs()` | `measure_pairs` | 测量边缘对 | ✅ 已符合 |
| `FuzzyMeasurePos()` | `fuzzy_measure_pos` | 模糊边缘测量 | ✅ 已符合 |
| `FuzzyMeasurePairs()` | `fuzzy_measure_pairs` | 模糊边缘对测量 | ✅ 已符合 |
| `ExtractMeasureProfile()` | - | 提取测量剖面 | ✅ 已符合 |
| `SelectEdges()` | `select_edge` | 筛选边缘 | ✅ 已符合 |
| `SortEdges()` | - | 排序边缘 | ✅ 已符合 |

### 2.2 MeasureHandle 函数 (已符合规范)

| 当前函数名 | 对应 Halcon 函数 | 说明 | 状态 |
|------------|------------------|------|------|
| `CreateMeasureRect()` | `gen_measure_rectangle2` | 创建矩形测量句柄 | ✅ 已符合 |
| `CreateMeasureArc()` | `gen_measure_arc` | 创建圆弧测量句柄 | ✅ 已符合 |
| `CreateMeasureConcentricCircles()` | - | 创建同心圆测量句柄 | ✅ 已符合 |

## 3. Core 模块

### 3.1 QImage 类

| 当前方法 | 说明 | 状态 |
|----------|------|------|
| `QImage::FromFile()` | 从文件创建 | ✅ 保留 |
| `QImage::FromData()` | 从数据创建 | ✅ 保留 |
| `QImage::Clone()` | 深拷贝 | ✅ 保留 |
| `QImage::SubImage()` | 子图像 | ✅ 保留 |
| `QImage::SaveToFile()` | 保存到文件 | ✅ 保留 |
| `QImage::ConvertTo()` | 类型转换 | ✅ 保留 |
| `QImage::ToGray()` | 转灰度 | ✅ 保留 |
| `QImage::SetDomain()` | 设置Domain | ✅ 保留 |
| `QImage::GetDomain()` | 获取Domain | ✅ 保留 |
| `QImage::ReduceDomain()` | 缩小Domain | ✅ 保留 |
| `QImage::ResetDomain()` | 重置Domain | ✅ 保留 |

### 3.2 QRegion 类

| 当前方法 | 说明 | 状态 |
|----------|------|------|
| `QRegion::Rectangle()` | 创建矩形区域 | ✅ 保留 |
| `QRegion::Circle()` | 创建圆形区域 | ✅ 保留 |
| `QRegion::Ellipse()` | 创建椭圆区域 | ✅ 保留 |
| `QRegion::Union()` | 区域并集 | ✅ 保留 |
| `QRegion::Intersection()` | 区域交集 | ✅ 保留 |
| `QRegion::Difference()` | 区域差集 | ✅ 保留 |
| `QRegion::Complement()` | 区域补集 | ✅ 保留 |
| `QRegion::Dilate()` | 膨胀 | ✅ 保留 |
| `QRegion::Erode()` | 腐蚀 | ✅ 保留 |
| `QRegion::Opening()` | 开运算 | ✅ 保留 |
| `QRegion::Closing()` | 闭运算 | ✅ 保留 |

### 3.3 QContour 类

| 当前方法 | 说明 | 状态 |
|----------|------|------|
| 待检查 | - | - |

## 4. Platform 模块

### 4.1 FileIO (待实现)

| 计划函数名 | 对应 Halcon 函数 | 说明 | 状态 |
|------------|------------------|------|------|
| `ReadImage()` | `read_image` | 读取图像 | 待实现 |
| `WriteImage()` | `write_image` | 写入图像 | 待实现 |
| `ReadBinary()` | - | 读取二进制 | 待实现 |
| `WriteBinary()` | - | 写入二进制 | 待实现 |

### 4.2 Timer

| 当前方法 | 说明 | 状态 |
|----------|------|------|
| 待检查 | - | - |

## 5. Internal 模块 (内部使用，不导出)

Internal 模块的函数命名不需要遵循 Halcon 风格，保持当前命名即可。

| 模块 | 说明 | 状态 |
|------|------|------|
| `AnglePyramid` | 角度金字塔 | ✅ 内部使用 |
| `Gradient` | 梯度计算 | ✅ 内部使用 |
| `Pyramid` | 图像金字塔 | ✅ 内部使用 |
| `Interpolate` | 插值 | ✅ 内部使用 |
| `Fitting` | 拟合 | ✅ 内部使用 |
| `Edge1D` | 1D边缘 | ✅ 内部使用 |
| `Steger` | Steger算法 | ✅ 内部使用 |
| `Hessian` | Hessian算法 | ✅ 内部使用 |

## 重命名策略

### 推荐方案：类内方法重命名

保持类结构，重命名类内方法：

```cpp
// 当前
class ShapeModel {
    bool Create(const QImage& image, const ModelParams& params);
    std::vector<MatchResult> Find(const QImage& image, const SearchParams& params);
    bool Save(const std::string& filename);
    bool Load(const std::string& filename);
};

// 重命名后
class ShapeModel {
    bool CreateModel(const QImage& image, const ModelParams& params);
    std::vector<MatchResult> FindModel(const QImage& image, const SearchParams& params);
    bool WriteModel(const std::string& filename);
    bool ReadModel(const std::string& filename);
    ModelStats GetParams() const;
    std::vector<ModelPoint> GetContours(int32_t level = 0) const;
};
```

### 备选方案：全局函数

提供 Halcon 风格的全局函数包装：

```cpp
// 全局函数包装
bool CreateShapeModel(ShapeModel& model, const QImage& image, const ModelParams& params);
std::vector<MatchResult> FindShapeModel(const ShapeModel& model, const QImage& image, const SearchParams& params);
bool WriteShapeModel(const ShapeModel& model, const std::string& filename);
bool ReadShapeModel(ShapeModel& model, const std::string& filename);
```

## 变更日志

### 2025-01-12
- 创建 API 重命名映射文档
- 分析所有公共 API 并确定重命名策略
- 确认 Measure 模块已符合命名规范
- ShapeModel 模块需要重命名
