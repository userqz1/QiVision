# QiVision API 命名规范

参考 Halcon API 命名风格，统一项目命名规范。

## 1. 命名格式

```
<动作><修饰词><模块名称>
```

## 2. 动作词列表

| 动作 | 含义 | 示例 |
|------|------|------|
| Create | 创建模型 | `CreateShapeModel()`, `CreateScaledShapeModel()` |
| Find | 查找/匹配 | `FindShapeModel()`, `FindScaledShapeModel()` |
| Read | 从文件读取 | `ReadShapeModel()` |
| Write | 写入文件 | `WriteShapeModel()` |
| Get | 获取属性 | `GetShapeModelParams()`, `GetShapeModelContours()` |
| Set | 设置属性 | `SetShapeModelOrigin()` |
| Clear | 清除/释放 | `ClearShapeModel()` |
| Determine | 自动确定参数 | `DetermineShapeModelParams()` |

## 3. ShapeModel 模块

| Halcon | QiVision | 说明 |
|--------|----------|------|
| `create_shape_model` | `CreateShapeModel()` | 只带旋转 |
| `create_scaled_shape_model` | `CreateScaledShapeModel()` | 各向同性缩放 |
| `create_aniso_shape_model` | `CreateAnisoShapeModel()` | 各向异性缩放 |
| `create_shape_model_xld` | `CreateShapeModelXld()` | 从轮廓创建 |
| `find_shape_model` | `FindShapeModel()` | 查找（只旋转） |
| `find_scaled_shape_model` | `FindScaledShapeModel()` | 查找（带缩放） |
| `find_aniso_shape_model` | `FindAnisoShapeModel()` | 查找（各向异性） |
| `find_shape_models` | `FindShapeModels()` | 多模型查找 |
| `read_shape_model` | `ReadShapeModel()` | 从文件读取 |
| `write_shape_model` | `WriteShapeModel()` | 写入文件 |
| `get_shape_model_params` | `GetShapeModelParams()` | 获取模型参数 |
| `get_shape_model_contours` | `GetShapeModelContours()` | 获取轮廓 |
| `set_shape_model_origin` | `SetShapeModelOrigin()` | 设置原点 |
| `clear_shape_model` | `ClearShapeModel()` | 释放模型 |

## 4. 其他模块

| 模块 | Create | Find/Apply | 说明 |
|------|--------|------------|------|
| NCC模板 | `CreateNccModel()` | `FindNccModel()` | 灰度相关匹配 |
| 卡尺 | `CreateMetrologyModel()` | `ApplyMetrologyModel()` | 测量 |
| 标定 | `CreateCalibData()` | `CalibrateCamera()` | 相机标定 |
| Blob | - | `Connection()`, `SelectShape()` | 区域分析 |

## 5. 当前代码待重命名列表

| 当前 | 应改为 |
|------|--------|
| `ShapeModel::Create()` | `CreateShapeModel()` 或类内 `CreateModel()` |
| `ShapeModel::Find()` | `FindShapeModel()` 或类内 `FindModel()` |
| `ShapeModel::Save()` | `WriteShapeModel()` 或类内 `WriteModel()` |
| `ShapeModel::Load()` | `ReadShapeModel()` 或类内 `ReadModel()` |
| `ShapeModel::GetStats()` | `GetShapeModelParams()` |
