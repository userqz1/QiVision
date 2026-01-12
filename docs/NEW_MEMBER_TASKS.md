# 新成员任务清单

分支: `feature/io-and-utils`

## 负责模块

### 1. Platform/FileIO (文件IO)

**位置**: `src/Platform/FileIO.cpp`, `include/QiVision/Platform/FileIO.h`

**任务**:
- [ ] 实现 `ReadImage()` - 读取图像文件 (BMP, PNG, JPG)
- [ ] 实现 `WriteImage()` - 写入图像文件
- [ ] 实现 `ReadBinary()` - 读取二进制文件
- [ ] 实现 `WriteBinary()` - 写入二进制文件
- [ ] 文件路径处理工具函数

**参考**: 使用 `stb_image.h` 和 `stb_image_write.h`

### 2. Platform/Timer (计时器)

**位置**: `src/Platform/Timer.cpp`, `include/QiVision/Platform/Timer.h`

**任务**:
- [ ] 实现高精度计时器类
- [ ] 支持 Start/Stop/Reset/ElapsedMs
- [ ] 跨平台支持 (Windows/Linux)

### 3. Core/Draw (绘图工具)

**位置**: `src/Core/Draw.cpp`, `include/QiVision/Core/Draw.h`

**任务**:
- [ ] 实现 `DrawLine()` - 画直线
- [ ] 实现 `DrawCircle()` - 画圆
- [ ] 实现 `DrawEllipse()` - 画椭圆
- [ ] 实现 `DrawPolygon()` - 画多边形
- [ ] 实现 `DrawText()` - 画文字 (可选)
- [ ] 实现 `DrawContour()` - 画轮廓
- [ ] 实现 `DrawRegion()` - 画区域

### 4. 简单工具函数

**位置**: `src/Core/Utils.cpp`

**任务**:
- [ ] 颜色空间转换 (RGB <-> Gray, RGB <-> HSV)
- [ ] 图像格式转换
- [ ] 简单图像操作 (翻转、旋转90度)

## 代码规范

1. 遵循 `docs/API_Naming_Convention.md` 命名规范
2. 遵循 `.claude/CLAUDE.md` 开发规范
3. 每个函数需要写单元测试
4. 代码需要有注释

## 提交规范

```
<type>(<scope>): <subject>

type: feat, fix, refactor, test, docs
scope: Platform, Core, etc.

示例:
feat(Platform): implement ReadImage with stb_image
test(Platform): add unit tests for FileIO
```

## 联系方式

有问题请在 GitHub Issue 中讨论。
