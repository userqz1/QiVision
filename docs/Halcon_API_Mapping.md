# Halcon API 对照表

> 整理 Halcon 核心算子与 QiVision 的对应关系
> 状态: ✅ 已实现 | 🟡 部分实现 | ⬜ 未实现

---

## 1. Region 区域操作

### 1.1 阈值分割

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `threshold` | Image, MinGray, MaxGray → Region | `ThresholdToRegion()` | ✅ |
| `binary_threshold` | Image, Method('max_separability') → Region, Threshold | `ComputeOtsuThreshold()` | ✅ |
| `dyn_threshold` | Image, ThresholdImage, Offset, LightDark → Region | `DynamicThreshold()` | ✅ |
| `auto_threshold` | Image, Sigma → Region | | ⬜ |
| `hysteresis_threshold` | Image, Low, High, MaxLength → Region | | ⬜ |
| `local_threshold` | Image, Method, LightDark, GenParamName, GenParamValue → Region | | ⬜ |
| `var_threshold` | Image, MaskWidth, MaskHeight, StdDevScale, LightDark → Region | | ⬜ |
| `char_threshold` | Image, HistoRegion, Sigma, Percent → Region, Threshold | | ⬜ |
| `dual_threshold` | Image, MinSize, MinGray, MaxGray → RegionCrossing | | ⬜ |

### 1.2 区域创建

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `gen_rectangle1` | Row1, Col1, Row2, Col2 → Region | `QRegion::Rectangle()` | ✅ |
| `gen_rectangle2` | Row, Col, Phi, Length1, Length2 → Region | | ⬜ |
| `gen_circle` | Row, Col, Radius → Region | `QRegion::Circle()` | ✅ |
| `gen_ellipse` | Row, Col, Phi, Radius1, Radius2 → Region | `QRegion::Ellipse()` | 🟡 |
| `gen_region_polygon` | Rows, Cols → Region | | ⬜ |
| `gen_region_polygon_filled` | Rows, Cols → Region | | ⬜ |
| `gen_region_points` | Rows, Cols → Region | `QRegion` from points | ⬜ |
| `gen_region_runs` | Row, ColBegin, ColEnd → Region | `QRegion(runs)` | ✅ |
| `gen_region_line` | Row1, Col1, Row2, Col2 → Region | | ⬜ |

### 1.3 区域集合运算

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `union1` | Regions → RegionUnion | `QRegion::Union()` | ✅ |
| `union2` | Region1, Region2 → RegionUnion | `QRegion::Union()` | ✅ |
| `intersection` | Region1, Region2 → RegionIntersection | `QRegion::Intersection()` | ✅ |
| `difference` | Region1, Region2 → RegionDifference | `QRegion::Difference()` | ✅ |
| `complement` | Region → RegionComplement | `QRegion::Complement()` | ✅ |
| `symm_difference` | Region1, Region2 → RegionDiff | `QRegion::SymmetricDifference()` | ✅ |

### 1.4 区域变换

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `move_region` | Region, Row, Col → RegionMoved | `QRegion::Translate()` | ✅ |
| `zoom_region` | Region, ScaleWidth, ScaleHeight → RegionZoomed | `QRegion::Scale()` | ⬜ |
| `mirror_region` | Region, Mode, RowCol → RegionMirrored | | ⬜ |
| `affine_trans_region` | Region, HomMat2D, Interpolation → RegionTrans | | ⬜ |
| `transpose_region` | Region, Row, Col → RegionTransposed | | ⬜ |

### 1.5 区域特征

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `area_center` | Region → Area, Row, Col | `QRegion::Area()`, `Centroid()` | ✅ |
| `smallest_rectangle1` | Region → Row1, Col1, Row2, Col2 | `QRegion::BoundingBox()` | ✅ |
| `smallest_rectangle2` | Region → Row, Col, Phi, Length1, Length2 | `SmallestRotatedRect()` | ✅ |
| `smallest_circle` | Region → Row, Col, Radius | `SmallestEnclosingCircle()` | ✅ |
| `elliptic_axis` | Region → Ra, Rb, Phi | `FitEllipse()` | ✅ |
| `circularity` | Region → Circularity | `RegionCircularity()` | ✅ |
| `compactness` | Region → Compactness | `RegionCompactness()` | ✅ |
| `convexity` | Region → Convexity | `RegionConvexity()` | ✅ |
| `eccentricity` | Region → Anisometry, Bulkiness, StructureFactor | | ⬜ |
| `moments_region_2nd` | Region → M11, M20, M02, Ia, Ib | `RegionMoments()` | ✅ |
| `moments_region_central` | Region → ... | | ⬜ |
| `region_features` | Region, Features → Value | `RegionFeatures.h` | 🟡 |
| `select_shape` | Regions, Features, Operation, Min, Max → SelectedRegions | `SelectRegionsByFeature()` | ✅ |
| `select_shape_std` | Regions, Shape, Percent → SelectedRegions | | ⬜ |

---

## 2. Connection 连通域分析

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `connection` | Region → ConnectedRegions | `LabelConnectedComponents()` + `ExtractComponents()` | ✅ |
| `count_obj` | Objects → Number | `numLabels` 返回值 | ✅ |
| `select_obj` | Objects, Index → ObjectSelected | `ExtractComponent()` | ✅ |
| `partition_dynamic` | Region, Distance, Percent → Partitioned | | ⬜ |
| `partition_rectangle` | Region, Width, Height → Partitioned | | ⬜ |
| `expand_region` | Region, ForbiddenArea, Iterations, Mode → RegionExpanded | | ⬜ |

---

## 3. Morphology 形态学

### 3.1 二值形态学

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `dilation_circle` | Region, Radius → RegionDilation | `DilateCircle()` | ✅ |
| `dilation_rectangle1` | Region, Width, Height → RegionDilation | `DilateRect()` | ✅ |
| `dilation1` | Region, StructElement, Iterations → RegionDilation | `Dilate()` | ✅ |
| `dilation2` | Region1, Region2, Iterations → RegionDilation | | ⬜ |
| `erosion_circle` | Region, Radius → RegionErosion | `ErodeCircle()` | ✅ |
| `erosion_rectangle1` | Region, Width, Height → RegionErosion | `ErodeRect()` | ✅ |
| `erosion1` | Region, StructElement, Iterations → RegionErosion | `Erode()` | ✅ |
| `erosion2` | Region1, Region2, Iterations → RegionErosion | | ⬜ |
| `opening_circle` | Region, Radius → RegionOpening | `OpeningCircle()` | ✅ |
| `opening_rectangle1` | Region, Width, Height → RegionOpening | `OpeningRect()` | ✅ |
| `opening` | Region, StructElement → RegionOpening | `Opening()` | ✅ |
| `closing_circle` | Region, Radius → RegionClosing | `ClosingCircle()` | ✅ |
| `closing_rectangle1` | Region, Width, Height → RegionClosing | `ClosingRect()` | ✅ |
| `closing` | Region, StructElement → RegionClosing | `Closing()` | ✅ |
| `boundary` | Region, BoundaryType → RegionBoundary | `Boundary()` | ✅ |
| `skeleton` | Region → Skeleton | `Skeleton()` | ✅ |
| `thinning` | Region, StructElement1, StructElement2, Iterations → RegionThin | `Thinning()` | ✅ |
| `thickening` | Region, StructElement1, StructElement2, Iterations → RegionThick | | ⬜ |
| `hit_or_miss` | Region, StructElement1, StructElement2 → RegionHitMiss | `HitOrMiss()` | ✅ |
| `pruning` | Region, Length → RegionPruned | | ⬜ |
| `fill_up` | Region → RegionFilled | `FillHoles()` | ✅ |
| `fill_up_shape` | Region, Feature, Min, Max → RegionFilled | | ⬜ |

### 3.2 灰度形态学

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `gray_dilation_rect` | Image, MaskHeight, MaskWidth → ImageDilated | `GrayDilateRect()` | ✅ |
| `gray_erosion_rect` | Image, MaskHeight, MaskWidth → ImageEroded | `GrayErodeRect()` | ✅ |
| `gray_dilation_shape` | Image, MaskShape, MaskHeight, MaskWidth → ImageDilated | `GrayDilate()` | ✅ |
| `gray_erosion_shape` | Image, MaskShape, MaskHeight, MaskWidth → ImageEroded | `GrayErode()` | ✅ |
| `gray_opening_rect` | Image, MaskHeight, MaskWidth → ImageOpened | `GrayOpeningRect()` | ✅ |
| `gray_closing_rect` | Image, MaskHeight, MaskWidth → ImageClosed | `GrayClosingRect()` | ✅ |
| `gray_opening_shape` | Image, MaskShape, MaskHeight, MaskWidth → ImageOpened | `GrayOpening()` | ✅ |
| `gray_closing_shape` | Image, MaskShape, MaskHeight, MaskWidth → ImageClosed | `GrayClosing()` | ✅ |
| `gray_tophat` | Image, MaskShape, MaskHeight, MaskWidth → ImageTopHat | `GrayTopHat()` | ✅ |
| `gray_bothat` | Image, MaskShape, MaskHeight, MaskWidth → ImageBotHat | `GrayBlackHat()` | ✅ |
| `gray_range_rect` | Image, MaskHeight, MaskWidth → ImageRange | `GrayRange()` | ✅ |
| `gray_skeleton` | Image → Skeleton | | ⬜ |

### 3.3 结构元素

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `gen_disc_se` | Type, Width, Height, Smax → SE | `StructElement::Ellipse()` | ✅ |
| `gen_rectangle_se` | Type, Width, Height → SE | `StructElement::Rectangle()` | ✅ |
| `gen_circle_se` | Type, Radius → SE | `StructElement::Circle()` | ✅ |
| `gen_rhombus_se` | Type, Radius → SE | `StructElement::Diamond()` | ✅ |
| `gen_struct_elements` | Type, ... → StructElements | `StructElement` 类 | ✅ |

---

## 4. Edge 边缘检测

### 4.1 像素级边缘

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `sobel_amp` | Image, FilterType, Size → EdgeAmplitude | `SobelAmp()` | ✅ |
| `sobel_dir` | Image, FilterType, Size → EdgeDirection | `SobelDir()` | ✅ |
| `prewitt_amp` | Image → EdgeAmplitude | | ⬜ |
| `roberts` | Image, FilterType → EdgeImage | | ⬜ |
| `laplace` | Image, ResultType, MaskSize, FilterMask → ImageLaplace | `Laplace()` | ✅ |
| `laplace_of_gauss` | Image, Sigma → ImageLoG | `LaplacianOfGaussian()` | ✅ |
| `diff_of_gauss` | Image, Sigma1, Sigma2 → ImageDoG | | ⬜ |
| `derivate_gauss` | Image, Sigma, Component → ImageDeriv | `DerivateGauss()` | ✅ |
| `edges_image` | Image, Filter, Alpha, NMS, Low, High → ImaAmp, ImaDir | | ⬜ |
| `frei_amp` | Image → EdgeAmplitude | | ⬜ |
| `kirsch_amp` | Image → EdgeAmplitude | | ⬜ |
| `highpass_image` | Image, Width, Height → ImageHighpass | `HighpassImage()` | ✅ |
| `bandpass_image` | Image, Filter → ImageBandpass | | ⬜ |

### 4.2 亚像素边缘 (XLD)

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `edges_sub_pix` | Image, Filter, Alpha, Low, High → Edges | `DetectCannyEdges()` → XLD | ✅ |
| `edges_color_sub_pix` | Image, Filter, Alpha, Low, High → Edges | | ⬜ |
| `lines_gauss` | Image, Sigma, Low, High, LightDark, ExtractWidth, LineModel, CompleteJunctions → Lines | `DetectStegerEdges()` | ✅ |
| `lines_facet` | Image, MaskSize, Low, High, LightDark → Lines | | ⬜ |
| `lines_color` | Image, Sigma, Low, High, ExtractWidth, CompleteJunctions → Lines | | ⬜ |
| `threshold_sub_pix` | Image, Threshold → Border | | ⬜ |
| `zero_crossing_sub_pix` | Image → ZeroCrossing | | ⬜ |

---

## 5. Contour XLD 轮廓操作

### 5.1 轮廓创建/转换

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `gen_contour_polygon_xld` | Row, Col → Contour | `QContour(points)` | ✅ |
| `gen_contour_region_xld` | Region, Mode → Contours | `RegionToContours()` | ✅ |
| `gen_region_contour_xld` | Contour, Mode → Region | `ContourToRegion()` | ✅ |
| `gen_polygons_xld` | Contour, Type, Alpha → Polygons | | ⬜ |
| `gen_parallels_xld` | Contour, Distance, Mode → Parallels | | ⬜ |

### 5.2 轮廓处理

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `smooth_contours_xld` | Contours, NumIterations → SmoothedContours | `SmoothContour()` | ✅ |
| `segment_contours_xld` | Contours, Algorithm, MaxLineDist1/2, MaxAngle → ContoursSplit | `SegmentContour()` | ✅ |
| `union_collinear_contours_xld` | Contours, MaxDistAbs, MaxDistRel, MaxShift, MaxAngle, Mode → UnionContours | | ⬜ |
| `union_adjacent_contours_xld` | Contours, MaxDistAbs, MaxDistRel, Mode → UnionContours | | ⬜ |
| `union_cotangential_contours_xld` | Contours, FitClippingLength, FitLength, MaxTangAngle, MaxDist, MaxDistPerp, MaxOverlap, Mode → UnionContours | | ⬜ |
| `close_contours_xld` | Contours → ClosedContours | `CloseContour()` | ✅ |
| `clip_contours_xld` | Contours, Row1, Col1, Row2, Col2 → ClippedContours | | ⬜ |
| `select_contours_xld` | Contours, Feature, Min, Max, MinLength, MaxLength → SelectedContours | `SelectContours()` | ✅ |
| `select_shape_xld` | Contours, Features, Operation, Min, Max → SelectedContours | | ⬜ |

### 5.3 轮廓特征

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `length_xld` | Contour → Length | `QContour::Length()` | ✅ |
| `area_center_xld` | Contour → Area, Row, Col, PointOrder | `ContourArea()`, `ContourCentroid()` | ✅ |
| `smallest_rectangle1_xld` | Contour → Row1, Col1, Row2, Col2 | `QContour::BoundingBox()` | ✅ |
| `smallest_rectangle2_xld` | Contour → Row, Col, Phi, Length1, Length2 | `SmallestRotatedRectXLD()` | ✅ |
| `smallest_circle_xld` | Contour → Row, Col, Radius | `SmallestCircleXLD()` | ✅ |
| `circularity_xld` | Contour → Circularity | `ContourCircularity()` | ✅ |
| `compactness_xld` | Contour → Compactness | `ContourCompactness()` | ✅ |
| `convexity_xld` | Contour → Convexity | `ContourConvexity()` | ✅ |
| `eccentricity_xld` | Contour → Anisometry, Bulkiness, StructureFactor | | ⬜ |
| `moments_xld` | Contour → M00, M10, M01, M20, M11, M02 | `ContourMoments()` | ✅ |
| `orientation_xld` | Contour → Phi | `ContourOrientation()` | ✅ |
| `get_contour_xld` | Contour → Row, Col | `QContour::GetPoints()` | ✅ |
| `get_contour_attrib_xld` | Contour, Name → Attrib | `GetAmplitudes()`, `GetDirections()` | ✅ |
| `contour_point_num_xld` | Contour → Num | `QContour::Size()` | ✅ |
| `test_closed_xld` | Contour → IsClosed | `QContour::IsClosed()` | ✅ |

### 5.4 轮廓拟合

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `fit_line_contour_xld` | Contour, Algorithm, MaxNumPoints, ClippingEndPoints, Iterations, ClippingFactor → RowBegin, ColBegin, RowEnd, ColEnd, Nr, Nc, Dist | `FitLine()` | ✅ |
| `fit_circle_contour_xld` | Contour, Algorithm, MaxNumPoints, MaxClosureDist, ClippingEndPoints, Iterations, ClippingFactor → Row, Col, Radius, StartPhi, EndPhi, PointOrder | `FitCircle()` | ✅ |
| `fit_ellipse_contour_xld` | Contour, Algorithm, MaxNumPoints, MaxClosureDist, ClippingEndPoints, VossTabSize, Iterations, ClippingFactor → Row, Col, Phi, Radius1, Radius2, StartPhi, EndPhi, PointOrder | `FitEllipse()` | ✅ |
| `fit_rectangle2_contour_xld` | Contour, Algorithm, MaxNumPoints, MaxClosureDist, ClippingEndPoints, Iterations, ClippingFactor → Row, Col, Phi, Length1, Length2, PointOrder | | ⬜ |

---

## 6. Filter 滤波

### 6.1 平滑滤波

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `gauss_filter` | Image, Size → ImageGauss | `GaussFilter()` | ✅ |
| `gauss_image` | Image, Size → ImageGauss | `GaussImage()` | ✅ |
| `mean_image` | Image, MaskWidth, MaskHeight → ImageMean | `MeanImage()` | ✅ |
| `median_image` | Image, MaskType, Radius, Margin → ImageMedian | `MedianImage()` | ✅ |
| `median_rect` | Image, MaskWidth, MaskHeight → ImageMedian | `MedianRect()` | ✅ |
| `median_weighted` | Image, MaskType, MaskSize → ImageMedian | | ⬜ |
| `binomial_filter` | Image, MaskWidth, MaskHeight → ImageBinomial | `BinomialFilter()` | ✅ |
| `bilateral_filter` | Image, SigmaSpatial, SigmaRange, GenParamName, GenParamValue → ImageFiltered | `BilateralFilter()` | ✅ |
| `guided_filter` | Image, ImageGuide, Radius, Amplitude → ImageFiltered | | ⬜ |
| `sigma_image` | Image, MaskSize, Sigma → ImageSigma | | ⬜ |
| `rank_image` | Image, MaskType, Radius, Rank, Margin → ImageRank | `RankImage()` | ✅ |
| `smooth_image` | Image, Filter, Alpha → ImageSmooth | | ⬜ |
| `anisotropic_diffusion` | Image, Mode, Contrast, Theta, Iterations → ImageAniso | `AnisoDiff()` | ✅ |

### 6.2 增强滤波

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `emphasize` | Image, MaskWidth, MaskHeight, Factor → ImageEmphasize | `EmphasizeImage()` | ✅ |
| `unsharp_mask` | Image, Alpha → ImageSharp | `UnsharpMask()` | ✅ (参数不同) |
| `shock_filter` | Image, Theta, Iterations, Mode, Sigma → ImageShock | `ShockFilter()` | ✅ |
| `mean_curvature_flow` | Image, Sigma, Theta, Iterations → ImageMCF | | ⬜ |
| `coherence_enhancing_diff` | Image, Sigma, Rho, Theta, Iterations → ImageCED | | ⬜ |

### 6.3 纹理滤波

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `deviation_image` | Image, Width, Height → ImageDeviation | `StdDevImage()` | ✅ |
| `entropy_image` | Image, Width, Height → ImageEntropy | `EntropyImage()` | ✅ |
| `texture_laws` | Image, FilterTypes, Shift, FilterSize → ImageTexture | | ⬜ |
| `local_max/min` | Image, Height, Width → ImageLocalMax | `MaxImage()`, `MinImage()` | ✅ |

### 6.4 自定义卷积

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `convol_image` | Image, FilterMask, Margin → ImageConvol | `ConvolImage()` | ✅ |
| `gen_filter_mask` | FilterMask, Scale, Width, Height → FilterMask | | ⬜ |
| `convol_gabor` | Image, GaborFilter → ImageGabor | | ⬜ |

---

## 7. Color 颜色

### 7.1 颜色空间转换

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `trans_from_rgb` | ImageR, ImageG, ImageB, ColorSpace → Image1, Image2, Image3 | `TransFromRgb()` | ✅ |
| `trans_to_rgb` | Image1, Image2, Image3, ColorSpace → ImageR, ImageG, ImageB | `TransToRgb()` | ✅ |
| `rgb1_to_gray` | ImageRGB → ImageGray | `Rgb1ToGray()` | ✅ |
| `rgb3_to_gray` | ImageR, ImageG, ImageB → ImageGray | `Rgb3ToGray()` | ✅ |

**支持的颜色空间**: hsv, hls, hsi, yiq, yuv, ciexyz, cielab, cieluv, ycbcr

| 颜色空间 | QiVision 状态 |
|----------|:-------------:|
| hsv | ✅ |
| hsl | ✅ |
| lab | ✅ |
| luv | ✅ |
| xyz | ✅ |
| ycrcb | ✅ |
| yuv | ✅ |
| yiq | ⬜ |
| hsi | ⬜ |

### 7.2 通道操作

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `decompose3` | ImageRGB → Image1, Image2, Image3 | `Decompose3()` | ✅ |
| `decompose4` | ImageRGBA → Image1, Image2, Image3, Image4 | `Decompose4()` | ✅ |
| `compose3` | Image1, Image2, Image3 → ImageRGB | `Compose3()` | ✅ |
| `compose4` | Image1, Image2, Image3, Image4 → ImageRGBA | `Compose4()` | ✅ |
| `access_channel` | Image, Channel → ImageChannel | `AccessChannel()` | ✅ |
| `count_channels` | Image → NumChannels | `CountChannels()` | ✅ |
| `channels_to_image` | Images → MultiChannelImage | `MergeChannels()` | ✅ |
| `image_to_channels` | MultiChannelImage → Images | `SplitChannels()` | ✅ |

### 7.3 颜色增强

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `scale_image` | Image, Mult, Add → ImageScaled | | ⬜ |
| `scale_image_max` | Image → ImageScaled | | ⬜ |
| `illuminate` | Image, MaskWidth, MaskHeight, Factor → ImageIlluminate | | ⬜ |
| `equ_histo_image` | Image → ImageEquHisto | | ⬜ |
| `adapt_histo_image` | Image, RegionAdapt, Factor → ImageAdapt | | ⬜ |
| `invert_image` | Image → ImageInvert | `InvertColors()` | ✅ |
| `gamma_image` | Image, Gamma, Offset, Threshold → ImageGamma | `AdjustGamma()` | ✅ |

### 7.4 LUT 和特殊转换

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `create_color_trans_lut` | ColorSpace, TransDirection, NumBits → ColorTransLUT | `CreateColorTransLut()` | ✅ |
| `apply_color_trans_lut` | Image1/2/3, ColorTransLUT → ImageRes1/2/3 | `ApplyColorTransLut()` | ✅ |
| `clear_color_trans_lut` | ColorTransLUT | `ClearColorTransLut()` | ✅ |
| `cfa_to_rgb` | CFAImage, CFAType, Interpolation → ImageRGB | `CfaToRgb()` | ✅ |
| `linear_trans_color` | Image, TransMat → ImageTrans | `LinearTransColor()` | ✅ |
| `principal_comp` | Image, NumChannels → ImagePCA | `PrincipalComp()` | ✅ |
| `gen_principal_comp_trans` | Image → TransMat, Mean, Eigenvalues | `GenPrincipalCompTrans()` | ✅ |

---

## 8. Matching 模板匹配

### 8.1 Shape-based Matching

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `create_shape_model` | Template, NumLevels, AngleStart, AngleExtent, AngleStep, Optimization, Metric, Contrast, MinContrast → ModelID | `CreateShapeModel()` | ✅ |
| `create_scaled_shape_model` | Template, NumLevels, AngleStart, AngleExtent, AngleStep, ScaleMin, ScaleMax, ScaleStep, Optimization, Metric, Contrast, MinContrast → ModelID | `CreateScaledShapeModel()` | ✅ |
| `create_aniso_shape_model` | Template, NumLevels, AngleStart, AngleExtent, AngleStep, ScaleRMin, ScaleRMax, ScaleRStep, ScaleCMin, ScaleCMax, ScaleCStep, Optimization, Metric, Contrast, MinContrast → ModelID | | ⬜ |
| `find_shape_model` | Image, ModelID, AngleStart, AngleExtent, MinScore, NumMatches, MaxOverlap, SubPixel, NumLevels, Greediness → Row, Column, Angle, Score | `FindShapeModel()` | ✅ |
| `find_scaled_shape_model` | Image, ModelID, AngleStart, AngleExtent, ScaleMin, ScaleMax, MinScore, NumMatches, MaxOverlap, SubPixel, NumLevels, Greediness → Row, Column, Angle, Scale, Score | `FindScaledShapeModel()` | ✅ |
| `find_aniso_shape_model` | ... | | ⬜ |
| `get_shape_model_contours` | ModelID, Level → Contours | `GetShapeModelContours()` | ✅ |
| `get_shape_model_origin` | ModelID → Row, Column | `GetShapeModelOrigin()` | ✅ |
| `set_shape_model_origin` | ModelID, Row, Column | `SetShapeModelOrigin()` | ✅ |
| `get_shape_model_params` | ModelID → NumLevels, AngleStart, AngleExtent, AngleStep, ScaleMin, ScaleMax, ScaleStep, Metric | `GetShapeModelParams()` | ✅ |
| `write_shape_model` | ModelID, FileName | `WriteShapeModel()` | ✅ |
| `read_shape_model` | FileName → ModelID | `ReadShapeModel()` | ✅ |
| `clear_shape_model` | ModelID | `ClearShapeModel()` | ✅ |
| `determine_shape_model_params` | Template, NumLevels, Contrast → ParameterName, ParameterValue | `DetermineShapeModelParams()` | 🟡 |
| `inspect_shape_model` | Image, ModelID, InspectionMode → ImagePyramid, RegionOrigin | `InspectShapeModel()` | 🟡 |

### 8.2 NCC-based Matching

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `create_ncc_model` | Template, NumLevels, AngleStart, AngleExtent, AngleStep, Metric → ModelID | | ⬜ |
| `find_ncc_model` | Image, ModelID, AngleStart, AngleExtent, MinScore, NumMatches, MaxOverlap, SubPixel, NumLevels → Row, Column, Angle, Score | | ⬜ |
| `read_ncc_model` | FileName → ModelID | | ⬜ |
| `write_ncc_model` | ModelID, FileName | | ⬜ |
| `clear_ncc_model` | ModelID | | ⬜ |
| `get_ncc_model_params` | ModelID → ... | | ⬜ |
| `set_ncc_model_origin` | ModelID, Row, Col | | ⬜ |

### 8.3 Component-based Matching

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `create_component_model` | ... | | ⬜ |
| `find_component_model` | ... | | ⬜ |
| `get_component_model_params` | ... | | ⬜ |
| `get_component_model_tree` | ... | | ⬜ |
| `clear_component_model` | ... | | ⬜ |

---

## 9. Measure 测量

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `gen_measure_rectangle2` | Row, Column, Phi, Length1, Length2, Width, Height, Interpolation → MeasureHandle | `MeasureRectangle2` 结构 | ✅ |
| `gen_measure_arc` | CenterRow, CenterCol, Radius, AngleStart, AngleExtent, AnnulusRadius, Width, Height, Interpolation → MeasureHandle | `MeasureArc` 结构 | ✅ |
| `measure_pos` | Image, MeasureHandle, Sigma, Threshold, Transition, Select → RowEdge, ColEdge, Amplitude, Distance | `MeasurePos()` | ✅ |
| `measure_pairs` | Image, MeasureHandle, Sigma, Threshold, Transition, Select → RowEdgeFirst, ColEdgeFirst, AmplitudeFirst, RowEdgeSecond, ColEdgeSecond, AmplitudeSecond, IntraDistance, InterDistance | `MeasurePairs()` | ✅ |
| `fuzzy_measure_pos` | Image, MeasureHandle, Sigma, AmpThresh, FuzzyThresh, Transition → RowEdge, ColEdge, Amplitude, FuzzyScore, Distance | `FuzzyMeasurePos()` | ✅ |
| `fuzzy_measure_pairs` | Image, MeasureHandle, Sigma, AmpThresh, FuzzyThresh, Transition → ... | `FuzzyMeasurePairs()` | ✅ |
| `fuzzy_measure_pairing` | ... | | ⬜ |
| `close_measure` | MeasureHandle | 自动析构 | ✅ |
| `translate_measure` | MeasureHandle, Row, Col | | ⬜ |
| `reset_fuzzy_measure` | MeasureHandle | | ⬜ |
| `set_fuzzy_measure` | MeasureHandle, Transition, Function → Sigma | | ⬜ |
| `set_fuzzy_measure_norm_pair` | ... | | ⬜ |
| `get_measure_param` | MeasureHandle, GenParamName → GenParamValue | | ⬜ |

---

## 10. IO 图像读写

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `read_image` | FileName → Image | `ReadImage()` | ✅ |
| `write_image` | Image, Format, FillColor, FileName | `WriteImage()` | ✅ |
| `list_image_files` | Directory, Extensions, Options → ImageFiles | `ReadDirectory()` | ✅ |
| `parse_filename` | FileName → BaseName, Extension, Directory | | ⬜ |
| `get_image_size` | Image → Width, Height | `QImage::Width()`, `Height()` | ✅ |
| `get_image_type` | Image → Type | `QImage::Type()` | ✅ |
| `get_domain` | Image → Domain | `QImage::GetDomain()` | ✅ |
| `get_image_pointer1` | Image → Pointer, Type, Width, Height | `QImage::Data()` | ✅ |
| `count_channels` | Image → NumChannels | `QImage::Channels()` | ✅ |

---

## 11. Transform 几何变换

### 11.1 仿射变换

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `hom_mat2d_identity` | → HomMat2D | `QMatrix::Identity()` | ✅ |
| `hom_mat2d_translate` | HomMat2D, Tx, Ty → HomMat2DTranslate | `QMatrix::Translate()` | ✅ |
| `hom_mat2d_rotate` | HomMat2D, Phi, Px, Py → HomMat2DRotate | `QMatrix::Rotate()` | ✅ |
| `hom_mat2d_scale` | HomMat2D, Sx, Sy, Px, Py → HomMat2DScale | `QMatrix::Scale()` | ✅ |
| `hom_mat2d_slant` | HomMat2D, Theta, Axis, Px, Py → HomMat2DSlant | | ⬜ |
| `hom_mat2d_reflect` | HomMat2D, Px, Py, Qx, Qy → HomMat2DReflect | | ⬜ |
| `hom_mat2d_compose` | HomMat2DLeft, HomMat2DRight → HomMat2DCompose | `QMatrix::Compose()` | ✅ |
| `hom_mat2d_invert` | HomMat2D → HomMat2DInvert | `QMatrix::Inverse()` | ✅ |
| `affine_trans_image` | Image, HomMat2D, Interpolation, AdaptImageSize → ImageAffineTrans | `AffineTransformImage()` | ✅ |
| `affine_trans_region` | Region, HomMat2D, Interpolation → RegionAffineTrans | | ⬜ |
| `affine_trans_contour_xld` | Contours, HomMat2D → ContoursAffineTrans | `QContour::Transform()` | ✅ |
| `affine_trans_point_2d` | HomMat2D, Px, Py → Qx, Qy | `QMatrix::TransformPoint()` | ✅ |
| `affine_trans_pixel` | HomMat2D, Row, Col → RowTrans, ColTrans | | ⬜ |
| `vector_angle_to_rigid` | Row1, Col1, Angle1, Row2, Col2, Angle2 → HomMat2D | | ⬜ |
| `vector_to_rigid` | Px1, Py1, Px2, Py2 → HomMat2D | | ⬜ |
| `vector_to_similarity` | Px1, Py1, Px2, Py2 → HomMat2D | | ⬜ |
| `vector_to_aniso` | Px1, Py1, Px2, Py2 → HomMat2D | | ⬜ |

### 11.2 透视变换

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `hom_mat3d_identity` | → HomMat3D | | ⬜ |
| `hom_mat3d_compose` | HomMat3DLeft, HomMat3DRight → HomMat3DCompose | | ⬜ |
| `projective_trans_image` | Image, HomMat2D, Interpolation, AdaptImageSize, TransformDomain → ImageTrans | `PerspectiveTransform()` | ✅ |
| `projective_trans_region` | Region, HomMat2D, Interpolation → RegionTrans | | ⬜ |
| `projective_trans_contour_xld` | Contours, HomMat2D → ContoursTrans | | ⬜ |
| `projective_trans_point_2d` | HomMat2D, Px, Py → Qx, Qy | `HomographyTransformPoint()` | ✅ |
| `vector_to_proj_hom_mat2d` | Px, Py, Qx, Qy, Method → HomMat2D | `ComputeHomography()` | ✅ |
| `hom_vector_to_proj_hom_mat2d` | Px, Py, Pw, Qx, Qy, Qw, Method → HomMat2D | | ⬜ |

### 11.3 极坐标变换

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `polar_trans_image` | Image, Row, Column, AngleStart, AngleEnd, RadiusStart, RadiusEnd, Width, Height, Interpolation → ImagePolar | | ⬜ |
| `polar_trans_image_inv` | ImagePolar, Row, Column, AngleStart, AngleEnd, RadiusStart, RadiusEnd, Width, Height, Interpolation → ImageCart | | ⬜ |
| `polar_trans_region` | Region, Row, Column, AngleStart, AngleEnd, RadiusStart, RadiusEnd, Width, Height, Interpolation → RegionPolar | | ⬜ |
| `polar_trans_contour_xld` | Contours, Row, Column, AngleStart, AngleEnd, RadiusStart, RadiusEnd, Width, Height → ContoursPolar | | ⬜ |

---

## 12. Distance / Fitting

### 12.1 距离变换

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `distance_transform` | Region, Metric, Foreground, Width, Height → DistanceImage | `DistanceTransform()` | ✅ |
| `distance_pc` | Region, RegionReference, Metric → DistanceMin, DistanceMax | | ⬜ |
| `distance_pr` | Region, Row, Column → DistanceMin, DistanceMax | `DistancePointToRegion()` | ✅ |
| `distance_pp` | Row1, Column1, Row2, Column2 → Distance | `Point2d::Distance()` | ✅ |
| `distance_pl` | Row, Column, Row1, Column1, Row2, Column2 → Distance | `DistancePointToLine()` | ✅ |
| `distance_ps` | Row, Column, Row1, Column1, Row2, Column2 → Distance | `DistancePointToSegment()` | ✅ |
| `distance_sc` | Contours, Row, Column → DistanceMin, DistanceMax | `DistancePointToContour()` | ✅ |
| `distance_cc` | Contours1, Contours2, Mode → DistanceMin, DistanceMax | `DistanceContourToContour()` | ⬜ |
| `distance_rr_min_dil` | Region1, Region2 → DistanceMin | | ⬜ |

### 12.2 几何拟合

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `fit_line` | Rows, Cols, Algorithm → RowBegin, ColBegin, RowEnd, ColEnd, Nr, Nc, Dist | `FitLine()` | ✅ |
| `fit_circle` | Rows, Cols, Algorithm → Row, Column, Radius | `FitCircle()` | ✅ |
| `fit_ellipse` | Rows, Cols, Algorithm → Row, Column, Phi, Radius1, Radius2 | `FitEllipse()` | ✅ |
| `fit_rectangle2` | Rows, Cols, Algorithm → Row, Column, Phi, Length1, Length2 | | ⬜ |

---

## 13. 霍夫变换

| Halcon 算子 | 参数 | QiVision 对应 | 状态 |
|-------------|------|---------------|:----:|
| `hough_lines` | RegionIn, AngleResolution, Threshold, AngleGap, DistGap → HoughLines | `HoughLines()` | ✅ |
| `hough_lines_dir` | ImageDir, ImageAmp, DirectionUncertainty, AngleResolution, Smoothing, FilterType, GenLines, ReturnHoughImage → HoughImage, Lines | | ⬜ |
| `hough_circles` | RegionIn, Radius, Percent, Mode → Row, Column | `HoughCircles()` | ✅ |
| `hough_circle_trans` | Region, Radius → HoughImage | | ⬜ |
| `select_lines` | HoughLines, Type, LinesSelected | | ⬜ |

---

## 统计

| 类别 | 已实现 | 部分实现 | 未实现 | 总计 |
|------|:------:|:--------:|:------:|:----:|
| Region 区域 | 28 | 1 | 15 | 44 |
| Connection 连通域 | 3 | 0 | 3 | 6 |
| Morphology 形态学 | 30 | 0 | 6 | 36 |
| Edge 边缘 | 10 | 0 | 11 | 21 |
| Contour XLD | 24 | 0 | 9 | 33 |
| Filter 滤波 | 20 | 0 | 9 | 29 |
| Color 颜色 | 22 | 0 | 5 | 27 |
| Matching 匹配 | 15 | 2 | 12 | 29 |
| Measure 测量 | 6 | 0 | 6 | 12 |
| IO 读写 | 7 | 0 | 2 | 9 |
| Transform 变换 | 14 | 0 | 16 | 30 |
| Distance/Fitting | 9 | 0 | 4 | 13 |
| Hough | 2 | 0 | 3 | 5 |
| **总计** | **190** | **3** | **101** | **294** |

**覆盖率**: 190/294 = **65%**
