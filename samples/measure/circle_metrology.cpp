/**
 * @file circle_metrology.cpp
 * @brief Circle metrology demo - measure multiple circles
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Display/Draw.h>
#include <QiVision/Color/ColorConvert.h>
#include <QiVision/Measure/Metrology.h>
#include <QiVision/Platform/Timer.h>
#include <QiVision/GUI/Window.h>

#include <iostream>
#include <iomanip>
#include <vector>

using namespace Qi::Vision;
using namespace Qi::Vision::Measure;
using namespace Qi::Vision::Platform;
using namespace Qi::Vision::GUI;

int main() {
    std::string imagePath = "tests/data/halcon_images/circle_plate.png";

    // 预设要测量的圆 (cx, cy, radius)
    std::vector<std::tuple<double, double, double>> circles = {
        {210, 420, 60},
        {500, 420, 60},
        {790, 420, 60},
        {1077, 420, 110},
    };

    QImage image = QImage::FromFile(imagePath);
    if (!image.IsValid()) {
        std::cerr << "Failed to load: " << imagePath << std::endl;
        return 1;
    }
    QImage grayImage = image.ToGray();

    std::cout << "=== Circle Metrology Demo ===" << std::endl;
    std::cout << "Image: " << image.Width() << " x " << image.Height() << std::endl;
    std::cout << "Circles to measure: " << circles.size() << std::endl;

    // 测量参数
    MetrologyMeasureParams params;
    params.numMeasures = 20;
    params.measureLength1 = 20.0;
    params.measureLength2 = 5.0;
    params.thresholdMode = ThresholdMode::Auto;
    params.fitMethod = MetrologyFitMethod::RANSAC;

    // 创建模型，添加所有圆
    // AddCircleMeasure(x, y, radius)
    MetrologyModel model;
    for (const auto& [cx, cy, r] : circles) {
        model.AddCircleMeasure(cx, cy, r, params);  // x, y, radius
    }

    // 测量
    Timer timer;
    timer.Start();
    model.Apply(grayImage);
    double elapsed = timer.ElapsedMs();

    // 输出结果
    std::cout << "\n=== Results ===" << std::endl;
    for (int i = 0; i < static_cast<int>(circles.size()); ++i) {
        auto [cx, cy, r] = circles[i];
        auto result = model.GetCircleResult(i);
        auto points = model.GetMeasuredPoints(i);

        // 使用新的统一接口获取卡尺工具信息
        const auto* obj = model.GetObject(i);
        auto center = obj->GetCenter();  // 获取初始几何中心

        std::cout << "\nCircle " << (i+1) << ":" << std::endl;
        std::cout << "  Initial center: (" << center.x << ", " << center.y << ") r=" << r << std::endl;
        std::cout << "  Fitted center:  (" << std::fixed << std::setprecision(2)
                  << result.x << ", " << result.y << ") r=" << result.radius << std::endl;
        std::cout << "  Points: " << result.numUsed << "/" << points.size()
                  << "  RMS: " << std::setprecision(3) << result.rmsError << " px" << std::endl;
    }
    std::cout << "\nTotal time: " << std::setprecision(2) << elapsed << " ms" << std::endl;

    // 可视化
    QImage colorImg;
    Color::GrayToRgb(grayImage, colorImg);
    Draw::MetrologyModelResult(colorImg, model);

    // 显示
    Window win("Circle Metrology");
    win.SetAutoResize(true);
    win.EnablePixelInfo(true);  // Show mouse position and pixel value
    win.DispImage(colorImg);
    std::cout << "\nMove mouse to see coordinates. Press any key to close..." << std::endl;
    win.WaitKey(0);

    return 0;
}
