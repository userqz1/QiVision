/**
 * @file circle_metrology.cpp
 * @brief Metrology demo - measure circles and rectangles
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

    QImage image = QImage::FromFile(imagePath);
    if (!image.IsValid()) {
        std::cerr << "Failed to load: " << imagePath << std::endl;
        return 1;
    }
    QImage grayImage = image.ToGray();

    std::cout << "=== Metrology Demo ===" << std::endl;
    std::cout << "Image: " << image.Width() << " x " << image.Height() << std::endl;

    // 测量参数
    MetrologyMeasureParams params;
    params.numMeasures = 20;
    params.measureLength1 = 20.0;
    params.measureLength2 = 5.0;
    params.thresholdMode = ThresholdMode::Auto;
    params.fitMethod = MetrologyFitMethod::RANSAC;

    // 创建模型
    MetrologyModel model;

    // 添加圆 (row, col, radius)
    model.AddCircleMeasure(420, 210, 60, params);
    model.AddCircleMeasure(420, 500, 60, params);
    model.AddCircleMeasure(420, 790, 60, params);
    model.AddCircleMeasure(420, 1077, 110, params);

    // 添加矩形 (row, col, phi, length1, length2)
    // 中心(col=210, row=705), length1=65(x方向), length2=70(y方向)
    MetrologyMeasureParams rectParams = params;
    rectParams.numMeasures = 32;  // 每边8个卡尺
    model.AddRectangle2Measure(705, 210, 0.0, 65, 70, rectParams);

    // 测量
    Timer timer;
    timer.Start();
    model.Apply(grayImage);
    double elapsed = timer.ElapsedMs();

    // 输出结果
    std::cout << "\n=== Results ===" << std::endl;
    int numObjects = model.NumObjects();
    for (int i = 0; i < numObjects; ++i) {
        const auto* obj = model.GetObject(i);
        auto points = model.GetMeasuredPoints(i);
        auto center = obj->GetCenter();

        std::cout << "\nObject " << (i+1) << " ("
                  << (obj->Type() == MetrologyObjectType::Circle ? "Circle" : "Rectangle") << "):" << std::endl;

        if (obj->HasCenter()) {
            std::cout << "  Initial center: (" << center.x << ", " << center.y << ")" << std::endl;
        }

        if (obj->Type() == MetrologyObjectType::Circle) {
            auto result = model.GetCircleResult(i);
            std::cout << "  Fitted center:  (" << std::fixed << std::setprecision(2)
                      << result.column << ", " << result.row << ") r=" << result.radius << std::endl;
            std::cout << "  Points: " << result.numUsed << "/" << points.size()
                      << "  RMS: " << std::setprecision(3) << result.rmsError << " px" << std::endl;
        } else if (obj->Type() == MetrologyObjectType::Rectangle2) {
            auto result = model.GetRectangle2Result(i);
            std::cout << "  Fitted center:  (" << std::fixed << std::setprecision(2)
                      << result.column << ", " << result.row << ") phi=" << result.phi << std::endl;
            std::cout << "  Size: length1=" << result.length1 << " length2=" << result.length2 << std::endl;
            std::cout << "  Points: " << result.numUsed << "/" << points.size()
                      << "  RMS: " << std::setprecision(3) << result.rmsError << " px" << std::endl;
        }
    }
    std::cout << "\nTotal time: " << std::setprecision(2) << elapsed << " ms" << std::endl;

    // 可视化
    QImage colorImg;
    Color::GrayToRgb(grayImage, colorImg);
    Draw::MetrologyModelResult(colorImg, model);

    // 显示
    Window win("Metrology Demo");
    win.SetAutoResize(true);
    win.EnablePixelInfo(true);
    win.DispImage(colorImg);
    std::cout << "\nMove mouse to see coordinates. Press any key to close..." << std::endl;
    win.WaitKey(0);

    return 0;
}
