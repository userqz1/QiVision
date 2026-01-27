/**
 * @file circle_metrology.cpp
 * @brief Metrology demo - measure circles and rectangles
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Display/Draw.h>
#include <QiVision/Color/ColorConvert.h>
#include <QiVision/Measure/Metrology.h>
#include <QiVision/IO/ImageIO.h>
#include <QiVision/Platform/Timer.h>
#include <QiVision/GUI/Window.h>

#include <iostream>
#include <iomanip>
#include <vector>

using namespace Qi::Vision;
using namespace Qi::Vision::Measure;
using namespace Qi::Vision::IO;
using namespace Qi::Vision::Platform;
using namespace Qi::Vision::GUI;

int main() {
    std::string imagePath = "tests/data/halcon_images/circle_plate.png";

    QImage grayImage;
    ReadImageGray(imagePath, grayImage);
    if (grayImage.Empty()) {
        std::cerr << "Failed to load: " << imagePath << std::endl;
        return 1;
    }

    std::cout << "=== Metrology Demo ===" << std::endl;
    std::cout << "Image: " << grayImage.Width() << " x " << grayImage.Height() << std::endl;

    // 创建模型
    MetrologyModel model;

    // 添加圆 (row, col, radius, measureLength1, measureLength2, transition, select, params)
    // 使用 MetrologyMeasureParams 传递额外参数
    MetrologyMeasureParams circleParams;
    circleParams.SetNumMeasures(20).SetThreshold("auto");
    model.AddCircleMeasure(420, 210, 60, 20.0, 5.0, "all", "all", circleParams);
    model.AddCircleMeasure(420, 500, 60, 20.0, 5.0, "all", "all", circleParams);
    model.AddCircleMeasure(420, 790, 60, 20.0, 5.0, "all", "all", circleParams);
    model.AddCircleMeasure(420, 1077, 110, 20.0, 5.0, "all", "all", circleParams);

    // 添加矩形 (row, col, phi, length1, length2, measureLength1, measureLength2, ...)
    // 中心(col=210, row=705), length1=65(x方向), length2=70(y方向)
    MetrologyMeasureParams rectParams;
    rectParams.SetNumMeasures(32).SetThreshold("auto");
    model.AddRectangle2Measure(705, 210, 0.0, 65, 70, 20.0, 5.0, "all", "all", rectParams);

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

    // 保存结果
    WriteImage(colorImg, "tests/output/metrology_result.png");

    // 显示
    Window win("Metrology Demo");
    win.SetAutoResize(true);
    win.EnablePixelInfo(true);
    win.DispImage(colorImg);
    std::cout << "\nMove mouse to see coordinates. Press any key to close..." << std::endl;
    win.WaitKey(0);

    return 0;
}
