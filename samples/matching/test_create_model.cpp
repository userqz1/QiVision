/**
 * @file test_create_model.cpp
 * @brief 测试模板创建 - 手动画 ROI 并显示提取的轮廓信息
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Core/QContour.h>
#include <QiVision/Display/Draw.h>
#include <QiVision/Color/ColorConvert.h>
#include <QiVision/Matching/ShapeModel.h>
#include <QiVision/Platform/Timer.h>
#include <QiVision/GUI/Window.h>

#include <iostream>
#include <iomanip>
#include <sstream>

using namespace Qi::Vision;
using namespace Qi::Vision::Matching;
using namespace Qi::Vision::GUI;

// 颜色表
static const Scalar kColors[] = {
    Scalar::Red(), Scalar::Green(), Scalar::Blue(), Scalar::Yellow(),
    Scalar::Cyan(), Scalar::Magenta(), Scalar(255,128,0), Scalar(128,0,255)
};

void PrintContourInfo(const QContourArray& contours, int32_t level) {
    std::cout << "\n=== Level " << level << " ===" << std::endl;
    std::cout << "轮廓数: " << contours.Size() << std::endl;

    size_t total = 0;
    for (size_t i = 0; i < contours.Size(); ++i) {
        const QContour& c = contours[i];
        total += c.Size();
        std::cout << "  #" << (i+1) << ": " << c.Size() << " 点, 长度 "
                  << std::fixed << std::setprecision(1) << c.Length() << " px" << std::endl;
    }
    std::cout << "总点数: " << total << std::endl;
}

int main(int argc, char* argv[]) {
    std::string imagePath = argc > 1 ? argv[1] : "tests/data/matching/image1/052640-20210901141310.jpg";

    std::cout << "=== 模板创建测试 ===" << std::endl;

    // 加载图像
    QImage img = QImage::FromFile(imagePath);
    if (!img.IsValid()) {
        std::cerr << "加载失败: " << imagePath << std::endl;
        return 1;
    }
    QImage gray = img.ToGray();
    std::cout << "图像: " << gray.Width() << " x " << gray.Height() << std::endl;

    // 创建窗口，绘制 ROI
    Window win("绘制 ROI");
    win.SetAutoResize(true);
    win.DispImage(gray);

    ROIResult roi = win.DrawRectangle();
    if (!roi.valid) {
        std::cerr << "已取消" << std::endl;
        return 1;
    }

    Rect2i rect;
    rect.x = static_cast<int32_t>(std::min(roi.col1, roi.col2));
    rect.y = static_cast<int32_t>(std::min(roi.row1, roi.row2));
    rect.width = static_cast<int32_t>(std::abs(roi.col2 - roi.col1));
    rect.height = static_cast<int32_t>(std::abs(roi.row2 - roi.row1));
    std::cout << "ROI: " << rect.x << "," << rect.y << " " << rect.width << "x" << rect.height << std::endl;

    // 创建模板
    Platform::Timer timer;
    timer.Start();
    ShapeModel model;
    CreateShapeModel(gray, rect, model, 4, 0, RAD(360), 0, "auto", "use_polarity", "auto", 10);
    std::cout << "创建耗时: " << timer.ElapsedMs() << " ms" << std::endl;

    if (!model.IsValid()) {
        std::cerr << "创建失败" << std::endl;
        return 1;
    }

    // 显示各层轮廓
    int32_t level = 1;
    bool running = true;

    while (running) {
        QContourArray contours;
        GetShapeModelXLD(model, level, contours);
        PrintContourInfo(contours, level);

        // 绘制
        QImage vis;
        Color::GrayToRgb(gray, vis);
        Draw::Rectangle(vis, rect.x, rect.y, rect.width, rect.height, Scalar::White(), 1);

        double cx = rect.x + rect.width / 2.0;
        double cy = rect.y + rect.height / 2.0;

        for (size_t i = 0; i < contours.Size(); ++i) {
            QContour shifted;
            for (size_t j = 0; j < contours[i].Size(); ++j) {
                Point2d p = contours[i].GetPoint(j);
                shifted.AddPoint(p.x + cx, p.y + cy);
            }
            Draw::Contour(vis, shifted, kColors[i % 8], 2);
        }

        Draw::Cross(vis, Point2d{cx, cy}, 15, 0, Scalar::Yellow(), 2);

        std::ostringstream title;
        title << "Level " << level << " - " << contours.Size() << " 轮廓 (空格切换, q退出)";
        win.SetTitle(title.str());
        win.DispImage(vis);

        int key = win.WaitKey(0);
        if (key == 'q' || key == 27) running = false;
        else level = (level % 4) + 1;
    }

    return 0;
}
