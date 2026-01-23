/**
 * @file caliper_circle_manual.cpp
 * @brief Manual circle measurement using multiple calipers
 *
 * 对比说明：
 * - Metrology 高级 API: AddCircleMeasure(row, col, radius) 自动放置卡尺
 * - Caliper 低级 API: 手动计算每个卡尺的位置和方向
 *
 * 本示例展示"手动放置"的含义：自己计算卡尺位置，沿圆周分布
 */

#include <QiVision/Core/QImage.h>
#include <QiVision/Display/Draw.h>
#include <QiVision/Color/ColorConvert.h>
#include <QiVision/Measure/Caliper.h>
#include <QiVision/Measure/MeasureHandle.h>
#include <QiVision/Internal/Fitting.h>
#include <QiVision/GUI/Window.h>

#include <iostream>
#include <iomanip>
#include <cmath>

using namespace Qi::Vision;
using namespace Qi::Vision::Measure;
using namespace Qi::Vision::GUI;

int main() {
    std::string imagePath = "tests/data/halcon_images/circle_plate.png";

    QImage image = QImage::FromFile(imagePath);
    if (!image.IsValid()) {
        std::cerr << "Failed to load: " << imagePath << std::endl;
        return 1;
    }
    QImage gray = image.ToGray();

    std::cout << "=== Manual Circle Measurement Demo ===" << std::endl;
    std::cout << "Image: " << image.Width() << " x " << image.Height() << std::endl;

    // =========================================================================
    // 圆的初始参数（大致位置）
    // =========================================================================
    double initCenterRow = 420.0;
    double initCenterCol = 210.0;
    double initRadius = 60.0;

    std::cout << "\nInitial circle estimate:" << std::endl;
    std::cout << "  Center: (" << initCenterCol << ", " << initCenterRow << ")" << std::endl;
    std::cout << "  Radius: " << initRadius << " px" << std::endl;

    // =========================================================================
    // 手动放置卡尺：沿圆周均匀分布
    // =========================================================================
    int numCalipers = 12;  // 卡尺数量
    double length1 = 20.0; // 卡尺投影方向半长（径向）
    double length2 = 5.0;  // 卡尺边缘方向半长（切向）

    std::vector<MeasureRectangle2> calipers;

    std::cout << "\n=== Manual Caliper Placement ===" << std::endl;
    std::cout << "Placing " << numCalipers << " calipers around circle..." << std::endl;

    for (int i = 0; i < numCalipers; ++i) {
        // 计算卡尺在圆周上的角度位置
        double angle = 2.0 * M_PI * i / numCalipers;

        // 卡尺中心：放在圆周上（或稍微偏外）
        double caliperRow = initCenterRow + initRadius * std::sin(angle);
        double caliperCol = initCenterCol + initRadius * std::cos(angle);

        // 卡尺方向 (phi)：切线方向 = angle + PI/2
        // 因为 phi 定义为边缘方向，投影方向 = phi + PI/2 = angle + PI = 径向
        double phi = angle + M_PI / 2.0;

        // 创建卡尺
        auto handle = GenMeasureRectangle2(caliperRow, caliperCol, phi, length1, length2);
        calipers.push_back(handle);

        std::cout << "  Caliper " << std::setw(2) << (i+1)
                  << ": pos=(" << std::fixed << std::setprecision(1)
                  << caliperCol << ", " << caliperRow << ")"
                  << " angle=" << std::setprecision(2) << (angle * 180 / M_PI) << " deg"
                  << std::endl;
    }

    // =========================================================================
    // 用每个卡尺检测边缘
    // =========================================================================
    // Halcon-style direct parameters
    double sigma = 1.0;
    double threshold = 20.0;
    std::string transition = "all";   // 检测所有边缘
    std::string selectMode = "first"; // 选最近的边缘

    std::vector<Point2d> edgePoints;

    std::cout << "\n=== Edge Detection ===" << std::endl;

    for (int i = 0; i < numCalipers; ++i) {
        auto edges = MeasurePos(gray, calipers[i], sigma, threshold, transition, selectMode);

        if (!edges.empty()) {
            // 取第一个边缘
            Point2d pt(edges[0].column, edges[0].row);
            edgePoints.push_back(pt);
            std::cout << "  Caliper " << std::setw(2) << (i+1)
                      << ": edge at (" << std::setprecision(2)
                      << pt.x << ", " << pt.y << ") amp=" << edges[0].amplitude
                      << std::endl;
        } else {
            std::cout << "  Caliper " << std::setw(2) << (i+1) << ": no edge found" << std::endl;
        }
    }

    // =========================================================================
    // 拟合圆（使用检测到的边缘点）
    // =========================================================================
    std::cout << "\n=== Circle Fitting ===" << std::endl;
    std::cout << "Points found: " << edgePoints.size() << "/" << numCalipers << std::endl;

    if (edgePoints.size() >= 3) {
        // 使用 RANSAC 拟合圆
        Internal::RansacParams ransacParams;
        ransacParams.SetThreshold(3.5).SetMaxIterations(100);
        auto result = Internal::FitCircleRANSAC(edgePoints, ransacParams);

        std::cout << "\nFitted circle:" << std::endl;
        std::cout << "  Center: (" << std::setprecision(2)
                  << result.circle.center.x << ", " << result.circle.center.y << ")" << std::endl;
        std::cout << "  Radius: " << result.circle.radius << " px" << std::endl;
        std::cout << "  Inliers: " << result.numInliers << "/" << edgePoints.size() << std::endl;
        std::cout << "  RMS error: " << std::setprecision(3) << result.residualRMS << " px" << std::endl;

        // =========================================================================
        // 可视化
        // =========================================================================
        Scalar cyan(0, 255, 255);    // 卡尺工具
        Scalar green(0, 255, 0);     // 边缘点（内点）
        Scalar red(255, 0, 0);       // 离群点
        Scalar yellow(255, 255, 0);  // 拟合结果

        QImage colorImg;
        Color::GrayToRgb(gray, colorImg);

        // 绘制所有卡尺
        for (const auto& caliper : calipers) {
            Draw::MeasureRect(colorImg, caliper, cyan, 1);
        }

        // 绘制边缘点（根据权重着色）
        for (size_t i = 0; i < edgePoints.size(); ++i) {
            bool isInlier = (i < result.weights.size() && result.weights[i] > 0.5);
            Scalar color = isInlier ? green : red;
            Draw::FilledCircle(colorImg, edgePoints[i], 4, color);
        }

        // 绘制拟合圆
        Draw::Circle(colorImg, result.circle.center, result.circle.radius, yellow, 2);
        Draw::Cross(colorImg, result.circle.center, 10, 0, yellow, 2);

        // 绘制初始圆（虚线效果用点表示）
        for (int i = 0; i < 36; ++i) {
            double angle = 2.0 * M_PI * i / 36;
            Point2d pt(initCenterCol + initRadius * std::cos(angle),
                       initCenterRow + initRadius * std::sin(angle));
            Draw::Pixel(colorImg, static_cast<int32_t>(pt.x), static_cast<int32_t>(pt.y), cyan);
        }

        // 显示
        Window win("Manual Circle Measurement");
        win.SetAutoResize(true);
        win.EnablePixelInfo(true);
        win.DispImage(colorImg);

        std::cout << "\n[Visualization]" << std::endl;
        std::cout << "  Cyan: Caliper tools (with projection lines)" << std::endl;
        std::cout << "  Green: Edge points (inliers)" << std::endl;
        std::cout << "  Red: Edge points (outliers)" << std::endl;
        std::cout << "  Yellow: Fitted circle" << std::endl;
        std::cout << "\nPress any key to close..." << std::endl;

        win.WaitKey(0);
    } else {
        std::cerr << "Not enough points to fit circle!" << std::endl;
        return 1;
    }

    return 0;
}
