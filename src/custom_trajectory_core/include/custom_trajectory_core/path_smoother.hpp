#ifndef CUSTOM_TRAJECTORY_CORE_PATH_SMOOTHER_HPP_
#define CUSTOM_TRAJECTORY_CORE_PATH_SMOOTHER_HPP_

#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>
#include <functional> // 用于引入回调函数

namespace custom_trajectory {


struct Point2D {
    double x;
    double y;
};


struct Obstacle {
    double x, y, radius;
};

class PathSmoother {
public:
    
    using CollisionChecker = std::function<bool(const Point2D&, const Point2D&, const std::vector<Obstacle>&)>;

    // 贪心剪枝法 
    std::vector<Point2D> prunePath(
        const std::vector<Point2D>& raw_path, 
        const std::vector<Obstacle>& obstacles,
        const CollisionChecker& check_collision);

    // 二阶贝塞尔曲线优化 
    std::vector<Point2D> bezierSmooth(
        const Point2D& p0, const Point2D& p1, const Point2D& p2, double radius);

    // 综合平滑接口
    std::vector<Point2D> processPath(
        const std::vector<Point2D>& raw_path, 
        const std::vector<Obstacle>& obstacles, 
        double radius,
        const CollisionChecker& check_collision);
};

} // namespace custom_trajectory

#endif // CUSTOM_TRAJECTORY_CORE_PATH_SMOOTHER_HPP_