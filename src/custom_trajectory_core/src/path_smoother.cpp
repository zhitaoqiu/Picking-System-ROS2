#include "custom_trajectory_core/path_smoother.hpp"

namespace custom_trajectory {

std::vector<Point2D> PathSmoother::prunePath(
    const std::vector<Point2D>& raw_path, 
    const std::vector<Obstacle>& obstacles,
    const CollisionChecker& check_collision) 
{
    if (raw_path.size() <= 2) {
        return raw_path;
    }

    std::vector<Point2D> smoothed_path;
    smoothed_path.push_back(raw_path.front()); 

    int current_id = 0;
    int target_id = raw_path.size() - 1;

    while (current_id <= target_id) {
        for (int i = target_id; i > current_id; --i) {
            
            if (!check_collision(raw_path[current_id], raw_path[i], obstacles)) {
                smoothed_path.push_back(raw_path[i]);
                current_id = i;
                break;
            }
        }
    }
    return smoothed_path;
}

std::vector<Point2D> PathSmoother::bezierSmooth(
    const Point2D& p0, const Point2D& p1, const Point2D& p2, double radius) 
{
    double d1 = std::hypot(p1.x - p0.x, p1.y - p0.y);
    double d2 = std::hypot(p2.x - p1.x, p2.y - p1.y);
    
    
    if (d1 < 1e-5 || d2 < 1e-5) return {p1}; 

    double actual_radius = std::min(radius, std::min(d1, d2) / 2.0);
    
    Point2D b0 = { p1.x - actual_radius * (p1.x - p0.x) / d1, 
                   p1.y - actual_radius * (p1.y - p0.y) / d1 };
    Point2D b1 = { p1.x + actual_radius * (p2.x - p1.x) / d2, 
                   p1.y + actual_radius * (p2.y - p1.y) / d2 };

    std::vector<Point2D> blended_path;
    const int num_points = 10;
    
    
    for (int i = 0; i <= num_points; ++i) { 
        double t = static_cast<double>(i) / num_points;
        double u = 1.0 - t;
        double bx = u * u * b0.x + 2 * t * u * p1.x + t * t * b1.x;
        double by = u * u * b0.y + 2 * t * u * p1.y + t * t * b1.y;
        blended_path.push_back({bx, by});
    }

    return blended_path;
}

std::vector<Point2D> PathSmoother::processPath(
    const std::vector<Point2D>& raw_path, 
    const std::vector<Obstacle>& obstacles, 
    double radius,
    const CollisionChecker& check_collision) 
{
    auto pruned_path = prunePath(raw_path, obstacles, check_collision);
    std::vector<Point2D> smoothed_path;
    
    for (size_t i = 0; i < pruned_path.size(); ++i) {
        if (i > 0 && i < pruned_path.size() - 1) {
            auto blended_points = bezierSmooth(pruned_path[i-1], pruned_path[i], pruned_path[i+1], radius);
            // 避免把拐点重复插入
            if (!smoothed_path.empty()) {
                 smoothed_path.pop_back(); 
            }
            smoothed_path.insert(smoothed_path.end(), blended_points.begin(), blended_points.end());
        } else {
            smoothed_path.push_back(pruned_path[i]);
        }
    }
    return smoothed_path;
}

} // namespace custom_trajectory