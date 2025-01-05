#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <vector>
#include <cmath>

class GlobalPathPlanner : public rclcpp::Node
{
public:
    GlobalPathPlanner() : Node("global_path_planner")
    {
        // Subscribers and Publishers
        map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "occupancy_map", 10,
            std::bind(&GlobalPathPlanner::map_callback, this, std::placeholders::_1));

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("global_path_marker", 10);

        RCLCPP_INFO(this->get_logger(), "Global Path Planner Node has started.");
    }

    std::vector<std::pair<int, int>> find_path(int start_x, int start_y, int goal_x, int goal_y)
    {
        std::vector<std::pair<int, int>> path;

        // Farkları hesapla
        int dx = goal_x - start_x;
        int dy = goal_y - start_y;

        // Çizgi interpolasyonu için adımları hesapla
        int steps = std::max(abs(dx), abs(dy));

        // Her bir adım için noktaları hesapla
        for (int i = 0; i <= steps; ++i)
        {
            int x = start_x + i * dx / steps;
            int y = start_y + i * dy / steps;
            path.emplace_back(x, y);
        }

        return path;
    }

    void publish_marker(const std::vector<std::pair<int, int>> &path)
    {
        if (path.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Path is empty. No marker will be published.");
            return;
        }

        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "global_path";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1; // Line width
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        for (const auto &[x, y] : path)
        {
            geometry_msgs::msg::Point p;
            p.x = x * map_->info.resolution + map_->info.origin.position.x;
            p.y = y * map_->info.resolution + map_->info.origin.position.y;
            p.z = 0.0;
            marker.points.push_back(p);
        }

        marker_publisher_->publish(marker);
        RCLCPP_INFO(this->get_logger(), "Published global path marker.");
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
    {
        // Save map for planning
        map_ = map;
        RCLCPP_INFO(this->get_logger(), "Occupancy map received.");

        // Başlangıç ve hedef koordinatları
        int start_x = 0, start_y = 0;
        int goal_x = 29, goal_y = 29;

        // Yol planlaması
        auto path = find_path(start_x, start_y, goal_x, goal_y);

        // Marker yayınla
        publish_marker(path);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalPathPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
