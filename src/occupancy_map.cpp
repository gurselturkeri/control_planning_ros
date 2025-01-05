#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class OccupancyMapNode : public rclcpp::Node
{
public:
    OccupancyMapNode() : Node("occupancy_map_node")
    {
        // OccupancyGrid publisher
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_map", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&OccupancyMapNode::publish_map, this));

        // Initialize TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publish_static_transform();

        RCLCPP_INFO(this->get_logger(), "Occupancy Map Node has been started.");
    }

private:
    void publish_map()
    {
        auto message = nav_msgs::msg::OccupancyGrid();

        // Set up the metadata
        message.header.stamp = this->now();  // Add timestamp
        message.header.frame_id = "map";     // Set the frame ID
        message.info.resolution = 1.0;       // 1 meter per cell
        message.info.width = 30;
        message.info.height = 30;
        message.info.origin.position.x = 0.0;
        message.info.origin.position.y = 0.0;
        message.info.origin.position.z = 0.0;
        message.info.origin.orientation.w = 1.0;

        // Fill the grid with default values (-1 for unknown)
        message.data.resize(message.info.width * message.info.height, 0); // 0 --> empty cell

        // For demonstration, set some cells as occupied
        for (int i = 10; i < 20; ++i)
        {
            for (int j = 10; j < 20; ++j)
            {
                message.data[i * message.info.width + j] = 100; // 100 means occupied
            }
        }

        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published occupancy map.");
    }

    void publish_static_transform()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "map";         // Parent frame
        transform.child_frame_id = "base_link";    // Child frame

        // Set translation (robot's position in the map frame)
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;

        // Set rotation (no rotation for simplicity)
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform);
        RCLCPP_INFO(this->get_logger(), "Published static transform: map -> base_link.");
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyMapNode>());
    rclcpp::shutdown();
    return 0;
}
