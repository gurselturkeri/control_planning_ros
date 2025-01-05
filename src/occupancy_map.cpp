#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class OccupancyMapNode : public rclcpp::Node
{
public:
    OccupancyMapNode() : Node("occupancy_map_node")
    {
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_map", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&OccupancyMapNode::publish_map, this));

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
        message.data.resize(message.info.width * message.info.height, 0); // 0--> empty cell

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

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyMapNode>());
    rclcpp::shutdown();
    return 0;
}
