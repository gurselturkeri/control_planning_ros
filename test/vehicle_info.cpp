#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include <string>
#include <iostream>

class ControlNode : public rclcpp::Node
{
public:
    ControlNode() : Node("control_node")
    {
        // Load the configuration file path
        std::string config_file = this->declare_parameter<std::string>("config_path", "config/robot_dimensions.yaml");

        // Parse the YAML file
        YAML::Node config = YAML::LoadFile(config_file);

        // Access robot dimensions
        auto base = config["robot_dimensions"]["base"];
        double base_length = base["length"].as<double>();
        double base_width = base["width"].as<double>();
        double base_height = base["height"].as<double>();

        RCLCPP_INFO(this->get_logger(), "Base Dimensions: length=%.2f, width=%.2f, height=%.2f",
                    base_length, base_width, base_height);

        // Access wheel positions
        auto wheels = config["robot_dimensions"]["wheels"]["positions"];
        double front_left_x = wheels["front_left"]["x"].as<double>();
        double front_left_y = wheels["front_left"]["y"].as<double>();
        double front_left_z = wheels["front_left"]["z"].as<double>();

        RCLCPP_INFO(this->get_logger(), "Front Left Wheel Position: x=%.2f, y=%.2f, z=%.2f",
                    front_left_x, front_left_y, front_left_z);

        // Use these values in your control logic...
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
