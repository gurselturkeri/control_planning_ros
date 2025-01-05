#include "rclcpp/rclcpp.hpp"
#include "vehicle_dimensions.hpp"

class VehicleInfo : public rclcpp::Node {
public:
    VehicleInfo() : Node("vehicle_info") {
        VehicleDimensions vehicle(this);

        BaseDimensions base = vehicle.getBaseDimensions();
        RCLCPP_INFO(this->get_logger(), "Base Dimensions: length=%.2f, width=%.2f, height=%.2f",
                    base.length, base.width, base.height);

        WheelSpecs wheels = vehicle.getWheelSpecs();
        RCLCPP_INFO(this->get_logger(), "Wheel Specs: radius=%.2f, thickness=%.2f",
                    wheels.radius, wheels.thickness);

        RCLCPP_INFO(this->get_logger(), "Front Left Wheel Position: x=%.2f, y=%.2f, z=%.2f",
                    wheels.front_left.x, wheels.front_left.y, wheels.front_left.z);

        RCLCPP_INFO(this->get_logger(), "Wheelbase: %.2f, Track Width: %.2f",
                    vehicle.getWheelbase(), vehicle.getTrackWidth());

        CameraSpecs camera = vehicle.getCameraSpecs();
        RCLCPP_INFO(this->get_logger(), "Camera: height=%.2f, forward_offset=%.2f",
                    camera.height, camera.forward_offset);

        LidarSpecs lidar = vehicle.getLidarSpecs();
        RCLCPP_INFO(this->get_logger(), "Lidar: height=%.2f, forward_offset=%.2f",
                    lidar.height, lidar.forward_offset);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleInfo>());
    rclcpp::shutdown();
    return 0;
}
