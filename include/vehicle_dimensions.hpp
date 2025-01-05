#ifndef VEHICLE_DIMENSIONS_HPP
#define VEHICLE_DIMENSIONS_HPP

#include "rclcpp/rclcpp.hpp"
#include <string>

struct BaseDimensions { // taban boyutlarını temsil edem yapı
    double length;
    double width;
    double height;
};

struct WheelPosition {  // tekerlek poz. temsil eden yapı
    double x;
    double y;
    double z;
};

struct WheelSpecs { // tekerlek özellikleri yapısı
    double radius;
    double thickness;
    WheelPosition front_left;
    WheelPosition front_right;
    WheelPosition rear_left;
    WheelPosition rear_right;
};

struct CameraSpecs {
    double height;
    double forward_offset;
};

struct LidarSpecs {
    double height;
    double forward_offset;
};

class VehicleDimensions { // araç boyutları sınıfı
public:
    VehicleDimensions(rclcpp::Node *node) {
        // Bu yapılandırıcı (constructor), VehicleDimensions sınıfı nesnesi oluşturulduğunda çağrılır
        // Amacı,ROS 2 düğümünden (node) parametreleri alarak robotun fiziksel özelliklerini ayarlamaktır.
        
        base_.length = node->declare_parameter<double>("robot_dimensions.base.length", 1.0);
        base_.width = node->declare_parameter<double>("robot_dimensions.base.width", 0.6);
        base_.height = node->declare_parameter<double>("robot_dimensions.base.height", 0.4);

        // Wheel Specs
        wheels_.radius = node->declare_parameter<double>("robot_dimensions.wheels.radius", 0.1);
        wheels_.thickness = node->declare_parameter<double>("robot_dimensions.wheels.thickness", 0.04);

        wheels_.front_left = loadWheelPosition(node, "robot_dimensions.wheels.positions.front_left");
        wheels_.front_right = loadWheelPosition(node, "robot_dimensions.wheels.positions.front_right");
        wheels_.rear_left = loadWheelPosition(node, "robot_dimensions.wheels.positions.rear_left");
        wheels_.rear_right = loadWheelPosition(node, "robot_dimensions.wheels.positions.rear_right");

        // Additional Specs
        wheelbase_ = node->declare_parameter<double>("robot_dimensions.wheelbase", 1.0);
        track_width_ = node->declare_parameter<double>("robot_dimensions.track_width", 0.6);

        camera_.height = node->declare_parameter<double>("robot_dimensions.camera.height", 0.6);
        camera_.forward_offset = node->declare_parameter<double>("robot_dimensions.camera.forward_offset", 0.5);

        lidar_.height = node->declare_parameter<double>("robot_dimensions.lidar.height", 0.5);
        lidar_.forward_offset = node->declare_parameter<double>("robot_dimensions.lidar.forward_offset", 0.0);
    }

    BaseDimensions getBaseDimensions() const { return base_; }
    WheelSpecs getWheelSpecs() const { return wheels_; }
    double getWheelbase() const { return wheelbase_; }
    double getTrackWidth() const { return track_width_; }
    CameraSpecs getCameraSpecs() const { return camera_; }
    LidarSpecs getLidarSpecs() const { return lidar_; }

private:
    BaseDimensions base_;
    WheelSpecs wheels_;
    double wheelbase_;
    double track_width_;
    CameraSpecs camera_;
    LidarSpecs lidar_;

    WheelPosition loadWheelPosition(rclcpp::Node *node, const std::string &base_name) {
        WheelPosition pos;
        pos.x = node->declare_parameter<double>(base_name + ".x", 0.0);
        pos.y = node->declare_parameter<double>(base_name + ".y", 0.0);
        pos.z = node->declare_parameter<double>(base_name + ".z", 0.0);
        return pos;
    }
};

#endif // VEHICLE_DIMENSIONS_HPP
