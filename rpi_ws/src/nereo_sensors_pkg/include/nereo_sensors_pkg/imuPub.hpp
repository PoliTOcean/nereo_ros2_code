#ifndef IMU_PUB_H
#define IMU_PUB_H

#include <chrono>
#include <memory>
#include <queue>
#include <string>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/quaternion.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nereo_sensors_pkg/qos_profiles.hpp"
#include "nereo_sensors_pkg/imu_libs/WT61P.h"

#define MAXN 20
#define WT61P_IIC_ADDR 0x50

// Defined in imuPub.cpp — extern to avoid multiple-definition linker errors
extern char i2c_device[];

typedef double float64;

struct Vec3 {
    float x;
    float y;
    float z;
};

struct CovarianceMatrix {
    float64 matrix[9];
};

enum Status {OK, WARN, ERROR, STALE};

class PublisherIMU: public rclcpp::Node
{
    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_publisher_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr imu_diagnostic_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        bool imu_acc_error    = false;
        bool imu_angle_error  = false;
        bool imu_ang_vel_error = false;

        std::queue<Vec3> acceleration_window;
        std::queue<Vec3> angular_velocity_window;
        std::queue<Vec3> angles_window;

        CovarianceMatrix matrix;

        void timer_callback();
        void push_window(std::queue<Vec3> &window, Vec3 sample);

    public:
        PublisherIMU();
};

#endif
