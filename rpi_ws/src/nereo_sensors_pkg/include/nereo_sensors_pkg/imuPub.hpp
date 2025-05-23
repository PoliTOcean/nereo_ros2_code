#ifndef IMU_PUB_H
#define IMU_PUB_H

#include <chrono>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/quaternion.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nereo_sensors_pkg/qos_profiles.hpp"
#include "nereo_sensors_pkg/imuPub.hpp"

#define MAXN 20
#define WT61P_IIC_ADDR 0x50

char *i2c_device = "/dev/i2c-1";

typedef double float64;

struct Vec3 {
    float x;
    float y;
    float z;
};

struct CovarianceMatrix{
    float64 matrix[9];
};


enum Status {OK, WARN, ERROR, STALE};

void calcCovMatrix(std::queue<Vec3> window, float64 *matrix);

class PublisherIMU: public rclcpp::Node
{
    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr imu_diagnostic_publisher_;

        /* Status indicators
        bool imu_acc_error = false;
        bool imu_angle_error = false;
        bool imu_ang_vel_error = false;*/
        Status communication_state = OK;

        std::queue<Vec3> acceleration_window;
        std::queue<Vec3> angular_velocity_window;
        std::queue<Vec3> angles_window;

        // We will use a single covariance matrix for all the data and then copy it to the message
        CovarianceMatrix matrix;

        void timer_callback();

    public:
        PublisherIMU();
};

#endif
