#ifndef BAR_PUB_H
#define BAR_PUB_H

#include <chrono>
#include <memory>
#include <string>
#include <stdint.h>
#include <stdbool.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "nereo_sensors_pkg/qos_profiles.hpp"
#include "nereo_sensors_pkg/barometer_libs/driver_ms5837.h"
#include "nereo_sensors_pkg/barometer_libs/driver_ms5837_basic.h"
#include "nereo_sensors_pkg/barometer_libs/driver_ms5837_interface.h"
#include "nereo_sensors_pkg/barometer_libs/iic.h"

// STATUS VALUES FOR DIAGNOSTIC
enum Status {OK, WARN, ERROR, STALE};

class PublisherBAR: public rclcpp::Node
{
    private:
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperature_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_salt_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_fresh_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_reference_srv_;

        void timer_callback();
        void reset_reference_callback(
            const std_srvs::srv::Trigger::Request::SharedPtr,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        std_msgs::msg::Float32 temperature_message = std_msgs::msg::Float32();
        sensor_msgs::msg::FluidPressure pressure_message = sensor_msgs::msg::FluidPressure();
        std_msgs::msg::Float32 depth_salt_message = std_msgs::msg::Float32();
        std_msgs::msg::Float32 depth_fresh_message = std_msgs::msg::Float32();
        diagnostic_msgs::msg::DiagnosticArray diagnostic_message = diagnostic_msgs::msg::DiagnosticArray();

        float reference_pressure_pa_ = 0.0f;  // pressure at ROV startup [Pa]
    public:
        PublisherBAR();
};

#endif
