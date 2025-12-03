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
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;

        void timer_callback();

        sensor_msgs::msg::Temperature temperature_message = sensor_msgs::msg::Temperature();
        sensor_msgs::msg::FluidPressure pressure_message = sensor_msgs::msg::FluidPressure();
        diagnostic_msgs::msg::DiagnosticArray diagnostic_message = diagnostic_msgs::msg::DiagnosticArray();
    public:
        PublisherBAR();
};

#endif
