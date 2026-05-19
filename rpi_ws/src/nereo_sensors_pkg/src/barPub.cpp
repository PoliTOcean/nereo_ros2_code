#include "nereo_sensors_pkg/barPub.hpp"
using namespace std::chrono_literals;

float temperature_celsius;
float pressure_mbar;
int res;
int has_error = 0;

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherBAR>());
    rclcpp::shutdown();
    return 0;
}

void PublisherBAR::timer_callback()
{
    has_error = ms5837_basic_read(&temperature_celsius, &pressure_mbar);

    // TEMPERATURE
    temperature_message.temperature = temperature_celsius;
    temperature_message.variance = 0;
    temperature_message.header.stamp = this->get_clock()->now();
    temperature_message.header.frame_id = "barometer";

    // PRESSURE — ms5837_basic_read returns mbar, FluidPressure expects Pa (1 mbar = 100 Pa)
    pressure_message.fluid_pressure = pressure_mbar * 100.0f;
    pressure_message.variance = 0;
    pressure_message.header.stamp = this->get_clock()->now();
    pressure_message.header.frame_id = "barometer";

    // DIAGNOSTIC — clear previous cycle status before filling
    diagnostic_message.status.clear();
    diagnostic_message.header.stamp = this->get_clock()->now();
    diagnostic_message.header.frame_id = "barometer";

    auto diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
    if (has_error) {
        diagnostic_status.level   = ERROR;
        diagnostic_status.name    = "Barometer acquisition";
        diagnostic_status.message = "Error while acquiring data from barometer";
        RCLCPP_WARN(this->get_logger(), "Barometer acquisition error: %d", has_error);
    } else {
        diagnostic_status.level   = OK;
        diagnostic_status.name    = "Barometer acquisition";
        diagnostic_status.message = "Data acquired correctly";
    }
    diagnostic_message.status.push_back(diagnostic_status);

    temperature_publisher_->publish(temperature_message);
    pressure_publisher_->publish(pressure_message);
    diagnostic_publisher_->publish(diagnostic_message);
}

PublisherBAR::PublisherBAR(): Node("bar_publisher")
{
    temperature_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>(
        "barometer_temperature", getSensorQoS());
    pressure_publisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>(
        "barometer_pressure", getSensorQoS());
    diagnostic_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "barometer_diagnostic", getSensorQoS());

    timer_ = this->create_wall_timer(300ms, std::bind(&PublisherBAR::timer_callback, this));

    res = ms5837_basic_init(MS5837_TYPE_30BA26);

    diagnostic_message.status.clear();
    auto diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
    if (res != 0) {
        diagnostic_status.level   = ERROR;
        diagnostic_status.name    = "Barometer initialization";
        diagnostic_status.message = "Error while initializing barometer";
        RCLCPP_ERROR(this->get_logger(), "Barometer initialization failed: %d", res);
    } else {
        diagnostic_status.level   = OK;
        diagnostic_status.name    = "Barometer initialization";
        diagnostic_status.message = "Barometer initialized correctly";
        RCLCPP_INFO(this->get_logger(), "Barometer initialized");
    }
    diagnostic_message.status.push_back(diagnostic_status);
}
