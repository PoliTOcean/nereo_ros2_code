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
    temperature_message.data = temperature_celsius;

    // PRESSURE — ms5837_basic_read returns mbar (for 02BA21), FluidPressure expects Pa
    float pressure_pa = pressure_mbar * 100.0f;
    pressure_message.fluid_pressure = pressure_pa;
    pressure_message.variance = 0;
    pressure_message.header.stamp = this->get_clock()->now();
    pressure_message.header.frame_id = "barometer";

    // DEPTH — relative to the pressure at startup (positive downward, in metres)
    constexpr float G = 9.80665f;
    constexpr float RHO_SALT  = 1025.0f;  // kg/m³ salt water
    constexpr float RHO_FRESH = 1000.0f;  // kg/m³ fresh water
    float delta_pa = pressure_pa - reference_pressure_pa_;
    depth_salt_message.data  = delta_pa / (RHO_SALT  * G);
    depth_fresh_message.data = delta_pa / (RHO_FRESH * G);

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
    depth_salt_publisher_->publish(depth_salt_message);
    depth_fresh_publisher_->publish(depth_fresh_message);
    diagnostic_publisher_->publish(diagnostic_message);
}

void PublisherBAR::reset_reference_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    float t_dummy, p_mbar;
    if (ms5837_basic_read(&t_dummy, &p_mbar) == 0) {
        reference_pressure_pa_ = p_mbar * 100.0f;
        RCLCPP_INFO(this->get_logger(), "Reference pressure reset to %.2f Pa", reference_pressure_pa_);
        response->success = true;
        response->message = "Reference pressure reset";
    } else {
        response->success = false;
        response->message = "Failed to read barometer";
    }
}

PublisherBAR::PublisherBAR(): Node("bar_publisher")
{
    temperature_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "barometer_temperature", getSensorQoS());
    pressure_publisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>(
        "barometer_pressure", getSensorQoS());
    depth_salt_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "barometer_depth_salt", getSensorQoS());
    depth_fresh_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "barometer_depth_fresh", getSensorQoS());
    diagnostic_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "barometer_diagnostic", getSensorQoS());

    timer_ = this->create_wall_timer(300ms, std::bind(&PublisherBAR::timer_callback, this));

    reset_reference_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "barometer_reset_reference",
        std::bind(&PublisherBAR::reset_reference_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    res = ms5837_basic_init(MS5837_TYPE_02BA21);

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

        // capture surface reference pressure
        float t_dummy;
        if (ms5837_basic_read(&t_dummy, &pressure_mbar) == 0) {
            reference_pressure_pa_ = pressure_mbar * 100.0f;
            RCLCPP_INFO(this->get_logger(), "Reference pressure set to %.2f Pa", reference_pressure_pa_);
        }
    }
    diagnostic_message.status.push_back(diagnostic_status);
}
