#include <cmath>
#include <string>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

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

// Human-readable label for the PROM-detected MS5837 variant, used in the
// startup log and exported as the ms5837_type diagnostic key.
static std::string ms5837TypeName(ms5837_type_t type)
{
    switch (type) {
        case MS5837_TYPE_02BA01: return "02BA01";
        case MS5837_TYPE_02BA21: return "02BA21";
        case MS5837_TYPE_30BA26: return "30BA26";
        default: return "unknown";
    }
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

    // DIAGNOSTIC — clear previous cycle status before filling
    diagnostic_message.status.clear();
    diagnostic_message.header.stamp = this->get_clock()->now();
    diagnostic_message.header.frame_id = "barometer";

    auto diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
    diagnostic_msgs::msg::KeyValue type_kv;
    type_kv.key = "ms5837_type";
    type_kv.value = ms5837_type_name_;
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
    diagnostic_status.values.push_back(type_kv);
    diagnostic_message.status.push_back(diagnostic_status);

    /*
     * The diagnostic always goes out -- it is how a dead sensor is
     * reported. The four data topics do not: on a failed read
     * ms5837_basic_read leaves the globals holding the previous
     * sample, and publishing that with a fresh header.stamp tells
     * every consumer a stale value is current.
     *
     * The flight controller judges freshness by message arrival, so
     * publishing through a failure made its freshness contract blind
     * to a dead sensor: it would detect a dead publisher but never a
     * dead barometer. Observed on the bench 2026-09-21 --
     * /pressure_data_valid stayed true with the sensor physically
     * unplugged while barometer_diagnostic already read ERROR.
     * Staying silent is what makes the contract work as designed.
     */
    diagnostic_publisher_->publish(diagnostic_message);

    if (has_error) {
        return;
    }

    // TARE — the first successful read after start (or after a
    // barometer_reset_reference call) becomes the reference; nothing is
    // published against an untared (0 Pa) reference (DEPTH-03).
    if (!has_reference_) {
        reference_pressure_pa_ = pressure_pa;
        has_reference_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Reference pressure set to %.2f Pa",
                    reference_pressure_pa_);
    }

    // DEPTH — the single conversion site is compute_depth_m(); density
    // and the mounting offset are live parameters, read every cycle.
    double water_density = this->get_parameter("water_density").as_double();
    double mounting_offset_m =
        this->get_parameter("mounting_offset_m").as_double();
    depth_message.data = compute_depth_m(
        pressure_pa, reference_pressure_pa_,
        static_cast<float>(water_density),
        static_cast<float>(mounting_offset_m));

    temperature_publisher_->publish(temperature_message);
    pressure_publisher_->publish(pressure_message);
    if (std::isfinite(depth_message.data)) {
        depth_publisher_->publish(depth_message);
    }
}

void PublisherBAR::reset_reference_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    float t_dummy, p_mbar;
    if (ms5837_basic_read(&t_dummy, &p_mbar) == 0) {
        reference_pressure_pa_ = p_mbar * 100.0f;
        has_reference_ = true;
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
    rcl_interfaces::msg::ParameterDescriptor density_descriptor;
    // water density [kg/m^3]: ~997 fresh, ~1025 salt (see
    // depth_conversion.hpp's DEFAULT_WATER_DENSITY_KG_M3 for the
    // salt-water default this parameter starts from). The literal
    // numbers stay out of the runtime description string itself so
    // this file carries no density number (DEPTH-02).
    density_descriptor.description =
        "water density [kg/m^3]: lower for fresh water, higher for "
        "salt water";
    rcl_interfaces::msg::FloatingPointRange density_range;
    density_range.from_value = 990.0;
    density_range.to_value = 1050.0;
    density_range.step = 0;
    density_descriptor.floating_point_range.push_back(density_range);
    this->declare_parameter(
        "water_density", DEFAULT_WATER_DENSITY_KG_M3, density_descriptor);

    rcl_interfaces::msg::ParameterDescriptor offset_descriptor;
    offset_descriptor.description =
        "vertical distance from the pressure port down to the vehicle's "
        "control reference point [m], positive when the reference point "
        "is below the sensor";
    rcl_interfaces::msg::FloatingPointRange offset_range;
    offset_range.from_value = -1.0;
    offset_range.to_value = 1.0;
    offset_range.step = 0;
    offset_descriptor.floating_point_range.push_back(offset_range);
    // default 0.20: sensor port is 20 cm above the ROV bottom, the
    // control reference point (operator, 2026-09-24)
    this->declare_parameter("mounting_offset_m", 0.20, offset_descriptor);

    temperature_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "barometer_temperature", getSensorQoS());
    pressure_publisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>(
        "barometer_pressure", getSensorQoS());
    depth_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "barometer_depth", getSensorQoS());
    diagnostic_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "barometer_diagnostic", getSensorQoS());

    timer_ = this->create_wall_timer(300ms, std::bind(&PublisherBAR::timer_callback, this));

    reset_reference_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "barometer_reset_reference",
        std::bind(&PublisherBAR::reset_reference_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // The vehicle's documented hardware is the MS5837-30BA; passed only
    // as the mismatch-notice expectation -- the chip's own PROM report
    // is what actually drives compensation (DEPTH-05).
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

        ms5837_type_t detected_type;
        if (ms5837_basic_get_type(&detected_type) == 0) {
            ms5837_type_name_ = ms5837TypeName(detected_type);
            RCLCPP_INFO(this->get_logger(),
                        "Barometer initialized, detected variant %s",
                        ms5837_type_name_.c_str());
            if (detected_type != MS5837_TYPE_30BA26) {
                RCLCPP_WARN(this->get_logger(),
                            "Detected MS5837 variant %s does not match the "
                            "documented vehicle hardware (30BA26)",
                            ms5837_type_name_.c_str());
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Barometer initialized");
        }
    }
    diagnostic_message.status.push_back(diagnostic_status);
}
