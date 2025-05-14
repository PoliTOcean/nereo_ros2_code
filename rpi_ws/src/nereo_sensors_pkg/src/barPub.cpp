#include "nereo_sensors_pkg/barPub.hpp"
using namespace std::chrono_literals;

// BAROMETER DATA VARIABLES
float temperature_celsius;       // Temperature in Celsius
float pressure_mbar;       // Pressure in mbar
float pressure_zero;       // Pressure at sea level
int res;                   // Result of barometer acquisition
int has_error = 0;     // Flag for error in barometer acquisition

//=========================================================================================================

// MAIN FUNCTION
int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherBAR>());
    rclcpp::shutdown();
    return 0;
}

// NODE CLASS 
void PublisherBAR::timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "Entered timer callback!");

    // ACQUISITION
    has_error = ms5837_basic_read(&temperature_celsius, &pressure_mbar);

    RCLCPP_INFO(this->get_logger(), "Data acquired!");

    // TEMPERATURE
    temperature_message.temperature = temperature_celsius;
    temperature_message.variance = 0;                                           // 0: Unknown variance
    temperature_message.header.stamp = this->get_clock()->now();                // Time data type from builtin_interfaces (???)
    temperature_message.header.frame_id = "Frame ID Barometer - Temperature";
    RCLCPP_INFO(this->get_logger(), "Temperature message filled!");
    
    // PRESSURE
    pressure_message.fluid_pressure = pressure_mbar;
    pressure_message.variance = 0;                                          // 0: Unknown variance
    pressure_message.header.stamp = this->get_clock()->now();
    pressure_message.header.frame_id = "Frame ID Barometer - Pressure";
    RCLCPP_INFO(this->get_logger(), "Pressure message filled!");

    diagnostic_message.header.stamp = this->get_clock()->now();
    diagnostic_message.header.frame_id = "Barometer Diagnostic";

    // DIAGNOSTIC DURING ACQUISITION
    if (has_error)
    {
        diagnostic_msgs::msg::DiagnosticStatus diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic_status.level = ERROR;
        diagnostic_status.name = "Barometer acqiuisition error";
        diagnostic_status.message = "Error while acquiring data from barometer";
        diagnostic_message.status.push_back(diagnostic_status);
        RCLCPP_INFO(this->get_logger(), "Error while acquiring data from barometer.");
    }
    else {
        diagnostic_msgs::msg::DiagnosticStatus diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic_status.level = OK;
        diagnostic_status.name = "Barometer acquisition";
        diagnostic_status.message = "Data acquired correctly";
        diagnostic_message.status.push_back(diagnostic_status);
        RCLCPP_INFO(this->get_logger(), "Data acquired correctly.");
    }

    // PUBLISHING
    RCLCPP_INFO(this->get_logger(), "Ready to publish!");
    temperature_publisher_->publish(temperature_message);
    pressure_publisher_->publish(pressure_message);
    diagnostic_publisher_->publish(diagnostic_message);
    RCLCPP_INFO(this->get_logger(), "Published!");
}

PublisherBAR::PublisherBAR(): Node("bar_publisher")
{
    temperature_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("barometer_temperature", getSensorQoS());
    pressure_publisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("barometer_pressure", getSensorQoS());
    diagnostic_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("barometer_diagnostic", getReliableQoS());
    RCLCPP_INFO(this->get_logger(), "Publishers created");
    
    timer_ = this->create_wall_timer(300ms, std::bind(&PublisherBAR::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Timer created");

    res = ms5837_basic_init(MS5837_TYPE_30BA26);

    // DIAGNOSTIC DURING INITIALIZATION
    if (res != 0){
        RCLCPP_INFO(this->get_logger(), "Error while initializing barometer");
        diagnostic_msgs::msg::DiagnosticStatus diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic_status.level = ERROR;
        diagnostic_status.name = "Barometer initialization error";
        diagnostic_status.message = "Error while initializing barometer";
        diagnostic_message.status.push_back(diagnostic_status);
        RCLCPP_INFO(this->get_logger(), "Barometer initialization error");
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Barometer initialized");
        diagnostic_msgs::msg::DiagnosticStatus diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic_status.level = OK;
        diagnostic_status.name = "Barometer initialization";
        diagnostic_status.message = "Barometer initialized correctly";
        diagnostic_message.status.push_back(diagnostic_status);
        RCLCPP_INFO(this->get_logger(), "Barometer initialized correctly");
    }
}
    // depth = (pressure_mbar-pressure_zero)*100/(9.80665*997.0f);  //997 = density fresh water [FORMULA DA RIVEDERE]
