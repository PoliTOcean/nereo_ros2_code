/**
 * @file      piHealthPub.cpp
 * @brief     publishes the Raspberry Pi's own health as a DiagnosticArray
 * @author    Davide Colabella
 *
 * Node `pi_health_publisher` publishes `pi_health`
 * (diagnostic_msgs/msg/DiagnosticArray) at about 1 Hz, best-effort via
 * getSensorQoS(). One DiagnosticStatus per signal, KeyValue pairs
 * carrying the wire contract below. A failed read or parse sets that
 * key's value to PI_HEALTH_UNAVAILABLE and the status level to ERROR;
 * every other key keeps publishing on schedule (T-06-11).
 *
 * Key contract (KeyValue keys on `pi_health`):
 *   cpu_load_percent   -- CPU load, %, filled here
 *   soc_temp_c         -- SoC temperature, degrees C (06-03 Task 2)
 *   disk_used_percent  -- root filesystem use, % (06-03 Task 2)
 *   under_voltage_now, freq_capped_now, throttled_now,
 *   soft_temp_limit_now, under_voltage_since_boot,
 *   freq_capped_since_boot, throttled_since_boot,
 *   soft_temp_limit_since_boot, throttled_raw, agent_running
 *                      -- filled in 06-04
 */

#include <cstdio>
#include <fstream>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"

#include "nereo_sensors_pkg/pi_health.hpp"
#include "nereo_sensors_pkg/qos_profiles.hpp"

using namespace std::chrono_literals;

class PublisherPiHealth : public rclcpp::Node
{
public:
    PublisherPiHealth() : Node("pi_health_publisher")
    {
        health_publisher_ =
            this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                "pi_health", getSensorQoS());

        // Prime the first sample so the very first published message
        // already carries a real load, not just PI_HEALTH_UNAVAILABLE.
        std::ifstream stat_file("/proc/stat");
        std::string line;
        if (stat_file.is_open() && std::getline(stat_file, line)) {
            has_prev_cpu_ = parse_proc_stat_cpu(line, &prev_cpu_);
        }

        timer_ = this->create_wall_timer(
            1s, std::bind(&PublisherPiHealth::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto array = diagnostic_msgs::msg::DiagnosticArray();
        array.header.stamp = this->get_clock()->now();
        array.header.frame_id = "pi";

        array.status.push_back(build_cpu_status());

        health_publisher_->publish(array);
    }

    diagnostic_msgs::msg::DiagnosticStatus build_cpu_status()
    {
        auto status = diagnostic_msgs::msg::DiagnosticStatus();
        status.name = "Pi CPU";
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "cpu_load_percent";

        std::ifstream stat_file("/proc/stat");
        std::string line;
        CpuJiffies cur{};
        bool parsed = stat_file.is_open() &&
                      std::getline(stat_file, line) &&
                      parse_proc_stat_cpu(line, &cur);

        double percent = 0.0;
        bool ok = parsed && has_prev_cpu_ &&
                  cpu_load_percent(prev_cpu_, cur, &percent);

        if (ok) {
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%.1f", percent);
            kv.value = buf;
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Pi CPU load read correctly";
        } else {
            kv.value = PI_HEALTH_UNAVAILABLE;
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "Failed to read /proc/stat";
            RCLCPP_WARN(this->get_logger(),
                        "Failed to read CPU load from /proc/stat");
        }
        // Even a failed cycle keeps prev_cpu_ at its last good value,
        // so the next successful read still yields a real delta.
        if (parsed) {
            prev_cpu_ = cur;
            has_prev_cpu_ = true;
        }

        status.values.push_back(kv);
        return status;
    }

    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        health_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    CpuJiffies prev_cpu_{};
    bool has_prev_cpu_ = false;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherPiHealth>());
    rclcpp::shutdown();
    return 0;
}
