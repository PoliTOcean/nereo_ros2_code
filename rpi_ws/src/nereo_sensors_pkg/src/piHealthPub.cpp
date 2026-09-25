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
 * Key contract (KeyValue keys on `pi_health`), in this exact order,
 * matching gui_pkg/freshness.py's THROTTLE_FLAGS tuple:
 *   cpu_load_percent   -- CPU load, %
 *   soc_temp_c         -- SoC temperature, degrees C
 *   disk_used_percent  -- root filesystem use, %
 *   under_voltage_now, freq_capped_now, throttled_now,
 *   soft_temp_limit_now, under_voltage_since_boot,
 *   freq_capped_since_boot, throttled_since_boot,
 *   soft_temp_limit_since_boot, throttled_raw
 *                      -- decoded from THROTTLE_SYSFS_PATH (below)
 *   agent_running      -- "true"/"false", from a /proc scan for
 *                          AGENT_PROCESS_COMM (pi_health.hpp)
 *
 * 06-04 probe findings on this Pi (Ubuntu 24.04 noble, Pi 5): BOTH
 * `vcgencmd get_throttled` (`/usr/bin/vcgencmd`, prints
 * "throttled=0x0\n") and the sysfs file
 * `/sys/devices/platform/soc/soc:firmware/get_throttled` (prints the
 * bare hex "0\n", no label) exist. Per the plan's stated priority, the
 * sysfs file wins -- it needs no fork/exec, unlike popen("vcgencmd
 * ..."). See THROTTLE_SYSFS_PATH below.
 */

#include <sys/statvfs.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"

#include "nereo_sensors_pkg/pi_health.hpp"
#include "nereo_sensors_pkg/qos_profiles.hpp"

using namespace std::chrono_literals;

// 06-04 probe: present on this Pi 5 (Ubuntu 24.04 noble), needs no
// fork/exec unlike popen("vcgencmd get_throttled"). See file header.
constexpr char THROTTLE_SYSFS_PATH[] =
    "/sys/devices/platform/soc/soc:firmware/get_throttled";

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
        array.status.push_back(build_temp_status());
        array.status.push_back(build_disk_status());
        array.status.push_back(build_throttle_status());
        array.status.push_back(build_agent_status());

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

    diagnostic_msgs::msg::DiagnosticStatus build_temp_status()
    {
        auto status = diagnostic_msgs::msg::DiagnosticStatus();
        status.name = "Pi temperature";
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "soc_temp_c";

        std::ifstream temp_file(
            "/sys/class/thermal/thermal_zone0/temp");
        std::string text;
        double celsius = 0.0;
        bool ok = temp_file.is_open() &&
                  std::getline(temp_file, text) &&
                  parse_millidegrees(text, &celsius);

        if (ok) {
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%.1f", celsius);
            kv.value = buf;
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Pi SoC temperature read correctly";
        } else {
            kv.value = PI_HEALTH_UNAVAILABLE;
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message =
                "Failed to read /sys/class/thermal/thermal_zone0/temp";
            RCLCPP_WARN(this->get_logger(),
                        "Failed to read SoC temperature");
        }

        status.values.push_back(kv);
        return status;
    }

    diagnostic_msgs::msg::DiagnosticStatus build_disk_status()
    {
        auto status = diagnostic_msgs::msg::DiagnosticStatus();
        status.name = "Pi disk";
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "disk_used_percent";

        struct statvfs vfs {};
        double percent = 0.0;
        bool ok = statvfs("/", &vfs) == 0 &&
                  disk_used_percent(vfs.f_blocks, vfs.f_bfree,
                                     vfs.f_bavail, &percent);

        if (ok) {
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%.1f", percent);
            kv.value = buf;
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Pi disk use read correctly";
        } else {
            kv.value = PI_HEALTH_UNAVAILABLE;
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "Failed to statvfs(\"/\")";
            RCLCPP_WARN(this->get_logger(),
                        "Failed to read root filesystem use");
        }

        status.values.push_back(kv);
        return status;
    }

    diagnostic_msgs::msg::DiagnosticStatus build_throttle_status()
    {
        auto status = diagnostic_msgs::msg::DiagnosticStatus();
        status.name = "Pi throttling";

        std::ifstream throttle_file(THROTTLE_SYSFS_PATH);
        std::string text;
        uint32_t bits = 0;
        bool ok = throttle_file.is_open() &&
                  std::getline(throttle_file, text) &&
                  parse_throttled(text, &bits);

        auto push = [&status](const char *key, const std::string &value) {
            diagnostic_msgs::msg::KeyValue kv;
            kv.key = key;
            kv.value = value;
            status.values.push_back(kv);
        };

        if (ok) {
            ThrottleFlags flags = decode_throttled(bits);
            push("under_voltage_now",
                 flags.under_voltage_now ? "true" : "false");
            push("freq_capped_now", flags.freq_capped_now ? "true" : "false");
            push("throttled_now", flags.throttled_now ? "true" : "false");
            push("soft_temp_limit_now",
                 flags.soft_temp_limit_now ? "true" : "false");
            push("under_voltage_since_boot",
                 flags.under_voltage_since_boot ? "true" : "false");
            push("freq_capped_since_boot",
                 flags.freq_capped_since_boot ? "true" : "false");
            push("throttled_since_boot",
                 flags.throttled_since_boot ? "true" : "false");
            push("soft_temp_limit_since_boot",
                 flags.soft_temp_limit_since_boot ? "true" : "false");

            char buf[16];
            std::snprintf(buf, sizeof(buf), "0x%X", bits);
            push("throttled_raw", buf);

            bool alert = flags.under_voltage_now || flags.freq_capped_now ||
                         flags.throttled_now || flags.soft_temp_limit_now;
            status.level = alert
                ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                : diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = alert ? "Pi is throttled right now"
                                    : "Pi throttling read correctly";
        } else {
            for (const char *key :
                 {"under_voltage_now", "freq_capped_now", "throttled_now",
                  "soft_temp_limit_now", "under_voltage_since_boot",
                  "freq_capped_since_boot", "throttled_since_boot",
                  "soft_temp_limit_since_boot", "throttled_raw"}) {
                push(key, PI_HEALTH_UNAVAILABLE);
            }
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message =
                std::string("Failed to read ") + THROTTLE_SYSFS_PATH;
            RCLCPP_WARN(this->get_logger(), "Failed to read Pi throttling");
        }

        return status;
    }

    diagnostic_msgs::msg::DiagnosticStatus build_agent_status()
    {
        auto status = diagnostic_msgs::msg::DiagnosticStatus();
        status.name = "micro-ROS agent";
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "agent_running";

        std::error_code ec;
        bool running = false;
        for (const auto &entry :
             std::filesystem::directory_iterator("/proc", ec)) {
            const std::string pid_name = entry.path().filename().string();
            bool all_digits =
                !pid_name.empty() &&
                std::all_of(pid_name.begin(), pid_name.end(),
                            [](unsigned char c) { return std::isdigit(c); });
            if (!all_digits) {
                continue;
            }
            std::ifstream comm_file(entry.path() / "comm");
            std::string comm;
            if (comm_file.is_open() && std::getline(comm_file, comm) &&
                is_agent_comm(comm)) {
                running = true;
                break;
            }
        }

        if (!ec) {
            kv.value = running ? "true" : "false";
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = running ? "micro-ROS agent process found"
                                      : "micro-ROS agent process not found";
        } else {
            kv.value = PI_HEALTH_UNAVAILABLE;
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "Failed to scan /proc for the agent process";
            RCLCPP_WARN(this->get_logger(), "Failed to scan /proc");
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
