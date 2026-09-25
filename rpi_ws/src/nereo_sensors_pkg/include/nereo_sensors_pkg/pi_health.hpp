/**
 * @file      pi_health.hpp
 * @brief     pure, hardware-free parsers behind pi_health_pub
 * @author    Davide Colabella
 *
 * Every function here takes text already read from /proc or /sys (or
 * statvfs() output) and returns a parsed value with no I/O of its own,
 * so each one is exercised by a plain gtest with a hand-computed
 * fixture and no Pi hardware (test/test_pi_health_parsers.cpp).
 */

#ifndef PI_HEALTH_HPP
#define PI_HEALTH_HPP

#include <cstdint>
#include <cstdlib>
#include <sstream>
#include <string>

/** @brief the literal published for a key whose read or parse failed */
constexpr char PI_HEALTH_UNAVAILABLE[] = "unavailable";

/**
 * @brief the micro-ROS agent's OS process name
 *
 * Confirmed against SETUP_RPI/micro_ros_connect.sh's own invocation --
 * `ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev
 * /dev/ttyAMA0` -- so this is the literal second argument to `ros2 run`,
 * not a guess. It is exactly 15 characters, so /proc/<pid>/comm (which
 * truncates at 15) never mangles it.
 */
constexpr char AGENT_PROCESS_COMM[] = "micro_ros_agent";

/** @brief one /proc/stat aggregate CPU sample, in jiffies */
struct CpuJiffies
{
    unsigned long long total;
    unsigned long long idle;
};

/**
 * @brief      parse the aggregate "cpu" line of /proc/stat
 * @param[in]  line is one line of /proc/stat text
 * @param[out] out receives the parsed jiffy totals on success
 * @return     true when the line's first token is exactly "cpu" and it
 *             carries at least the four mandatory fields (user, nice,
 *             system, idle); false otherwise. `out->idle` is idle plus
 *             iowait, and `out->total` sums user, nice, system, idle,
 *             iowait, irq, softirq and steal (the fields beyond the
 *             mandatory four default to 0 when absent)
 */
inline bool parse_proc_stat_cpu(const std::string &line, CpuJiffies *out)
{
    std::istringstream iss(line);
    std::string label;
    if (!(iss >> label) || label != "cpu") {
        return false;
    }

    unsigned long long user = 0, nice = 0, system = 0, idle = 0;
    unsigned long long iowait = 0, irq = 0, softirq = 0, steal = 0;
    if (!(iss >> user >> nice >> system >> idle)) {
        return false;
    }
    iss >> iowait >> irq >> softirq >> steal;

    out->total = user + nice + system + idle + iowait + irq + softirq
                 + steal;
    out->idle = idle + iowait;
    return true;
}

/**
 * @brief      compute CPU load percent between two /proc/stat samples
 * @param[in]  prev is the earlier sample
 * @param[in]  cur is the later sample
 * @param[out] percent receives the load, 0-100, on success
 * @return     false when `cur.total` is not greater than `prev.total`,
 *             or when the idle-jiffy delta exceeds the total-jiffy
 *             delta; true otherwise, with `*percent` set to
 *             `100 * (dtotal - didle) / dtotal`
 */
inline bool cpu_load_percent(const CpuJiffies &prev, const CpuJiffies &cur,
                              double *percent)
{
    if (cur.total <= prev.total) {
        return false;
    }
    unsigned long long dtotal = cur.total - prev.total;
    unsigned long long didle = cur.idle - prev.idle;
    if (didle > dtotal) {
        return false;
    }
    *percent = 100.0 * static_cast<double>(dtotal - didle)
               / static_cast<double>(dtotal);
    return true;
}

/**
 * @brief      parse the millidegree text from a thermal_zone temp file
 * @param[in]  text is the file's contents, e.g. "48312\n"
 * @param[out] celsius receives the parsed temperature on success
 * @return     true when, after stripping at most one trailing newline,
 *             the whole remaining string is a valid integer; false on
 *             an empty string or any non-numeric trailing content
 */
inline bool parse_millidegrees(const std::string &text, double *celsius)
{
    std::string trimmed = text;
    if (!trimmed.empty() && trimmed.back() == '\n') {
        trimmed.pop_back();
    }
    if (trimmed.empty()) {
        return false;
    }

    char *end = nullptr;
    long millidegrees = std::strtol(trimmed.c_str(), &end, 10);
    if (end != trimmed.c_str() + trimmed.size()) {
        return false;
    }

    *celsius = static_cast<double>(millidegrees) / 1000.0;
    return true;
}

/**
 * @brief      compute root-filesystem use percent from statvfs fields
 * @param[in]  blocks is f_blocks, the total block count
 * @param[in]  bfree is f_bfree, the free block count
 * @param[in]  bavail is f_bavail, the block count available to a
 *             non-privileged user
 * @param[out] percent receives the use percentage on success, matching
 *             `df`'s Use% formula
 * @return     false when `bfree` exceeds `blocks` or `used + bavail` is
 *             0; true otherwise, with `*percent` set to
 *             `100 * used / (used + bavail)`, `used` being
 *             `blocks - bfree`
 */
inline bool disk_used_percent(unsigned long long blocks,
                               unsigned long long bfree,
                               unsigned long long bavail, double *percent)
{
    if (bfree > blocks) {
        return false;
    }
    unsigned long long used = blocks - bfree;
    unsigned long long denominator = used + bavail;
    if (denominator == 0) {
        return false;
    }
    *percent = 100.0 * static_cast<double>(used)
               / static_cast<double>(denominator);
    return true;
}

/**
 * @brief      parse a get_throttled reading into its raw 32-bit value
 * @param[in]  text is the reading. This Pi's own sysfs
 *             get_throttled file has no label and no "0x" at all, just
 *             the bare hex digits plus a trailing newline (e.g. "0\n");
 *             `vcgencmd get_throttled` instead prints
 *             "throttled=0x50005\n". Both forms, and a bare "0x50005"
 *             with no trailing newline, are accepted
 * @param[out] bits receives the parsed value on success
 * @return     true when, after stripping one trailing newline, an
 *             optional "throttled=" prefix and an optional "0x", the
 *             remainder is a non-empty valid hex number consumed in
 *             full; false otherwise (including an empty remainder
 *             after stripping "throttled=")
 */
inline bool parse_throttled(const std::string &text, uint32_t *bits)
{
    std::string trimmed = text;
    if (!trimmed.empty() && trimmed.back() == '\n') {
        trimmed.pop_back();
    }

    const std::string label = "throttled=";
    if (trimmed.compare(0, label.size(), label) == 0) {
        trimmed.erase(0, label.size());
    }
    if (trimmed.compare(0, 2, "0x") == 0) {
        trimmed.erase(0, 2);
    }
    if (trimmed.empty()) {
        return false;
    }

    char *end = nullptr;
    unsigned long value = std::strtoul(trimmed.c_str(), &end, 16);
    if (end != trimmed.c_str() + trimmed.size()) {
        return false;
    }

    *bits = static_cast<uint32_t>(value);
    return true;
}

/** @brief the eight get_throttled flags decode_throttled() fills */
struct ThrottleFlags
{
    bool under_voltage_now = false;
    bool freq_capped_now = false;
    bool throttled_now = false;
    bool soft_temp_limit_now = false;
    bool under_voltage_since_boot = false;
    bool freq_capped_since_boot = false;
    bool throttled_since_boot = false;
    bool soft_temp_limit_since_boot = false;
};

/**
 * @brief      decode a get_throttled raw value into named flags
 * @param[in]  bits is the raw value parse_throttled() produced
 * @return     the eight flags, each true exactly when its bit is set in
 *             `bits`: bits 0-3 are the "now" flags (under-voltage,
 *             frequency capped, throttled, soft temperature limit) and
 *             bits 16-19 are the matching "since boot" flags; every
 *             other bit is ignored
 */
inline ThrottleFlags decode_throttled(uint32_t bits)
{
    ThrottleFlags flags;
    flags.under_voltage_now = (bits & (1u << 0)) != 0;
    flags.freq_capped_now = (bits & (1u << 1)) != 0;
    flags.throttled_now = (bits & (1u << 2)) != 0;
    flags.soft_temp_limit_now = (bits & (1u << 3)) != 0;
    flags.under_voltage_since_boot = (bits & (1u << 16)) != 0;
    flags.freq_capped_since_boot = (bits & (1u << 17)) != 0;
    flags.throttled_since_boot = (bits & (1u << 18)) != 0;
    flags.soft_temp_limit_since_boot = (bits & (1u << 19)) != 0;
    return flags;
}

/**
 * @brief      check whether a /proc/<pid>/comm reading names the agent
 * @param[in]  comm is the file's contents, e.g. "micro_ros_agent\n"
 * @return     true when, after stripping one trailing newline, `comm`
 *             equals AGENT_PROCESS_COMM exactly; false for anything
 *             shorter, longer, or otherwise different
 */
inline bool is_agent_comm(const std::string &comm)
{
    std::string trimmed = comm;
    if (!trimmed.empty() && trimmed.back() == '\n') {
        trimmed.pop_back();
    }
    return trimmed == AGENT_PROCESS_COMM;
}

#endif  // PI_HEALTH_HPP
