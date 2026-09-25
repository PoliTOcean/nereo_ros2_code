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

#include <sstream>
#include <string>

/** @brief the literal published for a key whose read or parse failed */
constexpr char PI_HEALTH_UNAVAILABLE[] = "unavailable";

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

#endif  // PI_HEALTH_HPP
