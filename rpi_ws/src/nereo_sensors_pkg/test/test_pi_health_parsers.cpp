/**
 * @file      test_pi_health_parsers.cpp
 * @brief     hand-computed fixture tests for pi_health.hpp
 * @author    Davide Colabella
 *
 * Every expected value below is hand-computed in the comment right
 * above the assertion that checks it, following
 * test_depth_conversion.cpp's convention.
 */

#include <gtest/gtest.h>

#include "nereo_sensors_pkg/pi_health.hpp"

// ── parse_proc_stat_cpu / cpu_load_percent ──────────────────────────────

TEST(ParseProcStatCpu, AggregateLineFiftyPercentLoad)
{
    CpuJiffies prev{};
    CpuJiffies cur{};
    ASSERT_TRUE(parse_proc_stat_cpu(
        "cpu  100 0 100 800 0 0 0 0 0 0", &prev));
    ASSERT_TRUE(parse_proc_stat_cpu(
        "cpu  150 0 150 900 0 0 0 0 0 0", &cur));

    // prev: total = 100+0+100+800 = 1000, idle = 800
    // cur:  total = 150+0+150+900 = 1200, idle = 900
    // dtotal = 200, didle = 100
    // load = 100 * (200 - 100) / 200 = 50.0
    double percent = 0.0;
    ASSERT_TRUE(cpu_load_percent(prev, cur, &percent));
    EXPECT_NEAR(50.0, percent, 1e-6);
}

TEST(ParseProcStatCpu, PerCoreLineIsRejected)
{
    CpuJiffies out{};
    EXPECT_FALSE(parse_proc_stat_cpu("cpu0 100 0 100 800", &out));
}

TEST(ParseProcStatCpu, BareLabelIsRejected)
{
    CpuJiffies out{};
    EXPECT_FALSE(parse_proc_stat_cpu("cpu", &out));
}

TEST(ParseProcStatCpu, NonNumericFieldIsRejected)
{
    CpuJiffies out{};
    EXPECT_FALSE(parse_proc_stat_cpu("cpu 100 abc 100 800", &out));
}

TEST(CpuLoadPercent, EqualTotalIsRejected)
{
    CpuJiffies prev{1000, 800};
    CpuJiffies cur{1000, 800};
    double percent = 0.0;
    EXPECT_FALSE(cpu_load_percent(prev, cur, &percent));
}

// ── parse_millidegrees ───────────────────────────────────────────────

TEST(ParseMillidegrees, TrailingNewlineIsTrimmed)
{
    // 48312 / 1000 = 48.312
    double celsius = 0.0;
    ASSERT_TRUE(parse_millidegrees("48312\n", &celsius));
    EXPECT_NEAR(48.312, celsius, 1e-9);
}

TEST(ParseMillidegrees, EmptyStringIsRejected)
{
    double celsius = 0.0;
    EXPECT_FALSE(parse_millidegrees("", &celsius));
}

TEST(ParseMillidegrees, NonNumericIsRejected)
{
    double celsius = 0.0;
    EXPECT_FALSE(parse_millidegrees("abc", &celsius));
}

// ── disk_used_percent ────────────────────────────────────────────────

TEST(DiskUsedPercent, TypicalUsage)
{
    // used = 1000 - 400 = 600; 100 * 600 / (600 + 350) = 63.15789...
    double percent = 0.0;
    ASSERT_TRUE(disk_used_percent(1000, 400, 350, &percent));
    EXPECT_NEAR(63.158, percent, 1e-3);
}

TEST(DiskUsedPercent, ZeroBlocksIsRejected)
{
    double percent = 0.0;
    EXPECT_FALSE(disk_used_percent(0, 0, 0, &percent));
}

TEST(DiskUsedPercent, BfreeGreaterThanBlocksIsRejected)
{
    double percent = 0.0;
    EXPECT_FALSE(disk_used_percent(100, 200, 0, &percent));
}

// ── parse_throttled / decode_throttled ──────────────────────────────

TEST(ParseThrottled, PrefixedHexIsAccepted)
{
    uint32_t bits = 0;
    ASSERT_TRUE(parse_throttled("throttled=0x50005\n", &bits));
    EXPECT_EQ(0x50005u, bits);
}

TEST(ParseThrottled, ZeroXPrefixedHexWithNoLabelIsAccepted)
{
    uint32_t bits = 0;
    ASSERT_TRUE(parse_throttled("0x50005", &bits));
    EXPECT_EQ(0x50005u, bits);
}

TEST(ParseThrottled, BareHexIsAccepted)
{
    // The Pi's own sysfs get_throttled file has no "0x" and no
    // "throttled=" label at all -- just the hex digits plus a newline.
    uint32_t bits = 0;
    ASSERT_TRUE(parse_throttled("50005\n", &bits));
    EXPECT_EQ(0x50005u, bits);
}

TEST(ParseThrottled, EmptyAfterLabelIsRejected)
{
    uint32_t bits = 0;
    EXPECT_FALSE(parse_throttled("throttled=", &bits));
}

TEST(ParseThrottled, NonHexIsRejected)
{
    uint32_t bits = 0;
    EXPECT_FALSE(parse_throttled("zz", &bits));
}

TEST(DecodeThrottled, UnderVoltageAndThrottledNowAndSinceBoot)
{
    // 0x50005 = 327685 decimal = bits 0, 2, 16 and 18 set:
    //   under_voltage_now (0), throttled_now (2),
    //   under_voltage_since_boot (16), throttled_since_boot (18) --
    //   and no other flag.
    ThrottleFlags flags = decode_throttled(0x50005);
    EXPECT_TRUE(flags.under_voltage_now);
    EXPECT_FALSE(flags.freq_capped_now);
    EXPECT_TRUE(flags.throttled_now);
    EXPECT_FALSE(flags.soft_temp_limit_now);
    EXPECT_TRUE(flags.under_voltage_since_boot);
    EXPECT_FALSE(flags.freq_capped_since_boot);
    EXPECT_TRUE(flags.throttled_since_boot);
    EXPECT_FALSE(flags.soft_temp_limit_since_boot);
}

TEST(DecodeThrottled, ZeroSetsNothing)
{
    ThrottleFlags flags = decode_throttled(0);
    EXPECT_FALSE(flags.under_voltage_now);
    EXPECT_FALSE(flags.freq_capped_now);
    EXPECT_FALSE(flags.throttled_now);
    EXPECT_FALSE(flags.soft_temp_limit_now);
    EXPECT_FALSE(flags.under_voltage_since_boot);
    EXPECT_FALSE(flags.freq_capped_since_boot);
    EXPECT_FALSE(flags.throttled_since_boot);
    EXPECT_FALSE(flags.soft_temp_limit_since_boot);
}

// ── is_agent_comm ────────────────────────────────────────────────────

TEST(IsAgentComm, ExactNamePlusNewlineIsAccepted)
{
    EXPECT_TRUE(is_agent_comm("micro_ros_agent\n"));
}

TEST(IsAgentComm, OneCharacterShorterIsRejected)
{
    EXPECT_FALSE(is_agent_comm("micro_ros_agen\n"));
}

TEST(IsAgentComm, LongerIsRejected)
{
    EXPECT_FALSE(is_agent_comm("micro_ros_agentx\n"));
}
