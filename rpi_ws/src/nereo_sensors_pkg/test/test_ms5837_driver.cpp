/**
 * @file      test_ms5837_driver.cpp
 * @brief     datasheet-example regression test of ms5837 type detection
 *            and pressure/temperature compensation
 * @author    Davide Colabella
 *
 * Runs the real driver_ms5837.c / driver_ms5837_basic.c against the
 * fake i2c bus (fake_ms5837_interface.c) with the two worked examples
 * from the TE MS5837-30BA / MS5837-02BA "pressure and temperature
 * calculation" datasheet section, on a simulated bus -- no i2c hardware
 * is touched. Before the DEPTH-05 fix these cases read 199.99 mbar and
 * 22000.50 mbar (the driver forced through the wrong compensation
 * branch); this pins the corrected values.
 */

#include <gtest/gtest.h>

#include "nereo_sensors_pkg/barometer_libs/driver_ms5837_basic.h"

extern "C" void fake_ms5837_set(const uint16_t prom[7], uint32_t d1,
                                 uint32_t d2);

TEST(Ms5837Driver, DetectsThirtyBarFromPromDespiteWrongExpectation)
{
    /* TE MS5837-30BA datasheet worked example: C1..C6 = 34982, 36352,
     * 20328, 22354, 26646, 26146; D1 = 4958179; D2 = 6815414; expected
     * pressure 3999.8 mbar, temperature 19.81 C (this driver's integer
     * truncation yields 19.82). PROM word 0 = 0x9340 (30BA26, valid CRC). */
    const uint16_t prom[7] = {0x9340, 34982, 36352, 20328,
                               22354, 26646, 26146};
    ms5837_type_t detected;
    float temperature_c;
    float pressure_mbar;

    fake_ms5837_set(prom, 4958179U, 6815414U);

    ASSERT_EQ(0, ms5837_basic_init(MS5837_TYPE_02BA21));
    ASSERT_EQ(0, ms5837_basic_get_type(&detected));
    EXPECT_EQ(MS5837_TYPE_30BA26, detected);

    ASSERT_EQ(0, ms5837_basic_read(&temperature_c, &pressure_mbar));
    EXPECT_NEAR(3999.8f, pressure_mbar, 0.05f);
    EXPECT_NEAR(19.81f, temperature_c, 0.02f);

    ms5837_basic_deinit();
}

TEST(Ms5837Driver, DetectsTwoBarFromPromDespiteWrongExpectation)
{
    /* TE MS5837-02BA datasheet worked example: C1..C6 = 46372, 43981,
     * 29059, 27842, 31553, 28165; D1 = 6465444; D2 = 8077636; expected
     * pressure 1100.02 mbar, temperature 20.00 C. PROM word 0 = 0x52A0
     * (02BA21, valid CRC). */
    const uint16_t prom[7] = {0x52A0, 46372, 43981, 29059,
                               27842, 31553, 28165};
    ms5837_type_t detected;
    float temperature_c;
    float pressure_mbar;

    fake_ms5837_set(prom, 6465444U, 8077636U);

    ASSERT_EQ(0, ms5837_basic_init(MS5837_TYPE_30BA26));
    ASSERT_EQ(0, ms5837_basic_get_type(&detected));
    EXPECT_EQ(MS5837_TYPE_02BA21, detected);

    ASSERT_EQ(0, ms5837_basic_read(&temperature_c, &pressure_mbar));
    EXPECT_NEAR(1100.02f, pressure_mbar, 0.01f);
    EXPECT_NEAR(20.00f, temperature_c, 0.01f);

    ms5837_basic_deinit();
}
