/**
 * @file      fake_ms5837_interface.c
 * @brief     scripted i2c bus double for the ms5837 driver, no hardware
 * @author    Davide Colabella
 *
 * Implements the six ms5837_interface_* functions the LibDriver core
 * links against (declared in driver_ms5837_interface.h), backed by a
 * caller-scripted PROM plus D1/D2 ADC words instead of a real i2c bus.
 * fake_ms5837_set() loads the script; ms5837_interface_iic_read() then
 * answers PROM reads (registers 0xA0..0xAC) and ADC reads (register
 * 0x00, disambiguated by the last conversion command's high nibble)
 * exactly as the real MS5837 would over i2c.
 */

#include <stdarg.h>
#include <stdio.h>

#include "nereo_sensors_pkg/barometer_libs/driver_ms5837_interface.h"

#define FAKE_MS5837_PROM_WORDS    7U
#define FAKE_MS5837_PROM_REG_LO   0xA0U
#define FAKE_MS5837_PROM_REG_HI   0xACU
#define FAKE_MS5837_ADC_REG       0x00U
#define FAKE_MS5837_CMD_D1_NIBBLE 0x4U
#define FAKE_MS5837_CMD_D2_NIBBLE 0x5U

static uint16_t fake_prom[FAKE_MS5837_PROM_WORDS];
static uint32_t fake_d1;
static uint32_t fake_d2;
static uint8_t fake_last_cmd;

/**
 * @brief     load the scripted prom words and adc conversion results
 * @param[in] prom points to the 7 datasheet-order prom words (word 0
 *            carries the crc and the type code)
 * @param[in] d1 is the scripted raw pressure adc word
 * @param[in] d2 is the scripted raw temperature adc word
 * @note      none
 */
void fake_ms5837_set(const uint16_t prom[7], uint32_t d1, uint32_t d2)
{
    uint8_t i;

    for (i = 0; i < FAKE_MS5837_PROM_WORDS; i++)
    {
        fake_prom[i] = prom[i];
    }
    fake_d1 = d1;
    fake_d2 = d2;
    fake_last_cmd = 0;
}

uint8_t ms5837_interface_iic_init(void)
{
    return 0;
}

uint8_t ms5837_interface_iic_deinit(void)
{
    return 0;
}

uint8_t ms5837_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf,
                                   uint16_t len)
{
    uint32_t word;

    (void)addr;
    if ((reg >= FAKE_MS5837_PROM_REG_LO) && (reg <= FAKE_MS5837_PROM_REG_HI)
        && (len == 2U))
    {
        word = fake_prom[(reg - FAKE_MS5837_PROM_REG_LO) / 2U];
        buf[0] = (uint8_t)((word >> 8) & 0xFFU);
        buf[1] = (uint8_t)(word & 0xFFU);

        return 0;
    }
    if ((reg == FAKE_MS5837_ADC_REG) && (len == 3U))
    {
        if ((fake_last_cmd >> 4) == FAKE_MS5837_CMD_D1_NIBBLE)
        {
            word = fake_d1;
        }
        else if ((fake_last_cmd >> 4) == FAKE_MS5837_CMD_D2_NIBBLE)
        {
            word = fake_d2;
        }
        else
        {
            return 1;
        }
        buf[0] = (uint8_t)((word >> 16) & 0xFFU);
        buf[1] = (uint8_t)((word >> 8) & 0xFFU);
        buf[2] = (uint8_t)(word & 0xFFU);

        return 0;
    }

    return 1;
}

uint8_t ms5837_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf,
                                    uint16_t len)
{
    (void)addr;
    (void)buf;
    (void)len;
    fake_last_cmd = reg;

    return 0;
}

void ms5837_interface_delay_ms(uint32_t ms)
{
    (void)ms;
}

void ms5837_interface_debug_print(const char *const fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    (void)vfprintf(stderr, fmt, args);
    va_end(args);
}
