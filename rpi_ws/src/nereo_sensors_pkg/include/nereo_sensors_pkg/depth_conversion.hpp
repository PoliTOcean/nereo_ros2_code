/**
 * @file      depth_conversion.hpp
 * @brief     the single pressure-to-depth conversion site on the Pi
 * @author    Davide Colabella
 *
 * compute_depth_m() is the only place a pressure reading becomes a depth
 * in metres. No other file on the Raspberry Pi may carry a density
 * number or perform this arithmetic (DEPTH-02).
 */

#ifndef DEPTH_CONVERSION_HPP
#define DEPTH_CONVERSION_HPP

/** @brief standard gravity, m/s^2 (used to convert a pressure delta into
 *         a hydrostatic head) */
constexpr float STANDARD_GRAVITY_M_S2 = 9.80665f;

/** @brief default water density, kg/m^3 -- salt water, matching the
 *         depth the GUI has always shown; pass 997.0 for pool runs
 * <!-- planner-discipline-allow: 1025 --> */
constexpr double DEFAULT_WATER_DENSITY_KG_M3 = 1025.0;
/* <!-- planner-discipline-allow: 997 --> */

/**
 * @brief      convert a pressure reading to a depth in metres
 * @param[in]  pressure_pa is the current pressure, in pascal
 * @param[in]  reference_pa is the tare pressure captured at the surface,
 *             in pascal
 * @param[in]  density_kg_m3 is the water density, in kg/m^3 (an input,
 *             not a constant -- fresh and salt water read differently)
 * @param[in]  mounting_offset_m is the vertical distance from the
 *             pressure port down to the vehicle's control reference
 *             point, in metres, positive when the reference point is
 *             below the sensor; applied here and nowhere else (DEPTH-04)
 * @return     depth in metres at the control reference point (DEPTH-06:
 *             positive downward). A pressure above the tare reads a
 *             positive depth; a pressure below it (the vehicle rising
 *             above the tare level) reads negative
 * @note       ponytail: the offset is not rotated by pitch/roll -- a
 *             level vehicle is assumed. Project it through the attitude
 *             quaternion if the lever arm ever matters at high tilt.
 */
inline float compute_depth_m(float pressure_pa, float reference_pa,
                              float density_kg_m3,
                              float mounting_offset_m)
{
    return (pressure_pa - reference_pa) /
           (density_kg_m3 * STANDARD_GRAVITY_M_S2) +
           mounting_offset_m;
}

#endif  // DEPTH_CONVERSION_HPP
