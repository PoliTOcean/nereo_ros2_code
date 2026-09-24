/**
 * @file      test_depth_conversion.cpp
 * @brief     hand-computed reference values for compute_depth_m
 * @author    Davide Colabella
 *
 * Every value below is hand-computed with g = 9.80665 m/s^2 from
 * depth_conversion.hpp's formula:
 *   depth = (pressure_pa - reference_pa) / (density_kg_m3 * g)
 *           + mounting_offset_m
 */

#include <gtest/gtest.h>

#include "nereo_sensors_pkg/depth_conversion.hpp"

TEST(ComputeDepthM, TareReadsZero)
{
    // (101325 - 101325) / (1025 * 9.80665) + 0 = 0.0
    EXPECT_NEAR(0.0f, compute_depth_m(101325.0f, 101325.0f, 1025.0f, 0.0f),
                1e-4f);
}

TEST(ComputeDepthM, OneMetreOfSaltWater)
{
    // (111376.81625 - 101325) / (1025 * 9.80665) + 0 = 1.0
    EXPECT_NEAR(1.0f,
                compute_depth_m(111376.81625f, 101325.0f, 1025.0f, 0.0f),
                1e-4f);
}

TEST(ComputeDepthM, TenMetresOfFreshWater)
{
    // (199097.3005 - 101325) / (997 * 9.80665) + 0 = 10.0
    EXPECT_NEAR(10.0f,
                compute_depth_m(199097.3005f, 101325.0f, 997.0f, 0.0f),
                1e-3f);
}

TEST(ComputeDepthM, DensityIsAnInput)
{
    // same delta as OneMetreOfSaltWater, at 1000 kg/m3 instead of 1025
    // (111376.81625 - 101325) / (1000 * 9.80665) + 0 = 1.025
    EXPECT_NEAR(1.025f,
                compute_depth_m(111376.81625f, 101325.0f, 1000.0f, 0.0f),
                1e-4f);
}

TEST(ComputeDepthM, AboveTareReadsNegative)
{
    // (96299.091875 - 101325) / (1025 * 9.80665) + 0 = -0.5
    EXPECT_NEAR(-0.5f,
                compute_depth_m(96299.091875f, 101325.0f, 1025.0f, 0.0f),
                1e-4f);
}

TEST(ComputeDepthM, MountingOffsetAppliedOnce)
{
    // tare + 0.15 m offset = 0.15 m
    EXPECT_NEAR(0.15f,
                compute_depth_m(101325.0f, 101325.0f, 1025.0f, 0.15f),
                1e-4f);
    // 1 m of salt water + 0.15 m offset = 1.15 m
    EXPECT_NEAR(1.15f,
                compute_depth_m(111376.81625f, 101325.0f, 1025.0f, 0.15f),
                1e-4f);
}

TEST(ComputeDepthM, Ms5837ThirtyBarFullScale)
{
    // MS5837-30BA full scale, 30 bar absolute = 3000000 Pa
    // (3000000 - 101325) / (1025 * 9.80665) + 0 = 288.373
    EXPECT_NEAR(288.373f,
                compute_depth_m(3000000.0f, 101325.0f, 1025.0f, 0.0f),
                0.01f);
}
