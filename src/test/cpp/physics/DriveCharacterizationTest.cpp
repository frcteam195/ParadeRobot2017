#include <vector>
#include <iostream>

#include "gtest/gtest.h"
#include "physics/DriveCharacterization.hpp"
#include "utils/CKMath.hpp"

TEST(DriveCharacterizationTest, DetermineOutput) { 
    double kTestEps = 1e-5;
    std::vector<ck::physics::VelocityDataPoint> velocityData;
    std::vector<ck::physics::AccelerationDataPoint> accelData;
    velocityData.push_back(ck::physics::VelocityDataPoint {1, 10});
    velocityData.push_back(ck::physics::VelocityDataPoint {2, 12});
    velocityData.push_back(ck::physics::VelocityDataPoint {3, 15});
    velocityData.push_back(ck::physics::VelocityDataPoint {4, 20});
    
    accelData.push_back(ck::physics::AccelerationDataPoint {1, 10, 20});
    accelData.push_back(ck::physics::AccelerationDataPoint {2, 12, 60});
    accelData.push_back(ck::physics::AccelerationDataPoint {3, 15, 120});
    accelData.push_back(ck::physics::AccelerationDataPoint {4, 20, 400});
    ck::physics::CharacterizationConstants cc = ck::physics::DriveCharacterization::characterizeDrive(velocityData, accelData);
    std::cout << cc << std::endl;
    ASSERT_NEAR(cc.ks, 6, kTestEps);
    ASSERT_NEAR(cc.kv, 3.3, kTestEps);
    ASSERT_NEAR(cc.ka, 0.002149321267, kTestEps);
}

TEST(DriveCharacterizationTest, PlausibleOutput) { 
    double kTestEps = 1e-4;
    double ks = 0.75;
    double kv = 0.2;
    double ka = 0.15;

    std::vector<ck::physics::VelocityDataPoint> velocityData;
    // generate velocity data points
    for (double v = 0; v < 1.0; v += 0.01) {
        velocityData.push_back(ck::physics::VelocityDataPoint{ck::math::max(0.0, (v - ks) / kv), v});
    }

    std::vector<ck::physics::AccelerationDataPoint> accelerationData;
    double v, a;
    v = 0;
    // generate acceleration data points
    for (int i = 0; i < 1000; ++i) {
        a = ck::math::max(0.0, 6.0 - kv * v - ks) / ka;
        v += a * kTestEps;
        accelerationData.push_back(ck::physics::AccelerationDataPoint{v, 6.0, a});
    }

    ck::physics::CharacterizationConstants driveConstants = ck::physics::DriveCharacterization::characterizeDrive(velocityData, accelerationData);

    ASSERT_NEAR(driveConstants.ks, ks, kTestEps);
    ASSERT_NEAR(driveConstants.kv, kv, kTestEps);
    ASSERT_NEAR(driveConstants.ka, ka, kTestEps);
}