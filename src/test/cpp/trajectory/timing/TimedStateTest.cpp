#include <vector>
#include <iostream>
#include <chrono>

#include "gtest/gtest.h"
#include "utils/CKMath.hpp"
#include "geometry/Geometry.hpp"
#include "trajectory/timing/TimedState.hpp"

using namespace std;
using namespace ck::geometry;
using namespace ck::trajectory::timing;

TEST(TimedStateTest, PlausibleOutput)
{
    double kTestEps = 1e-12;
    // At (0,0,0), t=0, v=0, acceleration=1
    Pose2d startPose = Pose2d::fromTranslation(Translation2d(0.0, 0.0));
    TimedState<Pose2d> start_state(startPose, 0.0, 0.0, 1.0);

    // At (.5,0,0), t=1, v=1, acceleration=0
    Pose2d endPose = Pose2d::fromTranslation(Translation2d(0.5, 0.0));
    TimedState<Pose2d> end_state(endPose, 1.0, 1.0, 0.0);

    ASSERT_EQ(start_state, start_state.interpolate(end_state, 0.0));
    ASSERT_EQ(end_state, start_state.interpolate(end_state, 1.0));
    ASSERT_EQ(end_state, end_state.interpolate(start_state, 0.0));
    ASSERT_EQ(start_state, end_state.interpolate(start_state, 1.0));

    TimedState<Pose2d> intermediate_state = start_state.interpolate(end_state, 0.5);
    ASSERT_NEAR(0.5, intermediate_state.t(), kTestEps);
    ASSERT_NEAR(start_state.acceleration(), intermediate_state.acceleration(), kTestEps);
    ASSERT_NEAR(0.5, intermediate_state.velocity(), kTestEps);
    ASSERT_NEAR(0.125, intermediate_state.state().getTranslation().x(), kTestEps);
}