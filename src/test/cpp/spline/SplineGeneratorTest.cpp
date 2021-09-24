#include <vector>
#include <iostream>
#include <chrono>

#include "gtest/gtest.h"
#include "spline/QuinticHermiteSpline.hpp"
#include "spline/SplineGenerator.hpp"
#include "utils/CKMath.hpp"

using namespace std;
using namespace ck::geometry;
using namespace ck::spline;

TEST(SplineGeneratorTest, PlausibleOutput)
{
    double kTestEps = 1e-12;
    // Create the test spline
    Pose2d p1(Translation2d(0, 0), Rotation2d());
    Pose2d p2(Translation2d(15, 10), Rotation2d(1, -5, true));
    QuinticHermiteSpline s(p1, p2);

    vector<Pose2dWithCurvature> samples = SplineGenerator::parameterizeSpline(s);

    double arclength = 0;
    Pose2dWithCurvature cur_pose = samples[0];
    for (Pose2dWithCurvature &sample : samples)
    {
        Twist2d t = Pose2d::log(cur_pose.getPose().inverse().transformBy(sample.getPose()));
        arclength += t.dx;
        cur_pose = sample;
    }

    ASSERT_NEAR(cur_pose.getTranslation().x(), 15.0, kTestEps);
    ASSERT_NEAR(cur_pose.getTranslation().y(), 10.0, kTestEps);
    ASSERT_NEAR(cur_pose.getRotation().getDegrees(), -78.69006752597981, kTestEps);
    ASSERT_NEAR(arclength, 23.17291953186379, kTestEps);
}