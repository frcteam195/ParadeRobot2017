#include <vector>
#include <iostream>

#include "geometry/Geometry.hpp"
#include "gtest/gtest.h"
#include "utils/CKMath.hpp"

using namespace ck::geometry;

TEST(SE2Math, Rotation2d) { 

    double kTestEps = 1e-12;

    // Test constructors
    Rotation2d rot1;
    ASSERT_NEAR(1, rot1.cos(), kTestEps);
    ASSERT_NEAR(0, rot1.sin(), kTestEps);
    ASSERT_NEAR(0, rot1.tan(), kTestEps);
    ASSERT_NEAR(0, rot1.getDegrees(), kTestEps);
    ASSERT_NEAR(0, rot1.getRadians(), kTestEps);

    rot1 = Rotation2d(1, 1, true);
    ASSERT_NEAR(std::sqrt(2) / 2, rot1.cos(), kTestEps);
    ASSERT_NEAR(std::sqrt(2) / 2, rot1.sin(), kTestEps);
    ASSERT_NEAR(1, rot1.tan(), kTestEps);
    ASSERT_NEAR(45, rot1.getDegrees(), kTestEps);
    ASSERT_NEAR(ck::math::PI / 4, rot1.getRadians(), kTestEps);

    rot1 = rot1.fromRadians(ck::math::PI / 2);
    ASSERT_NEAR(0, rot1.cos(), kTestEps);
    ASSERT_NEAR(1, rot1.sin(), kTestEps);
    ASSERT_TRUE(1 / kTestEps < rot1.tan());
    ASSERT_NEAR(90, rot1.getDegrees(), kTestEps);
    ASSERT_NEAR(ck::math::PI / 2, rot1.getRadians(), kTestEps);

    rot1 = rot1.fromDegrees(270);
    ASSERT_NEAR(0, rot1.cos(), kTestEps);
    ASSERT_NEAR(-1, rot1.sin(), kTestEps);
    std::cout << rot1.tan() << std::endl;
    ASSERT_TRUE(-1 / kTestEps > rot1.tan());
    ASSERT_NEAR(-90, rot1.getDegrees(), kTestEps);
    ASSERT_NEAR(-ck::math::PI / 2, rot1.getRadians(), kTestEps);

    // Test inversion
    rot1 = rot1.fromDegrees(270);
    Rotation2d rot2 = rot1.inverse();
    ASSERT_NEAR(0, rot2.cos(), kTestEps);
    ASSERT_NEAR(1, rot2.sin(), kTestEps);
    ASSERT_TRUE(1 / kTestEps < rot2.tan());
    ASSERT_NEAR(90, rot2.getDegrees(), kTestEps);
    ASSERT_NEAR(ck::math::PI / 2, rot2.getRadians(), kTestEps);

    rot1 = rot1.fromDegrees(1);
    rot2 = rot1.inverse();
    ASSERT_NEAR(rot1.cos(), rot2.cos(), kTestEps);
    ASSERT_NEAR(-rot1.sin(), rot2.sin(), kTestEps);
    ASSERT_NEAR(-1, rot2.getDegrees(), kTestEps);

    // Test rotateBy
    rot1 = rot1.fromDegrees(45);
    rot2 = rot2.fromDegrees(45);
    Rotation2d rot3 = rot1.rotateBy(rot2);
    ASSERT_NEAR(0, rot3.cos(), kTestEps);
    ASSERT_NEAR(1, rot3.sin(), kTestEps);
    ASSERT_TRUE(1 / kTestEps < rot3.tan());
    ASSERT_NEAR(90, rot3.getDegrees(), kTestEps);
    ASSERT_NEAR(ck::math::PI / 2, rot3.getRadians(), kTestEps);

    rot1 = rot1.fromDegrees(45);
    rot2 = rot2.fromDegrees(-45);
    rot3 = rot1.rotateBy(rot2);
    ASSERT_NEAR(1, rot3.cos(), kTestEps);
    ASSERT_NEAR(0, rot3.sin(), kTestEps);
    ASSERT_NEAR(0, rot3.tan(), kTestEps);
    ASSERT_NEAR(0, rot3.getDegrees(), kTestEps);
    ASSERT_NEAR(0, rot3.getRadians(), kTestEps);

    // A rotation times its inverse should be the identity
    Rotation2d identity;
    rot1 = rot1.fromDegrees(21.45);
    rot2 = rot1.rotateBy(rot1.inverse());
    ASSERT_NEAR(identity.cos(), rot2.cos(), kTestEps);
    ASSERT_NEAR(identity.sin(), rot2.sin(), kTestEps);
    ASSERT_NEAR(identity.getDegrees(), rot2.getDegrees(), kTestEps);

    // Test interpolation
    rot1 = rot1.fromDegrees(45);
    rot2 = rot2.fromDegrees(135);
    rot3 = rot1.interpolate(rot2, .5);
    ASSERT_NEAR(90, rot3.getDegrees(), kTestEps);

    rot1 = rot1.fromDegrees(45);
    rot2 = rot2.fromDegrees(135);
    rot3 = rot1.interpolate(rot2, .75);
    ASSERT_NEAR(112.5, rot3.getDegrees(), kTestEps);

    rot1 = rot1.fromDegrees(45);
    rot2 = rot2.fromDegrees(-45);
    rot3 = rot1.interpolate(rot2, .5);
    ASSERT_NEAR(0, rot3.getDegrees(), kTestEps);

    rot1 = rot1.fromDegrees(45);
    rot2 = rot2.fromDegrees(45);
    rot3 = rot1.interpolate(rot2, .5);
    ASSERT_NEAR(45, rot3.getDegrees(), kTestEps);

    rot1 = rot1.fromDegrees(45);
    rot2 = rot2.fromDegrees(45);
    rot3 = rot1.interpolate(rot2, .5);
    ASSERT_NEAR(45, rot3.getDegrees(), kTestEps);

    // Test parallel.
    rot1 = rot1.fromDegrees(45);
    rot2 = rot2.fromDegrees(45);
    ASSERT_TRUE(rot1.isParallel(rot2));

    rot1 = rot1.fromDegrees(45);
    rot2 = rot2.fromDegrees(-45);
    ASSERT_FALSE(rot1.isParallel(rot2));

    rot1 = rot1.fromDegrees(45);
    rot2 = rot2.fromDegrees(-135);
    ASSERT_TRUE(rot1.isParallel(rot2));
}

TEST(SE2Math, Translation2d) { 

    double kTestEps = 1e-12;

    // Test constructors
    Translation2d pos1;
    ASSERT_NEAR(0, pos1.x(), kTestEps);
    ASSERT_NEAR(0, pos1.y(), kTestEps);
    ASSERT_NEAR(0, pos1.norm(), kTestEps);

    pos1 = Translation2d(3, 4);
    ASSERT_NEAR(3, pos1.x(), kTestEps);
    ASSERT_NEAR(4, pos1.y(), kTestEps);
    ASSERT_NEAR(5, pos1.norm(), kTestEps);

    // Test inversion
    pos1 = Translation2d(3.152, 4.1666);
    Translation2d pos2 = pos1.inverse();
    ASSERT_NEAR(-pos1.x(), pos2.x(), kTestEps);
    ASSERT_NEAR(-pos1.y(), pos2.y(), kTestEps);
    ASSERT_NEAR(pos1.norm(), pos2.norm(), kTestEps);

    // Test rotateBy
    pos1 = Translation2d(2, 0);
    Rotation2d rot1 = rot1.fromDegrees(90);
    pos2 = pos1.rotateBy(rot1);
    ASSERT_NEAR(0, pos2.x(), kTestEps);
    ASSERT_NEAR(2, pos2.y(), kTestEps);
    ASSERT_NEAR(pos1.norm(), pos2.norm(), kTestEps);

    pos1 = Translation2d(2, 0);
    rot1 = rot1.fromDegrees(-45);
    pos2 = pos1.rotateBy(rot1);
    ASSERT_NEAR(std::sqrt(2), pos2.x(), kTestEps);
    ASSERT_NEAR(-std::sqrt(2), pos2.y(), kTestEps);
    ASSERT_NEAR(pos1.norm(), pos2.norm(), kTestEps);

    // Test translateBy
    pos1 = Translation2d(2, 0);
    pos2 = Translation2d(-2, 1);
    Translation2d pos3 = pos1.translateBy(pos2);
    ASSERT_NEAR(0, pos3.x(), kTestEps);
    ASSERT_NEAR(1, pos3.y(), kTestEps);
    ASSERT_NEAR(1, pos3.norm(), kTestEps);

    // A translation times its inverse should be the identity
    Translation2d identity = Translation2d();
    pos1 = Translation2d(2.16612, -23.55);
    pos2 = pos1.translateBy(pos1.inverse());
    ASSERT_NEAR(identity.x(), pos2.x(), kTestEps);
    ASSERT_NEAR(identity.y(), pos2.y(), kTestEps);
    ASSERT_NEAR(identity.norm(), pos2.norm(), kTestEps);

    // Test interpolation
    pos1 = Translation2d(0, 1);
    pos2 = Translation2d(10, -1);
    pos3 = pos1.interpolate(pos2, .5);
    ASSERT_NEAR(5, pos3.x(), kTestEps);
    ASSERT_NEAR(0, pos3.y(), kTestEps);

    pos1 = Translation2d(0, 1);
    pos2 = Translation2d(10, -1);
    pos3 = pos1.interpolate(pos2, .75);
    ASSERT_NEAR(7.5, pos3.x(), kTestEps);
    ASSERT_NEAR(-.5, pos3.y(), kTestEps);
}

TEST(SE2Math, Pose2d) { 

    double kTestEps = 1e-12;

    // Test constructors
    Pose2d pose1;
    ASSERT_NEAR(0, pose1.getTranslation().x(), kTestEps);
    ASSERT_NEAR(0, pose1.getTranslation().y(), kTestEps);
    ASSERT_NEAR(0, pose1.getRotation().getDegrees(), kTestEps);

    Translation2d t(3,4);
    Rotation2d r = r.fromDegrees(45);
    pose1 = Pose2d(t, r);
    ASSERT_NEAR(3, pose1.getTranslation().x(), kTestEps);
    ASSERT_NEAR(4, pose1.getTranslation().y(), kTestEps);
    ASSERT_NEAR(45, pose1.getRotation().getDegrees(), kTestEps);

    // Test transformation
    r = r.fromDegrees(90);
    pose1 = Pose2d(t, r);
    t = Translation2d(1,0);
    r = r.fromDegrees(0);
    Pose2d pose2(t, r);
    Pose2d pose3 = pose1.transformBy(pose2);
    ASSERT_NEAR(3, pose3.getTranslation().x(), kTestEps);
    ASSERT_NEAR(5, pose3.getTranslation().y(), kTestEps);
    ASSERT_NEAR(90, pose3.getRotation().getDegrees(), kTestEps);

    t = Translation2d(3,4);
    r = r.fromDegrees(90);
    pose1 = Pose2d(t, r);
    t = Translation2d(1,0);
    r = r.fromDegrees(-90);
    pose2 = Pose2d(t, r);
    pose3 = pose1.transformBy(pose2);
    ASSERT_NEAR(3, pose3.getTranslation().x(), kTestEps);
    ASSERT_NEAR(5, pose3.getTranslation().y(), kTestEps);
    ASSERT_NEAR(0, pose3.getRotation().getDegrees(), kTestEps);

    // A pose times its inverse should be the identity
    Pose2d identity;
    t = Translation2d(3.51512152, 4.23);
    r = r.fromDegrees(91.6);
    pose1 = Pose2d(t, r);
    pose2 = pose1.transformBy(pose1.inverse());
    ASSERT_NEAR(identity.getTranslation().x(), pose2.getTranslation().x(), kTestEps);
    ASSERT_NEAR(identity.getTranslation().y(), pose2.getTranslation().y(), kTestEps);
    ASSERT_NEAR(identity.getRotation().getDegrees(), pose2.getRotation().getDegrees(), kTestEps);

    // Test interpolation
    // Movement from pose1 to pose2 is along a circle with radius of 10 units centered at (3, -6)
    t = Translation2d(3, 4);
    r = r.fromDegrees(90);
    pose1 = Pose2d(t, r);
    t = Translation2d(13, -6);
    r = r.fromDegrees(0.0);
    pose2 = Pose2d(t, r);
    pose3 = pose1.interpolate(pose2, .5);
    double expected_angle_rads = ck::math::PI / 4;
    ASSERT_NEAR(3.0 + 10.0 * std::cos(expected_angle_rads), pose3.getTranslation().x(), kTestEps);
    ASSERT_NEAR(-6.0 + 10.0 * std::sin(expected_angle_rads), pose3.getTranslation().y(), kTestEps);
    ASSERT_NEAR(expected_angle_rads, pose3.getRotation().getRadians(), kTestEps);

    t = Translation2d(3, 4);
    r = r.fromDegrees(90);
    pose1 = Pose2d(t, r);
    t = Translation2d(13, -6);
    r = r.fromDegrees(0.0);
    pose2 = Pose2d(t, r);
    pose3 = pose1.interpolate(pose2, .75);
    expected_angle_rads = ck::math::PI / 8;
    ASSERT_NEAR(3.0 + 10.0 * std::cos(expected_angle_rads), pose3.getTranslation().x(), kTestEps);
    ASSERT_NEAR(-6.0 + 10.0 * std::sin(expected_angle_rads), pose3.getTranslation().y(), kTestEps);
    ASSERT_NEAR(expected_angle_rads, pose3.getRotation().getRadians(), kTestEps);
}

TEST(SE2Math, Twist2d) { 

    double kTestEps = 1e-12;

    // Exponentiation (integrate twist to obtain a Pose2d)
    Twist2d twist(1.0, 0.0, 0.0);
    Pose2d pose;
    pose = pose.exp(twist);
    ASSERT_NEAR(1.0, pose.getTranslation().x(), kTestEps);
    ASSERT_NEAR(0.0, pose.getTranslation().y(), kTestEps);
    ASSERT_NEAR(0.0, pose.getRotation().getDegrees(), kTestEps);

    // Scaled.
    twist = Twist2d(1.0, 0.0, 0.0);
    pose = pose.exp(twist.scaled(2.5));
    ASSERT_NEAR(2.5, pose.getTranslation().x(), kTestEps);
    ASSERT_NEAR(0.0, pose.getTranslation().y(), kTestEps);
    ASSERT_NEAR(0.0, pose.getRotation().getDegrees(), kTestEps);

    // Logarithm (find the twist to apply to obtain a given Pose2d)
    Translation2d t(2.0, 2.0);
    Rotation2d r = r.fromRadians(ck::math::PI / 2);
    pose = Pose2d(t, r);
    twist = pose.log(pose);
    ASSERT_NEAR(ck::math::PI, twist.dx, kTestEps);
    ASSERT_NEAR(0.0, twist.dy, kTestEps);
    ASSERT_NEAR(ck::math::PI / 2, twist.dtheta, kTestEps);

    // Logarithm is the inverse of exponentiation.
    Pose2d new_pose = pose.exp(twist);
    ASSERT_NEAR(new_pose.getTranslation().x(), pose.getTranslation().x(), kTestEps);
    ASSERT_NEAR(new_pose.getTranslation().y(), pose.getTranslation().y(), kTestEps);
    ASSERT_NEAR(new_pose.getRotation().getDegrees(), pose.getRotation().getDegrees(), kTestEps);
}

