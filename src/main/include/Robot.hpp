#pragma once

#include <string>
#include <frc/livewindow/LiveWindow.h>
#include <iostream>
#include <frc/TimedRobot.h>

#include "SubsystemManager.hpp"
#include "utils/Looper/Looper.hpp"
#include "reporters/NetworkDataType.hpp"

#include "subsystems/Drive.hpp"
#include "subsystems/Rotor.hpp"
#include "subsystems/Turret.hpp"
#include "subsystems/Input.hpp"

class Robot : public frc::TimedRobot {
public:
    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;

private:
    SubsystemManager* mSubsystemManager;
    Looper mEnabledLooper;
    Looper mDisabledLooper;
};
