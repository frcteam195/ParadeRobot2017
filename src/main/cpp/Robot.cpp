#include "Robot.hpp"

void Robot::RobotInit() {
    try {
        frc::LiveWindow::GetInstance()->DisableAllTelemetry();

        mSubsystemManager = &SubsystemManager::getInstance({
            &Input::getInstance(),
            &Drive::getInstance(),
            &Turret::getInstance()
        });

        mSubsystemManager->registerEnabledLoops(mEnabledLooper);
        mSubsystemManager->registerDisabledLoops(mDisabledLooper);
    } catch (std::exception &ex) {

    }
}

void Robot::RobotPeriodic() {
    // std::cout << "Running robot code periodic!" << std::endl;
}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
    mEnabledLooper.start();
}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

void Robot::DisabledInit() {
    mEnabledLooper.stop();
}

void Robot::DisabledPeriodic() {
    Turret::getInstance().report();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
