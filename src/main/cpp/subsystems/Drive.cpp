#include "subsystems/Drive.hpp"

DataReporter* Drive::logReporter = &NetworkDataReporter::getInstance();

Drive::Drive() {

}

void Drive::stop() {

}

void Drive::onFirstStart(double timestamp) {

}

void Drive::onStart(double timestamp) {

}

void Drive::onStop(double timestamp) {
    stop();
}

void Drive::onLoop(double timestamp) {
    
}

std::string Drive::getName() {
    return "DriveLoop";
}

void Drive::registerEnabledLoops(ILooper & enabledLooper) {
    enabledLooper.registerLoop(*this);
}

bool Drive::isSystemFaulted() {
    return false;
}

bool Drive::runDiagnostics() {
    return true;
}


double Drive::getLeftEncoderDistance() {
    
    //TODO: Implement after motion planners
    return 0.0;
    //return rotationsToInches(mPeriodicIO.left_position_rotations);
}

double Drive::getRightEncoderDistance() {
    //TODO: Implement after motion planners
    return 0.0;
    //return rotationsToInches(mPeriodicIO.right_position_rotations);
}

double Drive::getRightLinearVelocity() {
    
    //TODO: Implement after motion planners
    return 0.0;
    //return rotationsToInches(mPeriodicIO.right_velocity_RPM / 60.0);
}

double Drive::getLeftLinearVelocity() {
    
    //TODO: Implement after motion planners
    return 0.0;
    //return rotationsToInches(mPeriodicIO.left_velocity_RPM / 60.0);
}

double Drive::getLinearVelocity() {
    
    //TODO: Implement after motion planners
    return 0.0;
    //return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
}

void Drive::setDriveControlState(DriveControlState driveControlState) {
    std::scoped_lock<std::mutex> lock(memberAccessMtx);
    mDriveControlState = driveControlState;
}

Rotation2d Drive::getHeading() {
        //TODO: Implement after motion planners
        Rotation2d deleteMe;
		return deleteMe;
	}

void Drive::setHeading(Rotation2d heading) {
    //TODO: Implement after motion planners    
    //mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mGyro.getFusedHeading()).inverse());
    //mPeriodicIO.gyro_heading = heading;
}

Drive::PeriodicIO::PeriodicIO()
:DECLARE_REPORTED(logReporter,left_position_rotations)
,DECLARE_REPORTED(logReporter,right_position_rotations)
{
    // left_position_rotations = 3;
    // right_position_rotations = 4;
}
