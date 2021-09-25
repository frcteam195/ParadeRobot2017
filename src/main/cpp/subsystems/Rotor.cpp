#include "subsystems/Rotor.hpp"
#include "subsystems/Input.hpp"

Rotor::Rotor() {
    intake_l_motor.SetNeutralMode( NeutralMode::Coast );
    intake_r_motor.SetNeutralMode( NeutralMode::Coast );
    carousel_motor.SetNeutralMode( NeutralMode::Coast );
    roller_motor.SetNeutralMode( NeutralMode::Coast );
    setOff();
}

void Rotor::setOff(){
    intake_l_motor.Set(ControlMode::PercentOutput, 0);
    intake_r_motor.Set(ControlMode::PercentOutput, 0);
    carousel_motor.Set(ControlMode::PercentOutput, 0);
    roller_motor.Set(ControlMode::PercentOutput, 0);
}

void Rotor::controlIntake( int dir ){
    double percent = dir == 1 ? 1.0 : -1.0;
    intake_l_motor.Set(ControlMode::PercentOutput, percent);
    intake_r_motor.Set(ControlMode::PercentOutput, percent);
}


void Rotor::controlCarousel( int dir ){
    double percent = dir == 1 ? 1.0 : -1.0;
    carousel_motor.Set(ControlMode::PercentOutput, percent);
    roller_motor.Set(ControlMode::PercentOutput, percent);
}

void Rotor::stop() {
    setOff();
}

void Rotor::onFirstStart(double timestamp) {
    setOff();
}

void Rotor::onStart(double timestamp) {
    setOff();
}

void Rotor::onStop(double timestamp) {
    stop();
}

void Rotor::onLoop(double timestamp) {

    if( Input::getInstance().getJoystick().GetRawButtonPressed(9) ){
        is_system_on = !is_system_on;
    }

    if( is_system_on ){
        controlIntake(intake_dir);
        controlCarousel(carousel_dir);
    }else{
        setOff();
    }

}

std::string Rotor::getName() {
    return "RotorLoop";
}

void Rotor::registerEnabledLoops(ILooper & enabledLooper) {
    enabledLooper.registerLoop(*this);
}

bool Rotor::isSystemFaulted() {
    return false;
}

bool Rotor::runDiagnostics() {
    return true;
}
