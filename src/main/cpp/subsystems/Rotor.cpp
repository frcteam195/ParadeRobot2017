#include "subsystems/Rotor.hpp"
#include "subsystems/Input.hpp"

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

Rotor::Rotor() {
    SupplyCurrentLimitConfiguration limit(true, 15, 0, 0);
    intake_l_motor.ConfigSupplyCurrentLimit(limit);
    intake_r_motor.ConfigSupplyCurrentLimit(limit);
    carousel_motor.ConfigSupplyCurrentLimit(limit);
    roller_motor.ConfigSupplyCurrentLimit(limit);

    intake_l_motor.SetNeutralMode( NeutralMode::Coast );
    intake_r_motor.SetNeutralMode( NeutralMode::Coast );
    carousel_motor.SetNeutralMode( NeutralMode::Coast );
    roller_motor.SetNeutralMode( NeutralMode::Coast );

    intake_l_motor.ConfigOpenloopRamp(0.05);
    intake_r_motor.ConfigOpenloopRamp(0.05);

    setOff();

}

void Rotor::setOff(){
    setOffIntake();
    setOffCarousel();
}

void Rotor::setOffIntake(){
    intake_l_motor.Set(ControlMode::PercentOutput, 0);
    intake_r_motor.Set(ControlMode::PercentOutput, 0);
}

void Rotor::setOffCarousel(){
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

    if( Input::getInstance().getJoystick().GetRawButton(1) ){
        carousel_on = true;
    }
    else {
        carousel_on = false;
    }

    if( Input::getInstance().getJoystick().GetRawButton(2) ){
        intake_on = true;
    }
    else {
        intake_on = false;
    }

    if( Input::getInstance().getJoystick().GetRawButtonPressed(3) ){
        intake_dir = intake_dir == 1 ? -1 : 1;
    }

    if( carousel_on ){
        controlCarousel(carousel_dir);
    }else{
        setOffCarousel();
    }

    if( intake_on ){
        controlIntake(intake_dir);
    }else{
        setOffIntake();
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
