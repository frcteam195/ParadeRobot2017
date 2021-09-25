#include "subsystems/Input.hpp"

Input::Input() {

}

void Input::stop() {

}

void Input::onFirstStart(double timestamp) {
}

void Input::onStart(double timestamp) {
}

void Input::onStop(double timestamp) {
    stop();
}

void Input::onLoop(double timestamp) {
}

std::string Input::getName() {
    return "InputLoop";
}

void Input::registerEnabledLoops(ILooper & enabledLooper) {
    enabledLooper.registerLoop(*this);
}

bool Input::isSystemFaulted() {
    return false;
}

bool Input::runDiagnostics() {
    return true;
}

frc::Joystick& Input::getJoystick() {
    return joystick;
}
