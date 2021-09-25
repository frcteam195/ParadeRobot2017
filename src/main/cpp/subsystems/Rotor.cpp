#include "subsystems/Rotor.hpp"
#include "subsystems/Input.hpp"

Rotor::Rotor() {
}

void Rotor::stop() {
}

void Rotor::onFirstStart(double timestamp) {
}

void Rotor::onStart(double timestamp) {
}

void Rotor::onStop(double timestamp) {
    stop();
}

void Rotor::onLoop(double timestamp) {
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
