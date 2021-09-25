#pragma once

#include "frc/Joystick.h"
#include "ctre/Phoenix.h"
#include "utils/Subsystem.hpp"
#include "reporters/NetworkDataType.hpp"
#include "reporters/DataReporter.hpp"
#include "reporters/NetworkDataReporter.hpp"
#include "utils/Singleton.hpp"
#include <mutex>

class Input : public Subsystem, public Singleton<Input>, public Loop {
    friend Singleton;
    friend class PeriodicIO;

public:

    void stop() override;
    bool isSystemFaulted() override;
    bool runDiagnostics() override;
    void registerEnabledLoops(ILooper & enabledLooper) override;
    void onFirstStart(double timestamp) override;
    void onStart(double timestamp) override;
    void onStop(double timestamp) override;
    void onLoop(double timestamp) override;
    std::string getName() override;

    frc::Joystick& getJoystick();

private:
    Input();

    frc::Joystick joystick{0};
};
