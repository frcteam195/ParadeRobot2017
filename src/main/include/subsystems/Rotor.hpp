#pragma once

#include "ctre/Phoenix.h"
#include "utils/Subsystem.hpp"
#include "reporters/NetworkDataType.hpp"
#include "reporters/DataReporter.hpp"
#include "reporters/NetworkDataReporter.hpp"
#include "utils/Singleton.hpp"
#include "utils/GearRatioTemp.hpp"
#include <mutex>
#include <math.h>

class Rotor : public Subsystem, public Singleton<Rotor>, public Loop {
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

private:
    Rotor();
    TalonFX suck_motor = {13};
    TalonFX middle_motor = {13};
    TalonFX feed_motor = {13};
};

