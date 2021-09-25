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

class Turret : public Subsystem, public Singleton<Turret>, public Loop {
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
    Turret();
    TalonFX shoot_motor = {13};
};

