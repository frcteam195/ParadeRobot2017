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

    void setOff();
    void stop() override;
    bool isSystemFaulted() override;
    bool runDiagnostics() override;
    void registerEnabledLoops(ILooper & enabledLooper) override;
    void onFirstStart(double timestamp) override;
    void onStart(double timestamp) override;
    void onStop(double timestamp) override;
    void onLoop(double timestamp) override;
    std::string getName() override;
    void controlIntake( int dir );
    void controlCarousel( int dir );

private:
    Rotor();
    TalonSRX intake_l_motor = {12};
    TalonSRX intake_r_motor = {11};
    TalonSRX carousel_motor = {7};
    TalonSRX roller_motor = {8};
    bool is_system_on = false;
    int intake_dir = 1;
    int carousel_dir = -1;
};

