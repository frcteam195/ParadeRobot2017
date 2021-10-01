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

    const double sprocket_to_HOOD_ratio = 1/0.09278;
    const double sprocket_to_BASE_ratio = 1/0.09278;
    const double ROT_TO_DEG = 1/360.0;
    const double hood_step = 3;
    const double base_step = 5;
    const double shoot_step = 300;
    const double shoot_deadband = 500;
    const double max_shoot_accel = 800;

    const double base_rotate_speed = 0.1;
    const double hood_rotate_speed = 0.1;
    const double shoot_vel_target_max = 5000;

    enum class TURRET_STATE {
        IDLE,
        SPINNING,
    };

    void stop() override;
    bool isSystemFaulted() override;
    bool runDiagnostics() override;
    void registerEnabledLoops(ILooper & enabledLooper) override;
    void onFirstStart(double timestamp) override;
    void onStart(double timestamp) override;
    void onStop(double timestamp) override;
    void onLoop(double timestamp) override;
    std::string getName() override;
    void report();
    void filterAccel();

private:
    Turret();

    TURRET_STATE state = TURRET_STATE::IDLE;
    double last_time = 0;

    TalonFX hood_motor = {9};
    GearRatio hood_ratio;
    double hood_pos = 0;

    TalonFX shoot_motor = {13};
    double shoot_vel_target = 0;
    double shoot_vel = 0;
    GearRatio shoot_ratio;

    TalonFX base_motor = {10};
    GearRatio base_ratio;
    double base_pos = 0;

    bool is_shoot_on = false;

};
