#include "subsystems/Turret.hpp"
#include "subsystems/Input.hpp"

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

Turret::Turret() {
    SupplyCurrentLimitConfiguration limit(true, 10, 0, 0);


    shoot_vel = 0;
    shoot_vel_target = 0;
    shoot_motor.Set(ControlMode::Velocity, shoot_vel);
    shoot_motor.ConfigSupplyCurrentLimit(limit);

    // hood --------------------
    
    hood_motor.Set(ControlMode::PercentOutput, 0);
    hood_ratio = GearRatio(2048, 50 * ROT_TO_DEG * sprocket_to_HOOD_ratio, 600);
    hood_motor.ConfigSupplyCurrentLimit(limit);
    hood_motor.Config_kP(0, 0.375);
    hood_motor.Config_kI(0, 0);
    hood_motor.Config_kD(0, 0.5);
    double kf = 1023.0 / (6380 / 1.0 * 2048.0 / 600.0);
    hood_motor.Config_kF(0, kf);
    hood_motor.ConfigMotionCruiseVelocity(
        hood_ratio.convertRPMToNativeUnits( 2200 ));

    hood_motor.ConfigMotionAcceleration(
        hood_ratio.convertRPMToNativeUnits( 4800 ));

    hood_motor.ConfigMotionSCurveStrength(3);
    hood_motor.SetNeutralMode( NeutralMode::Brake );
    hood_motor.ConfigReverseSoftLimitThreshold(0, 0);
    hood_motor.ConfigReverseSoftLimitEnable(true, 0);


    // base --------------------
    base_motor.Set(ControlMode::PercentOutput, 0);
    base_ratio = GearRatio(2048, 50 * ROT_TO_DEG * sprocket_to_BASE_ratio, 600);
    base_motor.ConfigSupplyCurrentLimit(limit);
    base_motor.Config_kP(0, 0.375);
    base_motor.Config_kI(0, 0);
    base_motor.Config_kD(0, 0.5);
    kf = 1023.0 / (6380 / 1.0 * 2048.0 / 600.0);
    base_motor.Config_kF(0, kf);

    base_motor.ConfigMotionCruiseVelocity(
        base_ratio.convertRPMToNativeUnits( 1800 ));

    base_motor.ConfigMotionAcceleration(
        base_ratio.convertRPMToNativeUnits( 2200 ));

    base_motor.ConfigMotionSCurveStrength(3);
    base_motor.SetNeutralMode( NeutralMode::Brake );

    // vel control --------------------
    shoot_ratio = GearRatio(2048, 1, 600);
    shoot_motor.Set(ControlMode::PercentOutput, 0);
    shoot_motor.Config_kP(0, 0.2);
    shoot_motor.Config_kI(0, 0);
    //shoot_motor.ConfigMaxIntegralAccumulator(0, 0);
    shoot_motor.Config_kD(0, 22);
    shoot_motor.Config_kF(0, 0.046300);
    shoot_motor.SetNeutralMode( NeutralMode::Coast );


}

void Turret::stop() {
    shoot_motor.Set( ControlMode::PercentOutput, 0 );
    base_motor.Set(ControlMode::PercentOutput, 0);
    hood_motor.Set(ControlMode::PercentOutput, 0);
    shoot_vel = 0;
}

void Turret::onFirstStart(double timestamp) {
    shoot_motor.Set(ControlMode::Velocity, shoot_vel);
    hood_motor.SetSelectedSensorPosition( 0 );
    hood_motor.Set(ControlMode::MotionMagic, 0);
    base_motor.SetSelectedSensorPosition( 0 );
    base_motor.Set(ControlMode::MotionMagic, 0);
    last_time = timestamp;
}

void Turret::onStart(double timestamp) {
    shoot_motor.Set(ControlMode::Velocity, shoot_vel);
    hood_motor.Set(ControlMode::MotionMagic, 0);
    base_motor.Set(ControlMode::MotionMagic, 0);
    last_time = timestamp;
}

void Turret::onStop(double timestamp) {
    stop();
}

void Turret::onLoop(double timestamp) {
    double dt = timestamp - last_time;
    last_time = timestamp;

    double z_axis = -Input::useDeadband(Input::getInstance().getJoystick().GetZ());
    base_pos += base_rotate_speed*z_axis;
    base_pos = base_pos < -12 ? -12 : base_pos;
    base_pos = base_pos > 12 ? 12 : base_pos;

    double y_axis = Input::useDeadband(Input::getInstance().getJoystick().GetY());
    hood_pos += hood_rotate_speed*y_axis;

    if( Input::getInstance().getJoystick().GetRawButtonPressed(4) ){
        is_shoot_on = !is_shoot_on;
        printf("[shoot] ON:%d vel:%f target:%f\n", is_shoot_on, shoot_vel, shoot_vel_target );
    }

    if( Input::getInstance().getJoystick().GetRawButtonPressed(6) ){
        shoot_vel_target -= shoot_step;
        shoot_vel_target = shoot_vel_target < 0 ? 0 : shoot_vel_target;
        printf("[shoot] ON:%d vel:%f target:%f\n", is_shoot_on, shoot_vel, shoot_vel_target );
    }

    if( Input::getInstance().getJoystick().GetRawButtonPressed(8) ){
        shoot_vel_target += shoot_step;
        shoot_vel_target = shoot_vel_target < shoot_vel_target_max ? shoot_vel_target : shoot_vel_target_max;
        printf("[shoot] ON:%d vel:%f target:%f\n", is_shoot_on, shoot_vel, shoot_vel_target );
    }


    double vel_diff = shoot_vel_target - shoot_ratio.convertNativeUnitsToRPM( shoot_motor.GetSelectedSensorVelocity() );
    double vel_add = fmin(abs(vel_diff),(max_shoot_accel * dt)) * copysign(1.0, vel_diff);
    if( abs(vel_diff) > shoot_deadband ){
        shoot_vel = shoot_vel + vel_add;
    }else{
        shoot_vel = shoot_vel_target;
    }

    base_motor.Set(ControlMode::MotionMagic,
                   base_ratio.convertRotationsToNativeUnits(base_pos));

    hood_motor.Set(ControlMode::MotionMagic,
                   hood_ratio.convertRotationsToNativeUnits(hood_pos));



    if( is_shoot_on ){
        shoot_motor.Set(ControlMode::Velocity, shoot_ratio.convertRPMToNativeUnits(shoot_vel) );
    }else{
        shoot_vel = 0;
        shoot_motor.Set(ControlMode::PercentOutput, 0 );
    }

}

void Turret::report(){
    //std::cout << hood_motor.GetSelectedSensorPosition() << "\n";
    /*std::cout << hood_motor.GetSelectedSensorPosition() << ", "
              << hood_ratio.convertNativeUnitsToRotations( hood_motor.GetSelectedSensorPosition() ) << "\n";*/
}

std::string Turret::getName() {
    return "TurretLoop";
}

void Turret::registerEnabledLoops(ILooper & enabledLooper) {
    enabledLooper.registerLoop(*this);
}

bool Turret::isSystemFaulted() {
    return false;
}

bool Turret::runDiagnostics() {
    return true;
}
