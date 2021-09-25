#include "subsystems/Turret.hpp"
#include "subsystems/Input.hpp"


Turret::Turret() {
    shoot_vel = 0;
    shoot_motor.Set(ControlMode::Velocity, shoot_vel);

    // hood --------------------
    hood_motor.Set(ControlMode::PercentOutput, 0);
    hood_ratio = GearRatio(2048, 50 * ROT_TO_DEG * sprocket_to_HOOD_ratio, 600);

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


    // base --------------------
    base_motor.Set(ControlMode::PercentOutput, 0);
    base_ratio = GearRatio(2048, 50 * ROT_TO_DEG * sprocket_to_BASE_ratio, 600);

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


    if( Input::getInstance().getJoystick().GetRawButtonPressed(6) ){
        shoot_vel_target += shoot_step;
    }

    if( Input::getInstance().getJoystick().GetRawButtonPressed(5) ){
        shoot_vel_target -= shoot_step;
    }

    if( Input::getInstance().getJoystick().GetRawButtonPressed(3) ){
        hood_pos -= hood_step;
    }

    if( Input::getInstance().getJoystick().GetRawButtonPressed(4) ){
        hood_pos += hood_step;
    }

    if( Input::getInstance().getJoystick().GetRawButtonPressed(1) ){
        base_pos -= base_step;
    }

    if( Input::getInstance().getJoystick().GetRawButtonPressed(2) ){
        base_pos += base_step;
    }


    double vel_diff = shoot_vel_target - shoot_ratio.convertNativeUnitsToRPM( shoot_motor.GetSelectedSensorVelocity() );
    double vel_add = fmin(abs(vel_diff),(max_shoot_accel * dt)) * copysign(1.0, vel_diff);

    if( vel_diff > shoot_deadband ){
        shoot_vel = shoot_vel + vel_add;
    }else{
        shoot_vel = shoot_vel_target;
    }
    std::cout << "dt:" << dt << " " << vel_diff << " " << vel_add << "  .. "  << shoot_vel_target << " -> " << shoot_vel << "\n";

    if( state == TURRET_STATE::IDLE ){
        shoot_motor.Set(ControlMode::Velocity, 0);
        hood_motor.Set(ControlMode::MotionMagic, 0);
        base_motor.Set(ControlMode::MotionMagic, 0);

    }else if( state == TURRET_STATE::SPINNING ){


        shoot_motor.Set(ControlMode::Velocity, shoot_ratio.convertRPMToNativeUnits(shoot_vel) );

        hood_motor.Set(ControlMode::MotionMagic,
                       hood_ratio.convertRotationsToNativeUnits(hood_pos));

        base_motor.Set(ControlMode::MotionMagic,
                       base_ratio.convertRotationsToNativeUnits(base_pos));

    }else{
        std::cout << "Turret in known state\n";
        shoot_motor.Set(ControlMode::PercentOutput, 0);
        hood_motor.Set(ControlMode::PercentOutput, 0);
        base_motor.Set(ControlMode::PercentOutput, 0);
    }


    if ( Input::getInstance().getJoystick().GetRawButtonPressed(7) ){
        if( state == TURRET_STATE::SPINNING ){
            state = TURRET_STATE::IDLE;

        }else{
            state = TURRET_STATE::SPINNING;
        }
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
