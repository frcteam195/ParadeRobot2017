#include "utils/GearRatioTemp.hpp"

GearRatio::GearRatio(){
    GearRatio(0,0,0);
}

GearRatio::GearRatio(double _sensor_units_per_rotations,
                     double _gear_ratio_to_output_mech,
                     double _vel_RPM_time_conversion_factor){

    sensor_units_per_rotations = _sensor_units_per_rotations;
    gear_ratio_to_output_mech = _gear_ratio_to_output_mech;
    vel_RPM_time_conversion_factor = _vel_RPM_time_conversion_factor;
}

double GearRatio::convertNativeUnitsToRotations(double nativeUnitsPos) {
    return nativeUnitsPos / sensor_units_per_rotations / gear_ratio_to_output_mech;
}

int GearRatio::convertRotationsToNativeUnits(double rotations) {
    return (int) (rotations * sensor_units_per_rotations * gear_ratio_to_output_mech);
}

double GearRatio::convertNativeUnitsToRPM(int nativeUnits) {
    return (nativeUnits / sensor_units_per_rotations / gear_ratio_to_output_mech * vel_RPM_time_conversion_factor);
}

int GearRatio::convertRPMToNativeUnits(double rpm) {
    return (int) (rpm * sensor_units_per_rotations * gear_ratio_to_output_mech / vel_RPM_time_conversion_factor);
}
