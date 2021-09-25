#pragma once

class GearRatio {
public:
    double sensor_units_per_rotations;
    double gear_ratio_to_output_mech;
    double vel_RPM_time_conversion_factor;

    GearRatio();
    GearRatio( double _sensor_units_per_rotations,
               double _gear_ratio_to_output_mech,
               double _vel_RPM_time_conversion_factor);

    double convertNativeUnitsToRotations(double nativeUnitsPos);
    int convertRotationsToNativeUnits(double rotations);
    double convertNativeUnitsToRPM(int nativeUnits);
    int convertRPMToNativeUnits(double rpm);

};



