#pragma once

#include "ctre/Phoenix.h"
#include "utils/Subsystem.hpp"
#include "reporters/NetworkDataType.hpp"
#include "reporters/DataReporter.hpp"
#include "reporters/NetworkDataReporter.hpp"
#include "utils/Singleton.hpp"
#include "geometry/Pose2dWithCurvature.hpp"
#include <mutex>

using namespace ck::log;
using namespace ck::geometry;

class Drive : public Subsystem, public Singleton<Drive>, public Loop {
    friend Singleton;
    friend class PeriodicIO;
public:

	enum DriveControlState {
		OPEN_LOOP,
		PATH_FOLLOWING,
		VELOCITY,
		CLIMB,
		OPEN_LOOP_AUTOMATED
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
    
	double getLeftEncoderDistance();
	double getRightEncoderDistance();
	double getRightLinearVelocity();
	double getLeftLinearVelocity();
	double getLinearVelocity();

    void setDriveControlState(DriveControlState driveControlState);

    Rotation2d getHeading();
    void setHeading(Rotation2d heading);

private:
    Drive();
    DriveControlState mDriveControlState;
    static DataReporter* logReporter;
    std::mutex memberAccessMtx; 

    class PeriodicIO {
    public:
        PeriodicIO();

        NetworkDouble left_position_rotations;
        NetworkDouble right_position_rotations;
    };

    PeriodicIO mPeriodicIO;
};
