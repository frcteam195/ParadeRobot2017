#include "autonomous/modes/TestMode.hpp"
#include "autonomous/actions/DriveTrajectory.hpp"

void TestMode::routine() {
    //TODO: Revisit running statically declared trajectories or a way to ensure references are not destructed before/during execution
    //runAction(DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().test90DegPath.get(false), true));
}