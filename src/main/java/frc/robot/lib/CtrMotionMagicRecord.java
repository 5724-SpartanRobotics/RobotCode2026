package frc.robot.lib;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;

public record CtrMotionMagicRecord(
	AngularVelocity cruiseVelocity, AngularAcceleration acceleration,
	Velocity<AngularAccelerationUnit> jerk) {
}
