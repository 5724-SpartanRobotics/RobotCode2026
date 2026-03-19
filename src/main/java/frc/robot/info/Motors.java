package frc.robot.info;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class Motors {
	public static final AngularVelocity REDLINE_MAX_VELOCITY = Units.RadiansPerSecond.of(
		DCMotor.getAndymarkRs775_125(1).freeSpeedRadPerSec);
	public static final AngularVelocity VORTEX_MAX_VELOCITY = Units.RadiansPerSecond.of(
		DCMotor.getNeoVortex(1).freeSpeedRadPerSec);
	public static final AngularVelocity NEO_MAX_VELOCITY = Units.RadiansPerSecond.of(
		DCMotor.getNEO(1).freeSpeedRadPerSec);

	// Encoder resolution (I think number of encoder polls per frame)
	public static final double NEO_COUNTS_PER_REVOLUTION = 1.0;

	public static final Current KRAKENX60_STALL_CURRENT = Units.Amps.of(
		DCMotor.getKrakenX60(1).stallCurrentAmps);
}
