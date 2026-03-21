package frc.robot.info.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import frc.lib.PIDFfRecord;
import frc.robot.info.Math;

public final class ShooterConstants {
	private static final double SHOOTER_kFf = 0.0000;
	private static final double FEEDER_kFf = 0.00;

	public static final double GEAR_RATIO = 1.0;
	public static final PIDFfRecord SHOOTER_PIDF = new PIDFfRecord(
		// TODO: Tune the P
		0.025, 0.0, 0.0, SHOOTER_kFf,
		0.0,
		Units.VoltsPerRadianPerSecond
			.of(RobotConstants.NOMINAL_BATTERY_VOLTAGE.in(Units.Volts)
				/* volts */ * SHOOTER_kFf /* kFf */
				* (60.0 / Math.TWO_PI) /* rad/s */)
			.baseUnitMagnitude()
			/* motor V/rad/s */ * GEAR_RATIO /* flywheel V/rad/s */,
		0.0);
	public static final AngularVelocity MAX_VELOCITY = Units.RadiansPerSecond.of(
		DCMotor.getNeoVortex(1).freeSpeedRadPerSec);
	public static final AngularAcceleration MAX_ACCELERATION = Units.RadiansPerSecondPerSecond
		.of(1421).times(2.0);
	public static final Current MAX_CURRENT = Units.Amps.of(40);

	public static final AngularVelocity SOFT_LIMIT_VELOCITY = MAX_VELOCITY;

	public static final PIDFfRecord FEEDER_PIDF = new PIDFfRecord(
		// TODO: Tune the P
		0.0002, 0.0, 0.0, FEEDER_kFf,
		0.0,
		Units.VoltsPerRadianPerSecond
			.of(RobotConstants.NOMINAL_BATTERY_VOLTAGE.in(Units.Volts) /* volts */ * FEEDER_kFf /*
																								 * kFf
																								 */
				* (60.0 / Math.TWO_PI) /* rad/s */)
			.baseUnitMagnitude()
			/* motor V/rad/s */ * GEAR_RATIO /* flywheel V/rad/s */,
		0.0);
	public static final double FEEDER_SPEED_COEFF = 2.7;

	public static final Angle LAUNCH_ANGLE = Units.Degrees.of(25);
	public static final double LAUNCH_VELOCITY_FUDGE_COEFF = 3.33; // usually between 1.1 and
																	// 1.4;

	public static final Distance FLYWHEEL_DIAMETER = Units.Inches.of(4);
	public static final Distance FEEDER_PULLEY_DIAMETER = Units.Inches.of(1.2);
	public static final double DEFAULT_FLYWHEEL_SPEEDMOD = 1.1;
}
