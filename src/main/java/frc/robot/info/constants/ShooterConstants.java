package frc.robot.info.constants;

import java.util.Map;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Time;
import frc.lib.PIDFfRecord;
import frc.robot.info.Math;

public final class ShooterConstants {
	private static final double SHOOTER_kP = 0.0005;
	private static final double SHOOTER_kFf = 0.0000138225;
	private static final double FEEDER_kFf = 0.00;

	public static final double GEAR_RATIO = 1.0;
	public static final PIDFfRecord SHOOTER_PIDF = new PIDFfRecord(
		// TODO: Tune the P
		SHOOTER_kP, 0.0, 0.1 * SHOOTER_kP, SHOOTER_kFf,
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
	public static final double FEEDER_SPEED_COEFF = 0.95;

	public static final Angle LAUNCH_ANGLE = Units.Degrees.of(30);
	public static final double LAUNCH_VELOCITY_FUDGE_COEFF = 1.0; // usually between 1.1 and
																	// 1.4;

	public static final Distance FLYWHEEL_DIAMETER = Units.Inches.of(4);
	public static final Distance FEEDER_PULLEY_DIAMETER = Units.Inches.of(1.75);
	public static final double FEEDER_GEAR_RATIO = 5.0; // 5:1
	public static final double DEFAULT_FLYWHEEL_SPEEDMOD = 0.955;

	// --- Distance filtering ---
	public static final Time DISTANCE_FILTER_TIME_CONSTANT = Units.Seconds.of(0.25);

	// --- Shooter curve (quadratic example: RPM = a*d^2 + b*d + c) ---
	// How aggressively RPM ramps up at long distance
	public static final Per<AngularVelocityUnit, PerUnit<?, ?>> SHOOTER_RPM_CURVATURE = Units.RPM
		.of(6.5).per(Units.Meter.per(Units.Meter)); // RPM/m^2
	// How much RPM increases per meter
	public static final Per<AngularVelocityUnit, DistanceUnit> SHOOTER_RPM_SLOPE = Units.RPM.of(450)
		.per(Units.Meter); // RPM per meter
	// Minimum RPM needed to even reach the goal (close shots)
	public static final AngularVelocity SHOOTER_RPM_INTERCEPT = Units.RPM.of(2000); // Base RPM

	// --- Limits ---
	public static final AngularVelocity MIN_SHOOTER_VELOCITY = Units.RPM.of(1800);
	public static final AngularVelocity MAX_SHOOTER_VELOCITY = Units.RPM.of(4250);

	// --- Rate limiting ---
	public static final double MAX_RPM_CHANGE_PER_LOOP = 150.0; // RPM per 20ms loop

	// --- Optional quantization (set to 0 to disable) ---
	public static final AngularVelocity RPM_STEP_SIZE = Units.RPM.of(150);

	public static final Map<Distance, AngularVelocity> SPEED_MAP = Map.of(
		// Units.Meters.of(1.0), Units.RPM.of(3300),
		Units.Meters.of(2.0), Units.RPM.of(3150),
		Units.Meters.of(2.5), Units.RPM.of(3150),
		Units.Meters.of(2.85), Units.RPM.of(3300),
		// Units.Meters.of(3.0), Units.RPM.of(3450),
		// Units.Meters.of(4.0), Units.RPM.of(3600),
		Units.Meters.of(10), Units.RPM.of(6600));
}
