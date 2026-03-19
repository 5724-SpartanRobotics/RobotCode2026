package frc.robot.info.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public final class RobotConstants {
	public static final Voltage NOMINAL_BATTERY_VOLTAGE = Units.Volts.of(13.1);

	// TODO: These values will all need updated once the robot is finished.
	public static final Mass MASS = Units.Pounds.of(95);

	public static final MomentOfInertia MOMENT_OF_INERTIA = Units.KilogramSquareMeters.of(4);

	// 0.319 (wheel circumference meters) *
	// ((6784 motor max rpm / 8 gear ratio)/(60 seconds in minute))
	// = 0.319 * 14.13 = 4.51 m/s
	public static final LinearVelocity MAX_LINEAR_VELOCITY = Units.MetersPerSecond.of(5);

	public static final LinearAcceleration MAX_LINEAR_ACCELERATION = Units.MetersPerSecondPerSecond
		.of(10.2);

	public static final AngularVelocity MAX_ANGULAR_VELOCITY = Units.DegreesPerSecond
		.of(540.000);

	public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = Units.DegreesPerSecondPerSecond
		.of(2338);

	public static final double DEFAULT_SPEED_MOD = 0.7;
	public static final double DEFAULT_SPEED_MOD_HIGH = 1;
	public static final double DEFAULT_SPEED_MOD_LOW = 0.4;
}
