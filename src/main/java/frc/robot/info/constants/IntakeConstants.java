package frc.robot.info.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import frc.lib.PIDFfRecord;
import frc.robot.info.Math;

public final class IntakeConstants {
	public static final Dimensionless SPEED = Units.Percent.of(20);
	public static final double ON_ARM_GEAR_RATIO = 5; // 5:1
	public static final Distance UPPER_WHEEL_CURCUMFERENCE = Units.Inches.of(4).times(Math.PI);
	public static final Distance LOWER_WHEEL_CURCUMFERENCE = Units.Inches.of(2.25)
		.times(Math.PI);
	public static final double LOWER_FIXED_GEAR_RATIO = 4; // 4:1
	public static final double UPPER_FIXED_GEAR_RATIO = 1;

	public static final class Arm {
		public static final double GEAR_RATIO = 5 * 5; // 5:1 -> 5:1 = 25:1
		public static final AngularVelocity SETPOINT_RAMP_RATE = Units.DegreesPerSecond.of(60);
		public static final Angle MIN_ROTATION = Units.Degrees.of(0);
		public static final Angle MAX_ROTATION = Units.Degrees.of(92.5);

		private static final double kFf = 0.024;
		public static final PIDFfRecord PIDF = new PIDFfRecord(
			0.3, 0.001, 0.0000, kFf,
			0,
			Units.VoltsPerRadianPerSecond
				.of(RobotConstants.NOMINAL_BATTERY_VOLTAGE.in(Units.Volts) /* volts */ * kFf /*
																								 * kFf
																								 */
					* (60.0 / Math.TWO_PI) /* rad/s */)
				.baseUnitMagnitude()
				/* motor V/rad/s */ * GEAR_RATIO /* V/rad/s */,
			0);
	}
}
