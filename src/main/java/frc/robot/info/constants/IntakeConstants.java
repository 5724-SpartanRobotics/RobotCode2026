package frc.robot.info.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import frc.lib.PIDFfRecord;
import frc.robot.info.Math;

public final class IntakeConstants {
	public static final Dimensionless SPEED = Units.Percent.of(15);
	public static final double ON_ARM_GEAR_RATIO = 3; // 3:1

	public static final PIDFfRecord PIDF = new PIDFfRecord(0.0001, 0, 0, 0, 0, 0, 0);

	public static final class Arm {
		public static final double GEAR_RATIO = 9 * 3; // 9:1 -> 3:1 = 27:1
		public static final AngularVelocity SETPOINT_RAMP_RATE = Units.DegreesPerSecond.of(60);
		public static final Angle MIN_ROTATION = Units.Degrees.of(0);
		public static final Angle MAX_ROTATION = Units.Degrees.of(97);
		public static final Angle DEFAULT_ROTATION_SETPOINT = MAX_ROTATION
			.minus(Units.Degrees.of(4.5));

		private static final double kFf = 0.0275;
		public static final PIDFfRecord PIDF = new PIDFfRecord(
			0.275, 0.000005, 0.0000, kFf,
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
