package frc.robot.info.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import frc.lib.CtrMotionMagicRecord;
import frc.lib.PIDFfRecord;
import frc.robot.info.Math;

public final class ClimberConstants {
	private static final double kFf = 0.0002;
	public static final double GEAR_RATIO = 12;
	public static final Current MAX_CURRENT = Units.Amps.of(40);
	public static final PIDFfRecord PIDF = new PIDFfRecord(
		// TODO: Tune the P
		0.025, 0.0, 0.0, kFf,
		0.0,
		Units.VoltsPerRadianPerSecond
			.of(RobotConstants.NOMINAL_BATTERY_VOLTAGE.in(Units.Volts) /* volts */ * kFf /* kFf */
				* (60.0 / Math.TWO_PI) /* rad/s */)
			.baseUnitMagnitude()
			/* motor V/rad/s */ * GEAR_RATIO /* flywheel V/rad/s */,
		0.0);
	public static final CtrMotionMagicRecord MOTION_MAGIC = new CtrMotionMagicRecord(
		Units.RadiansPerSecond.of(0),
		Units.RadiansPerSecondPerSecond.of(6 * Math.TWO_PI),
		Units.RotationsPerSecondPerSecond.of(15).per(Units.Seconds));
}
