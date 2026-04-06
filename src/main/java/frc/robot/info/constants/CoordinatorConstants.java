package frc.robot.info.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.PIDFfRecord;
import frc.robot.info.Math;

public final class CoordinatorConstants {
	public static final AngularVelocity RUN_TO_SHOOTER_SETPOINT = Units.RPM.of(1300);
	public static final AngularVelocity RUN_TO_STORAGE_SETPOINT = Units.RPM.of(-800);

	private static final double GEAR_RATIO = 1.0;

	private static final double kFf = 0.0000155;
	public static final PIDFfRecord PIDF = new PIDFfRecord(
		// TODO: Tune the P
		0.000335, 0.000, 0.0, kFf,
		0.0,
		Units.VoltsPerRadianPerSecond
			.of(RobotConstants.NOMINAL_BATTERY_VOLTAGE.in(Units.Volts) /* volts */ * kFf /* kFf */
				* (60.0 / Math.TWO_PI) /* rad/s */)
			.baseUnitMagnitude()
			/* motor V/rad/s */ * GEAR_RATIO /* flywheel V/rad/s */,
		0.0);

	public static final AngularVelocity AGITATOR_RUN_SETPOINT = Units.RPM.of(1000);
	public static final PIDFfRecord AGITATOR_PIDF = new PIDFfRecord(0.0001, 0, 0, 0, 0, 0, 0);
}
