package frc.robot.info.constants;

import java.io.File;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;
import frc.lib.PIDFfRecord;

public final class DriveConstants {
	public static final File SWERVE_CONFIG = new File(Filesystem.getDeployDirectory(),
		"swerve");

	public static final Time WHEEL_LOCK_TIME = Units.Seconds.of(5);

	public static final PIDFfRecord ROTATE_TO_ANGLE_PID = new PIDFfRecord(
		3.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0);
}
