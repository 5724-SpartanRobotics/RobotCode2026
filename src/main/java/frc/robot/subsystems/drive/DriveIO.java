package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;

public interface DriveIO {
	@AutoLog
	public static class DriveIOInputs implements LoggableInputs {
		public double gyroYawRad = 0.0;
		public double[] moduleStates = new double[0]; // flatten if needed
		public double[] modulePositions = new double[0];
		public Pose2d pose = new Pose2d();

		@Override
		public void toLog(LogTable table) {
			table.put("GyroYawRad", gyroYawRad);
			table.put("ModuleStates", moduleStates);
			table.put("ModulePositions", modulePositions);
			table.put("Pose", pose);
		}

		@Override
		public void fromLog(LogTable table) {
			gyroYawRad = table.get("GyroYawRad", 0.0);
			moduleStates = table.get("ModuleStates", new double[0]);
			modulePositions = table.get("ModulePositions", new double[0]);
			pose = table.get("Pose", new Pose2d());
		}
	}

	default void updateInputs(DriveIOInputs inputs) {
	}
}