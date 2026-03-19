package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ShooterIO {
	@AutoLog
	public static class ShooterIOInputs implements LoggableInputs {
		// State
		public boolean enabled = false;
		public boolean reversed = false;

		// Distance + tuning
		public double distanceMeters = 0.0;
		public double flywheelSpeedMod = 0.0;

		// Feeder
		public double feederVelocityRPM = 0.0;
		public double feederSetpointRPM = 0.0;
		public boolean feederRunning = false;

		// Calculated shooter
		public double targetFlywheelRPM = 0.0;

		@Override
		public void toLog(LogTable table) {
			table.put("Enabled", enabled);
			table.put("Reversed", reversed);

			table.put("DistanceMeters", distanceMeters);
			table.put("FlywheelSpeedMod", flywheelSpeedMod);

			table.put("FeederVelocityRPM", feederVelocityRPM);
			table.put("FeederSetpointRPM", feederSetpointRPM);
			table.put("FeederRunning", feederRunning);

			table.put("TargetFlywheelRPM", targetFlywheelRPM);
		}

		@Override
		public void fromLog(LogTable table) {
			enabled = table.get("Enabled", false);
			reversed = table.get("Reversed", false);

			distanceMeters = table.get("DistanceMeters", 0.0);
			flywheelSpeedMod = table.get("FlywheelSpeedMod", 0.0);

			feederVelocityRPM = table.get("FeederVelocityRPM", 0.0);
			feederSetpointRPM = table.get("FeederSetpointRPM", 0.0);
			feederRunning = table.get("FeederRunning", false);

			targetFlywheelRPM = table.get("TargetFlywheelRPM", 0.0);
		}
	}
}
