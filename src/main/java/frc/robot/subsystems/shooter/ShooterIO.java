package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ShooterIO {
	@AutoLog
	public static class ShooterIOInputs implements LoggableInputs {
		// State
		public boolean enabledFlywheel = false;
		public boolean reversed = false;

		// Distance + tuning
		public double distanceMeters = 0.0;
		public double flywheelSpeedMod = 0.0;

		// Calculated shooter
		public double targetFlywheelRPM = 0.0;

		@Override
		public void toLog(LogTable table) {
			table.put("EnabledFlywheel", enabledFlywheel);
			// table.put("EnabledFeeder", enabledFeeder);
			table.put("Reversed", reversed);

			table.put("DistanceMeters", distanceMeters);
			table.put("FlywheelSpeedMod", flywheelSpeedMod);

			table.put("TargetFlywheelRPM", targetFlywheelRPM);
		}

		@Override
		public void fromLog(LogTable table) {
			enabledFlywheel = table.get("EnabledFlywheel", false);
			reversed = table.get("Reversed", false);

			distanceMeters = table.get("DistanceMeters", 0.0);
			flywheelSpeedMod = table.get("FlywheelSpeedMod", 0.0);

			targetFlywheelRPM = table.get("TargetFlywheelRPM", 0.0);
		}
	}
}
