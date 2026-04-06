package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface FeederIO {
	@AutoLog
	public static class FeederIOInputs implements LoggableInputs {
		// State
		public boolean enabledFeeder = false;
		public boolean reversed = false;

		// Feeder
		public double feederVelocityRPM = 0.0;
		public double feederSetpointRPM = 0.0;
		public boolean feederRunning = false;

		@Override
		public void toLog(LogTable table) {
			table.put("EnabledFeeder", enabledFeeder);
			table.put("Reversed", reversed);

			table.put("FeederSetpointRPM", feederSetpointRPM);
			table.put("FeederRunning", feederRunning);
		}

		@Override
		public void fromLog(LogTable table) {
			enabledFeeder = table.get("EnabledFeeder", false);
			reversed = table.get("Reversed", false);

			feederSetpointRPM = table.get("FeederSetpointRPM", 0.0);
			feederRunning = table.get("FeederRunning", false);
		}
	}
}
