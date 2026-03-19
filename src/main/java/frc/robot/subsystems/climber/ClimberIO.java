package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberIO {
	@AutoLog
	public static class ClimberIOInputs implements LoggableInputs {
		public double appliedVolts = 0.0;
		public double supplyCurrentAmps = 0.0;
		public double statorCurrentAmps = 0.0;
		public double velocityRPS = 0.0;
		public double positionRotations = 0.0;
		public double tempCelsius = 0.0;

		public double setpointVolts = 0.0;
		public boolean isEnabled = false;

		@Override
		public void toLog(LogTable table) {
			table.put("AppliedVolts", appliedVolts);
			table.put("SupplyCurrentAmps", supplyCurrentAmps);
			table.put("StatorCurrentAmps", statorCurrentAmps);
			table.put("VelocityRPS", velocityRPS);
			table.put("PositionRotations", positionRotations);
			table.put("TempCelsius", tempCelsius);

			table.put("SetpointVolts", setpointVolts);
			table.put("IsEnabled", isEnabled);
		}

		@Override
		public void fromLog(LogTable table) {
			appliedVolts = table.get("AppliedVolts", 0.0);
			supplyCurrentAmps = table.get("SupplyCurrentAmps", 0.0);
			statorCurrentAmps = table.get("StatorCurrentAmps", 0.0);
			velocityRPS = table.get("VelocityRPS", 0.0);
			positionRotations = table.get("PositionRotations", 0.0);
			tempCelsius = table.get("TempCelsius", 0.0);

			setpointVolts = table.get("SetpointVolts", 0.0);
			isEnabled = table.get("IsEnabled", false);
		}
	}
}