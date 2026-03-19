package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface RobotIO {
	@AutoLog
	public static class RobotIOInputs implements LoggableInputs {
		// Battery / power
		public double batteryVoltage = 0.0;
		public boolean brownedOut = false;

		// RIO stats
		public double cpuTempCelsius = 0.0;

		// CAN
		public double canUtilization = 0.0;
		public int canTxFullCount = 0;
		public int canReceiveErrorCount = 0;
		public int canTransmitErrorCount = 0;

		// Timing
		public double fpgaTimeSeconds = 0.0;

		// DS
		public boolean dsAttached = false;
		public boolean enabled = false;
		public boolean autonomous = false;
		public boolean teleop = false;

		@Override
		public void toLog(LogTable table) {
			table.put("BatteryVoltage", batteryVoltage);
			table.put("BrownedOut", brownedOut);
			table.put("CpuTempCelsius", cpuTempCelsius);
			table.put("CanUtilization", canUtilization);
			table.put("CanTxFullCount", canTxFullCount);
			table.put("CanReceiveErrorCount", canReceiveErrorCount);
			table.put("CanTransmitErrorCount", canTransmitErrorCount);
			table.put("FpgaTimeSeconds", fpgaTimeSeconds);
			table.put("DsAttached", dsAttached);
			table.put("Enabled", enabled);
			table.put("Autonomous", autonomous);
			table.put("Teleop", teleop);
		}

		@Override
		public void fromLog(LogTable table) {
			batteryVoltage = table.get("BatteryVoltage", 0);
			brownedOut = table.get("BrownedOut", false);
			cpuTempCelsius = table.get("CpuTempCelsius", 0);
			canUtilization = table.get("CanUtilization", 0);
			canTxFullCount = table.get("CanTxFullCount", 0);
			canReceiveErrorCount = table.get("CanReceiveErrorCount", 0);
			canTransmitErrorCount = table.get("CanTransmitErrorCount", 0);
			fpgaTimeSeconds = table.get("FpgaTimeSeconds", 0);
			dsAttached = table.get("DsAttached", false);
			enabled = table.get("Enabled", false);
			autonomous = table.get("Autonomous", false);
			teleop = table.get("Teleop", false);
		}
	}

	default void updateInputs(RobotIOInputs inputs) {
	}
}