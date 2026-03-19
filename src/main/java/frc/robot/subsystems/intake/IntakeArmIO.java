package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeArmIO {
	@AutoLog
	public static class IntakeArmIOInputs implements LoggableInputs {
		// Position
		public double positionDeg = 0.0;
		public double slavePositionDeg = 0.0;

		// Setpoints
		public double setpointDeg = 0.0;
		public double rampedSetpointDeg = 0.0;

		// Internal control
		public double motorSetpointRotations = 0.0;

		// Electrical
		public double masterCurrentAmps = 0.0;
		public double slaveCurrentAmps = 0.0;

		// Derived
		public double errorDeg = 0.0;
		public boolean atSetpoint = false;

		@Override
		public void toLog(LogTable table) {
			table.put("PositionDeg", positionDeg);
			table.put("SlavePositionDeg", slavePositionDeg);

			table.put("SetpointDeg", setpointDeg);
			table.put("RampedSetpointDeg", rampedSetpointDeg);

			table.put("MotorSetpointRotations", motorSetpointRotations);

			table.put("MasterCurrentAmps", masterCurrentAmps);
			table.put("SlaveCurrentAmps", slaveCurrentAmps);

			table.put("ErrorDeg", errorDeg);
			table.put("AtSetpoint", atSetpoint);
		}

		@Override
		public void fromLog(LogTable table) {
			positionDeg = table.get("PositionDeg", 0.0);
			slavePositionDeg = table.get("SlavePositionDeg", 0.0);

			setpointDeg = table.get("SetpointDeg", 0.0);
			rampedSetpointDeg = table.get("RampedSetpointDeg", 0.0);

			motorSetpointRotations = table.get("MotorSetpointRotations", 0.0);

			masterCurrentAmps = table.get("MasterCurrentAmps", 0.0);
			slaveCurrentAmps = table.get("SlaveCurrentAmps", 0.0);

			errorDeg = table.get("ErrorDeg", 0.0);
			atSetpoint = table.get("AtSetpoint", false);
		}
	}
}
