package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIO {
	@AutoLog
	public static class IntakeIOInputs implements LoggableInputs {
		// Arm
		public double armPositionDeg = 0.0;
		public double armSetpointDeg = 0.0;
		public double armLeftCurrentAmps = 0.0;
		public double armRightCurrentAmps = 0.0;

		// Motor outputs (what you're commanding)
		public double onArmPercent = 0.0;

		// Measured (optional but very useful)
		public double onArmVelocityRPM = 0.0;

		// Derived state
		public boolean intakeActive = false;
		public boolean reversed = false;

		@Override
		public void toLog(LogTable table) {
			table.put("ArmPositionDeg", armPositionDeg);
			table.put("ArmSetpointDeg", armSetpointDeg);
			table.put("ArmLeftCurrentAmps", armLeftCurrentAmps);
			table.put("ArmRightCurrentAmps", armRightCurrentAmps);

			table.put("OnArmPercent", onArmPercent);

			table.put("OnArmVelocityRPM", onArmVelocityRPM);

			table.put("IntakeActive", intakeActive);
			table.put("Reversed", reversed);
		}

		@Override
		public void fromLog(LogTable table) {
			armPositionDeg = table.get("ArmPositionDeg", 0.0);
			armSetpointDeg = table.get("ArmSetpointDeg", 0.0);
			armLeftCurrentAmps = table.get("ArmLeftCurrentAmps", 0.0);
			armRightCurrentAmps = table.get("ArmRightCurrentAmps", 0.0);

			onArmPercent = table.get("OnArmPercent", 0.0);

			onArmVelocityRPM = table.get("OnArmVelocityRPM", 0.0);

			intakeActive = table.get("IntakeActive", false);
			reversed = table.get("Reversed", false);
		}
	}
}
