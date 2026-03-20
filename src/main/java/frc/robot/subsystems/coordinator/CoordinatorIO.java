package frc.robot.subsystems.coordinator;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public interface CoordinatorIO {
	public static class CoordinatorIOInputs implements LoggableInputs {
		public double positionRotations = 0.0;
		public double velocityRPS = 0.0;
		public double appliedVolts = 0.0;
		public double busVoltage = 0.0;
		public double outputCurrentAmps = 0.0;
		public double tempCelsius = 0.0;
		public AngularVelocity velocitySetpoint = Units.RPM.of(0);

		@Override
		public void toLog(LogTable table) {
			table.put("Position", positionRotations);
			table.put("Velocity", velocityRPS);
			table.put("AppliedVolts", appliedVolts);
			table.put("BusVoltage", busVoltage);
			table.put("Current", outputCurrentAmps);
			table.put("TempCelcius", tempCelsius);
			table.put("VelocitySetpointRPM", velocitySetpoint.in(Units.RPM));
		}

		@Override
		public void fromLog(LogTable table) {
			positionRotations = table.get("Position", 0.0);
			velocityRPS = table.get("Velocity", 0.0);
			appliedVolts = table.get("AppliedVolts", 0.0);
			busVoltage = table.get("BusVoltage", 0.0);
			outputCurrentAmps = table.get("Current", 0.0);
			tempCelsius = table.get("TempCelcius", 0.0);
			velocitySetpoint = Units.RPM.of(table.get("VelocitySetpointRPM", 0.0));
		}
	}

	default void updateInputs(CoordinatorIOInputs inputs) {
	}

	default void setVelocity(AngularVelocity output) {
	}

	default void stop() {
	}
}
