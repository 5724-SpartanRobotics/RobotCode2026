package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IndexerIO {
	@AutoLog
	public static class IndexerIOInputs implements LoggableInputs {
		public double lower_positionRotations = 0.0;
		public double lower_velocityRPS = 0.0;
		public double lower_appliedVolts = 0.0;
		public double lower_busVoltage = 0.0;
		public double lower_outputCurrentAmps = 0.0;
		public double lower_tempCelsius = 0.0;
		public double lower_dutyCycle = 0.0;

		public double upper_positionRotations = 0.0;
		public double upper_velocityRPS = 0.0;
		public double upper_appliedVolts = 0.0;
		public double upper_busVoltage = 0.0;
		public double upper_outputCurrentAmps = 0.0;
		public double upper_tempCelsius = 0.0;
		public double upper_dutyCycle = 0.0;

		@Override
		public void toLog(LogTable table) {
			table.put("Lower/Position", lower_positionRotations);
			table.put("Lower/Velocity", lower_velocityRPS);
			table.put("Lower/AppliedVolts", lower_appliedVolts);
			table.put("Lower/BusVoltage", lower_busVoltage);
			table.put("Lower/Current", lower_outputCurrentAmps);
			table.put("Lower/Temp", lower_tempCelsius);
			table.put("Lower/DutyCycle", lower_dutyCycle);

			table.put("Upper/Position", upper_positionRotations);
			table.put("Upper/Velocity", upper_velocityRPS);
			table.put("Upper/AppliedVolts", upper_appliedVolts);
			table.put("Upper/BusVoltage", upper_busVoltage);
			table.put("Upper/Current", upper_outputCurrentAmps);
			table.put("Upper/Temp", upper_tempCelsius);
			table.put("Upper/DutyCycle", upper_dutyCycle);
		}

		@Override
		public void fromLog(LogTable table) {
			lower_positionRotations = table.get("Lower/Position", 0.0);
			lower_velocityRPS = table.get("Lower/Velocity", 0.0);
			lower_appliedVolts = table.get("Lower/AppliedVolts", 0.0);
			lower_busVoltage = table.get("Lower/BusVoltage", 0.0);
			lower_outputCurrentAmps = table.get("Lower/Current", 0.0);
			lower_tempCelsius = table.get("Lower/Temp", 0.0);
			lower_dutyCycle = table.get("Lower/DutyCycle", 0.0);

			upper_positionRotations = table.get("Upper/Position", 0.0);
			upper_velocityRPS = table.get("Upper/Velocity", 0.0);
			upper_appliedVolts = table.get("Upper/AppliedVolts", 0.0);
			upper_busVoltage = table.get("Upper/BusVoltage", 0.0);
			upper_outputCurrentAmps = table.get("Upper/Current", 0.0);
			upper_tempCelsius = table.get("Upper/Temp", 0.0);
			upper_dutyCycle = table.get("Upper/DutyCycle", 0.0);
		}
	}

	default void updateInputs(IndexerIOInputs inputs) {
	}

	default void setDutyCycle(double output) {
	}

	default void stop() {
	}
}
