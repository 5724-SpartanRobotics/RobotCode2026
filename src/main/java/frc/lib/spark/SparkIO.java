package frc.lib.spark;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface SparkIO {
	@AutoLog
	public static class SparkIOInputs implements LoggableInputs {
		public String motorType = "Unknown";
		public double positionRotations = 0.0;
		public double velocityRPS = 0.0;
		public double appliedVolts = 0.0;
		public double busVoltage = 0.0;
		public double outputCurrentAmps = 0.0;
		public double tempCelsius = 0.0;

		@Override
		public void toLog(LogTable table) {
			table.put("MotorType", motorType);
			table.put("PositionRotations", positionRotations);
			table.put("VelocityRPS", velocityRPS);
			table.put("AppliedVolts", appliedVolts);
			table.put("SupplyCurrentAmps", busVoltage);
			table.put("StatorCurrentAmps", outputCurrentAmps);
			table.put("TempCelsius", tempCelsius);
		}

		@Override
		public void fromLog(LogTable table) {
			motorType = table.get("MotorType", motorType);
			positionRotations = table.get("PositionRotations", positionRotations);
			velocityRPS = table.get("VelocityRPS", velocityRPS);
			appliedVolts = table.get("AppliedVolts", appliedVolts);
			busVoltage = table.get("SupplyCurrentAmps", busVoltage);
			outputCurrentAmps = table.get("StatorCurrentAmps", outputCurrentAmps);
			tempCelsius = table.get("TempCelsius", tempCelsius);
		}
	}

	/** Updates the set of loggable inputs */
	default void updateInputs(SparkIOInputs inputs) {
	}

	/** Open-loop [-1, 1] */
	default void set(double speed) {
	}

	default void set(double speed, boolean isClosedLoop) {
	}

	default void setDutyCycle(double speed, boolean isClosedLoop) {
		set(speed, isClosedLoop);
	}

	/** Voltage control */
	default void setVoltage(Voltage volts, boolean isClosedLoop) {
	}

	/** Position (rotations) */
	default void setPosition(Angle angle, boolean isClosedLoop, boolean useMaxMotion) {
	}

	/** Closed-loop velocity */
	default void setVelocity(AngularVelocity velocity, boolean isClosedLoop, boolean useMaxMotion) {
	}
}
