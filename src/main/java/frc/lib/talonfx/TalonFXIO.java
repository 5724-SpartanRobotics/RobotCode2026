package frc.lib.talonfx;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface TalonFXIO {

	/** Container for all sensor inputs */
	@AutoLog
	public static class TalonFXIOInputs implements LoggableInputs {
		public double positionRotations = 0.0;
		public double velocityRPS = 0.0;
		public double appliedVolts = 0.0;
		public double supplyCurrentAmps = 0.0;
		public double statorCurrentAmps = 0.0;
		public double tempCelsius = 0.0;

		@Override
		public void toLog(LogTable table) {
			table.put("PositionRotations", positionRotations);
			table.put("VelocityRPS", velocityRPS);
			table.put("AppliedVolts", appliedVolts);
			table.put("SupplyCurrentAmps", supplyCurrentAmps);
			table.put("StatorCurrentAmps", statorCurrentAmps);
			table.put("TempCelsius", tempCelsius);
		}

		@Override
		public void fromLog(LogTable table) {
			positionRotations = table.get("PositionRotations", positionRotations);
			velocityRPS = table.get("VelocityRPS", velocityRPS);
			appliedVolts = table.get("AppliedVolts", appliedVolts);
			supplyCurrentAmps = table.get("SupplyCurrentAmps", supplyCurrentAmps);
			statorCurrentAmps = table.get("StatorCurrentAmps", statorCurrentAmps);
			tempCelsius = table.get("TempCelsius", tempCelsius);
		}
	}

	/** Updates the set of loggable inputs */
	default void updateInputs(TalonFXIOInputs inputs) {
	}

	/** Open-loop [-1, 1] */
	default void set(double speed) {
	}

	/** Voltage control */
	default void setVoltage(Voltage volts) {
	}

	/** Position (rotations) */
	default void setPosition(Angle angle) {
	}

	/** Closed-loop velocity */
	default void setVelocity(AngularVelocity velocity) {
	}
}