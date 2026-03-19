package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.lib.spark.SparkIO_SparkFlex;

public class IndexerIO_RealAndSim implements IndexerIO {
	private final SparkIO_SparkFlex lowerMotor;

	private final SparkIO_SparkFlex upperMotor;

	private double lower_dutySetpoint = 0.0;
	private double upper_dutySetpoint = 0.0;

	public IndexerIO_RealAndSim(int lowerCanId, int upperCanId) {
		lowerMotor = new SparkIO_SparkFlex(lowerCanId, MotorType.kBrushless);
		upperMotor = new SparkIO_SparkFlex(upperCanId, MotorType.kBrushless);
	}

	@Override
	public void updateInputs(IndexerIO.IndexerIOInputs inputs) {
		inputs.lower_positionRotations = lowerMotor.getEncoder().getPosition();
		inputs.lower_velocityRPS = lowerMotor.getEncoder().getVelocity() / 60.0;
		inputs.lower_appliedVolts = lowerMotor.getBusVoltage() * lowerMotor.getAppliedOutput();
		inputs.lower_busVoltage = lowerMotor.getBusVoltage();
		inputs.lower_outputCurrentAmps = lowerMotor.getOutputCurrent();
		inputs.lower_tempCelsius = lowerMotor.getMotorTemperature();
		inputs.lower_dutyCycle = lower_dutySetpoint;

		inputs.upper_positionRotations = upperMotor.getEncoder().getPosition();
		inputs.upper_velocityRPS = upperMotor.getEncoder().getVelocity() / 60.0;
		inputs.upper_appliedVolts = upperMotor.getBusVoltage() * upperMotor.getAppliedOutput();
		inputs.upper_busVoltage = upperMotor.getBusVoltage();
		inputs.upper_outputCurrentAmps = upperMotor.getOutputCurrent();
		inputs.upper_tempCelsius = upperMotor.getMotorTemperature();
		inputs.upper_dutyCycle = upper_dutySetpoint;
	}

	@Override
	public void setDutyCycle(double output) {
		lower_dutySetpoint = output;
		lowerMotor.setDutyCycle(lower_dutySetpoint, true);

		upper_dutySetpoint = output;
		upperMotor.setDutyCycle(upper_dutySetpoint, true);
	}

	@Override
	public void stop() {
		lower_dutySetpoint = 0.0;
		lowerMotor.stopMotor();

		upper_dutySetpoint = 0.0;
		upperMotor.stopMotor();
	}
}