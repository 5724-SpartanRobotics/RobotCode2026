package frc.robot.subsystems.indexer;

import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.lib.PIDFfRecord;
import frc.lib.motor.ClosedLoopMotor;
import frc.lib.motor.spark.SparkIO_SparkFlex;

public class IndexerIO_RealAndSim implements IndexerIO {
	private final ClosedLoopMotor motor;

	private double dutySetpoint = 0.0;

	public IndexerIO_RealAndSim(int lowerCanId, int upperCanId) {
		motor = new SparkIO_SparkFlex(lowerCanId);
		motor.applyPidfsva(PIDFfRecord.zero());
		motor.as_SparkIO().applyConfiguration(
			new SparkFlexConfig()
				.apply(new LimitSwitchConfig()
					.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
					.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor))
				.idleMode(IdleMode.kBrake));
	}

	@Override
	public void updateInputs(IndexerIO.IndexerIOInputs inputs) {
		final var m = motor.as_SparkIO().as_IOSparkFlex();
		inputs.positionRotations = m.getEncoder().getPosition();
		inputs.velocityRPS = m.getEncoder().getVelocity() / 60.0;
		inputs.appliedVolts = m.getBusVoltage() * m.getAppliedOutput();
		inputs.busVoltage = m.getBusVoltage();
		inputs.outputCurrentAmps = m.getOutputCurrent();
		inputs.tempCelsius = m.getMotorTemperature();
		inputs.dutyCycle = dutySetpoint;
	}

	@Override
	public void setDutyCycle(double output) {
		dutySetpoint = output;
		motor.as_SparkIO().as_IOSparkFlex().setDutyCycle(dutySetpoint, true);
	}

	@Override
	public void stop() {
		dutySetpoint = 0.0;
		motor.as_SparkIO().as_IOSparkFlex().stopMotor();
	}
}