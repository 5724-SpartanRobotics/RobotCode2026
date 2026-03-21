package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.lib.spark.SparkIO_SparkFlex;

public class IndexerIO_RealAndSim implements IndexerIO {
	private final SparkIO_SparkFlex motor;

	private double dutySetpoint = 0.0;

	public IndexerIO_RealAndSim(int lowerCanId, int upperCanId) {
		motor = new SparkIO_SparkFlex(lowerCanId, MotorType.kBrushless);
		motor.configure(
			new SparkFlexConfig()
				.apply(new LimitSwitchConfig()
					.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
					.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor))
				.apply(new ClosedLoopConfig()
					// TODO: Tune PIDs and Feedforward
					.pid(0, 0, 0)
					.apply(new FeedForwardConfig()
						.sva(0, 0, 0))
					.feedbackSensor(FeedbackSensor.kPrimaryEncoder))
				.idleMode(IdleMode.kBrake),
			ResetMode.kResetSafeParameters,
			PersistMode.kNoPersistParameters);
	}

	@Override
	public void updateInputs(IndexerIO.IndexerIOInputs inputs) {
		inputs.positionRotations = motor.getEncoder().getPosition();
		inputs.velocityRPS = motor.getEncoder().getVelocity() / 60.0;
		inputs.appliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
		inputs.busVoltage = motor.getBusVoltage();
		inputs.outputCurrentAmps = motor.getOutputCurrent();
		inputs.tempCelsius = motor.getMotorTemperature();
		inputs.dutyCycle = dutySetpoint;
	}

	@Override
	public void setDutyCycle(double output) {
		dutySetpoint = output;
		motor.setDutyCycle(dutySetpoint, true);
	}

	@Override
	public void stop() {
		dutySetpoint = 0.0;
		motor.stopMotor();
	}
}