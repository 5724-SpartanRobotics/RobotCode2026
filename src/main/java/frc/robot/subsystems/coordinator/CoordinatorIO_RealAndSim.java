package frc.robot.subsystems.coordinator;

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

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.LoggedSlewRateLimiter;
import frc.lib.spark.SparkIO_SparkFlex;
import frc.robot.info.constants.CoordinatorConstants;

public class CoordinatorIO_RealAndSim implements CoordinatorIO {
	public final LoggedSlewRateLimiter rateLimiter;
	public final SparkIO_SparkFlex motor;

	private AngularVelocity velocitySetpoint = Units.RPM.of(0);

	public CoordinatorIO_RealAndSim(int canId) {
		motor = new SparkIO_SparkFlex(canId, MotorType.kBrushless);
		motor.configure(
			new SparkFlexConfig()
				.apply(new LimitSwitchConfig()
					.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
					.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor))
				.apply(new ClosedLoopConfig()
					// TODO: Tune PIDs and Feedforward
					.pid(
						CoordinatorConstants.PIDF.kP(),
						CoordinatorConstants.PIDF.kI(),
						CoordinatorConstants.PIDF.kD())
					.apply(new FeedForwardConfig()
						.sva(
							CoordinatorConstants.PIDF.kFfS(),
							CoordinatorConstants.PIDF.kFfV(),
							CoordinatorConstants.PIDF.kFfA()))
					.feedbackSensor(FeedbackSensor.kPrimaryEncoder))
				.idleMode(IdleMode.kCoast)
				.inverted(true),
			ResetMode.kResetSafeParameters,
			PersistMode.kNoPersistParameters);
		rateLimiter = new LoggedSlewRateLimiter("Coordinator",
			Units.RotationsPerSecondPerSecond.of(500 * 3).in(Units.RotationsPerSecondPerSecond));
	}

	@Override
	public void updateInputs(CoordinatorIOInputs inputs) {
		inputs.positionRotations = motor.getEncoder().getPosition();
		inputs.velocityRPS = motor.getEncoder().getVelocity() / 60.0;
		inputs.appliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
		inputs.busVoltage = motor.getBusVoltage();
		inputs.outputCurrentAmps = motor.getOutputCurrent();
		inputs.tempCelsius = motor.getMotorTemperature();
		inputs.velocitySetpoint = velocitySetpoint;
	}

	@Override
	public void setVelocity(AngularVelocity output) {
		velocitySetpoint = output;
		motor.setVelocity(velocitySetpoint, true, false);
	}

	@Override
	public void stop() {
		velocitySetpoint = Units.RPM.of(0.0);
		motor.stopMotor();
	}

	public LoggedSlewRateLimiter getRateLimiter() {
		return rateLimiter;
	}
}
