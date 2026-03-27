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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.LoggedSlewRateLimiter;
import frc.lib.motor.spark.SparkIO_SparkFlex;
import frc.robot.info.constants.CoordinatorConstants;

public class CoordinatorIO_RealAndSim implements CoordinatorIO {
	public final LoggedSlewRateLimiter rateLimiter;
	public final SparkIO_SparkFlex coordinator;
	public final SparkIO_SparkFlex agitator;

	private AngularVelocity velocitySetpoint = Units.RPM.of(0);

	public CoordinatorIO_RealAndSim(int coordinatorCanId, int agitatorCanId) {
		coordinator = new SparkIO_SparkFlex(coordinatorCanId, MotorType.kBrushless);
		coordinator.configure(
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
				.inverted(false),
			ResetMode.kResetSafeParameters,
			PersistMode.kNoPersistParameters);
		agitator = new SparkIO_SparkFlex(agitatorCanId, MotorType.kBrushless);
		agitator.configure(
			new SparkFlexConfig()
				.apply(new LimitSwitchConfig()
					.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
					.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor))
				.apply(new ClosedLoopConfig()
					// TODO: Tune PIDs and Feedforward
					.pid(
						0.0001,
						0,
						0)
					.apply(new FeedForwardConfig()
						.sva(
							0,
							0,
							0))
					.feedbackSensor(FeedbackSensor.kPrimaryEncoder))
				.idleMode(IdleMode.kCoast)
				.inverted(false),
			ResetMode.kResetSafeParameters,
			PersistMode.kNoPersistParameters);

		rateLimiter = new LoggedSlewRateLimiter("Coordinator",
			Units.RotationsPerSecondPerSecond.of(500 * 3).in(Units.RotationsPerSecondPerSecond));
	}

	@Override
	public void updateInputs(CoordinatorIOInputs inputs) {
		inputs.positionRotations = coordinator.getEncoder().getPosition();
		inputs.velocityRPS = coordinator.getEncoder().getVelocity() / 60.0;
		inputs.appliedVolts = coordinator.getBusVoltage() * coordinator.getAppliedOutput();
		inputs.busVoltage = coordinator.getBusVoltage();
		inputs.outputCurrentAmps = coordinator.getOutputCurrent();
		inputs.tempCelsius = coordinator.getMotorTemperature();
		inputs.velocitySetpoint = velocitySetpoint;
	}

	@Override
	public void setVelocity(AngularVelocity output) {
		velocitySetpoint = output;
		coordinator.setVelocity(velocitySetpoint, true, false);
		double rpm = velocitySetpoint.in(Units.RPM);
		rpm = MathUtil.clamp(rpm,
			CoordinatorConstants.AGITATOR_RUN_SETPOINT.times(-1.0).in(Units.RPM),
			CoordinatorConstants.AGITATOR_RUN_SETPOINT.in(Units.RPM));
		agitator.setVelocity(Units.RPM.of(rpm), true, false);
	}

	@Override
	public void stop() {
		velocitySetpoint = Units.RPM.of(0.0);
		coordinator.stopMotor();
		agitator.stopMotor();
	}

	public LoggedSlewRateLimiter getRateLimiter() {
		return rateLimiter;
	}
}
