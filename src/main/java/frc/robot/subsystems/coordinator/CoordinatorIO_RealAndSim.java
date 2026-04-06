package frc.robot.subsystems.coordinator;

import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.LoggedSlewRateLimiter;
import frc.lib.motor.ClosedLoopMotor;
import frc.lib.motor.spark.SparkIO_SparkFlex;
import frc.robot.info.constants.CoordinatorConstants;

public class CoordinatorIO_RealAndSim implements CoordinatorIO {
	public final LoggedSlewRateLimiter rateLimiter;
	public final ClosedLoopMotor coordinator;
	public final ClosedLoopMotor agitator;

	private AngularVelocity velocitySetpoint = Units.RPM.of(0);

	public CoordinatorIO_RealAndSim(int coordinatorCanId, int agitatorCanId) {
		coordinator = new SparkIO_SparkFlex(coordinatorCanId);
		coordinator.applyPidfsva(CoordinatorConstants.PIDF);
		coordinator.as_SparkIO().applyConfiguration(
			new SparkFlexConfig()
				.apply(new LimitSwitchConfig()
					.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
					.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor))
				.idleMode(IdleMode.kCoast)
				.inverted(false));

		agitator = new SparkIO_SparkFlex(agitatorCanId);
		agitator.applyPidfsva(CoordinatorConstants.AGITATOR_PIDF);
		agitator.as_SparkIO().applyConfiguration(
			new SparkFlexConfig()
				.apply(new LimitSwitchConfig()
					.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
					.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor))
				.idleMode(IdleMode.kCoast)
				.inverted(false));

		rateLimiter = new LoggedSlewRateLimiter("Coordinator",
			Units.RotationsPerSecondPerSecond.of(1500).in(Units.RotationsPerSecondPerSecond));
	}

	@Override
	public void updateInputs(CoordinatorIOInputs inputs) {
		inputs.positionRotations = coordinator.getPosition().in(Units.Rotations);
		inputs.velocityRPS = coordinator.getVelocity().in(Units.RPM) / 60.0;
		inputs.appliedVolts = coordinator.as_SparkIO().as_IOSparkFlex().getBusVoltage()
			* coordinator.as_SparkIO().as_IOSparkFlex().getAppliedOutput();
		inputs.busVoltage = coordinator.as_SparkIO().as_IOSparkFlex().getBusVoltage();
		inputs.outputCurrentAmps = coordinator.as_SparkIO().as_IOSparkFlex().getOutputCurrent();
		inputs.tempCelsius = coordinator.as_SparkIO().as_IOSparkFlex().getMotorTemperature();
		inputs.velocitySetpoint = velocitySetpoint;
	}

	@Override
	public void setVelocity(AngularVelocity output) {
		velocitySetpoint = output;
		coordinator.as_SparkIO().as_IOSparkFlex().setVelocity(velocitySetpoint, true, false);
		double rpm = velocitySetpoint.in(Units.RPM);
		rpm = MathUtil.clamp(rpm,
			CoordinatorConstants.AGITATOR_RUN_SETPOINT.times(-1.0).in(Units.RPM),
			CoordinatorConstants.AGITATOR_RUN_SETPOINT.in(Units.RPM));
		agitator.as_SparkIO().as_IOSparkFlex().setVelocity(Units.RPM.of(rpm), true, false);
	}

	@Override
	public void stop() {
		velocitySetpoint = Units.RPM.of(0.0);
		coordinator.as_SparkIO().as_IOSparkFlex().stopMotor();
		agitator.as_SparkIO().as_IOSparkFlex().stopMotor();
	}

	public LoggedSlewRateLimiter getRateLimiter() {
		return rateLimiter;
	}
}
