package frc.robot.subsystems.feeder;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.NopSubsystemBase;
import frc.lib.motor.ClosedLoopMotor;
import frc.lib.motor.spark.SparkIO_SparkFlex;
import frc.robot.info.Debug;
import frc.robot.info.Motors;
import frc.robot.info.constants.CanIdConstants;
import frc.robot.info.constants.ShooterConstants;

public class FeederSubsystem extends NopSubsystemBase {
	private final ClosedLoopMotor m_motor;
	// private final RelativeEncoder m_encoder;

	private final AtomicReference<AngularVelocity> _setpoint = new AtomicReference<>(
		Units.RPM.of(0));
	private final AtomicBoolean m_enable = new AtomicBoolean(false);
	private final AtomicBoolean m_reverse = new AtomicBoolean(false);

	private static final LoggedNetworkNumber kSpeedRPM = new LoggedNetworkNumber(
		"/Tuning/Feeder/RPM", 1000);

	private FeederIO.FeederIOInputs inputs = new FeederIO.FeederIOInputs();

	private FeederSubsystem() {
		m_motor = new SparkIO_SparkFlex(CanIdConstants.SHOOTER_UPPER_FEED);
		m_motor.applyPidfsva(ShooterConstants.FEEDER_PIDF);
		m_motor.as_SparkIO().applyConfiguration(
			new SparkFlexConfig()
				.apply(new LimitSwitchConfig()
					.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
					.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor))
				.apply(new ClosedLoopConfig().feedbackSensor(FeedbackSensor.kPrimaryEncoder))
				.idleMode(IdleMode.kBrake)
				.inverted(true));
	}

	private static final class Holder {
		private static final FeederSubsystem INSTANCE = new FeederSubsystem();
	}

	public static synchronized FeederSubsystem getInstance() {
		return Holder.INSTANCE;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType(this.getClass().getName());
		builder.addBooleanProperty("Feeder Enabled", () -> m_enable.get(), null);
		builder.addBooleanProperty("Feeder Reversed", () -> m_reverse.get(), null);
		if (Debug.DebugLevel.isOrAll(Debug.DebugLevel.Shooter)) {
			builder.addBooleanProperty("Feeder Velocity >0",
				() -> (int) m_motor.getVelocity().in(Units.RPM) > 0, null);
			builder.addDoubleProperty("Feeder Velocity RPM",
				() -> m_motor.getVelocity().in(Units.RPM), null);
			builder.addDoubleProperty("Feeder Setpoint RPM", () -> _setpoint.get().in(Units.RPM),
				null);
		}
	}

	@Override
	public void periodic() {
		log();
		Logger.processInputs("Feeder", inputs);

		SmartDashboard.putData(this);
	}

	private void log() {
		inputs.enabledFeeder = false;
		inputs.reversed = m_reverse.get();

		// Feeder
		inputs.feederVelocityRPM = m_motor.getVelocity().in(Units.RPM);
		inputs.feederVelocityRPM = 0;
		inputs.feederRunning = Math.abs(inputs.feederVelocityRPM) > 1.0;

		inputs.feederSetpointRPM = _setpoint.get().in(Units.RPM);
	}

	public void enableForward() {
		m_reverse.set(false);
		m_enable.set(true);
		AngularVelocity feederSetpoint = Units.RPM.of(kSpeedRPM.get())
			.times(ShooterConstants.FEEDER_GEAR_RATIO);
		feederSetpoint = Units.RPM.of(
			MathUtil.clamp(feederSetpoint.in(Units.RPM), 0,
				Motors.VORTEX_MAX_VELOCITY.in(Units.RPM)));
		_setpoint.set(feederSetpoint);
		((SparkIO_SparkFlex) m_motor).setVelocity(feederSetpoint, true, false);
	}

	public void enableReverse() {
		m_reverse.set(true);
		m_enable.set(true);
		AngularVelocity feederSetpoint = Units.RPM.of(kSpeedRPM.get())
			.times(ShooterConstants.FEEDER_GEAR_RATIO)
			.times(-1.0);
		feederSetpoint = Units.RPM.of(
			MathUtil.clamp(feederSetpoint.in(Units.RPM),
				Motors.VORTEX_MAX_VELOCITY.times(-1.0).in(Units.RPM), 0));
		_setpoint.set(feederSetpoint);
		((SparkIO_SparkFlex) m_motor).setVelocity(feederSetpoint, true, false);
	}

	public void disable() {
		m_enable.set(false);
		m_reverse.set(false);
		((SparkIO_SparkFlex) m_motor).stopMotor();
	}

	public void setReverse(boolean reverse) {
		m_reverse.set(reverse);
	}

	public Command enableCommand() {
		return runOnce(this::enableForward);
	}

	public Command enableForeverCommand() {
		return run(this::enableForward);
	}

	public Command disableCommand() {
		return runOnce(this::disable);
	}
}
