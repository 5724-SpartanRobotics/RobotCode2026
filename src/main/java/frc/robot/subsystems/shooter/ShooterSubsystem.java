package frc.robot.subsystems.shooter;

import java.util.concurrent.atomic.AtomicBoolean;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NopSubsystemBase;
import frc.robot.info.Debug;
import frc.robot.info.constants.CanIdConstants;
import frc.robot.info.constants.RobotConstants;
import frc.robot.info.constants.ShooterConstants;

public class ShooterSubsystem extends NopSubsystemBase {
	private final ShooterFlywheel m_flywheel;
	private final SparkFlex m_feederMotor;
	private final SparkClosedLoopController m_feederPid;
	private final RelativeEncoder m_feederEncoder;

	private AtomicBoolean m_enable = new AtomicBoolean(false);
	private AtomicBoolean m_reverse = new AtomicBoolean(false);
	private AngularVelocity _feederSetpoint = Units.RPM.of(0);

	public Distance hypotenuseToAllianceHub = Units.Meters.of(0);
	public double flywheelSpeedMod = 1.1;

	private ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();

	private ShooterSubsystem() {
		m_flywheel = new ShooterFlywheel(this);

		m_feederMotor = new SparkFlex(CanIdConstants.SHOOTER_UPPER_FEED, MotorType.kBrushless);
		m_feederMotor.configure(
			new SparkFlexConfig()
				.apply(new LimitSwitchConfig()
					.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
					.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor))
				.apply(new ClosedLoopConfig()
					// TODO: Tune PIDs and Feedforward
					.pid(
						ShooterConstants.FEEDER_PIDF.kP(),
						ShooterConstants.FEEDER_PIDF.kI(),
						ShooterConstants.FEEDER_PIDF.kD())
					.apply(new FeedForwardConfig()
						.sva(
							ShooterConstants.FEEDER_PIDF.kFfS(),
							ShooterConstants.FEEDER_PIDF.kFfV(),
							ShooterConstants.FEEDER_PIDF.kFfA()))
					.feedbackSensor(FeedbackSensor.kPrimaryEncoder))
				.idleMode(IdleMode.kBrake),
			ResetMode.kResetSafeParameters,
			PersistMode.kNoPersistParameters);
		m_feederPid = m_feederMotor.getClosedLoopController();
		m_feederEncoder = m_feederMotor.getEncoder();
	}

	private static final class Holder {
		private static final ShooterSubsystem INSTANCE = new ShooterSubsystem();
	}

	public static ShooterSubsystem getInstance() {
		return Holder.INSTANCE;
	}

	@Override
	public void periodic() {
		m_flywheel.periodic();

		var setpointVelocity = setMotorVelocities();

		log(setpointVelocity);

		if (Debug.DebugLevel.isOrAll(Debug.DebugLevel.Shooter))
			SmartDashboard.putData(this);

		Logger.processInputs("Shooter", inputs);
	}

	private void log(AngularVelocity setpointVelocity) {
		inputs.enabled = m_enable.get();
		inputs.reversed = m_reverse.get();

		inputs.distanceMeters = hypotenuseToAllianceHub.in(Units.Meters);
		inputs.flywheelSpeedMod = flywheelSpeedMod;

		// Feeder
		inputs.feederVelocityRPM = m_feederEncoder.getVelocity();
		inputs.feederRunning = Math.abs(inputs.feederVelocityRPM) > 1.0;

		// You'll need to store this when you calculate it
		inputs.targetFlywheelRPM = setpointVelocity.in(Units.RPM);

		// If you want feeder setpoint, store it when computed:
		AngularVelocity feederSetpoint = setpointVelocity
			.times(ShooterConstants.FLYWHEEL_DIAMETER.div(ShooterConstants.FEEDER_PULLEY_DIAMETER))
			.div(ShooterConstants.FEEDER_SPEED_COEFF)
			.times(m_reverse.get() ? -1.0 : 1.0);

		inputs.feederSetpointRPM = feederSetpoint.in(Units.RPM);
	}

	@Override
	public void simulationPeriodic() {
		m_flywheel.simulationPeriodic();
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType(this.getClass().getName());
		m_flywheel.initSendable(builder);
		builder.addBooleanProperty("Feeder Enabled", () -> {
			return Math.abs((int) m_feederEncoder.getVelocity()) > 0;
		}, null);
		builder.addDoubleProperty("Feeder Velocity RPM", () -> m_feederEncoder.getVelocity(), null);
		builder.addDoubleProperty("Feeder Setpoint RPM", () -> _feederSetpoint.in(Units.RPM), null);
		builder.addDoubleProperty("Flywheel SpeedMod", () -> flywheelSpeedMod,
			(newMod) -> flywheelSpeedMod = newMod);
		builder.addBooleanProperty("Shooter Enabled", () -> m_enable.get(), null);
		builder.addBooleanProperty("Belt Reversed", () -> m_reverse.get(), null);
	}

	private AngularVelocity calculateShooterSpeedFromRobotDistance() {
		Distance copy = hypotenuseToAllianceHub;
		// copy = Units.Meters.of(1);
		double d = copy.in(Units.Meters);
		double g = frc.robot.info.Math.g.in(Units.MetersPerSecondPerSecond);
		double v = Math.sqrt(
			(d * g)
				/
				Math.sin(2.0 * ShooterConstants.LAUNCH_ANGLE.in(Units.Radians))); // projectile
																					// motion range
																					// equation
		AngularVelocity omega = Units.RadiansPerSecond.of(
			v / ShooterConstants.FLYWHEEL_DIAMETER.div(2.0).in(Units.Meters)); // v/r
		double lowVoltageMultiplier = RobotController.getBatteryVoltage()
			/ RobotConstants.NOMINAL_BATTERY_VOLTAGE.in(Units.Volts);
		lowVoltageMultiplier = 1.01 * (1.0 / lowVoltageMultiplier);
		return omega
			.times(lowVoltageMultiplier)
			.times(Math.min(1.0, flywheelSpeedMod))
			.times(ShooterConstants.LAUNCH_VELOCITY_FUDGE_COEFF);
	}

	private AngularVelocity setMotorVelocities() {
		if (!m_enable.get()) {
			m_flywheel.disable();
			m_feederMotor.set(0);
			m_feederMotor.stopMotor();
			return Units.RPM.of(0);
		}

		var velocity = calculateShooterSpeedFromRobotDistance();
		m_flywheel.enable(velocity);
		AngularVelocity feederSetpoint = velocity.times(
			ShooterConstants.FLYWHEEL_DIAMETER.div(ShooterConstants.FEEDER_PULLEY_DIAMETER))
			.div(ShooterConstants.FEEDER_SPEED_COEFF).times(m_reverse.get() ? -1.0 : 1.0);
		if (Debug.DebugLevel.isOrAll(Debug.DebugLevel.Shooter))
			SmartDashboard.putNumber("Feeder Setpoint RPM", feederSetpoint.in(Units.RPM));
		_feederSetpoint = feederSetpoint;
		m_feederPid.setSetpoint(feederSetpoint.in(Units.RPM), ControlType.kVelocity);
		return velocity;
	}

	public void enable() {
		m_reverse.set(false);
		m_enable.set(true);
	}

	public void enableReverse() {
		m_enable.set(true);
		m_reverse.set(true);
	}

	public void disable() {
		m_enable.set(false);
	}

	public Command toggle() {
		return Commands.startEnd(
			() -> enable(),
			() -> disable(),
			this);
	}

	public Command toggleFeederReverse() {
		return Commands.startEnd(
			() -> enableReverse(),
			() -> disable(),
			this);
	}

	public Command runForCommand(Time duration) {
		return Commands.sequence(
			Commands.run(this::enable, this),
			Commands.waitTime(duration),
			Commands.run(this::disable, this));
	}

	public Command enableForeverCommand() {
		var subsystem = this;
		return new Command() {
			private final ShooterSubsystem s;

			{
				s = subsystem;
			}

			@Override
			public void execute() {
				s.enable();
			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end(boolean interrupted) {
				s.disable();
			}
		}.withName("ShootCommand");
	}
}
