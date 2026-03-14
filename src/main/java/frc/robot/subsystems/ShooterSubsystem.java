package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicBoolean;

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

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
	private static ShooterSubsystem instance = null;

	private final ShooterFlywheel m_flywheel;
	private final SparkFlex m_feederMotor;
	private final SparkClosedLoopController m_feederPid;
	private final RelativeEncoder m_feederEncoder;

	private AtomicBoolean m_enable = new AtomicBoolean(false);
	private AtomicBoolean m_reverse = new AtomicBoolean(false);

	public Distance hypotenuseToAllianceHub = Units.Meters.of(0);
	public double flywheelSpeedMod = 1.0;

	private ShooterSubsystem() {
		m_flywheel = new ShooterFlywheel(this);

		m_feederMotor = new SparkFlex(Constants.CanId.SHOOTER_UPPER_FEED, MotorType.kBrushless);
		m_feederMotor.configure(
			new SparkFlexConfig()
				.apply(new LimitSwitchConfig()
					.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
					.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor))
				.apply(new ClosedLoopConfig()
					// TODO: Tune PIDs and Feedforward
					.pid(
						Constants.Shooter.FEEDER_PIDF.kP(),
						Constants.Shooter.FEEDER_PIDF.kI(),
						Constants.Shooter.FEEDER_PIDF.kD())
					.apply(new FeedForwardConfig()
						.sva(
							Constants.Shooter.FEEDER_PIDF.kFfS(),
							Constants.Shooter.FEEDER_PIDF.kFfV(),
							Constants.Shooter.FEEDER_PIDF.kFfA()))
					.feedbackSensor(FeedbackSensor.kPrimaryEncoder))
				.idleMode(IdleMode.kBrake),
			ResetMode.kResetSafeParameters,
			PersistMode.kNoPersistParameters);
		m_feederPid = m_feederMotor.getClosedLoopController();
		m_feederEncoder = m_feederMotor.getEncoder();
	}

	public static void createInstance() {
		getInstance();
	}

	public static ShooterSubsystem getInstance() {
		if (instance == null)
			instance = new ShooterSubsystem();
		return instance;
	}

	@Override
	public void periodic() {
		m_flywheel.periodic();

		setMotorVelocities();

		NetworkTableInstance.getDefault().getEntry("/Shooter Enabled").setBoolean(m_enable.get());
		NetworkTableInstance.getDefault().getEntry("/Belt Reversed").setBoolean(m_reverse.get());

		SmartDashboard.putData(this);
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
		builder.addDoubleProperty("Flywheel SpeedMod", () -> flywheelSpeedMod,
			(newMod) -> flywheelSpeedMod = newMod);
	}

	private AngularVelocity calculateShooterSpeedFromRobotDistance() {
		Distance copy = hypotenuseToAllianceHub;
		// copy = Units.Inches.of(150);
		double d = copy.in(Units.Meters);
		double g = Constants.g.in(Units.MetersPerSecondPerSecond);
		double v = Math.sqrt(
			(d * g)
				/
				Math.sin(2.0 * Constants.Shooter.LAUNCH_ANGLE.in(Units.Radians))); // projectile
																					// motion range
																					// equation
		AngularVelocity omega = Units.RadiansPerSecond.of(
			v / Constants.Shooter.FLYWHEEL_DIAMETER.div(2.0).in(Units.Meters)); // v/r
		double lowVoltageMultiplier = RobotController.getBatteryVoltage()
			/ Units.Volts.of(13.1).in(Units.Volts);
		lowVoltageMultiplier = 1.01 * (1.0 / lowVoltageMultiplier);
		return omega
			.times(lowVoltageMultiplier)
			.times(Math.min(1.0, flywheelSpeedMod))
			.times(Constants.Shooter.LAUNCH_VELOCITY_FUDGE_COEFF);
	}

	private void setMotorVelocities() {
		if (!m_enable.get()) {
			m_flywheel.disable();
			m_feederMotor.set(0);
			m_feederMotor.stopMotor();
			return;
		}

		var velocity = calculateShooterSpeedFromRobotDistance();
		m_flywheel.enable(velocity);
		AngularVelocity feederSetpoint = velocity.times(
			Constants.Shooter.FLYWHEEL_DIAMETER.div(Constants.Shooter.FEEDER_PULLEY_DIAMETER))
			.div(2.75).times(m_reverse.get() ? -1.0 : 1.0);
		if (Constants.DebugLevel.isOrAll(Constants.DebugLevel.Shooter))
			SmartDashboard.putNumber("Feeder Setpoint RPM", feederSetpoint.in(Units.RPM));
		m_feederPid.setSetpoint(feederSetpoint.in(Units.RPM), ControlType.kVelocity);
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
}
