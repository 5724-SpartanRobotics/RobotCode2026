package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.LoggedSlewRateLimiter;
import frc.lib.spark.SparkIO_SparkFlex;
import frc.robot.info.Debug;
import frc.robot.info.constants.CanIdConstants;
import frc.robot.info.constants.ShooterConstants;
import frc.robot.subsystems.led.LedSubsystem;

public class ShooterFlywheel {
	private static final boolean kIsDebug = Debug.DebugLevel
		.isOrAll(Debug.DebugLevel.Shooter);

	private final ShooterSubsystem m_subsystem;
	private final LoggedSlewRateLimiter m_rateLimiter;

	// private final SmartMotorControllerConfig smcConfig;
	private final SparkIO_SparkFlex m_motor;
	// private final SmartMotorController m_smc;
	// private final FlyWheelConfig shooterConfig;
	// private final FlyWheel m_flywheel;

	private boolean shooterEnabled = false;
	private double measuredVelocity = 0;
	private AngularVelocity setpointVelocity = Units.RPM.of(0);

	public ShooterFlywheel(ShooterSubsystem shooterSubsystem) {
		m_subsystem = shooterSubsystem;

		// smcConfig = new SmartMotorControllerConfig(m_subsystem)
		// .withControlMode(ControlMode.CLOSED_LOOP)
		// // Feedback Constants (PID Constants)
		// .withClosedLoopController(
		// ShooterConstants.SHOOTER_PIDF.kP(),
		// ShooterConstants.SHOOTER_PIDF.kI(),
		// ShooterConstants.SHOOTER_PIDF.kD(),
		// ShooterConstants.MAX_VELOCITY,
		// ShooterConstants.MAX_ACCELERATION)
		// .withSimClosedLoopController(
		// ShooterConstants.SHOOTER_PIDF.kP(),
		// ShooterConstants.SHOOTER_PIDF.kI(),
		// ShooterConstants.SHOOTER_PIDF.kD(),
		// ShooterConstants.MAX_VELOCITY,
		// ShooterConstants.MAX_ACCELERATION)
		// // Feedforward Constants
		// .withFeedforward(new SimpleMotorFeedforward(
		// ShooterConstants.SHOOTER_PIDF.kFfS(),
		// ShooterConstants.SHOOTER_PIDF.kFfV(),
		// ShooterConstants.SHOOTER_PIDF.kFfA()))
		// .withSimFeedforward(new SimpleMotorFeedforward(
		// ShooterConstants.SHOOTER_PIDF.kFfS(),
		// ShooterConstants.SHOOTER_PIDF.kFfV(),
		// ShooterConstants.SHOOTER_PIDF.kFfA()))
		// // Telemetry name and verbosity level
		// .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
		// // Gearing from the motor rotor to final shaft.
		// // In this example GearBox.fromReductionStages(3,4) is the same as
		// // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox
		// // attached to your motor.
		// // You could also use .withGearing(12) which does the same thing.
		// .withGearing(ShooterConstants.GEAR_RATIO)
		// // Motor properties to prevent over currenting.
		// .withIdleMode(MotorMode.COAST)
		// .withStatorCurrentLimit(ShooterConstants.MAX_CURRENT);

		m_motor = new SparkIO_SparkFlex(CanIdConstants.SHOOTER, MotorType.kBrushless);
		m_motor.configure(
			new SparkFlexConfig()
				.apply(new LimitSwitchConfig()
					.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
					.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor))
				.apply(new ClosedLoopConfig()
					// TODO: Tune PIDs and Feedforward
					.pid(
						ShooterConstants.SHOOTER_PIDF.kP(),
						ShooterConstants.SHOOTER_PIDF.kI(),
						ShooterConstants.SHOOTER_PIDF.kD())
					.apply(new FeedForwardConfig()
						.sva(
							ShooterConstants.SHOOTER_PIDF.kFfS(),
							ShooterConstants.SHOOTER_PIDF.kFfV(),
							ShooterConstants.SHOOTER_PIDF.kFfA()))
					.feedbackSensor(FeedbackSensor.kPrimaryEncoder))
				.apply(new SoftLimitConfig())
				.idleMode(IdleMode.kCoast)
				.smartCurrentLimit((int) ShooterConstants.MAX_CURRENT.in(Units.Amps)),
			ResetMode.kResetSafeParameters,
			PersistMode.kNoPersistParameters);
		// m_smc = new SparkWrapper(
		// m_motor,
		// DCMotor.getNeoVortex(1),
		// smcConfig.withMotorInverted(false));
		// shooterConfig = new FlyWheelConfig(m_smc)
		// .withDiameter(ShooterConstants.FLYWHEEL_DIAMETER)
		// .withMass(Units.Pounds.of(1))
		// .withTelemetry("Shooter",
		// kIsDebug ? TelemetryVerbosity.HIGH : TelemetryVerbosity.LOW)
		// .withSoftLimit(
		// ShooterConstants.SOFT_LIMIT_VELOCITY.times(-1),
		// ShooterConstants.SOFT_LIMIT_VELOCITY)
		// .withSpeedometerSimulation(ShooterConstants.SOFT_LIMIT_VELOCITY.times(3.0 / 2.0));
		// m_flywheel = new FlyWheel(shooterConfig);
		m_rateLimiter = new LoggedSlewRateLimiter("ShooterFlywheel", 2500); // rpm/s
	}

	public void periodic() {
		// m_flywheel.updateTelemetry();

		if ((int) setpointVelocity.in(Units.RPM) > 0)
			m_motor.setVelocity(Units.RPM.of(
				m_rateLimiter.calculate(setpointVelocity.in(Units.RPM))),
				true, false);
		else
			m_motor.stopMotor();

		double setpointVelocityPercent = m_motor.getClosedLoopController().getSetpoint();
		measuredVelocity = getVelocity().abs(Units.RPM);
		shooterEnabled = measuredVelocity > 0.05;

		if ((shooterEnabled && DriverStation.isDisabled()) ||
			(shooterEnabled && DriverStation.isEnabled()
				&& MathUtil.isNear(setpointVelocityPercent, measuredVelocity, 10.0)))
			LedSubsystem.kInactiveColor = LedSubsystem.kNotification1Color;
		else
			LedSubsystem.kInactiveColor = Color.kBlack;
	}

	public void simulationPeriodic() {
		// m_flywheel.simIterate();
	}

	public void initSendable(SendableBuilder builder) {
		builder.addBooleanProperty("Shooter Enabled", () -> shooterEnabled, null);
		if (kIsDebug) {
			builder.addDoubleProperty("Shooter Setpoint RPM", () -> setpointVelocity.in(Units.RPM),
				null);
			builder.addDoubleProperty("Shooter Velocity RPM", () -> measuredVelocity, null);
			builder.addDoubleProperty("Shooter Internal Reference", () -> {
				return m_motor.getClosedLoopController().getSetpoint();
			}, null);
		}
	}

	public AngularVelocity getVelocity() {
		return Units.RPM.of(m_motor.getEncoder().getVelocity());
	}

	public AngularVelocity getSignlessVelocity() {
		// AngularVelocityUnit u = Units.RPM;
		double l = m_motor.getEncoder().getVelocity();
		// return u.of(l);
		return Units.RPM.of(l);
	}

	public void enable(AngularVelocity velocity) {
		setpointVelocity = velocity;
		// m_motor.setVelocity(velocity, true, false);
	}

	public void disable() {
		setpointVelocity = Units.RPM.of(0);
		// m_motor.setVelocity(setpointVelocity, true, false);
		m_motor.stopMotor();
	}
}
