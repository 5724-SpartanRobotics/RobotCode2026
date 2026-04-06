package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.LoggedSlewRateLimiter;
import frc.lib.motor.ClosedLoopMotor;
import frc.lib.motor.talonfx.TalonFXIO_Wrapper;
import frc.lib.motor.talonfx.TalonFXWrapper;
import frc.robot.info.Debug;
import frc.robot.info.constants.CanIdConstants;
import frc.robot.info.constants.IntakeConstants;
import frc.robot.info.constants.ShooterConstants;
import frc.robot.subsystems.led.LedSubsystem;

public class ShooterFlywheel {
	private static final boolean kIsDebug = Debug.DebugLevel
		.isOrAll(Debug.DebugLevel.Shooter);

	private final LoggedSlewRateLimiter m_rateLimiter;

	// private final SmartMotorControllerConfig smcConfig;
	private final ClosedLoopMotor m_motorLeftLeader;
	private final ClosedLoopMotor m_motorRightFollower;
	// private final SmartMotorController m_smc;
	// private final FlyWheelConfig shooterConfig;
	// private final FlyWheel m_flywheel;

	private boolean shooterEnabled = false;
	private double measuredVelocityLeft = 0;
	private double measuredVelocityRight = 0;
	private AngularVelocity setpointVelocity = Units.RPM.of(0);

	private static final LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/Shooter/P",
		ShooterConstants.SHOOTER_PIDF.kP());

	private static final LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/Shooter/V",
		ShooterConstants.SHOOTER_PIDF.kFfV());

	private double _p = kP.get();
	private double _v = kV.get();

	public ShooterFlywheel() {
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

		m_motorLeftLeader = new TalonFXIO_Wrapper(new TalonFXWrapper(CanIdConstants.SHOOTER_LEFT));
		m_motorLeftLeader.applyPidfsva(ShooterConstants.SHOOTER_PIDF);
		m_motorLeftLeader.as_TalonFXIOWrapper()
			.getWrapper().withConfiguration(new TalonFXConfiguration()
				.withHardwareLimitSwitch(new HardwareLimitSwitchConfigs()
					.withForwardLimitEnable(false)
					.withReverseLimitEnable(false))
				.withCurrentLimits(new CurrentLimitsConfigs()
					.withStatorCurrentLimit(ShooterConstants.MAX_CURRENT)
					.withStatorCurrentLimitEnable(true)))
			.withNeutralMode(NeutralModeValue.Coast)
			.withSlot0Pidf(IntakeConstants.PIDF)
			.withInverted(false);
		m_motorRightFollower = new TalonFXIO_Wrapper(
			new TalonFXWrapper(CanIdConstants.SHOOTER_RIGHT));
		m_motorRightFollower.applyPidfsva(ShooterConstants.SHOOTER_PIDF);
		m_motorRightFollower.as_TalonFXIOWrapper()
			.getWrapper().withConfiguration(new TalonFXConfiguration()
				.withHardwareLimitSwitch(new HardwareLimitSwitchConfigs()
					.withForwardLimitEnable(false)
					.withReverseLimitEnable(false))
				.withCurrentLimits(new CurrentLimitsConfigs()
					.withStatorCurrentLimit(ShooterConstants.MAX_CURRENT)
					.withStatorCurrentLimitEnable(true)))
			.withNeutralMode(NeutralModeValue.Coast)
			.withSlot0Pidf(IntakeConstants.PIDF)
			.withInverted(true)
			.getMotor()
			.setControl(new Follower(CanIdConstants.SHOOTER_RIGHT, MotorAlignmentValue.Opposed));
		/*
		 * m_motorLeftLeader.configure( new SparkFlexConfig() .apply(new LimitSwitchConfig()
		 * .forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
		 * .reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)) .apply(new
		 * ClosedLoopConfig() // TODO: Tune PIDs and Feedforward .pid(
		 * ShooterConstants.SHOOTER_PIDF.kP(), ShooterConstants.SHOOTER_PIDF.kI(),
		 * ShooterConstants.SHOOTER_PIDF.kD()) .apply(new FeedForwardConfig() .sva(
		 * ShooterConstants.SHOOTER_PIDF.kFfS(), ShooterConstants.SHOOTER_PIDF.kFfV(),
		 * ShooterConstants.SHOOTER_PIDF.kFfA())) .feedbackSensor(FeedbackSensor.kPrimaryEncoder))
		 * .apply(new SoftLimitConfig()) .idleMode(IdleMode.kCoast) .smartCurrentLimit((int)
		 * ShooterConstants.MAX_CURRENT.in(Units.Amps)), ResetMode.kResetSafeParameters,
		 * PersistMode.kNoPersistParameters);
		 */
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
		m_rateLimiter = new LoggedSlewRateLimiter("ShooterFlywheel", 3200); // rpm/s
	}

	private static final class Holder {
		private static final ShooterFlywheel INSTANCE = new ShooterFlywheel();
	}

	public static synchronized ShooterFlywheel getInstance() {
		return Holder.INSTANCE;
	}

	public void periodic() {
		// m_flywheel.updateTelemetry();

		final double newKp = kP.get();
		final double newKv = kV.get();
		if (_p != newKp) {
			_p = newKp;
			m_motorLeftLeader.as_TalonFXIOWrapper().getWrapper().applySlot0Config(
				new Slot0Configs().withKP(_p));
		}
		if (_v != newKv) {
			_v = newKv;
			m_motorLeftLeader.as_TalonFXIOWrapper().getWrapper().applySlot0Config(
				new Slot0Configs().withKV(_v));
		}

		if ((int) setpointVelocity.in(Units.RPM) > 0) {
			var limitedVelocity = Units.RPM.of(
				// m_rateLimiter.calculate(setpointVelocity.in(Units.RPM)));
				m_rateLimiter.calculate(100)); // RPM
			m_motorLeftLeader.setVelocity(limitedVelocity);
			m_motorRightFollower.setVelocity(limitedVelocity);
		} else {
			m_motorLeftLeader.setVelocity(Units.RPM.zero());
			m_motorRightFollower.setVelocity(Units.RPM.zero());
		}

		measuredVelocityLeft = getVelocityLeft().abs(Units.RPM);
		measuredVelocityRight = getVelocityRight().abs(Units.RPM);
		shooterEnabled = measuredVelocityLeft > 0.05 || measuredVelocityRight > 0.05;

		if ((shooterEnabled && DriverStation.isDisabled()) ||
			(shooterEnabled && DriverStation.isEnabled()
				&& MathUtil.isNear(setpointVelocity.in(Units.RPM), measuredVelocityLeft, 10.0)))
			LedSubsystem.kInactiveColor = LedSubsystem.kNotification1Color;
		else
			LedSubsystem.kInactiveColor = Color.kBlack;
	}

	public void simulationPeriodic() {
		// m_flywheel.simIterate();
	}

	public void initSendable(SendableBuilder builder) {
		builder.addBooleanProperty("At Setpoint", () -> {
			var setpointRPM = this.setpointVelocity.in(Units.RPM);
			var actualRPM = getAverageSignlessVelocity().in(Units.RPM);
			return MathUtil.isNear(setpointRPM, actualRPM, 200);
		}, null);
		if (kIsDebug) {
			builder.addDoubleProperty("Shooter Setpoint RPM", () -> setpointVelocity.in(Units.RPM),
				null);
			builder.addDoubleProperty("Shooter Velocity Left RPM", () -> measuredVelocityLeft,
				null);
			builder.addDoubleProperty("Shooter Velocity Right RPM", () -> measuredVelocityRight,
				null);
			// builder.addDoubleProperty("Shooter Internal Reference", () -> {
			// return m_motorLeftLeader.getClosedLoopController().getSetpoint();
			// }, null);
		}
	}

	public AngularVelocity getVelocityLeft() {
		return m_motorLeftLeader.as_TalonFXIOWrapper().getMotor().getVelocity().getValue();
	}

	public AngularVelocity getVelocityRight() {
		return m_motorRightFollower.as_TalonFXIOWrapper().getMotor().getVelocity().getValue();
	}

	public AngularVelocity getAverageSignlessVelocity() {
		// AngularVelocityUnit u = Units.RPM;
		// return u.of(l);
		return getVelocityLeft().plus(Units.RPM.of(getVelocityRight().abs(Units.RPM))).div(2.0);
	}

	public void enable(AngularVelocity velocity) {
		setpointVelocity = velocity;
		// m_motor.setVelocity(velocity, true, false);
	}

	public void enableReverse(AngularVelocity velocity) {
		setpointVelocity = velocity.times(-1.0);
		// m_motor.setVelocity(velocity, true, false);
	}

	public void disable() {
		setpointVelocity = Units.RPM.of(0);
		// m_motor.setVelocity(setpointVelocity, true, false);
		m_motorLeftLeader.setVelocity(Units.RPM.zero());
	}
}
