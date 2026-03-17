package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.subsystems.LedSubsystem;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterFlywheel {
	private static final boolean kIsDebug = Constants.DebugLevel
		.isOrAll(Constants.DebugLevel.Shooter);

	private final ShooterSubsystem m_subsystem;

	private final SmartMotorControllerConfig smcConfig;
	private final SparkMax m_motor;
	private final SmartMotorController m_smc;
	private final FlyWheelConfig shooterConfig;
	private final FlyWheel m_flywheel;

	private boolean shooterEnabled = false;
	private double measuredVelocity = 0;
	private AngularVelocity setpointVelocity = Units.RPM.of(0);

	public ShooterFlywheel(ShooterSubsystem shooterSubsystem) {
		m_subsystem = shooterSubsystem;

		smcConfig = new SmartMotorControllerConfig(m_subsystem)
			.withControlMode(ControlMode.CLOSED_LOOP)
			// Feedback Constants (PID Constants)
			.withClosedLoopController(
				Constants.Shooter.SHOOTER_PIDF.kP(),
				Constants.Shooter.SHOOTER_PIDF.kI(),
				Constants.Shooter.SHOOTER_PIDF.kD(),
				Constants.Shooter.MAX_VELOCITY,
				Constants.Shooter.MAX_ACCELERATION)
			.withSimClosedLoopController(
				Constants.Shooter.SHOOTER_PIDF.kP(),
				Constants.Shooter.SHOOTER_PIDF.kI(),
				Constants.Shooter.SHOOTER_PIDF.kD(),
				Constants.Shooter.MAX_VELOCITY,
				Constants.Shooter.MAX_ACCELERATION)
			// Feedforward Constants
			.withFeedforward(new SimpleMotorFeedforward(
				Constants.Shooter.SHOOTER_PIDF.kFfS(),
				Constants.Shooter.SHOOTER_PIDF.kFfV(),
				Constants.Shooter.SHOOTER_PIDF.kFfA()))
			.withSimFeedforward(new SimpleMotorFeedforward(
				Constants.Shooter.SHOOTER_PIDF.kFfS(),
				Constants.Shooter.SHOOTER_PIDF.kFfV(),
				Constants.Shooter.SHOOTER_PIDF.kFfA()))
			// Telemetry name and verbosity level
			.withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
			// Gearing from the motor rotor to final shaft.
			// In this example GearBox.fromReductionStages(3,4) is the same as
			// GearBox.fromStages("3:1","4:1") which corresponds to the gearbox
			// attached to your motor.
			// You could also use .withGearing(12) which does the same thing.
			.withGearing(Constants.Shooter.GEAR_RATIO)
			// Motor properties to prevent over currenting.
			.withIdleMode(MotorMode.COAST)
			.withStatorCurrentLimit(Constants.Shooter.MAX_CURRENT);

		m_motor = new SparkMax(Constants.CanId.SHOOTER, MotorType.kBrushless);
		m_smc = new SparkWrapper(
			m_motor,
			DCMotor.getNeoVortex(1),
			smcConfig.withMotorInverted(false));
		shooterConfig = new FlyWheelConfig(m_smc)
			.withDiameter(Constants.Shooter.FLYWHEEL_DIAMETER)
			.withMass(Units.Pounds.of(1))
			.withTelemetry("Shooter",
				kIsDebug ? TelemetryVerbosity.HIGH : TelemetryVerbosity.LOW)
			.withSoftLimit(
				Constants.Shooter.SOFT_LIMIT_VELOCITY.times(-1),
				Constants.Shooter.SOFT_LIMIT_VELOCITY)
			.withSpeedometerSimulation(Constants.Shooter.SOFT_LIMIT_VELOCITY.times(3.0 / 2.0));
		m_flywheel = new FlyWheel(shooterConfig);
	}

	public void periodic() {
		m_flywheel.updateTelemetry();

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
		m_flywheel.simIterate();
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
		AngularVelocityUnit u = Units.RPM;
		double l = m_flywheel.getSpeed().abs(u);
		return u.of(l);
	}

	public void enable(AngularVelocity velocity) {
		setpointVelocity = velocity;
		m_smc.setVelocity(velocity);
	}

	public void disable() {
		setpointVelocity = Units.RPM.of(0);
		m_smc.setVelocity(setpointVelocity);
		m_motor.stopMotor();
	}
}
