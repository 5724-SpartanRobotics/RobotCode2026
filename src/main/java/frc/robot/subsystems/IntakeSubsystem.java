package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
	private static IntakeSubsystem instance = null;

	/** NEO Vortex on SparkFlex */
	private final SparkFlex m_onArmIntake;
	private double onArmIntakeSpeedReference = 0;
	/** Redline on SparkMax */
	private final SparkMax m_lowerFixedIntake;
	private double lowerIntakeSpeedReference = 0;
	/** NEO Vortex on SparkFlex */
	private final SparkFlex m_upperFixedIntake;
	private double upperIntakeSpeedReference = 0;

	private final IntakeArm m_arm = IntakeArm.getInstance();

	private IntakeSubsystem() {
		m_onArmIntake = new SparkFlex(Constants.CanId.INTAKE_ON_ARM, MotorType.kBrushless);
		m_onArmIntake.configure(
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

		m_lowerFixedIntake = new SparkMax(Constants.CanId.INTAKE_LOWER_FIXED, MotorType.kBrushed);
		m_lowerFixedIntake.configure(
			new SparkFlexConfig()
				.apply(new LimitSwitchConfig()
					.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
					.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor))
				// I don't think we need a Closed Loop controller because this
				// is just movement, no setpoints.
				.inverted(true)
				.idleMode(IdleMode.kBrake),
			ResetMode.kResetSafeParameters,
			PersistMode.kNoPersistParameters);

		m_upperFixedIntake = new SparkFlex(Constants.CanId.INTAKE_UPPER_FIXED,
			MotorType.kBrushless);
		m_upperFixedIntake.configure(
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

	public static void createInstance() {
		getInstance();
	}

	public static IntakeSubsystem getInstance() {
		if (instance == null)
			instance = new IntakeSubsystem();
		return instance;
	}

	@Override
	public void periodic() {
		m_arm.periodic();

		if (Constants.DebugLevel.isOrAll(Constants.DebugLevel.Intake))
			SmartDashboard.putData(this);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType(this.getClass().getName());
		m_arm.initSendable(builder);
		builder.addDoubleProperty(
			"ArmPositionDeg",
			() -> m_arm.getAngle().in(Units.Degrees), null);
		builder.addDoubleProperty(
			"ArmSetpointDeg",
			() -> m_arm.getSetpoint().in(Units.Degrees), null);
		builder.addDoubleProperty(
			"ArmLeftCurrentAmps",
			() -> m_arm.getMasterOutputCurrent().in(Units.Amps), null);
		builder.addDoubleProperty(
			"ArmRightCurrentAmps",
			() -> m_arm.getSlaveOutputCurrent().in(Units.Amps), null);
		builder.addDoubleProperty(
			"IntakeOnArmSpeedPercent",
			() -> onArmIntakeSpeedReference, null);
		builder.addDoubleProperty(
			"IntakeLowerFixedSpeedPercent",
			() -> lowerIntakeSpeedReference, null);
		builder.addDoubleProperty(
			"IntakeUpperFixedSpeedPercent",
			() -> upperIntakeSpeedReference, null);
	}

	private static double calculateLowerReferenceInRelationToArmIntake(double setpoint) {
		final double speedMod = 1.0 / 65.0;
		AngularVelocity upperVelocity = Constants.Motors.VORTEX_MAX_VELOCITY.times(setpoint);
		AngularVelocity otherSide = upperVelocity.div(Constants.Intake.ON_ARM_GEAR_RATIO);
		double linearSpeedSurfaceSpeedInchesPerMinute = otherSide.in(Units.RPM) *
			Constants.Intake.UPPER_WHEEL_CURCUMFERENCE.in(Units.Inches);
		double lowerMotorRpm = linearSpeedSurfaceSpeedInchesPerMinute *
			Constants.Intake.LOWER_WHEEL_CURCUMFERENCE.in(Units.Inches);
		double lowerRpmWithReducer = lowerMotorRpm * Constants.Intake.LOWER_FIXED_GEAR_RATIO;
		double lowerReference = lowerRpmWithReducer /
			Constants.Motors.REDLINE_MAX_VELOCITY.in(Units.RPM);
		return lowerReference * speedMod;
	}

	private static double calculateUpperReferenceInRelationToArmIntake(double setpoint) {
		return setpoint / Constants.Intake.UPPER_FIXED_GEAR_RATIO;
	}

	public void extendArm() {
		m_arm.rotateOut();
	}

	public void retractArm() {
		m_arm.rotateIn();
	}

	public void enableIntake() {
		final double speed = Constants.Intake.SPEED.in(Units.Value); // Value gives n/100
		onArmIntakeSpeedReference = speed;
		lowerIntakeSpeedReference = calculateLowerReferenceInRelationToArmIntake(speed);
		upperIntakeSpeedReference = calculateUpperReferenceInRelationToArmIntake(speed);
		m_onArmIntake.set(onArmIntakeSpeedReference);
		m_upperFixedIntake.set(upperIntakeSpeedReference);
		m_lowerFixedIntake.set(lowerIntakeSpeedReference);
	}

	public void enableSpitout() {
		final double speed = Constants.Intake.SPEED.times(-1.0).in(Units.Value); // of 100, not 1
		onArmIntakeSpeedReference = speed;
		lowerIntakeSpeedReference = calculateLowerReferenceInRelationToArmIntake(speed);
		upperIntakeSpeedReference = calculateUpperReferenceInRelationToArmIntake(speed);
		m_onArmIntake.set(onArmIntakeSpeedReference);
		m_upperFixedIntake.set(upperIntakeSpeedReference);
		m_lowerFixedIntake.set(lowerIntakeSpeedReference);
	}

	public void enableReverse() {
		final double speed = Constants.Intake.SPEED.times(-1.0).in(Units.Value);
		onArmIntakeSpeedReference = speed;
		lowerIntakeSpeedReference = calculateLowerReferenceInRelationToArmIntake(speed);
		upperIntakeSpeedReference = calculateUpperReferenceInRelationToArmIntake(speed);
		m_onArmIntake.set(onArmIntakeSpeedReference);
		m_upperFixedIntake.set(upperIntakeSpeedReference);
		m_lowerFixedIntake.set(lowerIntakeSpeedReference);
	}

	public void disableIntake() {
		onArmIntakeSpeedReference = lowerIntakeSpeedReference = upperIntakeSpeedReference = 0;
		m_onArmIntake.set(onArmIntakeSpeedReference);
		m_upperFixedIntake.set(upperIntakeSpeedReference);
		m_lowerFixedIntake.set(lowerIntakeSpeedReference);
	}

	public Command toggleIntake() {
		return Commands.startEnd(this::enableIntake, this::disableIntake, this);
	}

	public Command toggleArm() {
		return Commands.startEnd(this::extendArm, this::retractArm, this);
	}

	public Command extendArmCommand() {
		return Commands.run(this::extendArm, this);
	}

	public Command retractArmCommand() {
		return Commands.run(this::retractArm, this);
	}

	public Command incrementArmCommand() {
		return Commands.runOnce(m_arm::increment, this);
	}

	public Command decrementArmCommand() {
		return Commands.runOnce(m_arm::decrement, this);
	}

	public Command stopArm() {
		return Commands.run(m_arm::stop, this);
	}

	public Command toggleAll() {
		return Commands.startEnd(
			() -> {
				extendArm();
				enableIntake();
			},
			() -> {
				disableIntake();
				retractArm();
			},
			this);
	}

	public Command runForCommand(Time duration) {
		return Commands.sequence(
			Commands.run(() -> {
				extendArm();
				enableIntake();
			}, this),
			Commands.waitTime(duration),
			Commands.run(() -> {
				disableIntake();
				retractArm();
			}, this));
	}
}
