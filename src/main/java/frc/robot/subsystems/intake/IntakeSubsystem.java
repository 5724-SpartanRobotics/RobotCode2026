package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NopSubsystemBase;
import frc.lib.spark.SparkIO_SparkFlex;
import frc.lib.spark.SparkIO_SparkMax;
import frc.robot.info.Debug;
import frc.robot.info.Motors;
import frc.robot.info.constants.CanIdConstants;
import frc.robot.info.constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends NopSubsystemBase {
	/** NEO Vortex on SparkFlex */
	private final SparkIO_SparkFlex m_onArmIntake;
	private double onArmIntakeSpeedReference = 0;
	/** Redline on SparkMax */
	private final SparkIO_SparkMax m_lowerFixedIntake;
	private double lowerIntakeSpeedReference = 0;
	/** NEO Vortex on SparkFlex */
	private final SparkIO_SparkFlex m_upperFixedIntake;
	private double upperIntakeSpeedReference = 0;

	private final IntakeArm m_arm = IntakeArm.getInstance();

	private final IntakeIOInputs inputs = new IntakeIOInputs();

	private IntakeSubsystem() {
		m_onArmIntake = new SparkIO_SparkFlex(CanIdConstants.INTAKE_ON_ARM, MotorType.kBrushless);
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

		m_lowerFixedIntake = new SparkIO_SparkMax(CanIdConstants.INTAKE_LOWER_FIXED,
			MotorType.kBrushed);
		m_lowerFixedIntake.configure(
			new SparkFlexConfig()
				.apply(new LimitSwitchConfig()
					.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
					.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor))
				.inverted(true)
				.idleMode(IdleMode.kBrake),
			ResetMode.kResetSafeParameters,
			PersistMode.kNoPersistParameters);

		m_upperFixedIntake = new SparkIO_SparkFlex(CanIdConstants.INTAKE_UPPER_FIXED,
			MotorType.kBrushless);
		m_upperFixedIntake.configure(
			new SparkFlexConfig()
				.apply(new LimitSwitchConfig()
					.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
					.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor))
				.apply(new ClosedLoopConfig()
					// TODO: Tune PIDs and Feedforward
					.pid(0.25, 0, 0)
					.apply(new FeedForwardConfig()
						.sva(0, 0, 0))
					.feedbackSensor(FeedbackSensor.kPrimaryEncoder))
				.idleMode(IdleMode.kBrake),
			ResetMode.kResetSafeParameters,
			PersistMode.kNoPersistParameters);
	}

	private static final class Holder {
		private static final IntakeSubsystem INSTANCE = new IntakeSubsystem();
	}

	public static synchronized IntakeSubsystem getInstance() {
		return Holder.INSTANCE;
	}

	@Override
	public void periodic() {
		m_arm.periodic();

		log();

		if (Debug.DebugLevel.isOrAll(Debug.DebugLevel.Intake))
			SmartDashboard.putData(this);

		Logger.processInputs("Intake", inputs);
		Logger.processInputs("IntakeArm", m_arm.getInputs());

		Logger.recordOutput(
			"Intake/AI_JamDetected",
			inputs.intakeActive &&
				Math.abs(inputs.onArmVelocityRPM) < 100 &&
				Math.abs(inputs.onArmPercent) > 0.3);

		Logger.recordOutput(
			"Intake/AI_SpeedMismatch",
			inputs.onArmVelocityRPM - inputs.upperVelocityRPM);

		Logger.recordOutput(
			"Intake/AI_ArmErrorDeg",
			inputs.armSetpointDeg - inputs.armPositionDeg);
	}

	public void log() {
		inputs.armPositionDeg = m_arm.getAngle().in(Units.Degrees);
		inputs.armSetpointDeg = m_arm.getSetpoint().in(Units.Degrees);
		inputs.armLeftCurrentAmps = m_arm.getMasterOutputCurrent().in(Units.Amps);
		inputs.armRightCurrentAmps = m_arm.getSlaveOutputCurrent().in(Units.Amps);

		// Commanded outputs
		inputs.onArmPercent = onArmIntakeSpeedReference;
		inputs.lowerPercent = lowerIntakeSpeedReference;
		inputs.upperPercent = upperIntakeSpeedReference;

		// Measured values (if available)
		inputs.onArmVelocityRPM = m_onArmIntake.getEncoder().getVelocity();
		inputs.upperVelocityRPM = m_upperFixedIntake.getEncoder().getVelocity();
		inputs.lowerCurrentAmps = m_lowerFixedIntake.getOutputCurrent();

		// Derived state
		inputs.intakeActive = Math.abs(onArmIntakeSpeedReference) > 0.01 ||
			Math.abs(lowerIntakeSpeedReference) > 0.01 ||
			Math.abs(upperIntakeSpeedReference) > 0.01;

		inputs.reversed = onArmIntakeSpeedReference < 0;
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
		AngularVelocity upperVelocity = Motors.VORTEX_MAX_VELOCITY.times(setpoint);
		AngularVelocity otherSide = upperVelocity.div(IntakeConstants.ON_ARM_GEAR_RATIO);
		double linearSpeedSurfaceSpeedInchesPerMinute = otherSide.in(Units.RPM) *
			IntakeConstants.UPPER_WHEEL_CURCUMFERENCE.in(Units.Inches);
		double lowerMotorRpm = linearSpeedSurfaceSpeedInchesPerMinute *
			IntakeConstants.LOWER_WHEEL_CURCUMFERENCE.in(Units.Inches);
		double lowerRpmWithReducer = lowerMotorRpm * IntakeConstants.LOWER_FIXED_GEAR_RATIO;
		double lowerReference = lowerRpmWithReducer /
			Motors.REDLINE_MAX_VELOCITY.in(Units.RPM);
		return lowerReference * speedMod;
	}

	private static double calculateUpperReferenceInRelationToArmIntake(double setpoint) {
		return setpoint / IntakeConstants.UPPER_FIXED_GEAR_RATIO * 1.01;
	}

	public void extendArm() {
		m_arm.rotateOut();
	}

	public void retractArm() {
		m_arm.rotateIn();
	}

	public void enableIntake() {
		final double speed = IntakeConstants.SPEED.in(Units.Value); // Value gives n/100
		onArmIntakeSpeedReference = speed * 1.1;
		lowerIntakeSpeedReference = calculateLowerReferenceInRelationToArmIntake(speed);
		upperIntakeSpeedReference = calculateUpperReferenceInRelationToArmIntake(speed);
		m_onArmIntake.setDutyCycle(onArmIntakeSpeedReference, true);
		m_upperFixedIntake.setDutyCycle(upperIntakeSpeedReference, true);
		m_lowerFixedIntake.setDutyCycle(lowerIntakeSpeedReference, true);
	}

	public void enableSpitout() {
		final double speed = IntakeConstants.SPEED.times(-1.0).in(Units.Value); // of 100, not 1
		onArmIntakeSpeedReference = speed;
		lowerIntakeSpeedReference = calculateLowerReferenceInRelationToArmIntake(speed);
		upperIntakeSpeedReference = calculateUpperReferenceInRelationToArmIntake(speed);
		m_onArmIntake.setDutyCycle(onArmIntakeSpeedReference, true);
		m_upperFixedIntake.setDutyCycle(upperIntakeSpeedReference, true);
		m_lowerFixedIntake.setDutyCycle(lowerIntakeSpeedReference, true);
	}

	public void enableReverse() {
		final double speed = IntakeConstants.SPEED.times(-1.0).in(Units.Value);
		onArmIntakeSpeedReference = speed;
		lowerIntakeSpeedReference = calculateLowerReferenceInRelationToArmIntake(speed);
		upperIntakeSpeedReference = calculateUpperReferenceInRelationToArmIntake(speed);
		m_onArmIntake.setDutyCycle(onArmIntakeSpeedReference, true);
		m_upperFixedIntake.setDutyCycle(upperIntakeSpeedReference, true);
		m_lowerFixedIntake.setDutyCycle(lowerIntakeSpeedReference, true);
	}

	public void disableIntake() {
		onArmIntakeSpeedReference = lowerIntakeSpeedReference = upperIntakeSpeedReference = 0;
		m_onArmIntake.setDutyCycle(onArmIntakeSpeedReference, true);
		m_upperFixedIntake.setDutyCycle(upperIntakeSpeedReference, true);
		m_lowerFixedIntake.setDutyCycle(lowerIntakeSpeedReference, true);
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

	public Command enableIntakeForeverCommand() {
		var subsystem = this;
		return new Command() {
			private final IntakeSubsystem s;

			{
				s = subsystem;
			}

			@Override
			public void execute() {
				s.extendArm();
				s.enableIntake();
			}

			@Override
			public boolean isFinished() {
				return false;
			}

			@Override
			public void end(boolean interrupted) {
				s.disableIntake();
				s.retractArm();
			}
		}.withName("IntakeCommand");
	}
}
