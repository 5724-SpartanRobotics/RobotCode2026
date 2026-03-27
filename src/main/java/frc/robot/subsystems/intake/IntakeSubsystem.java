package frc.robot.subsystems.intake;

import java.nio.BufferOverflowException;
import java.nio.BufferUnderflowException;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NopSubsystemBase;
import frc.lib.motor.talonfx.TalonFXIO_Wrapper;
import frc.lib.motor.talonfx.TalonFXWrapper;
import frc.robot.info.Debug;
import frc.robot.info.constants.CanIdConstants;
import frc.robot.info.constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends NopSubsystemBase {
	/** Kraken X60 on TalonFX */
	private final TalonFXIO_Wrapper m_onArmIntake;
	private double onArmIntakeSpeedReference = 0;

	private final IntakeArm m_arm = IntakeArm.getInstance();

	private final IntakeIOInputs inputs = new IntakeIOInputs();

	private IntakeSubsystem() {
		m_onArmIntake = new TalonFXIO_Wrapper(
			new TalonFXWrapper(CanIdConstants.INTAKE_ON_ARM)
				.withConfiguration(new TalonFXConfiguration()
					.withHardwareLimitSwitch(new HardwareLimitSwitchConfigs()
						.withForwardLimitEnable(false)
						.withReverseLimitEnable(false)))
				.withNeutralMode(NeutralModeValue.Brake)
				.withSlot0Pidf(IntakeConstants.PIDF)
				.withInverted(true));
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

		try {
			Logger.recordOutput(
				"Intake/AI_JamDetected",
				inputs.intakeActive &&
					Math.abs(inputs.onArmVelocityRPM) < 100 &&
					Math.abs(inputs.onArmPercent) > 0.3);

			Logger.recordOutput(
				"Intake/AI_ArmErrorDeg",
				inputs.armSetpointDeg - inputs.armPositionDeg);
		} catch (BufferUnderflowException e) {
		} catch (BufferOverflowException e) {
		}
	}

	public void log() {
		inputs.armPositionDeg = m_arm.getAngle().in(Units.Degrees);
		inputs.armSetpointDeg = m_arm.getSetpoint().in(Units.Degrees);
		inputs.armLeftCurrentAmps = m_arm.getMasterOutputCurrent().in(Units.Amps);
		inputs.armRightCurrentAmps = m_arm.getSlaveOutputCurrent().in(Units.Amps);

		// Commanded outputs
		inputs.onArmPercent = onArmIntakeSpeedReference;

		// Measured values (if available)
		inputs.onArmVelocityRPM = m_onArmIntake.getMotor().getVelocity().getValue().in(Units.RPM);

		// Derived state
		inputs.intakeActive = Math.abs(onArmIntakeSpeedReference) > 0.01;

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
	}

	public void extendArm() {
		m_arm.rotateOut();
	}

	public void retractArm() {
		m_arm.rotateIn();
	}

	public void enableIntake() {
		final double speed = IntakeConstants.SPEED.times(IntakeConstants.ON_ARM_GEAR_RATIO)
			.in(Units.Value); // Value gives n/100
		onArmIntakeSpeedReference = speed * 1.1;
		m_onArmIntake.setDutyCycle(onArmIntakeSpeedReference);
	}

	public void enableSpitout() {
		final double speed = IntakeConstants.SPEED.times(IntakeConstants.ON_ARM_GEAR_RATIO)
			.times(-1.0).in(Units.Value); // of 100, not 1
		onArmIntakeSpeedReference = speed;
		m_onArmIntake.setDutyCycle(onArmIntakeSpeedReference);
	}

	public void enableReverse() {
		final double speed = IntakeConstants.SPEED.times(IntakeConstants.ON_ARM_GEAR_RATIO)
			.times(-1.0).in(Units.Value);
		onArmIntakeSpeedReference = speed;
		m_onArmIntake.setDutyCycle(onArmIntakeSpeedReference);
	}

	public void disableIntake() {
		onArmIntakeSpeedReference = 0;
		m_onArmIntake.setDutyCycle(onArmIntakeSpeedReference);
	}

	public Command toggleIntake() {
		return startEnd(this::enableIntake, this::disableIntake);
	}

	public Command toggleArm() {
		return startEnd(this::extendArm, this::retractArm);
	}

	public Command extendArmCommand() {
		return runOnce(this::extendArm);
	}

	public Command retractArmCommand() {
		return runOnce(this::retractArm);
	}

	public Command incrementArmCommand() {
		return runOnce(m_arm::increment);
	}

	public Command decrementArmCommand() {
		return runOnce(m_arm::decrement);
	}

	public Command stopArm() {
		return runOnce(m_arm::stop);
	}

	public Command toggleAll() {
		return startEnd(
			() -> {
				extendArm();
				enableIntake();
			},
			() -> {
				disableIntake();
				retractArm();
			});
	}

	public Command runForCommand(Time duration) {
		return Commands.sequence(
			runOnce(() -> {
				extendArm();
				enableIntake();
			}),
			Commands.waitTime(duration),
			runOnce(() -> {
				disableIntake();
				retractArm();
			}));
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
