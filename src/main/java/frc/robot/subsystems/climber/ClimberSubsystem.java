package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NopSubsystemBase;
import frc.lib.talonfx.TalonFXIO_Wrapper;
import frc.lib.talonfx.TalonFXWrapper;
import frc.robot.info.Debug;
import frc.robot.info.constants.CanIdConstants;
import frc.robot.info.constants.ClimberConstants;

public class ClimberSubsystem extends NopSubsystemBase {
	private final TalonFXIO_Wrapper m_motor;

	private Voltage setpoint = Units.Volts.of(0);
	private boolean isEnabled = false;

	private final ClimberIO.ClimberIOInputs inputs = new ClimberIO.ClimberIOInputs();

	private ClimberSubsystem() {
		m_motor = new TalonFXIO_Wrapper(
			new TalonFXWrapper(CanIdConstants.CLIMBER)
				.withConfiguration(new TalonFXConfiguration()
					.withCurrentLimits(new CurrentLimitsConfigs()
						.withStatorCurrentLimit(ClimberConstants.MAX_CURRENT)
						.withStatorCurrentLimitEnable(true)))
				.withNeutralMode(NeutralModeValue.Brake)
				.withSlot0Pidf(ClimberConstants.PIDF)
				.withMotionMagicConfig(ClimberConstants.MOTION_MAGIC));
	}

	private static final class Holder {
		private static final ClimberSubsystem INSTANCE = new ClimberSubsystem();
	}

	public static synchronized ClimberSubsystem getInstance() {
		return Holder.INSTANCE;
	}

	@Override
	public void periodic() {
		isEnabled = Math.abs((int) (m_motor.getMotor().getMotorVoltage().getValueAsDouble())) > 0;

		Logger.processInputs("Climber", inputs);
		Logger.recordOutput(
			"Climber/Stalled",
			inputs.statorCurrentAmps > 80 && Math.abs(inputs.velocityRPS) < 0.1);

		if (Debug.DebugLevel.isOrAll(Debug.DebugLevel.Climb))
			SmartDashboard.putData(this);
	}

	public void log() {
		final var motor = m_motor.getMotor();
		inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
		inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
		inputs.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
		inputs.velocityRPS = motor.getVelocity().getValueAsDouble();
		inputs.positionRotations = motor.getPosition().getValueAsDouble();
		inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
		inputs.setpointVolts = setpoint.in(Units.Volts);
		inputs.isEnabled = inputs.appliedVolts != 0;
	}

	@Override
	public void simulationPeriodic() {
		m_motor.simulationPeriodic();
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType(this.getClass().getName());
		builder.addBooleanProperty("Enabled", () -> isEnabled, null);
		builder.addDoubleProperty("SetpointVolts", () -> setpoint.in(Units.Volts), null);
	}

	private void enable(boolean invert) {
		var spoolRotationsPerSecond = Units.RotationsPerSecond.of(0.5);

		var velocity = spoolRotationsPerSecond
			.times(ClimberConstants.GEAR_RATIO)
			.times(invert ? -1.0 : 1.0);

		m_motor.setVelocity(velocity);

		// 🔥 log-friendly: reflect intent
		setpoint = Units.Volts.of(12 * (invert ? -1.0 : 1.0)); // approximate
	}

	public void enableForward() {
		enable(false);
	}

	public void enableReverse() {
		enable(true);
	}

	public void stop() {
		setpoint = Units.Volts.of(0);
		m_motor.setVoltage(setpoint);
	}

	public Command toggleForward() {
		return Commands.startEnd(this::enableForward, this::stop, this);
	}

	public Command toggleReverse() {
		return Commands.startEnd(this::enableReverse, this::stop, this);
	}
}
