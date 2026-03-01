package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.TalonFXWrapper;

public class ClimberSubsystem extends SubsystemBase {
	private static ClimberSubsystem instance = null;

	private final TalonFXWrapper m_motor;

	private Voltage setpoint = Units.Volts.of(0);
	private boolean isEnabled = false;

	private ClimberSubsystem() {
		m_motor = new TalonFXWrapper(Constants.CanId.CLIMBER)
			.withConfiguration(new TalonFXConfiguration()
				.withCurrentLimits(new CurrentLimitsConfigs()
					.withStatorCurrentLimit(Constants.Climber.MAX_CURRENT)
					.withStatorCurrentLimitEnable(true))
			// Do we invert?
			// .withMotorOutput(new MotorOutputConfigs()
			// .withInverted(InvertedValue.Clockwise_Positive)
			// )
			)
			.withNeutralMode(NeutralModeValue.Brake);
	}

	public static ClimberSubsystem getInstance() {
		if (instance == null)
			instance = new ClimberSubsystem();
		return instance;
	}

	@Override
	public void periodic() {
		isEnabled = setpoint.gt(Units.Volts.of(0)) ||
			(int) (m_motor.getMotor().getMotorVoltage().getValueAsDouble()) == 0;

		if (Constants.DebugLevel.isOrAll(Constants.DebugLevel.Climb))
			SmartDashboard.putData(this);
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
		// TODO: See if a reducer affects the setpoint voltage (I don't see why it
		// would)
		// Right now we calculate the voltage by using the stall current
		// TODO: Instead of using a nominal voltage, should we grab the real voltage of
		// the battery
		// double realVoltage = Units.Volts.of(RobotController.getBatteryVoltage());
		Voltage nominalBusVoltage = Units.Volts.of(12);
		setpoint = nominalBusVoltage.times(
			Constants.Climber.MAX_CURRENT.div(Constants.Motors.KRAKENX60_STALL_CURRENT))
			.times(invert ? -1.0 : 1.0);
		m_motor.set(setpoint);
	}

	public void enableForward() {
		enable(false);
	}

	public void enableReverse() {
		enable(true);
	}

	public void stop() {
		setpoint = Units.Volts.of(0);
		m_motor.set(setpoint);
	}

	public Command toggleForward() {
		return Commands.startEnd(this::enableForward, this::stop, this);
	}

	public Command toggleReverse() {
		return Commands.startEnd(this::enableReverse, this::stop, this);
	}
}
