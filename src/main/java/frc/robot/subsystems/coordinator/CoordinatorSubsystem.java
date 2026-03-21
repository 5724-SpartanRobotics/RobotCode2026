package frc.robot.subsystems.coordinator;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.ColorUtil;
import frc.lib.NopSubsystemBase;
import frc.robot.info.Alliance;
import frc.robot.info.Debug;
import frc.robot.info.constants.CanIdConstants;
import frc.robot.info.constants.CoordinatorConstants;
import frc.robot.subsystems.led.LedSubsystem;

public class CoordinatorSubsystem extends NopSubsystemBase {
	private final CoordinatorIO io;
	private final CoordinatorIO.CoordinatorIOInputs inputs = new CoordinatorIO.CoordinatorIOInputs();

	private AngularVelocity setpoint = Units.RPM.of(0);

	private CoordinatorSubsystem() {
		io = new CoordinatorIO_RealAndSim(CanIdConstants.COORDINATOR);
	}

	private static final class Holder {
		private static final CoordinatorSubsystem INSTANCE = new CoordinatorSubsystem();
	}

	public static synchronized CoordinatorSubsystem getInstance() {
		return Holder.INSTANCE;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		BooleanSupplier enabled = () -> ((int) setpoint.abs(Units.RPM)) > 0;
		BooleanSupplier reversed = () -> enabled.getAsBoolean() && setpoint.lt(Units.RPM.of(0));

		builder.setSmartDashboardType(this.getClass().getName());
		builder.addDoubleProperty("Velocity Setpoint RPM", () -> setpoint.in(Units.RPM), null);
		builder.addBooleanProperty("Enabled", enabled, null);
		builder.addBooleanProperty("Reversed", reversed, null);
		builder.addBooleanProperty("To Storage", reversed, null);
		builder.addBooleanProperty("To Shooter", () -> !reversed.getAsBoolean(), null);
		builder.addStringProperty("To", () -> reversed.getAsBoolean() ? "STORAGE" : "SHOOTER",
			null);
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Coordinator", inputs);

		io.setVelocity(Units.RPM.of(
			io.getRateLimiter().calculate(setpoint.in(Units.RPM))));

		if (Debug.DebugLevel.isOrAll(Debug.DebugLevel.Indexer))
			SmartDashboard.putData(this);
	}

	public void enableToStorage() {
		setpoint = CoordinatorConstants.RUN_TO_STORAGE_SETPOINT;
		LedSubsystem.getInstance().setPersistentNotify(Alliance.getAllianceColor());
	}

	public void enableToShooter() {
		setpoint = CoordinatorConstants.RUN_TO_SHOOTER_SETPOINT;
		LedSubsystem.getInstance().setPersistentNotify(
			ColorUtil.plusRGB(Alliance.getAllianceColor(), 0, 100, 180));
	}

	public void disable() {
		setpoint = Units.RPM.of(0);
		io.stop();

		LedSubsystem.getInstance().clearPersistentNotify(Alliance.getAllianceColor());
		LedSubsystem.getInstance()
			.clearPersistentNotify(ColorUtil.plusRGB(Alliance.getAllianceColor(), 0, 100, 180));
	}

	public Command toggleToShooter() {
		return Commands.startEnd(
			() -> enableToShooter(),
			() -> disable(),
			this);
	}

	public Command toggleToStorage() {
		return Commands.startEnd(
			() -> enableToStorage(),
			() -> disable(),
			this);
	}
}