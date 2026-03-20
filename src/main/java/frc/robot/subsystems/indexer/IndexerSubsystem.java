package frc.robot.subsystems.indexer;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.NopSubsystemBase;
import frc.robot.info.Debug;
import frc.robot.info.constants.CanIdConstants;
import frc.robot.info.constants.IndexerConstants;
import frc.robot.subsystems.led.LedSubsystem;

public class IndexerSubsystem extends NopSubsystemBase {
	private final IndexerIO io;
	private final IndexerIO.IndexerIOInputs inputs = new IndexerIO.IndexerIOInputs();

	private double setpoint = 0;

	private IndexerSubsystem() {
		io = new IndexerIO_RealAndSim(CanIdConstants.INDEXER, CanIdConstants.COORDINATOR);
	}

	private static final class Holder {
		private static final IndexerSubsystem INSTANCE = new IndexerSubsystem();
	}

	public static IndexerSubsystem getInstance() {
		return Holder.INSTANCE;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		BooleanSupplier enabled = () -> Math.abs((int) setpoint) > 0;

		builder.setSmartDashboardType(this.getClass().getName());
		builder.addDoubleProperty("Duty Cycle Setpoint", () -> setpoint, null);
		builder.addBooleanProperty("Enabled", enabled, null);
		builder.addBooleanProperty("Reversed", () -> enabled.getAsBoolean() && setpoint < 0, null);
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Indexer", inputs);

		if (Debug.DebugLevel.isOrAll(Debug.DebugLevel.Indexer))
			SmartDashboard.putData(this);
	}

	public void enable() {
		setpoint = IndexerConstants.RUN_SETPOINT;
		io.setDutyCycle(setpoint);

		LedSubsystem.getInstance().setPersistentNotify(
			LedSubsystem.kNotification2Color);
	}

	public void enableReverse() {
		setpoint = IndexerConstants.RUN_SETPOINT * -1.0;
		io.setDutyCycle(setpoint);

		LedSubsystem.getInstance().setPersistentNotify(
			LedSubsystem.kNotification3Color);
	}

	public void disable() {
		setpoint = 0;
		io.stop();

		LedSubsystem.getInstance().clearPersistentNotify(
			LedSubsystem.kNotification2Color);
		LedSubsystem.getInstance().clearPersistentNotify(
			LedSubsystem.kNotification3Color);
	}
}