package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.NopSubsystemBase;

public class AlertSubsystem extends NopSubsystemBase {
	private final Alert canAlert = new Alert("CAN warnings/errors detected!", AlertType.kError);

	private final Alert canUtilizationAlert = new Alert("CAN bus utilization is too high!",
		AlertType.kWarning);

	private final Alert brownoutAlert = new Alert("Robot is browning out!", AlertType.kError);

	private final Alert lowVoltageAlert = new Alert("Battery voltage low", AlertType.kWarning);

	private AlertSubsystem() {
	}

	private static final class Holder {
		private static final AlertSubsystem INSTANCE = new AlertSubsystem();
	}

	public static AlertSubsystem getInstance() {
		return Holder.INSTANCE;
	}

	@Override
	public void periodic() {
		// CAN
		var can = RobotController.getCANStatus();
		canAlert.set(
			can.receiveErrorCount > 0 ||
				can.transmitErrorCount > 0);

		canUtilizationAlert.setText(
			String.format(
				"CAN bus utilization is too high (%.1f%%)!",
				can.percentBusUtilization * 100.0));
		canUtilizationAlert.set(can.percentBusUtilization > .65);

		// Power
		double voltage = RobotController.getBatteryVoltage();
		brownoutAlert.set(RobotController.isBrownedOut() || voltage < 7.0);
		lowVoltageAlert.set(voltage < 10.5);

		Logger.recordOutput("Health/BatteryVoltage", voltage);
		Logger.recordOutput("Health/BrownedOut", RobotController.isBrownedOut());
		Logger.recordOutput("Health/CANErrors", can.receiveErrorCount);
		Logger.recordOutput("Health/CANWarnings", can.transmitErrorCount);
		Logger.recordOutput("Health/CANUtilization", can.percentBusUtilization * 100.0);
	}
}
