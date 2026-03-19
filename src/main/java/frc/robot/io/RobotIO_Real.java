package frc.robot.io;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

public class RobotIO_Real implements RobotIO {

	private CANStatus canStatus = new CANStatus();

	@Override
	public void updateInputs(RobotIOInputs inputs) {
		// Battery
		inputs.batteryVoltage = RobotController.getBatteryVoltage();
		inputs.brownedOut = RobotController.isBrownedOut();

		// RIO stats
		inputs.cpuTempCelsius = RobotController.getCPUTemp();

		// CAN
		canStatus = RobotController.getCANStatus();
		inputs.canUtilization = canStatus.percentBusUtilization; // 0.0–1.0
		inputs.canTxFullCount = canStatus.txFullCount;
		inputs.canReceiveErrorCount = canStatus.receiveErrorCount;
		inputs.canTransmitErrorCount = canStatus.transmitErrorCount;

		// Time
		inputs.fpgaTimeSeconds = RobotController.getFPGATime() / 1e6;

		// Driver Station
		inputs.dsAttached = DriverStation.isDSAttached();
		inputs.enabled = DriverStation.isEnabled();
		inputs.autonomous = DriverStation.isAutonomous();
		inputs.teleop = DriverStation.isTeleop();
	}
}