// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
	public RobotContainer() {
		configureBindings();
	}

	private void configureBindings() {}

	public void robotFinishedBooting() {
		// Last year, we:
		// Flash the LEDs on the robot for 2s as an indicator
		// Zero the gyro
		if (Constants.DebugLevel.isAny()) {
			System.out.println(
				"[via RobotContainter.robotFinishedBooting] " +
				"[via DebugLevel] Robot Code Debugging is ON. " +
				"The \"Debug Mode\" is available in SmartDashboard."
			);
		}
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
