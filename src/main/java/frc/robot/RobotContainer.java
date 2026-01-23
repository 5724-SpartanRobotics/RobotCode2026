// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Drive;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
	private DriveSubsystem _DriveSubsystem = new DriveSubsystem(Constants.Drive.SWERVE_CONFIG);
	private VisionSubsystem _VisionSubsystem;

	private CommandJoystick _DriverController = new CommandJoystick(0);
	private CommandXboxController _OperatorController = new CommandXboxController(1);

	public RobotContainer() {
		DriveCommand.initialize(_DriveSubsystem, _DriverController);

		configureControllerBindings();
	}

	private void configureControllerBindings() {
		_DriveSubsystem.setDefaultCommand(
			DriveCommand.getCommand(DriveCommand.DriveType.FO_DirectAngle, Robot.isSimulation())
		);

		// TODO: Finish implementations starting a L151
		// https://github.com/Yet-Another-Software-Suite/YAGSL/blob/main/examples/drivebase_with_PhotonVision/src/main/java/frc/robot/RobotContainer.java#L151
		// but i'm tired so I'll do it later.

		// _DriverController.button(1).whileTrue();
	}

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

	public void visionPeriodic() {}

	public void setMotorBrake(boolean brake) {
		_DriveSubsystem.setMotorBrake(brake);
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
