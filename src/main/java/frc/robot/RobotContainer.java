// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.YagslDriveCommand;
import frc.robot.commands.autos.DriveAuto;
import frc.robot.lib.Elastic;
import frc.robot.subsystems.YagslDriveSubsystem;

public class RobotContainer {
	private YagslDriveSubsystem _DriveSubsystem = new YagslDriveSubsystem(Constants.Drive.SWERVE_CONFIG);
	// private CustomDriveSubsystem _DriveSubsystem = CustomDriveSubsystem.initialize(true);

	private CommandJoystick _DriverController = new CommandJoystick(0);
	private CommandXboxController _OperatorController = new CommandXboxController(1);

	public RobotContainer() {
		YagslDriveCommand.initialize(_DriveSubsystem, _DriverController);

		configureControllerBindings();
	}

	private void configureControllerBindings() {
		_DriveSubsystem.setDefaultCommand(
			YagslDriveCommand.getCommand(YagslDriveCommand.DriveType.FO_AngularVelocity, Robot.isSimulation())
			// _DriveSubsystem.getTeleopCommand(_DriverController)
		);

		configureSimAndTestBindings();

		/* USING YAGSL */
		_DriverController.button(Constants.Controller.DriverMap.DRIVE_TO_POSE).whileTrue(
			_DriveSubsystem.aimAtTarget()
		);
		_DriverController.button(Constants.Controller.DriverMap.ZERO_GYRO).onTrue(Commands.run(
			_DriveSubsystem::zeroGyro
		));
		_DriverController.button(Constants.Controller.DriverMap.RESET_ODOMETRY).onTrue(
			_DriveSubsystem.resetOdometryCommand()
		);
		_DriverController.button(Constants.Controller.DriverMap.RESET_ODOMETRY).onTrue(
			_DriveSubsystem.resetOdometryFlippedCommand()
		);
		_DriverController.button(Constants.Controller.DriverMap.CENTER_SWERVES).whileTrue(
			_DriveSubsystem.centerModulesCommand()
		);
		_DriverController.button(Constants.Controller.DriverMap.SPEEDMOD_MAX).whileTrue(
			Commands.runOnce(() -> YagslDriveCommand.speedMod = 0.9)
		).onFalse(
			Commands.runOnce(() -> YagslDriveCommand.speedMod = Constants.Robot.DEFAULT_SPEED_MOD, _DriveSubsystem)
		);
		_DriverController.button(Constants.Controller.DriverMap.SPEEDMOD_MID).whileTrue(
			Commands.runOnce(() -> YagslDriveCommand.speedMod = 0.65)
		).onFalse(
			Commands.runOnce(() -> YagslDriveCommand.speedMod = Constants.Robot.DEFAULT_SPEED_MOD)
		);

		/* USING CUSTOM IMPLEMENTATION */
		// _DriverController.button(Constants.Controller.DriverMap.ZERO_GYRO).onTrue(Commands.runOnce(() -> {
		// 	_DriveSubsystem.resetOdometry();
		// 	_DriveSubsystem.zeroGyro();
		// }, _DriveSubsystem));
		// _DriverController.button(Constants.Controller.DriverMap.CENTER_SWERVES).whileTrue(
		// 	Commands.run(() -> _DriveSubsystem.centerModules(), _DriveSubsystem)
		// );
		// _DriverController.button(Constants.Controller.DriverMap.SPEEDMOD_MAX).whileTrue(
		// 	Commands.runOnce(() -> _DriveSubsystem.setSpeedMod(1.0), _DriveSubsystem)
		// ).onFalse(
		// 	Commands.runOnce(() -> _DriveSubsystem.resetSpeedMod(), _DriveSubsystem)
		// );
		// _DriverController.button(Constants.Controller.DriverMap.SPEEDMOD_MID).whileTrue(
		// 	Commands.runOnce(() -> _DriveSubsystem.setSpeedMod(0.65), _DriveSubsystem)
		// ).onFalse(
		// 	Commands.runOnce(() -> _DriveSubsystem.resetSpeedMod(), _DriveSubsystem)
		// );
	}

	private void configureSimAndTestBindings() {
		if (RobotBase.isSimulation()) {
			Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
			// YagslDriveCommand.DriveDirectAngle_Keyboard().driveToPose(
			// 	() -> target,
			// 	new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
			// 	new ProfiledPIDController(5, 0, 0, new Constraints(
			// 		Units.Radians.of(Constants.TWO_PI).baseUnitMagnitude(),
			// 		Units.Radians.of(Math.PI).baseUnitMagnitude()
			// 	))
			// );
		}
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

		if (DriverStation.isDSAttached() && Robot.isFirstConnection.compareAndSet(true, false)) {
			Elastic.selectTab("Auto");
			CommandScheduler.getInstance().schedule(Commands.parallel(
				_DriveSubsystem.resetOdometryCommand(),
				Commands.runOnce(_DriveSubsystem::zeroGyro)
			));
		}

		SmartDashboard.putNumber("Aim At Target/X", 0);
		SmartDashboard.putNumber("Aim At Target/Y", 0);
		SmartDashboard.putNumber("Aim At Target/Theta (Degrees)", 0);
		SmartDashboard.putNumber("Aim At Target/Aim Constant (Degrees)", 30.0);
	}

	public void teleopInit() {
		// CommandScheduler.getInstance().schedule(Commands.parallel(
		// 	_DriveSubsystem.resetOdometryCommand(),
		// 	Commands.runOnce(_DriveSubsystem::zeroGyro)
		// ));
	}

	private boolean hasBeenEnabledYet = false;
	public void visionPeriodic() {
		if (!hasBeenEnabledYet) {
			hasBeenEnabledYet = DriverStation.isEnabled();
			// CommandScheduler.getInstance().schedule(Commands.parallel(
			// 	_DriveSubsystem.resetOdometryCommand(),
			// 	Commands.runOnce(_DriveSubsystem::zeroGyro)
			// ));
		}
	}

	public void setMotorBrake(boolean brake) {
		// _DriveSubsystem.setMotorBrake(brake);
	}

	public Command getAutonomousCommand() {
		return new DriveAuto(_DriveSubsystem);
	}
}
