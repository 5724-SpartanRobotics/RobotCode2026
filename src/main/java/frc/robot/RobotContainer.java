// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.lib.Elastic;
import frc.robot.lib.Elastic.Notification;
import frc.robot.lib.Elastic.NotificationLevel;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(Constants.Drive.SWERVE_CONFIG);
	private final ClimberSubsystem m_climberSubsystem = ClimberSubsystem.getInstance();
	private final IntakeSubsystem m_intakeSubsystem = IntakeSubsystem.getInstance();
	private final IndexerSubsystem m_indexerSubsystem = IndexerSubsystem.getInstance();
	private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();

	private final CommandJoystick m_driverController = new CommandJoystick(0);
	private final CommandXboxController m_operatorController = new CommandXboxController(1);
	private final CommandJoystick m_driverController2;

	private final SendableChooser<Command> m_autoChooser;

	private boolean hasBeenEnabledYet = false;

	public RobotContainer() {
		if (Constants.Controller.IS_DOUBLE_DRIVE_CONTROLLER)
			m_driverController2 = new CommandJoystick(3);
 
		if (Constants.Controller.IS_DOUBLE_DRIVE_CONTROLLER)
			DriveCommand.initialize(m_driveSubsystem, m_driverController);
		else 
			DriveCommand.initialize(m_driveSubsystem, m_driverController, m_driverController2);
		
		LedSubsystem.createInstance();

		configureNamedCommands();
		configureControllerBindings();

		/* configureAutos */ {
			m_autoChooser = AutoBuilder.buildAutoChooser(); // default auto is Commands.none();
			SmartDashboard.putData("Auto choices", m_autoChooser);
		}
	}

	private void configureControllerBindings() {
		m_driveSubsystem.setDefaultCommand(
			DriveCommand.getCommand(DriveCommand.DriveType.FO_AngularVelocity, RobotBase.isSimulation())
		);

		configureSimAndTestBindings();

		m_driverController.button(Constants.Controller.DriverMap.DRIVE_TO_POSE).whileTrue(
			m_driveSubsystem.driveToTargetCommand().repeatedly()
		);
		m_driverController.button(Constants.Controller.DriverMap.ZERO_GYRO).onTrue(Commands.run(
			m_driveSubsystem::zeroGyro
		));
		m_driverController.button(Constants.Controller.DriverMap.RESET_ODOMETRY).onTrue(
			m_driveSubsystem.resetOdometryFlippedCommand()
		);
		m_driverController.button(Constants.Controller.DriverMap.CENTER_SWERVES).whileTrue(
			m_driveSubsystem.centerModulesCommand()
		);
		m_driverController.button(Constants.Controller.DriverMap.DRIVE_TO_INITIAL_POSE).whileTrue(
			m_driveSubsystem.driveToInitialPosition(0.8).repeatedly()
		);
		m_driverController.button(Constants.Controller.DriverMap.SPEEDMOD_MAX).whileTrue(
			DriveCommand.setSpeedModCommand(0.9)
		).onFalse(
			DriveCommand.resetSpeedModCommand()
		);
		m_driverController.button(Constants.Controller.DriverMap.SPEEDMOD_MID).whileTrue(
			DriveCommand.setSpeedModCommand(0.65)
		).onFalse(
			DriveCommand.resetSpeedModCommand()
		);
		m_driverController.button(Constants.Controller.DriverMap.TOGGLE_NOTIFICATION).onTrue(
			LedSubsystem.getInstance().togglePersistentNotificationCommand(LedSubsystem.kNotification3Color)
		);

		m_operatorController.axisGreaterThan(XboxController.Axis.kRightY.value, 0.1).whileTrue(
			Commands.run(m_intakeSubsystem::enableIntake)
		).onFalse(
			Commands.run(m_intakeSubsystem::disableIntake)
		);
		m_operatorController.a().toggleOnTrue(m_climberSubsystem.toggleReverse());
		m_operatorController.x().toggleOnTrue(m_climberSubsystem.toggleReverse());
	}

	private void configureSimAndTestBindings() {}

	private void configureNamedCommands() {
		NamedCommands.registerCommands(
			Map.of(
				"Intake", m_intakeSubsystem.runForCommand(Units.Seconds.of(3)),
				"Find Pose", m_driveSubsystem.driveToTargetCommand().withTimeout(2),
				"Shoot", m_shooterSubsystem.runForCommand(Units.Seconds.of(3))
			)
		);
	}

	public void robotFinishedBooting() {
		// LEDs do their thing automatically.

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
				m_driveSubsystem.resetOdometryCommand(),
				Commands.runOnce(m_driveSubsystem::zeroGyro)
			));
		}
	}

	public void teleopInit() {}

	public void indicateWheelsUnlocked() {
		Elastic.sendNotification(new Notification(
			NotificationLevel.INFO,
			"Robot Brake State Changed",
			"The wheel brake has been disabled and the robot can move freely."
		));
	}

	public void robotPeriodic() {
		if (!hasBeenEnabledYet) {
			hasBeenEnabledYet = DriverStation.isEnabled();
		}

		m_shooterSubsystem.hypotenuseToAllianceHub = m_driveSubsystem.getHypotToAllianceHub();
	}

	public void setMotorBrake(boolean brake) {
		m_driveSubsystem.setMotorBrake(brake);
	}

	public Command getAutonomousCommand() {
		return m_autoChooser.getSelected();
	}
}