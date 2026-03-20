// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.ClassFieldMapStringToInt;
import frc.robot.commands.DriveCommands;
import frc.robot.info.constants.CanIdConstants;
import frc.robot.info.constants.ControllerConstants;
import frc.robot.info.constants.PdhChannelConstants;
import frc.robot.subsystems.AlertSubsystem;
import frc.robot.subsystems.PdhSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
	private final SendableChooser<Command> m_autoChooser;

	private final CommandJoystick m_driverController;
	private final CommandXboxController m_operatorController;

	private RobotContainer() {
		ClassFieldMapStringToInt.invalidateDuplicates(CanIdConstants.class);
		ClassFieldMapStringToInt.invalidateDuplicates(PdhChannelConstants.class);
		ClassFieldMapStringToInt.invalidateDuplicates(ControllerConstants.DriverMap.class);
		ClassFieldMapStringToInt.invalidateDuplicates(ControllerConstants.OperatorMap.class);

		nops();
		createInstances();

		m_driverController = new CommandJoystick(0);
		m_operatorController = new CommandXboxController(1);
		configureBindings();
		configureNamedCommands();

		m_autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto choices", m_autoChooser);
	}

	private static final class Holder {
		private static final RobotContainer INSTANCE = new RobotContainer();
	}

	public static RobotContainer getInstance() {
		return Holder.INSTANCE;
	}

	public void nops() {
		AlertSubsystem.staticNop();
		PdhSubsystem.staticNop();

		LedSubsystem.staticNop();
		ClimberSubsystem.staticNop();
		DriveSubsystem.staticNop();
		IndexerSubsystem.staticNop();
		IntakeSubsystem.staticNop();
		ShooterSubsystem.staticNop();
		VisionSubsystem.staticNop();
	}

	public void createInstances() {
		AlertSubsystem.getInstance();
		PdhSubsystem.getInstance();

		LedSubsystem.getInstance();
		ClimberSubsystem.getInstance();
		DriveSubsystem.getInstance();
		IndexerSubsystem.getInstance();
		IntakeSubsystem.getInstance();
		ShooterSubsystem.getInstance();
		VisionSubsystem.getInstance();

		DriveCommands.initialize(m_driverController);
	}

	public void configureBindings() {
		DriveSubsystem.getInstance().setDefaultCommand(
			DriveCommands.getCommand(DriveCommands.DriveType.FO_AngularVelocity));
	}

	public void configureNamedCommands() {
		NamedCommands.registerCommands(
			Map.of(
				"Extend Arm", IntakeSubsystem.getInstance().extendArmCommand(),
				"Retract Arm", IntakeSubsystem.getInstance().retractArmCommand(),
				"Intake", IntakeSubsystem.getInstance().enableIntakeForeverCommand(),
				"Find Pose", DriveSubsystem.getInstance().driveToTargetCommand().withTimeout(2),
				"Shoot", ShooterSubsystem.getInstance().enableForeverCommand()));
	}

	public Command getAutonomousCommand() {
		Command selected = m_autoChooser.getSelected();
		if (selected == null)
			return Commands.none();
		return selected;
	}
}
