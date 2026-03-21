// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.info.constants.ControllerConstants.DriverMap;
import frc.robot.info.constants.PdhChannelConstants;
import frc.robot.subsystems.AlertSubsystem;
import frc.robot.subsystems.PdhSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.coordinator.CoordinatorSubsystem;
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

	public static synchronized RobotContainer getInstance() {
		return Holder.INSTANCE;
	}

	public void nops() {
		AlertSubsystem.staticNop();
		PdhSubsystem.staticNop();

		LedSubsystem.staticNop();
		ClimberSubsystem.staticNop();
		CoordinatorSubsystem.staticNop();
		DriveSubsystem.staticNop();
		IndexerSubsystem.staticNop();
		IntakeSubsystem.staticNop();
		ShooterSubsystem.staticNop();
		VisionSubsystem.staticNop();
	}

	public void createInstances() {
		AlertSubsystem.getInstance();
		// PdhSubsystem.getInstance();

		LedSubsystem.getInstance();
		ClimberSubsystem.getInstance();
		CoordinatorSubsystem.getInstance();
		DriveSubsystem.getInstance();
		IndexerSubsystem.getInstance();
		IntakeSubsystem.getInstance();
		ShooterSubsystem.getInstance();
		VisionSubsystem.getInstance();

		DriveCommands.initialize(() -> m_driverController);
	}

	public void configureBindings() {
		DriveSubsystem.getInstance().setDefaultCommand(
			DriveCommands.getCommand(DriveCommands.DriveType.FO_AngularVelocity));

		m_driverController.button(DriverMap.DRIVE_TO_POSE).whileTrue(
			DriveSubsystem.getInstance().driveToTargetCommand().repeatedly());
		m_driverController.button(DriverMap.ZERO_GYRO).onTrue(
			Commands.run(DriveSubsystem.getInstance()::zeroGyro, DriveSubsystem.getInstance()));
		m_driverController.button(DriverMap.RESET_ODOMETRY).onTrue(
			DriveSubsystem.getInstance().resetOdometryFlippedCommand());
		m_driverController.button(DriverMap.CENTER_SWERVES).whileTrue(
			DriveSubsystem.getInstance().centerModulesCommand());
		m_driverController.povUp().onTrue(IntakeSubsystem.getInstance().extendArmCommand());
		m_driverController.povDown().onTrue(IntakeSubsystem.getInstance().retractArmCommand());
		m_driverController.povRight().onTrue(IntakeSubsystem.getInstance().incrementArmCommand());
		m_driverController.povLeft().onTrue(IntakeSubsystem.getInstance().decrementArmCommand());
		m_driverController.button(DriverMap.TOGGLE_SHOOTER)
			.toggleOnTrue(ShooterSubsystem.getInstance().toggle());
		m_driverController.button(DriverMap.ENABLE_INTAKE_IN)
			.whileTrue(ControllerActions.enableCoordinatedIntake())
			.onFalse(ControllerActions.disableCoordinatedIntake());
		m_driverController.button(DriverMap.ENABLE_INTAKE_EXPEL)
			.whileTrue(ControllerActions.enableCoordinatedIntakeReverse())
			.onFalse(ControllerActions.disableCoordinatedIntake());

		final double OPERATOR_AXIS_THRESHOLD = 0.1;
		m_operatorController
			.axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, OPERATOR_AXIS_THRESHOLD)
			.whileTrue(
				Commands.run(() -> {
					double axis = m_operatorController
						.getRawAxis(XboxController.Axis.kRightY.value);
					if (axis < -OPERATOR_AXIS_THRESHOLD) {
						IntakeSubsystem.getInstance().enableIntake();
						IndexerSubsystem.getInstance().enable();
					} else
						if (axis > OPERATOR_AXIS_THRESHOLD) {
							IntakeSubsystem.getInstance().enableSpitout();
							IndexerSubsystem.getInstance().enableReverse();
						} else {
							IntakeSubsystem.getInstance().disableIntake();
							IndexerSubsystem.getInstance().disable();
						}
				}, IntakeSubsystem.getInstance(), IndexerSubsystem.getInstance()))
			.onFalse(ControllerActions.disableCoordinatedIntake());
		m_operatorController.y().toggleOnTrue(ShooterSubsystem.getInstance().toggle());
		m_operatorController.b().toggleOnTrue(ShooterSubsystem.getInstance().toggleFeederReverse());
		m_operatorController.leftBumper()
			.toggleOnTrue(CoordinatorSubsystem.getInstance().toggleToStorage());
		m_operatorController.rightBumper()
			.toggleOnTrue(CoordinatorSubsystem.getInstance().toggleToShooter());
		m_operatorController
			.axisGreaterThan(XboxController.Axis.kRightTrigger.value, OPERATOR_AXIS_THRESHOLD)
			.whileTrue(ShooterSubsystem.getInstance().changeFlywheelSpeedMod(
				() -> m_operatorController.getRawAxis(XboxController.Axis.kRightTrigger.value)))
			.onFalse(ShooterSubsystem.getInstance().resetFlywheelSpeedMod());
		m_operatorController.povUp().onTrue(IntakeSubsystem.getInstance().extendArmCommand());
		m_operatorController.povDown().onTrue(IntakeSubsystem.getInstance().retractArmCommand());
		m_operatorController.povRight().onTrue(IntakeSubsystem.getInstance().incrementArmCommand());
		m_operatorController.povLeft().onTrue(IntakeSubsystem.getInstance().decrementArmCommand());
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

	public static final class ControllerActions {
		public static Command enableCoordinatedIntake() {
			return Commands.run(() -> {
				IntakeSubsystem.getInstance().enableIntake();
				IndexerSubsystem.getInstance().enable();
			}, IntakeSubsystem.getInstance(), IndexerSubsystem.getInstance());
		}

		public static Command enableCoordinatedIntakeReverse() {
			return Commands.run(() -> {
				IntakeSubsystem.getInstance().enableSpitout();
				IndexerSubsystem.getInstance().enableReverse();
			}, IntakeSubsystem.getInstance(), IndexerSubsystem.getInstance());
		}

		public static Command disableCoordinatedIntake() {
			return Commands.run(() -> {
				IntakeSubsystem.getInstance().disableIntake();
				IndexerSubsystem.getInstance().disable();
			}, IntakeSubsystem.getInstance(), IndexerSubsystem.getInstance());
		}
	}
}
