// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.ClassFieldMapStringToInt;
import frc.robot.commands.DriveCommands;
import frc.robot.info.RobotMode;
import frc.robot.info.constants.CanIdConstants;
import frc.robot.info.constants.ControllerConstants;
import frc.robot.info.constants.ControllerConstants.DriverMap;
import frc.robot.info.constants.PdhChannelConstants;
import frc.robot.subsystems.AlertSubsystem;
import frc.robot.subsystems.GameTimerSubsystem;
import frc.robot.subsystems.PdhSubsystem;
import frc.robot.subsystems.coordinator.CoordinatorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
	private final SendableChooser<Command> m_autoChooser;

	private final CommandJoystick m_driverController;
	private final CommandXboxController m_operatorController;

	private Map<String, Command> m_namedCommands;

	private RobotContainer() {
		ClassFieldMapStringToInt.invalidateDuplicates(CanIdConstants.class);
		ClassFieldMapStringToInt.invalidateDuplicates(PdhChannelConstants.class);
		ClassFieldMapStringToInt.invalidateDuplicates(ControllerConstants.DriverMap.class);
		ClassFieldMapStringToInt.invalidateDuplicates(ControllerConstants.OperatorMap.class);

		WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
		PortForwarder.add(5805, "photonvision-front.local", 5800); // dashboard
		PortForwarder.add(5806, "photonvision-front.local", 1182); // stream
		PortForwarder.add(5807, "photonvision-back.local", 5800); // dashboard
		PortForwarder.add(5808, "photonvision-back.local", 1182); // stream
		PortForwarder.add(5809, "photonvision-right.local", 5800); // dashboard
		PortForwarder.add(5810, "photonvision-right.local", 1182); // stream

		nops();
		createInstances();

		m_driverController = new CommandJoystick(0);
		m_operatorController = new CommandXboxController(1);
		configureBindings();
		configureNamedCommands();

		m_autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto choices", m_autoChooser);

		SmartDashboard.putBoolean("Auto Coordinated Shooter Enabled", false);
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
		GameTimerSubsystem.staticNop();
		CoordinatorSubsystem.staticNop();
		DriveSubsystem.staticNop();
		FeederSubsystem.staticNop();
		IndexerSubsystem.staticNop();
		IntakeSubsystem.staticNop();
		ShooterSubsystem.staticNop();
		VisionSubsystem.staticNop();
	}

	public void createInstances() {
		AlertSubsystem.getInstance();
		if (RobotMode.is(RobotMode.Real)) {
			PdhSubsystem.getInstance();
		}

		// LedSubsystem.getInstance();
		// GameTimerSubsystem.getInstance();
		// CoordinatorSubsystem.getInstance();
		// DriveSubsystem.getInstance();
		// FeederSubsystem.getInstance();
		// IndexerSubsystem.getInstance();
		// IntakeSubsystem.getInstance();
		// ShooterSubsystem.getInstance();
		// VisionSubsystem.getInstance();

		DriveCommands.initialize(() -> m_driverController);
	}

	public void configureBindings() {
		DriveSubsystem.getInstance().setDefaultCommand(
			DriveCommands.getCommand(DriveCommands.DriveType.FO_AngularVelocity));

		m_driverController.button(DriverMap.DRIVE_TO_POSE)
			.whileTrue(DriveCommands.faceAllianceHub());
		m_driverController.button(DriverMap.ZERO_GYRO).onTrue(
			Commands.runOnce(DriveSubsystem.getInstance()::zeroGyro, DriveSubsystem.getInstance()));
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
		m_driverController.button(DriverMap.DIST_FROM_HUB_2METERS).whileTrue(
			DriveSubsystem.getInstance().driveToTargetCommand(2));

		// m_driverController.button(16).onTrue(IntakeSubsystem.getInstance().extendArmCommand());
		// m_driverController.button(15).onTrue(IntakeSubsystem.getInstance().retractArmCommand());

		final double OPERATOR_AXIS_THRESHOLD = 0.1;
		m_operatorController
			.axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, OPERATOR_AXIS_THRESHOLD)
			.whileTrue(
				IndexerSubsystem.getInstance().runOnce(() -> {
					double axis = m_operatorController
						.getRawAxis(XboxController.Axis.kRightY.value);
					if (axis < -OPERATOR_AXIS_THRESHOLD) {
						IndexerSubsystem.getInstance().enable();
					} else
						if (axis > OPERATOR_AXIS_THRESHOLD) {
							IndexerSubsystem.getInstance().enableReverse();
						} else {
							IndexerSubsystem.getInstance().disable();
						}
				}))
			.onFalse(
				IndexerSubsystem.getInstance()
					.runOnce(() -> IndexerSubsystem.getInstance().disable()));
		m_operatorController
			.axisMagnitudeGreaterThan(XboxController.Axis.kLeftY.value, OPERATOR_AXIS_THRESHOLD)
			.whileTrue(
				IntakeSubsystem.getInstance().runOnce(() -> {
					double axis = m_operatorController
						.getRawAxis(XboxController.Axis.kLeftY.value);
					if (axis < -OPERATOR_AXIS_THRESHOLD) {
						IntakeSubsystem.getInstance().enableIntake();
					} else
						if (axis > OPERATOR_AXIS_THRESHOLD) {
							IntakeSubsystem.getInstance().enableSpitout();
						} else {
							IntakeSubsystem.getInstance().disableIntake();
						}
				}))
			.onFalse(IntakeSubsystem.getInstance()
				.runOnce(() -> IntakeSubsystem.getInstance().disableIntake()));
		m_operatorController.y().toggleOnTrue(ShooterSubsystem.getInstance().toggle());
		m_operatorController.b().toggleOnTrue(ShooterSubsystem.getInstance().toggleFeederReverse());
		m_operatorController.leftBumper()
			.toggleOnTrue(CoordinatorSubsystem.getInstance().toggleToShooter());
		m_operatorController.rightBumper()
			.toggleOnTrue(CoordinatorSubsystem.getInstance().toggleToStorage());
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
		m_namedCommands = Map.of(
			"Extend Arm", IntakeSubsystem.getInstance().extendArmCommand(),
			"Retract Arm", IntakeSubsystem.getInstance().retractArmCommand(),
			"Intake", IntakeSubsystem.getInstance().enableIntakeForeverCommand(),
			"Find Pose", DriveSubsystem.getInstance().faceTargetCommand().withTimeout(2),
			"Shoot", AutoActions.newShootCommand(),
			"Force Enable Feeder (Continuous)",
			FeederSubsystem.getInstance().enableCommand().repeatedly(),
			"Shoot (Continuous)", AutoActions.enableCoordinatedShooterForever(),
			"Warmup Flywheel", ShooterSubsystem.getInstance().warmupFlywheelCommand(),
			"UnWarmup Flywheel", ShooterSubsystem.getInstance().cooldownFlywheelCommand(),
			"ZZZ CANCEL ALL", AutoActions.disableCoordinatedShooter());
		NamedCommands.registerCommands(m_namedCommands);
	}

	public Map<String, Command> getNamedCommands() {
		return m_namedCommands;
	}

	public Command getAutonomousCommand() {
		Command selected = m_autoChooser.getSelected();
		if (selected == null)
			return Commands.none();
		return selected;
	}

	public static final class ControllerActions {
		public static Command enableCoordinatedIntake() {
			return Commands.runOnce(() -> {
				IntakeSubsystem.getInstance().enableIntake();
				IndexerSubsystem.getInstance().enable();
			}, IntakeSubsystem.getInstance(), IndexerSubsystem.getInstance());
		}

		public static Command enableCoordinatedIntakeReverse() {
			return Commands.runOnce(() -> {
				IntakeSubsystem.getInstance().enableSpitout();
				IndexerSubsystem.getInstance().enableReverse();
			}, IntakeSubsystem.getInstance(), IndexerSubsystem.getInstance());
		}

		public static Command disableCoordinatedIntake() {
			return Commands.runOnce(() -> {
				IntakeSubsystem.getInstance().disableIntake();
				IndexerSubsystem.getInstance().disable();
			}, IntakeSubsystem.getInstance(), IndexerSubsystem.getInstance());
		}
	}

	public static final class AutoActions {
		public static Command enableCoordinatedShooter() {
			return Commands.runOnce(() -> {
				SmartDashboard.putBoolean("Auto Coordinated Shooter Enabled", true);
				ShooterSubsystem.getInstance().enableForward();
				FeederSubsystem.getInstance().enableForward();
				IndexerSubsystem.getInstance().enable();
				CoordinatorSubsystem.getInstance().enableToShooter();
			}, ShooterSubsystem.getInstance(), IndexerSubsystem.getInstance(),
				CoordinatorSubsystem.getInstance(), FeederSubsystem.getInstance());
			// .finallyDo(() -> {
			// SmartDashboard.putBoolean("Auto Coordinated Shooter Enabled", false);
			// ShooterSubsystem.getInstance().disable();
			// FeederSubsystem.getInstance().disable();
			// IndexerSubsystem.getInstance().disable();
			// CoordinatorSubsystem.getInstance().disable();
			// });
		}

		public static Command enableCoordinatedShooterForever() {
			return Commands.run(() -> {
				SmartDashboard.putBoolean("Auto Coordinated Shooter Enabled", true);
				ShooterSubsystem.getInstance().enableForward();
				FeederSubsystem.getInstance().enableForward();
				IndexerSubsystem.getInstance().enable();
				CoordinatorSubsystem.getInstance().enableToShooter();
			}, ShooterSubsystem.getInstance(), IndexerSubsystem.getInstance(),
				CoordinatorSubsystem.getInstance(), FeederSubsystem.getInstance()).repeatedly();
		}

		public static Command disableCoordinatedShooter() {
			return new InstantCommand(() -> {
				SmartDashboard.putBoolean("Auto Coordinated Shooter Enabled", false);
				ShooterSubsystem.getInstance().disable();
				FeederSubsystem.getInstance().disable();
				IndexerSubsystem.getInstance().disable();
				CoordinatorSubsystem.getInstance().disable();
			}, ShooterSubsystem.getInstance(), IndexerSubsystem.getInstance(),
				CoordinatorSubsystem.getInstance(), FeederSubsystem.getInstance());
		}

		public static Command enableFeederForever() {
			return FeederSubsystem.getInstance().enableCommand();
		}

		public static Command newShootCommand() {
			return Commands.parallel(
				ShooterSubsystem.getInstance().enableForeverCommand().repeatedly(),
				FeederSubsystem.getInstance().enableForeverCommand().repeatedly(),
				IndexerSubsystem.getInstance().enableForeverCommand().repeatedly(),
				CoordinatorSubsystem.getInstance().enableToShooterForeverCommand().repeatedly());
		}
	}
}
