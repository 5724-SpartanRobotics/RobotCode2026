package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.info.constants.ControllerConstants;
import frc.robot.info.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import swervelib.SwerveInputStream;

public class DriveCommands {
	private static DriveSubsystem m_driveSubsystem;
	private static CommandJoystick m_joystick;

	public static double speedMod = RobotConstants.DEFAULT_SPEED_MOD;

	public static void initialize(CommandJoystick joystick) {
		m_driveSubsystem = DriveSubsystem.getInstance();
		m_joystick = joystick;
		SmartDashboard.putBoolean("DriveCommands Initialized", true);
	}

	private static double applyJoystickDeadband(double realValue, double deadband) {
		if (Math.abs(realValue) < deadband)
			return 0.0;
		return realValue;
	}

	private static double applyJoystickScale(double realValue, double scale) {
		return realValue * scale;
	}

	private static double applyJoystickDeadbandAndScale(double realValue, double deadband) {
		return applyJoystickScale(
			applyJoystickDeadband(realValue, deadband),
			speedMod);
	}

	public static Command setSpeedModCommand(double s) {
		return Commands.runOnce(() -> speedMod = s, m_driveSubsystem);
	}

	public static Command resetSpeedModCommand() {
		return Commands.runOnce(() -> speedMod = RobotConstants.DEFAULT_SPEED_MOD,
			m_driveSubsystem);
	}

	public static SwerveInputStream DriveAngularVelocity() {
		return SwerveInputStream.of(
			m_driveSubsystem.getSwerveDrive(),
			() -> applyJoystickDeadbandAndScale(
				m_joystick.getRawAxis(1) * -1.0 /*
												 * * (Constants.isRedAlliance() ? -1.0 : 1.0)
												 */,
				ControllerConstants.DRIVER_DEADBAND_XY), // Y axis (forward/back)
			() -> applyJoystickDeadbandAndScale(
				m_joystick.getRawAxis(0) * -1.0 /*
												 * * (Constants.isRedAlliance() ? -1.0 : 1.0)
												 */,
				ControllerConstants.DRIVER_DEADBAND_XY) // X axis (strafe)
		)
			.withControllerRotationAxis(() -> applyJoystickDeadbandAndScale(
				// Gonna invert this based on alliance, idk if that's right (we'll see later)
				m_joystick.getRawAxis(2) * -1.0 /*
												 * * (Constants.isRedAlliance() ? -1.0 : 1.0)
												 */,
				ControllerConstants.DRIVER_DEADBAND_Z)) // twist / rotation
			.deadband(ControllerConstants.DRIVER_DEADBAND_Z)
			.scaleTranslation(1.05)
			.allianceRelativeControl(true);
	}

	public static SwerveInputStream DriveRobotOriented() {
		// System.out.println("RETURNING A DRIVE COMMAND: DRO");
		return DriveAngularVelocity().copy()
			.robotRelative(false)
			.allianceRelativeControl(false);
	}

	public enum DriveType {
		FO_AngularVelocity, FO_DirectAngle, RO_AngularVelocity;
	}

	public static Command getCommand(DriveType t) {
		return switch (t) {
			case FO_AngularVelocity -> m_driveSubsystem.driveFieldOriented(DriveAngularVelocity());
			case RO_AngularVelocity -> m_driveSubsystem.driveFieldOriented(DriveRobotOriented());
			default -> Commands.none();
		};
	}
}