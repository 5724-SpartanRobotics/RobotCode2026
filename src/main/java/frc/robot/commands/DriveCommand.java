package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import swervelib.SwerveInputStream;

public class DriveCommand {
    public static SwerveInputStream command;

	public static void initialize(DriveSubsystem drive, CommandJoystick joystick) {
		command = SwerveInputStream.of(
            drive.getSwerveDrive(),
            () -> joystick.getRawAxis(1) * -1.0, // Y axis (forward/back)
            () -> joystick.getRawAxis(0)  // X axis (strafe)
        )
            .withControllerRotationAxis(() -> joystick.getRawAxis(2)) // twist / rotation
            .deadband(Constants.Controller.DEFAULT_DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);
	}
}
