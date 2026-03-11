package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.Supplier;

public class RotateToAngleCommand extends Command {
	private final DriveSubsystem drive;
	private final Supplier<Rotation2d> targetRotationSupplier;
	private final ProfiledPIDController thetaController;

	public RotateToAngleCommand(
		DriveSubsystem drive,
		Supplier<Rotation2d> targetRotationSupplier) {
		this.drive = drive;
		this.targetRotationSupplier = targetRotationSupplier;

		TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
			drive.getSwerveDrive().getMaximumChassisAngularVelocity(), // rad/s
			Constants.Robot.MAX_ANGULAR_ACCELERATION.in(Units.RadiansPerSecondPerSecond) // rad/s^2
		);

		thetaController = new ProfiledPIDController(
			Constants.Drive.ROTATE_TO_ANGLE_PID.kP(),
			Constants.Drive.ROTATE_TO_ANGLE_PID.kI(),
			Constants.Drive.ROTATE_TO_ANGLE_PID.kD(),
			constraints);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		addRequirements(drive);
	}

	@Override
	public void initialize() {
		Rotation2d target = targetRotationSupplier.get();
		thetaController.reset(drive.getPose().getRotation().getRadians());
		thetaController.setGoal(target.getRadians());
	}

	@Override
	public void execute() {
		double currentTheta = drive.getPose().getRotation().getRadians();
		double outputOmega = thetaController.calculate(currentTheta);
		// Convert to chassis speeds: zero x,y, angular = outputOmega
		ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, outputOmega);
		drive.drive(speeds);
	}

	public static double normalizeRadians(double angle) {
		return Math.atan2(Math.sin(angle), Math.cos(angle));
	}

	@Override
	public boolean isFinished() {
		double currentTheta = drive.getPose().getRotation().getRadians();
		double error = Math.abs(normalizeRadians(
			thetaController.getGoal().position - currentTheta));
		return error < Math.toRadians(4.0); // epsilon: 2 degrees
	}

	@Override
	public void end(boolean interrupted) {
		drive.stop();
	}
}
