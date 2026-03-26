package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.info.constants.DriveConstants;
import frc.robot.info.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class FacePointDriveCommand extends Command {
	private final DriveSubsystem drive;

	private final Supplier<Double> xSupplier;
	private final Supplier<Double> ySupplier;
	private final Supplier<Translation2d> targetSupplier;
	private final Supplier<Rotation2d> offsetSupplier;

	private final ProfiledPIDController thetaController;

	public FacePointDriveCommand(
		Supplier<Double> xSupplier,
		Supplier<Double> ySupplier,
		Supplier<Translation2d> targetSupplier) {
		this(xSupplier, ySupplier, targetSupplier,
			() -> Rotation2d.kZero);
	}

	public FacePointDriveCommand(
		Supplier<Double> xSupplier,
		Supplier<Double> ySupplier,
		Supplier<Translation2d> targetSupplier,
		Supplier<Rotation2d> offsetBy) {
		this.drive = DriveSubsystem.getInstance();
		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;
		this.targetSupplier = targetSupplier;
		this.offsetSupplier = offsetBy;

		thetaController = new ProfiledPIDController(
			DriveConstants.ROTATE_TO_ANGLE_PID.kP(),
			DriveConstants.ROTATE_TO_ANGLE_PID.kI(),
			DriveConstants.ROTATE_TO_ANGLE_PID.kD(),
			new TrapezoidProfile.Constraints(
				RobotConstants.MAX_ANGULAR_VELOCITY.in(Units.RadiansPerSecond),
				RobotConstants.MAX_ANGULAR_ACCELERATION.in(Units.RadiansPerSecondPerSecond)));

		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		addRequirements(drive);
	}

	@Override
	public void initialize() {
		thetaController.reset(drive.getPose().getRotation().getRadians());
	}

	@Override
	public void execute() {
		var pose = drive.getPose();
		var robotPos = pose.getTranslation();
		var target = targetSupplier.get();

		// Compute desired angle every loop
		Rotation2d angleToTarget = new Rotation2d(
			target.getX() - robotPos.getX(),
			target.getY() - robotPos.getY());

		Rotation2d desiredRotation = angleToTarget.plus(offsetSupplier.get());
		double desiredAngle = desiredRotation.getRadians();
		double currentTheta = pose.getRotation().getRadians();

		thetaController.setGoal(desiredAngle);
		double omega = thetaController.calculate(currentTheta);

		// Driver translation
		double xSpeed = xSupplier.get();
		double ySpeed = ySupplier.get();

		ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
			xSpeed,
			ySpeed,
			omega,
			pose.getRotation()).times(3.05);

		drive.getSwerveDrive().driveFieldOriented(speeds);
	}

	@Override
	public boolean isFinished() {
		return false; // runs while held
	}

	@Override
	public void end(boolean interrupted) {
		drive.stop();
	}
}