package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToAngleCommand extends Command {
    private final DriveSubsystem drive;
    private final Supplier<Rotation2d> targetRotationSupplier;
    private final ProfiledPIDController thetaController;

    public RotateToAngleCommand(DriveSubsystem drive, Supplier<Rotation2d> targetRotationSupplier) {
        this.drive = drive;
        this.targetRotationSupplier = targetRotationSupplier;

        // Tunable gains and constraints
        double kP = 3.0;
        double kI = 0.0;
        double kD = 0.0;
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            drive.getSwerveDrive().getMaximumChassisAngularVelocity(), // rad/s
            Units.Degrees.of(720).in(Units.Radians)  // rad/s^2
        );

        thetaController = new ProfiledPIDController(kP, kI, kD, constraints);
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
        double error = Math.abs(normalizeRadians(thetaController.getGoal().position - currentTheta));
        return error < Math.toRadians(2.0); // epsilon: 2 degrees
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}