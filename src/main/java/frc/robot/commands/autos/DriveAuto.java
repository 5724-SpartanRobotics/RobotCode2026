package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class DriveAuto extends SequentialCommandGroup {
    public DriveAuto(DriveSubsystem driveSubsystem) {
        addRequirements(driveSubsystem);
        addCommands(Commands.parallel(
            Commands.runOnce(() -> System.out.println("<<<<< Auto >>>>>")).repeatedly(),
            driveSubsystem.driveForward().withDeadline(Commands.waitSeconds(5))
        ));
    }
}
