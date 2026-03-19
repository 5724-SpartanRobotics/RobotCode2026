package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.Pathfinder_LocalADStarAK;
import frc.robot.info.Alliance;

public final class PathPlanner {
	private static final Alert kSetupErrorAlert = new Alert("PathPlanner setup failed",
		AlertType.kError);

	public static void configure(DriveSubsystem sub) {
		RobotConfig config;
		try {
			config = RobotConfig.fromGUISettings();
			final boolean enableFeedForward = true;

			AutoBuilder.configure(
				sub::getPose,
				sub::resetOdometry,
				sub::getRobotVelocity,
				(speedsRobotRelative, moduleFeedForwards) -> {
					if (enableFeedForward) {
						sub.getSwerveDrive().drive(
							speedsRobotRelative,
							sub.getSwerveDrive().kinematics
								.toSwerveModuleStates(speedsRobotRelative),
							moduleFeedForwards.linearForces());
					} else {
						sub.getSwerveDrive().setChassisSpeeds(speedsRobotRelative);
					}
				},
				new PPHolonomicDriveController(
					// TODO: Do these PIDs need adjusted?
					new PIDConstants(5.0, 0.0, 0.0),
					new PIDConstants(5.0, 0.0, 0.0)),
				config,
				Alliance::isRedAlliance,
				sub);
		} catch (Exception e) {
			kSetupErrorAlert.set(true);
			DriverStation.reportError(sub.getName(), e.getStackTrace());
		}

		// Configure custom pathfinding for dynamic obstacle avoidance
		Pathfinding.setPathfinder(new Pathfinder_LocalADStarAK());

		// Log active path for visualization
		PathPlannerLogging.setLogActivePathCallback(
			(activePath) -> {
				Logger.recordOutput(
					"Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
			});

		// Log target pose for visualization
		PathPlannerLogging.setLogTargetPoseCallback(
			(targetPose) -> {
				Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
			});

		CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
	}
}
