// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.commands.RotateToAngleCommand;
import frc.robot.subsystems.VisionSubsystem.Cameras;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * {@link https://github.com/Yet-Another-Software-Suite/YAGSL/blob/main/examples/drivebase_with_PhotonVision/src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java}
 */
public class DriveSubsystem extends SubsystemBase {
	private final Pose2d kInitialPose;

	private final Pose2d kInitialPoseRed = new Pose2d(
		new Translation2d(12.873565673828125, 1.8854589462280273),
		Rotation2d.k180deg
	);
	private final Pose2d kInitialPoseBlue = new Pose2d(
		new Translation2d(3.5681846141815186, 6.073807716369629),
		Rotation2d.kZero
	);
	/**
	 * Swerve drive object.
	 */
	private final SwerveDrive m_swerveDrive;

	private final boolean _IsVisionDriveTest =
		Constants.DebugLevel.isOrAll(Constants.DebugLevel.Vision);

	private VisionSubsystem m_visionSubsystem;

	private static void configureTelemetry() {
		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
	}

	/**
	 * Initialize {@link SwerveDrive} with the directory provided.
	 *
	 * @param directory Directory of swerve drive config files.
	 */
	public DriveSubsystem(File directory)
	{ 

		boolean blueAlliance = !Constants.isRedAlliance();
		
		// 2026-02-02: Commenting this out for testing right now, we'll just use the initial pose
		// as written in the readme (12.873565673828125, 1.8854589462280273) meters, always-blue
		// origin. Since we are facing the blue alliance, we'll need to rotate the robot 180deg.
		// Pose2d startingPose = blueAlliance ?
		// 	new Pose2d(
		// 		new Translation2d(Meter.of(1), Meter.of(4)),
		// 		Rotation2d.fromDegrees(0)
		// 	) : new Pose2d(
		// 		new Translation2d(Meter.of(16), Meter.of(4)),
		// 		Rotation2d.fromDegrees(180)
		// 	);
		Pose2d startingPose = blueAlliance ? kInitialPoseBlue : kInitialPoseRed;
		kInitialPose = startingPose;

		// Configure the Telemetry before creating the SwerveDrive to avoid
		// unnecessary objectsbeing created.
		configureTelemetry();

		try {
			m_swerveDrive = new SwerveParser(directory)
				.createSwerveDrive(
					Constants.Robot.MAX_LINEAR_VELOCITY.in(Units.MetersPerSecond),
					startingPose
			);
			// Alternative method if you don't want to supply the conversion factor via JSON files.
		} catch (Exception e) {
			throw new RuntimeException(e);
		}

		// Heading correction should only be used while controlling the robot via angle.
		m_swerveDrive.setHeadingCorrection(false);
		m_swerveDrive.setCosineCompensator(false);
		//Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
		m_swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
		// Enable if you want to resynchronize your absolute encoders and motor
		// encoders periodically when they are not moving.
		m_swerveDrive.setModuleEncoderAutoSynchronize(true, 1);

		setupPhotonVision();
		if (_IsVisionDriveTest) {
			m_swerveDrive.stopOdometryThread();
		}
		setupPathPlanner();

		SmartDashboard.putString("VisSubSysMsg", "<empty string>");
	}

	/**
	 * Construct the swerve drive.
	 *
	 * @param driveCfg SwerveDriveConfiguration for the swerve.
	 * @param controllerCfg Swerve Controller.
	 */
	public DriveSubsystem(
		SwerveDriveConfiguration driveCfg,
		SwerveControllerConfiguration controllerCfg
	) {
		Pose2d pose = new Pose2d(
			new Translation2d(Meter.of(2), Meter.of(0)),
			Rotation2d.fromDegrees(0)
		);
		kInitialPose = pose;
		configureTelemetry();
		m_swerveDrive = new SwerveDrive(
			driveCfg,
			controllerCfg,
			Constants.Robot.MAX_LINEAR_VELOCITY.in(Units.MetersPerSecond),
			pose
		);
	}

	public void setupPhotonVision() {
		m_visionSubsystem = new VisionSubsystem(m_swerveDrive::getPose, m_swerveDrive.field);
		m_visionSubsystem.setDriveSubsystem(this);

		SmartDashboard.putString("Tag9Pose", m_visionSubsystem.getTagPose(9).get().toString());
	}

	public void setupPathPlanner() {
		RobotConfig config;
		try {
			config = RobotConfig.fromGUISettings();
			final boolean enableFeedForward = true;

			AutoBuilder.configure(
				this::getPose,
				this::resetOdometry,
				this::getRobotVelocity,
				(speedsRobotRelative, moduleFeedForwards) -> {
					if (enableFeedForward) {
						m_swerveDrive.drive(
							speedsRobotRelative,
							m_swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
							moduleFeedForwards.linearForces()
						);
					} else {
						m_swerveDrive.setChassisSpeeds(speedsRobotRelative);
					}
				},
				new PPHolonomicDriveController(
					// TODO: Do these PIDs need adjusted?
					new PIDConstants(5.0, 0.0, 0.0),
					new PIDConstants(5.0, 0.0, 0.0)
				),
				config,
				() -> Constants.isRedAlliance(),
				this
			);
		} catch (Exception e) {
			DriverStation.reportError(getName(), e.getStackTrace());
		}

		CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
	}

	private Optional<PhotonPipelineResult> getBestAcrossAllCameras() {
		return Arrays.stream(VisionSubsystem.Cameras.values())
			.map(VisionSubsystem.Cameras::getLatestResult)
			.filter(Objects::nonNull)
			.filter(Optional::isPresent)
			.map(Optional::get)
			.filter(r -> r.hasTargets())
			.min(Comparator.comparingDouble(r -> r.getBestTarget().getPoseAmbiguity()));
	}

	public Command aimAtTarget(Cameras camera) {
		if (camera == null) return Commands.none();
		return run(() -> {
			Optional<PhotonPipelineResult> resultO = camera.getBestResult();
			if (resultO == null || resultO.isEmpty()) return;
			PhotonPipelineResult result = resultO.get();
			if (!result.hasTargets()) return;
			PhotonTrackedTarget target = result.getBestTarget();
			if (target == null) return;
			double yaw = target.getYaw();
			this.drive(
				this.getTargetSpeeds(
					0, 0,
					Rotation2d.fromDegrees(yaw).unaryMinus().plus(getHeading())
				)
			);
		});
	}

	public Command aimAtTarget() {
		double x_override = SmartDashboard.getNumber("Aim At Target/X", 0);
		double y_override = SmartDashboard.getNumber("Aim At Target/Y", 0);
		double theta_override = SmartDashboard.getNumber("Aim At Target/Theta (Degrees)", 0);
		double aim_constant = SmartDashboard.getNumber("Aim At Target/Aim Constant (Degrees)", 30);
		return run(() -> {
			SmartDashboard.putBoolean("Photon/Ok", false);
			Optional<PhotonPipelineResult> resultO = getBestAcrossAllCameras();
			if (resultO == null || resultO.isEmpty()) return;
			PhotonPipelineResult result = resultO.get();
			if (!result.hasTargets()) return;
			PhotonTrackedTarget target = result.getBestTarget();
			if (target == null) return;
			SmartDashboard.putBoolean("Photon/Ok", true);
			SmartDashboard.putString("Photon/Pipeline Result", result.toString());
			SmartDashboard.putString("Photon/Best Target", target.toString());
			double yaw = target.getYaw();
			var gyroRotation = m_swerveDrive.getGyroRotation3d().toRotation2d().unaryMinus().times(2.0);
			this.drive(
				this.getTargetSpeeds(
					x_override, y_override,
					Rotation2d.fromDegrees(theta_override == 0 ? yaw : theta_override).unaryMinus()
						.plus(gyroRotation).plus(Rotation2d.fromDegrees(aim_constant))
				)
			);
		});
	}

	private Pose2d visionTargetPose = new Pose2d();
	public Command driveToTarget() {
		final double allSpeedsMultiplier = 0.8;
		return Commands.sequence(
			Commands.run(() -> {
				m_visionSubsystem.periodic();
				var resList = VisionSubsystem.Cameras.CENTER_CAM.resultsList;

				if (resList.isEmpty()) {
					SmartDashboard.putString("VisSubSysMsg", "driveToTarget: camera resultList empty");
					return;
				}

				// DIRECT camera read — no caching, no unread draining
				PhotonPipelineResult result = resList.get(0);

				if (!result.hasTargets()) {
					SmartDashboard.putString("VisSubSysMsg", "driveToTarget: no targets in latest result");
					return;
				}

				Pose2d tagPoseForGoal;

				// Try multi-tag if available
				var multiO = result.getMultiTagResult(); // may be null depending on PV version
				if (multiO.isPresent() && !multiO.get().fiducialIDsUsed.isEmpty()) {
					var multi = multiO.get();
					var ids = multi.fiducialIDsUsed;

					var poses = ids.stream()
						.map(m_visionSubsystem::getTagPose)
						.flatMap(Optional::stream)
						.toList();

					if (!poses.isEmpty()) {
						double meanX = poses.stream().mapToDouble(Pose2d::getX).average().getAsDouble();
						double meanY = poses.stream().mapToDouble(Pose2d::getY).average().getAsDouble();
						tagPoseForGoal = new Pose2d(meanX, meanY, new Rotation2d());
					} else {
						SmartDashboard.putString("VisSubSysMsg","driveToTarget: multi-tag IDs not in field layout");
						return;
					}

				} else {
					// FALLBACK: single tag
					PhotonTrackedTarget target = result.getBestTarget();
					if (target == null) {
						SmartDashboard.putString("VisSubSysMsg","driveToTarget: bestTarget is null");
						return;
					}

					var tagPoseO = m_visionSubsystem.getTagPose(target.getFiducialId());
					if (tagPoseO.isEmpty()) {
						SmartDashboard.putString("VisSubSysMsg","driveToTarget: tag " + target.getFiducialId() + " not in layout");
						return;
					}

					tagPoseForGoal = tagPoseO.get();
				}

				// 1 meter behind the tag
				Transform2d offset = new Transform2d(-1.0, 0.0, new Rotation2d());
				Pose2d goalPose = tagPoseForGoal.transformBy(offset);

				// Face the tag
				Rotation2d facing = tagPoseForGoal.getTranslation()
					.minus(goalPose.getTranslation())
					.getAngle();
				goalPose = new Pose2d(goalPose.getTranslation(), facing);
				visionTargetPose = goalPose;

				SmartDashboard.putString("VisSubSysMsg","driveToTarget: goalPose = " + visionTargetPose);
			}),
			AutoBuilder.pathfindToPose(
				visionTargetPose,
				new PathConstraints(
					m_swerveDrive.getMaximumChassisVelocity() * allSpeedsMultiplier,
					Constants.Robot.MAX_LINEAR_ACCELERATION
						.times(allSpeedsMultiplier)
						.in(Units.MetersPerSecondPerSecond),
					m_swerveDrive.getMaximumChassisAngularVelocity() * allSpeedsMultiplier,
					Units.Degrees.of(720).in(Units.Radians)
				)
			)
		);
	}

	public Command driveToTargetCommandOld() {
		final double allSpeedsMultiplier = 0.8;
		var tagPoseO = m_visionSubsystem.getTagPose(9);
		if (tagPoseO.isEmpty()) return Commands.none();
		var pose = tagPoseO.get();
		double offsetXMeters = Constants.isRedAlliance() ? 1.0 : -1.0;
		double offsetYMeters = pose.getY() > 3.7 /* past the center of the HUB */ ? 1.0 : -1.0;
		// Right now this drives to tag 9, y-1m and 1m closer to the red driver station, facing an angle of 1rad
		// (idk if that's right). I'd like a way to stay 1m+23in from the center of the HUB at all times, having
		// a rotation that always faces the center of the HUB. I don't know the correct math to do this, and also
		// what offsets are necessary based on Y component and alliance (X component). I'm thinking that this will
		// more easily be done by using polar, staying that radius away (1m+23in), and then somehow computing the
		// theta on the fly using the best tag pose and the robot's current gyro heading. The math should be simple
		// but I barely learned geometry/trig so IDK how to do it. Then we'd have to convert from polar back to
		// Cartesian/rectangular and add the offsets to the tag pose (and its offset the the center of the HUB),
		// and this would also have to be mindful of alliance/theta. Overall not hard but IDK trig so it is. 
		return this.driveToPose(
			new Pose2d(pose.getTranslation().plus(
				new Translation2d(offsetXMeters, offsetYMeters)
			), Rotation2d.k180deg.minus(Rotation2d.fromRadians(0.5))),
			allSpeedsMultiplier
		);
	}

	public Command driveToTargetCommand2() {
		final double allSpeedsMultiplier = 0.8;
		return this.driveToPose(() -> {
			m_visionSubsystem.periodic();
			final Pose2d robotPose = getPose();
			Translation2d robotTranslation = robotPose.getTranslation();
			Translation2d targetCenterTranslation = Constants.isRedAlliance() ? Constants.Field.RED_HUB_CENTER : Constants.Field.BLUE_HUB_CENTER;
			double diffX = targetCenterTranslation.getX() - robotTranslation.getX();
			double diffY = targetCenterTranslation.getY() - robotTranslation.getY();
			Angle requiredRotation = Units.Radians.of(Math.atan2(diffY, diffX));
			double absV = Math.sqrt( Math.pow(diffX, 2) + Math.pow(diffY, 2) );
			double x_new = targetCenterTranslation.getX() - ( 2.0 * ((diffX) / (absV)) );
			double y_new = targetCenterTranslation.getY() - ( 2.0 * ((diffY) / (absV)) );
			Angle newHdg = Units.Radians.of(Math.atan2(targetCenterTranslation.getY() - y_new, targetCenterTranslation.getX() - x_new));
			Angle rotationAvg = requiredRotation.plus(newHdg).div(2);
			return new Pose2d(new Translation2d(x_new, y_new), new Rotation2d(rotationAvg));
		}, allSpeedsMultiplier);
	}

	public Command driveToTargetCommand() {
		return new InstantCommand(() -> {
			// 1) Read current robot pose and compute the offset pose ONCE
			Pose2d currentPose = getPose(); // ensure odometry is up-to-date
			Translation2d robotTranslation = currentPose.getTranslation();
			Translation2d hub = Constants.isRedAlliance()
				? Constants.Field.RED_HUB_CENTER
				: Constants.Field.BLUE_HUB_CENTER;

			double diffX = hub.getX() - robotTranslation.getX();
			double diffY = hub.getY() - robotTranslation.getY();
			double dist = Math.hypot(diffX, diffY);
			double ux = diffX / dist;
			double uy = diffY / dist;

			double x_new = hub.getX() - 2.0 * ux;
			double y_new = hub.getY() - 2.0 * uy;
			Rotation2d heading = new Rotation2d(Math.atan2(diffY, diffX));

			Pose2d staticTarget = new Pose2d(new Translation2d(x_new, y_new), heading);

			Command pathCmd = this.driveToPose(staticTarget, 0.8);
			Command rotCmd = new RotateToAngleCommand(this, () -> staticTarget.getRotation());
			CommandScheduler.getInstance().schedule(pathCmd.andThen(rotCmd));
		}, this);
	}

	public Distance getHypotToAllianceHub() {
		Pose2d currentPose = getPose(); // ensure odometry is up-to-date
		Translation2d robotTranslation = currentPose.getTranslation();
		Translation2d hub = Constants.isRedAlliance()
			? Constants.Field.RED_HUB_CENTER
			: Constants.Field.BLUE_HUB_CENTER;
		double diffX = hub.getX() - robotTranslation.getX();
		double diffY = hub.getY() - robotTranslation.getY();
		double dist = Math.hypot(diffX, diffY);
		return Units.Meters.of(dist);
	}

	public Command driveToInitialPosition() {
		return this.driveToPose(kInitialPose);
	}

	public Command driveToInitialPosition(double maxSpeedMultiplier) {
		// THIS METHOD WORKS SO WELL
		// but only when the robot is about >1 meter from the inital pose
		return this.driveToPose(kInitialPose, maxSpeedMultiplier);
	}

	public Command getAutonomousCommand(String pathName) {
		return new PathPlannerAuto(pathName);
	}

	public Command driveToPose(Pose2d pose) {
		PathConstraints constraints = new PathConstraints(
			m_swerveDrive.getMaximumChassisVelocity(),
			Constants.Robot.MAX_LINEAR_ACCELERATION.in(Units.MetersPerSecondPerSecond),
			m_swerveDrive.getMaximumChassisAngularVelocity(),
			Units.Degrees.of(720).in(Units.Radians)
		);
		return AutoBuilder.pathfindToPose(pose, constraints, Units.MetersPerSecond.of(0));
	}

	public Command driveToPose(Pose2d pose, double allSpeedsMultiplier) {
		PathConstraints c = new PathConstraints(
			m_swerveDrive.getMaximumChassisVelocity() * allSpeedsMultiplier,
			Constants.Robot.MAX_LINEAR_ACCELERATION.times(allSpeedsMultiplier).in(Units.MetersPerSecondPerSecond),
			m_swerveDrive.getMaximumChassisAngularVelocity() * allSpeedsMultiplier,
			Units.Degrees.of(720).in(Units.Radians)
		);
		return AutoBuilder.pathfindToPose(pose, c, Units.MetersPerSecond.of(0));
	}

	public Command driveToPose(Supplier<Pose2d> poseSupplier, double allSpeedsMultiplier) {
		return driveToPose(poseSupplier.get(), allSpeedsMultiplier);
	}

	public void stop() {
		this.drive(new ChassisSpeeds());
	}

	private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeeds)
	throws IOException, org.json.simple.parser.ParseException {
		SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
			RobotConfig.fromGUISettings(),
			m_swerveDrive.getMaximumChassisAngularVelocity()
		);
		AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
			new SwerveSetpoint(
				m_swerveDrive.getRobotVelocity(),
				m_swerveDrive.getStates(),
				DriveFeedforwards.zeros(m_swerveDrive.getModules().length)
			)
		);
		AtomicReference<Double> previousTime = new AtomicReference<>();

		return startRun(
			() -> previousTime.set(Timer.getFPGATimestamp()),
			() -> {
				double newTime = Timer.getFPGATimestamp();
				SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(
					prevSetpoint.get(),
					robotRelativeChassisSpeeds.get(),
					newTime - previousTime.get()
				);

				m_swerveDrive.drive(
					newSetpoint.robotRelativeSpeeds(),
					newSetpoint.moduleStates(),
					newSetpoint.feedforwards().linearForces()
				);
				prevSetpoint.getAndSet(newSetpoint);
				previousTime.set(newTime);
			}
		);
	}

	public Command driveWithSetpointGeneratorFieldRelative(
		Supplier<ChassisSpeeds> fieldRelativeSpeeds
	) {
		try {
			return driveWithSetpointGenerator(() -> {
				return ChassisSpeeds.fromFieldRelativeSpeeds(
					fieldRelativeSpeeds.get(), getHeading()
				);
			});
		} catch (Exception e) {
			DriverStation.reportError(e.toString(), true);
		}
		return Commands.none();
	}

	@Override
	public void periodic() {
		if (_IsVisionDriveTest) {
		}
		m_swerveDrive.updateOdometry();
		// 2026-02-02: Disable updating the vision pose estimation for testing
		// we leave the swerve drive odometry cause it's visionless and is very necessary
		// This is the old "periodic" function from the YAGSL example
		// m_visionSubsystem.updatePoseEstimation(m_swerveDrive);
		// This is the "new" periodic function from the Photon example.
		// I'm going to leave this in cause it will only update NT, not add to the Drive pose estimate
		m_visionSubsystem.periodic();
	}

	@Override
	public void simulationPeriodic() {}

	public void addVisionMeasurement(Pose2d pose, double timestamp)
	{
		m_swerveDrive.addVisionMeasurement(pose, timestamp);
	}

	public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs)
	{
		m_swerveDrive.addVisionMeasurement(pose, timestamp, stdDevs);
	}

	/**
	 * Command to characterize the robot drive motors using SysId
	 *
	 * @return SysId Drive Command
	 */
	public Command sysIdDriveMotorCommand()
	{
		return SwerveDriveTest.generateSysIdCommand(
			SwerveDriveTest.setDriveSysIdRoutine(
				new Config(),
				this,
				m_swerveDrive,
				12,
				true
			),
			3.0,
			5.0,
			3.0
		);
	}

	/**
	 * Command to characterize the robot angle motors using SysId
	 *
	 * @return SysId Angle Command
	 */
	public Command sysIdAngleMotorCommand()
	{
		return SwerveDriveTest.generateSysIdCommand(
			SwerveDriveTest.setAngleSysIdRoutine(
				new Config(),
				this,
				m_swerveDrive
			),
			3.0,
			5.0,
			3.0
		);
	}

	/**
	 * Returns a Command that centers the modules of the SwerveDrive subsystem.
	 *
	 * @return a Command that centers the modules of the SwerveDrive subsystem
	 */
	public Command centerModulesCommand()
	{
		return run(
			() -> Arrays.asList(m_swerveDrive.getModules()).forEach(it -> it.setAngle(0.0))
		);
	}

	/**
	 * Returns a Command that tells the robot to drive forward until the command ends.
	 *
	 * @return a Command that tells the robot to drive forward until the command ends
	 */
	public Command driveForward()
	{
		return run(() -> {
			m_swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
		}).finallyDo(() -> m_swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
	}


	/**
	 * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
	 *
	 * @param kS the static gain of the feedforward
	 * @param kV the velocity gain of the feedforward
	 * @param kA the acceleration gain of the feedforward
	 */
	public void replaceSwerveModuleFeedforward(double kS, double kV, double kA)
	{
		m_swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
	}

	/**
	 * Command to drive the robot using translative values and heading as angular velocity.
	 *
	 * @param translationX		 Translation in the X direction. Cubed for smoother controls.
	 * @param translationY		 Translation in the Y direction. Cubed for smoother controls.
	 * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
	 * @return Drive command.
	 */
	public Command driveCommand(
		DoubleSupplier translationX,
		DoubleSupplier translationY,
		DoubleSupplier angularRotationX
	) {
		return run(() -> {
			// Make the robot move
			m_swerveDrive.drive(
				SwerveMath.scaleTranslation(
					new Translation2d(
						translationX.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity(),
						translationY.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity()
					),
					0.8
				),
				Math.pow(angularRotationX.getAsDouble(), 3) *
					m_swerveDrive.getMaximumChassisAngularVelocity(),
				true,
				false
			);
		});
	}

	/**
	 * Command to drive the robot using translative values and heading as a setpoint.
	 *
	 * @param translationX Translation in the X direction. Cubed for smoother controls.
	 * @param translationY Translation in the Y direction. Cubed for smoother controls.
	 * @param headingX		 Heading X to calculate angle of the joystick.
	 * @param headingY		 Heading Y to calculate angle of the joystick.
	 * @return Drive command.
	 */
	public Command driveCommand(
		DoubleSupplier translationX,
		DoubleSupplier translationY,
		DoubleSupplier headingX,
		DoubleSupplier headingY
	)
	{
		return run(() -> {
			Translation2d scaledInputs = SwerveMath.scaleTranslation(
				new Translation2d(
					translationX.getAsDouble(),
					translationY.getAsDouble()
				),
				0.8
			);

			// Make the robot move
			driveFieldOriented(
				m_swerveDrive.swerveController.getTargetSpeeds(
					scaledInputs.getX(), scaledInputs.getY(),
					headingX.getAsDouble(), headingY.getAsDouble(),
					m_swerveDrive.getOdometryHeading().getRadians(),
					m_swerveDrive.getMaximumChassisVelocity()
				)
			);
		});
	}

	/**
	 * The primary method for controlling the drivebase.
	 * Takes a {@link Translation2d} and a rotation rate, and
	 * calculates and commands module states accordingly.
	 * Can use either open-loop or closed-loop velocity control for the wheel velocities.
	 * Also has field- and robot-relative modes, which affect how the translation vector is used.
	 *
	 * @param translation {@link Translation2d} that is the commanded linear 
	 * elocity of the robot, in meters per second. In robot-relative mode, positive
	 * x is torwards the bow (front) and positive y is torwards port (left).
	 * In field-relative mode, positive x is away from the alliance wall
	 * (field North) and positive y is torwards the left wall when
	 * looking through the driver station glass (field West).
	 * @param rotation Robot angular rate, in radians per second.
	 * CCW positive. Unaffected by field/robot relativity.
	 * @param fieldRelative Drive mode.	True for field-relative, false for robot-relative.
	 */
	public void drive(Translation2d translation, double rotation, boolean fieldRelative)
	{
		m_swerveDrive.drive(
			translation,
			rotation,
			fieldRelative,
			false
		); // Open loop is disabled since it shouldn't be used most of the time.
	}

	/**
	 * Drive the robot given a chassis field oriented velocity.
	 *
	 * @param velocity Velocity according to the field.
	 */
	public void driveFieldOriented(ChassisSpeeds velocity)
	{
		m_swerveDrive.driveFieldOriented(velocity);
	}

	/**
	 * Drive the robot given a chassis field oriented velocity.
	 *
	 * @param velocity Velocity according to the field.
	 */
	public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
	{
		return run(() -> {
			ChassisSpeeds v = velocity.get();
			// System.out.println("!!! This should make the robot go\n\t" + v.toString());
			SmartDashboard.putString("DRIVE VELOCITIES", v.toString());

			m_swerveDrive.driveFieldOriented(v);
		});
	}

	/**
	 * Drive according to the chassis robot oriented velocity.
	 *
	 * @param velocity Robot oriented {@link ChassisSpeeds}
	 */
	public void drive(ChassisSpeeds velocity)
	{
		m_swerveDrive.drive(velocity);
	}

	/**
	 * Get the swerve drive kinematics object.
	 *
	 * @return {@link SwerveDriveKinematics} of the swerve drive.
	 */
	public SwerveDriveKinematics getKinematics()
	{
		return m_swerveDrive.kinematics;
	}

	/**
	 * Resets odometry to the given pose. Gyro angle and module positions
	 * do not need to be reset when calling this method. However, if either gyro
	 * angle or module position is reset, this must be called in
	 * order for odometry to keep working.
	 *
	 * @param initialHolonomicPose The pose to set the odometry to
	 */
	public void resetOdometry(Pose2d initialHolonomicPose)
	{
		m_swerveDrive.resetOdometry(initialHolonomicPose);
	}

	public Command resetOdometryCommand() {
		return Commands.runOnce(() -> this.resetOdometry(new Pose2d(3, 3, new Rotation2d())));
	}

	public Command resetOdometryFlippedCommand() {
		return Commands.runOnce(() -> this.resetOdometry(new Pose2d(3, 3, Rotation2d.k180deg)));
	}

	public Command flipOdometry() {
		var nowPose = getPose();
		return Commands.runOnce(() -> this.resetOdometry(
			nowPose.rotateBy(Rotation2d.k180deg)
		));
	}

	/**
	 * Gets the current pose (position and rotation) of the robot, as reported by odometry.
	 *
	 * @return The robot's pose
	 */
	public Pose2d getPose()
	{
		return m_swerveDrive.getPose();
	}

	/**
	 * Set chassis speeds with closed-loop velocity control.
	 *
	 * @param chassisSpeeds Chassis Speeds to set.
	 */
	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
	{
		m_swerveDrive.setChassisSpeeds(chassisSpeeds);
	}

	/**
	 * Post the trajectory to the field.
	 *
	 * @param trajectory The trajectory to post.
	 */
	public void postTrajectory(Trajectory trajectory)
	{
		m_swerveDrive.postTrajectory(trajectory);
	}

	/**
	 * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
	 */
	public void zeroGyro()
	{
		m_swerveDrive.zeroGyro();
	}

	/**
	 * This will zero (calibrate) the robot to assume the current position is facing forward
	 * <p>
	 * If red alliance rotate the robot 180 after the drviebase zero command
	 */
	public void zeroGyroWithAlliance()
	{
		if (Constants.isRedAlliance())
		{
			zeroGyro();
			//Set the pose 180 degrees
			resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.k180deg));
		} else
		{
			zeroGyro();
		}
	}

	/**
	 * Sets the drive motors to brake/coast mode.
	 *
	 * @param brake True to set motors to brake mode, false for coast.
	 */
	public void setMotorBrake(boolean brake)
	{
		m_swerveDrive.setMotorIdleMode(brake);
	}

	/**
	 * Gets the current yaw angle of the robot, as reported by the swerve
	 * pose estimator in the underlying drivebase.
	 * Note, this is not the raw gyro reading,
	 * this may be corrected from calls to resetOdometry().
	 *
	 * @return The yaw angle
	 */
	public Rotation2d getHeading()
	{
		return getPose().getRotation();
	}

	/**
	 * Get the chassis speeds based on controller input of 2 joysticks.
	 * One for speeds in which direction. The other for
	 * the angle of the robot.
	 *
	 * @param xInput X joystick input for the robot to move in the X direction.
	 * @param yInput Y joystick input for the robot to move in the Y direction.
	 * @param headingX X joystick which controls the angle of the robot.
	 * @param headingY Y joystick which controls the angle of the robot.
	 * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
	 */
	public ChassisSpeeds getTargetSpeeds(
		double xInput, double yInput,
		double headingX, double headingY
	) {
		Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
		return m_swerveDrive.swerveController.getTargetSpeeds(
			scaledInputs.getX(),
			scaledInputs.getY(),
			headingX,
			headingY,
			getHeading().getRadians(),
			Constants.Robot.MAX_LINEAR_VELOCITY.in(Units.MetersPerSecond)
		);
	}

	/**
	 * Get the chassis speeds based on controller input of 1 joystick and one angle.
	 * Control the robot at an offset of 90deg.
	 *
	 * @param xInput X joystick input for the robot to move in the X direction.
	 * @param yInput Y joystick input for the robot to move in the Y direction.
	 * @param angle	The angle in as a {@link Rotation2d}.
	 * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
	 */
	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
	{
		Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
		return m_swerveDrive.swerveController.getTargetSpeeds(
			scaledInputs.getX(),
			scaledInputs.getY(),
			angle.getRadians(),
			getHeading().getRadians(),
			Constants.Robot.MAX_LINEAR_VELOCITY.in(Units.MetersPerSecond)
		);
	}

	/**
	 * Gets the current field-relative velocity (x, y and omega) of the robot
	 *
	 * @return A ChassisSpeeds object of the current field-relative velocity
	 */
	public ChassisSpeeds getFieldVelocity()
	{
		return m_swerveDrive.getFieldVelocity();
	}

	/**
	 * Gets the current velocity (x, y and omega) of the robot
	 *
	 * @return A {@link ChassisSpeeds} object of the current velocity
	 */
	public ChassisSpeeds getRobotVelocity()
	{
		return m_swerveDrive.getRobotVelocity();
	}

	/**
	 * Get the {@link SwerveController} in the swerve drive.
	 *
	 * @return {@link SwerveController} from the {@link SwerveDrive}.
	 */
	public SwerveController getSwerveController()
	{
		return m_swerveDrive.swerveController;
	}

	/**
	 * Get the {@link SwerveDriveConfiguration} object.
	 *
	 * @return The {@link SwerveDriveConfiguration} fpr the current drive.
	 */
	public SwerveDriveConfiguration getSwerveDriveConfiguration()
	{
		return m_swerveDrive.swerveDriveConfiguration;
	}

	/**
	 * Lock the swerve drive to prevent it from moving.
	 */
	public void lock()
	{
		m_swerveDrive.lockPose();
	}

	/**
	 * Gets the current pitch angle of the robot, as reported by the imu.
	 *
	 * @return The heading as a {@link Rotation2d} angle
	 */
	public Rotation2d getPitch()
	{
		return m_swerveDrive.getPitch();
	}

	/**
	 * Gets the swerve drive object.
	 *
	 * @return {@link SwerveDrive}
	 */
	public SwerveDrive getSwerveDrive()
	{
		return m_swerveDrive;
	}
}