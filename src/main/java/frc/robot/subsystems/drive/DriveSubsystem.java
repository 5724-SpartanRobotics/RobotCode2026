// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.NopSubsystemBase;
import frc.robot.commands.RotateToAngleCommand;
import frc.robot.info.Alliance;
import frc.robot.info.Debug;
import frc.robot.info.Field;
import frc.robot.info.constants.DriveConstants;
import frc.robot.info.constants.RobotConstants;
import frc.robot.info.constants.VisionConstants.CameraConfigurations;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * {@link https://github.com/Yet-Another-Software-Suite/YAGSL/blob/main/examples/drivebase_with_PhotonVision/src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java}
 */
public class DriveSubsystem extends NopSubsystemBase {
	private final Pose2d kInitialPose;

	private final Pose2d kInitialPoseRed = new Pose2d(
		new Translation2d(12.873565673828125, 1.8854589462280273), Rotation2d.k180deg);
	private final Pose2d kInitialPoseBlue = new Pose2d(
		new Translation2d(3.5681846141815186, 6.073807716369629), Rotation2d.kZero);

	/**
	 * Swerve drive object.
	 */
	private final SwerveDrive m_swerveDrive;
	private final DriveIO io;
	private final DriveIO_YAGSL.DriveIOInputs inputs = new DriveIO_YAGSL.DriveIOInputs();

	private final Lock m_odomLock = new ReentrantLock();

	private static void configureTelemetry() {
		SwerveDriveTelemetry.verbosity = Debug.DebugLevel.isOrAll(Debug.DebugLevel.Drive)
			? TelemetryVerbosity.HIGH
			: TelemetryVerbosity.POSE;
	}

	/**
	 * Initialize {@link SwerveDrive} with the directory provided.
	 *
	 * @param directory
	 *            Directory of swerve drive config files.
	 */
	private DriveSubsystem() {
		// Configure the Telemetry before creating the SwerveDrive to avoid
		// unnecessary objectsbeing created.
		configureTelemetry();

		boolean blueAlliance = Alliance.isBlueAlliance();

		Pose2d startingPose = blueAlliance ? kInitialPoseBlue : kInitialPoseRed;
		kInitialPose = startingPose;

		try {
			m_swerveDrive = new SwerveParser(DriveConstants.SWERVE_CONFIG)
				.createSwerveDrive(
					RobotConstants.MAX_LINEAR_VELOCITY.in(Units.MetersPerSecond),
					startingPose);
		} catch (Exception e) {
			throw new RuntimeException(e);
		}

		// Heading correction should only be used while controlling the robot via angle.
		m_swerveDrive.setHeadingCorrection(false);
		m_swerveDrive.setCosineCompensator(false);

		// Correct for skew that gets worse as angular velocity increases. Start
		// with a coefficient of 0.1.
		m_swerveDrive.setAngularVelocityCompensation(true, true, 0.1);

		// Enable if you want to resynchronize your absolute encoders and motor
		// encoders periodically when they are not moving.
		m_swerveDrive.setModuleEncoderAutoSynchronize(true, 1);

		HAL.report(tResourceType.kResourceType_RobotDrive,
			tInstances.kRobotDriveSwerve_AdvantageKit);

		PathPlanner.configure(this);

		m_swerveDrive.stopOdometryThread();

		io = new DriveIO_YAGSL(m_swerveDrive);
	}

	private static final class Holder {
		private static final DriveSubsystem INSTANCE = new DriveSubsystem();
	}

	public static synchronized DriveSubsystem getInstance() {
		return Holder.INSTANCE;
	}

	public Command driveToTargetCommand() {
		return new InstantCommand(() -> {
			// 1) Read current robot pose and compute the offset pose ONCE
			Pose2d currentPose = getPose(); // ensure odometry is up-to-date
			Translation2d robotTranslation = currentPose.getTranslation();
			Translation2d hub = Alliance.isRedAlliance()
				? Field.RED_HUB_CENTER
				: Field.BLUE_HUB_CENTER;

			double diffX = hub.getX() - robotTranslation.getX();
			double diffY = hub.getY() - robotTranslation.getY();
			double dist = Math.hypot(diffX, diffY);
			double ux = diffX / dist;
			double uy = diffY / dist;

			double x_new = hub.getX() - 2.0 * ux;
			double y_new = hub.getY() - 2.0 * uy;
			Rotation2d heading = new Rotation2d(Math.atan2(diffY, diffX)).plus(Rotation2d.k180deg);

			Pose2d staticTarget = new Pose2d(new Translation2d(x_new, y_new), heading);

			Command pathCmd = this.driveToPose(staticTarget, 0.8);
			Command rotCmd = new RotateToAngleCommand(() -> staticTarget.getRotation());
			CommandScheduler.getInstance().schedule(
				pathCmd.andThen(
					rotCmd.withName("RotateToTarget")).withName("DriveToTarget"));
		}, this).withName("DriveToTargetWrapper");
	}

	public Distance getHypotToAllianceHub() {
		Pose2d currentPose = getPose(); // ensure odometry is up-to-date
		Translation2d robotTranslation = currentPose.getTranslation();
		Translation2d hub = Alliance.isRedAlliance()
			? Field.RED_HUB_CENTER
			: Field.BLUE_HUB_CENTER;
		double diffX = hub.getX() - robotTranslation.getX();
		double diffY = hub.getY() - robotTranslation.getY();
		double dist = Math.hypot(diffX, diffY);
		return Units.Meters.of(dist);
	}

	public Command driveToInitialPosition(double maxSpeedMultiplier) {
		// THIS METHOD WORKS SO WELL
		// but only when the robot is about >1 meter from the inital pose
		return this.driveToPose(kInitialPose, maxSpeedMultiplier)
			.andThen(new RotateToAngleCommand(() -> kInitialPose.getRotation())
				.withName("RotateToInitialPosition"))
			.withName("DriveToInitialPosition");
	}

	public Command driveToPose(Pose2d pose) {
		PathConstraints constraints = new PathConstraints(
			m_swerveDrive.getMaximumChassisVelocity(),
			RobotConstants.MAX_LINEAR_ACCELERATION.in(Units.MetersPerSecondPerSecond),
			m_swerveDrive.getMaximumChassisAngularVelocity(),
			Units.Degrees.of(720).in(Units.Radians));
		return AutoBuilder.pathfindToPose(pose, constraints, Units.MetersPerSecond.of(0));
	}

	public Command driveToPose(Pose2d pose, double allSpeedsMultiplier) {
		PathConstraints c = new PathConstraints(
			m_swerveDrive.getMaximumChassisVelocity() * allSpeedsMultiplier,
			RobotConstants.MAX_LINEAR_ACCELERATION.times(allSpeedsMultiplier)
				.in(Units.MetersPerSecondPerSecond),
			m_swerveDrive.getMaximumChassisAngularVelocity() * allSpeedsMultiplier,
			Units.Degrees.of(720).in(Units.Radians));
		return AutoBuilder.pathfindToPose(pose, c, Units.MetersPerSecond.of(0));
	}

	public Command driveToPose(Supplier<Pose2d> poseSupplier, double allSpeedsMultiplier) {
		return driveToPose(poseSupplier.get(), allSpeedsMultiplier);
	}

	public void stop() {
		this.drive(new ChassisSpeeds());
	}

	@Override
	public void periodic() {
		m_odomLock.lock();
		try {
			m_swerveDrive.updateOdometry();
			io.updateInputs(inputs);
			Logger.processInputs("Drive", inputs);
		} finally {
			m_odomLock.unlock();
		}

		// Log empty setpoints when disabled to avoid dashboard confusion
		if (DriverStation.isDisabled()) {
			Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[]{});
			Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[]{});
		}

		updateCameraPositions();

		if (Debug.DebugLevel.isOrAll(Debug.DebugLevel.Drive))
			SmartDashboard.putData(this);
	}

	private void updateCameraPositions() {
		Pose3d robotPose3d = new Pose3d(getPose());

		// Transform all camera poses to global coordinates
		CameraConfigurations[] cams = CameraConfigurations.values();
		int cameraCount = cams.length;
		Pose3d[] globalCameraPositions = new Pose3d[cameraCount];
		for (int i = 0; i < cameraCount; i++) {
			globalCameraPositions[i] = robotPose3d.transformBy(cams[i].getTransform3d());
		}

		// Log for visualization
		Logger.recordOutput("CameraPositions", globalCameraPositions);

		getModuleStates();
	}

	@Override
	public void simulationPeriodic() {
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType(this.getClass().getName());
		for (var m : m_swerveDrive.getModules()) {
			builder.addDoubleProperty(
				"Module " + m.moduleNumber + " Turn Encoder Degrees",
				() -> m.getAngleMotor().getPosition(),
				null);
		}
	}

	public void addVisionMeasurement(Pose2d pose, double timestamp) {
		m_odomLock.lock();
		try {
			m_swerveDrive.field.getObject("VisionEstimation").setPose(pose);
			m_swerveDrive.addVisionMeasurement(pose, timestamp);
		} finally {
			m_odomLock.unlock();
		}
	}

	public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
		m_odomLock.lock();
		try {
			m_swerveDrive.field.getObject("VisionEstimation").setPose(pose);
			m_swerveDrive.addVisionMeasurement(pose, timestamp, stdDevs);
		} finally {
			m_odomLock.unlock();
		}
	}

	@AutoLogOutput(key = "SwerveStates/Measured")
	public SwerveModuleState[] getModuleStates() {
		m_odomLock.lock();
		try {
			SwerveModule[] mods = m_swerveDrive.getModules();
			SwerveModuleState[] swerveModuleStates = new SwerveModuleState[mods.length];
			for (int i = 0; i < mods.length; i++) {
				swerveModuleStates[i] = mods[i].getState();
			}
			return swerveModuleStates;
		} finally {
			m_odomLock.unlock();
		}
	}

	@AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
	public ChassisSpeeds getChassisSpeeds() {
		return m_swerveDrive.kinematics.toChassisSpeeds(getModuleStates());
	}

	/**
	 * Returns a Command that centers the modules of the SwerveDrive subsystem.
	 *
	 * @return a Command that centers the modules of the SwerveDrive subsystem
	 */
	public Command centerModulesCommand() {
		return run(
			() -> Arrays.asList(m_swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
	}

	/**
	 * Returns a Command that tells the robot to drive forward until the command ends.
	 *
	 * @return a Command that tells the robot to drive forward until the command ends
	 */
	public Command driveForward() {
		return run(() -> {
			m_swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
		}).finallyDo(() -> m_swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
	}

	/**
	 * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a
	 * rotation rate, and calculates and commands module states accordingly. Can use either
	 * open-loop or closed-loop velocity control for the wheel velocities. Also has field- and
	 * robot-relative modes, which affect how the translation vector is used.
	 *
	 * @param translation
	 *            {@link Translation2d} that is the commanded linear elocity of the robot, in meters
	 *            per second. In robot-relative mode, positive x is torwards the bow (front) and
	 *            positive y is torwards port (left). In field-relative mode, positive x is away
	 *            from the alliance wall (field North) and positive y is torwards the left wall when
	 *            looking through the driver station glass (field West).
	 * @param rotation
	 *            Robot angular rate, in radians per second. CCW positive. Unaffected by field/robot
	 *            relativity.
	 * @param fieldRelative
	 *            Drive mode. True for field-relative, false for robot-relative.
	 */
	public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
		m_swerveDrive.drive(
			translation,
			rotation,
			fieldRelative,
			false); // Open loop is disabled since it shouldn't be used most of the time.
	}

	/**
	 * Drive the robot given a chassis field oriented velocity.
	 *
	 * @param velocity
	 *            Velocity according to the field.
	 */
	public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
		if (velocity == null) {
			return Commands.none();
		}
		return run(() -> {
			ChassisSpeeds v = velocity.get();
			SmartDashboard.putString("Drive Velovities", v.toString());

			m_swerveDrive.driveFieldOriented(v);
		});
	}

	/**
	 * Drive according to the chassis robot oriented velocity.
	 *
	 * @param velocity
	 *            Robot oriented {@link ChassisSpeeds}
	 */
	public void drive(ChassisSpeeds velocity) {
		m_swerveDrive.drive(velocity);
	}

	/**
	 * Get the swerve drive kinematics object.
	 *
	 * @return {@link SwerveDriveKinematics} of the swerve drive.
	 */
	public SwerveDriveKinematics getKinematics() {
		return m_swerveDrive.kinematics;
	}

	/**
	 * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset
	 * when calling this method. However, if either gyro angle or module position is reset, this
	 * must be called in order for odometry to keep working.
	 *
	 * @param initialHolonomicPose
	 *            The pose to set the odometry to
	 */
	public void resetOdometry(Pose2d initialHolonomicPose) {
		m_odomLock.lock();
		try {
			m_swerveDrive.resetOdometry(initialHolonomicPose);
		} finally {
			m_odomLock.unlock();
		}
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
			nowPose.rotateBy(Rotation2d.k180deg)));
	}

	/**
	 * Gets the current pose (position and rotation) of the robot, as reported by odometry.
	 *
	 * @return The robot's pose
	 */
	@AutoLogOutput(key = "EstimatedPose")
	private Pose2d getPose(boolean x) {
		m_odomLock.lock();
		try {
			return m_swerveDrive.getPose();
		} finally {
			m_odomLock.unlock();
		}
	}

	@AutoLogOutput(key = "Odometry/Robot")
	public Pose2d getPose() {
		return getPose(true);
	}

	/**
	 * Set chassis speeds with closed-loop velocity control.
	 *
	 * @param chassisSpeeds
	 *            Chassis Speeds to set.
	 */
	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		m_swerveDrive.setChassisSpeeds(chassisSpeeds);
	}

	/**
	 * Post the trajectory to the field.
	 *
	 * @param trajectory
	 *            The trajectory to post.
	 */
	public void postTrajectory(Trajectory trajectory) {
		m_swerveDrive.postTrajectory(trajectory);
	}

	/**
	 * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
	 */
	public void zeroGyro() {
		m_swerveDrive.zeroGyro();
	}

	/**
	 * This will zero (calibrate) the robot to assume the current position is facing forward
	 * <p>
	 * If red alliance rotate the robot 180 after the drviebase zero command
	 */
	public void zeroGyroWithAlliance() {
		if (Alliance.isRedAlliance()) {
			zeroGyro();
			// Set the pose 180 degrees
			resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.k180deg));
		} else {
			zeroGyro();
		}
	}

	/**
	 * Sets the drive motors to brake/coast mode.
	 *
	 * @param brake
	 *            True to set motors to brake mode, false for coast.
	 */
	public void setMotorBrake(boolean brake) {
		m_swerveDrive.setMotorIdleMode(brake);
	}

	/**
	 * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the
	 * underlying drivebase. Note, this is not the raw gyro reading, this may be corrected from
	 * calls to resetOdometry().
	 *
	 * @return The yaw angle
	 */
	public Rotation2d getHeading() {
		return getPose().getRotation();
	}

	/**
	 * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which
	 * direction. The other for the angle of the robot.
	 *
	 * @param xInput
	 *            X joystick input for the robot to move in the X direction.
	 * @param yInput
	 *            Y joystick input for the robot to move in the Y direction.
	 * @param headingX
	 *            X joystick which controls the angle of the robot.
	 * @param headingY
	 *            Y joystick which controls the angle of the robot.
	 * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
	 */
	public ChassisSpeeds getTargetSpeeds(
		double xInput, double yInput,
		double headingX, double headingY) {
		Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
		return m_swerveDrive.swerveController.getTargetSpeeds(
			scaledInputs.getX(),
			scaledInputs.getY(),
			headingX,
			headingY,
			getHeading().getRadians(),
			RobotConstants.MAX_LINEAR_VELOCITY.in(Units.MetersPerSecond));
	}

	/**
	 * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the
	 * robot at an offset of 90deg.
	 *
	 * @param xInput
	 *            X joystick input for the robot to move in the X direction.
	 * @param yInput
	 *            Y joystick input for the robot to move in the Y direction.
	 * @param angle
	 *            The angle in as a {@link Rotation2d}.
	 * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
	 */
	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
		Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
		return m_swerveDrive.swerveController.getTargetSpeeds(
			scaledInputs.getX(),
			scaledInputs.getY(),
			angle.getRadians(),
			getHeading().getRadians(),
			RobotConstants.MAX_LINEAR_VELOCITY.in(Units.MetersPerSecond));
	}

	/**
	 * Gets the current field-relative velocity (x, y and omega) of the robot
	 *
	 * @return A ChassisSpeeds object of the current field-relative velocity
	 */
	public ChassisSpeeds getFieldVelocity() {
		return m_swerveDrive.getFieldVelocity();
	}

	/**
	 * Gets the current velocity (x, y and omega) of the robot
	 *
	 * @return A {@link ChassisSpeeds} object of the current velocity
	 */
	public ChassisSpeeds getRobotVelocity() {
		return m_swerveDrive.getRobotVelocity();
	}

	/**
	 * Get the {@link SwerveController} in the swerve drive.
	 *
	 * @return {@link SwerveController} from the {@link SwerveDrive}.
	 */
	public SwerveController getSwerveController() {
		return m_swerveDrive.swerveController;
	}

	/**
	 * Get the {@link SwerveDriveConfiguration} object.
	 *
	 * @return The {@link SwerveDriveConfiguration} fpr the current drive.
	 */
	public SwerveDriveConfiguration getSwerveDriveConfiguration() {
		return m_swerveDrive.swerveDriveConfiguration;
	}

	/**
	 * Lock the swerve drive to prevent it from moving.
	 */
	public void lock() {
		m_swerveDrive.lockPose();
	}

	/**
	 * Gets the current pitch angle of the robot, as reported by the imu.
	 *
	 * @return The heading as a {@link Rotation2d} angle
	 */
	public Rotation2d getPitch() {
		return m_swerveDrive.getPitch();
	}

	/**
	 * Gets the swerve drive object.
	 *
	 * @return {@link SwerveDrive}
	 */
	public SwerveDrive getSwerveDrive() {
		return m_swerveDrive;
	}
}
