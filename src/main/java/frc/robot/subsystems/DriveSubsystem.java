// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
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
public class DriveSubsystem extends SubsystemBase
{
	/**
	 * Swerve drive object.
	 */
	private final SwerveDrive _SwerveDrive;

	private final boolean _IsVisionDriveTest =
		Constants.DebugLevel.isOrAll(Constants.DebugLevel.Vision);

	private VisionSubsystem _VisionSubsystem;

	/**
	 * Initialize {@link SwerveDrive} with the directory provided.
	 *
	 * @param directory Directory of swerve drive config files.
	 */
	public DriveSubsystem(File directory)
	{ 
		boolean blueAlliance = false;
		Pose2d startingPose = blueAlliance ?
			new Pose2d(
				new Translation2d(Meter.of(1), Meter.of(4)),
				Rotation2d.fromDegrees(0)
			) : new Pose2d(
				new Translation2d(Meter.of(16), Meter.of(4)),
				Rotation2d.fromDegrees(180)
			);
		// Configure the Telemetry before creating the SwerveDrive to avoid
		// unnecessary objectsbeing created.
		SwerveDriveTelemetry.verbosity = Constants.DebugLevel.isOrAll(Constants.DebugLevel.Drive) ?
			TelemetryVerbosity.HIGH : TelemetryVerbosity.LOW;
		
		try {
			_SwerveDrive = new SwerveParser(directory)
				.createSwerveDrive(
					Constants.Robot.MAX_LINEAR_VELOCITY.in(Units.MetersPerSecond),
					startingPose
			);
			// Alternative method if you don't want to supply the conversion factor via JSON files.
		} catch (Exception e) {
			throw new RuntimeException(e);
		}

		// Heading correction should only be used while controlling the robot via angle.
		_SwerveDrive.setHeadingCorrection(false);
		_SwerveDrive.setCosineCompensator(false);
		//Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
		_SwerveDrive.setAngularVelocityCompensation(true, true, 0.1);
		// Enable if you want to resynchronize your absolute encoders and motor
		// encoders periodically when they are not moving.
		_SwerveDrive.setModuleEncoderAutoSynchronize(false, 1);

		if (_IsVisionDriveTest) {
			setupPhotonVision();
			_SwerveDrive.stopOdometryThread();
		}
	}

	/**
	 * Construct the swerve drive.
	 *
	 * @param driveCfg			SwerveDriveConfiguration for the swerve.
	 * @param controllerCfg Swerve Controller.
	 */
	public DriveSubsystem(
		SwerveDriveConfiguration driveCfg,
		SwerveControllerConfiguration controllerCfg
	) {
		_SwerveDrive = new SwerveDrive(
			driveCfg,
			controllerCfg,
			Constants.Robot.MAX_LINEAR_VELOCITY.in(Units.MetersPerSecond),
			new Pose2d(
				new Translation2d(Meter.of(2), Meter.of(0)),
				Rotation2d.fromDegrees(0)
			)
		);
	}

	public void setupPhotonVision() {
		_VisionSubsystem = new VisionSubsystem(_SwerveDrive::getPose, _SwerveDrive.field);
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
						_SwerveDrive.drive(
							speedsRobotRelative,
							_SwerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
							moduleFeedForwards.linearForces()
						);
					} else {
						_SwerveDrive.setChassisSpeeds(speedsRobotRelative);
					}
				},
				new PPHolonomicDriveController(
					new PIDConstants(5.0, 0.0, 0.0),
					new PIDConstants(5.0, 0.0, 0.0)
				),
				config,
				() -> {
					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
				this
			);
		} catch (Exception e) {
			DriverStation.reportError(getName(), e.getStackTrace());
		}

		PathfindingCommand.warmupCommand().schedule();
	}

	public Command aimAtTarget(Cameras camera) {
		return run(() -> {
			Optional<PhotonPipelineResult> resultO = camera.getBestResult();
			if (resultO.isPresent()) {
				var result = resultO.get();
				if (result.hasTargets()) {
					this.drive(
						this.getTargetSpeeds(
							0, 0,
							Rotation2d.fromDegrees(result.getBestTarget().getYaw())
						)
					);
				}
			}
		});
	}

	public Command getAutonomousCommand(String pathName) {
		return new PathPlannerAuto(pathName);
	}

	public Command driveToPose(Pose2d pose) {
		PathConstraints constraints = new PathConstraints(
			_SwerveDrive.getMaximumChassisVelocity(),
			4.0,
			_SwerveDrive.getMaximumChassisAngularVelocity(),
			Units.Degrees.of(720).in(Units.Radians)
		);
		return AutoBuilder.pathfindToPose(pose, constraints, Units.MetersPerSecond.of(0));
	}

	@Override
	public void periodic() {
		if (_IsVisionDriveTest) {
			_SwerveDrive.updateOdometry();
			_VisionSubsystem.updatePoseEstimation(_SwerveDrive);
		}
	}

	@Override
	public void simulationPeriodic() {}

	public void addVisionMeasurement(Pose2d pose, double timestamp)
	{
		_SwerveDrive.addVisionMeasurement(pose, timestamp);
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
				_SwerveDrive,
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
				_SwerveDrive
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
			() -> Arrays.asList(_SwerveDrive.getModules())
				.forEach(it -> it.setAngle(0.0))
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
			_SwerveDrive.drive(new Translation2d(1, 0), 0, false, false);
		}).finallyDo(() -> _SwerveDrive.drive(new Translation2d(0, 0), 0, false, false));
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
		_SwerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
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
			_SwerveDrive.drive(
				SwerveMath.scaleTranslation(
					new Translation2d(
						translationX.getAsDouble() * _SwerveDrive.getMaximumChassisVelocity(),
						translationY.getAsDouble() * _SwerveDrive.getMaximumChassisVelocity()
					),
					0.8
				),
				Math.pow(angularRotationX.getAsDouble(), 3) *
					_SwerveDrive.getMaximumChassisAngularVelocity(),
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
				_SwerveDrive.swerveController.getTargetSpeeds(
					scaledInputs.getX(), scaledInputs.getY(),
					headingX.getAsDouble(), headingY.getAsDouble(),
					_SwerveDrive.getOdometryHeading().getRadians(),
					_SwerveDrive.getMaximumChassisVelocity()
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
		_SwerveDrive.drive(
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
		_SwerveDrive.driveFieldOriented(velocity);
	}

	/**
	 * Drive the robot given a chassis field oriented velocity.
	 *
	 * @param velocity Velocity according to the field.
	 */
	public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
	{
		return run(() -> {
			_SwerveDrive.driveFieldOriented(velocity.get());
		});
	}

	/**
	 * Drive according to the chassis robot oriented velocity.
	 *
	 * @param velocity Robot oriented {@link ChassisSpeeds}
	 */
	public void drive(ChassisSpeeds velocity)
	{
		_SwerveDrive.drive(velocity);
	}

	/**
	 * Get the swerve drive kinematics object.
	 *
	 * @return {@link SwerveDriveKinematics} of the swerve drive.
	 */
	public SwerveDriveKinematics getKinematics()
	{
		return _SwerveDrive.kinematics;
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
		_SwerveDrive.resetOdometry(initialHolonomicPose);
	}

	/**
	 * Gets the current pose (position and rotation) of the robot, as reported by odometry.
	 *
	 * @return The robot's pose
	 */
	public Pose2d getPose()
	{
		return _SwerveDrive.getPose();
	}

	/**
	 * Set chassis speeds with closed-loop velocity control.
	 *
	 * @param chassisSpeeds Chassis Speeds to set.
	 */
	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
	{
		_SwerveDrive.setChassisSpeeds(chassisSpeeds);
	}

	/**
	 * Post the trajectory to the field.
	 *
	 * @param trajectory The trajectory to post.
	 */
	public void postTrajectory(Trajectory trajectory)
	{
		_SwerveDrive.postTrajectory(trajectory);
	}

	/**
	 * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
	 */
	public void zeroGyro()
	{
		_SwerveDrive.zeroGyro();
	}

	/**
	 * Checks if the alliance is red, defaults to false if alliance isn't available.
	 *
	 * @return true if the red alliance, false if blue. Defaults to false if none is available.
	 */
	private boolean isRedAlliance()
	{
		var alliance = DriverStation.getAlliance();
		return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
	}

	/**
	 * This will zero (calibrate) the robot to assume the current position is facing forward
	 * <p>
	 * If red alliance rotate the robot 180 after the drviebase zero command
	 */
	public void zeroGyroWithAlliance()
	{
		if (isRedAlliance())
		{
			zeroGyro();
			//Set the pose 180 degrees
			resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
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
		_SwerveDrive.setMotorIdleMode(brake);
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
		return _SwerveDrive.swerveController.getTargetSpeeds(
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
		return _SwerveDrive.swerveController.getTargetSpeeds(
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
		return _SwerveDrive.getFieldVelocity();
	}

	/**
	 * Gets the current velocity (x, y and omega) of the robot
	 *
	 * @return A {@link ChassisSpeeds} object of the current velocity
	 */
	public ChassisSpeeds getRobotVelocity()
	{
		return _SwerveDrive.getRobotVelocity();
	}

	/**
	 * Get the {@link SwerveController} in the swerve drive.
	 *
	 * @return {@link SwerveController} from the {@link SwerveDrive}.
	 */
	public SwerveController getSwerveController()
	{
		return _SwerveDrive.swerveController;
	}

	/**
	 * Get the {@link SwerveDriveConfiguration} object.
	 *
	 * @return The {@link SwerveDriveConfiguration} fpr the current drive.
	 */
	public SwerveDriveConfiguration getSwerveDriveConfiguration()
	{
		return _SwerveDrive.swerveDriveConfiguration;
	}

	/**
	 * Lock the swerve drive to prevent it from moving.
	 */
	public void lock()
	{
		_SwerveDrive.lockPose();
	}

	/**
	 * Gets the current pitch angle of the robot, as reported by the imu.
	 *
	 * @return The heading as a {@link Rotation2d} angle
	 */
	public Rotation2d getPitch()
	{
		return _SwerveDrive.getPitch();
	}

	/**
	 * Gets the swerve drive object.
	 *
	 * @return {@link SwerveDrive}
	 */
	public SwerveDrive getSwerveDrive()
	{
		return _SwerveDrive;
	}
}