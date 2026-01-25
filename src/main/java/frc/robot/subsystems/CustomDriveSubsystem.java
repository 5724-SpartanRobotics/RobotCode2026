package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.hardware.Pigeon2;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.lib.DriveSubsystem;
import frc.robot.lib.SwerveModule;

public class CustomDriveSubsystem extends DriveSubsystem {
	public static final boolean kFieldRelativeDriveDefault = true;
	private static CustomDriveSubsystem m_instance;

	public enum ModulePosition { FL, BL, BR, FR }
	
	private final Pigeon2 m_gyroscope;
	/** Start from FL and move counter-clockwise to BL, etc. */
	private final EnumMap<ModulePosition, SwerveModule> m_modules = new EnumMap<>(ModulePosition.class);

	private final SwerveDriveOdometry m_SwerveDriveOdometry;
	private final SwerveDriveKinematics m_SwerveDriveKinematics;
	private final SwerveDrivePoseEstimator m_SwerveDrivePoseEstimator;
	private final PIDController m_xController = new PIDController(10.0, 0.0, 0.0);
	private final PIDController m_yController = new PIDController(10.0, 0.0, 0.0);
	private final PIDController m_hdgController = new PIDController(7.5, 0.0, 0.0);

	private AtomicReference<Rotation2d> lastUpdatedGyroHeading = new AtomicReference<>();
	private AtomicReference<Pose2d> robotPose = new AtomicReference<>(new Pose2d());
	private Field2d field = new Field2d();
	private double speedMod = Constants.Robot.DEFAULT_SPEED_MOD;

	private final SwerveSample[] emptyTrajectory = new SwerveSample[0];
	public SwerveSample[] currentTrajectory = emptyTrajectory;

	private CustomDriveSubsystem() throws Exception {
		m_gyroscope = new Pigeon2(Constants.CanId.PIGEON2);
		resetGyro();

		m_modules.put(ModulePosition.FL, new SwerveModule("FL", Constants.Drive.SWERVE_MOTOR, Constants.CanId.FL));
		m_modules.put(ModulePosition.BL, new SwerveModule("BL", Constants.Drive.SWERVE_MOTOR, Constants.CanId.BL));
		m_modules.put(ModulePosition.BR, new SwerveModule("BR", Constants.Drive.SWERVE_MOTOR, Constants.CanId.BR));
		m_modules.put(ModulePosition.FR, new SwerveModule("FR", Constants.Drive.SWERVE_MOTOR, Constants.CanId.FR));

		m_SwerveDriveKinematics = new SwerveDriveKinematics(
			Constants.Drive.SwerveModuleOffsets.FL,
			Constants.Drive.SwerveModuleOffsets.BL,
			Constants.Drive.SwerveModuleOffsets.BR,
			Constants.Drive.SwerveModuleOffsets.FR
		);

		SwerveModulePosition[] swerveInitialPositions = getModulePositions();

		m_SwerveDriveOdometry = new SwerveDriveOdometry(
			m_SwerveDriveKinematics,
			getGyroHeading(),
			swerveInitialPositions,
			robotPose.get()
		);
		m_SwerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
			m_SwerveDriveKinematics,
			getGyroHeading(),
			swerveInitialPositions,
			robotPose.get()
		);
		m_hdgController.enableContinuousInput(-1.0 * Math.PI, Math.PI);
	}

	public static void initialize() {
		if (m_instance != null) return;
		try {
			m_instance = new CustomDriveSubsystem();
		} catch (Exception e) {
			DriverStation.reportError(e.getMessage(), e.getStackTrace());
		}
	}

	public static CustomDriveSubsystem initialize(boolean getInstance) {
		initialize();
		if (getInstance) return getInstance();
		return null;
	}

	public static CustomDriveSubsystem getInstance() {
		return m_instance;
	}

	@Override
	public void periodic() {
		super.periodic();
		updateGyro();

		Rotation2d currentHdg = getGyroHeading();
		SwerveModulePosition[] positions = getModulePositions();
		Pose2d nowPose = getPose();
		robotPose.set(m_SwerveDriveOdometry.update(currentHdg, positions));
		field.setRobotPose(nowPose);

		if (Constants.DebugLevel.isOrAll(Constants.DebugLevel.Drive)) {
			SmartDashboard.putNumber("Robot Pose/X (Meters)", nowPose.getX());
			SmartDashboard.putNumber("Robot Pose/Y (Meters)", nowPose.getY());
			SmartDashboard.putNumber("Gyro Yaw (Degrees)", getGyroHeading().getDegrees());
			m_modules.forEach((mp, sm) -> {
				sm.report(SwerveModule.Report.All);
			});
		}

		m_SwerveDrivePoseEstimator.update(currentHdg, positions);
		NetworkTableInstance.getDefault().getEntry("/Gyro")
			.setDouble(m_gyroscope.getYaw().getValue().abs(Units.Degrees) % 360.0);
	}

	private SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {
			m_modules.get(ModulePosition.FL).getPosition(),
			m_modules.get(ModulePosition.BL).getPosition(),
			m_modules.get(ModulePosition.BR).getPosition(),
			m_modules.get(ModulePosition.FR).getPosition()
		};
	}

	public Rotation2d getGyroHeading() {
		return lastUpdatedGyroHeading.get();
	}

	public double getGyroRate() {
		return m_gyroscope.getAngularVelocityZWorld().getValueAsDouble();
	}

	public SwerveDrivePoseEstimator getPoseEstimator() {
		return m_SwerveDrivePoseEstimator;
	}

	public CustomDriveSubsystem drive(Translation2d translation, double rotation) {
		SwerveModuleState[] states = m_SwerveDriveKinematics.toSwerveModuleStates(
			ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getGyroHeading())
		);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Robot.MAX_LINEAR_VELOCITY.in(Units.MetersPerSecond));

		m_modules.forEach((mp, sm) -> {
			sm.setDesiredState(states[mp.ordinal()]);
		});

		return this;
	}

	public CustomDriveSubsystem brake() {
		m_modules.forEach((mp, sm) -> {
			sm.setDesiredState(
				new SwerveModuleState(0, sm.getState().angle)
			);
		});
		return this;
	}

	public InstantCommand brakeCmd() {
		return new InstantCommand(() -> {this.brake();});
	}

	private void updateGyro() {
		lastUpdatedGyroHeading.set(
			Rotation2d.fromDegrees(m_gyroscope.getYaw().getValueAsDouble())
		);
	}

	private void resetGyro() {
		m_gyroscope.reset();
		updateGyro();
	}

	public void zeroGyro() {
		m_gyroscope.setYaw(0);
	}

	public void flipGyro() {
		m_gyroscope.setYaw(Math.PI);
	}

	public void driveFieldRelative(ChassisSpeeds speeds) {
		SwerveModuleState[] states = m_SwerveDriveKinematics.toSwerveModuleStates(
			ChassisSpeeds.fromFieldRelativeSpeeds(
				speeds.vxMetersPerSecond,
				speeds.vyMetersPerSecond,
				speeds.omegaRadiansPerSecond,
				getGyroHeading()
			)
		);
	
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Robot.MAX_LINEAR_VELOCITY.in(Units.MetersPerSecond));
	
		m_modules.forEach((mp, sm) -> {
			sm.setDesiredState(states[mp.ordinal()]);
		});
	}

	@Override
	public Command driveForward() {
		return run(() -> drive(new Translation2d(1, 0), 0.0))
			.finallyDo(() -> drive(new Translation2d(0, 0), 0.0));
	}

	public Pose2d getPose() {
		return robotPose.get();
	}

	public void setSpeedMod(double mod) {
		speedMod = mod;
	}

	public void resetSpeedMod() {
		speedMod = Constants.Robot.DEFAULT_SPEED_MOD;
	}

	public Command getTeleopCommand(CommandJoystick joystick) {
		var swervedrive = this;
		return new Command() {
			@Override
			public void initialize() {
				addRequirements(swervedrive);
			}

			@Override
			public void execute() {
				if (DriverStation.isAutonomous()) return;
				final double jsDeadbandXY = SmartDashboard.getNumber("Joystick Deadband/XY", Constants.Controller.DRIVER_DEADBAND_XY);
				final double jsDeadbandZ = SmartDashboard.getNumber("Joystick Deadband/Z", Constants.Controller.DRIVER_DEADBAND_Z);
				double
					xAxis = -1.0 * joystick.getX(),
					yAxis = -1.0 * joystick.getX(),
					zAxis = -1.0 * joystick.getTwist();
				
				xAxis = Math.abs(xAxis) < jsDeadbandXY ? 0 : xAxis * speedMod;
				yAxis = Math.abs(yAxis) < jsDeadbandXY ? 0 : yAxis * speedMod;
				zAxis = Math.abs(zAxis) < jsDeadbandZ ? 0 : zAxis * speedMod * 0.7;

				double rotation = zAxis * Constants.Robot.MAX_ANGULAR_VELOCITY.in(Units.RadiansPerSecond);

				if (Constants.DebugLevel.isOrAll(Constants.DebugLevel.Drive)) {
					SmartDashboard.putNumber("Driver Controller/X (scaled)", xAxis);
					SmartDashboard.putNumber("Driver Controller/Y (scaled)", yAxis);
					SmartDashboard.putNumber("Driver Controller/Rotation (scaled)", rotation);
				}

				Translation2d tx = new Translation2d(yAxis, xAxis)
					.times(Constants.Robot.MAX_LINEAR_VELOCITY.abs(Units.MetersPerSecond));
				swervedrive.drive(tx, rotation);
			}
		};
	}

	public void resetOdometry(Pose2d pose) {
		robotPose.set(pose);
		m_SwerveDriveOdometry.resetPose(pose);
	}

	public void resetOdometry() {
		resetOdometry(new Pose2d(3, 3, new Rotation2d()));
	}

	public void centerModules() {
		m_modules.forEach((mp, sm) -> {
			sm.setDesiredState(new SwerveModuleState(0, Rotation2d.kZero));
		});
	}

	public void followTrajectory(SwerveSample sample) {
		Pose2d pose = robotPose.get();
		ChassisSpeeds speeds = new ChassisSpeeds(
			sample.vx + m_xController.calculate(pose.getX(), sample.x),
			sample.vy + m_yController.calculate(pose.getY(), sample.y),
			sample.omega + m_hdgController.calculate(pose.getRotation().getRadians(), sample.heading)
		);

		driveFieldRelative(speeds);
	}
  
	public void logTrajectory(choreo.trajectory.Trajectory<SwerveSample> traj, boolean isStarting) {
		currentTrajectory = isStarting ? traj.samples().toArray(SwerveSample[]::new) : emptyTrajectory;
	}
}