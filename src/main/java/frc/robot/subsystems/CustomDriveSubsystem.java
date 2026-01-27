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
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.Robot;
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
			Constants.Drive.SwerveModuleOffsets.FL.translation,
			Constants.Drive.SwerveModuleOffsets.BL.translation,
			Constants.Drive.SwerveModuleOffsets.BR.translation,
			Constants.Drive.SwerveModuleOffsets.FR.translation
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

		SmartDashboard.putData("Field", field);

		Telemetry.maxSpeed = Constants.Robot.MAX_LINEAR_VELOCITY.in(Units.MetersPerSecond);
		Telemetry.maxAngularVelocity = Constants.Robot.MAX_ANGULAR_VELOCITY.in(Units.DegreesPerSecond);
		Telemetry.moduleCount = m_modules.size();
		Telemetry.sizeFrontBack = Constants.Drive.Wheel.X_FROM_CENTER.times(2.0).in(Units.Inches);
		Telemetry.sizeLeftRight = Constants.Drive.Wheel.Y_FROM_CENTER.times(2.0).in(Units.Inches);
		Telemetry.wheelLocations = new double[Telemetry.moduleCount * 2];
		m_modules.forEach((mp, module) -> {
			final int ord = mp.ordinal();
			final int idx = ord * 2;
			final var off = Constants.Drive.SwerveModuleOffsets.values()[ord];
			Telemetry.wheelLocations[idx] = Units.Meters.of(off.translation.getX()).in(Units.Inches);
			Telemetry.wheelLocations[idx+1] = Units.Meters.of(off.translation.getY()).in(Units.Inches);
		});
		Telemetry.measuredStates = new double[Telemetry.moduleCount * 2];
		Telemetry.desiredStates = new double[Telemetry.moduleCount * 2];
		Telemetry.desiredStatesObj = new SwerveModuleState[Telemetry.moduleCount];
		Telemetry.measuredStatesObj = new SwerveModuleState[Telemetry.moduleCount];
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
		
		updateOdometry(currentHdg, positions, nowPose);
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

	private void setDesiredStateTelemetryCallback(int i, SwerveModuleState s) {
		Telemetry.desiredStatesObj[i] = s;
		Telemetry.endCtrlCycle();
	}

	public CustomDriveSubsystem drive(Translation2d translation, double rotation) {
		Telemetry.startCtrlCycle();
		ChassisSpeeds desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
			translation.getX(), translation.getY(),
			rotation, getGyroHeading()
		);
		SwerveModuleState[] states = m_SwerveDriveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Robot.MAX_LINEAR_VELOCITY.in(Units.MetersPerSecond));

		Telemetry.desiredChassisSpeedsObj = desiredChassisSpeeds;
		Telemetry.desiredStatesObj = states;

		m_modules.forEach((mp, sm) -> {
			sm.setDesiredState(states[mp.ordinal()], this::setDesiredStateTelemetryCallback);
		});

		return this;
	}

	public CustomDriveSubsystem brake() {
		Telemetry.startCtrlCycle();
		m_modules.forEach((mp, sm) -> {
			sm.setDesiredState(
				new SwerveModuleState(0, sm.getState().angle),
				this::setDesiredStateTelemetryCallback
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

	private void updateOdometry(
		Rotation2d hdg,
		SwerveModulePosition[] swervePositions,
		Pose2d pose
	) {
		Telemetry.startOdomCycle();
		robotPose.set(m_SwerveDriveOdometry.update(hdg, swervePositions));
		Telemetry.measuredChassisSpeedsObj = new ChassisSpeeds();
		Telemetry.robotRotationObj = getGyroHeading();
		m_modules.forEach((mp, sm) -> {
			final int ord = mp.ordinal();

			sm.updateTelemetry();
			Telemetry.measuredStatesObj[ord] = sm.getState();
		});
		Telemetry.updateData();
		Telemetry.endOdomCycle();
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
		Telemetry.startCtrlCycle();
		ChassisSpeeds desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
			speeds.vxMetersPerSecond,
			speeds.vyMetersPerSecond,
			speeds.omegaRadiansPerSecond,
			getGyroHeading()
		);
		SwerveModuleState[] states = m_SwerveDriveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Robot.MAX_LINEAR_VELOCITY.in(Units.MetersPerSecond));

		Telemetry.desiredChassisSpeedsObj = desiredChassisSpeeds;
		Telemetry.desiredStatesObj = states;
	
		m_modules.forEach((mp, sm) -> {
			sm.setDesiredState(states[mp.ordinal()], this::setDesiredStateTelemetryCallback);
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
		Telemetry.startCtrlCycle();
		m_modules.forEach((mp, sm) -> {
			sm.setDesiredState(
				new SwerveModuleState(0, Rotation2d.kZero),
				this::setDesiredStateTelemetryCallback
			);
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

	public class Telemetry {
		private static final NetworkTable SMARTDASHBOARD =
			NetworkTableInstance.getDefault().getTable("SmartDashboard");
		private static final DoublePublisher moduleCountPublisher =
			SMARTDASHBOARD.getDoubleTopic("swerve/moduleCount").publish();
		private static final DoubleArrayPublisher measuredStatesArrayPublisher =
			SMARTDASHBOARD.getDoubleArrayTopic("swerve/measuredStates").publish();
		private static final DoubleArrayPublisher desiredStatesArrayPublisher =
			SMARTDASHBOARD.getDoubleArrayTopic("swerve/desiredStates").publish();
		private static final DoubleArrayPublisher measuredChassisSpeedsArrayPublisher =
			SMARTDASHBOARD.getDoubleArrayTopic("swerve/measuredChassisSpeeds").publish();
		private static final DoubleArrayPublisher desiredChassisSpeedsArrayPublisher =
			SMARTDASHBOARD.getDoubleArrayTopic("swerve/desiredChassisSpeeds").publish();
		private static final DoublePublisher robotRotationPublisher =
			SMARTDASHBOARD.getDoubleTopic("swerve/robotRotation").publish();
		private static final DoublePublisher maxAngularVelocityPublisher =
			SMARTDASHBOARD.getDoubleTopic("swerve/maxAngularVelocity").publish();
		private static final StructArrayPublisher<SwerveModuleState> measuredStatesStruct =
			SMARTDASHBOARD.getStructArrayTopic("swerve/advantagescope/currentStates", SwerveModuleState.struct).publish();
		private static final StructArrayPublisher<SwerveModuleState> desiredStatesStruct =
			SMARTDASHBOARD.getStructArrayTopic("swerve/advantagescope/desiredStates", SwerveModuleState.struct).publish();
		private static final StructPublisher<ChassisSpeeds> measuredChassisSpeedsStruct =
			SMARTDASHBOARD.getStructTopic("swerve/advantagescope/measuredChassisSpeeds", ChassisSpeeds.struct).publish();
		private static final StructPublisher<ChassisSpeeds> desiredChassisSpeedsStruct =
			SMARTDASHBOARD.getStructTopic("swerve/advantagescope/desiredChassisSpeeds", ChassisSpeeds.struct).publish();
		private static final StructPublisher<Rotation2d> robotRotationStruct =
			SMARTDASHBOARD.getStructTopic("swerve/advantagescope/robotRotation", Rotation2d.struct).publish();
		private static final DoubleArrayPublisher wheelLocationsArrayPublisher =
			SMARTDASHBOARD.getDoubleArrayTopic("swerve/wheelLocation").publish();
		private static final DoublePublisher maxSpeedPublisher =
			SMARTDASHBOARD.getDoubleTopic("swerve/maxSpeed").publish();
		private static final StringPublisher rotationUnitPublisher =
			SMARTDASHBOARD.getStringTopic("swerve/rotationUnit").publish();
		private static final DoublePublisher sizeLeftRightPublisher =
			SMARTDASHBOARD.getDoubleTopic("swerve/sizeLeftRight").publish();
		private static final DoublePublisher sizeFrontBackPublisher =
			SMARTDASHBOARD.getDoubleTopic("swerve/sizeFrontBack").publish();
		private static final StringPublisher forwardDirectionPublisher =
			SMARTDASHBOARD.getStringTopic("swerve/forwardDirection").publish();
		private static final DoublePublisher odomCycleTime =
			SMARTDASHBOARD.getDoubleTopic("swerve/odomCycleMS").publish();
		private static final DoublePublisher ctrlCycleTime =
			SMARTDASHBOARD.getDoubleTopic("swerve/controlCycleMS").publish();
		
		private static final Timer odomTimer = new Timer();
		private static final Timer ctrlTimer = new Timer();

		public static SwerveModuleState[] measuredStatesObj = new SwerveModuleState[4];
		public static SwerveModuleState[] desiredStatesObj = new SwerveModuleState[4];
		public static ChassisSpeeds measuredChassisSpeedsObj = new ChassisSpeeds();
		public static ChassisSpeeds desiredChassisSpeedsObj = new ChassisSpeeds();
		public static Rotation2d robotRotationObj = new Rotation2d();
		public static boolean isSimulation = Robot.isSimulation();
		public static int moduleCount;
		public static double[] wheelLocations;
		public static double[] measuredStates;
		public static double[] desiredStates;
		public static double robotRotation;
		public static double maxSpeed;
		public static String rotationUnit = "degrees";
		public static double sizeLeftRight;
		public static double sizeFrontBack;
		public static String forwardDirection = "up";
		public static double maxAngularVelocity;
		public static double[] measuredChassisSpeeds = new double[3];
		public static double[] desiredChassisSpeeds = new double[3];
		public static boolean updateSettings = true;

		public static void startCtrlCycle() {
			if (ctrlTimer.isRunning()) ctrlTimer.reset();
			else ctrlTimer.start();
		}

		public static void endCtrlCycle() {
			if (DriverStation.isTeleopEnabled() || DriverStation.isAutonomousEnabled() || DriverStation.isTestEnabled())
				ctrlCycleTime.set(ctrlTimer.get() * 1000);
			ctrlTimer.reset();
		}

		public static void startOdomCycle() {
			if (odomTimer.isRunning()) odomTimer.reset();
			else odomTimer.reset();
		}

		public static void endOdomCycle() {
			if (DriverStation.isTeleopEnabled() || DriverStation.isAutonomousEnabled() || DriverStation.isTestEnabled())
				odomCycleTime.set(odomTimer.get() * 1000);
			odomTimer.reset();
		}

		public static void updateSwerveTelemetrySettings() {
			if (updateSettings) {
				updateSettings = false;
				wheelLocationsArrayPublisher.set(wheelLocations);
				maxSpeedPublisher.set(maxSpeed);
				rotationUnitPublisher.set(rotationUnit);
				sizeLeftRightPublisher.set(sizeLeftRight);
				sizeFrontBackPublisher.set(sizeFrontBack);
				forwardDirectionPublisher.set(forwardDirection);
			}
		}

		public static void updateData() {
			if (updateSettings) updateSwerveTelemetrySettings();

			measuredChassisSpeeds[0] = measuredChassisSpeedsObj.vxMetersPerSecond;
			measuredChassisSpeeds[1] = measuredChassisSpeedsObj.vyMetersPerSecond;
			measuredChassisSpeeds[2] = Math.toDegrees(measuredChassisSpeedsObj.omegaRadiansPerSecond);

			desiredChassisSpeeds[0] = desiredChassisSpeedsObj.vxMetersPerSecond;
			desiredChassisSpeeds[1] = desiredChassisSpeedsObj.vyMetersPerSecond;
			desiredChassisSpeeds[2] = Math.toDegrees(desiredChassisSpeedsObj.omegaRadiansPerSecond);

			robotRotation = robotRotationObj.getDegrees();

			for (int i = 0; i < measuredStatesObj.length; i++) {
				SwerveModuleState state = measuredStatesObj[i];
				if (state != null) {
					measuredStates[i * 2] = state.angle.getDegrees();
					measuredStates[i * 2 + 1] = state.speedMetersPerSecond;
				}
			}

			for (int i = 0; i < desiredStatesObj.length; i++) {
				SwerveModuleState state = desiredStatesObj[i];
				if (state != null) {
					desiredStates[i * 2] = state.angle.getDegrees();
					desiredStates[i * 2 + 1] = state.speedMetersPerSecond;
				}
			}

			moduleCountPublisher.set(moduleCount);
			measuredStatesArrayPublisher.set(measuredStates);
			desiredStatesArrayPublisher.set(desiredStates);
			robotRotationPublisher.set(robotRotation);
			maxAngularVelocityPublisher.set(maxAngularVelocity);

			measuredChassisSpeedsArrayPublisher.set(measuredChassisSpeeds);
			desiredChassisSpeedsArrayPublisher.set(desiredChassisSpeeds);

			desiredStatesStruct.set(desiredStatesObj);
			measuredStatesStruct.set(measuredStatesObj);
			desiredChassisSpeedsStruct.set(desiredChassisSpeedsObj);
			measuredChassisSpeedsStruct.set(measuredChassisSpeedsObj);
			robotRotationStruct.set(robotRotationObj);
		}
	}
}