package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;

public final class DriveCommand {
	private static DriveSubsystem m_driveSubsystem;
	private static CommandJoystick m_joystick;

	public static double speedMod = Constants.Robot.DEFAULT_SPEED_MOD;

	public static void initialize(DriveSubsystem drive, CommandJoystick joystick) {
		m_driveSubsystem = drive;
		m_joystick = joystick;
		SmartDashboard.putBoolean("DriveCommands Initialized", true);
	}

	private static double applyJoystickDeadband(double realValue, double deadband) {
		if (Math.abs(realValue) < deadband)
			return 0.0;
		return realValue;
	}

	private static double applyJoystickScale(double realValue, double scale) {
		return realValue * scale;
	}

	private static double applyJoystickDeadbandAndScale(double realValue, double deadband) {
		return applyJoystickScale(
			applyJoystickDeadband(realValue, deadband),
			speedMod);
	}

	public static Command setSpeedModCommand(double s) {
		return Commands.run(() -> speedMod = s, m_driveSubsystem);
	}

	public static Command resetSpeedModCommand() {
		return Commands.run(() -> speedMod = Constants.Robot.DEFAULT_SPEED_MOD, m_driveSubsystem);
	}

	public static SwerveInputStream DriveAngularVelocity() {
		return SwerveInputStream.of(
			m_driveSubsystem.getSwerveDrive(),
			() -> applyJoystickDeadbandAndScale(
				m_joystick.getRawAxis(1) *
					(Constants.isRedAlliance() ? -1.0 : 1.0),
				Constants.Controller.DRIVER_DEADBAND_XY), // Y axis (forward/back)
			() -> applyJoystickDeadbandAndScale(
				m_joystick.getRawAxis(0) *
					(Constants.isRedAlliance() ? -1.0 : 1.0),
				Constants.Controller.DRIVER_DEADBAND_XY) // X axis (strafe)
		)
			.withControllerRotationAxis(() -> applyJoystickDeadbandAndScale(
				// Gonna invert this based on alliance, idk if that's right (we'll see later)
				// TODO: Test this on the actual robot
				m_joystick.getRawAxis(2) *
					(Constants.isRedAlliance() ? -1.0 : 1.0),
				Constants.Controller.DRIVER_DEADBAND_Z)) // twist / rotation
			.deadband(Constants.Controller.DRIVER_DEADBAND_XY)
			.scaleTranslation(0.8)
			.allianceRelativeControl(true);
	}

	public static SwerveInputStream DriveRobotOriented() {
		// System.out.println("RETURNING A DRIVE COMMAND: DRO");
		return DriveAngularVelocity().copy()
			.robotRelative(false)
			.allianceRelativeControl(false);
	}

	public static SwerveInputStream DriveAngularVelocity_Keyboard() {
		// System.out.println("RETURNING A DRIVE COMMAND: DAVK");
		return SwerveInputStream.of(
			m_driveSubsystem.getSwerveDrive(),
			() -> m_joystick.getRawAxis(1) * -1.0,
			() -> m_joystick.getRawAxis(0) * -1.0)
			.withControllerRotationAxis(() -> m_joystick.getRawAxis(2))
			.deadband(Constants.Controller.DRIVER_DEADBAND_XY)
			.scaleTranslation(0.8)
			.allianceRelativeControl(true);
	}

	public static SwerveInputStream DriveDirectAngle_Keyboard() {
		// System.out.println("RETURNING A DRIVE COMMAND: DDAK");
		return DriveAngularVelocity_Keyboard().copy()
			.withControllerHeadingAxis(
				() -> Math.sin(m_joystick.getRawAxis(2) * Math.PI) * Constants.TWO_PI,
				() -> Math.cos(m_joystick.getRawAxis(2) * Math.PI) * Constants.TWO_PI)
			.headingWhile(true)
			.translationHeadingOffset(true)
			.translationHeadingOffset(Rotation2d.fromDegrees(0));
	}

	public enum DriveType {
		FO_AngularVelocity, FO_DirectAngle, RO_AngularVelocity, SetpointGenerator
	}

	public static Command getCommand(DriveType t, boolean isKeyboard) {
		if (t == DriveType.RO_AngularVelocity && isKeyboard) {
			return Commands.none();
		}
		return switch (t) {
			case FO_AngularVelocity -> m_driveSubsystem.driveFieldOriented(
				!isKeyboard ? DriveAngularVelocity() : DriveAngularVelocity_Keyboard());
			case FO_DirectAngle -> m_driveSubsystem.driveFieldOriented(
				!isKeyboard ? null : DriveDirectAngle_Keyboard());
			case RO_AngularVelocity -> m_driveSubsystem.driveFieldOriented(DriveRobotOriented());
			case SetpointGenerator -> m_driveSubsystem.driveWithSetpointGeneratorFieldRelative(
				!isKeyboard ? null : DriveDirectAngle_Keyboard());
		};
	}

	public class AbsoluteDrive extends Command {
		private final DriveSubsystem m_driveSubsystem;
		private final DoubleSupplier _VX, _VY, _HdgHorizontal, _HdgVertical;
		private boolean _initRotation = false;
		private Translation2d tx = null;

		public AbsoluteDrive(
			DriveSubsystem DriveSubsystem,
			DoubleSupplier vX,
			DoubleSupplier vY,
			DoubleSupplier hdgHorizontal,
			DoubleSupplier hdgVertical) {
			this.m_driveSubsystem = DriveSubsystem;
			this._VX = vX;
			this._VY = vY;
			this._HdgHorizontal = hdgHorizontal;
			this._HdgVertical = hdgVertical;

			addRequirements(this.m_driveSubsystem);
		}

		@Override
		public void initSendable(SendableBuilder builder) {
			super.initSendable(builder);
			builder.addDoubleProperty("LimitedTranslation", tx::getX, null);
			builder.addStringProperty("Translation", tx::toString, null);
		}

		@Override
		public void initialize() {
			_initRotation = true;
		}

		@Override
		public void execute() {
			ChassisSpeeds desiredSpeeds = m_driveSubsystem.getTargetSpeeds(
				_VX.getAsDouble(), _VY.getAsDouble(),
				_HdgHorizontal.getAsDouble(), _HdgVertical.getAsDouble());

			if (_initRotation) {
				if (_HdgHorizontal.getAsDouble() == 0 && _HdgVertical.getAsDouble() == 0) {
					Rotation2d firstLoopHdg = m_driveSubsystem.getHeading();
					desiredSpeeds = m_driveSubsystem.getTargetSpeeds(
						0, 0,
						firstLoopHdg.getSin(), firstLoopHdg.getCos());
				}
				_initRotation = false;
			}

			tx = SwerveController.getTranslation2d(desiredSpeeds);
			tx = SwerveMath.limitVelocity(
				tx,
				m_driveSubsystem.getFieldVelocity(),
				m_driveSubsystem.getPose(),
				Constants.Drive.SWERVE_LOOP_TIME.in(Units.Seconds),
				Constants.Robot.MASS.in(Units.Kilograms),
				List.of(Constants.Drive.CHASSIS),
				m_driveSubsystem.getSwerveDriveConfiguration());

			if (Constants.DebugLevel.isOrAll(Constants.DebugLevel.Drive))
				SmartDashboard.putData(this);

			m_driveSubsystem.drive(tx, desiredSpeeds.omegaRadiansPerSecond, true);
		}

		@Override
		public void end(boolean interrupted) {
		}

		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public class AbsoluteDriveAdvanced extends Command {
		private final DriveSubsystem m_driveSubsystem;
		private final DoubleSupplier _VX, _VY, _HdgAdjust;
		private final BooleanSupplier _LookAway, _LookTowards, _LookLeft, _LookRight;

		private boolean _resetHeading = false;
		private Translation2d tx = null;

		public AbsoluteDriveAdvanced(
			DriveSubsystem DriveSubsystem,
			DoubleSupplier vX,
			DoubleSupplier vY,
			DoubleSupplier hdgAdjust,
			BooleanSupplier lookAway,
			BooleanSupplier lookTowards,
			BooleanSupplier lookLeft,
			BooleanSupplier lookRight) {
			this.m_driveSubsystem = DriveSubsystem;
			this._VX = vX;
			this._VY = vY;
			this._HdgAdjust = hdgAdjust;
			this._LookAway = lookAway;
			this._LookTowards = lookTowards;
			this._LookLeft = lookLeft;
			this._LookRight = lookRight;

			addRequirements(this.m_driveSubsystem);
		}

		@Override
		public void initSendable(SendableBuilder builder) {
			super.initSendable(builder);
			builder.addDoubleProperty("LimitedTranslation", tx::getX, null);
			builder.addStringProperty("Translation", tx::toString, null);
		}

		@Override
		public void initialize() {
			_resetHeading = true;
		}

		@Override
		public void execute() {
			double hdgX = 0;
			double hdgY = 0;

			if (_LookAway.getAsBoolean()) {
				hdgY = -1;
			}
			if (_LookRight.getAsBoolean()) {
				hdgX = 1;
			}
			if (_LookLeft.getAsBoolean()) {
				hdgX = -1;
			}
			if (_LookTowards.getAsBoolean()) {
				hdgY = 1;
			}

			if (_resetHeading) {
				if (hdgX == 0 && hdgY == 0 && Math.abs(_HdgAdjust.getAsDouble()) == 0) {
					Rotation2d currentHdg = m_driveSubsystem.getHeading();
					hdgX = currentHdg.getSin();
					hdgY = currentHdg.getCos();
				}
				_resetHeading = false;
			}

			ChassisSpeeds desiredSpeeds = m_driveSubsystem.getTargetSpeeds(
				_VX.getAsDouble(), _VY.getAsDouble(),
				hdgX, hdgY);
			tx = SwerveController.getTranslation2d(desiredSpeeds);
			tx = SwerveMath.limitVelocity(
				tx,
				m_driveSubsystem.getFieldVelocity(),
				m_driveSubsystem.getPose(),
				Constants.Drive.SWERVE_LOOP_TIME.in(Units.Seconds),
				Constants.Robot.MASS.in(Units.Kilograms),
				List.of(Constants.Drive.CHASSIS),
				m_driveSubsystem.getSwerveDriveConfiguration());

			if (Constants.DebugLevel.isOrAll(Constants.DebugLevel.Drive))
				SmartDashboard.putData(this);

			if (hdgX == 0 && hdgY == 0 && Math.abs(_HdgAdjust.getAsDouble()) > 0) {
				_resetHeading = true;
				m_driveSubsystem.drive(
					tx,
					(Constants.Controller.DRIVER_TURN_CONSTANT * -1.0
						* _HdgAdjust.getAsDouble()),
					true);
			} else {
				m_driveSubsystem.drive(tx, desiredSpeeds.omegaRadiansPerSecond, true);
			}
		}

		@Override
		public void end(boolean interrupted) {
		}

		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public class AbsoluteFieldDrive extends Command {
		private final DriveSubsystem m_driveSubsystem;
		private final DoubleSupplier _VX, _VY, _Hdg;
		private Translation2d tx = null;

		public AbsoluteFieldDrive(
			DriveSubsystem DriveSubsystem,
			DoubleSupplier vX,
			DoubleSupplier vY,
			DoubleSupplier hdg) {
			this.m_driveSubsystem = DriveSubsystem;
			this._VX = vX;
			this._VY = vY;
			this._Hdg = hdg;

			addRequirements(this.m_driveSubsystem);
		}

		@Override
		public void initSendable(SendableBuilder builder) {
			super.initSendable(builder);
			builder.addDoubleProperty("LimitedTranslation", tx::getX, null);
			builder.addStringProperty("Translation", tx::toString, null);
		}

		@Override
		public void initialize() {
		}

		@Override
		public void execute() {
			ChassisSpeeds desiredSpeeds = m_driveSubsystem.getTargetSpeeds(
				_VX.getAsDouble(), _VY.getAsDouble(),
				new Rotation2d(_Hdg.getAsDouble() * Math.PI));
			Translation2d tx = SwerveController.getTranslation2d(desiredSpeeds);
			tx = SwerveMath.limitVelocity(
				tx,
				m_driveSubsystem.getFieldVelocity(),
				m_driveSubsystem.getPose(),
				Constants.Drive.SWERVE_LOOP_TIME.in(Units.Seconds),
				Constants.Robot.MASS.in(Units.Kilograms),
				List.of(Constants.Drive.CHASSIS),
				m_driveSubsystem.getSwerveDriveConfiguration());

			if (Constants.DebugLevel.isOrAll(Constants.DebugLevel.Drive))
				SmartDashboard.putData(this);

			m_driveSubsystem.drive(tx, desiredSpeeds.omegaRadiansPerSecond, true);
		}

		@Override
		public void end(boolean interrupted) {
		}

		@Override
		public boolean isFinished() {
			return false;
		}
	}
}
