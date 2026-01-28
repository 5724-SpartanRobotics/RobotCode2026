package frc.robot.commands;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.subsystems.YagslDriveSubsystem;
import swervelib.SwerveController;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;

public class YagslDriveCommand {
	private static YagslDriveSubsystem _DriveSubsystem;
	private static CommandJoystick _Joystick;
	// public static SwerveInputStream DriveAngularVelocity;
	// public static SwerveInputStream DriveDirectAngle;
	// public static SwerveInputStream DriveRobotOriented;
	// public static SwerveInputStream DriveAngularVelocity_Keyboard;
	// public static SwerveInputStream DriveDirectAngle_Keyboard;

	public static double speedMod = Constants.Robot.DEFAULT_SPEED_MOD;

	private static double applyJoystickDeadband(double realValue, double deadband) {
		if (Math.abs(realValue) < deadband) return 0.0;
		return realValue;
	}

	private static double applyJoystickScale(double realValue, double scale) {
		return realValue * scale;
	}

	private static double applyJoystickDeadbandAndScale(double realValue, double deadband) {
		return applyJoystickScale(
			applyJoystickDeadband(realValue, deadband),
			speedMod
		);
	}

	public static void initialize(YagslDriveSubsystem drive, CommandJoystick joystick) {
		_DriveSubsystem = drive;
		_Joystick = joystick;
		System.out.println("===== The <<DriveCommand>>s have been initialized. =====");
	}

	public static SwerveInputStream DriveAngularVelocity() {
		System.out.println("RETURNING A DRIVE COMMAND: DAV");
		return SwerveInputStream.of(
			_DriveSubsystem.getSwerveDrive(),
			() -> applyJoystickDeadbandAndScale(
				_Joystick.getRawAxis(1) * -1.0,
				Constants.Controller.DRIVER_DEADBAND_XY
			), // Y axis (forward/back)
			() -> applyJoystickDeadbandAndScale(
				_Joystick.getRawAxis(0),
				Constants.Controller.DRIVER_DEADBAND_XY
			) // X axis (strafe)
		)
			.withControllerRotationAxis(() ->
				applyJoystickDeadbandAndScale(
					_Joystick.getRawAxis(2) * -1.0,
					Constants.Controller.DRIVER_DEADBAND_Z
				)
			) // twist / rotation
			.deadband(Constants.Controller.DRIVER_DEADBAND_XY)
			.scaleTranslation(0.8)
			.allianceRelativeControl(true);
	}
	public static SwerveInputStream DriveDirectAngle() {
		System.out.println("RETURNING A DRIVE COMMAND: DDA");
		return DriveAngularVelocity().copy()
			.withControllerHeadingAxis(
				() -> Math.sin(_Joystick.getX() * Math.PI) * Constants.TWO_PI,
				() -> Math.cos(_Joystick.getY() * Math.PI * -1.0) * Constants.TWO_PI
			)
			.headingWhile(true);
	}
	public static SwerveInputStream DriveRobotOriented() {
		System.out.println("RETURNING A DRIVE COMMAND: DRO");
		return DriveAngularVelocity().copy()
			.robotRelative(false)
			.allianceRelativeControl(false);
	}

	public static SwerveInputStream DriveAngularVelocity_Keyboard() {
		System.out.println("RETURNING A DRIVE COMMAND: DAVK");
		return SwerveInputStream.of(
			_DriveSubsystem.getSwerveDrive(),
			() -> _Joystick.getRawAxis(1) * -1.0,
			() -> _Joystick.getRawAxis(0) * -1.0
		)
			.withControllerRotationAxis(() -> _Joystick.getRawAxis(2))
			.deadband(Constants.Controller.DRIVER_DEADBAND_XY)
			.scaleTranslation(0.8)
			.allianceRelativeControl(true);
	}
	public static SwerveInputStream DriveDirectAngle_Keyboard() {
		System.out.println("RETURNING A DRIVE COMMAND: DDAK");
		return DriveAngularVelocity_Keyboard().copy()
			.withControllerHeadingAxis(
				() -> Math.sin(_Joystick.getRawAxis(2) * Math.PI) * Constants.TWO_PI,
				() -> Math.cos(_Joystick.getRawAxis(2) * Math.PI) * Constants.TWO_PI
			)
			.headingWhile(true)
			.translationHeadingOffset(true)
			.translationHeadingOffset(Rotation2d.fromDegrees(0));
	}

	public enum DriveType {
		FO_AngularVelocity,
		FO_DirectAngle,
		RO_AngularVelocity,
		SetpointGenerator
	}

	public static Command getCommand(DriveType t, boolean isKeyboard) {
		System.out.println("\\\\\\\\\\\\\\ DRIVE COMMAND GOTTEN");
		if (t == DriveType.RO_AngularVelocity && isKeyboard) {
			return Commands.none();
		}
		return switch (t) {
			case FO_AngularVelocity -> _DriveSubsystem.driveFieldOriented(!isKeyboard ?
				DriveAngularVelocity() : DriveAngularVelocity_Keyboard());
			case FO_DirectAngle -> _DriveSubsystem.driveFieldOriented(!isKeyboard ?
				DriveDirectAngle() : DriveDirectAngle_Keyboard());
			case RO_AngularVelocity -> _DriveSubsystem.driveFieldOriented(DriveRobotOriented());
			case SetpointGenerator -> _DriveSubsystem.driveWithSetpointGeneratorFieldRelative(
				!isKeyboard ? DriveDirectAngle() : DriveDirectAngle_Keyboard());
		};
	}

	public class AbsoluteDrive extends Command {
		private final YagslDriveSubsystem _DriveSubsystem;
		private final DoubleSupplier _VX, _VY, _HdgHorizontal, _HdgVertical;
		private boolean _initRotation = false;

		public AbsoluteDrive(
			YagslDriveSubsystem YagslDriveSubsystem,
			DoubleSupplier vX,
			DoubleSupplier vY,
			DoubleSupplier hdgHorizontal,
			DoubleSupplier hdgVertical
		) {
			this._DriveSubsystem = YagslDriveSubsystem;
			this._VX = vX;
			this._VY = vY;
			this._HdgHorizontal = hdgHorizontal;
			this._HdgVertical = hdgVertical;

			addRequirements(this._DriveSubsystem);
		}

		@Override
		public void initialize() {
			_initRotation = true;
		}

		@Override
		public void execute() {
			System.out.println("AbsoluteDrive");

			ChassisSpeeds desiredSpeeds = _DriveSubsystem.getTargetSpeeds(
				_VX.getAsDouble(), _VY.getAsDouble(),
				_HdgHorizontal.getAsDouble(), _HdgVertical.getAsDouble()
			);

			if (_initRotation) {
				if (_HdgHorizontal.getAsDouble() == 0 && _HdgVertical.getAsDouble() == 0) {
					Rotation2d firstLoopHdg = _DriveSubsystem.getHeading();
					desiredSpeeds = _DriveSubsystem.getTargetSpeeds(
						0, 0,
						firstLoopHdg.getSin(), firstLoopHdg.getCos()
					);
				}
				_initRotation = false;
			}

			Translation2d tx = SwerveController.getTranslation2d(desiredSpeeds);
			tx = SwerveMath.limitVelocity(
				tx,
				_DriveSubsystem.getFieldVelocity(),
				_DriveSubsystem.getPose(),
				Constants.Drive.SWERVE_LOOP_TIME.in(Units.Seconds),
				Constants.Robot.MASS.in(Units.Kilograms),
				List.of(Constants.Drive.CHASSIS),
				_DriveSubsystem.getSwerveDriveConfiguration()
			);
			SmartDashboard.putNumber("LimitedTranslation", tx.getX());
			SmartDashboard.putString("Translation", tx.toString());

			_DriveSubsystem.drive(tx, desiredSpeeds.omegaRadiansPerSecond, true);
		}

		@Override
		public void end(boolean interrupted) {}

		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public class AbsoluteDriveAdvanced extends Command {
		private final YagslDriveSubsystem _DriveSubsystem;
		private final DoubleSupplier _VX, _VY, _HdgAdjust;
		private final BooleanSupplier _LookAway, _LookTowards, _LookLeft, _LookRight;

		private boolean _resetHeading = false;

		public AbsoluteDriveAdvanced(
			YagslDriveSubsystem YagslDriveSubsystem,
			DoubleSupplier vX,
			DoubleSupplier vY,
			DoubleSupplier hdgAdjust,
			BooleanSupplier lookAway,
			BooleanSupplier lookTowards,
			BooleanSupplier lookLeft,
			BooleanSupplier lookRight
		) {
			this._DriveSubsystem = YagslDriveSubsystem;
			this._VX = vX;
			this._VY = vY;
			this._HdgAdjust = hdgAdjust;
			this._LookAway = lookAway;
			this._LookTowards = lookTowards;
			this._LookLeft = lookLeft;
			this._LookRight = lookRight;

			addRequirements(this._DriveSubsystem);
		}

		@Override
		public void initialize() {
			_resetHeading = true;
		}

		@Override
		public void execute() {
			System.out.println("AbsoluteDriveAdv");
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
					Rotation2d currentHdg = _DriveSubsystem.getHeading();
					hdgX = currentHdg.getSin();
					hdgY = currentHdg.getCos();
				}
				_resetHeading = false;
			}

			ChassisSpeeds desiredSpeeds = _DriveSubsystem.getTargetSpeeds(
				_VX.getAsDouble(), _VY.getAsDouble(),
				hdgX, hdgY
			);
			Translation2d tx = SwerveController.getTranslation2d(desiredSpeeds);
			tx = SwerveMath.limitVelocity(
				tx,
				_DriveSubsystem.getFieldVelocity(),
				_DriveSubsystem.getPose(),
				Constants.Drive.SWERVE_LOOP_TIME.in(Units.Seconds),
				Constants.Robot.MASS.in(Units.Kilograms),
				List.of(Constants.Drive.CHASSIS),
				_DriveSubsystem.getSwerveDriveConfiguration()
			);
			SmartDashboard.putNumber("LimitedTranslation", tx.getX());
			SmartDashboard.putString("Translation", tx.toString());

			if (hdgX == 0 && hdgY == 0 && Math.abs(_HdgAdjust.getAsDouble()) > 0) {
				_resetHeading = true;
				_DriveSubsystem.drive(
					tx,
					(Constants.Controller.DRIVER_TURN_CONSTANT * -1.0 * _HdgAdjust.getAsDouble()),
					true
				);
			} else {
				_DriveSubsystem.drive(tx, desiredSpeeds.omegaRadiansPerSecond, true);
			}
		}

		@Override
		public void end(boolean interrupted) {}

		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public class AbsoluteFieldDrive extends Command {
		private final YagslDriveSubsystem _DriveSubsystem;
		private final DoubleSupplier _VX, _VY, _Hdg;

		public AbsoluteFieldDrive(
			YagslDriveSubsystem YagslDriveSubsystem,
			DoubleSupplier vX,
			DoubleSupplier vY,
			DoubleSupplier hdg
		) {
			this._DriveSubsystem = YagslDriveSubsystem;
			this._VX = vX;
			this._VY = vY;
			this._Hdg = hdg;

			addRequirements(this._DriveSubsystem);
		}

		@Override
		public void initialize() {}

		@Override
		public void execute() {
			System.out.println("AbsoluteFieldDrive");

			ChassisSpeeds desiredSpeeds = _DriveSubsystem.getTargetSpeeds(
				_VX.getAsDouble(), _VY.getAsDouble(),
				new Rotation2d(_Hdg.getAsDouble() * Math.PI)
			);
			Translation2d tx = SwerveController.getTranslation2d(desiredSpeeds);
			tx = SwerveMath.limitVelocity(
				tx,
				_DriveSubsystem.getFieldVelocity(),
				_DriveSubsystem.getPose(),
				Constants.Drive.SWERVE_LOOP_TIME.in(Units.Seconds),
				Constants.Robot.MASS.in(Units.Kilograms),
				List.of(Constants.Drive.CHASSIS),
				_DriveSubsystem.getSwerveDriveConfiguration()
			);
			SmartDashboard.putNumber("LimitedTranslation", tx.getX());
			SmartDashboard.putString("Translation", tx.toString());

			_DriveSubsystem.drive(tx, desiredSpeeds.omegaRadiansPerSecond, true);
		}

		@Override
		public void end(boolean interrupted) {}

		@Override
		public boolean isFinished() {
			return false;
		}
	}
}
