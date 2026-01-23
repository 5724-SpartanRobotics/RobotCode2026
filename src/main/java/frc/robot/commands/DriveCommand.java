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
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import swervelib.SwerveController;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;

public class DriveCommand {
	private static DriveSubsystem _DriveSubsystem;
	private static SwerveInputStream _DriveAngularVelocity;
	private static SwerveInputStream _DriveDirectAngle;
	private static SwerveInputStream _DriveRobotOriented;
	private static SwerveInputStream _DriveAngularVelocity_Keyboard;
	private static SwerveInputStream _DriveDirectAngle_Keyboard;

	public static void initialize(DriveSubsystem drive, CommandJoystick joystick) {
		_DriveSubsystem = drive;
		_DriveAngularVelocity = SwerveInputStream.of(
			drive.getSwerveDrive(),
			() -> joystick.getRawAxis(1) * -1.0, // Y axis (forward/back)
			() -> joystick.getRawAxis(0) // X axis (strafe)
		)
			.withControllerRotationAxis(() -> joystick.getRawAxis(2)) // twist / rotation
			.deadband(Constants.Controller.DRIVER_DEADBAND)
			.scaleTranslation(0.8)
			.allianceRelativeControl(true);
		_DriveDirectAngle = _DriveAngularVelocity.copy()
			.withControllerHeadingAxis(
				() -> Math.sin(joystick.getRawAxis(2) * Math.PI) * Constants.TWO_PI,
				() -> Math.cos(joystick.getRawAxis(2) * Math.PI) * Constants.TWO_PI
			)
			.headingWhile(true);
		_DriveRobotOriented = _DriveAngularVelocity.copy()
			.robotRelative(false)
			.allianceRelativeControl(false);
		
		_DriveAngularVelocity_Keyboard = SwerveInputStream.of(
			drive.getSwerveDrive(),
			() -> joystick.getRawAxis(1) * -1.0,
			() -> joystick.getRawAxis(0) * -1.0
		)
			.withControllerRotationAxis(() -> joystick.getRawAxis(2))
			.deadband(Constants.Controller.DRIVER_DEADBAND)
			.scaleTranslation(0.8)
			.allianceRelativeControl(true);
		_DriveDirectAngle_Keyboard = _DriveAngularVelocity_Keyboard.copy()
			.withControllerHeadingAxis(
				() -> Math.sin(joystick.getRawAxis(2) * Math.PI) * Constants.TWO_PI,
				() -> Math.cos(joystick.getRawAxis(2) * Math.PI) * Constants.TWO_PI
			)
			.headingWhile(true)
			.translationHeadingOffset(true)
			.translationHeadingOffset(Rotation2d.fromDegrees(0));
	}

	public static Command getCommand() {
		return _DriveSubsystem.driveFieldOriented(_DriveAngularVelocity);
	}

	public class AbsoluteDrive extends Command {
		private final DriveSubsystem _DriveSubsystem;
		private final DoubleSupplier _VX, _VY, _HdgHorizontal, _HdgVertical;
		private boolean _initRotation = false;

		public AbsoluteDrive(
			DriveSubsystem driveSubsystem,
			DoubleSupplier vX,
			DoubleSupplier vY,
			DoubleSupplier hdgHorizontal,
			DoubleSupplier hdgVertical
		) {
			this._DriveSubsystem = driveSubsystem;
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
				Constants.Drive.SWERVE_LOOP_TIME,
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
		private final DriveSubsystem _DriveSubsystem;
		private final DoubleSupplier _VX, _VY, _HdgAdjust;
		private final BooleanSupplier _LookAway, _LookTowards, _LookLeft, _LookRight;

		private boolean _resetHeading = false;

		public AbsoluteDriveAdvanced(
			DriveSubsystem driveSubsystem,
			DoubleSupplier vX,
			DoubleSupplier vY,
			DoubleSupplier hdgAdjust,
			BooleanSupplier lookAway,
			BooleanSupplier lookTowards,
			BooleanSupplier lookLeft,
			BooleanSupplier lookRight
		) {
			this._DriveSubsystem = driveSubsystem;
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
				Constants.Drive.SWERVE_LOOP_TIME,
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
		private final DriveSubsystem _DriveSubsystem;
		private final DoubleSupplier _VX, _VY, _Hdg;

		public AbsoluteFieldDrive(
			DriveSubsystem driveSubsystem,
			DoubleSupplier vX,
			DoubleSupplier vY,
			DoubleSupplier hdg
		) {
			this._DriveSubsystem = driveSubsystem;
			this._VX = vX;
			this._VY = vY;
			this._Hdg = hdg;

			addRequirements(this._DriveSubsystem);
		}

		@Override
		public void initialize() {}

		@Override
		public void execute() {
			ChassisSpeeds desiredSpeeds = _DriveSubsystem.getTargetSpeeds(
				_VX.getAsDouble(), _VY.getAsDouble(),
				new Rotation2d(_Hdg.getAsDouble() * Math.PI)
			);
			Translation2d tx = SwerveController.getTranslation2d(desiredSpeeds);
			tx = SwerveMath.limitVelocity(
				tx,
				_DriveSubsystem.getFieldVelocity(),
				_DriveSubsystem.getPose(),
				Constants.Drive.SWERVE_LOOP_TIME,
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
