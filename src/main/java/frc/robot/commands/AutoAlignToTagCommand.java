package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.TagMeasurement;

public class AutoAlignToTagCommand extends Command {
	private final DriveSubsystem _DriveSubsystem;
	private final VisionSubsystem _VisionSubsystem;
	private final int _TagID;
	private final PIDController _TurnPID;

	public AutoAlignToTagCommand(DriveSubsystem drive, VisionSubsystem vision, int tagId) {
		this._DriveSubsystem = drive;
		this._VisionSubsystem = vision;
		this._TagID = tagId;
		this._TurnPID = new PIDController(0.03, 0.0, 0.001); // tune these
		this._TurnPID.setTolerance(2.0); // degrees
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		_VisionSubsystem.setWantedTag(_TagID);
	}

	@Override
	public void execute() {
		TagMeasurement m = _VisionSubsystem.getTagMeasurement(_TagID);
		if (m == null) {
			// No valid tag: stop rotation
			_DriveSubsystem.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
			return;
		}

		double errorDeg = m.angleDegrees; // Pi gives angle relative to camera center
		double omegaRadPerSec = _TurnPID.calculate(errorDeg, 0.0);

		// Rotate in place, no translation
		ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, omegaRadPerSec);
		_DriveSubsystem.setChassisSpeeds(speeds);
	}

	@Override
	public void end(boolean interrupted) {
		_DriveSubsystem.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
	}

	@Override
	public boolean isFinished() {
		return _TurnPID.atSetpoint();
	}
}
