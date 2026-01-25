package frc.robot.lib;

import edu.wpi.first.math.controller.PIDController;

public record SwerveModuleConstantsRecord(
	int driveId, int turnId, int encoderId, double encoderOffset,
	boolean invert, PIDController pid
) { }
