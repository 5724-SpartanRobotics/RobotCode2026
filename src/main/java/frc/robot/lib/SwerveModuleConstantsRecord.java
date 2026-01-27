package frc.robot.lib;

import edu.wpi.first.math.controller.PIDController;

/**
 * @param driveId Drive motor CAN ID
 * @param turnId Turn motor CAN ID
 * @param encoderId Absolute encoder CAN ID
 * @param encoderOffset Absolute encoder offset in <i>radians</i>
 * @param invert Whether to invert the motor.
 * @param pid Controller PID gains (esp. turn)
 */
public record SwerveModuleConstantsRecord(
	int driveId, int turnId, int encoderId, double encoderOffset,
	boolean invert, PIDController pid
) { }
