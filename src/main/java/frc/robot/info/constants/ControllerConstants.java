package frc.robot.info.constants;

import frc.lib.ClassFieldMapStringToInt;
import frc.robot.info.Math;

public final class ControllerConstants {
	public static final double DRIVER_DEADBAND_XY = 0.15;
	public static final double DRIVER_DEADBAND_Z = 0.2;
	public static final double DRIVER_TURN_CONSTANT = Math.TWO_PI;

	public static final class DriverMap implements ClassFieldMapStringToInt {
		public static final int SPEEDMOD_MIN = 1; // trigger
		public static final int SPEEDMOD_MAX = 2; // thumb button
		public static final int ZERO_GYRO = 7;
		public static final int DRIVE_TO_POSE = 11;
		public static final int CENTER_SWERVES = 10;
		public static final int RESET_ODOMETRY = 8;
		public static final int RESET_ODOMETRY_FLIPPED = 5;
		public static final int DRIVE_TO_INITIAL_POSE = 9;
		public static final int TOGGLE_NOTIFICATION = 12;
	}

	public static final class OperatorMap implements ClassFieldMapStringToInt {
	}
}
