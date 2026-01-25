package frc.robot.lib;

import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.Constants.Drive;

public class ConversionLib {
	/**
	 * Converts falcon motor counts to radians - specific to the turn motor
	 * @param counts The counts to be converted to radians
	 * @return The radians for the specified counts.
	 */
	public static double vortexToRadians(double counts){
		return counts * (Constants.TWO_PI / (Drive.SWERVE_TURN_GEAR_RATIO * 1.0));
	}

	/**
	 * Converts turn motor sensor counts to wheel degrees using the turn gear ratio from DriveConstants
	 * @param counts Falcon sensor counts
	 * @return Wheel angle in degrees
	 */
	public static double vortexToDegrees(double counts){
		return counts * (360.0 / (Drive.SWERVE_TURN_GEAR_RATIO * 1.0));
	}

	/**
	 * Converts radians to Falcon motor counts. Specific to the turn motor
	 * @param radians the radians to convert to Falcon motor counts
	 * @return Falcon Counts - 1.0 = 1 rotation of the motor
	 */
	public static double radiansToVortex(double radians){
		return radians / (Constants.TWO_PI / (Drive.SWERVE_TURN_GEAR_RATIO * 1.0));
	}

	public static double degreesToFalcon(double degrees){
		return degrees / (360 / (Drive.SWERVE_TURN_GEAR_RATIO * 1.0));
	}

	/**
	 * Converts the drive motor velocity counts to motor RPM using the DriveConstants driveGearRatio
	 * @param velocitycounts Falcon motor counts which are counts in the last .1 seconds
	 * @return Motor RPM
	 */
	public static double falconToRPM(double velocitycounts)
	{
		//motor controller velocity counts are counts in 100 ms, or 0.1 seconds.
		// in one minute there are 600 of these .1 second intervals
		double motorRPM = velocitycounts * (600.0 / 1.0);
		double wheelRPM = motorRPM / Drive.SWERVE_DRIVE_GEAR_RATIO;
		return wheelRPM;
	}

	/**
	 * Converts drive motor velocity counts to wheel MPS
	 * @param velocitycounts
	 * @return
	 */
	public static double vortexToMPS(double velocitycounts)
	{
		double wheelRPM = falconToRPM(velocitycounts);
		double wheelMPS = (wheelRPM * Drive.Wheel.RADIUS.in(Units.Inches) * Constants.TWO_PI)/60;
		return wheelMPS;
	}

	public static double vortexToMeters(double positionCounts)
	{
		return positionCounts / Drive.SWERVE_DRIVE_GEAR_RATIO * Drive.Wheel.RADIUS.in(Units.Inches) * Constants.TWO_PI;
	}

	///
	/**
	 * Converts drive motor RPM to m/s.
	 * @param rpm
	 * @return
	 */
	public static double rpmToMps(double rpm) {
		return rpm * Constants.TWO_PI * Units.Inches.of(2.0 * Drive.Wheel.RADIUS.in(Units.Inches)).in(Units.Meters) / 60;
	}

	public static Integer[] intArrayToIntegerArray(int[] intArray) {
		Integer[] iArray = new Integer[intArray.length];
		for (int i = 0; i < intArray.length; i++) {
			iArray[i] = intArray[i];
		}
		return iArray;
	}

	public static int castBoolToInt(boolean bool) {
		return bool ? 1 : 0;
	}

	public static double falconToMeters(double positionCounts) {
		double distanceMeters = positionCounts / 1.0 / Drive.SWERVE_DRIVE_GEAR_RATIO * Drive.Wheel.RADIUS.in(Units.Inches) * Constants.TWO_PI;
		return distanceMeters;
	}

	/**
	 * Converts turn motor sensor counts to wheel degrees using the turn gear ratio from DriveConstants
	 * @param counts Falcon sensor counts
	 * @return Wheel angle in degrees
	 */
	public static double falconToDegrees(double counts){
		return counts * (360.0 / (Drive.SWERVE_TURN_GEAR_RATIO * 1.0));
	}

	/**
	 * Converts radians to Falcon motor counts. Specific to the turn motor
	 * @param radians the radians to convert to Falcon motor counts
	 * @return Falcon Counts - 1.0 = 1 rotation of the motor
	 */
	public static double radiansToFalcon(double radians){
		return radians / (Constants.TWO_PI / (Drive.SWERVE_TURN_GEAR_RATIO * 1.0));
	}
}