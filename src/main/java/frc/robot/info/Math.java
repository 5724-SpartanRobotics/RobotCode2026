package frc.robot.info;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearAcceleration;

public final class Math {
	public static final double E = java.lang.Math.E;
	public static final double PI = java.lang.Math.PI;
	public static final double TWO_PI = Math.PI * 2.0;
	public static final double HALF_PI = Math.PI / 2.0;
	public static final double THREE_HALVES_PI = (3.0 * Math.PI) / 2.0;
	public static final LinearAcceleration g = Units.MetersPerSecondPerSecond.of(9.80665);

	public static final Class<java.lang.Math> JavaMath = java.lang.Math.class;
}