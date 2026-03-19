package frc.robot.info;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;

public final class Period {
	protected static Frequency PERIOD_FREQ = Units.Hertz.of(50);
	protected static Time PERIOD_TIME = PERIOD_FREQ.asPeriod();

	public static long setPeriod(Time period) {
		PERIOD_TIME = period.copy();
		PERIOD_FREQ = PERIOD_TIME.asFrequency();
		return getPeriod();
	}

	public static long getPeriod() {
		return Double.valueOf(PERIOD_TIME.in(Units.Seconds)).longValue();
	}

	public static void logPeriod() {
		NetworkTableInstance.getDefault().getEntry("/Period/Hz")
			.setDouble(PERIOD_FREQ.in(Units.Hertz));
		NetworkTableInstance.getDefault().getEntry("/Period/Seconds")
			.setDouble(PERIOD_TIME.in(Units.Seconds));
	}
}
