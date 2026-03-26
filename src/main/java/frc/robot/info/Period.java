package frc.robot.info;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;

public final class Period {
	protected static Time PERIOD_TIME = Units.Seconds.of(0.02);
	protected static Frequency PERIOD_FREQ = PERIOD_TIME.asFrequency();

	public static double setPeriod(Time period) {
		PERIOD_TIME = period.copy();
		PERIOD_FREQ = PERIOD_TIME.asFrequency();
		return getPeriod();
	}

	public static double getPeriod() {
		return PERIOD_TIME.in(Units.Seconds);
	}

	public static void logPeriod() {
		NetworkTableInstance.getDefault().getEntry("/Period/Hz")
			.setDouble(PERIOD_FREQ.in(Units.Hertz));
		NetworkTableInstance.getDefault().getEntry("/Period/Seconds")
			.setDouble(PERIOD_TIME.in(Units.Seconds));
	}
}
