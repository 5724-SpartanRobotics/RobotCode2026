package frc.robot.info;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;

public final class Alliance {
	/**
	 * Checks if the alliance is red, defaults to false if alliance isn't available.
	 *
	 * @return true if the red alliance, false if blue. Defaults to false if none is available.
	 */
	public static boolean isRedAlliance() {
		var alliance = DriverStation.getAlliance();
		return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
	}

	public static boolean isBlueAlliance() {
		return !isRedAlliance();
	}

	public static Color getAllianceColor() {
		var alliance = DriverStation.getAlliance();
		if (alliance.isEmpty())
			return Color.kWhite;
		return switch (alliance.get()) {
			case Red -> Color.kRed;
			case Blue -> Color.kBlue;
			default -> Color.kWhite;
		};
	}

	public static Color getInverseAllianceColor() {
		var alliance = DriverStation.getAlliance();
		if (alliance.isEmpty())
			return Color.kWhite;
		return switch (alliance.get()) {
			case Red -> Color.kBlue;
			case Blue -> Color.kRed;
			default -> Color.kWhite;
		};
	}
}
