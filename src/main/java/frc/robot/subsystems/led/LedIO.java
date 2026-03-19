package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface LedIO {
	@AutoLog
	public static class LedIOInputs implements LoggableInputs {
		public boolean hasEverEnabled = false;
		public String baseState = "UNKNOWN";

		public int notificationCount = 0;
		public String topNotificationColor = "None";

		public double timeSinceDisabled = 0.0;
		public double rainbowOffset = 0.0;

		public double[] currentColorRGB = new double[3];
		public String currentColorHex = "#000000";

		public double[] ledStripRGB = new double[0];

		@Override
		public void toLog(LogTable table) {
			table.put("HasEverEnabled", hasEverEnabled);
			table.put("BaseState", baseState);

			table.put("NotificationCount", notificationCount);
			table.put("TopNotificationColor", topNotificationColor);

			table.put("TimeSinceDisabled", timeSinceDisabled);
			table.put("RainbowOffset", rainbowOffset);

			table.put("CurrentColorRGB", currentColorRGB);
			table.put("CurrentColorHex", currentColorHex);

			table.put("LEDStripRGB", ledStripRGB);
		}

		@Override
		public void fromLog(LogTable table) {
			hasEverEnabled = table.get("HasEverEnabled", false);
			baseState = table.get("BaseState", "UNKNOWN");

			notificationCount = table.get("NotificationCount", 0);
			topNotificationColor = table.get("TopNotificationColor", "None");

			timeSinceDisabled = table.get("TimeSinceDisabled", 0.0);
			rainbowOffset = table.get("RainbowOffset", 0.0);

			currentColorRGB = table.get("CurrentColorRGB", new double[]{0, 0, 0});
			currentColorHex = table.get("CurrentColorHex", "#000000");

			ledStripRGB = table.get("LEDStripRGB", new double[0]);
		}
	}
}
