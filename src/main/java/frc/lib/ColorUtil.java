package frc.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;

public final class ColorUtil {
	public static Color plusRGB(Color c, int r, int g, int b) {
		return new Color(
			MathUtil.clamp(c.red + r, 0, 255),
			MathUtil.clamp(c.green + g, 0, 255),
			MathUtil.clamp(c.blue + b, 0, 255));
	}

	public static Color minusRGB(Color c, int r, int g, int b) {
		return new Color(
			MathUtil.clamp(c.red - r, 0, 255),
			MathUtil.clamp(c.green - g, 0, 255),
			MathUtil.clamp(c.blue - b, 0, 255));
	}

	public static Color plus(Color c, Color other) {
		return new Color(
			MathUtil.clamp(c.red + other.red, 0, 255),
			MathUtil.clamp(c.green + other.green, 0, 255),
			MathUtil.clamp(c.blue + other.blue, 0, 255));
	}

	public static Color minus(Color c, Color other) {
		return new Color(
			MathUtil.clamp(c.red - other.red, 0, 255),
			MathUtil.clamp(c.green - other.green, 0, 255),
			MathUtil.clamp(c.blue - other.blue, 0, 255));
	}
}
