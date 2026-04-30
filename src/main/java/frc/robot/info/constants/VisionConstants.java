package frc.robot.info.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;

public class VisionConstants {
	// AprilTag layout
	public static AprilTagFieldLayout LAYOUT = AprilTagFieldLayout
		.loadField(AprilTagFields.k2026RebuiltAndymark);

	public enum CameraConfigurations {
		Front( // bad
			"front",
			// Camera translation relative to robot center (tune as needed)
			new Transform3d(
				new Translation3d(
					Units.Inches.of(-0.5).in(Units.Meters),
					Units.Inches.of(14.25 - 3.0).in(Units.Meters),
					Units.Inches.of(20).in(Units.Meters)),
				new Rotation3d(0, 0, 0)),
			0.9),

		Back( // bad
			"back",
			new Transform3d(
				new Translation3d(
					Units.Inches.of(-4.0).in(Units.Meters),
					Units.Inches.of(14.25 - 3.0).in(Units.Meters),
					Units.Inches.of(20).in(Units.Meters)),
				new Rotation3d(0, 0, Math.PI)),
			0.9),

		Right( // good
			"right",
			new Transform3d(
				new Translation3d(
					Units.Inches.of(6.25).in(Units.Meters),
					Units.Inches.of(-14.25).in(Units.Meters),
					Units.Inches.of(18.5).in(Units.Meters)),
				new Rotation3d(0, 0, frc.robot.info.Math.THREE_HALVES_PI)),
			0.775);

		private final String name;
		private final Transform3d robotToCamera;
		private final double trustFactor;

		CameraConfigurations(
			String name,
			Transform3d robotCenterToCamera,
			double trustFactor) {
			this.name = name;
			this.robotToCamera = robotCenterToCamera;
			this.trustFactor = trustFactor;
		}

		public String getName() {
			return name;
		}

		public Transform3d getTransform3d() {
			return robotToCamera;
		}

		public double getTrustFactor() {
			return trustFactor;
		}
	}

	// Basic filtering thresholds
	public static final double MAX_AMBIGUITY = 0.3;
	public static final double MAX_Z_ERROR = 0.75;

	// Standard deviation baselines, for 1 meter distance and 1 tag
	// (Adjusted automatically based on distance and # of tags)
	public static final double LINEAR_STDEV_BASELINE = 0.5; // Meters
	public static final double ANGULAR_STDEV_BASELINE = 1.5; // Radians

	// Multipliers to apply for MegaTag 2 observations
	public static final double LINEAR_STDEB_MEGATAG2_FACTOR = 0.5; // More stable than full 3D solve
	public static final double ANGULAR_STDEV_MEGATAG2_FACTOR = Double.POSITIVE_INFINITY; // No
																							// rotation
	// data
	// available

	public static final boolean DESKTOP_OPEN_CAMERA_SIM = false;
}