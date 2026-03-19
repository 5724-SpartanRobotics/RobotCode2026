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
		Front(
			"front",
			// Camera translation relative to robot center (tune as needed)
			new Transform3d(
				new Translation3d(
					Units.Inches.of(2.0 + 7.0 / 8.0).in(Units.Meters),
					Units.Inches.of(14.5).in(Units.Meters),
					Units.Inches.of(12.75).in(Units.Meters)),
				new Rotation3d(0, 0, 0))),

		Back(
			"back",
			new Transform3d(
				new Translation3d(
					Units.Inches.of(-11.0).in(Units.Meters),
					Units.Inches.of(8.0 + 1.0 / 8.0).in(Units.Meters),
					Units.Inches.of(13.0).in(Units.Meters)),
				new Rotation3d(0, 0, Math.PI))),

		Right(
			"right",
			new Transform3d(
				new Translation3d(
					Units.Inches.of(1.0 / 8.0).in(Units.Meters),
					Units.Inches.of(-11.5).in(Units.Meters),
					Units.Inches.of(19.0 + 1.0 / 8.0).in(Units.Meters)),
				new Rotation3d(0, 0, frc.robot.info.Math.THREE_HALVES_PI)));

		private final String name;
		private final Transform3d robotToCamera;

		CameraConfigurations(
			String name,
			Transform3d robotCenterToCamera) {
			this.name = name;
			this.robotToCamera = robotCenterToCamera;
		}

		public String getName() {
			return name;
		}

		public Transform3d getTransform3d() {
			return robotToCamera;
		}
	}

	// Basic filtering thresholds
	public static double MAX_AMBIGUITY = 0.3;
	public static double MAX_Z_ERROR = 0.75;

	// Standard deviation baselines, for 1 meter distance and 1 tag
	// (Adjusted automatically based on distance and # of tags)
	public static double LINEAR_STDEV_BASELINE = 0.02; // Meters
	public static double ANGULAR_STDEV_BASELINE = 0.06; // Radians

	// Standard deviation multipliers for each camera
	// (Adjust to trust some cameras more than others)
	public static double[] CAMERA_STDEV_FACTORS = new double[]{
		1.0, // Camera 0 (Front)
		1.0, // Camera 1 (Back)
		0.9 // Camera 2 (Right)
	};

	// Multipliers to apply for MegaTag 2 observations
	public static double LINEAR_STDEB_MEGATAG2_FACTOR = 0.5; // More stable than full 3D solve
	public static double ANGULAR_STDEV_MEGATAG2_FACTOR = Double.POSITIVE_INFINITY; // No rotation
																					// data
																					// available
}