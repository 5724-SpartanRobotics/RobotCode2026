package frc.robot.lib;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.subsystems.VisionSubsystem;

import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public class VisionPoseEstimator {
	private final AprilTagFieldLayout _FieldLayout;
	private final Transform3d _Camera2RobotTransform;

	public static class VisionPose {
		public final Pose2d pose;
		public final double timestamp;

		public VisionPose(Pose2d pose, double timestamp) {
			this.pose = pose;
			this.timestamp = timestamp;
		}
	}

	public VisionPoseEstimator(AprilTagFieldLayout fieldLayout, Transform3d cameraToRobot) {
		this._FieldLayout = fieldLayout;
		this._Camera2RobotTransform = cameraToRobot;
	}

	/** Compute robot pose from a single tag measurement. */
	public Optional<Pose2d> estimatePoseFromTag(VisionSubsystem.TagMeasurement m) {
		Optional<Pose3d> tagPoseOpt = _FieldLayout.getTagPose(m.tagId);
		if (tagPoseOpt.isEmpty()) {
			return Optional.empty();
		}

		Pose3d tagPose = tagPoseOpt.get();

		double distanceMeters = Units.feetToMeters(m.distanceFeet);
		Rotation2d yaw = Rotation2d.fromDegrees(m.angleDegrees);

		// Camera sees tag at (distance, angle) in the camera's XY plane
		Translation2d cameraToTagTranslation =
				new Translation2d(distanceMeters, 0.0).rotateBy(yaw);

		Transform2d cameraToTag2d =
				new Transform2d(cameraToTagTranslation, new Rotation2d());

		// Lift to 3D (assume flat field, no pitch/roll)
		Transform3d cameraToTag3d = new Transform3d(
				new Translation3d(
						cameraToTag2d.getX(),
						cameraToTag2d.getY(),
						0.0
				),
				new Rotation3d(0.0, 0.0, cameraToTag2d.getRotation().getRadians())
		);

		// field→robot = field→tag ∘ (camera→tag)^-1 ∘ camera→robot
		Pose3d robotPose3d =
				tagPose.transformBy(cameraToTag3d.inverse()).transformBy(_Camera2RobotTransform);

		return Optional.of(robotPose3d.toPose2d());
	}

	/**
	 * Pick the "best" tags (closest) and return a fused pose list
	 * you can feed into your swerve pose estimator.
	 */
	public void addBestVisionPosesToSwerve(
			frc.robot.subsystems.DriveSubsystem drive,
			VisionSubsystem vision
	) {
		List<VisionSubsystem.TagMeasurement> measurements = vision.getAllMeasurements();
		if (measurements.isEmpty()) {
			return;
		}

		// Sort by distance (closest first)
		measurements.sort(Comparator.comparingDouble(m -> m.distanceFeet));

		// Use up to the 2 closest tags
		int maxTags = Math.min(2, measurements.size());
		double timestamp = Timer.getFPGATimestamp();

		for (int i = 0; i < maxTags; i++) {
			VisionSubsystem.TagMeasurement m = measurements.get(i);
			estimatePoseFromTag(m).ifPresent(pose -> {
				drive.addVisionMeasurement(pose, timestamp);
			});
		}
	}
}
