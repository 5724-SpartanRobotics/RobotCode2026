/* VisionSubsystem.java */

package frc.robot.subsystems.vision;

import java.awt.Desktop;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.lib.Camera;
import frc.robot.lib.NopSubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;

@AutoLog
public class VisionSubsystem extends NopSubsystemBase {
	public static final AprilTagFieldLayout FIELD_LAYOUT = Constants.Vision.FIELD_LAYOUT;

	public VisionSystemSim m_visionSim;

	private final Supplier<Pose2d> m_currentPose2d;
	private final Field2d m_field2d;
	private DriveSubsystem m_driveSubsystem;

	private static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
	private static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

	public VisionSubsystem(Supplier<Pose2d> currentPose, Field2d field) {
		this.m_currentPose2d = currentPose;
		this.m_field2d = field;

		if (RobotBase.isSimulation()) {
			m_visionSim = new VisionSystemSim("Vision");
			m_visionSim.addAprilTags(FIELD_LAYOUT);

			for (Cameras c : Cameras.values()) {
				c.addToVisionSim(m_visionSim);
			}

			openSimCameraViews();
		}
	}

	public double getDistanceFromAprilTag(int id) {
		Optional<Pose3d> tag = FIELD_LAYOUT.getTagPose(id);
		return tag.map(
			pose3d -> PhotonUtils.getDistanceToPose(m_currentPose2d.get(), pose3d.toPose2d()))
			.orElse(-1.0);
	}

	public Optional<Pose2d> getTagPose(int id) {
		var tag = FIELD_LAYOUT.getTagPose(id);
		if (tag.isEmpty())
			return Optional.empty();
		return Optional.of(tag.get().toPose2d());
	}

	public VisionSystemSim getM_visionSim() {
		return m_visionSim;
	}

	private void openSimCameraViews() {
		if (Desktop.isDesktopSupported()
			&& Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
			try {
				Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
			} catch (IOException | URISyntaxException e) {
				e.printStackTrace();
			}
		}
	}

	public void setDriveSubsystem(DriveSubsystem yds) {
		this.m_driveSubsystem = yds;
	}

	// Main periodic: update results, estimate pose,
	@Override
	public void periodic() {
		for (var camera : Cameras.values()) {
			var c = camera.getCamera();

			c.periodic();

			Optional<EstimatedRobotPose> est;
			Matrix<N3, N1> std;

			synchronized (c.lock) {
				est = c.getLatestThreadedEstimate();
				std = c.getLatestStdDevs();

				if (est.isEmpty())
					continue;

				m_field2d.getObject("VisionEstimation")
					.setPose(est.get().estimatedPose.toPose2d());

				if (m_driveSubsystem != null) {
					m_driveSubsystem.addVisionMeasurement(
						est.get().estimatedPose.toPose2d(),
						est.get().timestampSeconds, std);
				}
			}
		}
	}

	public static Matrix<N3, N1> updateEstimationStdDevs(
		Camera camera,
		Optional<EstimatedRobotPose> estPose,
		List<PhotonTrackedTarget> targets) {
		if (estPose.isEmpty()) {
			camera.singleTagStdDevs = camera.curStdDevs = kSingleTagStdDevs;
			return kSingleTagStdDevs;
		}

		var estStdDevs = kSingleTagStdDevs;
		int numTags = 0;
		double avgDist = 0;

		for (var t : targets) {
			var tagPose = camera.poseEstimator.getFieldTags().getTagPose(t.getFiducialId());
			if (tagPose.isEmpty())
				continue;
			numTags++;
			avgDist += tagPose.get()
				.toPose2d().getTranslation()
				.getDistance(estPose.get().estimatedPose.toPose2d().getTranslation());
		}

		if (numTags == 0) {
			camera.singleTagStdDevs = camera.multiTagStdDevs = camera.curStdDevs = kSingleTagStdDevs;
		} else {
			avgDist /= numTags;
			if (numTags > 1)
				estStdDevs = kMultiTagStdDevs;
			if (numTags == 1 && avgDist > 4)
				estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
			else
				estStdDevs = estStdDevs.times(1.0 + (avgDist * avgDist / 30.0));
			camera.multiTagStdDevs = camera.curStdDevs = estStdDevs;
		}
		return camera.curStdDevs;
	}

	public enum Cameras {
		// All rotations are CCW positive in radians
		FRONT(
			// 🔴 CHANGE THIS STRING to match the camera name in PhotonVision exactly
			"front",
			// Camera rotation relative to robot (tune as needed)
			new Rotation3d(0, 0, 0),
			// Camera translation relative to robot center (tune as needed)
			new Translation3d(
				Units.inchesToMeters(2.0 + 7.0 / 8.0),
				Units.inchesToMeters(14.5),
				Units.inchesToMeters(12.75)),
			// Single-tag std devs
			VecBuilder.fill(1.0, 1.0, 3.0),
			// Multi-tag std devs
			VecBuilder.fill(0.5, 0.5, 1.0)),

		BACK(
			// 🔴 CHANGE THIS STRING to match the camera name in PhotonVision exactly
			"back",
			// Camera rotation relative to robot (tune as needed)
			// This one might have a roll of 180deg.
			new Rotation3d(0, 0, Units.degreesToRadians(180)),
			// Camera translation relative to robot center (tune as needed)
			new Translation3d(
				Units.inchesToMeters(-11.0),
				Units.inchesToMeters(8.0 + 1.0 / 8.0),
				Units.inchesToMeters(13.0)),
			// Single-tag std devs
			VecBuilder.fill(1.0, 1.0, 3.0),
			// Multi-tag std devs
			VecBuilder.fill(0.5, 0.5, 1.0)),

		RIGHT(
			// 🔴 CHANGE THIS STRING to match the camera name in PhotonVision exactly
			"right",
			// Camera rotation relative to robot (tune as needed)
			new Rotation3d(0, 0, Constants.THREE_HALVES_PI),
			// Camera translation relative to robot center (tune as needed)
			new Translation3d(
				Units.inchesToMeters(1.0 / 8.0),
				Units.inchesToMeters(-11.5),
				Units.inchesToMeters(19.0 + 1.0 / 8.0)),
			// Single-tag std devs
			VecBuilder.fill(1.0, 1.0, 3.0),
			// Multi-tag std devs
			VecBuilder.fill(0.5, 0.5, 1.0));

		private final Camera c;

		Cameras(String name,
			Rotation3d robotToCamRotation,
			Translation3d robotToCamTranslation,
			Matrix<N3, N1> singleTagStdDevs,
			Matrix<N3, N1> multiTagStdDevsMatrix) {
			c = new Camera(name,
				robotToCamRotation,
				robotToCamTranslation,
				singleTagStdDevs,
				multiTagStdDevsMatrix);
		}

		public void addToVisionSim(VisionSystemSim systemSim) {
			c.addToVisionSim(systemSim);
		}

		public Camera getCamera() {
			return c;
		}
	}
}
