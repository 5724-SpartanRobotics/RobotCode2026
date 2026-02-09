package frc.robot.subsystems;

import java.awt.Desktop;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

public class VisionSubsystem {

	public static final AprilTagFieldLayout FIELD_LAYOUT = Constants.Vision.FIELD_LAYOUT;

	public VisionSystemSim m_visionSim;

	private final Supplier<Pose2d> m_currentPose2d;
	private final Field2d m_field2d;
	private DriveSubsystem m_driveSubsystem;

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

	public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
		Optional<Pose3d> aprilTagPose3d = FIELD_LAYOUT.getTagPose(aprilTag);
		if (aprilTagPose3d.isPresent()) {
			return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
		} else {
			throw new RuntimeException(
					"Cannot get AprilTag " + aprilTag +
					" from field " + FIELD_LAYOUT.toString());
		}
	}

	public void updatePoseEstimation(SwerveDrive swerveDrive) {
		// no-op, you’re using periodic() below for pose fusion
	}

	public void _updatePoseEstimation(SwerveDrive swerveDrive) {
		if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
			m_visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
		}
		for (Cameras camera : Cameras.values()) {
			if (camera == null) continue;
			Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
			if (poseEst != null && poseEst.isPresent()) {
				var pose = poseEst.get();
				if (pose == null) continue;
				swerveDrive.addVisionMeasurement(
						pose.estimatedPose.toPose2d(),
						pose.timestampSeconds,
						camera.curStdDevs);
			}
		}
	}

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
		Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
		if (RobotBase.isSimulation()) {
			Field2d debugField = m_visionSim.getDebugField();
			poseEst.ifPresentOrElse(
					est -> debugField.getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
					() -> debugField.getObject("VisionEstimation").setPoses());
		}
		return poseEst;
	}

	public double getDistanceFromAprilTag(int id) {
		Optional<Pose3d> tag = FIELD_LAYOUT.getTagPose(id);
		return tag.map(pose3d ->
				PhotonUtils.getDistanceToPose(m_currentPose2d.get(), pose3d.toPose2d()))
				.orElse(-1.0);
	}

	public Optional<Pose2d> getTagPose(int id) {
		var tag = FIELD_LAYOUT.getTagPose(id);
		if (tag.isEmpty()) return Optional.empty();
		return Optional.of(tag.get().toPose2d());
	}

	public PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
		PhotonTrackedTarget target = null;
		for (PhotonPipelineResult result : camera.resultsList) {
			if (result.hasTargets()) {
				for (PhotonTrackedTarget i : result.getTargets()) {
					if (i.getFiducialId() == id) {
						return i;
					}
				}
			}
		}
		return target;
	}

	public VisionSystemSim getM_visionSim() {
		return m_visionSim;
	}

	private void openSimCameraViews() {
		if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
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

	// Main periodic: update results, estimate pose, fuse into drivetrain
	public void periodic() {
		for (var camera : Cameras.values()) {
			// Pull all unread results
			for (var result : camera.camera.getAllUnreadResults()) {

				// Keep a small cache so getLatestResult() works
				camera.resultsList.add(0, result);
				if (camera.resultsList.size() > 20) {
					camera.resultsList.remove(camera.resultsList.size() - 1);
				}

				// Multi-tag first, then lowest ambiguity
				Optional<EstimatedRobotPose> visionEst =
						camera.poseEstimator.estimateCoprocMultiTagPose(result);
				if (visionEst.isEmpty()) {
					visionEst = camera.poseEstimator.estimateLowestAmbiguityPose(result);
				}

				SmartDashboard.putString(
						String.format("VisionSubsystemCameras/%s/EstPose", camera.friendlyName),
						visionEst.orElse(new EstimatedRobotPose(new Pose3d(), 0, new ArrayList<>()))
								.estimatedPose.toString());
				SmartDashboard.putNumberArray(
						String.format("VisionSubsystemCameras/%s/TargetIds", camera.friendlyName),
						result.getTargets().stream().mapToDouble(PhotonTrackedTarget::getFiducialId).toArray());

				updateEstimationStdDevs(camera, visionEst, result.getTargets());

				visionEst.ifPresentOrElse(est -> {
					m_field2d.getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d());
				}, () -> {
					m_field2d.getObject("VisionEstimation").setPoses();
				});

				visionEst.ifPresent(est -> {
					var estStdDevs = camera.curStdDevs;
					if (m_driveSubsystem != null) {
						m_driveSubsystem.addVisionMeasurement(
								est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
					}
				});

				camera.estimatedRobotPose = visionEst;
			}
		}
	}

    // public void periodic() {}

	public void updateEstimationStdDevs(
			Cameras camera,
			Optional<EstimatedRobotPose> estPose,
			List<PhotonTrackedTarget> targets) {

		final var SingleTagStdDevs = VecBuilder.fill(4, 4, 8);
		final var MultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

		if (estPose.isEmpty()) {
			camera.curStdDevs = SingleTagStdDevs;
			return;
		}

		var estStdDevs = SingleTagStdDevs;
		int numTags = 0;
		double avgDist = 0;

		for (var t : targets) {
			var tagPose = camera.poseEstimator.getFieldTags().getTagPose(t.getFiducialId());
			if (tagPose.isEmpty()) continue;
			numTags++;
			avgDist += tagPose.get()
					.toPose2d().getTranslation()
					.getDistance(estPose.get().estimatedPose.toPose2d().getTranslation());
		}

		if (numTags == 0) {
			camera.curStdDevs = SingleTagStdDevs;
		} else {
			avgDist /= numTags;
			if (numTags > 1) estStdDevs = MultiTagStdDevs;
			if (numTags == 1 && avgDist > 4)
				estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
			else estStdDevs = estStdDevs.times(1.0 + (avgDist * avgDist / 30.0));
			camera.curStdDevs = estStdDevs;
		}
	}

	public enum Cameras {

		CENTER_CAM(
			// 🔴 CHANGE THIS STRING to match the camera name in PhotonVision exactly
			"center",
			// Camera rotation relative to robot (tune as needed)
			new Rotation3d(0, Units.degreesToRadians(15), 0),
			// Camera translation relative to robot center (tune as needed)
			new Translation3d(
				Units.inchesToMeters(-8),
				Units.inchesToMeters(0),
				Units.inchesToMeters(21)
			),
			// Single-tag std devs
			VecBuilder.fill(1.0, 1.0, 3.0),
			// Multi-tag std devs
			VecBuilder.fill(0.5, 0.5, 1.0)
		),
		CENTER_BACK_CAM(
			// 🔴 CHANGE THIS STRING to match the camera name in PhotonVision exactly
			"center-back",
			// Camera rotation relative to robot (tune as needed)
			new Rotation3d(0, 0, 0),
			// Camera translation relative to robot center (tune as needed)
			new Translation3d(
				Units.inchesToMeters(-14.5),
				Units.inchesToMeters(3.75),
				Units.inchesToMeters(26.5)
			),
			// Single-tag std devs
			VecBuilder.fill(1.0, 1.0, 3.0),
			// Multi-tag std devs
			VecBuilder.fill(0.5, 0.5, 1.0)
		);

		public final Alert latencyAlert;
		public final PhotonCamera camera;
		public final PhotonPoseEstimator poseEstimator;
		// private final Matrix<N3, N1> singleTagStdDevs;
		// private final Matrix<N3, N1> multiTagStdDevs;
		private final Transform3d robotToCamTransform;
		public Matrix<N3, N1> curStdDevs;
		public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();
		public PhotonCameraSim cameraSim;
		public List<PhotonPipelineResult> resultsList = new ArrayList<>();
		public final String friendlyName;

		Cameras(
				String name,
				Rotation3d robotToCamRotation,
				Translation3d robotToCamTranslation,
				Matrix<N3, N1> singleTagStdDevs,
				Matrix<N3, N1> multiTagStdDevsMatrix) {

			friendlyName = name;
			latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

			camera = new PhotonCamera(name);
			// 🔴 CHANGE THIS INDEX to your AprilTag pipeline index in PhotonVision
			camera.setPipelineIndex(0);

			robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

			poseEstimator = new PhotonPoseEstimator(
					Constants.Vision.FIELD_LAYOUT,
					robotToCamTransform);

			// this.singleTagStdDevs = singleTagStdDevs;
			// this.multiTagStdDevs = multiTagStdDevsMatrix;
			this.curStdDevs = singleTagStdDevs;

			if (RobotBase.isSimulation()) {
				SimCameraProperties cameraProp = new SimCameraProperties();
				cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
				cameraProp.setCalibError(0.25, 0.08);
				cameraProp.setFPS(30);
				cameraProp.setAvgLatencyMs(35);
				cameraProp.setLatencyStdDevMs(5);

				cameraSim = new PhotonCameraSim(camera, cameraProp);
				cameraSim.enableDrawWireframe(true);
			}

			SmartDashboard.putString(
					String.format("VisionSubsystemCameras/%s/ZNote", name),
					"EstPose is a Pose3d of the vision estimate, or all zeros/empty if none.");
		}

		public void addToVisionSim(VisionSystemSim systemSim) {
			if (RobotBase.isSimulation()) {
				systemSim.addCamera(cameraSim, robotToCamTransform);
			}
		}

		public Optional<PhotonPipelineResult> getBestResult() {
			if (resultsList.isEmpty()) {
				return Optional.empty();
			}

			PhotonPipelineResult bestResult = resultsList.get(0);
			double ambiguity = bestResult.getBestTarget().getPoseAmbiguity();
			for (PhotonPipelineResult result : resultsList) {
				double currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
				if (currentAmbiguity < ambiguity && currentAmbiguity > 0) {
					bestResult = result;
					ambiguity = currentAmbiguity;
				}
			}
			return Optional.of(bestResult);
		}

		public Optional<PhotonPipelineResult> getLatestResult() {
			return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
		}

		public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
			return estimatedRobotPose;
		}

    }

	// Helper you can use in your drive subsystem:
	public static Optional<PhotonPipelineResult> getBestAcrossAllCameras() {
		return Arrays.stream(Cameras.values())
				.map(Cameras::getLatestResult)
				.flatMap(Optional::stream)
				.filter(PhotonPipelineResult::hasTargets)
				.min((a, b) -> Double.compare(
						a.getBestTarget().getPoseAmbiguity(),
						b.getBestTarget().getPoseAmbiguity()));
	}
}
