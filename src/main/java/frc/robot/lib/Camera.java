/* Camera.java */

package frc.robot.lib;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.VisionSubsystem;

public class Camera {
	// should be about 20ms, we should be faster (hence /3)
	private static final long kNominalSleepMs = RobotBase.isSimulation()
		? (1)
		: (Constants.getPeriod() / 3);

	public final Alert latencyAlert;
	public final PhotonCamera camera;
	public final PhotonPoseEstimator poseEstimator;
	// private final Matrix<N3, N1> singleTagStdDevs;
	// private final Matrix<N3, N1> multiTagStdDevs;
	private final Transform3d robotToCamTransform;
	public Matrix<N3, N1> curStdDevs;
	public PhotonCameraSim cameraSim;
	public final String friendlyName;

	public final Object lock = new Object();
	private volatile Optional<EstimatedRobotPose> latestThreadedEstimate = Optional.empty();
	private volatile Matrix<N3, N1> latestStdDevs;
	private Thread workerThread;
	private double lastTimestamp = -1;

	public Camera(
		String name,
		Rotation3d robotToCamRotation,
		Translation3d robotToCamTranslation,
		Matrix<N3, N1> singleTagStdDevs,
		Matrix<N3, N1> multiTagStdDevsMatrix) {

		if (name.toLowerCase().equals("_note")) {
			throw new IllegalArgumentException(
				"The Camera cannot be named \"_note\" (case insensitive)!");
		}

		friendlyName = name;
		latencyAlert = new Alert(
			"'" + name + "' Camera is experiencing high latency.",
			AlertType.kWarning);

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

		if (Constants.DebugLevel.isOrAll(Constants.DebugLevel.Vision))
			SmartDashboard.putString(
				"VisionSubsystemCameras/_Note",
				"EstPose is a Pose3d of the vision estimate, or all zeros/empty if none.");

		configureThread();
	}

	private void configureThread() {
		workerThread = new Thread(() -> {
			while (true) {
				for (var result : camera.getAllUnreadResults()) {
					if (result.getTimestampSeconds() <= lastTimestamp)
						continue;
					lastTimestamp = result.getTimestampSeconds();

					Optional<EstimatedRobotPose> est = poseEstimator
						.estimateCoprocMultiTagPose(result);
					if (est.isEmpty()) {
						est = poseEstimator.estimateLowestAmbiguityPose(result);
					}

					Matrix<N3, N1> std = VisionSubsystem.updateEstimationStdDevs(this, est,
						result.getTargets());

					synchronized (lock) {
						latestThreadedEstimate = est;
						latestStdDevs = std;
					}
				}

				try {
					Thread.sleep(kNominalSleepMs);
				} catch (InterruptedException e) {
				}
			}
		});
		workerThread.setDaemon(true);
		workerThread.start();
	}

	public void addToVisionSim(VisionSystemSim systemSim) {
		if (RobotBase.isSimulation()) {
			systemSim.addCamera(cameraSim, robotToCamTransform);
		}
	}

	public Optional<EstimatedRobotPose> getLatestThreadedEstimate() {
		return latestThreadedEstimate;
	}

	public Matrix<N3, N1> getLatestStdDevs() {
		return latestStdDevs;
	}
}
