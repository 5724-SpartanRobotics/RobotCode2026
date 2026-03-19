// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Notifier;
import frc.lib.NopSubsystemBase;
import frc.robot.info.RobotMode;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.VisionFrame;

public class VisionSubsystem extends NopSubsystemBase {
	private static final Frequency kFrequency = Units.Hertz.of(25);

	private final Notifier visionThread;

	private VisionConsumer consumer;
	private final VisionIO[] io;
	private final VisionIO.VisionIOInputs[] inputs;
	private final Alert[] disconnectedAlerts;

	private final AtomicReference<VisionFrame[]> latestFrames = new AtomicReference<>();

	private VisionSubsystem(VisionConsumer consumer, VisionIO... io) {
		this.consumer = consumer;
		this.io = io;

		// Initialize inputs
		this.inputs = new VisionIO.VisionIOInputs[io.length];
		for (int i = 0; i < inputs.length; i++) {
			inputs[i] = new VisionIO.VisionIOInputs();
		}

		// Initialize disconnected alerts
		this.disconnectedAlerts = new Alert[io.length];
		for (int i = 0; i < inputs.length; i++) {
			disconnectedAlerts[i] = new Alert(
				"Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
		}

		ensureDriveSubsystem();

		visionThread = new Notifier(this::updateLoop);
		visionThread.setName("VisionThread");
		visionThread.startPeriodic(kFrequency);
	}

	private void ensureDriveSubsystem() {
		DriveSubsystem.getInstance();
	}

	private static final class Holder {
		private static final VisionSubsystem INSTANCE = switch (RobotMode.get()) {
			case Real -> new VisionSubsystem(
				DriveSubsystem.getInstance()::addVisionMeasurement,
				new VisionIO_Photon(
					frc.robot.info.constants.VisionConstants.CameraConfigurations.Front),
				new VisionIO_Photon(
					frc.robot.info.constants.VisionConstants.CameraConfigurations.Back),
				new VisionIO_Photon(
					frc.robot.info.constants.VisionConstants.CameraConfigurations.Right));

			case Simulation -> new VisionSubsystem(
				DriveSubsystem.getInstance()::addVisionMeasurement,
				new VisionIO_PhotonSim(
					frc.robot.info.constants.VisionConstants.CameraConfigurations.Front,
					DriveSubsystem.getInstance()::getPose),
				new VisionIO_PhotonSim(
					frc.robot.info.constants.VisionConstants.CameraConfigurations.Back,
					DriveSubsystem.getInstance()::getPose),
				new VisionIO_PhotonSim(
					frc.robot.info.constants.VisionConstants.CameraConfigurations.Right,
					DriveSubsystem.getInstance()::getPose));

			default -> new VisionSubsystem(
				DriveSubsystem.getInstance()::addVisionMeasurement,
				new VisionIO() {
				}, new VisionIO() {
				}, new VisionIO() {
				});
		};
	}

	public static VisionSubsystem getInstance() {
		return Holder.INSTANCE;
	}

	@Override
	public void periodic() {
		VisionFrame[] frames = latestFrames.getAndSet(null);

		if (frames != null) {
			for (VisionFrame frame : frames) {
				int i = frame.cameraIndex;
				inputs[i] = frame.inputs;

				Logger.processInputs("Vision/Camera" + i, inputs[i]);
			}
		}
	}

	public void updateLoop() {
		// for (int i = 0; i < io.length; i++) {
		// io[i].updateInputs(inputs[i]);
		// Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
		// }
		VisionFrame[] frames = new VisionFrame[io.length];
		for (int i = 0; i < io.length; i++) {
			VisionIO.VisionIOInputs newInputs = new VisionIO.VisionIOInputs();
			io[i].updateInputs(newInputs);
			frames[i] = new VisionFrame(i, newInputs);
		}
		latestFrames.set(frames);

		// Initialize logging values
		List<Pose3d> allTagPoses = new LinkedList<>();
		List<Pose3d> allRobotPoses = new LinkedList<>();
		List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
		List<Pose3d> allRobotPosesRejected = new LinkedList<>();

		// Loop over cameras
		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
			// Update disconnected alert
			disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

			// Initialize logging values
			List<Pose3d> tagPoses = new LinkedList<>();
			List<Pose3d> robotPoses = new LinkedList<>();
			List<Pose3d> robotPosesAccepted = new LinkedList<>();
			List<Pose3d> robotPosesRejected = new LinkedList<>();

			// Add tag poses
			for (int tagId : inputs[cameraIndex].tagIds) {
				var tagPose = frc.robot.info.constants.VisionConstants.LAYOUT
					.getTagPose(tagId);
				if (tagPose.isPresent()) {
					tagPoses.add(tagPose.get());
				}
			}

			// Loop over pose observations
			for (var observation : inputs[cameraIndex].poseObservations) {
				// Check whether to reject pose
				boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
					|| (observation.tagCount() == 1
						&& observation
							.ambiguity() > frc.robot.info.constants.VisionConstants.MAX_AMBIGUITY)
					// Cannot be high ambiguity

					|| Math.abs(
						observation.pose()
							.getZ()) > frc.robot.info.constants.VisionConstants.MAX_Z_ERROR
					// Must have realistic Z coordinate

					// Must be within the field boundaries
					|| observation.pose().getX() < 0.0
					|| observation.pose()
						.getX() > frc.robot.info.constants.VisionConstants.LAYOUT
							.getFieldLength()
					|| observation.pose().getY() < 0.0
					|| observation.pose()
						.getY() > frc.robot.info.constants.VisionConstants.LAYOUT
							.getFieldWidth();

				// Add pose to log
				robotPoses.add(observation.pose());
				if (rejectPose) {
					robotPosesRejected.add(observation.pose());
				} else {
					robotPosesAccepted.add(observation.pose());
				}

				// Skip if rejected
				if (rejectPose) {
					continue;
				}

				// Calculate standard deviations
				double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0)
					/ observation.tagCount();
				double linearStdDev = frc.robot.info.constants.VisionConstants.LINEAR_STDEV_BASELINE
					* stdDevFactor;
				double angularStdDev = frc.robot.info.constants.VisionConstants.ANGULAR_STDEV_BASELINE
					* stdDevFactor;
				if (observation.type() == PoseObservationType.MEGATAG_2) {
					linearStdDev *= frc.robot.info.constants.VisionConstants.LINEAR_STDEB_MEGATAG2_FACTOR;
					angularStdDev *= frc.robot.info.constants.VisionConstants.ANGULAR_STDEV_MEGATAG2_FACTOR;
				}
				if (cameraIndex < frc.robot.info.constants.VisionConstants.CAMERA_STDEV_FACTORS.length) {
					linearStdDev *= frc.robot.info.constants.VisionConstants.CAMERA_STDEV_FACTORS[cameraIndex];
					angularStdDev *= frc.robot.info.constants.VisionConstants.CAMERA_STDEV_FACTORS[cameraIndex];
				}

				// Send vision observation
				consumer.accept(
					observation.pose().toPose2d(),
					observation.timestamp(),
					VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
			}

			// Log camera datadata
			Logger.recordOutput(
				"Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
				tagPoses.toArray(new Pose3d[tagPoses.size()]));
			Logger.recordOutput(
				"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
				robotPoses.toArray(new Pose3d[robotPoses.size()]));
			Logger.recordOutput(
				"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
				robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
			Logger.recordOutput(
				"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
				robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
			allTagPoses.addAll(tagPoses);
			allRobotPoses.addAll(robotPoses);
			allRobotPosesAccepted.addAll(robotPosesAccepted);
			allRobotPosesRejected.addAll(robotPosesRejected);
		}

		// Log summary data
		Logger.recordOutput(
			"Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
		Logger.recordOutput(
			"Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
		Logger.recordOutput(
			"Vision/Summary/RobotPosesAccepted",
			allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
		Logger.recordOutput(
			"Vision/Summary/RobotPosesRejected",
			allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
	}

	@FunctionalInterface
	public static interface VisionConsumer {
		public void accept(
			Pose2d visionRobotPoseMeters,
			double timestampSeconds,
			Matrix<N3, N1> visionMeasurementStdDevs);
	}
}