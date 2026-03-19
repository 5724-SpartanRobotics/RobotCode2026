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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {
	@AutoLog
	public static class VisionIOInputs implements LoggableInputs {
		public boolean connected = false;
		public TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(),
			new Rotation2d());
		public PoseObservation[] poseObservations = new PoseObservation[0];
		public int[] tagIds = new int[0];

		@Override
		public void toLog(LogTable table) {
			table.put("Connected", connected);

			// TargetObservation (assuming two Rotation2d fields: tx, ty)
			table.put("Target/Tx", latestTargetObservation.tx.getRadians());
			table.put("Target/Ty", latestTargetObservation.ty.getRadians());

			// PoseObservations
			table.put("PoseObservationsCount", poseObservations.length);
			for (int i = 0; i < poseObservations.length; i++) {
				PoseObservation obs = poseObservations[i];
				LogTable sub = table.getSubtable("PoseObservations/" + i);

				sub.put("Timestamp", obs.timestamp());

				// Pose3d → translation
				sub.put("Pose/X", obs.pose().getX());
				sub.put("Pose/Y", obs.pose().getY());
				sub.put("Pose/Z", obs.pose().getZ());

				// Pose3d → rotation (quaternion)
				var q = obs.pose().getRotation().getQuaternion();
				sub.put("Pose/Rot/W", q.getW());
				sub.put("Pose/Rot/X", q.getX());
				sub.put("Pose/Rot/Y", q.getY());
				sub.put("Pose/Rot/Z", q.getZ());

				sub.put("Ambiguity", obs.ambiguity());
				sub.put("TagCount", obs.tagCount());
				sub.put("AverageTagDistance", obs.averageTagDistance());

				sub.put("Type", obs.type().name());
			}

			// Tag IDs
			table.put("TagIds", tagIds);
		}

		@Override
		public void fromLog(LogTable table) {
			connected = table.get("Connected", false);

			// TargetObservation (assuming fields: tx, ty)
			double tx = table.get("Target/Tx", 0.0);
			double ty = table.get("Target/Ty", 0.0);
			latestTargetObservation = new TargetObservation(new Rotation2d(tx), new Rotation2d(ty));

			// PoseObservations
			int count = table.get("PoseObservationsCount", 0);
			poseObservations = new PoseObservation[count];

			for (int i = 0; i < count; i++) {
				LogTable sub = table.getSubtable("PoseObservations/" + i);

				double timestamp = sub.get("Timestamp", 0.0);

				// Translation
				double x = sub.get("Pose/X", 0.0);
				double y = sub.get("Pose/Y", 0.0);
				double z = sub.get("Pose/Z", 0.0);

				// Rotation (quaternion)
				double qw = sub.get("Pose/Rot/W", 1.0);
				double qx = sub.get("Pose/Rot/X", 0.0);
				double qy = sub.get("Pose/Rot/Y", 0.0);
				double qz = sub.get("Pose/Rot/Z", 0.0);

				Rotation3d rotation = new Rotation3d(new Quaternion(qw, qx, qy, qz));
				Pose3d pose = new Pose3d(x, y, z, rotation);

				double ambiguity = sub.get("Ambiguity", 0.0);
				int tagCount = sub.get("TagCount", 0);
				double avgDist = sub.get("AverageTagDistance", 0.0);

				String typeStr = sub.get("Type", PoseObservationType.UNKNOWN.name());
				PoseObservationType type = PoseObservationType.valueOf(typeStr);

				poseObservations[i] = new PoseObservation(timestamp, pose, ambiguity, tagCount,
					avgDist, type);
			}

			// Tag IDs
			tagIds = table.get("TagIds", new int[0]);
		}
	}

	/** Represents the angle to a simple target, not used for pose estimation. */
	public static record TargetObservation(Rotation2d tx, Rotation2d ty) {
	}

	/** Represents a robot pose sample used for pose estimation. */
	public static record PoseObservation(
		double timestamp,
		Pose3d pose,
		double ambiguity,
		int tagCount,
		double averageTagDistance,
		PoseObservationType type) {

		public void toLog(LogTable table) {
			table.put("Timestamp", timestamp);

			// Pose3d → translation
			table.put("Pose/X", pose.getX());
			table.put("Pose/Y", pose.getY());
			table.put("Pose/Z", pose.getZ());

			// Pose3d → rotation (quaternion is safest)
			table.put("Pose/Rot/W", pose.getRotation().getQuaternion().getW());
			table.put("Pose/Rot/X", pose.getRotation().getQuaternion().getX());
			table.put("Pose/Rot/Y", pose.getRotation().getQuaternion().getY());
			table.put("Pose/Rot/Z", pose.getRotation().getQuaternion().getZ());

			table.put("Ambiguity", ambiguity);
			table.put("TagCount", tagCount);
			table.put("AverageTagDistance", averageTagDistance);

			// Enum → String (more stable than ordinal)
			table.put("Type", type.name());
		}
	}

	public static enum PoseObservationType {
		MEGATAG_1, MEGATAG_2, PHOTONVISION, UNKNOWN
	}

	public static class VisionFrame {
		public final int cameraIndex;
		public final VisionIO.VisionIOInputs inputs;

		public VisionFrame(int cameraIndex, VisionIO.VisionIOInputs inputs) {
			this.cameraIndex = cameraIndex;
			this.inputs = inputs;
		}
	}

	public default void updateInputs(VisionIOInputs inputs) {
	}
}