package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

public class VisionSubsystem extends SubsystemBase {
	public static class TagMeasurement {
		public final int tagId;
		public final double distanceFeet;
		public final double angleDegrees;
		public final int cameraId;
		public final boolean valid;

		public TagMeasurement(
            int tagId,
            double distanceFeet,
            double angleDegrees,
            int cameraId,
            boolean valid
        ) {
			this.tagId = tagId;
			this.distanceFeet = distanceFeet;
			this.angleDegrees = angleDegrees;
			this.cameraId = cameraId;
			this.valid = valid;
		}
	}

	private final NetworkTable visionTable;

	// Adjust this to however many tags you actually use
	private static final int MAX_TAG_ID = 32;

	public VisionSubsystem() {
		visionTable = NetworkTableInstance.getDefault().getTable("Vision");
	}

	/** Tell the Pi which tag we care about most (for its own logic). */
	public void setWantedTag(int tagId) {
		visionTable.getEntry("wantedTag").setNumber(tagId);
	}

	/** Get a single tag measurement by ID, or null if invalid/not present. */
	public TagMeasurement getTagMeasurement(int tagId) {
		double[] arr = visionTable.getEntry("aprilTag" + tagId).getDoubleArray(new double[0]);
		if (arr.length < 4) {
			return null;
		}
		double distanceFeet = arr[0];
		double angleDegrees = arr[1];
		int cameraId = (int) arr[2];
		boolean valid = ((int) arr[3]) == 1;
		if (!valid) {
			return null;
		}
		return new TagMeasurement(tagId, distanceFeet, angleDegrees, cameraId, true);
	}

	/** Get all currently valid tag measurements. */
	public List<TagMeasurement> getAllMeasurements() {
		List<TagMeasurement> list = new ArrayList<>();
		for (int id = 0; id <= MAX_TAG_ID; id++) {
			TagMeasurement m = getTagMeasurement(id);
			if (m != null) {
				list.add(m);
			}
		}
		return list;
	}
}
