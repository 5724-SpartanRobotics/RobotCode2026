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

import java.awt.Desktop;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.info.constants.VisionConstants;
import frc.robot.info.constants.VisionConstants.CameraConfigurations;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIO_PhotonSim extends VisionIO_Photon {
	private static VisionSystemSim visionSim;
	private static int i = 0;

	private final Supplier<Pose2d> poseSupplier;
	private final PhotonCameraSim cameraSim;

	/**
	 * Creates a new VisionIOPhotonVisionSim.
	 *
	 * @param name
	 *            The name of the camera.
	 * @param poseSupplier
	 *            Supplier for the robot pose to use in simulation.
	 */
	public VisionIO_PhotonSim(
		CameraConfigurations cam, Supplier<Pose2d> poseSupplier) {
		super(cam);
		this.poseSupplier = poseSupplier;

		// Initialize vision sim
		if (visionSim == null) {
			visionSim = new VisionSystemSim("main");
			visionSim.addAprilTags(VisionConstants.LAYOUT);
		}

		// Add sim camera
		var cameraProperties = new SimCameraProperties();
		cameraProperties.setCalibration(640, 480, Rotation2d.fromDegrees(70));
		cameraProperties.setFPS(15);
		cameraSim = new PhotonCameraSim(camera, cameraProperties);
		visionSim.addCamera(cameraSim, robotToCamera);

		if (Desktop.isDesktopSupported()
			&& Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
			try {
				int port = 1182 + i;
				Desktop.getDesktop().browse(new URI("http://localhost:" + port + "/"));
			} catch (IOException | URISyntaxException e) {
				e.printStackTrace();
			}
		}
		i++;
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		visionSim.update(poseSupplier.get());
		super.updateInputs(inputs);
	}
}