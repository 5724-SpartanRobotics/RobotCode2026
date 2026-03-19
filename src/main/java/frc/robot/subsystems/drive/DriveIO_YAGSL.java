package frc.robot.subsystems.drive;

import frc.robot.info.RobotMode;
import swervelib.SwerveDrive;

public class DriveIO_YAGSL implements DriveIO {
	private final SwerveDrive drive;

	public DriveIO_YAGSL(SwerveDrive drive) {
		this.drive = drive;
	}

	@Override
	public void updateInputs(DriveIOInputs inputs) {
		inputs.gyroYawRad = drive.getYaw().getRadians();

		var states = drive.getStates(); // SwerveModuleState[]
		inputs.moduleStates = new double[states.length * 2];
		for (int i = 0; i < states.length; i++) {
			inputs.moduleStates[i * 2] = states[i].speedMetersPerSecond;
			inputs.moduleStates[i * 2 + 1] = states[i].angle.getRadians();
		}

		var positions = drive.getModulePositions(); // SwerveModulePosition[]
		inputs.modulePositions = new double[positions.length * 2];
		for (int i = 0; i < positions.length; i++) {
			inputs.modulePositions[i * 2] = positions[i].distanceMeters;
			inputs.modulePositions[i * 2 + 1] = positions[i].angle.getRadians();
		}

		inputs.pose = switch (RobotMode.get()) {
			case Simulation -> drive.getSimulationDriveTrainPose().get();
			default -> drive.getPose();
		};
	}
}