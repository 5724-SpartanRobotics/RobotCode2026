package frc.robot.info;

import edu.wpi.first.wpilibj.RobotBase;

public enum RobotMode {
	Real, Simulation, Replay;

	public static RobotMode get() {
		return RobotBase.isReal() ? Real : Simulation;
	}

	public static boolean is(RobotMode other) {
		return get() == other;
	}
}
