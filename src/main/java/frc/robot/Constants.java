package frc.robot;

import java.io.File;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.Filesystem;

public final class Constants {
	public static final double TWO_PI = Math.PI * 2.0;
	public static final double HALF_PI = Math.PI / 2.0;
	public static final DebugLevel DEBUG_TRACE_LEVEL = null;

	public static final class CanId {}

	public static final class Robot {
		public static final Mass MASS = Units.Pounds.of(110);
		public static final MomentOfInertia MOMENT_OF_INERTIA = Units.KilogramSquareMeters.of(16.5);
		public static final LinearVelocity MAX_LINEAR_VELOCITY = Units.FeetPerSecond.of(14.5); // about 4.42m/s
		public static final LinearAcceleration MAX_LINEAR_ACCELERATION = Units.FeetPerSecondPerSecond.of(13.12); // about 4m/s^2
	}

	public static final class Drive {
		public static final File SWERVE_CONFIG = new File(Filesystem.getDeployDirectory(), "swerve");

		public static final class Wheel {
			public static final Distance RADIUS = Units.Inches.of(2.0);
			public static final double COF = 1.2;
			public static final Distance BASE = Units.Inches.of(20.75); // this might need updated?
			public static final Distance TRACK_WIDTH = Units.Inches.of(23.0); // this might need updated?
		}

		public static final class SwerveModuleOffsets {
			public static final Translation2d LF = new Translation2d(Wheel.BASE.in(Units.Meters) / 2.0, Wheel.TRACK_WIDTH.in(Units.Meters) / 2.0);
			public static final Translation2d RF = new Translation2d(Wheel.BASE.in(Units.Meters) / 2.0, -Wheel.TRACK_WIDTH.in(Units.Meters) / 2.0);
			public static final Translation2d LR = new Translation2d(-Wheel.BASE.in(Units.Meters) / 2.0, Wheel.TRACK_WIDTH.in(Units.Meters) / 2.0);
			public static final Translation2d RR = new Translation2d(-Wheel.BASE.in(Units.Meters) / 2.0, -Wheel.TRACK_WIDTH.in(Units.Meters) / 2.0);
		}
	}

	public static final class Controller {
		public static final double DEFAULT_DEADBAND = 0.5;

		public static final class DriverMap {}
		
		public static final class OperatorMap {}
	}

	public static final class Vision {
		public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
		public static final Transform3d CAMERA_TO_ROBOT = new Transform3d();
	}

	public static enum DebugLevel {
		Off,
		All,
		Autonomous,
		Drive,
		Vision;

		/**
		 * Check to see if {@link DebugTraceLevel} is set to All.
		 * @return Whether the DebugTraceLevel satisfies the level All
		 */
		public static boolean isAll() {
			return DEBUG_TRACE_LEVEL == All;
		}

		/**
		 * Check to see if {@link DebugTraceLevel} is set to any non-Off value.
		 * @return Whether the DebugTraceLevel satisfies the level that is not Off
		 */
		public static boolean isAny() {
			return DEBUG_TRACE_LEVEL != Off;
		}

		/**
		 * Check to see if {@link DebugTraceLevel} is set to the provided value.
		 * @param level DebugLevel to check against
		 * @return Whether the DebugTraceLevel satisfies the given level
		 */
		public static boolean is(DebugLevel level) {
			return DEBUG_TRACE_LEVEL == level;
		}
		
		/**
		 * Check to see if {@link DebugTraceLevel} is set to any one of the provided values.
		 * @param level Spread of <code>DebugLevel</code>s to check against 
		 * @return Whether the DebugTraceLevel satisfies any one of the given levels
		 */
		public static boolean isAnyOf(DebugLevel... level) {
			for (DebugLevel l : level) {
				if (is(l)) return true;
			}
			return false;
		}
		
		/**
		 * Check to see if {@link DebugTraceLevel} is set to either the provided value or All.
		 * @param level DebugLevel to check against
		 * @return Whether the DebugTraceLevel satisfies the given level or All
		 */
		public static boolean isOrAll(DebugLevel level) {
			return is(level) || DEBUG_TRACE_LEVEL == All;
		}
	}

	public static boolean isBeanDebug() {
		// Code from ChatGPT. It sees if a debugger is attached by looking
		// at the arguements passed to the JVM when it was started.
		return java.lang.management.ManagementFactory.getRuntimeMXBean()
			.getInputArguments().toString().contains("-agentlib:jdwp");
	}
}
