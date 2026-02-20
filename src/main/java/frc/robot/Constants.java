package frc.robot;

import java.io.File;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.util.Color;
import swervelib.math.Matter;

public final class Constants {
	public static final double TWO_PI = Math.PI * 2.0;
	public static final double HALF_PI = Math.PI / 2.0;
	public static final DebugLevel DEBUG_TRACE_LEVEL = DebugLevel.All;

		/**
	 * Checks if the alliance is red, defaults to false if alliance isn't available.
	 *
	 * @return true if the red alliance, false if blue. Defaults to false if none is available.
	 */
	public static boolean isRedAlliance() {
		var alliance = DriverStation.getAlliance();
		return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
	}

	public static Color getAllianceColor() {
		var alliance = DriverStation.getAlliance();
		if (alliance.isEmpty()) return Color.kWhite;
		return switch (alliance.get()) {
			case Red -> Color.kRed;
			case Blue -> Color.kBlue;
			default -> Color.kWhite;
		};
	}

	public static Color getInverseAllianceColor() {
		var alliance = DriverStation.getAlliance();
		if (alliance.isEmpty()) return Color.kWhite;
		return switch (alliance.get()) {
			case Red -> Color.kBlue;
			case Blue -> Color.kRed;
			default -> Color.kWhite;
		};
	}

	public static final class CanId {
		// https://docs.google.com/spreadsheets/d/1t8Ids1RzCOn0vlFqbZAr0mgEd12h2sJ_fCfxJkntMR0/edit?gid=0#gid=0
		public static final int ROBORIO = 0;
		public static final int PDH = 1;
		public static final int PIGEON2 = 2;

		public static final int FL_DRIVE = 3;
		public static final int FL_ENCODER = 4;
		public static final int FL_TURN = 5;
		public static final int BL_DRIVE = 6;
		public static final int BL_ENCODER = 7;
		public static final int BL_TURN = 8;
		public static final int BR_DRIVE = 9;
		public static final int BR_ENCODER = 10;
		public static final int BR_TURN = 11;
		public static final int FR_DRIVE = 12;
		public static final int FR_ENCODER = 13;
		public static final int FR_TURN = 14;

		public static final int INTAKE_UPPER = 20;
		public static final int INTAKE_LOWER = 21;
		public static final int ARM_LEFT_MASTER = 22;
		public static final int ARM_RIGHT_SLAVE = 23;

		public static final int CLIMBER = 25;

		public static final int INDEXER = 27;
		public static final int SHOOTER = 28;
		public static final int SHOOTER_UPPER_FEED = 29;
	}

	public static final class Robot {
		// TODO: These values will all need updated once the robot is finished.
		public static final Mass MASS = Units.Pounds.of(110);
		public static final MomentOfInertia MOMENT_OF_INERTIA =
			Units.KilogramSquareMeters.of(16.5);
		public static final LinearVelocity MAX_LINEAR_VELOCITY =
			Units.FeetPerSecond.of(14.5); // about 4.42m/s
		public static final LinearAcceleration MAX_LINEAR_ACCELERATION =
			Units.FeetPerSecondPerSecond.of(13.12); // about 4m/s^2

		public static final double DEFAULT_SPEED_MOD = 0.425;
	}

	public static final class Drive {
		public static final File SWERVE_CONFIG =
			new File(Filesystem.getDeployDirectory(), "swerve");
		public static final Time SWERVE_LOOP_TIME = Units.Milliseconds.of(20).plus(Units.Milliseconds.of(110));
		public static final Time WHEEL_LOCK_TIME = Units.Seconds.of(10);
		// TODO: Calculate the CoM of the robot once it's finished
		public static final Matter CHASSIS = new Matter(new Translation3d(
			Units.Millimeters.of(0).in(Units.Meters),
			Units.Millimeters.of(0).in(Units.Meters),
			Units.Millimeters.of(8).in(Units.Meters)
		), Robot.MASS.in(Units.Kilograms));

		public static final class Wheel {
			public static final Distance RADIUS = Units.Inches.of(2.0);
			public static final double COF = 1.2;
			public static final Distance X_FROM_CENTER = Units.Inches.of(11.96);
			public static final Distance Y_FROM_CENTER = Units.Inches.of(11.016);
		}

		public static final class SwerveModuleOffsets {
			public static final Translation2d FL = new Translation2d(
				Wheel.X_FROM_CENTER.in(Units.Meters), Wheel.Y_FROM_CENTER.in(Units.Meters)
			);
			public static final Translation2d BL = new Translation2d(
				-1.0 * Wheel.X_FROM_CENTER.in(Units.Meters), Wheel.Y_FROM_CENTER.in(Units.Meters)
			);
			public static final Translation2d BR = new Translation2d(
				-1.0 * Wheel.X_FROM_CENTER.in(Units.Meters), -1.0 * Wheel.Y_FROM_CENTER.in(Units.Meters)
			);
			public static final Translation2d FR = new Translation2d(
				Wheel.X_FROM_CENTER.in(Units.Meters), -1.0 * Wheel.Y_FROM_CENTER.in(Units.Meters)
			);
		}
	}

	public static final class Controller {
		public static final double DRIVER_DEADBAND_XY = 0.025;
		public static final double DRIVER_DEADBAND_Z = 0.35 / 1.5;
		public static final double DRIVER_TURN_CONSTANT = TWO_PI;

		public static final class DriverMap {
			public static final int SPEEDMOD_MID = 1; // trigger
			public static final int SPEEDMOD_MAX = 2; // thumb button
			public static final int ZERO_GYRO = 7;
			public static final int DRIVE_TO_POSE = 11;
			public static final int CENTER_SWERVES = 10;
			public static final int RESET_ODOMETRY = 8;
			public static final int RESET_ODOMETRY_FLIPPED = 5;
			public static final int DRIVE_TO_INITIAL_POSE = 9;
			public static final int TOGGLE_NOTIFICATION = 12;
		}
		
		public static final class OperatorMap {}
	}

	public static final class Vision {
		public static final AprilTagFieldLayout FIELD_LAYOUT =
			AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
		public static final double MAX_AMBIGUITY = 0.25;
		public static final Transform3d CAMERA_TO_ROBOT = new Transform3d();
	}

	public static final class Field {
		public static final Translation2d RED_HUB_CENTER = new Translation2d(Units.Meters.of(11.91515064239502), Units.Meters.of(4.038271903991699));
		public static final Translation2d BLUE_HUB_CENTER = new Translation2d(Units.Meters.of(4.625269412994385), Units.Meters.of(4.039185523986816));
	}

	public static final class LED {
        public static final int PORT = 0;
        public static final int LED_COUNT = 300;

		public static final double STRIP_BITS_PER_PIXEL_0 = 18.0;
    	public static final double STRIP_BITS_PER_PIXEL_1 = 13.0;
	}

	public static final class Motors {
		public static final AngularVelocity REDLINE_MAX_VELOCITY = Units.RPM.of(21020);
		public static final AngularVelocity VORTEX_MAX_VELOCITY = Units.RPM.of(6784);
		public static final AngularVelocity NEO_MAX_VELOCITY = Units.RPM.of(5676);
		
		public static final double NEO_COUNTS_PER_REVOLUTION = 1.0;

		public static final Current KRAKENX60_STALL_CURRENT = Units.Amps.of(233);
	}

	public static final class Intake {
		public static final Dimensionless SPEED = Units.Percent.of(50);
		public static final double UPPER_GEAR_RATIO = 5; // 5:1
		public static final Distance UPPER_WHEEL_CURCUMFERENCE = Units.Inches.of(4).times(Math.PI);
		public static final Distance LOWER_WHEEL_CURCUMFERENCE = Units.Inches.of(2.25).times(Math.PI);
		public static final double LOWER_GEAR_RATIO = 4; // 4:1

		public static final class Arm {
			public static final double GEAR_RATIO = 5; // 5:1
			public static final AngularVelocity SETPOINT_RAMP_RATE = Units.DegreesPerSecond.of(60);
			public static final Angle MIN_ROTATION = Units.Degrees.of(0);
			public static final Angle MAX_ROTATION = Units.Degrees.of(100);
		}
	}

	public static final class Climber {
		public static final double GEAR_RATIO = 12;
		public static final Current MAX_CURRENT = Units.Amps.of(40);
	}

	public static enum DebugLevel {
		Off,
		All,
		Autonomous,
		Climb,
		Drive,
		Intake,
		LED,
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
