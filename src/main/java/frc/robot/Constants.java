package frc.robot;

public final class Constants {
	public static final double TwoPI = Math.PI * 2.0;
	public static final double HalfPI = Math.PI / 2.0;
	public static final DebugLevel DebugTraceLevel = null;

	public static final class CanId {}

	public static final class Robot {}

	public static final class Drive {}

	public static final class Controller {}

	public static enum DebugLevel {
		Off,
		All,
		Autonomous,
		Swerve,
		ArmRotate,
		Elevator,
		Algae,
		Claw,
		Wrist,
		Vision;

        /**
		 * Check to see if {@link DebugTraceLevel} is set to All.
		 * @return Whether the DebugTraceLevel satisfies the level All
		 */
		public static boolean isAll() {
			return DebugTraceLevel == All;
		}

        /**
		 * Check to see if {@link DebugTraceLevel} is set to any non-Off value.
		 * @return Whether the DebugTraceLevel satisfies the level that is not Off
		 */
		public static boolean isAny() {
			return DebugTraceLevel != Off;
		}

		/**
		 * Check to see if {@link DebugTraceLevel} is set to the provided value.
		 * @param level DebugLevel to check against
		 * @return Whether the DebugTraceLevel satisfies the given level
		 */
		public static boolean is(DebugLevel level) {
			return DebugTraceLevel == level;
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
			return is(level) || DebugTraceLevel == All;
		}
	}

    public static boolean isBeanDebug() {
        // Code from ChatGPT. It sees if a debugger is attached by looking
        // at the arguements passed to the JVM when it was started.
        return java.lang.management.ManagementFactory.getRuntimeMXBean()
            .getInputArguments().toString().contains("-agentlib:jdwp");
    }
}
