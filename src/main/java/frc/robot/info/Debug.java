package frc.robot.info;

public class Debug {
	protected static final DebugLevel DEBUG_TRACE_LEVEL = DebugLevel.All;

	public static enum DebugLevel {
		Off, All, Autonomous, Climb, Drive, Indexer, Intake, Led, Shooter, Vision;

		/**
		 * Check to see if {@link DebugTraceLevel} is set to All.
		 *
		 * @return Whether the DebugTraceLevel satisfies the level All
		 */
		public static boolean isAll() {
			return DEBUG_TRACE_LEVEL == All;
		}

		/**
		 * Check to see if {@link DebugTraceLevel} is set to any non-Off value.
		 *
		 * @return Whether the DebugTraceLevel satisfies the level that is not Off
		 */
		public static boolean isAny() {
			return DEBUG_TRACE_LEVEL != Off;
		}

		/**
		 * Check to see if {@link DebugTraceLevel} is set to the provided value.
		 *
		 * @param level
		 *            DebugLevel to check against
		 * @return Whether the DebugTraceLevel satisfies the given level
		 */
		public static boolean is(DebugLevel level) {
			return DEBUG_TRACE_LEVEL == level;
		}

		/**
		 * Check to see if {@link DebugTraceLevel} is set to any one of the provided values.
		 *
		 * @param level
		 *            Spread of <code>DebugLevel</code>s to check against
		 * @return Whether the DebugTraceLevel satisfies any one of the given levels
		 */
		public static boolean isAnyOf(DebugLevel... level) {
			for (DebugLevel l : level) {
				if (is(l))
					return true;
			}
			return false;
		}

		/**
		 * Check to see if {@link DebugTraceLevel} is set to either the provided value or All.
		 *
		 * @param level
		 *            DebugLevel to check against
		 * @return Whether the DebugTraceLevel satisfies the given level or All
		 */
		public static boolean isOrAll(DebugLevel level) {
			return is(level) || DEBUG_TRACE_LEVEL == All;
		}

		public static DebugLevel get() {
			return DEBUG_TRACE_LEVEL;
		}
	}

	public static boolean isBeanDebug() {
		// Code from ChatGPT. It sees if a debugger is attached by looking
		// at the arguements passed to the JVM when it was started.
		return java.lang.management.ManagementFactory.getRuntimeMXBean()
			.getInputArguments().toString().contains("-agentlib:jdwp");
	}
}
