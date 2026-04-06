// Adapted from 2106 Junkyard Dogs

package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.NopSubsystemBase;
import frc.robot.info.Alliance;

public class GameTimerSubsystem extends NopSubsystemBase {
	private static final double[] thresholds = {
		130, 105, 80, 55, 30
	};

	private final Notifier thread;

	private final AtomicReference<Double> countdown = new AtomicReference<>(0.0);
	private final AtomicBoolean isHubActive = new AtomicBoolean(false);
	private final AtomicBoolean shift1Active = new AtomicBoolean(false);

	private GameTimerSubsystem() {
		thread = new Notifier(this::updateLoop);
		thread.setName("GameTimer");
	}

	private static final class Holder {
		private static final GameTimerSubsystem INSTANCE = new GameTimerSubsystem();
	}

	public static synchronized GameTimerSubsystem getInstance() {
		return Holder.INSTANCE;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType(this.getClass().getName());
		builder.addBooleanProperty("Hub Active", () -> isHubActive.get(), null);
		builder.addDoubleProperty("Countdown Until Hub Active", () -> countdown.get(), null);
	}

	@Override
	public void periodic() {
		SmartDashboard.putData(this);
	}

	public void updateLoop() {
		isHubActive.set(isHubActive());
		countdown.set(computeCountdown());
	}

	public void startTimer() {
		thread.startPeriodic(Units.Hertz.of(100));
	}

	public static double getMatchTime() {
		return DriverStation.getMatchTime();
	}

	private double computeCountdown() {
		if (DriverStation.isAutonomousEnabled())
			return 0;

		double matchTime = getMatchTime();
		shift1Active.set(computeShift1Active());
		boolean stateNow = isHubActiveAtMatchTime(matchTime);

		for (double t : thresholds) {
			if (matchTime > t) {
				boolean stateAfter = isHubActiveAtMatchTime(t - 0.5);
				if (stateAfter != stateNow) {
					return matchTime - t;
				}
			}
		}

		return 0;
	}

	private boolean computeShift1Active() {
		var alliance = Alliance.getDsAlliance();
		if (alliance.isEmpty())
			return false;

		var gameData = DriverStation.getGameSpecificMessage();
		if (gameData.isEmpty())
			return true; // assume active hub

		boolean redInactiveFirst = switch (gameData.charAt(0)) {
			case 'R' -> true;
			case 'B' -> false;
			default -> {
				yield false;
			}
		};

		return switch (alliance.get()) {
			case Red -> !redInactiveFirst;
			case Blue -> redInactiveFirst;
		};
	}

	private boolean isHubActiveAtMatchTime(double time) {
		if (time > 130)
			return true;
		if (time > 105)
			return shift1Active.get();
		if (time > 80)
			return !shift1Active.get();
		if (time > 55)
			return shift1Active.get();
		if (time > 30)
			return !shift1Active.get();
		return true;
	}

	private boolean isHubActive() {
		var alliance = Alliance.getDsAlliance();
		if (alliance.isEmpty())
			return false;
		if (DriverStation.isAutonomousEnabled())
			return true;
		if (!DriverStation.isTeleopEnabled())
			return false;

		double matchTime = getMatchTime();
		String gameData = DriverStation.getGameSpecificMessage();
		if (gameData.isEmpty())
			return true;

		shift1Active.set(computeShift1Active());
		return isHubActiveAtMatchTime(matchTime);
	}
}
