package frc.robot.subsystems.shooter;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NopSubsystemBase;
import frc.robot.info.Debug;
import frc.robot.info.Period;
import frc.robot.info.constants.ShooterConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;

public class ShooterSubsystem extends NopSubsystemBase {
	private final ShooterFlywheel m_flywheel;

	private AtomicBoolean m_enable = new AtomicBoolean(false);
	private AtomicBoolean m_reverse = new AtomicBoolean(false);

	public AtomicReference<Distance> hypotenuseToAllianceHub = new AtomicReference<>(
		Units.Meters.of(0));
	public double flywheelSpeedMod = ShooterConstants.DEFAULT_FLYWHEEL_SPEEDMOD;

	private ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();

	private final LinearFilter distanceFilter = LinearFilter.singlePoleIIR(0.1, Period.getPeriod());
	private final InterpolatingDoubleTreeMap speedMap_MetersToRPM = new InterpolatingDoubleTreeMap();
	private double filteredSetpointRPM = 0;

	private static final LoggedNetworkNumber kA = new LoggedNetworkNumber("/Tuning/Shooter/A",
		ShooterConstants.SHOOTER_RPM_CURVATURE.in(Units.RPM.per(Units.Meter.per(Units.Meter))));

	private static final LoggedNetworkNumber kB = new LoggedNetworkNumber("/Tuning/Shooter/B",
		ShooterConstants.SHOOTER_RPM_SLOPE.in(Units.RPM.per(Units.Meter)));

	private static final LoggedNetworkNumber kC = new LoggedNetworkNumber("/Tuning/Shooter/C",
		ShooterConstants.SHOOTER_RPM_INTERCEPT.in(Units.RPM));

	private ShooterSubsystem() {
		m_flywheel = ShooterFlywheel.getInstance();

		ShooterConstants.SPEED_MAP
			.forEach((dist, speed) -> speedMap_MetersToRPM.put(dist.in(Units.Meters),
				speed.in(Units.RPM)));
	}

	private static final class Holder {
		private static final ShooterSubsystem INSTANCE = new ShooterSubsystem();
	}

	public static synchronized ShooterSubsystem getInstance() {
		return Holder.INSTANCE;
	}

	@Override
	public void periodic() {
		var setpointVelocity = setMotorVelocities();

		m_flywheel.periodic();

		log(setpointVelocity);

		if (Debug.DebugLevel.isOrAll(Debug.DebugLevel.Shooter))
			SmartDashboard.putData(this);

		Logger.processInputs("Shooter", inputs);

		SmartDashboard.putData(this);
	}

	private void log(AngularVelocity setpointVelocity) {
		inputs.enabledFlywheel = m_enable.get();
		inputs.reversed = m_reverse.get();

		inputs.distanceMeters = hypotenuseToAllianceHub.get().in(Units.Meters);
		inputs.flywheelSpeedMod = flywheelSpeedMod;

		// You'll need to store this when you calculate it
		inputs.targetFlywheelRPM = setpointVelocity.in(Units.RPM);
	}

	@Override
	public void simulationPeriodic() {
		m_flywheel.simulationPeriodic();
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType(this.getClass().getName());
		m_flywheel.initSendable(builder);
		builder.addDoubleProperty("Flywheel SpeedMod", () -> flywheelSpeedMod,
			(newMod) -> flywheelSpeedMod = newMod);
		builder.addBooleanProperty("Shooter Enabled", () -> m_enable.get(), null);
		builder.addBooleanProperty("Belt Reversed", () -> m_reverse.get(), null);
		if (Debug.DebugLevel.isOrAll(Debug.DebugLevel.Shooter)) {
			builder.addDoubleProperty("Distance from HUB Meters",
				() -> hypotenuseToAllianceHub.get().in(Units.Meters), null);
			builder.addDoubleArrayProperty("Meters -> RPM Map from quadratic", () -> new double[]{
				1.0, calculateShooterSpeedFromRobotDistance_quadratic(1.0).in(Units.RPM),
				2.0, calculateShooterSpeedFromRobotDistance_quadratic(2.0).in(Units.RPM),
				2.5, calculateShooterSpeedFromRobotDistance_quadratic(2.5).in(Units.RPM),
				2.85, calculateShooterSpeedFromRobotDistance_quadratic(2.85).in(Units.RPM),
				3.0, calculateShooterSpeedFromRobotDistance_quadratic(3.0).in(Units.RPM),
				4.0, calculateShooterSpeedFromRobotDistance_quadratic(4.0).in(Units.RPM)
			}, null);
		}
	}

	public static double roundToNearest(double x, double k) {
		if (k == 0) {
			throw new IllegalArgumentException("k must not be 0");
		}
		return Math.round(x / k) * k;
	}

	@SuppressWarnings("unused")
	private AngularVelocity calculateShooterSpeedFromRobotDistance_idealPhysics() {
		Distance copy = hypotenuseToAllianceHub.get();
		copy = Units.Meters.of(1);
		double d = distanceFilter.calculate(copy.in(Units.Meters));
		double g = frc.robot.info.Math.g.in(Units.MetersPerSecondPerSecond);
		double v = Math.sqrt(
			(d * g)
				/
				Math.sin(2.0 * ShooterConstants.LAUNCH_ANGLE.in(Units.Radians))); // projectile
																					// motion range
																					// equation
		AngularVelocity omega = Units.RadiansPerSecond.of(
			v / ShooterConstants.FLYWHEEL_DIAMETER.div(2.0).in(Units.Meters)); // v/r
		// double lowVoltageMultiplier = RobotController.getBatteryVoltage()
		// / RobotConstants.NOMINAL_BATTERY_VOLTAGE.in(Units.Volts);
		// lowVoltageMultiplier = 1.01 * (1.0 / lowVoltageMultiplier);
		double lowVoltageMultiplier = 1.0;
		double rpm = omega
			.times(lowVoltageMultiplier)
			.times(Math.min(1.0, flywheelSpeedMod))
			.times(ShooterConstants.LAUNCH_VELOCITY_FUDGE_COEFF)
			.in(Units.RPM);
		double nearestK = roundToNearest(rpm, 100);
		return Units.RPM.of(nearestK);
	}

	private AngularVelocity calculateShooterSpeedFromRobotDistance_quadratic(
		double filteredDistMeters) {
		double distanceMeters = filteredDistMeters;
		double distancePow2 = Math.pow(distanceMeters, 2);

		// --- Shooter curve ---
		double targetRPM = kA.get() * distancePow2 +
			kB.get() * distanceMeters +
			kC.get();

		// --- Clamp ---
		targetRPM = MathUtil.clamp(
			targetRPM,
			ShooterConstants.MIN_SHOOTER_VELOCITY.in(Units.RPM),
			ShooterConstants.MAX_SHOOTER_VELOCITY.in(Units.RPM));

		// --- Rate limiting ---
		double delta = targetRPM - filteredSetpointRPM;
		delta = MathUtil.clamp(
			delta,
			-ShooterConstants.MAX_RPM_CHANGE_PER_LOOP,
			ShooterConstants.MAX_RPM_CHANGE_PER_LOOP);
		filteredSetpointRPM += delta;

		// --- Optional quantization ---
		if (ShooterConstants.RPM_STEP_SIZE.in(Units.RPM) > 0) {
			filteredSetpointRPM = Math
				.round(filteredSetpointRPM / ShooterConstants.RPM_STEP_SIZE.in(Units.RPM))
				* ShooterConstants.RPM_STEP_SIZE.in(Units.RPM);
		}

		// --- Global scaling ---
		filteredSetpointRPM *= ShooterConstants.LAUNCH_VELOCITY_FUDGE_COEFF;

		return Units.RPM.of(MathUtil.clamp(
			roundToNearest(filteredSetpointRPM, 50),
			ShooterConstants.MIN_SHOOTER_VELOCITY.in(Units.RPM),
			ShooterConstants.MAX_SHOOTER_VELOCITY.in(Units.RPM)));
	}

	private AngularVelocity calculateShooterSpeedFromRobotDistance() {
		double distMeters = distanceFilter.calculate(
			hypotenuseToAllianceHub.get().in(Units.Meters));
		double quadValueRPM = calculateShooterSpeedFromRobotDistance_quadratic(distMeters)
			.in(Units.RPM);
		try {
			double mapValueRPM = speedMap_MetersToRPM.get(distMeters);

			double weight = 0.8; // trust map more
			return Units.RPM.of(weight * mapValueRPM + (1.0 - weight) * quadValueRPM);
		} catch (NullPointerException e) {
			return Units.RPM.of(quadValueRPM);
		}
		// return Units.RPM.of(quadValueRPM);
	}

	private AngularVelocity setMotorVelocities() {
		var _enableFlywheel = m_enable.get();
		// var _enableFeeder = m_enableFeeder.get();

		if (!_enableFlywheel) {
			m_flywheel.disable();
		}

		if (!_enableFlywheel) {
			return Units.RPM.of(0);
		}

		var velocity = calculateShooterSpeedFromRobotDistance();
		if (_enableFlywheel && !m_reverse.get())
			m_flywheel.enable(velocity);
		else
			if (_enableFlywheel && m_reverse.get())
				m_flywheel.enableReverse(velocity);
		// AngularVelocity feederSetpoint = velocity.times(
		// ShooterConstants.FLYWHEEL_DIAMETER.div(ShooterConstants.FEEDER_PULLEY_DIAMETER))
		// .div(ShooterConstants.FEEDER_SPEED_COEFF).times(m_reverse.get() ? -1.0 : 1.0)
		// .times(ShooterConstants.FEEDER_GEAR_RATIO);
		return velocity;
	}

	private void enableFlywheel() {
		m_enable.set(true);
	}

	public void enableForward() {
		m_reverse.set(false);
		enableFlywheel();
	}

	public void enableReverse() {
		m_reverse.set(true);
		enableFlywheel();
		FeederSubsystem.getInstance().enableReverse();
	}

	public void disable() {
		disableFlywheel();
	}

	private void disableFlywheel() {
		m_enable.set(false);
	}

	public Command toggle() {
		return Commands.startEnd(
			() -> enableForward(),
			() -> disable(),
			this);
	}

	public Command toggleFeederReverse() {
		return Commands.startEnd(
			() -> enableReverse(),
			() -> disable(),
			this);
	}

	public Command runForCommand(Time duration) {
		return Commands.sequence(
			runOnce(this::enableForward),
			Commands.waitTime(duration),
			runOnce(this::disable));
	}

	public Command warmupFlywheelCommand() {
		return this.runOnce(this::enableFlywheel).withName("WarmupFlywheel").withTimeout(0.5);
	}

	public Command cooldownFlywheelCommand() {
		return this.runOnce(this::disableFlywheel).withName("CooldownFlywheel").withTimeout(0.5);
	}

	public Command enableForeverCommand() {
		return run(this::enableFlywheel).withName("EnableFlywheel");
	}

	public Command changeFlywheelSpeedMod(DoubleSupplier rawAxis) {
		return runOnce(() -> {
			double axis = rawAxis.getAsDouble() + 1.0; // default 1
			double newFlywheelSpeedMod = axis * 0.5;
			flywheelSpeedMod = newFlywheelSpeedMod;
		});
	}

	public Command resetFlywheelSpeedMod() {
		return runOnce(() -> flywheelSpeedMod = ShooterConstants.DEFAULT_FLYWHEEL_SPEEDMOD);
	}
}
