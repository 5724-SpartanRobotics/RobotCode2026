package frc.robot.lib;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

public class TalonFXWrapper {
	private final TalonFX m_motor;
	private final VelocityVoltage m_slot0VelocityVoltage = new VelocityVoltage(0).withSlot(0);

	private TalonFXSimState m_sim = null;
	private double _simVelocity = 0;
	private boolean isClosedLoop = false;

	public TalonFXWrapper(int canID) {
		m_motor = new TalonFX(canID);
		replaceSim();
	}

	public TalonFXWrapper withConfiguration(TalonFXConfiguration config) {
		m_motor.getConfigurator().apply(config);
		replaceSim();
		return this;
	}

	public TalonFXWrapper withNeutralMode(NeutralModeValue mode) {
		m_motor.setNeutralMode(mode);
		replaceSim();
		return this;
	}

	public TalonFXWrapper withSlot0Pidf(PIDFfRecord pidf) {
		m_motor.getConfigurator().apply(new Slot0Configs()
			.withKP(pidf.kP()).withKI(pidf.kI()).withKD(pidf.kD())
			.withKS(pidf.kFfS()).withKV(pidf.kFfV()).withKA(pidf.kFfA()));
		replaceSim();
		isClosedLoop = true;
		return this;
	}

	public TalonFXWrapper withMotionMagicConfig(CtrMotionMagicRecord motionMagic) {
		m_motor.getConfigurator().apply(new MotionMagicConfigs()
			.withMotionMagicCruiseVelocity(motionMagic.cruiseVelocity())
			.withMotionMagicAcceleration(motionMagic.acceleration())
			.withMotionMagicJerk(motionMagic.jerk()));
		replaceSim();
		isClosedLoop = true;
		return this;
	}

	public void simulationPeriodic() {
		if (m_sim != null) {
			double volts = m_motor.getMotorVoltage().getValue().in(Units.Volts);
			_simVelocity = volts * 5.0;
			m_sim.setSupplyVoltage(Units.Volts.of(12));
			m_sim.setRotorVelocity(_simVelocity);
		}
	}

	public TalonFX getMotor() {
		return m_motor;
	}

	public TalonFXSimState getSim() {
		return m_sim;
	}

	private void replaceSim() {
		if (m_motor == null)
			return;
		if (RobotBase.isSimulation()) {
			m_sim = m_motor.getSimState();
		}
	}

	/**
	 * Common interface for seting the direct voltage output of a motor controller.
	 *
	 * @param v
	 *            The voltage to output.
	 */
	public void set(Voltage v) {
		m_motor.setVoltage(v.in(Units.Volts));
	}

	/**
	 * Common interface for setting the speed of a motor controller.
	 *
	 * @param speed
	 *            The speed to set. Value should be between -1.0 and 1.0.
	 */
	public void set(double speed) {
		m_motor.set(speed);
	}

	/**
	 * Sets the mechanism position of the device in mechanism rotations.
	 * <p>
	 * This will wait up to 0.100 seconds (100ms) by default.
	 * 
	 * @param a
	 *            Value to set to. Units are in rotations.
	 */
	public void set(Angle a) {
		m_motor.setPosition(a);
	}

	/**
	 * Sets the angular velocity on the MotionMagic Profile of slot 0.
	 * 
	 * @param v
	 *            Velocity
	 * @return The set velocity of the motor
	 * @throws IllegalStateException
	 *             Throw if the wrapper is not configured with closed-loop. See
	 *             {@link#withSlot0Pidf} and/or {@link#withMotionMagicConfig}.
	 */
	public void set(AngularVelocity v) {
		if (!isClosedLoop)
			throw new IllegalStateException(
				"TalonFXWrapper: Closed-loop is not active. Use withSlot0Pidf to enable.");

		m_motor.setControl(m_slot0VelocityVoltage.withVelocity(v));
	}
}
