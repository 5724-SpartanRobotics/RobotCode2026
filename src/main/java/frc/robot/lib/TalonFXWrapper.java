package frc.robot.lib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

public class TalonFXWrapper {
    private final TalonFX m_motor;

    private TalonFXSimState m_sim = null;
    private double _simVelocity = 0;

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
        if (m_motor == null) return;
        if (RobotBase.isSimulation()) {
            m_sim = m_motor.getSimState();
        }
    }

    public void set(Voltage v) {
        m_motor.setVoltage(v.in(Units.Volts));
    }

    public void set(double speed) {
        m_motor.set(speed);
    }

    public void set(Angle a) {
        m_motor.setPosition(a);
    }
}
