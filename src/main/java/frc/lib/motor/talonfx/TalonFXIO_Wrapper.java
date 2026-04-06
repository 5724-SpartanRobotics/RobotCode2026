package frc.lib.motor.talonfx;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.PIDFfRecord;
import frc.lib.motor.spark.SparkIO;

public class TalonFXIO_Wrapper implements TalonFXIO {
	private final TalonFXWrapper wrapper;
	private final TalonFX motor;

	public TalonFXIO_Wrapper(TalonFXWrapper wrapper) {
		this.wrapper = wrapper;
		this.motor = wrapper.getMotor();
	}

	@Override
	public void updateInputs(TalonFXIOInputs inputs) {
		inputs.positionRotations = motor.getPosition().getValue().in(Units.Rotations);
		inputs.velocityRPS = motor.getVelocity().getValue().in(Units.RotationsPerSecond);
		inputs.appliedVolts = motor.getMotorVoltage().getValue().in(Units.Volts);
		inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValue().in(Units.Amps);
		inputs.statorCurrentAmps = motor.getStatorCurrent().getValue().in(Units.Amps);
		inputs.tempCelsius = motor.getDeviceTemp().getValue().in(Units.Celsius);
	}

	@Override
	public SparkIO as_SparkIO() {
		throw new UnsupportedOperationException("Cannot convert TalonFX to Spark");
	}

	@Override
	public TalonFXIO_Wrapper as_TalonFXIOWrapper() {
		return this;
	}

	@Override
	public TalonFXIO_Wrapper applyPidfsva(PIDFfRecord pid) {
		this.wrapper.withSlot0Pidf(pid);
		return this;
	}

	@Override
	public Angle getPosition() {
		return this.wrapper.getMotor().getPosition().getValue();
	}

	@Override
	public AngularVelocity getVelocity() {
		return this.wrapper.getMotor().getVelocity().getValue();
	}

	@Override
	public void set(double speed) {
		wrapper.set(speed);
	}

	@Override
	public void setDutyCycle(double setpoint) {
		wrapper.set(setpoint);
	}

	@Override
	public void setVoltage(Voltage volts) {
		wrapper.set(volts);
	}

	@Override
	public void setPosition(Angle angle) {
		wrapper.set(angle);
	}

	@Override
	public void setVelocity(AngularVelocity velocity) {
		wrapper.set(velocity);
	}

	public void simulationPeriodic() {
		wrapper.simulationPeriodic();
	}

	public TalonFXWrapper getWrapper() {
		return wrapper;
	}

	public TalonFX getMotor() {
		return wrapper.getMotor();
	}

	public TalonFXSimState getSim() {
		return wrapper.getSim();
	}
}