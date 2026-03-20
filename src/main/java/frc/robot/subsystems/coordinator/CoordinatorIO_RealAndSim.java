package frc.robot.subsystems.coordinator;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.spark.SparkIO_SparkFlex;

public class CoordinatorIO_RealAndSim implements CoordinatorIO {
	public final SparkIO_SparkFlex motor;

	private AngularVelocity velocitySetpoint = Units.RPM.of(0);

	public CoordinatorIO_RealAndSim(int canId) {
		motor = new SparkIO_SparkFlex(canId, MotorType.kBrushless);
	}

	@Override
	public void updateInputs(CoordinatorIOInputs inputs) {
		inputs.positionRotations = motor.getEncoder().getPosition();
		inputs.velocityRPS = motor.getEncoder().getVelocity() / 60.0;
		inputs.appliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
		inputs.busVoltage = motor.getBusVoltage();
		inputs.outputCurrentAmps = motor.getOutputCurrent();
		inputs.tempCelsius = motor.getMotorTemperature();
		inputs.velocitySetpoint = velocitySetpoint;
	}

	@Override
	public void setVelocity(AngularVelocity output) {
		velocitySetpoint = output;
		motor.setVelocity(velocitySetpoint, true, false);
	}

	@Override
	public void stop() {
		velocitySetpoint = Units.RPM.of(0.0);
		motor.stopMotor();
	}
}
