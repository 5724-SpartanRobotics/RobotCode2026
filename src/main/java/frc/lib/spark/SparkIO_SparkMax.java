package frc.lib.spark;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class SparkIO_SparkMax extends SparkMax implements SparkIO {
	private final MotorType motorType;

	public SparkIO_SparkMax(int deviceId, MotorType type) {
		super(deviceId, type);
		motorType = type;
	}

	@Override
	public void updateInputs(SparkIOInputs inputs) {
		inputs.motorType = switch (motorType) {
			case kBrushed -> "Brushed";
			case kBrushless -> "Brushless";
			default -> "Unknown";
		};
		if (motorType == MotorType.kBrushless) {
			inputs.positionRotations = this.getEncoder().getPosition(); // rotations
			inputs.velocityRPS = this.getEncoder().getVelocity() /* RPM */ / 60.0; // rps
		} else {
			inputs.positionRotations = Double.NaN;
			inputs.velocityRPS = Double.NaN;
		}
		inputs.appliedVolts = this.getBusVoltage() * this.getAppliedOutput(); // Volts * Duty cycle
																				// = Volts
		inputs.busVoltage = this.getBusVoltage(); // Volts
		inputs.outputCurrentAmps = this.getOutputCurrent(); // Amps
		inputs.tempCelsius = this.getMotorTemperature(); // Celcius
	}

	@Override
	public void set(double speed) {
		this.set(speed, false);
	}

	@Override
	public void set(double speed, boolean isClosedLoop) {
		if (isClosedLoop) {
			super.getClosedLoopController().setSetpoint(speed, ControlType.kDutyCycle);
		} else {
			super.set(speed);
		}
	}

	@Override
	public void setVoltage(Voltage volts, boolean isClosedLoop) {
		if (isClosedLoop)
			super.getClosedLoopController().setSetpoint(volts.in(Units.Volts),
				ControlType.kVoltage);
		else
			super.setVoltage(volts.in(Units.Volts));
	}

	@Override
	public void setPosition(Angle angle, boolean isClosedLoop, boolean useMaxMotion) {
		if (!isClosedLoop)
			throw new IllegalArgumentException(
				"Cannot setPosition on an open-loop SparkMax (must used closed-loop)");
		super.getClosedLoopController().setSetpoint(angle.in(Units.Rotations),
			useMaxMotion ? ControlType.kMAXMotionPositionControl : ControlType.kPosition);
	}

	@Override
	public void setVelocity(AngularVelocity velocity, boolean isClosedLoop, boolean useMaxMotion) {
		if (!isClosedLoop)
			throw new IllegalArgumentException(
				"Cannot setVelocity on an open-loop SparkMax (must used closed-loop)");
		super.getClosedLoopController().setSetpoint(velocity.in(Units.RPM),
			useMaxMotion ? ControlType.kMAXMotionPositionControl : ControlType.kPosition);
	}
}
