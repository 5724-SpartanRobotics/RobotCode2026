package frc.lib.motor.spark;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.PIDFfRecord;
import frc.lib.motor.ClosedLoopMotor;
import frc.lib.motor.talonfx.TalonFXIO_Wrapper;

public class SparkIO_SparkMax extends SparkMax implements SparkIO {
	private final MotorType motorType;
	private final SparkMaxConfig config = new SparkMaxConfig();

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
	public SparkIO_SparkMax as_SparkIO() {
		return this;
	}

	@Override
	public TalonFXIO_Wrapper as_TalonFXIOWrapper() {
		throw new UnsupportedOperationException("Cannot convert Spark to TalonFX");
	}

	@Override
	public SparkIO_SparkMax as_IOSparkMax() {
		return this;
	}

	@Override
	public SparkIO_SparkFlex as_IOSparkFlex() {
		throw new UnsupportedOperationException();
	}

	@SuppressWarnings("removal")
	public ClosedLoopMotor applyPidfsva(PIDFfRecord pid) {
		this.config.apply(new ClosedLoopConfig()
			.pidf(pid.kP(), pid.kI(), pid.kD(), pid.kFf())
			.apply(new FeedForwardConfig()
				.sva(pid.kFfS(), pid.kFfV(), pid.kFfA()))
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder));
		return this.applyConfiguration(this.config);
	}

	@Override
	public SparkIO_SparkMax applyConfiguration(SparkBaseConfig config) {
		this.config.apply(config);
		this.configure(this.config, com.revrobotics.ResetMode.kResetSafeParameters,
			com.revrobotics.PersistMode.kNoPersistParameters);
		return this;
	}

	@Override
	public Angle getPosition() {
		return Units.Rotations.of(this.getEncoder().getPosition());
	}

	@Override
	public AngularVelocity getVelocity() {
		return Units.RPM.of(this.getEncoder().getVelocity());
	}

	@Override
	public void set(double speed) {
		super.set(speed);
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
			useMaxMotion ? ControlType.kMAXMotionVelocityControl : ControlType.kVelocity);
	}

	@Override
	public void setDutyCycle(double setpoint) {
		setDutyCycle(setpoint, true);
	}

	@Override
	public void setVoltage(Voltage outputVoltage) {
		setVoltage(outputVoltage, true);
	}

	@Override
	public void setPosition(Angle angle) {
		setPosition(angle, true, false);
	}

	@Override
	public void setVelocity(AngularVelocity velocity) {
		setVelocity(velocity, true, false);
	}
}
