package frc.lib.motor;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.PIDFfRecord;
import frc.lib.motor.spark.SparkIO;
import frc.lib.motor.talonfx.TalonFXIO_Wrapper;

public interface ClosedLoopMotor {
	public void setDutyCycle(double setpoint);
	public void setVoltage(Voltage volts);
	public void setPosition(Angle angle);
	public void setVelocity(AngularVelocity velocity);

	public AngularVelocity getVelocity();
	public Angle getPosition();

	/**
	 * @return Spark (IO) wrapper
	 * @throws UnsupportedOperationException
	 *             if the motor cannot convert to SparkIO
	 */
	public SparkIO as_SparkIO();
	/**
	 * @return TalonFXIO_Wrapper TalonFX (IO) wrapper
	 * @throws UnsupportedOperationException
	 *             if the motor cannot convert to TalonFXIO_Wrapper
	 */
	public TalonFXIO_Wrapper as_TalonFXIOWrapper();

	public ClosedLoopMotor applyPidfsva(PIDFfRecord pid);
}
