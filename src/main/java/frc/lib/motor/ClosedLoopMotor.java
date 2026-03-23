package frc.lib.motor;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface ClosedLoopMotor {
	public void setDutyCycle(double setpoint);
	public void setVoltage(Voltage volts);
	public void setPosition(Angle angle);
	public void setVelocity(AngularVelocity velocity);
}
