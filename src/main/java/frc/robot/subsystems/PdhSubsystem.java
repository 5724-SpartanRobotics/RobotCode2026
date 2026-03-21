package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.lib.ClassFieldMapStringToInt;
import frc.lib.NopSubsystemBase;
import frc.robot.info.constants.CanIdConstants;
import frc.robot.info.constants.PdhChannelConstants;

public class PdhSubsystem extends NopSubsystemBase {
	private final PowerDistribution m_pdh;

	private PdhIO.PdhIOInputs inputs = new PdhIO.PdhIOInputs();

	private PdhSubsystem() {
		m_pdh = new PowerDistribution(CanIdConstants.PDH, ModuleType.kRev);
	}

	private static final class Holder {
		private static final PdhSubsystem INSTANCE = new PdhSubsystem();
	}

	public static synchronized PdhSubsystem getInstance() {
		return Holder.INSTANCE;
	}

	public static void createInstance() {
		getInstance();
	}

	@Override
	public void periodic() {
		inputs.voltage = m_pdh.getVoltage();
		inputs.totalCurrentAmps = m_pdh.getTotalCurrent();
		inputs.totalPowerWatts = m_pdh.getTotalPower();
		inputs.totalEnergyJoules = m_pdh.getTotalEnergy();
		inputs.temperatureCelsius = m_pdh.getTemperature();

		int numChannels = m_pdh.getNumChannels();
		double[] currents = new double[numChannels];
		for (int i = 0; i < numChannels; i++) {
			currents[i] = m_pdh.getCurrent(i);
		}
		inputs.channelCurrents = currents;

		Logger.processInputs("PDH", inputs);
		ClassFieldMapStringToInt.getAsMap(PdhChannelConstants.class).forEach((device, port) -> {
			Logger.recordOutput("PDH/Channel" + port + "_" + device, inputs.channelCurrents[port]);
		});
	}

	public PowerDistribution getPDH() {
		return m_pdh;
	}

	public interface PdhIO {
		@AutoLog
		public static class PdhIOInputs implements LoggableInputs {
			public double voltage = 0.0;
			public double totalCurrentAmps = 0.0;
			public double totalPowerWatts = 0.0;
			public double totalEnergyJoules = 0.0;
			public double temperatureCelsius = 0.0;

			public double[] channelCurrents = new double[0];

			@Override
			public void toLog(LogTable table) {
				table.put("Voltage", voltage);
				table.put("TotalCurrentAmps", totalCurrentAmps);
				table.put("TotalPowerWatts", totalPowerWatts);
				table.put("TotalEnergyJoules", totalEnergyJoules);
				table.put("TemperatureCelsius", temperatureCelsius);

				table.put("ChannelCurrents", channelCurrents);
			}

			@Override
			public void fromLog(LogTable table) {
				voltage = table.get("Voltage", 0.0);
				totalCurrentAmps = table.get("TotalCurrentAmps", 0.0);
				totalPowerWatts = table.get("TotalPowerWatts", 0.0);
				totalEnergyJoules = table.get("TotalEnergyJoules", 0.0);
				temperatureCelsius = table.get("TemperatureCelsius", 0.0);

				channelCurrents = table.get("ChannelCurrents", new double[0]);
			}
		}
	}
}
