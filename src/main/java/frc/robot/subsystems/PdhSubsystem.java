package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants;
import frc.robot.lib.NopSubsystemBase;

@AutoLog
public class PdhSubsystem extends NopSubsystemBase {
	private static PdhSubsystem instance = null;

	private final PowerDistribution m_pdh;

	private PdhSubsystem() {
		m_pdh = new PowerDistribution(Constants.CanId.PDH, ModuleType.kRev);
	}

	public static PdhSubsystem getInstance() {
		if (instance == null)
			instance = new PdhSubsystem();
		return instance;
	}

	public static void createInstance() {
		getInstance();
	}

	public PowerDistribution getPDH() {
		return m_pdh;
	}
}
