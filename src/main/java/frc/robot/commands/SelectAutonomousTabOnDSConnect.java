package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Elastic;

public class SelectAutonomousTabOnDSConnect extends Command {

	private final String tabName;

	public SelectAutonomousTabOnDSConnect(String tabName) {
		this.tabName = tabName;
	}

	@Override
	public void execute() {
		// Keep trying until DS connects
		if (DriverStation.isDSAttached()) {
			Elastic.selectTab(tabName);
		}
	}

	@Override
	public boolean isFinished() {
		// Finish once DS is attached
		return DriverStation.isDSAttached();
	}
}