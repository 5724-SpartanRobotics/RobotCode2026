// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DebugLevel;
import frc.robot.lib.Elastic;

public class Robot extends TimedRobot {
	private final RobotContainer _RobotContainer;
	private AtomicBoolean _IsFirstConnection = new AtomicBoolean(true);

	public Robot() {
		if (!Constants.isBeanDebug()) {
			// Trying to start a DataLogManager on a debug session crashes for some reason
			DataLogManager.start();
			DriverStation.startDataLog(DataLogManager.getLog());
		}

		_RobotContainer = new RobotContainer();

		// Implement other things here (shouldn't be many)

		_RobotContainer.robotFinishedBooting();

		if (DriverStation.isDSAttached() && _IsFirstConnection.compareAndSet(true, false)) {
			Elastic.selectTab("Auto");
		}

		if (isSimulation() || DebugLevel.isOrAll(DebugLevel.Autonomous)) {
			DriverStation.silenceJoystickConnectionWarning(true);
		}
		SmartDashboard.putString("Debug Mode", Constants.DEBUG_TRACE_LEVEL.toString());
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		_RobotContainer.visionPeriodic();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
