// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.CustomPeriodLoggedRobot;
import frc.lib.Elastic;
import frc.robot.commands.SelectAutonomousTabOnDSConnect;
import frc.robot.info.Debug;
import frc.robot.info.RobotMode;
import frc.robot.info.constants.BuildConstants;
import frc.robot.io.RobotIO;
import frc.robot.io.RobotIO_Real;
import frc.robot.subsystems.led.LedSubsystem;

public class Robot extends CustomPeriodLoggedRobot {
	private final Command m_selectAutoTabOnBootCommand = new SelectAutonomousTabOnDSConnect(
		"Autonomous");
	private Command m_autonomousCommand;

	private final RobotContainer m_robotContainer;

	private final RobotIO io;
	private final RobotIO.RobotIOInputs inputs = new RobotIO.RobotIOInputs();

	public Robot() {
		if (!Debug.isBeanDebug()) {
			// Trying to start a DataLogManager on a debug session crashes for some reason
			DataLogManager.start();
			DriverStation.startDataLog(DataLogManager.getLog());
			URCL.start();
		}

		// Record metadata
		Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		switch (BuildConstants.DIRTY) {
			case 0 :
				Logger.recordMetadata("GitDirty", "All changes committed");
				break;
			case 1 :
				Logger.recordMetadata("GitDirty", "Uncomitted changes");
				break;
			default :
				Logger.recordMetadata("GitDirty", "Unknown");
				break;
		}

		// Set up data receivers & replay source
		switch (RobotMode.get()) {
			case Real :
				// Running on a real robot, log to a USB stick ("/U/logs")
				Logger.addDataReceiver(new WPILOGWriter());
				Logger.addDataReceiver(new NT4Publisher());
				break;

			case Simulation :
				// Running a physics simulator, log to NT
				Logger.addDataReceiver(new NT4Publisher());
				break;

			case Replay :
				// Replaying a log, set up replay source
				setUseTiming(false); // Run as fast as possible
				String logPath = LogFileUtil.findReplayLog();
				Logger.setReplaySource(new WPILOGReader(logPath));
				Logger.addDataReceiver(
					new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
				break;
		}

		Logger.start();

		io = new RobotIO_Real();

		m_robotContainer = RobotContainer.getInstance();
		CommandScheduler.getInstance().schedule(m_selectAutoTabOnBootCommand);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

		io.updateInputs(inputs);
		Logger.processInputs("Robot", inputs);
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		if (m_selectAutoTabOnBootCommand != null)
			m_selectAutoTabOnBootCommand.cancel();

		m_autonomousCommand = m_robotContainer.getAutonomousCommand();
		if (m_autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(m_autonomousCommand);
		}

		Elastic.selectTab("Autonomous");
		LedSubsystem.getInstance().clearAllPersistentNotify();
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		if (m_selectAutoTabOnBootCommand != null)
			m_selectAutoTabOnBootCommand.cancel();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		Elastic.selectTab("Teleoperated");
		LedSubsystem.getInstance().clearAllPersistentNotify();
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void teleopExit() {
		LedSubsystem.getInstance().clearAllPersistentNotify();
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}
}
