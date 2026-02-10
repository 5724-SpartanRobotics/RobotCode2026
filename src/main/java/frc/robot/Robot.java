// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DebugLevel;

public class Robot extends TimedRobot {
	public static AtomicBoolean isFirstConnection = new AtomicBoolean(true);

	private static final AtomicReference<Timer> _DisabledTimer = new AtomicReference<>(new Timer());
	private final RobotContainer _RobotContainer;

	public Robot() {
		if (!Constants.isBeanDebug()) {
			// Trying to start a DataLogManager on a debug session crashes for some reason
			DataLogManager.start();
			DriverStation.startDataLog(DataLogManager.getLog());
		}

		_RobotContainer = new RobotContainer();

		// Implement other things here (shouldn't be many)

		_RobotContainer.robotFinishedBooting();

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
	public void disabledInit() {
		_DisabledTimer.get().reset();
		_DisabledTimer.get().start();
	}

	@Override
	public void disabledPeriodic() {
		if (_DisabledTimer.get().hasElapsed(Constants.Drive.WHEEL_LOCK_TIME.in(Units.Seconds))) {
			_RobotContainer.setMotorBrake(false);
			_DisabledTimer.get().stop();
			_DisabledTimer.get().reset();
			_RobotContainer.indicateWheelsUnlocked();
		}
	}

	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		_RobotContainer.setMotorBrake(true);
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		CommandScheduler.getInstance().cancelAll();
		_RobotContainer.teleopInit();
	}

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
