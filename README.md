# Spartan Robotics 2026 Robot Code Changelog
We'll (try to) add our newest changes at the *top* of this file.

## Build Status
![CI](https://github.com/5724-SpartanRobotics/RobotCode2026/workflows/CI/badge.svg)

*Internals*: Please try to keep code nice and adhere to the style: **indent using tabs**,
**keep lines 100 characters or shorter**, **do not push code that will not build**,
**do not push code (that will build) with warnings**, try to
**remove linting warnings before commit**. Before writing a commit message, please update this
document.

## Licensing Information
All code created by Spartan Robotics (FRC 5724) and/or its contributors is made available under the
permissions of the WPILib License and the SPDX-identified LGPL-3.0-or-later. Both licenses can be
found in this repository.

## 14 Jan 2026
- Create new project based on the Command Robot (Advanced) template
- Install vendor dependencies / libraries that I think we'll need
	- I don't know which CTRE version to get so I'm skipping that for now.
- Implement a few basic things (constants)
- Editorconfig: Indent using tabs (not spaces) and set max line length at 100 characters
- Initial commit
### Checklist for next few commits
- [ ] Implement Swerve code (perhaps not custom code this year :/)
- [ ] Implement robot-side vision consumer (vision processing done on coprocessor)

## 15 Jan 2026
- Add Robot & Drive constants
- Rename constants in ALL_CAPS, following a better style
- Units for constants use the built-in WPILib Units of Measure  

## 16 Jan 2026
- Add Phoenix libraries, Rev & YAGSL (add back required libraries)
- Add Elastic
- Add example swerve drive subsystem directly from yagsl (https://github.com/Yet-Another-Software-Suite/YAGSL/blob/main/examples/drivebase_only_2026/src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java)

## 17 Jan 2026
- Fix some style (spaces indent to tabs)
- Add drive subsystem and implement drive command (copied from yagsl)
	- have not yet fixed the constants/json and the file names are probably wrong (runtime error)
- Use Joystick for frame and Xbox for operator
- Add AI-generated vision code based on my Python library... hopefully it will work

## 18 Jan 2026
- Add swerve offsets from YAGSL configuration website
- Refactor code (split lines over 100 chars)
- Add some preliminary CAN IDs. I don't expect these to change, just add more.
### Checklist
- [ ] Implement Photon and remove custom code (YAGSL examples have Photon examples)

## 22 Jan 2026
- Add necessary libraries (photon and pathplanner)
- Start implementing Photon code from YAGSL examples
- Remove old custom vision implementation

## 23 Jan 2026
- Finish implementation of photon
- Add new necessary constants
### Checklist
- [ ] Remove & replace deprecated calls
- [ ] Wait for robot to be finished to find center of mass, etc.

## 28 Jan 2026
- We learned that the offset for the absolute encoder in the YAGSL config is in degrees.
- The FO_DDA command did some weird stuff with the joystick so now we just use FO_DAV and it seems to work.
- Constants were only updated in the YAGSL config jsons, not in our Constants.java file

## 30 Jan 2026
- Switch from Pi 4 photon (~5fps) to Limelight 2+ photon (~45fps)
- It's a hot mess. Sometimes locking onto the pose will lock it to the center, and sometimes it will lock it to an extrema of a the camera's viewport (either far left or right). There is some problem with the gyro such that it starts the robot with about +58deg and I can't figure that out; no amount of zeroing the gyro will make it actually zero; I cannot tell if this problem introduces other problems or not. Zeroing the pose also breaks things sometimes: it seems like zeroing the pose and then trying to use the `aimToTarget` function will do nothing (or little) because the pose is now constantly stuck at zero for an unknown reason. Unrelated, but with some quick testing: adjusting `SmartDashboard/Aim At Target/Aim Constant (Degrees)` seems to adjust where the pose will be locked on to, but it also seems somewhat arbitrary. High latency between the camera and Elastic is a problem, but it doesn't seem to affect vision locking; not sure if that matters or not. This is gonna take a long time to trace down but we don't have that time.
- Seems like the same problem in sim now? Idk this is really confusing.
- In an attempt to trace the constant CommandScheduler loop overrun -- Some snooping with VisualVM shows that perhaps the lies in `frc.robot.subsystems.VisionSubsystem.updatePoseEstimation (swervelib.SwerveDrive)`, which seems very plausible, but I also cannot just blindly trust this as the readings were taken from sim, and updating the visionsim is very cpu intensive. I'll see if I can sort this out on the real robot tomorrow, but idk.
- PS: I'm very close to scrapping this all and moving back to a custom swerve implementation that I know works, and a custom vision implementation that I know I can get to work (cause it is mostly there), and I know basically all the ins and outs of each system. That way I can also debug basically as deep as I need without wishing that it was my code. It seems like "Plug'n'Play" is turning out to be more like "Plug'n'Sob-and-Wonder-Why-Nothing-Works" (but of course I'm not annoyed or anything, wink wink). 

## 2 Feb 2026 (Notes as I work on some code debugging)
- 2026 REBUILT field uses a [rotated field setup](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#mirrored-field-vs-rotated-field).
![2026 REBUILT field](https://raw.githubusercontent.com/pranavgundu/Strategy-Board/a0b54ab5c105db7abd68f5407c4208cd250ede1e/src/images/2026.png)
- Looking at the way the coordinate system/FMS interact, I think it's best if we use the "always blue" strategy, even though our lab is red.
- Invert joystick for Red alliance
- We'll say our starting pose for testing is (12.87357, 1.88546)m or (506.833294, 74.2306672)in from the absolue-blue origin, or (3.36383, 1.88546)m in (red origin x, blue origin y). That means that when the robot faces the blue alliance, the left bumper (or side of the robot, really) will align with the left edge of the bump, and the front bumper will be aligned with the line in front of the bump. [See this image](https://drive.google.com/file/d/13pm5pMjGclVOwoO9fd6nxytOIUW3MV8C/view?usp=sharing).
- Implement/change to photon vision example from yagsl. we'll see how it does. Leave most of that disabled for now but a few commented out lines can be added back in to introduce the functionality (see below).
- We can now enable, at our discretion, the adding of vision measurements in VisionSubsystem.java:300. I've left it commented out for now so we can do testing on the robot pose without vision (solely relying on motor encoders and gyro). The VisionSubsystem.periodic() will stay enabled in YagslDriveSubsystem.periodic() because it only updates to NT for now (see YagslDriveSubsystem.periodic() method).

## 3 Feb 2026
- Get rid of deprecated methods/calls in favor of newer/more specific (less general) ones. 
- Discovered via VirtualVM that the loop overrun may be caused by unnecessary telemetry of the swerve library (go from HIGH to INFO once we are done testint).
- The pose and all is fine but the autolock doesn't work. Tried correcting it by using position instead of offset, but haven't yet tested.

## 5 Feb 2026
- Vision pose estimate works.
- Drive to pose & align to pose doesn't.
- Auto return to inital pose works but it has some drift, seems about 6in.

# 6 Feb 2026
- See YagslDriveSystem.java:363

# 7 Feb 2026
- Did some trig on a paper and I think I might have something that'll point to the center of a HUB.
- Going to pose button works.
- Add another limelight facing the other direction.
- Added LEDs and can manipulate them (commands don't work)
	- They turn the alliance color on boot.
	- Enable --> off, correct. Disable --> still off, incorrect, they should turn back to the alliance color.
	- Enable --> off, correct. Button 12 (toggle on) --> Flash notification (white), quickly fade to red, incorrect, they should stay white. Button 12 (toggle off) --> stay red, incorrect. Disable --> stay red, incorrect.
	- there was another combo but I forgot it but it was also incorrect
	- Correct: Boot --> alliance color; Enable --> off; Button 12 (toggle on) --> white; Button 12 (toggle off) --> off; Disable --> return to alliance color. Any disable during the Button 12 toggled on time should turn them the alliance color, and reenabling after that should have them off.
