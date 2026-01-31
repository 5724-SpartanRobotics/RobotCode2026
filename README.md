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