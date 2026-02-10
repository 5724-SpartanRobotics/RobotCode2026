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

## 9 Feb 2026
- Implement/Import the working subsystems from floor-demon branch
- Need to re-do the YAGSL config because robot dimensions changed.
- Fix/change some problems/oddities across the drive system
- Update (to an extent) YAGSL config
- add basic PathPlanner config based on floor demon
- Intake system with arm and rollers (not tested in sim)
- TalonFX wrapper created to allow "easy" working in sim (i.e., no crashing)
- Climber system with Kraken X60 (not tested in sim)
- Constants for everything
- Remove old stuff & duplicates from RobotContainer
- controller triggers updated but unsure if they'll work as intended.
