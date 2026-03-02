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

## 10 Feb 2026
- calculate the setpoint for the lower intake stage from the top stage's setpoint
	> Upper reference = 0.5 = 6784 RPM * 0.5 = 3392 RPM. On the other side of the gear box, the hex shaft is going 3392 / 5 RPM = 678.4. The circumference of the 4 inch wheel is Pi * 4 = 12.566 inches. Therefore the surface speed is 678.4*12.566 = 8525 inches per minute. The lower intake hex shaft RPM for 8525 inches per minute would be 8525 / (2.25 * Pi) = 1206 RPM. With the lower gear ratio of 4:1 the redline motor would run 1206 * 4 = 4824 RPM and with top speed of 21020, this would mean a reference of 0.23. So, in software I think we should make the upper speed a constant that we can change of 0.5 and make the lower speed the upper speed * 0.46.
- Find out whether reducer/gearbox for climber changes the output voltage for the motor (I don't think it will). 
- Lots of debugging to find out that our LED strips are very strange. See the comments throughout the LedSubsystem.java for more details. Something to do with two different bits per pixel, and not being entirely WPILib compatible, but our hack mostly works. Tested on actual LED strips we have and everything looks good and right, plus no strange color problems.
	- We do need to think about applying voltage & ground from either end of the strip (bright colors such as white fade to yellow toward the end).
	- It was suggested to use a voltage divider, but I'm not entirely sure that's necessary if we're using 5V from the VRM (which is what our strip takes). The rio will overvolt it if we use the 6V on the PWM port (I kept almost burning my fingers, that's how hot the connector was getting). rio datasheet says that signal is 5V and this seems to work fine.

## 22 Feb 2026
- Add YAMS
- Add indexer subsystem. Runs at 40% for on, or 0% for off.
- Add shooter flywheel via YAMS. Runs at a max of Neo Vortex max velocity (~710.5 rad/s), max acceleration at 710.5rad/s/0.5s=1421 rad/s/s.
- Flywheel runs at a velocity commanded by the robot's direct distance (xy-hypot) from the alliance's hub.
- Add feeder from indexer to shooter, runs at 4/1.2=3.3x the flywheel speed.
- Nothing has been tested, not even in sim!
- had to correct the max accel because it was far too slow (12min to max speed instead of sub-1s).

## 24 Feb 2026
- updated the robot constants (wheels, mass, etc.)
- start writing paths. For now, we will use PathPlanner but I kinda hate it so maybe we'll use Choreo.

## 25 Feb 2026
- update first path I made, completed it.
- add the necessary robot code for auto and paths.

## 26 Feb 2026
- Clean up LED code
- Lots more paths for blue side (need few more)
- Next steps: make same paths for red side

## 28 Feb 2026
- Tune swerve PIDs
	- Driving may be inverted, look into this.
- Tune intake
- Tune indexer
- Begin tuning flywheel, will require more tuning
- Feeder belt wasn't running, look into this.

## 1 Mar 2026
- CRLF to LF
- Add Spotless to correct writing consistency. Will apply on pre-commit.
- Remove unused methods from Drive and Vision.
- Next I'm trying to thread Vision to make it less blocking.
- Schemas for YAGSL.
- Remove old & unused code.
- Thread-ize vision, we'll see if it works (it did in sim, to the extent that sim "works").
