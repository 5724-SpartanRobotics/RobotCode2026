# Spartan Robotics 2026 Robot Code Changelog
We'll (try to) add our newest changes at the *top* of this file.

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
[ ] Implement Swerve code (perhaps not custom code this year :/)
[ ] Implement robot-side vision consumer (vision processing done on coprocessor)
