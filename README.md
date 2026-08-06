6-DOF Robot Arm
A 400mm reach, 250g payload robot arm I designed and built from scratch.

<img width="1336" height="839" alt="Screenshot 2026-08-06 085240" src="https://github.com/user-attachments/assets/b75f3cce-34c3-496f-b104-8731cab686f2" />

Specs
6 degrees of freedom
400mm reach, 250g payload
3× NEMA 17 steppers (base, shoulder, elbow)
3× Feetech STS3215 bus servos (wrist joints + gripper)
Teensy 4.1 controller
24V power system
Designed in Fusion 360, printed in PETG / PA6-CF / PLA

The software used with this arm is not my work. This repo is hardware only.

The arm is compatible with:
LeRobot
SO-ARM100
For anything software-related, go to those projects directly.


P.S. — A few notes on what's not included in the full-assembly STEP file:

No screws or bolts. I left these out because mapping every fastener would have cluttered the file without adding useful information. The BOM and assembly guide list what fasteners go where.
No electronics. The Teensy 4.1, TMC2209 drivers, buck converters, and other PCBs aren't modeled because I couldn't find accurate CAD files for them, and the manufacturers don't publish dimensioned drawings. Once I get my hands on this, I will eventually update the files to reflect that. 
Planetary gearbox mounting points are also not known due to the manufacturer not specifying. This again will be updated once I can measure myself. 
