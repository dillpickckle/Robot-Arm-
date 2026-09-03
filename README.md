5-DOF Robot Arm

A robotic arm with 400mm reach and a 250g payload, designed and built from scratch.

<img width="1336" height="839" alt="Robot arm" src="https://github.com/user-attachments/assets/b75f3cce-34c3-496f-b104-8731cab686f2" />
Status

Hardware: Complete — CAD, STEP files, wiring diagrams, and BOM are in this repo.

Firmware: In development. The Teensy 4.1 and stepper drivers haven't arrived yet, so it can't be tested until parts are in hand. Pin assignments and steps-per-degree constants are derived from the wiring design and CAD.

Specs
5 degrees of freedom + gripper
400mm reach, 250g payload
3× NEMA 17 steppers with planetary gearboxes (base 5:1, shoulder 20:1, elbow 20:1)
3× Feetech STS3215 bus servos (wrist pitch, wrist roll, gripper)
Teensy 4.1 controller, 3× TMC2209 stepper drivers
24V power system with 12V and 5V rails
Designed in Fusion 360, printed in PETG / PA6-CF / PLA



Notes on the CAD files

No screws or bolts. Mapping every fastener would have cluttered the file without adding useful information. The BOM and assembly guide list what goes where.

No electronics. The Teensy, TMC2209s, buck converters, and other PCBs aren't modeled — I couldn't find accurate CAD for them and the manufacturers don't publish dimensioned drawings. I'll update these once I have the parts in hand to measure.

Planetary gearbox mounting points are approximate. The manufacturer doesn't specify them. Also getting updated once I can measure them myself.
