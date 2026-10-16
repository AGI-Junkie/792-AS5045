#* Abstract

This thesis describes the design and experimental evaluation of a two-link planar drawing arm. The arm is driven by two geared stepper motors through DRV8825 drivers and controlled by a PSoC 5LP microcontroller. An AS5045 12-bit magnetic encoder sits on the rear shaft of each motor, so the firmware can compare every commanded position with an independent measurement. The links are 180 mm and 176.4 mm long. A Windows host application generates shapes, solves the inverse kinematics, plans the timing under per-motor speed and acceleration limits, and uploads up to 2048 timed joint-space nodes. The firmware executes the list with hardware-timed step generation and supervises every motion against the encoders, a 250 ms host heartbeat, limit inputs and driver faults. Every stop invalidates the coordinate reference, and the machine must be homed again before it moves.

Eight experiments (E1–E8) were carried out on the finished machine. The planned paths deviate from the ideal shapes by at most 0.136 mm. Encoder noise at rest has a standard deviation of 0.81 LSB, below 0.002° at the joints. The arm-1 homing origin repeated within 10 LSB (0.022°) over ten homings, while the arm-2 proximity sensor drifted by 0.39° before settling within 0.034°. The RMS difference between encoder and command was 0.04–0.07° at every speed and for every shape tested. About 97–98 % of this difference is a component that repeats once per motor revolution; its size rules out real rotor lag, and it is attributed to the encoder measurement. Removing it leaves 0.006–0.012°. On paper, a 100 mm square was drawn with a mean side of 100.01 mm and equal diagonals, and circles were round within 0.03 mm, but absolute circle diameters differed between sheets by −1.9 to +0.4 mm while the encoder-derived diameters agreed with the design within 0.07 mm. The pen mounting therefore limits what the paper measurements can show about the arm itself. Every protective stop that was triggered behaved as designed, and point-to-point moves up to the firmware limit of 5000 microsteps/s completed 30 out of 30 times. The main remaining limitations are that both encoders are on the motor side and cannot see gearbox backlash, and that stalls of the host user interface can trigger the heartbeat stop.

**Keywords:** two-link manipulator; stepper motor; magnetic encoder; trajectory planning; PSoC 5LP; machine supervision; experimental evaluation

# 1 Introduction

## 1.1 Background and motivation

Small planar arms driven by stepper motors are common in teaching laboratories, pen plotters and drawing machines. Stepper motors are attractive for this use because they can be positioned without feedback: every step pulse turns the rotor by a known angle, and the controller tracks position by counting pulses [1]. The weakness of this open-loop scheme is that the controller cannot tell when the motor did not follow. A missed step, a stall against an obstacle or a slipping coupling leaves the counted position wrong, and nothing shows it until the drawing is visibly wrong.

A low-cost absolute magnetic encoder on each motor gives the controller an independent measurement that it can compare with the command at every moment. This does not make the drive a servo, because the motor is still commanded step by step, but it makes the open-loop drive *supervised*: the controller can stop when command and measurement disagree.

A drawing arm also depends on a host computer. Drawing a shape needs inverse kinematics, joint-limit checks and a timing plan, and these are easier to write, test and change on a PC than in microcontroller firmware. A desktop PC and its USB link are not real-time systems, however; a busy desktop can pause an application for hundreds of milliseconds. The design question is therefore not only how to draw accurately, but how to divide the work between host and firmware so that a stalled host can never leave a motor running on old commands.

## 1.2 Objectives

The work had four objectives:

- Build a two-link drawing arm with geared stepper motors and motor-side encoders, a PSoC 5LP controller, and a host application for planning and operation.
- Supervise every motion: the firmware must stop on lost communication, encoder disagreement, limit inputs and driver faults, and must never continue with a coordinate reference it can no longer trust.
- Plan shapes on the host so that the executed path stays within a stated tolerance of the ideal shape and inside the joint ranges.
- Measure what the finished machine actually does: sensor noise, homing repeatability, following error against speed and shape, accuracy on paper, protection behaviour, the cost of telemetry, and the speed limit.

## 1.3 Scope

The thesis covers the mechanical and electrical configuration as built, the firmware, the communication protocol, the host software, and eight experiments on the finished machine. Several things are outside its scope: closed-loop correction of the motor position from the encoder signal, an actuator for lifting the pen (the pen is lifted and lowered by hand), an emergency stop that removes motor power independently of the firmware, and direct measurement at the output shafts of the gearboxes.

## 1.4 Contributions

- An architecture in which the host plans and the firmware enforces. The firmware checks every uploaded node again, invalidates the coordinate reference on every stop, and runs only while the host keeps renewing a short lease.
- A characterisation of the finished machine in eight experiments, with every result kept in one data workbook together with the raw records.
- The finding that the dominant part of the measured "following error" is a once-per-motor-revolution component of the encoder reading rather than a lag of the motors, and a simple model that removes it.
- An honest account of what pen-on-paper measurements can and cannot show about the arm.

## 1.5 Outline

Chapter 2 summarises the background: two-link kinematics, stepper motors, magnetic encoders, trajectory timing and machine stopping. Chapter 3 describes the system design. Chapter 4 covers kinematics and trajectory planning, together with experiment E1 on planning quality. Chapter 5 covers calibration and homing with experiments E2 and E3. Chapter 6 reports the motion experiments E4–E8. Chapter 7 discusses the results and the limitations, and Chapter 8 concludes and lists future work.

# 2 Background

## 2.1 Kinematics of a planar two-link arm

A planar arm with two revolute joints is the standard first example in robotics textbooks [6]–[9]. With link lengths *L*~1~ and *L*~2~, the forward kinematics is a sum of two rotating vectors, and the inverse kinematics has a closed-form solution with two branches, usually called elbow-up and elbow-down. The reachable workspace is an annulus with inner radius |*L*~1~ − *L*~2~| and outer radius *L*~1~ + *L*~2~. Near the outer boundary, where the arm is almost straight, and near the inner boundary, the Jacobian becomes singular: small tip movements then need large joint movements, which matters for planning because the joint speed limits bind there first.

## 2.2 Stepper motors and microstepping

A hybrid stepper motor with 200 full steps per revolution turns 1.8° per full step. Microstepping drivers such as the DRV8825 shape the two phase currents as sampled sine waves, dividing each full step into smaller steps; the DRV8825 supports up to 1/32 step [2]. Microstepping reduces vibration and improves resolution, but the rotor position within a full step depends on load torque and on the motor's magnetic characteristics, so the angular error repeats with the electrical cycle of four full steps [1]. Under a static load the rotor lags its commanded position; the restoring torque is largest at a lag of one full step, and a lag beyond that leads to a lost step [1]. The available torque falls with speed, so the risk of losing steps grows at higher step rates.

## 2.3 Magnetic rotary encoders

The AS5045 measures the direction of the field of a diametrically magnetised magnet on the end of a shaft with an integrated Hall-sensor array, and outputs a 12-bit absolute angle (4096 positions per turn) over a synchronous serial interface (SSI) [3]. Several devices can be daisy-chained so that one serial read returns all angles. Status bits report whether the field strength is in range and whether the conversion is valid, and a parity bit protects each reading. The datasheet specifies the accuracy for a magnet placed within a small lateral and axial tolerance; an off-centre or tilted magnet adds an error that repeats with the shaft angle [3].

Mounting the encoder on the motor shaft rather than the joint multiplies the effective resolution by the gear ratio, but it also means the encoder cannot see anything that happens after the gearbox: backlash, gear transmission error and the compliance of the links and of the pen mount.

## 2.4 Trajectory timing

A drawing task gives a geometric path; the planner must add timing. A common approach separates the path from its time scaling: the path is parameterised by a progress variable, and a scalar profile maps time to progress [9], [10]. Polynomial profiles with zero velocity and acceleration at both ends, for example a quintic, start and stop smoothly [10]. For a given profile shape, the shortest duration that respects joint speed and acceleration limits follows from the largest joint derivatives along the path.

When the controller only executes straight segments in joint space between nodes, the tip moves along a curve between two nodes rather than along the intended path. This chord error depends on the node spacing and on the curvature of the map from joint space to the plane, and it must be checked against a tolerance.

## 2.5 Stopping and supervision

IEC 60204-1 defines three stop categories for machines [11]: category 0 removes power from the actuators immediately; category 1 brings the machine to a controlled stop and then removes power; category 2 is a controlled stop that leaves power on. Systems that depend on a remote controller commonly use a heartbeat, or dead-man, timer: the drive may only move while it keeps receiving fresh messages, and it stops when they cease. A hardware watchdog plays the same role for the firmware itself. None of these replaces a hard-wired emergency stop that removes power independently of the software.

## 2.6 Performance measures

ISO 9283 defines performance criteria and test methods for industrial manipulators, including pose accuracy, pose repeatability and path accuracy [12]. The experiments in this thesis borrow the ideas of repeated approach and of comparing commanded and attained paths, but they do not follow the standard's test conditions and do not claim conformance with it.

# 3 System design

## 3.1 Requirements

Table: Table 3.1. Requirements for the drawing arm.
| ID | Requirement | Where it is met |
| R1 | Draw lines, rectangles, circles, ellipses, arcs, rounded rectangles, figure-8 curves, regular polygons, spirals, custom outlines and freehand strokes. | Host planner (Ch. 4) |
| R2 | Supervise every motion with the encoders; stop when command and measurement disagree beyond a threshold. | Firmware (3.5.3) |
| R3 | Stop within a bounded time when the host stops sending. | Heartbeat lease, 250 ms (3.6) |
| R4 | After any stop, refuse to move until a new coordinate reference has been established. | Reference epoch (3.5.2) |
| R5 | Keep the executed path within 0.15 mm of the planned path. | Chord check (4.4) |
| R6 | Record every run with its plan and feedback so it can be analysed afterwards. | Experiment recorder (3.7) |
| R7 | Make the control logic testable without hardware. | Simulation tests (3.8) |

## 3.2 Architecture

Figure 3.1 shows the system. The host PC runs a .NET 10 WPF application that generates and checks shapes, plans their timing and uploads them, and that shows and records the feedback. The PSoC 5LP on a CY8CKIT-059 board [4], [5] runs at 48 MHz. It generates the step and direction signals for two DRV8825 drivers, reads both AS5045 encoders in one daisy-chained SSI transfer, monitors the home sensors, a global limit switch and the drivers' fault outputs, and stores trajectories in its flash memory. The host connects through the board's USB device port (USB CDC) or, as a fallback, through the KitProg USB–UART bridge at 115200 baud.

![Figure 3.1. System architecture. Commands flow from the host through the PSoC to the drivers and motors; the encoders and switches report back to the PSoC.](figures/fig_system_architecture.png){6.3}

The division of work follows one rule: the host may plan anything, but the firmware decides whether a motion may start and whether it may continue. The host's checks make planning convenient and give the operator clear messages; the firmware's checks are the ones that hold when the host is wrong, slow or gone.

## 3.3 Mechanics and drives

Table 3.2 lists the drive parameters. Arm 1 (the shoulder) is driven by a NEMA-17 stepper with a 40:1 gearbox, arm 2 (the elbow) by an Anaheim Automation 11YPG302S-LW4-R51 stepper with a 51.06:1 planetary gearbox. Both drivers run at 1/8 microstep. The motor of arm 2 is mounted on link 1, so it turns the elbow angle, that is the angle of link 2 relative to link 1. The pen is clamped at the end of link 2 and is lifted and lowered by hand.

Table: Table 3.2. Drive and sensor parameters.
| Quantity | Arm 1 (shoulder) | Arm 2 (elbow) |
| Link length (planning model) | 180 mm | 176.38 mm |
| Motor | NEMA-17, 1.8° | Anaheim 11YPG302S-LW4-R51, 1.8° |
| Gear ratio | 40:1 | 51.06:1 |
| Microstepping | 1/8 | 1/8 |
| Microsteps per output turn | 64,000 | 81,696 |
| Microstep at the output | 0.00563° | 0.00441° |
| Encoder LSB at the output | 0.00220° | 0.00172° |
| Encoder LSB per microstep | 2.56 | 2.56 |
| Firmware rate limit (5000 microsteps/s) | 28.1°/s | 22.0°/s |
| Home sensor | P0[3], mechanical switch (NC) | P1[0], NPN proximity sensor |

A second mechanical switch, identical to the arm-1 home switch and mounted at the symmetric position, is wired to P0[4] as a global limit. The measured mechanical stroke of the elbow is −170.6° to +165.5°.

## 3.4 Sensing

The two AS5045 encoders are daisy-chained, and one SSI transfer of 38 clock cycles returns both 12-bit angles with their status and parity bits. The firmware rejects a reading whose parity is wrong, whose status bits flag an invalid measurement, or whose angle jumps further than any real motion could explain. Each encoder is read on the motor shaft and accumulated over many turns: arm 1 turns 9° at the output for each motor revolution, the elbow about 7.05°. The output angles are therefore

$$ q_1 = \frac{360°}{4096 \cdot 40} e_1, \qquad θ_e = \frac{360°}{4096 \cdot 51.06} e_2 $$ (3.1)

where *e*~1~ and *e*~2~ are the accumulated encoder counts relative to the reference and *θ*~e~ is the elbow angle. As Table 3.2 shows, one encoder LSB is 2.56 times finer than one microstep, so the encoders resolve the commanded motion.

## 3.5 Firmware

### 3.5.1 Structure and hardware offload

The firmware has two levels. A foreground loop with a 1 ms tick runs the supervision: it reads the encoders, checks the inputs, parses frames from the host, renews or expires the heartbeat lease, and formats status lines. The trajectory itself is stepped by an interrupt from a hardware event timer, so step timing does not depend on how long the foreground takes. Several tasks were moved from software into the PSoC's programmable hardware:

- **SSI read-out**: a UDB SPI master clocks the encoders (four 10-bit words, 40 clocks for the 38 needed) instead of bit-banging.
- **Step pulses**: each axis has a pulse-request register and a PWM that produces a 10 µs step pulse from a 12 MHz clock, so the CPU never busy-waits for a pulse.
- **Pin interrupts**: a driver fault (nFAULT) or the global limit disables the step PWMs immediately in an interrupt, before the foreground sees the event.
- **Hardware dead-man**: if the foreground loop stops running for 100 ms, a hardware timer cuts the step PWMs. The watchdog (256–384 ms) is the last level and resets the controller.

### 3.5.2 Modes, stops and the coordinate reference

Figure 3.2 shows the operating modes. After power-up both drivers are disabled; nothing moves until the host sends an enable command, and even then only jogging is allowed until a coordinate reference exists. The reference is set either by switch homing (Chapter 5) or by an operator confirming that the arm is straight. Point-to-point moves and trajectories require a valid reference.

![Figure 3.2. Firmware operating modes. Any stop returns the firmware to the disabled mode and voids the coordinate reference.](figures/fig_firmware_modes.png){6.3}

All stops go through one function. Its first action is to print a fault snapshot with the phase, positions, encoder errors, inputs and the age of the last heartbeat; only then does it clear the motion state, disable the motion, clear the homed flags and increment the reference *epoch*. The host includes the current epoch in every motion frame, and the firmware rejects frames that carry an old epoch, so a command planned before a stop can never be executed after it. The stops differ only in what happens to the drivers (Table 3.3).

Table: Table 3.3. Stop causes and driver state after the stop.
| Cause | Drivers after the stop |
| Operator stop (raw stop byte, Space/Esc, host closes the link) | Disabled, no holding torque |
| Driver fault (nFAULT) | Disabled, latched until the next enable |
| Heartbeat timeout (no valid heartbeat for 250 ms) | Powered, holding |
| UART receive error or overflow | Powered, holding |
| Global limit P0[4] | Powered, holding, latched |
| Encoder lost, encoder following error, start or final position mismatch | Powered, holding |
| Unexpected sensor, time or joint-range violation during motion | Powered, holding |

In terms of IEC 60204-1 [11], the operator stop resembles a category 0 stop at the driver level, and the other stops resemble category 2 stops. Neither removes the motor supply voltage (VMOT); the machine has no hard-wired emergency stop, which is listed as a limitation in Chapter 7.

### 3.5.3 Encoder supervision

The firmware converts the commanded position of each axis into encoder units and compares it with the encoder in every loop. A difference of 328 LSB (four electrical cycles of the motor, 28.8° at the motor shaft or 0.72° at the arm-1 output) stops any motion. During jogging, a difference of 31 LSB halves the allowed speed. At the end of a point-to-point move, both axes must be within 228 LSB (about 0.5° at the arm-1 output) of the target, and before a move starts the encoders must agree with the commanded start position. These thresholds detect lost steps and stalls; they are not accuracy claims.

### 3.5.4 Trajectory execution

A trajectory is a list of up to 2048 nodes. Each node gives the two joint angles and the duration of the segment that ends there, between 50 and 500 ms. The firmware accepts a trajectory only if it starts at the current pose and every node is inside the joint ranges and within the per-axis speed limit of 5000 microsteps/s. Within a segment, each axis moves at a constant step rate, so the segment is a straight line in joint space. The firmware also monitors how late each step is relative to its due time; if the lag approaches a whole segment, it stops with a timing-overrun error.

### 3.5.5 Trajectory archive

Up to 32 trajectories can be stored in the PSoC's flash memory. The archive uses two 64 KB regions that are rewritten alternately, so a power cut during a save leaves the previous archive intact. Each stored trajectory carries a CRC-32 of its node stream and may carry a 256-byte description of the settings that produced it. When a stored trajectory is loaded, every node is checked again against the current joint ranges and speed limits, exactly as if it had been uploaded.

## 3.6 Communication protocol

The host sends framed binary messages. A full frame carries a 17-byte payload: type, sequence number, reference epoch, two angles in units of 0.01°, a 16-bit value, and a CRC-16/CCITT checksum [13] over the first 15 bytes. Each payload byte is sent as two encoded nibbles in the range 0x81–0xBF, so data can never be confused with the frame delimiters or with raw one-byte commands below 0x80. Motion frames (point-to-point, trajectory, reference) and control frames (heartbeat, enable, home, ranges, speeds, telemetry rate, archive) have separate sequence counters, so heartbeats never break the sequence of trajectory nodes. A frame that repeats the previous payload exactly is acknowledged again but not executed again, which makes retries safe. The raw byte 0x79 stops the machine at any time. Appendix B lists the frame types.

The heartbeat is a 6-byte compact frame sent every 20 ms that also carries the jog keys. Two leases protect against a stalled host (Table 3.4). On the firmware side, motion is allowed only while a valid heartbeat has arrived within the last 250 ms. On the host side, the heartbeat is sent by a writer thread, but only while the user interface thread keeps proving that it is alive: if the interface stops responding for 200 ms, the writer stops sending, and the firmware stops 250 ms after the last heartbeat. The reasoning is that if the operator cannot see the machine state or press Stop, the machine should not keep moving.

Table: Table 3.4. Timing constants of the host–firmware link.
| Constant | Value |
| Heartbeat period (compact / full frame) | 20 ms / 40 ms |
| Jog key validity on the host | 100 ms |
| Host lease: UI unresponsive → heartbeat stops | 200 ms |
| Firmware dead-man: no heartbeat → stop | 250 ms |
| Hardware dead-man: foreground stalled → step PWMs cut | 100 ms |
| Watchdog | 256–384 ms |

The firmware answers with text lines: state, capabilities (including the protocol version, currently 6, which must match the host's), position telemetry at 5–20 Hz, fault snapshots and acknowledgements. The host refuses to move a machine whose firmware reports a different protocol version.

## 3.7 Host application

The host application guides the operator through a fixed sequence shown as a workflow bar: connect, enable, establish the reference, check the trajectory, move to the start, upload, run, and review the results. Each step shows whether it is done, and the first unfinished step explains what to do next or why it cannot be done yet. Shapes are previewed on a drawing of the reachable region before anything is sent to the machine.

Every point-to-point move and every trajectory run is recorded as a folder with the settings and calibration version (`metadata.json`), the planned nodes (`plan.csv`), the telemetry (`feedback.csv`), a summary with RMS and peak errors (`summary.json`), and, if the run ended in a stop, the fault snapshot with the preceding 15 s of telemetry. An analysis window replays any record: it aligns the plan with the telemetry on the device clock (the residual on real records is about 0.005°), plots command, encoder and planned angles, errors, speeds and accelerations, shows the arm pose in the plane, and overlays two runs for comparison. Positions in the plane are always labelled as model estimates, because both encoders measure on the motor side.

A calibration wizard records measurements (link lengths, the true pose after homing, the mechanical stroke and repeated approach tests) as versioned files. It deliberately does not change the machine: compensation is a separate decision made after looking at the data. The interface is available in Chinese and English, and every text, including dynamic status messages, switches language immediately.

## 3.8 Verification approach

A release script builds the firmware, checks that the pin assignment in the hardware design matches the firmware source, builds the host, runs the host self-tests (including a replay of real interface scheduling that fails if the lease gap reaches 200 ms) and runs the firmware simulation tests. The simulation compiles the unmodified firmware source with simulated GPIO, timers, encoders and UART, and runs the real foreground loop. Directed tests cover each stop cause, the protocol rules and upload retries. Randomised tests apply random event sequences of 6000 simulated milliseconds per seed and check invariants after every millisecond, for example:

- when the firmware is disabled, homing, point-to-point motion, trajectory execution and the reference are all inactive;
- a step pulse is only issued in a millisecond that started enabled and without an active driver fault or limit;
- the reference epoch and the control sequence never decrease;
- while enabled, the following error of each axis stays below the stop threshold plus a small margin, so a jammed motor is always stopped before its error grows further.

These tests show that the control logic is consistent; they do not show that the real UART timing, motor torque or sensor wiring are right, which is what the experiments in Chapters 5 and 6 address.

# 4 Kinematics and trajectory planning

## 4.1 Model and angle convention

Figure 4.1 shows the convention used throughout the host and firmware. Both *q*~1~ and *q*~2~ are absolute angles of the links measured from the fixed +X axis, and both are zero when the arm is straight along +X. The elbow angle *θ*~e~ = *q*~2~ − *q*~1~ is what motor 2 actually turns, and it has its own range, separate from the ranges of *q*~1~ and *q*~2~.

![Figure 4.1. Two-link geometry and angle convention. *q*~1~ and *q*~2~ are absolute link angles; the elbow angle *θ*~e~ = *q*~2~ − *q*~1~ is turned by motor 2.](figures/fig_two_link.png){5.0}

The pen position is

$$ x = L_1 \cos{q_1} + L_2 \cos{q_2} $$ (4.1)

$$ y = L_1 \sin{q_1} + L_2 \sin{q_2} $$ (4.2)

and the inverse kinematics is

$$ \cos{θ_e} = \frac{x^2 + y^2 − L_1^2 − L_2^2}{2 L_1 L_2} $$ (4.3)

$$ q_1 = \operatorname{atan2}\left(y, x\right) − \operatorname{atan2}\left(L_2 \sin{θ_e}, L_1 + L_2 \cos{θ_e}\right) $$ (4.4)

with *q*~2~ = *q*~1~ + *θ*~e~. Equation (4.3) gives two solutions for *θ*~e~ with opposite signs. The operator chooses the branch, positive or negative elbow, and the whole shape is planned on that branch.

## 4.2 Shape generation and range checks

Parametric shapes are defined around an anchor point and then rotated, scaled and given a drawing direction. Custom outlines can be typed as point lists or SVG paths, and freehand strokes drawn on the preview are fitted with a continuous cubic spline that stays within 1 mm of the sketch. Rectangle corners and polygon vertices are explicit stops, where the pen comes to rest; a rounded rectangle is one continuous curve.

All shapes share one check. Sample points no more than 1 mm apart, the extremes between samples (exact for lines and circles, three extra points per millimetre for general curves) and every quantised node must lie within the three ranges for *q*~1~, *q*~2~ and the elbow. The same three ranges are sent to the firmware, which checks them again for every node and every point-to-point target.

## 4.3 Time planning

The path between two stops is called a stroke. Each stroke is parameterised by a progress variable *u* from 0 to 1, and time enters through a normalised profile *u*(*z*), with *z* = *t*/*T* and stroke duration *T*. The profile has zero velocity and zero acceleration at both ends. The simplest choice is the quintic

$$ u(z) = 10z^3 − 15z^4 + 6z^5 $$ (4.5)

whose peak speed is 1.875 times the average. The planner can also use quintic ramps over the first and last fraction *a* of the stroke with constant speed in between, which raises the peak speed only to 1/(1 − *a*) times the average but needs larger accelerations. For motor *i* the joint velocity and acceleration are

$$ \frac{\mathrm{d}q_i}{\mathrm{d}t} = \frac{q_i′ u′}{T}, \qquad \frac{\mathrm{d}^2 q_i}{\mathrm{d}t^2} = \frac{q_i″ {u′}^2 + q_i′ u″}{T^2} $$ (4.6)

where *q*~i~′ = d*q*~i~/d*u*, *q*~i~″ = d^2^*q*~i~/d*u*^2^ and *u*′, *u*″ are the derivatives of the profile with respect to *z*. The path derivatives come analytically from the inverse kinematics. For a fixed profile, the shortest admissible duration is

$$ T = \max\left(\frac{\max{|q_i′ u′|}}{v_i}, \sqrt{\frac{\max{|q_i″ {u′}^2 + q_i′ u″|}}{α_i}}\right) $$ (4.7)

where the inner maxima are taken along the stroke, the outer one over both motors, and *v*~i~ and *α*~i~ are the speed and acceleration limits. The planner evaluates these maxima at 1001 points of each stroke, adds a 5 % margin, and chooses the ramp fraction on a coarse grid. A ramp helps when the speed limit binds; when the acceleration limit binds, the single quintic is better. Speed limits are per motor, so the elbow limit applies to *θ*~e~, not to *q*~2~.

## 4.4 Node export and quantisation

The timed path is exported as nodes, at least one every 250 ms within each stroke, with times rounded to whole milliseconds and angles quantised to the protocol's 0.01° and then to whole microsteps, exactly as the firmware does. Because the firmware moves in a straight line in joint space between nodes, the host reconstructs each segment as the firmware will execute it and compares nine points along it with the timed path. The tip must stay within 0.15 mm of the path, and the step rate of every segment must stay within the limit after rounding. If either check fails, the whole time plan is stretched by a factor of 1.15 and exported again, up to twelve times. Finally the node list must satisfy the firmware's own limits: segments of 50–500 ms and at most 2048 nodes.

One side effect was visible in the experiments. Raising the speed setting does not always shorten a plan: above about 9.7°/s the circle used in E4 took about 26 s, and one faster setting produced 27.2 s, because a stretch was triggered. For this reason the experiments characterise speed by the planned peak step rate rather than by the speed setting.

## 4.5 Experiment E1: planning quality

**Method.** E1 asks how far the path that the firmware actually executes deviates from the ideal shape. For every complete trajectory record, the executed tip path was reconstructed from the planned node step counts with linear interpolation in joint space (60 points per segment) and compared with the ideal shape computed from its defining formula. Both sides use the same kinematic model, so this isolates planning and quantisation errors; errors in link lengths or gearboxes are the subject of E4 and E5. Records with identical shape, size and speed settings were counted once, leaving 25 distinct plans across eight shape types.

**Results.** Table 4.1 and Figure 4.2 summarise the results. The largest deviation from the ideal shape over all plans was 0.136 mm (a figure-8 of 80 × 50 mm with node spacing up to 9 mm), and the RMS deviation of every plan was at most 0.056 mm. At the nodes themselves, where only quantisation acts, the deviation was at most 0.038 mm.

Table: Table 4.1. E1: deviation of the executed path from the ideal shape, by shape type.
| Shape | Plans | Max deviation (mm) | Largest RMS (mm) | Max at nodes (mm) | Largest node spacing (mm) | Smallest joint margin (°) |
| Circle | 10 | 0.106 | 0.049 | 0.028 | 6.10 | 30.8 |
| Line | 3 | 0.115 | 0.052 | 0.038 | 10.93 | 11.8 |
| Rectangle | 3 | 0.045 | 0.013 | 0.031 | 8.61 | 41.5 |
| Rounded rectangle | 2 | 0.127 | 0.037 | 0.028 | 4.49 | 29.3 |
| Regular polygon | 2 | 0.065 | 0.021 | 0.032 | 9.63 | 7.7 |
| Ellipse | 1 | 0.111 | 0.041 | 0.027 | 9.10 | 13.1 |
| Figure-8 | 2 | 0.136 | 0.056 | 0.032 | 8.97 | 0.8 |
| Spiral | 2 | 0.064 | 0.012 | 0.029 | 3.89 | 5.0 |
| All | 25 | 0.136 | 0.056 | 0.038 | 10.93 | 0.8 |

![Figure 4.2. E1: largest deviation from the ideal shape against the largest node spacing of each plan. The dotted line marks the largest deviation found at the nodes themselves (step quantisation).](figures/fig_e1_planning.png){6.0}

The deviation grows with node spacing, which confirms that it comes mainly from the straight joint-space segments between nodes. The same circle of radius 80 mm deviated by 0.033 mm at 4.8°/s, where nodes were 2.7 mm apart, and by 0.106 mm at 18.9°/s, where they were 6.1 mm apart. Straight lines are curves in joint space, so they show the effect clearly: at about 10 mm between nodes they deviate by about 0.11 mm. The deviation also depends on where a segment lies in the workspace and in which direction it runs, which is why the rectangles deviated less than the long lines at similar spacing. Every plan stayed within the 0.15 mm chord tolerance, and all deviations are well below the width of a pencil line (0.4–0.6 mm, Section 6.5). The smallest margin to a joint range was 0.8°, for a large 120 × 140 mm figure-8; every other plan kept at least 5°.

# 5 Calibration and homing

## 5.1 Calibration record

The machine was calibrated with the wizard described in Section 3.7 before the experiments; the record is cal-20261014-202118-v1. The link lengths measured with a caliper were *L*~1~ = 180 mm and *L*~2~ = 176.5 mm, each with an uncertainty of about ±0.5 mm. The planning model still uses *L*~2~ = 176.38 mm; the 0.12 mm difference is within the measurement uncertainty. After homing, the true pose differed from the straight reference by 0.2° in *q*~1~ and 0.25° in *q*~2~. These offsets were recorded but deliberately not compensated, so that all experiments use the same uncompensated configuration. At a reach of 250 mm, a 0.2° rotation of *q*~1~ moves the tip by about 0.9 mm, but it rotates the whole drawing and does not change its size. The measured elbow stroke between the mechanical stops is −170.6° to +165.5°.

## 5.2 Homing procedure

Homing is a non-blocking state machine in the firmware. Arm 1 approaches its switch quickly, releases it, backs off by 64 microsteps, approaches it again slowly (at most 400 microsteps/s), waits 50 ms for the arm to settle and averages 12 encoder readings to define the origin. It then moves 126° to the straight position. Arm 2 repeats the double touch on its NPN proximity sensor, which is mounted on link 1, and then unfolds 162° (36,763 microsteps). The direction of each encoder is identified from the homing motion itself. During the whole procedure the firmware continues to supervise the heartbeat, encoders, faults and the global limit. Only when both axes have finished does it report `HOME OK`, increment the epoch and accept motion commands. A complete homing took about 55 s.

## 5.3 Experiment E2: encoder noise at rest

**Method.** With the drivers enabled and holding and the arm at rest, the encoder readings were logged for 120 s, giving 1428 telemetry samples.

**Results.** Figure 5.1 shows the distributions. The standard deviation was 0.81 LSB for both encoders, which is 0.0018° at the arm-1 output and 0.0014° at the elbow. The peak-to-peak range was 6 LSB (0.013°) for arm 1 and 5 LSB (0.0086°) for arm 2.

![Figure 5.1. E2: deviation of the encoder readings from their mean at rest, 120 s, 1428 samples.](figures/fig_e2_static_noise.png){6.3}

The noise is far below the supervision thresholds (31 and 328 LSB) and about thirty times smaller than the RMS errors measured during motion in Chapter 6 (about 0.06°, or 27 LSB, for arm 1). Differences between experiments in Chapter 6 are therefore not caused by encoder noise.

## 5.4 Experiment E3: homing repeatability

**Method.** Ten homings were run back to back within one enable session by a script, each guarded by a 60 s watchdog. After each homing, the averaged origin (in motor-shaft encoder counts) and the encoder error at the straight position were recorded.

**Results.** Table 5.1 and Figure 5.2 show the results.

Table: Table 5.1. E3: spread of the homing origin over ten consecutive homings.
| Measure | Arm 1 (mechanical switch) | Arm 2 (NPN proximity sensor) |
| Standard deviation | 3.0 LSB (0.0066°) | 74 LSB (0.127°) |
| Range, all 10 homings | 10 LSB (0.022°) | 227 LSB (0.391°) |
| Range, last 5 homings | 4 LSB (0.0088°) | 20 LSB (0.034°) |
| Largest error at the straight position | 1 LSB | 2 LSB |

![Figure 5.2. E3: homing origin of each run relative to the mean of the ten runs, expressed at the output.](figures/fig_e3_homing.png){6.0}

The mechanical switch of arm 1 is repeatable within 0.022° at the output. The origin found with the NPN proximity sensor of arm 2 moved by 0.39° over the first runs, mostly in one direction (from 2985 to 2758 LSB), and settled within 0.034° over the last five. A drift of the switching point while the sensor warms up is the most likely explanation, but it was not proven. The effect matters at the tip: 0.39° of elbow error moves the pen by about 1.2 mm at the end of the 176.4 mm link 2. Homing should therefore wait until the sensor has warmed up, or the sensor should be replaced by a mechanical switch.

The error at the straight position measures how consistently the arm moves from the origin to the straight pose; it was at most 2 LSB. It does not measure the repeatability of the origin itself.

# 6 Experiments and results

## 6.1 Method

All experiments were run through the host application, so the protocol, the checks and the recording were the same as in normal use. Repeated runs were driven by scripts that pressed the application's own buttons through Windows UI Automation; the scripts did not bypass the protocol. A script was also used to wait for each run to finish by following the connection log.

Unless stated otherwise, *error* means encoder minus command, expressed as an angle at the output: *q*~1~ for arm 1 and the elbow angle *θ*~e~ for arm 2. RMS values are taken over the drawing phase of a run, and *peak* is the largest absolute value. Telemetry was requested at 10 Hz, and with the status blocks the actual rate was about 12 samples/s. Positions in the plane computed from the encoders are model estimates and not measurements of the pen.

All results were entered in one Excel workbook, where summary tables are computed by formulas from the per-run rows. Values that cannot be recomputed in the workbook, such as harmonic fits or quantities read from the connection logs, are marked with their source. Table 6.1 lists the experiments.

Table: Table 6.1. Experiments.
| ID | Question | Method | Runs |
| E1 | How far does the executed path deviate from the ideal shape? | Offline reconstruction of 25 plans | 25 plans |
| E2 | How noisy are the encoders at rest? | 120 s at rest | 1428 samples |
| E3 | How repeatable is homing? | 10 consecutive homings | 10 |
| E4 | How large is the following error, and does it depend on speed or shape? | Circle at 4 speeds; 6 shapes | 27 |
| E5 | How accurate is the drawing on paper? | Square and circles measured with a caliper | 3 sheets |
| E6 | Do the protective stops work? | Each stop triggered deliberately | 9 |
| E7 | Does telemetry disturb the firmware timing? | Same circle at 5, 10 and 20 Hz | 9 |
| E8 | How fast can point-to-point moves go? | Speed ladder up to the firmware limit | 30 |

## 6.2 Experiment E4-A: following error against speed

**Method.** A circle of radius 80 mm centred at (220, 0) mm was drawn at four speed settings, three times each. Table 6.2 lists the settings with the planned duration and the planned peak step rates.

Table: Table 6.2. E4-A: circle r = 80 mm at four speeds (three runs each; mean values).
| Level | Speed (°/s, motor 1 / 2) | Plan (s) | Peak rate (microsteps/s, motor 1 / 2) | RMS *q*~1~ (°) | Spread of RMS *q*~1~ (°) | RMS elbow (°) | Peak *q*~1~ (°) |
| L1 | 4.8 / 5.0 | 53.0 | 815 / 972 | 0.0631 | 0.0015 | 0.0479 | 0.111 |
| L2 | 7.2 / 7.5 | 35.3 | 1218 / 1456 | 0.0630 | 0.0019 | 0.0480 | 0.112 |
| L3 | 14.5 / 15.1 | 26.7 | 1614 / 1916 | 0.0622 | 0.0044 | 0.0484 | 0.114 |
| L4 | 18.9 / 19.6 | 23.0 | 1876 / 2233 | 0.0635 | 0.0017 | 0.0484 | 0.117 |

**Results.** Figure 6.1 shows that the RMS error does not depend on speed: it stays at 0.062–0.064° for *q*~1~ and at 0.048° for the elbow while the peak step rate more than doubles. The differences between levels are of the same size as the spread between repeated runs (at most 0.0044°). The peak error rises slightly, from 0.111° to 0.117°. A lag caused by the dynamics of the drive would grow with speed, so the error is not dynamic lag.

![Figure 6.1. E4-A: RMS and peak error against the planned peak step rate of motor 1. Error bars show the range of three runs.](figures/fig_e4a_speed.png){6.3}

## 6.3 Experiment E4-B: following error against shape

**Method.** Five more shapes were drawn three times each at speed level L3: a line from (180, −100) to (300, 100) mm, a 120 × 120 mm square with its lower-left corner at (170, −60) mm and stops at the corners, the same square with 20 mm rounded corners and no stops, a pentagon with a circumradius of 70 mm at (230, 0) mm, and a figure-8 of 80 × 50 mm at (230, 0) mm.

**Results.** Table 6.3 lists the results together with the circle of E4-A. The RMS error was 0.047–0.072° for *q*~1~ and 0.041–0.059° for the elbow. The shapes differ in how much of each motor revolution they cover and at which motor angles they spend their time, which, as the next section shows, explains the differences.

Table: Table 6.3. E4-B: error by shape at speed level L3 (three runs each; mean values).
| Shape | Plan (s) | Peak rate (microsteps/s, motor 1 / 2) | RMS *q*~1~ (°) | RMS elbow (°) | Peak *q*~1~ (°) |
| Circle | 26.7 | 1614 / 1916 | 0.0622 | 0.0484 | 0.114 |
| Line | 10.0 | 2460 / 2815 | 0.0472 | 0.0588 | 0.100 |
| Rectangle | 36.1 | 1614 / 2078 | 0.0682 | 0.0509 | 0.118 |
| Rounded rectangle | 29.9 | 1085 / 1851 | 0.0582 | 0.0417 | 0.112 |
| Pentagon | 32.2 | 1606 / 2012 | 0.0676 | 0.0411 | 0.119 |
| Figure-8 | 26.1 | 1984 / 2544 | 0.0720 | 0.0500 | 0.119 |

## 6.4 Decomposition of the error

Plotting the error against the motor-shaft angle, that is the output angle multiplied by the gear ratio, taken modulo 360°, shows one smooth cycle per motor revolution (Figure 6.2). The error was therefore modelled as a constant plus the first two harmonics of the motor angle *φ*:

$$ e(φ) = b + c_1 \cos{φ} + s_1 \sin{φ} + c_2 \cos{2φ} + s_2 \sin{2φ} $$ (6.1)

The coefficients were fitted by least squares to all drawing samples of the twelve E4-A circle runs (Table 6.4). The model explains 97 % of the variance for *q*~1~ and 98 % for the elbow, and leaves a residual of about 0.006°.

![Figure 6.2. Error against the motor-shaft angle for all E4-A circle samples, with the fitted model of Eq. (6.1).](figures/fig_e4_decomposition.png){6.3}

Table: Table 6.4. Fitted error model at the output (all E4-A circle samples).
| Axis | Constant b (°) | First harmonic amplitude (°) | Second harmonic amplitude (°) | Residual RMS (°) |
| *q*~1~ (motor = 40 × output) | 0.0494 | 0.0532 | 0.0022 | 0.0060 |
| Elbow (motor = 51.06 × output) | 0.0323 | 0.0546 | 0.0096 | 0.0053 |

The coefficients fitted on the circles were then applied unchanged to all the other shapes. The RMS error dropped from 0.041–0.072° to 0.006–0.012° for every shape and both axes (Figure 6.3). The model is therefore a property of the machine, not of the circle.

![Figure 6.3. RMS error per shape as measured and after subtracting the per-revolution model fitted on the circle runs only.](figures/fig_e4b_shapes.png){6.3}

**Interpretation.** Three observations indicate that this component is an error of the measurement rather than a real motion of the rotor:

- It repeats exactly once per motor revolution and does not change with speed (E4-A) or shape (E4-B). Microstepping errors of a stepper motor repeat with the electrical cycle, 50 times per revolution [1], so they cannot produce it.
- Its first-harmonic amplitude is 0.053° at the arm-1 output and 0.055° at the elbow, which is 2.1° and 2.8° at the motor shafts. A stepper motor under load lags by at most about one full step (1.8°) before it loses a step [1]. If the rotor really lagged this much, steps would be lost, and the error would accumulate from revolution to revolution instead of returning to the same pattern. It never did.
- A magnet that is slightly off-centre or tilted relative to the AS5045 produces exactly this kind of error, periodic with the shaft angle [3].

The constant term has a simple explanation in the same picture. The reference is captured from the encoder at the homing position, so the encoder error at that particular motor angle is absorbed into the origin, and every later reading is offset by it. The constant therefore depends on where the motor shaft happens to be at homing, which agrees with the small change of the mean error observed after re-homing in E7.

With the measurement error removed, the motors follow their commands within 0.006–0.012° RMS at the output, that is within one to two microsteps, as far as a motor-side encoder can tell. The tip-space positions computed from the raw encoder readings overstate the tip error by the amount of this component: 0.06° of *q*~1~ corresponds to up to 0.37 mm at full reach.

## 6.5 Experiment E5: drawing on paper

**Method.** A pencil was clamped at the end of link 2 and lifted and lowered by hand. Drawings were measured with a vernier caliper. Each dimension was measured from outer edge to outer edge and from inner edge to inner edge of the line; their mean gives the centre-line value, independent of the line width, and half their difference gives the line width. Four sheets were drawn:

- Sheet 1 was discarded because the pen worked loose during the run. Its encoder data remain valid.
- Sheet 2: a 100 mm square with its lower-left corner at (180, −50) mm, drawn three times counter-clockwise and once clockwise.
- Sheet 3: a circle of 50 mm radius, drawn from a trajectory stored in the board archive. Its run is not in the records on the experiment computer, so there is no encoder data for it.
- Sheet 4: concentric circles centred at (230, 0) mm with radii of 25, 50 and 75 mm, all drawn after one homing; the 50 mm circle three times counter-clockwise and once clockwise.

**Results.** Table 6.5 lists the measurements and Figure 6.4 compares the sheet-4 diameters with the diameters computed from the encoders by fitting a circle to the forward kinematics of the encoder readings.

Table: Table 6.5. E5: pen-on-paper measurements (centre-line values).
| Sheet | Shape | Quantity | Result |
| 2 | Square 100 mm | Mean side (6 measurements) | 100.009 mm |
| 2 | Square 100 mm | Range of the sides | 0.125 mm |
| 2 | Square 100 mm | Diagonals D1 − D2 | 0.00 mm (both 141.4 mm) |
| 2 | Square 100 mm | Line width, mean / single stroke | 0.42 mm / 0.53 mm |
| 2 | Square 100 mm | Width of the three-pass bundle | 0.89 mm |
| 2 | Square 100 mm | Offset of the clockwise pass | 0.23 mm |
| 3 | Circle Ø 100 mm | Mean diameter (4 directions) | 100.42 mm (+0.42) |
| 3 | Circle Ø 100 mm | Roundness (max − min diameter) | 0.01 mm |
| 3 | Circle Ø 100 mm | Closure gap | 0.06 mm |
| 4 | Circle Ø 50 mm | Mean diameter / roundness | 48.06 mm (−1.94) / 0.005 mm |
| 4 | Circle Ø 100 mm | Mean diameter / roundness | 98.57 mm (−1.43) / 0.03 mm |
| 4 | Circle Ø 150 mm | Mean diameter / roundness | 148.82 mm (−1.18) / 0.025 mm |
| 4 | All three circles | Fit d~paper~ = (1 + k) d + c | k = +0.76 %, c = −2.28 mm, residual 0.07 mm |
| 4 | Circle Ø 100 mm | Closure gap / offset of the clockwise pass | 0.03 mm / 0.00 mm |
| 4 | All three circles | Diameter from the encoders | 49.93 / 99.93 / 149.94 mm (−0.07 mm each) |

![Figure 6.4. E5, sheet 4: diameter error of the three concentric circles measured on paper (four directions each) and computed from the encoders.](figures/fig_e5_circles.png){6.0}

The shape quality on paper is very good. The square's diagonals are equal within the reading resolution, its sides agree within 0.13 mm, the circles are round within 0.03 mm, and the circles close within 0.06 mm. Repeated counter-clockwise passes lie on top of each other to the eye; the clockwise pass of the square was offset by 0.23 mm.

The absolute size, however, is not consistent between sheets. On sheet 2 the square came out within 0.01 mm of its design size, on sheet 3 a 100 mm circle came out 0.42 mm too large, and on sheet 4, in the same region of the workspace on the same day, a 100 mm circle came out 1.43 mm too small. The encoders show that during the sheet-4 runs the motors reproduced the commanded geometry within 0.07 mm for all three circles. The difference must therefore arise after the motor shafts or in the measurement itself.

The data cannot separate the possible causes. The pencil was clamped by hand and was not perfectly rigid; the pressure on the paper was not controlled; a pencil tip wears down during drawing and the line width changes; and how the caliper jaws are placed on a 0.4–0.6 mm line affects every reading. When the direction of motion reverses, the friction force on the tip reverses too, and the tip shifts slightly, which is consistent with the 0.23 mm offset of the clockwise pass. The gearboxes and links, which the motor-side encoders cannot see, may also contribute. The constant term *c* = −2.28 mm of the sheet-4 fit, which does not scale with the circle size, suggests an offset of this kind rather than a scale error in the kinematic model, but this rests on three circles on one sheet.

The conclusion is limited accordingly: the pen-tip measurements describe the drawing on paper, not the true position of the end of link 2. Up to the motor shafts the geometric error is below 0.1 mm; on paper the shape quality is within 0.03–0.13 mm, but the absolute size varied by −1.9 to +0.4 mm between sheets for reasons that this setup cannot resolve. All caliper readings are reported as measured.

## 6.6 Experiment E6: protective stops

**Method.** Each protective function was triggered deliberately while the arm moved, and the firmware's fault snapshot and telemetry were taken from the connection log. The operator stop was pressed in the middle of a point-to-point move. Heartbeat loss was produced by freezing the host process for 0.6 s during a move. The global limit switch was pressed by hand once during the fast touch of homing (arm 1 at 2577 microsteps/s) and once during a slow 2°/s move. The joint-range check was tested with a target, (20, 0) mm, that needs an elbow angle of about 174°, beyond the configured limit of 164.5°.

**Results.** Table 6.6 summarises the results, and Figure 6.5 shows the commanded and measured *q*~1~ around one operator stop and one heartbeat-loss stop.

Table: Table 6.6. E6: protective stops.
| Case | Trials | Result | Drivers | Further encoder motion (LSB, arm 1 / arm 2) |
| Joint range | 1 | Plan refused, move button disabled, no motion | — | — |
| Operator stop | 3 | Host-measured round trip 45–57 ms; no step pulse after the stop | Disabled | ≤ 4 / ≤ 7 |
| Heartbeat loss | 3 | Stopped at a heartbeat age of 251 ms in all trials; no step pulse after the stop | Holding | ≤ 3 / ≤ 5 |
| Global limit | 2 | Immediate stop in fast homing and in a slow move | Holding | ≤ 13 / ≤ 5 |
| Stall | 0 | Not tested on hardware; covered by the simulation invariant | — | — |

![Figure 6.5. E6: commanded and measured *q*~1~ around an operator stop (left) and a heartbeat-loss stop (right); time zero is the firmware's stop.](figures/fig_e6_stops.png){6.3}

Every triggered stop behaved as designed. The heartbeat-loss stops occurred at 251 ms, the 250 ms limit plus the 1 ms loop tick. Four further heartbeat stops that occurred unplanned during other experiments (Section 7.3) also occurred at 251 ms. After every stop no further step pulse was issued, and the encoders moved on by at most 13 LSB, about 0.03° at the output, which at 2577 microsteps/s corresponds to about 2 ms of motion. A stall test was not performed, because blocking the output shaft behind a 40:1 or 51.06:1 gearbox risks damaging the gearbox and injuring the hand; the simulation invariant of Section 3.8 covers it.

## 6.7 Experiment E7: cost of telemetry

**Method.** The L3 circle of E4-A was drawn three times each with telemetry at 5, 10 and 20 Hz. The firmware's timing counters were read from the connection log after each run.

**Results.** Table 6.7 shows that the telemetry rate had no measurable effect on the firmware. The largest trajectory step lag was 11 µs and the longest foreground loop 24 ms at every rate, and no telemetry line was skipped. Formatting one telemetry line took at most 540–612 µs, which runs in the foreground and does not delay steps, because steps are generated by the event timer interrupt.

Table: Table 6.7. E7: telemetry rate (three runs each).
| Telemetry | Samples per second | Upload (s) | Skipped lines | Max format time (µs) | Max step lag (µs) | Max loop (ms) | RMS *q*~1~ (°) | RMS elbow (°) |
| 5 Hz | 7.04 | 1.29 | 0 | 612 | 11 | 24 | 0.0677 | 0.0499 |
| 10 Hz | 12.04 | 0.93 | 0 | 540 | 11 | 24 | 0.0622 | 0.0484 |
| 20 Hz | 22.02 | 0.59 | 0 | 612 | 11 | 24 | 0.0671 | 0.0502 |

The RMS error at 10 Hz is slightly lower than at 5 and 20 Hz. The 10 Hz runs were made before a re-homing whose origin differed by 5 LSB from the earlier one; as Section 6.4 explains, the constant part of the error depends on the motor angle at homing, so a small change is expected and is not caused by the telemetry. The upload of the 109-node plan took 0.6–1.3 s in all cases.

## 6.8 Experiment E8: point-to-point speed

**Method.** The arm moved between A = (242.7, −244.1) mm and B = (34.0, 174.9) mm, a move of 80° in *q*~1~ and 90° at the elbow. The accelerations were set to their maximum of 16.9°/s² and 13.2°/s². Five speed levels were tested, with requested rates from 2577 to 4977 microsteps/s, the last just below the firmware limit of 5000. Each level made three round trips (six moves).

**Results.** All 30 moves completed without any protective stop (Table 6.8, Figure 6.6). The move time fell from 9.5 s to 6.5 s. The final error stayed at most 18 LSB for arm 1 and 38 LSB for arm 2, far inside the 228 LSB arrival limit, and did not change with speed. The peak following error rose from 0.117° to 0.141° for *q*~1~ and from 0.092° to 0.110° for the elbow.

Table: Table 6.8. E8: speed ladder (six moves per level).
| Level | Requested rate (microsteps/s) | Completed | Mean move time (s) | Max final error, arm 1 / arm 2 (LSB) | Max peak error, *q*~1~ / elbow (°) |
| V1 | 2577 | 6 / 6 | 9.52 | 16 / 38 | 0.117 / 0.092 |
| V2 | 3200 | 6 / 6 | 8.45 | 16 / 38 | 0.125 / 0.093 |
| V3 | 3733 | 6 / 6 | 7.49 | 18 / 38 | 0.128 / 0.100 |
| V4 | 4355 | 6 / 6 | 6.93 | 18 / 38 | 0.136 / 0.110 |
| V5 | 4977 | 6 / 6 | 6.53 | 17 / 38 | 0.141 / 0.105 |

![Figure 6.6. E8: move time and peak error against the requested step rate. The dashed line marks the firmware limit of 5000 microsteps/s.](figures/fig_e8_speed.png){6.3}

The final error of arm 2 was 38 LSB every time the arm arrived at A and 1–4 LSB every time it arrived at B. An error that depends on the target position and repeats exactly is the signature of the position-dependent encoder error of Section 6.4, not of random positioning error. Because the elbow travels further, arm 1 is slowed down so that both axes arrive together, and the measured peak command rates were 9.5–17.4°/s for motor 1 and 10.7–19.6°/s for motor 2 (estimated by differencing telemetry, so slightly low). The speed limit found here is the firmware's 5000 microsteps/s, not the motors': the speed at which the motors would start to lose steps was not measured, because that would require raising the firmware limit.

# 7 Discussion

## 7.1 What the encoders can and cannot see

The encoders on the motor shafts did what they were installed for. They resolve the motion finely (2.56 LSB per microstep), they are quiet (σ 0.81 LSB), and they made it possible to show that the motors follow their commands within one to two microsteps once the encoders' own per-revolution error is removed. But they also showed their limits. The largest part of the measured following error, 0.04–0.07° RMS, is an error of the encoder reading, and an error of the same kind decides the final position check of the elbow in E8. A supervision threshold has to lie above this error: the stop threshold of 328 LSB is about five times the largest peak error observed (0.141°, about 64 LSB), so it leaves a comfortable margin, but it cannot be made much tighter without compensating the per-revolution component first.

Everything after the motor shafts is invisible: gearbox backlash, gear transmission error, the stiffness of the links and of the pen clamp. E5 showed that these matter on paper at the level of a millimetre, while the motor-side geometry is accurate to better than 0.1 mm. The machine can therefore be described as precise and supervised up to the motor shafts, and uncharacterised after them.

## 7.2 Error budget

Table 7.1 collects the error sources found in this work with their size, expressed where possible at the pen.

Table: Table 7.1. Error sources and their magnitude.
| Source | Magnitude | Evidence |
| Planning (straight joint-space segments) | ≤ 0.136 mm, RMS ≤ 0.056 mm | E1 |
| Step quantisation at the nodes | ≤ 0.038 mm | E1 |
| Encoder noise at rest | σ ≤ 0.002° | E2 |
| Per-revolution encoder error (measurement only) | about 0.05° amplitude; up to 0.37 mm in model XY | E4 |
| Motor following, after removing the encoder error | 0.006–0.012° RMS | E4 |
| Arm-2 homing drift with a cold sensor | up to 0.39°, about 1.2 mm at the pen | E3 |
| Zero offsets (not compensated) | 0.2° (*q*~1~), 0.25° (*q*~2~); rotate the drawing | Calibration |
| Link length L2, model against caliper | 0.12 mm (within ±0.5 mm uncertainty) | Calibration |
| Geometry up to the motor shafts | ≤ 0.07 mm in circle diameter | E5 (encoders) |
| Pen, gearbox and measurement on paper | −1.9 to +0.4 mm in absolute size between sheets | E5 (caliper) |

The planning and motor-side terms are all an order of magnitude smaller than the pen line width. The terms that matter for the drawing are the arm-2 homing drift, the uncompensated zero offsets, and everything between the motor shafts and the paper.

## 7.3 Safety and availability

All protective stops that were tested worked, and the design rule that every stop voids the reference held in every case: after each stop the host required a new homing before any further motion. Tying the heartbeat to the responsiveness of the user interface is a deliberate choice: if the operator cannot see the state of the machine or press Stop, the machine should not move.

The price of this choice became visible during the experiments. The host interface stalled several times for 233 ms, 499 ms, 1258 ms and 2459 ms, and each stall caused a heartbeat stop. The first two coincided with the automation script reading the text of the whole window through UI Automation, which runs on the interface thread of the host, while the user was also working on the same computer. After the scripts were changed to cache their controls and to follow the connection log instead, the longest stall during a full session was 183 ms, below the 200 ms lease, and no stop occurred. The later stalls of 1.3 s and 2.5 s happened while the machine was idle and no script was running, which shows that the host itself can stall for more than a second. No data were lost, because every stall led to a clean stop, but the operator had to home again each time. The cause is being investigated separately.

Two safety limitations remain. The machine has no emergency stop that removes motor power independently of the firmware; disabling the drivers does not disconnect VMOT. And the fast and slow touches of homing have no search limit, so a failed home sensor can only be caught by the operator or by another protection.

## 7.4 Limitations of the experiments

- All experiments were made on one machine over two days, and most conditions were repeated only three times.
- The pen tests cover three usable sheets, and sheet 3 has no encoder record.
- Stall detection was not tested on the hardware.
- Gearbox backlash and output-side accuracy were not measured; the calibration wizard's repeated-approach test needs a measurement at the pen or at the output shaft, which was not available.
- Temperature was not controlled or recorded, which matters for the NPN sensor drift.
- The motors' pull-out speed was not measured, because the firmware limit was kept.

# 8 Conclusion and future work

## 8.1 Conclusion

A two-link drawing arm with geared stepper motors, motor-side magnetic encoders, a PSoC 5LP controller and a Windows host application was designed, built and evaluated in eight experiments.

The division of work between host and firmware proved effective. The host plans shapes whose executed paths stay within 0.136 mm of the ideal shape, well within the 0.15 mm tolerance and far below the width of a pencil line. The firmware checks every node again, supervises every motion with the encoders, and stops on heartbeat loss, operator command, limit inputs and range violations. Every stop that was triggered behaved as designed, the heartbeat stops occurred at 251 ms, no step pulse followed any stop, and the reference was always voided.

The encoders are quiet and fine enough to characterise the drive. The measured following error of 0.04–0.07° RMS does not depend on speed or shape, and 97–98 % of it is a once-per-motor-revolution component of the encoder reading; without it, the motors follow their commands within one to two microsteps. Point-to-point moves at up to the firmware limit of 5000 microsteps/s completed every time.

On paper the shapes are good (equal diagonals, roundness within 0.03 mm), but the absolute size varied between sheets by up to 2 mm, while the encoders showed the motor-side geometry to be correct within 0.07 mm. With the pen mount used here, measurements on paper describe the drawing, not the arm. The arm-2 proximity sensor drifted by 0.39° when cold, which is the largest single error source found inside the machine.

## 8.2 Future work

- **Encoder error compensation.** Identify the per-revolution model of Eq. (6.1) for each motor during commissioning and subtract it in the firmware. This would allow much tighter encoder checks and a more truthful model-based tip estimate.
- **Output-side measurement.** Add an encoder or a dial indicator at the joint outputs to measure backlash and transmission error, and use the calibration wizard's repeated-approach test with a measurement at the pen.
- **Pen holder.** Replace the hand-tightened clamp with a rigid holder with a defined pen force and a servo for lifting, and repeat E5 with a fine-liner pen whose line width does not change.
- **Arm-2 home sensor.** Warm up the NPN sensor before homing, or replace it with a mechanical switch like the one on arm 1.
- **Hard-wired emergency stop.** Add a stop circuit that removes VMOT independently of the firmware before the machine is used outside the laboratory.
- **Host stalls.** Find the cause of the 1–2.5 s interface stalls; consider keeping the operator-presence check while moving work off the interface thread.
- **Calibration.** Decide whether to apply the measured link length and zero offsets, and repeat E4 and E5 to show the effect.
- **Speed limit.** Raise the firmware limit in a controlled test to find the speed at which the motors start to lose steps, and set the limit with a known margin.

#* References

[1] P. P. Acarnley, *Stepping Motors: A Guide to Theory and Practice*, 4th ed. London, U.K.: The Institution of Electrical Engineers, 2002.

[2] Texas Instruments, "DRV8825 stepper motor controller IC," datasheet SLVSA73, Dallas, TX, USA.

[3] ams AG, "AS5045 12-bit programmable magnetic rotary encoder," datasheet, Premstaetten, Austria.

[4] Cypress Semiconductor, "PSoC 5LP: CY8C58LP family datasheet," Doc. No. 001-84932, San Jose, CA, USA.

[5] Cypress Semiconductor, "CY8CKIT-059 PSoC 5LP prototyping kit guide," San Jose, CA, USA.

[6] J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 3rd ed. Upper Saddle River, NJ, USA: Pearson Prentice Hall, 2005.

[7] M. W. Spong, S. Hutchinson, and M. Vidyasagar, *Robot Modeling and Control*. Hoboken, NJ, USA: Wiley, 2006.

[8] B. Siciliano, L. Sciavicco, L. Villani, and G. Oriolo, *Robotics: Modelling, Planning and Control*. London, U.K.: Springer, 2009.

[9] K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*. Cambridge, U.K.: Cambridge University Press, 2017.

[10] L. Biagiotti and C. Melchiorri, *Trajectory Planning for Automatic Machines and Robots*. Berlin, Germany: Springer, 2008.

[11] *Safety of Machinery — Electrical Equipment of Machines — Part 1: General Requirements*, IEC 60204-1:2016, International Electrotechnical Commission, Geneva, Switzerland, 2016.

[12] *Manipulating Industrial Robots — Performance Criteria and Related Test Methods*, ISO 9283:1998, International Organization for Standardization, Geneva, Switzerland, 1998.

[13] W. W. Peterson and D. T. Brown, "Cyclic codes for error detection," *Proceedings of the IRE*, vol. 49, no. 1, pp. 228–235, 1961.

#* Appendix A. Data and software

All experiment results are kept in the workbook `experiments/实验数据.xlsx` in the project repository. Its sheets hold the summary tables (computed by formulas), one row per automated run, the planning-quality table, the homing origins, the caliper readings, the protective-stop trials, the speed ladder and a log of observations and problems. Every run row names its record folder.

The raw records (plan, telemetry, summary and fault snapshot of every run) and the connection logs are stored by the host application under `%LOCALAPPDATA%\TwoLinkCommissioning\`. The automation and analysis scripts are in `out/experiments-20261014/`. The data behind each figure of this thesis are exported to `thesis/figdata/`, and the figures are produced by `thesis/make_figures.m` (MATLAB) and `thesis/make_diagrams.py` (matplotlib).

The firmware used for all experiments was build stage2-v11-r5000-slip-mref2-ssififo-upstat1-store32v2-meta6-read1-guards2 (protocol version 6). The host was the matching release build.

#* Appendix B. Frame types of the protocol

Table: Table B.1. Frame types (host to firmware).
| Type | Meaning | Fields | Sequence |
| 1 | Point-to-point move | *q*~1~, *q*~2~ target; value = rate 20–5000 microsteps/s | Motion |
| 2 | Trajectory begin | *q*~1~, *q*~2~ = start (must equal the current pose); value = node count 2–2048 | Motion |
| 3 | Trajectory node | *q*~1~, *q*~2~; value = segment duration 50–500 ms | Motion |
| 4 | Trajectory run | — | Motion |
| 5 | Manual reference (arm straight) | — | Motion |
| 6 | Heartbeat and jog keys | value = key mask | Control |
| 7 | Enable | — | Control |
| 8 | Switch homing | — | Control |
| 9 | Joint range | *q*~1~ = min, *q*~2~ = max; value = axis (*q*~1~, *q*~2~ or elbow) | Control |
| 10 | Speed and acceleration | *q*~1~ = speed, *q*~2~ = acceleration; value = axis | Control |
| 11 | Telemetry rate | value = 0 or 5–20 lines/s | Control |
| 12 | Trajectory archive | *q*~1~ = slot; *q*~2~ = query, save, clear, read description or read nodes | Control |
| 13 | Load trajectory from the archive | *q*~1~ = slot | Motion |
| 14 | Trajectory description block | four data bytes; value = block number | Control |
