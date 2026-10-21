# Control of the Mitsubishi RV-2AJ: Methods, Constraints and Findings

A detailed account of the control approaches applied to a Mitsubishi RV-2AJ
manipulator with a CR1-571 controller, intended as source material for
publication.

Specifications are cited from Mitsubishi's own documentation. Behaviour marked
**OBSERVED** was measured on the physical system. Anything not verified is
marked **UNVERIFIED** and should be checked before being published as fact.

---

## Table of contents

1. [Platform description](#1-platform-description)
2. [The five-degree-of-freedom constraint](#2-the-five-degree-of-freedom-constraint)
3. [Method A: manual teaching via the pendant](#3-method-a-manual-teaching-via-the-pendant)
4. [Method B: host control over RS-232](#4-method-b-host-control-over-rs-232)
5. [The command-level interface problem](#5-the-command-level-interface-problem)
6. [Method C: resident program with host triggering](#6-method-c-resident-program-with-host-triggering)
7. [Continuous-path motion](#7-continuous-path-motion)
8. [The implemented system: teach-and-repeat with per-point parameters and sensor-triggered automation](#8-the-implemented-system-teach-and-repeat-with-per-point-parameters-and-sensor-triggered-automation)
9. [Comparison of methods](#9-comparison-of-methods)
10. [An AI control layer and why it was constrained](#10-an-ai-control-layer-and-why-it-was-constrained)
11. [Notes for publication](#11-notes-for-publication)

---

## 1. Platform description

### 1.1 Manipulator specifications

All values from the **Mitsubishi RV-1A / RV-2AJ Standard Specifications
Manual**, Table 2-1. **CONFIRMED.**

| Property | Value |
|---|---|
| Degrees of freedom | **5** |
| Axes present | J1, J2, J3, J5, J6 (**no J4**) |
| Maximum payload | 2 kg |
| Rated payload | 1.5 kg |
| Maximum reach | 410 mm |
| Pose repeatability | **±0.02 mm** |
| Mass | approx. 17 kg |
| Ambient temperature | 0 to 40 °C |

Maximum payload applies with the flange facing downward within ±10 degrees.

### 1.2 Joint operating ranges and maximum speeds

**CONFIRMED** from the same source. These are the values a validator should
clamp against.

| Joint | Function | Range (deg) | Span | Max speed (deg/s) |
|---|---|---|---|---|
| **J1** | Waist rotation | −150 to +150 | 300 | 180 |
| **J2** | Shoulder | −60 to +120 | 180 | 90 |
| **J3** | Elbow | −110 to +120 | 230 | 135 |
| **J4** | **does not exist** | | | |
| **J5** | Wrist pitch | −90 to +90 | 180 | 180 |
| **J6** | Wrist roll | −200 to +200 | 400 | 210 |

Note that the controller retains the **J4** label in its numbering even though
the axis is absent. Software written for a 6-axis MELFA will index wrongly on
this arm if that is not handled.

### 1.3 Controller and interfaces

| Item | Value |
|---|---|
| Controller | CR1-571 |
| Language | MELFA-BASIC IV |
| Teaching pendant | R28TB |
| Host interface | RS-232C, front panel connector |
| Fieldbus to cell | SysLink, 24-pin, to `XPA4` |
| Ethernet | **none on this controller** |

The absence of Ethernet is a defining constraint of this platform and shapes
everything in sections 4 to 6.

---

## 2. The five-degree-of-freedom constraint

This is the most consequential property of the platform and deserves explicit
treatment in any paper.

A general rigid-body pose in three dimensions requires **six** degrees of
freedom: three for position, three for orientation. The RV-2AJ has **five**.
The missing axis is J4, the wrist yaw.

The consequence is that the manipulator **cannot reach an arbitrary pose**.
For any reachable position, the attainable orientations form a one-parameter
restricted set rather than the full SO(3). In practice:

- The approach vector is constrained relative to the arm's configuration
- Top-down picking is natural; arbitrary-angle picking is not
- Grasp planning must either fix the approach direction or solve a reduced
  inverse-kinematics problem that accepts orientation error

For a vision-guided picking application this is not a minor detail. A pose
estimate from a camera delivers six parameters, and two of them cannot be
realised independently. Any system built on this arm must either:

1. Constrain the task so that the lost degree of freedom does not matter, for
   example by restricting to planar objects picked from above, or
2. Project the desired pose onto the reachable manifold and accept the
   resulting orientation error, or
3. Exploit object symmetry, which is why cubes and cylinders are forgiving
   targets and arbitrary parts are not

This repository's tidying task was scoped to small cubes for exactly this
reason.

---

## 3. Method A: manual teaching via the pendant

The classical industrial approach, often termed **online programming** or
**lead-through teaching** in the literature.

### 3.1 Procedure

1. Controller to **TEACH** mode, pendant **enabled**
2. Select or create a program number
3. Jog the manipulator to the desired pose using the pendant's axis keys, in
   either joint or Cartesian jog mode
4. Store the pose into a position variable
5. Repeat for each waypoint
6. Write the motion program referencing those variables
7. Verify by stepping through the program at reduced speed
8. Controller to **AUTO**, run

### 3.2 R28TB key combinations

Several widely circulated instructions are **wrong for this pendant**. The
following are **CONFIRMED** from the manual and by operation:

| Action | Keys |
|---|---|
| Decrease speed | `STEP/MOVE` + `BACKWD` |
| **Increase speed** | `STEP/MOVE` + `FORWD` |
| **Store a position** | `STEP/MOVE` + `ADD`, held **together** |
| Step forward through a program | `+/FORWD` + `INP/EXE` |

The R28TB has **no dedicated OVRD up/down keys**. Any procedure referencing
them belongs to a different pendant generation.

Two errors arising from this were observed and corrected during the work:
position storage attempted as `ADD` then `INP/EXE` (which does not store), and
an attempt to reduce speed with `STEP/MOVE` + `FORWD`, which increases it.

### 3.3 Characteristics

**Advantages**

- No external hardware or software required
- The taught pose is the true achieved pose, so kinematic model error,
  calibration error and gravitational deflection are all absorbed implicitly
- Repeatability of ±0.02 mm applies directly to taught points
- Safe: the operator is present with the enabling device and can see the arm

**Limitations**

- Points are **static**. The program cannot respond to a part in a new place
- Teaching time scales linearly with the number of waypoints
- The arm must be taken out of production to reteach
- No access to external sensing, computation or decision-making
- Position data lives only in the controller unless deliberately backed up

### 3.4 The backup-battery dependency

Taught positions, program data, parameters **and the origin** are held in
battery-backed memory. When the controller's backup batteries failed, all of
this was at risk and the origin was lost, producing joint readings of
approximately 22676 degrees.

**Finding, CONFIRMED against Mitsubishi's troubleshooting documentation:**
re-entering the factory **DATA** origin codes does **not** restore the origin
after battery loss. The **ABS** method, using the physical alignment marks on
the arm, is the documented recovery procedure. The DATA codes were entered
correctly several times with no effect before this was established.

This is a reproducible failure mode of the platform and worth reporting. The
mean time to failure of the backup batteries is far shorter than the service
life of the manipulator, and the recovery procedure is not the one most
operators attempt first.

---

## 4. Method B: host control over RS-232

### 4.1 Physical and link layer

**CONFIRMED** by establishing a working link.

| Parameter | Value |
|---|---|
| Connector | RS-232C, front of the CR1-571 |
| Host device | `/dev/ttyUSB0` via a USB-serial adapter |
| Baud | 9600 |
| Data bits | 8 |
| Parity | **Even** |
| Stop bits | **2** |
| Flow control | none |

Controller-side parameters:

| Parameter | Value |
|---|---|
| `COMDEV` | `RS232` |
| `CPRCE11` | `0` |

Preconditions: the pendant must be **disabled**, and the controller must be in
**AUTO (Ext)**.

### 4.2 Frame format

```
1;1;<COMMAND>\r
```

The two leading fields are the robot number and the slot number. Responses:

| Response | Meaning |
|---|---|
| `Qo` | Command accepted |
| `Qe` | Error |
| `Qx` | Error |

### 4.3 Confirmed command set

Verified by readback on the physical hardware. **OBSERVED.**

| Command | Function | Class |
|---|---|---|
| `CNTLON` | Acquire control rights | Session |
| `CNTLOFF` | Release control rights | Session |
| `SRVON` | Servo on | Power |
| `SRVOFF` | Servo off | Power |
| `EXECJOVRD <n>` | Set joint speed override, percent | Motion parameter |
| `EXECM_OUT(n)=1` | Set digital output bit n | I/O write |
| `VALM_INB(0)` | Read digital input byte 0 | I/O read |
| `VALM_OUTB(0)` | Read digital output byte 0 | I/O read |

Two prefixes are visible in this set and the distinction matters:

- **`EXEC`** submits a single MELFA-BASIC line for **immediate execution**
- **`VAL`** reads the value of a variable or expression

### 4.4 The control-rights model

The controller enforces exclusive control. `CNTLON` acquires it; while held,
the pendant cannot command motion, and vice versa. This is a safety
interlock, not a software limitation, and it means a host program and a human
operator cannot both drive the arm simultaneously.

### 4.5 Methodological discipline applied

MELFA-BASIC IV has **no JOG instruction**. This was confirmed from the manual
after repeated requests to implement continuous jogging from a graphical
interface.

Rather than guess at motion syntax, an **empirical syntax-finder** was written
with the following safety properties:

- Dry-run by default; motion requires an explicit `--arm` flag
- Forces `EXECJOVRD 5`, i.e. 5 percent of maximum speed
- Restricts candidate motions to a 2 degree step on **J1 only**
- Pauses for operator confirmation before each candidate command
- Logs the controller's response to every attempt

The principle generalises: on a PLC an incorrect guess produces a compile
error, on a manipulator it produces a collision. Any published methodology for
reverse-engineering a legacy robot interface should state its containment
measures explicitly.

---

## 5. The command-level interface problem

This is the central technical finding of the work and the part most likely to
be of interest to a reader.

### 5.1 Observation

When motion commands are issued individually over RS-232, the manipulator
**decelerates to zero velocity at every waypoint**, regardless of speed and
acceleration settings. Attempting to issue a second motion command while the
first is executing returns:

```
error 5640: Cannot execute during RUN
```

### 5.2 Explanation

`EXEC` submits **one line** for immediate execution. The controller's
trajectory planner receives a single target and has no knowledge of any
subsequent target. Corner blending requires the planner to know the next
waypoint **while still in motion toward the current one**, in order to compute
the departure from the nominal path. With a one-command-at-a-time interface
this information does not exist at planning time.

The deceleration to zero is therefore not a tuning deficiency. It is a
**necessary consequence of the interface granularity**.

### 5.3 Generalisation

This is worth stating in general terms because it applies well beyond this
platform:

> An external interface that exposes **commands** rather than **trajectories**
> imposes a hard lower bound on cycle time equal to the sum of the
> accelerate-decelerate transients at every waypoint. No amount of parameter
> tuning on the host side can remove it.

Legacy industrial controllers commonly expose exactly this kind of interface.
Modern alternatives, by contrast, accept a trajectory or a stream of setpoints
at a fixed control rate, which is what makes visual servoing and
sensor-reactive motion feasible.

### 5.4 Compounding factor: serial latency

At 9600 baud, 8E2, each character consumes 11 bit periods, so throughput is
approximately 873 characters per second. A short command plus its response
costs on the order of tens of milliseconds, before any controller processing.
A host that waits for completion before issuing the next command therefore
adds this latency to **every** waypoint, on top of the mechanical transient.

**UNVERIFIED:** the per-waypoint overhead was not measured quantitatively.
A paper making a performance claim should instrument this directly, for example
by logging host-side timestamps across a fixed multi-waypoint path under each
control method and reporting the distribution.

---

## 6. Method C: resident program with host triggering

### 6.1 Principle

Invert the division of labour. Rather than the host issuing motion, the
**trajectory resides in the controller** and the host issues only high-level
commands.

```
Method B:   Host ──> Mov P1 ──> (wait) ──> Mov P2 ──> (wait) ──> ...

Method C:   Host ──> "run program 1"
            Controller, autonomously:  Cnt 1 ; Mov P1 ; Mov P2 ; Mov P3 ...
```

The controller's planner now sees the whole sequence and can blend.

### 6.2 For static paths

Write the program on the pendant, store it, and trigger it from the host. This
requires **no new serial syntax** beyond a program-run command and is the
fastest route to a working result.

### 6.3 For dynamic paths

Where waypoints are computed at runtime, for instance from vision, a
**producer-consumer buffer** is appropriate:

1. A resident program loops, reading waypoints from position variables
2. The host writes the next waypoints into those variables **ahead of** the
   manipulator reaching them
3. The planner always has lookahead, so motion stays continuous

The host must stay one or two waypoints ahead. The buffer depth needed is a
function of serial latency and of the time taken to traverse one segment, and
is a measurable quantity worth reporting.

### 6.4 Prerequisite, not yet established

The serial commands to **upload** and **run** a stored program, and to **write
a position variable**, have not been confirmed. **These must not be guessed.**
Two acceptable routes:

- Read them from the CR1 Controller Instruction Manual, communication section
- Discover them empirically using the same containment discipline as section
  4.5: read-only commands first, servo off, no motion until syntax is
  understood

---

## 7. Continuous-path motion

### 7.1 The CNT instruction

**CONFIRMED** syntax from the MELFA-BASIC IV command list:

```basic
CNT 1,100,200
```

The command list describes it as designating **smooth operation**. `CNT 1`
enables continuous path; `CNT 0` disables it.

**UNVERIFIED:** the meaning of the two numeric arguments. They are widely
understood to be deceleration-start and acceleration-end distances in
millimetres, but this was not confirmed from an authoritative source during
this work. **Do not publish a definition of these arguments without checking
the CR1 instruction manual.** Begin with `Cnt 1` alone.

### 7.2 Related motion parameters

**CONFIRMED** syntax:

| Instruction | Example | Effect |
|---|---|---|
| `OVRD` | `OVRD 100` | Speed override, percent, in 0.1 percent units |
| `ACCEL` | `ACCEL 50,80` | Acceleration and deceleration time as a percentage of maximum, in 1 percent units |
| `SPD` | `SPD 123.5` | Linear and circular interpolation speed, mm/s |
| `FINE` | `FINE 200` | Positioning completion condition, in encoder pulses |

`FINE` is the parameter controlling how tightly the controller verifies
arrival before proceeding. It is the mechanism underlying the per-waypoint
dwell, and relaxing it is an alternative to `CNT` where approximate positioning
is acceptable.

### 7.3 Program structure

```basic
10 Ovrd 100
20 Accel 100,100
30 Cnt 1
40 Mov P1
50 Mov P2
60 Mov P3
70 Cnt 0
80 Mov P4
90 End
```

### 7.4 The accuracy trade-off

With `Cnt 1` the manipulator **rounds the corner** and does not pass through
the intermediate waypoints exactly. This is the defining trade-off of
continuous-path operation and must be stated whenever a cycle-time improvement
is claimed.

The practical pattern, used above, is selective: `Cnt 1` for transit segments
where only the endpoints matter, `Cnt 0` before any pick or place pose where
the ±0.02 mm repeatability is the point of using this arm at all.

A paper reporting a speed improvement from `Cnt` should report the
corresponding **path deviation**, measured rather than assumed, otherwise the
result is not meaningful.

---

## 8. The implemented system: teach-and-repeat with per-point parameters and sensor-triggered automation

The system actually built is Method B productionised: a host-side
teach-and-repeat sequencer running on the laptop and driving the arm
point-to-point over RS-232. It accepts the per-waypoint dwell that section 9
marks as unavoidable for Method B, and engineers away only the part of that
dwell that is **not** physical.

### 8.1 Teach-and-repeat over RS-232

Each taught point stores joint angles, a gripper state, a per-point **speed**
and a set of digital **outputs**. Playback issues each point as an absolute
joint move in the verified form of section 4.3 (`JTMP = J_CURR`,
`JTMP.Jn = JTMP.Jn + RAD(d)`, `VAL`, `EXECMOV JTMP`), polling `JPOSF` until the
pose settles — reached is "at target and still for 0.2 s" — before the next.

- **Per-point speed.** Each transition sets the speed override (`OVRD` /
  `JOVRD`, 1–100 %) to the **slower of the two endpoints' speeds** — a
  "slowest-of-two" rule — so a slow point slows both its approach and its
  departure. Home is treated as a transit point with a fixed speed of its own.
  Only the override is varied; `ACCEL` is not, because it was not accepted over
  this link (section 4.3), so acceleration shaping is deliberately out of scope.
- **Per-point outputs.** A point may carry output bits actuated **on arrival**,
  in the same step as the gripper, so the taught sequence drives the station's
  pneumatics and signals, not only the arm.

### 8.2 Reducing the per-waypoint dead time

The dead time first measured at each point (~1.4 s) came not from motion but
from **host-side redundancy**: re-reading four higher I/O bytes at every point,
re-commanding an unchanged gripper (with its ~0.6 s settle), and re-sending an
unchanged speed override. Suppressing the redundant commands — and reading the
higher I/O bytes round-robin while moving, all of them at once only when the arm
is idle — cut the inter-point dead time to ~0.4 s.

The residual is the physical deceleration to a full stop at each point, which
cannot be removed without `Cnt` blending; section 7 and error 5640 establish
that blending is impossible over this link. This is the honest ceiling: the
dwell is **reduced, not eliminated**, and any cycle-time claim must say so.

### 8.3 Sensor-triggered single-pass automation

A part sensor mounted on the robot's tool and wired to input **IN1** lets the
cell run one assembly pass per arriving part. Because the sensor rides with the
arm, a reading is only meaningful at the park pose, so the sequencer **gates the
trigger on the arm being idle and within a joint tolerance (10°) of the home /
park pose**. One pass then runs from the point *after* the park point to the
last point and returns home to re-arm.

- The trigger is the **rising edge** of *(idle ∧ at-home ∧ sensor)*. Readings
  taken mid-pass, while the sensor is moving through the air near the stations,
  are ignored; returning home re-arms for the next part, so a queue of parts is
  served one pass at a time **without an explicit queue**.
- This is the sensor-reactive behaviour that the comparison table (section 9)
  marks "per-waypoint only" for Method B — obtained here at the **pass level**
  (one trigger per pass) rather than inside the motion loop, which is exactly
  what the serial latency of section 5.4 permits.
- **Safety.** Servo power is enabled only by a human; arming requires the servo
  on and at least one taught point; **STOP disarms** the automation; and it
  starts disarmed on every restart.

### 8.4 Relationship to the methods

This is Method B applied with discipline: static teaching for the path, dynamic
host control for sequencing, per-point parameters for speed and I/O, and a
single external sensor for part-level reactivity. It does **not** reach Method
C's buffered continuous motion, which would require the program-upload path of
section 6.4 that has not been established. What it does demonstrate is that a
legacy, serial-only controller can still support **parameterised,
sensor-triggered automation**, provided the reactivity is placed at the pass
level rather than inside the motion loop.

---

## 9. Comparison of methods

| Property | A: Pendant | B: Serial per-command | C: Resident + trigger |
|---|---|---|---|
| External hardware | none | serial adapter | serial adapter |
| Waypoints | static | dynamic | static or dynamic |
| Continuous path | available | **impossible** | available |
| Per-waypoint dwell | avoidable via `Cnt` | **unavoidable** | avoidable via `Cnt` |
| Host latency in the motion loop | none | every waypoint | none |
| Sensor-reactive | no | per-waypoint only | with buffering |
| Setup effort | low | medium | medium to high |
| Reteaching cost | high | none | none for the dynamic case |
| Safety supervision | operator present | remote | remote |

Method B is **not** a superset of Method A. It gains programmatic control and
loses continuous motion. This non-obvious trade is, in the author's view, the
most useful thing to communicate about legacy robot integration.

---

## 10. An AI control layer and why it was constrained

### 10.1 What was considered

A natural-language and vision-driven control layer was explored, intended to
run on modest local hardware (Ryzen 5 6600, RTX 3050 4 GB, 8 GB RAM). The
components considered were a fine-tuned small language model for instruction
interpretation, a camera for perception, and a mapping from intent to robot
motion.

### 10.2 Why end-to-end visual servoing was ruled out

Visual servoing and visuomotor policies assume a control loop closed at tens
of hertz. Section 5 establishes that this platform cannot accept setpoints at
that rate through its external interface, and error 5640 makes it explicit.

This is a **hardware-imposed** constraint, not an implementation difficulty.
Reporting it honestly is more useful than reporting a system that was quietly
scoped down to avoid it.

### 10.3 The architecture adopted instead

A **propose-and-validate** structure:

1. The model proposes a target pose or a discrete action
2. A **deterministic validator** checks and, where necessary, repairs it using
   the real kinematics and the confirmed joint limits of section 1.2
3. Only validated targets are issued, as waypoints rather than as a continuous
   stream

The validator is not a safety afterthought. Given a model that produces
plausible but occasionally invalid outputs, it is the component that makes the
system usable at all, and the joint limits in section 1.2 are precisely what
it clamps against.

### 10.4 Perception hardware

| Device | Status |
|---|---|
| Single USB webcam | available |
| Dual-lens USB module, approx. 60 mm baseline | available, **not yet characterised** |
| DVT Legend 510MR smart camera | on the cell, see the main README |

The dual-lens module must be established as a genuine **synchronised stereo
pair** rather than an RGB-plus-infrared liveness module before any depth claim
is made. The distinguishing test is whether it enumerates a side-by-side
resolution such as 2560x720, which indicates both sensors captured in one
frame. These modules are **not factory calibrated**; `cv2.stereoCalibrate` and
`cv2.stereoRectify` must be run, and at that baseline the useful depth range is
roughly 0.2 to 1.5 m.

---

## 11. Notes for publication

### 11.1 What is defensible as a contribution

- A documented, reproducible account of returning a specific obsolete
  manipulator and cell to service, including the failure modes encountered
- The **interface-granularity finding** of section 5, stated generally
- The **origin-recovery finding** of section 3.4, which contradicts commonly
  given advice
- The **negative result** on conveyor control, which is exhaustive rather than
  assumed, see the main README section 2.7
- A methodology for safely reverse-engineering an undocumented robot interface
- The **implemented teach-and-repeat system** of section 8: that a serial-only
  legacy controller can still be driven as a parameterised (per-point speed and
  I/O), **sensor-triggered** cell when reactivity is placed at the pass level
  rather than inside the motion loop — including the honest accounting that the
  per-point dwell is reduced (host-side redundancy removed) but not eliminated
  (physical deceleration remains; no `Cnt` over serial)

### 11.2 What is not yet defensible

| Claim | What is missing |
|---|---|
| Any precise cycle-time improvement figure | The per-point dead-time numbers in section 8.2 (~1.4 s → ~0.4 s) are coarse operator observations, not instrumented measurements with a timing harness and repeats |
| The meaning of the `CNT` arguments | Not confirmed from an authoritative source |
| Path deviation under `Cnt 1` | Never measured |
| Buffer depth required for Method C | Method C not yet implemented |
| Anything about the stereo module | Not characterised |

### 11.3 Experiments that would strengthen a paper

1. **Cycle time per method.** A fixed path of N waypoints, executed under
   Methods A, B and C, with host-side timestamps. Report mean and spread over
   repeated runs.
2. **Path deviation versus `Cnt` arguments.** Sweep the arguments, measure the
   deviation at the corner. This simultaneously establishes what the arguments
   mean, resolving section 7.1.
3. **Serial latency characterisation.** Round-trip time distribution for a
   null command, separating link latency from controller processing.
4. **Buffer depth versus segment duration** for Method C, establishing the
   lookahead needed to maintain continuous motion.
5. **Validator intervention rate.** How often the model's proposal is out of
   range and has to be clamped. This quantifies the validator's necessity
   rather than asserting it.

### 11.4 Reproducibility

Everything needed to repeat the serial work is in section 4: link parameters,
frame format and the confirmed command set. The joint limits in section 1.2 are
cited to the manufacturer's manual. The failure modes in section 3.4 and the
negative result on conveyor control are reproducible on any comparable cell.

What is **not** reproducible from this document is the program-upload command
set of section 6.4, because it has not been established. A paper should either
establish it or scope around it explicitly.

---

## References

- [Mitsubishi Industrial Robot RV-1A / RV-2AJ Series Standard Specifications Manual, BFP-A8050](https://dl.mitsubishielectric.com/dl/fa/document/manual/robot/bfp-a8050/bfp-a8050k.pdf) — source of all specifications in sections 1.1 and 1.2
- [Mitsubishi CR1 / CR2 / CR3 / CR4 / CR7 / CR8 / CR9 Controller Instruction Manual, BFP-A5992](https://www.roboex.com/Mitsubishi%20Movemaster%20Robot%20Manual/RV-1A/CR1%20Controller%20Instruction%20Manual.pdf)
- [MELFA-BASIC IV command list, Mitsubishi RV-3SJ Series Standard Specifications Manual](https://www.manualslib.com/manual/1949566/Mitsubishi-Rv-3sj-Series.html?page=98) — source of the `CNT`, `FINE`, `ACCEL`, `OVRD`, `SPD` syntax
- [Mitsubishi MELFA CR1D Instruction Manual](https://www.manualslib.com/manual/2162775/Mitsubishi-Melfa-Cr1d.html) — contains the sections on Continuous Movement and target-position confirmation
