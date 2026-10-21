# Retrofit of a Festo MPS 500 Transfer Line with a Siemens S7-1200

Revival and modernisation of a Festo MPS 500 pallet transfer line with an
integrated Mitsubishi RV-2AJ robot cell, after the failure of the original
Festo E.IPC master controller.

This repository documents the hardware, the diagnostics, the wiring, the PLC
program and the open problems. Everything marked **CONFIRMED** was verified
against the physical hardware or against manufacturer documentation. Anything
else is marked as a hypothesis.

---

## Table of contents

1. [Hardware inventory](#1-hardware-inventory)
2. [The Mitsubishi RV-2AJ robot](#2-the-mitsubishi-rv-2aj-robot)
3. [The Festo MPS 500 cell and the dead master](#3-the-festo-mps-500-cell-and-the-dead-master)
4. [The S7-1200 migration](#4-the-s7-1200-migration)
5. [Sensor and actuator bring-up](#5-sensor-and-actuator-bring-up)
6. [PLC program](#6-plc-program)
7. [The DVT Legend 510MR vision sensor](#7-the-dvt-legend-510mr-vision-sensor)
8. [Open problems](#8-open-problems)
9. [Lessons learned](#9-lessons-learned)

---

## 1. Hardware inventory

### Robot cell

| Item | Part number | Notes |
|---|---|---|
| Robot arm | Mitsubishi **RV-2AJ** | 5 axes, no J4 |
| Controller | **CR1-571** | MELFA-BASIC IV |
| Teaching pendant | **R28TB** | No OVRD up/down keys |
| Gripper | Festo **HGP-16-A-B** | Pneumatic parallel gripper |
| Robot serial number | AH501043 | Origin data label is inside the rear cover |

### Transfer line

| Item | Part number | Notes |
|---|---|---|
| System | Festo **MPS 500 / FMS 500** | Pallet transfer loop, 6 stations |
| Fieldbus | **AS-Interface (AS-i)** | Yellow flat cable, not Profibus |
| Original master | Festo **E.IPC** | Beck IPC HC2X, ROM-DOS, FST-IPC Runtime 2.24.3. **DEAD** |
| AS-i slaves | Siemens **3RG9002-0DC00** | 4 in / 4 out, 24 VDC PNP |
| Conveyor drive | Siemens **MICROMASTER 420** | Terminal control |
| Safety | Pilz **PNOZ X9** | Hardwired chain, untouched |
| Cylinder | Festo **AEVUZ-16-10-P-A** (157212) | Single-acting, pulling, magnetic piston |
| Valve | Festo **CPE10-M1BH-5L-M7** (196927) | 5/2 single solenoid, 3 to 8 bar |
| Valve coil | Festo **MSZE-3-24 DC** (662 523) | 24 VDC, 1.28 W, approx 450 ohm |
| Position sensor | Festo **SME-8** family | 2-wire reed, blue and black |

### New control hardware

| Item | Part number | Notes |
|---|---|---|
| CPU | Siemens **S7-1200 CPU 1214C DC/DC/DC** | `6ES7 214-1AG40-0XB0`, FS02 |
| | | 14 DI, 10 DQ at 0.5 A, 2 AI |
| | | MAC `28-63-36-88-06-64` |
| Power supply | Delta **DVP-PS01** | 24 VDC, 1 A, 24 W |
| Interface relay | Weidmüller **TRS 24VDC 1CO** | `1122770000`, 6 A changeover |
| AS-i master (planned) | Siemens **CM 1243-2** | `3RK7243-2AA30-0XB0`, AS-i V3.0 |

### Vision

| Item | Part number | Notes |
|---|---|---|
| Smart camera | DVT **Legend 510MR** | s/n 510221142, now Cognex |
| Break-out board | DVT **Isolated BOB** | 24V/GND, 8 configurable I/O |
| Switch | Heuer Automation | 2-port, P1/P2 |
| Camera IP | **192.168.10.242** | CONFIRMED |
| Camera MAC | **00:90:68:00:f6:12** | OUI = DVT CORP |

---

## 2. The Mitsubishi RV-2AJ robot

> **For the full treatment of control methods, see
> [`ROBOT_CONTROL.md`](ROBOT_CONTROL.md).** That document covers the
> manipulator specifications and joint limits, the 5-DOF constraint, pendant
> teaching, host control over RS-232, the command-level interface problem, the
> proposed resident-program architecture, and notes for publication. This
> section is a summary.

### 2.0 Specifications, CONFIRMED

From the Mitsubishi RV-1A / RV-2AJ Standard Specifications Manual, Table 2-1:

| Property | Value |
|---|---|
| Degrees of freedom | 5 (J1, J2, J3, J5, J6, **no J4**) |
| Maximum payload | 2 kg |
| Rated payload | 1.5 kg |
| Maximum reach | 410 mm |
| Pose repeatability | ±0.02 mm |
| Mass | approx. 17 kg |

Joint operating ranges and maximum speeds:

| Joint | Range (deg) | Max speed (deg/s) |
|---|---|---|
| J1 | −150 to +150 | 180 |
| J2 | −60 to +120 | 90 |
| J3 | −110 to +120 | 135 |
| J5 | −90 to +90 | 180 |
| J6 | −200 to +200 | 210 |

### 2.1 Origin recovery after battery loss

The controller's backup batteries had failed, which lost the origin and left
the joints reading nonsense values (around 22676 degrees).

**Key finding, CONFIRMED against Mitsubishi's troubleshooting manual:**

> Re-entering the factory DATA origin codes **does not** restore the origin
> after a battery loss. The **ABS method** is required.

This cost significant time. The DATA codes were entered correctly several
times and the joints still read 22676 degrees. The ABS method using the
alignment marks on the arm is the documented recovery path.

Origin setting methods available on the CR1-571:

| Method | Use |
|---|---|
| **DATA** | Factory codes, for a controller swap with the same arm |
| **ABS** | Alignment marks on the arm. **Required after battery loss** |
| MECH | Mechanical stopper |
| JIG | Dedicated jig |
| USER | User-defined |

### 2.2 Character entry errors

The DATA codes are case and character sensitive. Two errors were made and
caught:

- `D` entered as `001EY8` instead of `001DY8`
- `J5` entered as `Z2UZI$` instead of `Z2UZI&`, which produced error **C1760**
  (checksum error)

If you get C1760, the codes are mistyped. Check every character.

### 2.3 R28TB pendant key combinations

Several pieces of common advice turned out to be wrong for this pendant.
**CONFIRMED from the manual:**

| Action | Keys |
|---|---|
| Decrease speed | `STEP/MOVE` + `BACKWD` |
| **Increase speed** | `STEP/MOVE` + `FORWD` |
| Save a position | `STEP/MOVE` + `ADD` held together |
| Step feed forward | `+/FORWD` + `INP/EXE` |

The R28TB has **no OVRD up/down keys**. Advice referencing them is for a
different pendant.

### 2.4 Serial link

**CONFIRMED working.**

| Parameter | Value |
|---|---|
| Port | `/dev/ttyUSB0` via USB-RS232 adapter |
| Physical | Front **RS-232C** connector on the CR1-571 |
| Baud | 9600 |
| Data bits | 8 |
| Parity | **Even** |
| Stop bits | **2** |
| Controller settings | `COMDEV = RS232`, `CPRCE11 = 0` |
| Pendant | Must be **disabled** |
| Controller mode | **AUTO (Ext)** |

Frame format:

```
1;1;<COMMAND>\r
```

Replies:

| Reply | Meaning |
|---|---|
| `Qo` | OK |
| `Qe` | Error |
| `Qx` | Error |

### 2.5 Confirmed serial commands

Verified by readback on the physical hardware:

| Command | Purpose |
|---|---|
| `CNTLON` | Take control rights |
| `CNTLOFF` | Release control rights |
| `SRVON` | Servo on |
| `SRVOFF` | Servo off |
| `EXECJOVRD <n>` | Set joint override percentage |
| `EXECM_OUT(n)=1` | Set output bit n |
| `VALM_INB(0)` | Read input byte 0 |
| `VALM_OUTB(0)` | Read output byte 0 |

**Observed I/O state, read-only survey of 2026-10-16.** A full read of every
general-purpose and hand I/O byte returned the following (raw controller
replies preserved in `io_survey_2026-10-16.txt`):

| Byte | Command | Reply value | Bits set |
|---|---|---|---|
| Input byte 0 | `VALM_INB(0)` | `+16` | IN4 |
| Input byte 8 | `VALM_INB(8)` | `+170` | IN9, IN11, IN13, IN15 |
| Output byte 0 | `VALM_OUTB(0)` | `+0` | none |
| Hand output byte 900 | `VALM_OUTB(900)` | `+86` | 901, 902, 904, 906 |

All other general-purpose bytes (inputs/outputs 16, 24, 32) read zero, and the
hand **input** byte 900 read zero. The four hand outputs that are set
(901, 902, 904, 906) are one side of each pneumatic valve pair, i.e. the
gripper's rest state. The active signals therefore sit only on input byte 0,
input byte 8 and the hand bytes; everything else is idle. `VALM_INW(0)` read
back `−22000`, the word form of the same byte-0 state plus the sign the
controller applies to the high word.

**Output write permission, CONFIRMED.** On 2026-09-25 the controller
**rejected** `EXECM_OUT(0)=1` with reply `Qer515000000`: the general-purpose
outputs OUT0–OUT3 were not writable in the controller's parameter set. After the
operator opened the relevant parameter on 2026-10-16, OUT0–OUT3 became settable
alongside OUT4–OUT7 (written one bit at a time and read back to confirm). The
robot's station handshake itself uses only OUT4–OUT7 (section 3.2).

### 2.6 Error codes encountered

| Code | Meaning | Resolution |
|---|---|---|
| H0070 | Ionisation / encoder | |
| H1121 | Battery / encoder backup | Replace controller batteries |
| C7520 | Battery low | Replace, then reset battery timer |
| C1760 | Checksum error on origin data | Mistyped DATA code |
| L3282 | Position out of range | |
| L5150 | | |
| **5640** | **Cannot execute during RUN** | See 2.8 |

### 2.7 Conveyor control: a definitive negative result

**The robot has no conveyor start/stop signal.**

This was established exhaustively, not assumed:

- The robot's SysLink cable goes to `XPA4` on the cell
- Two complete passes through all 16 states of outputs OUT4 to OUT7 were
  executed over the serial link
- `M_INB` returned `+16` constantly throughout, with zero change
- Conclusion: there is no path from the robot to the conveyor drive

Do not spend time on this. The conveyor runs continuously when the cell is in
AUTO and is not under robot control.

### 2.8 Error 5640 and continuous motion

**Problem:** the robot stops at every taught point, making multi-point moves
slow.

**Root cause, CONFIRMED:** when motion commands are sent one at a time over
RS-232 using `EXEC`, the controller executes exactly that one line and has no
knowledge of the next point. It must decelerate to zero. This is also the
source of error 5640 "Cannot execute during RUN".

**Solution:** the path must be a **program resident in the controller**, not a
stream of single commands from the PC. Only then can the controller see the
next point while still moving and blend the corner.

MELFA-BASIC IV instructions involved, syntax CONFIRMED from the Mitsubishi
command list:

| Instruction | Syntax | Effect |
|---|---|---|
| `CNT` | `CNT 1,100,200` | Smooth / continuous operation. `CNT 0` disables |
| `FINE` | `FINE 200` | Positioning completion condition, in pulses |
| `ACCEL` | `ACCEL 50,80` | Acceleration and deceleration as a percentage |
| `OVRD` | `OVRD 100` | Speed override as a percentage |
| `SPD` | `SPD 123.5` | Linear and circular speed in mm/s |

Example program structure:

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

**Trade-off:** with `Cnt 1` the robot rounds corners and does not pass exactly
through the intermediate points. Use `Cnt 0` before any pick or place position
where accuracy matters.

**Note:** the exact meaning of the two numeric arguments to `CNT` is not
confirmed from the sources consulted. Start with `Cnt 1` alone.

### 2.9 Discipline on unverified syntax

MELFA-BASIC IV has **no JOG instruction**. This was confirmed from the manual
after repeated requests to implement jogging from a GUI. Rather than guess at
motion command syntax, an empirical syntax-finder script was written that:

- Is dry-run by default and requires `--arm` to move anything
- Forces `EXECJOVRD 5` (5 percent speed)
- Uses a 2 degree step on J1 only
- Pauses for operator confirmation before each candidate

**Never guess robot motion syntax.** A wrong command on a 5-axis arm is not a
compile error, it is a collision.

### 2.10 Host-side teach-and-repeat and sensor-triggered automation

Given the continuous-motion limit of section 2.8, the robot is run as a
**host-side teach-and-repeat** cell rather than a resident-program cell. A
sequencer on the PC drives the arm point-to-point over the serial link, each
taught point carrying joint angles, a gripper state, a **per-point speed**
(a "slowest-of-two" override on each transition) and **per-point outputs** that
fire the station's pneumatics on arrival. The per-point dead time was cut by
removing host-side redundancy (unchanged gripper/override re-commands,
all-bytes I/O polling), but the physical stop at each point remains because
blending is unavailable over serial (section 2.8).

**Per-point speed, CONFIRMED.** Each taught point stores its own override; the
transition between two points is run at the **slower of the two endpoints'
overrides** (a "slowest-of-two" rule), so a slow point slows both its approach
and its departure. Two presets are exposed, Slow = 10 % and Fast = 40 %. Home
has no taught speed of its own and is given a fixed transit value of 40 %. Only
the speed override (`EXECJOVRD`) is varied; `ACCEL` is **not**, because the
controller did not accept it over this serial link (section 2.8), so
acceleration shaping is deliberately out of scope.

**Per-point dead time.** The dead time first observed at each point (~1.4 s)
came not from motion but from host-side redundancy: re-reading four higher I/O
bytes at every point, re-commanding an unchanged gripper (with its ~0.6 s
settle), and re-sending an unchanged override. Suppressing the redundant
commands, and reading the higher I/O bytes round-robin while moving (all of them
at once only when the arm is idle), cut it to ~0.4 s. These two figures are
**coarse operator observations, not instrumented measurements.** The residual is
the physical deceleration to a full stop, which cannot be removed without `Cnt`
blending (section 2.8).

**Sensor-triggered single-pass automation.** On top of this, the cell runs **one
assembly pass per part**. A part sensor mounted on the robot's tool and wired to
input **IN1** triggers a pass. Because the sensor rides with the arm, a reading
is only meaningful at the park pose, so the trigger is gated on the arm being
**idle and within 10° (`AUTO_HOME_TOL_DEG`) of the home pose** on every fitted
joint. Home was moved on 2026-10-21 to the pick/wait pose beside the parts; it
coincides with the taught point **P2E**. A pass therefore starts at the point
**after P2E** and runs to the last taught point, then returns home — it does not
restart from the top or wrap into a continuous loop. The trigger is the **rising
edge** of *(idle ∧ at-home ∧ sensor)*: the arm leaves home during the pass, so
readings taken mid-pass (the sensor moving through the air) are ignored, and
returning home re-arms for the next part, serving a queue of parts one pass at a
time without any explicit queue. Servo power is enabled only by a human (the
automation never enables it); arming requires the servo on and at least one
taught point; **STOP disarms** the automation; and it starts disarmed on every
restart. This is the part-level reactivity a serial-only controller can
support — one trigger per pass, not a control loop closed at the waypoint rate.
The full method, state machine and safety model are in the companion
[`ROBOT_CONTROL.md`](ROBOT_CONTROL.md) section 8 and in the main repository
README.

---

## 3. The Festo MPS 500 cell and the dead master

### 3.1 Diagnosis of the E.IPC

The Festo E.IPC master controller is **dead and not recoverable** without
specialist hardware or service. Evidence:

- VGA output shows the ROM-DOS boot screen but reports
  `Configuration: unknown (please run setup)`
- Packet driver fails to load
- `dir` returns `Access error`
- `?` returns `Access error`
- Every command attempted returns an access error
- FST software was obtained and installed, but the controller cannot be
  reached over its COM port or over Ethernet

Attempts made and failed: VGA plus keyboard console, Ethernet via the 10BaseT
port, access through the robot's network connection.

### 3.2 The station communication plan

**CONFIRMED from the cell's own documentation** (photographed from the binder).
This is the robot's Station 4 handshake:

| Direction | Signal | Bit |
|---|---|---|
| Station to robot | Order bit 0 | OUT2 to robot IN6 |
| Station to robot | Order bit 1 | OUT3 to robot IN7 |
| Robot to station | Error code 0 | robot OUT4 to IN0 |
| Robot to station | Error code 1 | robot OUT5 to IN1 |
| Robot to station | Order active (busy) | robot OUT6 to IN2 |
| Robot to station | Station ready | robot OUT7 to IN3 |

Order encoding:

| Order bits | Meaning |
|---|---|
| 01 | Assemble |
| 10 | Sort out |
| 11 | Take from pallet |

Four-phase handshake:

1. Set the order bits
2. Wait for the robot's Busy signal to rise
3. Clear the order bits
4. Wait for Busy to fall, then read the error code bits

### 3.3 SysLink convention, for reference

Festo's transfer-system SysLink interface splits one 8-bit connector as:

- Bits 0 to 3: operation panel signals
- Bits 4 to 7: conveyor module signals

Useful as a starting hypothesis. Confirm every bit against hardware.

---

## 4. The S7-1200 migration

### 4.1 Scope

Replace only the dead E.IPC master. Keep the AS-i bus, the station slaves, the
MICROMASTER drive, the Pilz safety chain and the robot cell.

The economic argument for the project is exactly this: one controller is
replaced, not the whole cell.

### 4.2 Terminal map, CPU 1214C DC/DC/DC

**Top strip X10**, left to right:

```
 1    2    3  │  4    5   │  6   │  7 ... 14      │  15 ... 20
┌────┬────┬───┼────┬─────┼──────┼────────────────┼────────────┐
│ L+ │ M  │ ⏚ │ L+ │ M   │ 1M   │ DI a.0 .. a.7  │ DI b.0..b.5│
└────┴────┴───┴────┴─────┴──────┴────────────────┴────────────┘
  supply in      sensor      DI
                 supply    common
                  out
```

**X11:** `2M`, `AI 0`, `AI 1`

**Bottom strip X12**, left to right:

```
 1     2    │ 3 ... 10        │ 11, 12
┌─────┬─────┼─────────────────┼─────────┐
│ 3L+ │ 3M  │ DQ a.0 .. a.7   │ DQ b.0,1│
└─────┴─────┴─────────────────┴─────────┘
 output stage
    supply
```

### 4.3 Power wiring

| From | To | Terminal |
|---|---|---|
| Mains L | Delta `L` | |
| Mains N | Delta `N` | |
| Earth | Delta `⏚` | |
| Delta `24V` | CPU top strip pos 1 | `L+` |
| Delta `0V` | CPU top strip pos 2 | `M` |
| Earth | CPU top strip pos 3 | `⏚` |
| Delta `24V` | CPU bottom strip pos 1 | `3L+` |
| Delta `0V` | CPU bottom strip pos 2 | `3M` |

### 4.4 The two terminals that cause most first-day failures

**`1M` must be tied to 0 V.** It is the common for all 14 digital inputs.
With PNP sensors the inputs are sourcing, so `1M` goes to 0 V. Left floating,
every input reads false forever.

```
jumper: top strip pos 5 (M) ──> top strip pos 6 (1M)
```

**`3L+` and `3M` must both be powered.** They supply the output transistors.
Without them the output terminal reads 0 V **even though the `DQ a.0` LED is
green**. This symptom cost several hours.

### 4.5 Power budget

| Load | Current |
|---|---|
| CPU 1214C, no expansion | up to 500 mA |
| Inductive sensor | approx 10 mA |
| Festo valve coil (1.28 W) | 53 mA |
| **Total** | **approx 560 mA** |

Against the Delta DVP-PS01's 1 A. Adequate but not generous. The DVT camera
needs 7.5 W on its own and must have a separate supply.

### 4.6 Network

| Device | IP |
|---|---|
| S7-1200 | `192.168.0.1 / 255.255.255.0` |
| Programming PC | `192.168.0.10 / 255.255.255.0` |
| DVT camera | `192.168.10.242 / 255.255.255.0` |

The CPU has **no IP address from the factory**. It is found by MAC through
**Online access, Update accessible devices**, which works at layer 2, then
assigned an IP through **Online and diagnostics, Functions, Assign IP address**.

### 4.7 Software constraint

TIA Portal is **Windows only**. There is no Linux build and Wine does not carry
it. On Ubuntu the practical route is a Windows VM with a **bridged** network
adapter. NAT breaks the accessible-devices scan, which runs below IP level.

From Linux directly it is possible to:

- Discover the CPU and assign its IP using `pnio-dcp` (PROFINET DCP, needs root)
- Read and write memory with `python-snap7`, **but only after** "Permit access
  with PUT/GET communication" has been enabled once in TIA Portal

There is no open-source toolchain that can download a program to an S7-1200.

---

## 5. Sensor and actuator bring-up

### 5.1 Inductive proximity sensor, 3-wire PNP

| Wire | Terminal | Position |
|---|---|---|
| Brown | `L+` (sensor supply) | top strip 4 |
| Blue | `M` | top strip 5 |
| Black | `DI a.0` = `%I0.0` | top strip 7 |
| jumper | pos 5 to pos 6, `M` to `1M` | |

Verification with a meter, black probe on `M`:

- Brown should read **+24 V**
- Black should read **0 V** at rest and **+24 V** with metal in front

If black sits at 24 V and drops to 0, the sensor is **NPN**. Then `1M` goes to
`L+` instead and the logic inverts.

### 5.2 Cylinder position sensor, 2-wire reed

The Festo SME-8 is a **mechanical contact in series**, not a 3-wire sensor.

```
+24 V ────── sensor ────── DI a.1 (%I0.1)
```

| Wire | Terminal |
|---|---|
| Black | `L+`, top strip 4 |
| Blue | `DI a.1` = `%I0.1`, top strip 8 |

If it does not respond, swap the two wires. The internal LED and protection
diode make it polarity sensitive, but reversing causes no damage.

A reed switch has **no leakage current** when open, so unlike an electronic
2-wire sensor it needs no bleeder resistor on an S7-1200 input.

Voltage drop across the sensor is about 3.5 V, leaving around 20.5 V at the
input. The 1214C needs 15 V minimum for a logic 1, so there is margin.

### 5.3 Pneumatic valve

**Critical distinction that cost time:** a pneumatic cylinder has **no
electrical connection**. It has air ports and, optionally, position sensors
clipped to its barrel. The thing the PLC drives is the **solenoid valve**.

```
PLC %Q0.0 ──electric──> VALVE ──air tube──> CYLINDER
                                                │
                           sensor on cylinder <─┘
                                   │
                              PLC %I0.1
```

Direct drive, no relay:

| From | To |
|---|---|
| Delta `24V` | `3L+`, bottom strip 1 |
| Delta `0V` | `3M`, bottom strip 2 |
| `DQ a.0` = `%Q0.0` | valve coil terminal 1 |
| Delta `0V` | valve coil terminal 2 |

The MSZE coil is 1.28 W, 53 mA against a 0.5 A output. No polarity.

### 5.4 Via an interface relay

Used because the Festo valve's moulded plug could not be rewired in place.

Weidmüller **TRS 24VDC 1CO** terminals:

| Terminal | Function |
|---|---|
| `A1+` | coil positive |
| `A2−` | coil negative |
| `11` | COM |
| `12` | NC |
| `14` | NO |

```
PLC SIDE (coil)                   LOAD SIDE (contact)

%Q0.0  ──────> A1+                Delta 24V ────> 11 (COM)
3M     ──────> A2−                           14 (NO) ──> valve wire 1
                                  Delta 0V <──────────── valve wire 2
```

The coil has a built-in free-wheeling diode and LED, so **polarity matters**.
A1 must be positive.

**The relay does not remove the need for `3L+` and `3M`.** The output still
has to source current into the relay coil.

### 5.5 Pneumatic notes

| Item | Value |
|---|---|
| CPE10 operating pressure | **3 to 8 bar**. It will not shift below 3 bar |
| AEVUZ-16-10 | **Single-acting**, one air port |
| AEVUZ `Z` suffix | **Pulling** variant. Air retracts, spring extends |
| AEVUZ `A` suffix | Magnetic piston, for position sensing |
| AEVUZ pmax | 10 bar |

**Safety consequence:** with a single-solenoid spring-return valve, loss of
power or a CPU STOP drops the output and the cylinder springs to its rest
position. With the `Z` pulling variant that means the rod **extends** when
everything is off. Confirm this is the safe direction before pressurising.

### 5.6 Diagnostic ladder

When the valve does not move, work outward from the CPU. Each step is
measurable.

1. **`DQ a.0` LED lit?** No means the CPU is not in RUN, or the program did
   not download, or the tag is wrong.
2. **24 V between `DQ a.0` and `3M`?** No means `3L+` and `3M` are not powered.
3. **24 V at the coil?** No means a broken wire or the return is not on 0 V.
4. **Does it click, air off?** No, with 24 V present, means the coil is open.
   Measure its resistance, expect about 450 ohm.
5. **Manual override with air on.** This is the single best test. It splits
   electrical from pneumatic in one action.

Pneumatic checklist: pressure at or above 3 bar, shut-off valve open, port 1
of the valve fed, tubes pushed fully home, exhaust throttles open.

---

## 6. PLC program

### 6.1 Tag table

| Name | Data type | Address | Comment |
|---|---|---|---|
| Sensor | Bool | `%I0.0` | Inductive proximity, 3-wire PNP |
| CylSensor | Bool | `%I0.1` | Cylinder end-of-stroke reed |
| Valve | Bool | `%Q0.0` | Festo valve, or interface relay coil |
| SensorStable | Bool | `%M10.0` | Debounced detect |
| NewPart | Bool | `%M10.1` | One scan pulse per part |
| Cmd_ResetCount | Bool | `%M10.2` | Set to 1 to clear the counter |
| EdgeMem | Bool | `%M10.3` | Edge detection memory |
| ValveLatch | Bool | `%M10.4` | Stroke latch |
| PartCount | Int | `%MW14` | Parts counted |

### 6.2 SCL

See [`plc/SensorRead.scl`](plc/SensorRead.scl). Import through
**External source files, Add new external file**, then right click and
**Generate blocks from source**.

The function block does three things:

**Debounce, both directions.** A proximity sensor on a moving conveyor
chatters as the edge of a part crosses its field. Without debouncing, the
counter increments three or four times per part.

```scl
#tOn (IN := #SensorRaw,     PT := #DebounceTime);
#tOff(IN := NOT #SensorRaw, PT := #DebounceTime);

IF #tOn.Q  THEN #Stable := TRUE;  END_IF;
IF #tOff.Q THEN #Stable := FALSE; END_IF;

#Detected := #Stable;
```

**Edge detection.** `NewPart` is TRUE for exactly one scan. Trigger downstream
actions on this, never on the level, or they fire every scan while the part
sits there.

```scl
#rtDetect(CLK := #Stable);
#NewPart := #rtDetect.Q;
```

**Count, with reset winning over count.**

```scl
IF #ResetCount THEN
    #PartCount := 0;
ELSIF #NewPart THEN
    #PartCount := #PartCount + 1;
END_IF;
```

### 6.3 Ladder equivalent

OB1 is LAD by default in TIA Portal, so this can be typed directly.

**Network 1, debounce**

```
    "Sensor"           "T_Debounce"
     %I0.0                 TON
  -----| |--------------[ IN    Q ]--------( )------
                        [  Time    ]   "SensorStable"
         T#20ms --------[ PT   ET ]        %M10.0
```

**Network 2, one-shot**

```
  "SensorStable"    "EdgeMem"         "NewPart"
     %M10.0          %M10.3            %M10.1
  -----| |------------| P |-------------( )------
```

**Network 3, count**

```
    "NewPart"            "C_Parts"
     %M10.1                 CTU
  -----| |---------------[ CU    Q ]
                         [  Int     ]
   "Cmd_ResetCount"      [          ]
     %M10.2              [          ]
  -----| |---------------[ R    CV ]------- "PartCount" %MW14
          9999 ----------[ PV      ]
```

**Network 4, start the stroke**

```
    "NewPart"                       "ValveLatch"
     %M10.1                            %M10.4
  -----| |----------------------------( S )------
```

**Network 5, end the stroke**

```
  "ValveLatch"        "T_Stroke"           "ValveLatch"
     %M10.4              TON                  %M10.4
  -----| |------------[ IN    Q ]-------------( R )-----
                      [  Time    ]
          T#1s -------[ PT   ET ]
```

**Network 6, drive the valve**

```
  "ValveLatch"         "Valve"
     %M10.4             %Q0.0
  -----| |---------------( )------
```

**Improvement once the cylinder sensor is positioned:** replace Network 5 with
a reset on `CylSensor`, so the valve releases when the cylinder actually
arrives rather than when a timer expires. Keep the timer alongside as a
timeout, because a timer that expires with no arrival means a jam or low air,
and that is worth knowing.

### 6.4 Tuning

| Symptom | Fix |
|---|---|
| `PartCount` jumps by several per part | Raise `DebounceTime` to `T#50ms` |
| Parts missed on a fast line | Lower it to `T#5ms` |

---

## 7. The DVT Legend 510MR vision sensor

### 7.1 Background

DVT was acquired by Cognex in 2005. This is roughly 2004 hardware. It is a
self-contained vision computer, not a webcam: inspections are configured from
a PC, then it runs standalone and reports Pass, Fail, Warn and Busy.

### 7.2 Discovery from Ubuntu, CONFIRMED

All of the following is backed by the captured output in `dvt_legend/`
(`step*.txt`, `boot.pcap`); see [`dvt_legend/findings.md`](../dvt_legend/findings.md).

**The link.** The laptop's only wired port is `enp2s0` (r8169 driver, MAC
`bc:ec:a0:1e:2e:fe`), carrier up at 100 Mb/s full duplex (`ethtool`). Before
probing it had no IPv4 address — NetworkManager was stuck in "connecting
(getting IP configuration)", i.e. DHCP with no answer, which is consistent with
a device that does not run a DHCP server.

**The camera has no fixed factory IP** and self-assigns a link-local address, so
an ordinary ping sweep finds nothing and the device looks dead.

**What a no-root scan could and could not prove.** Without passwordless sudo
(and with `arp-scan`/`nmap` not installed), a laptop-side-only probe was run
from `169.254.1.10/16` with an on-link route to `192.168.0.0/24`:

- ICMP + kernel ARP to `192.168.0.2..254` (excluding `.1`): **0 replies,
  253/253 ARP entries FAILED**.
- Broadcast pings to `169.254.255.255` and `255.255.255.255`: **0 replies**.
- The `enp2s0` RX counter held at **241461 packets before and after** a ~2 min
  window, while TX rose — so **the camera transmitted nothing at all** over that
  time: no ARP replies, no broadcasts, no announcements.

This is CONFIRMED to rule out an answering IP in `192.168.0.2–254`, but it
**cannot** reveal an address in any other subnet, because ARP only answers for
an exact IP. (A first attempt was discarded as INVALID: GNOME Shell tore down the
probe profile 3 s after activation, so those packets went out with no source
address.)

**The boot capture that found it.** With the operator running `tcpdump` as root
and power-cycling the camera (`boot.pcap`), it announced itself with two
gratuitous ARPs:

```
00:90:68:00:f6:12 > ff:ff:ff:ff:ff:ff  ARP who-has 169.254.246.18 tell 169.254.246.18
... 14 s later ...
                                        ARP who-has 192.168.10.242 tell 192.168.10.242
```

OUI `00:90:68` resolves to **DVT CORP**. Giving the laptop `192.168.10.10/24`
then reached the camera: `ping 192.168.10.242` → **3/3 replies, 0.37 ms**, and
the neighbour table showed `192.168.10.242 lladdr 00:90:68:00:f6:12 REACHABLE`.
The link-local `169.254.246.18` stopped answering once the routable address came
up. **Camera = 192.168.10.242, MAC 00:90:68:00:f6:12.**

**Method, generalised:** run `tcpdump` as root, then power-cycle the device.
Embedded network stacks announce themselves at boot, which reveals both MAC and
IP even on an unknown subnet. This is worth remembering for any undocumented
networked device.

### 7.3 Open ports, CONFIRMED

`nmap -Pn -sT -p- --reason -T4` (a TCP connect scan; `-sS` needs root) found
**7 ports open and 65528 closed** (connection-refused), i.e. a full 1–65535
sweep with no filtered ports:

| Port | nmap name | Passive banner (connect, send nothing, 5 s) | `nmap -sV` behaviour |
|---|---|---|---|
| **502** | mbap (Modbus TCP) | none | none |
| 3246 | dvt-system | `System Terminal Driver\r\n?` | same banner; a bare `\r\n` gives another `?` prompt |
| 3247 | dvt-data | `1\r\n` | replies `1\r\n` per probe line |
| 4883 | (wmlserver) | none | none |
| 5000 | (upnp) | `System Terminal Driver\r\n?` | identical to 3246 |
| 5001 | (commplex-link) | `1\r\n` | identical to 3247 |
| 7500 | (silhouette) | 64 bytes: `04 00` + 62 zero bytes | repeating 64-byte records `04 00 NN 00 ...`, NN counting 00..08 |

Ports 5000/5001 are the "older systems" aliases of 3246/3247 (section 7.4), so
the camera exposes the System terminal and DataLink on both the new and the old
port numbers at once.

**Closed:** 21 (FTP), 23 (telnet), 80 (HTTP) — so there is no FTP image server
and no web interface. **UDP** (`nmap -sU --top-ports 100`, run as root): 99
closed (port-unreachable), **5000/udp open|filtered** (no reply, so open and
filtered cannot be distinguished). UDP 5000 is what the vendor software uses for
network discovery.

### 7.4 Protocol, recovered from the vendor installer (CONFIRMED)

The protocol was not guessed: the **FrameWork 2.8.0** installer was obtained from
Cognex and **unpacked without running it**. Its ~33 MB Wise overlay is a series
of back-to-back raw-deflate streams, which `carve_wise.py` inflated directly
(7-Zip cannot open a Wise overlay). The carve yielded, among others, two PDFs and
six `.chm` help files (see
[`dvt_legend/protocol.md`](../dvt_legend/protocol.md) for file:line of every
item):

- **UG** — "FrameWork Installation and User Guide for DVT Vision Sensors", 9th
  ed., June 2007.
- **SR** — "DVT Script Reference Manual", 5th ed., May 2007.

Facts quoted from those documents:

| Item | Source | What it says |
|---|---|---|
| System terminal | UG:2266, :4764 | port **3246** (new systems) / **5000** (older) accepts commands "without using FrameWork" |
| DataLink output | UG:4660 | user-configured strings sent over TCP to **3247** (new) / **5001** (old); "in some cases, no string will be sent out" |
| Modbus | UG:4680 | the sensor can be a **Modbus slave on port 502**; the master chooses read or write |
| VDX output stream | SR:5385 | 64-byte telegrams on **port 7500**, one **after every inspection**, cyclic; one client at a time; enabled by default |
| Output telegram (camera → client) | SR:5396-5402 | **byte 0:** bit0 Pass, bit1 Warn, bit2 Fail, bit3 Wrong Code (4-7 reserved); **byte 1:** error code (1 product-select failed, 2 save-product error, 3 trigger error); **byte 2:** cycle count; **byte 3:** user bits 1-8; bytes 4-63 script data |
| Input telegram (client → camera) | SR:5404-5412 | byte 0: bit0 Trigger (acts on the **rising edge**), bit1 Prod. Select, bit2 Relearn, bit3 Save to flash, bit4 Reset, bit5 Trigger mode (0 internal / 1 external), bit6 Run mode; bytes 2-3 product ID |
| Terminal `#Rs` | help chm 0041:149 | writes Registers — a WRITE, **not used** |
| Terminal reply codes | help chm 0045 | reply ends `% <code>` then `?` (success) or `!` (error); codes: 0 ok, 1 general error, 2 access denied, 3 out of memory, 4 flash error, 5 network error, 6 invalid parameter, 7 locked |

**Not found in the vendor documents:** the command `#YI` (zero hits in the UG,
the SR and all six `.chm` files — see section 7.7); the full list of `#`/`*`
system-terminal commands (the UG defers to "the FrameWork help files", which in
this installer do not carry the list); any FTP port or procedure (matching the
closed port 21); and any image-over-Ethernet protocol (image transfer is
described only as a FrameWork option, UG:2295).

Installers are hosted by Cognex:
[Intellect 1.5.1](https://support.cognex.com/en/downloads/detail/dvt/689/1033),
[FrameWork 2.8.0](https://support.cognex.com/en/resources/software-firmware/installfwk280/english/pP2fPnDISgejyYU8VWvYNw).

### 7.5 Modbus TCP: mapped read-only, with a negative result

**Modbus TCP is open on port 502**, a published standard rather than a
proprietary protocol, so the register space was mapped read-only with
`pymodbus` 3.16.1 (`read_*` calls only, unit/device id 1, addresses 0–255 for
FC1–FC4). Findings (CONFIRMED,
[`dvt_legend/modbus_map.md`](../dvt_legend/modbus_map.md)):

- **Every address 0–255 answers on all four function codes, no exception.** FC1
  (coils) == FC2 (discrete inputs) and FC3 (holding) == FC4 (input): the camera
  mirrors them.
- **Static at idle:** bits 32, 52, 58, 73 = 1; register 2 = 256 (`0x0100`),
  register 4 = 2; everything else 0.
- **Free-running:** register 3 and bits 53, 54, 55, 59 cycle about once per
  second (reg 3 steps `0x1004` → `0x9004` → a burst value → back), in step with
  the ~1 Hz inspection rate. HYPOTHESIS: a heartbeat / rotating I/O-task status,
  not a result.
- **Negative result (CONFIRMED):** no address differed between a baseline read
  and a part-under-the-lens read, and none changed around a trigger. **No
  Pass/Fail signal was located in 0–255 on unit 1.** (Not tried: addresses above
  255, other unit ids.) The documented result channel is instead the VDX output
  stream on port 7500 (section 7.6).

The architecture argument still holds in principle — the **S7-1200 speaks
Modbus TCP natively** through `MB_CLIENT`, so a camera result could reach the
PLC over Ethernet with no digital I/O wiring, no BOB terminals and no 24 V
handshake lines:

```
S7-1200 ──[ MB_CLIENT over Ethernet ]──> camera 192.168.10.242:502
```

But on this camera, as currently configured, that path needs a result register
to be mapped first (via FrameWork), because none was found in the default range.
Both devices would also have to be on the same subnet.

### 7.6 The VDX output stream: the result channel (CONFIRMED)

Reading port 7500 receive-only (`dvt_result.py`) confirmed the documented
64-byte telegram live:

- The camera runs **~1 inspection per second** on its own internal trigger.
- Every telegram is `04 00 NN 00 ...`: byte 0 = `0x04` = the **Fail** bit,
  byte 1 = 0 (no error), byte 2 = `NN` a **cycle counter** incrementing once per
  second, bytes 3–63 = 0. So the result is **FAIL on every inspection**, with or
  without a part under the lens; which product is loaded and why it fails is
  UNKNOWN without FrameWork.
- Passive 60 s capture: exactly 60 records of 64 bytes, one per second. With a
  part under the lens: 57 records, only the counter byte differs — so **the
  stream does not react to the scene** (CONFIRMED).
- Ports 3247/5001 emit `1\r\n` once per second (a DataLink string configured as
  "1" per inspection); a `#YI` on 3246 at t = 40 s produced no change on either.
  They are heartbeats / keepalives, **not** a result channel.

### 7.7 The system terminal and the `#YI` correction

The system terminal on 3246 takes `#CMD args<CR>`; replies end `% <code>` then
`?` (success) or `!` (error). An early note (from an IAI integration document,
not independently verified) treated `#YI` as the inspection trigger, and the
terminal did answer it `% 1`, `!`.

Reading the recovered reply-code table (section 7.4) corrected this: **`% 1` is
a general error and `!` is the error prompt**, so `#YI` → `% 1`, `!` is an
**error, not an accepted trigger**; `#` alone → `% 6` (invalid parameter) and
`*` → `)06` are errors too. `#YI` appears in none of the vendor files, so
triggering an inspection from Linux via the terminal is **UNKNOWN**. The
documented alternative is the VDX **input** telegram (byte 0 bit 0 Trigger, on
the rising edge, section 7.4) sent to port 7500; it writes control bits and was
**not tried** (it needs approval). With the internal trigger already producing
~1 inspection/s, an explicit trigger may be unnecessary.

Discipline point: a device accepting a command string is not the same as the
command being valid. The reply code has to be read, not assumed.

### 7.8 Image retrieval and the no-lens root cause (CONFIRMED)

With no FTP or HTTP service, image transfer turned out to be available through
the System terminal's documented image commands (vendor reference
`Image_Management_Commands.htm` and companions). Read-only decimal-mode queries
on 3246 (`#CMD args<CR>`, replies are ASCII `# <packet type> <id> ...`):

| Command | Result seen |
|---|---|
| `#I` | `FrameWork Version 2.7.4 (Build 14012005)` — the camera's firmware |
| `#IC` | one **DVT520** sensor, **640×480** |
| `#PQ` | products: 1 **FMS50**, 2 **train**, 3 **murat** |
| `#PQ 1` | product 1: exposure **8300**, antiblooming 0, illumination **ON**, gain **128**, full window |
| `#Ii` | inspection statistics (2037 total at first read, none missed) |
| `#IQ` | the 8 newest saved image ids (2084..2091, rotating) |
| `#IS <id> 0 0 639 479 0 1 1 0 0` | the full image as type-16 (header), type-20 (name) and type-18 (data) packets, ending `% 0`, `?` |

A full 640×480 frame (307200 pixels) takes ~21.7 s, limited to ~70 KB/s by the
camera's terminal rather than by the network; `#Is` with no arguments returns
only a 1-pixel window.

The first frames were **saturated**: image 2091 had 99.97 % of pixels at 255
(min 242), a flooded sensor rather than a broken transfer. The exposure was then
lowered, **operator-authorised and in RAM only**, with the documented
`#PP 1 E <us>`:

| Exposure (µs) | Image (214×160 window) |
|---|---|
| 8300 (original) | 100 % saturated, min 242 |
| 1000 | min 0, mean 15, max 43 (too dark) |
| 3000 | min 38, mean 83, max 128, 0 % saturated (good) |

At 3000 µs the image was a **uniform grey with a mild vertical gradient and no
objects**. The operator then confirmed the **root cause of "no picture": no lens
is fitted; the sensor is bare.** That explains every observation — the smooth
illumination gradient with no scene detail, the frame-to-frame difference equal
to sensor noise, the sharp dark specks that are dust on the cover glass (the only
thing "in focus" without a lens), and the strobe doubling the brightness
(mean 83 with strobe vs 40 without). Row-major decoding was verified correct
(column-major gives obvious banding).

**Conclusion (CONFIRMED):** the Linux image path — `#IS` on port 3246 — works
**end to end**; it only needs a C/CS-mount lens to show a scene. The camera was
left with product 1 at exposure 3000 µs, strobe ON, **in RAM only** (a power
cycle restores the flash value of 8300 µs). Every interaction with the camera in
this project was **read-only, except the operator-authorised, RAM-only exposure
change** (logged in `exposure_changes.log`).

### 7.9 Wiring fallback: digital I/O instead of Ethernet

If the Ethernet result path is not used, the Legend has **8 configurable I/O
lines**, each an input or an output. **Pin 8 is reserved for strobe
illumination**, so use lines 1 to 7.

| Direction | Camera function | PLC |
|---|---|---|
| PLC to camera | Inspection Trigger (input) | `%Q0.1` |
| Camera to PLC | Pass (output) | `%I0.2` |
| Camera to PLC | Fail (output) | `%I0.3` |
| Camera to PLC | Busy (output) | `%I0.4` |

The BOB is isolated, so each channel has its own `+` and `−` pair. For camera
outputs feeding the PLC, tie each channel's `−` to the PLC `0V`.

---

## 8. Open problems

| # | Problem | Status |
|---|---|---|
| 1 | Complete AS-i I/O map | Not started. Needs a bus scan plus physical tracing, or the Part C circuit diagram from the Festo binder |
| 2 | Does the robot's Station 4 interface sit on AS-i or on the old E.IPC local I/O? | Unknown. Decides whether an SM 1223 is needed |
| 3 | What drives the MICROMASTER 420 digital inputs? | Unknown. Check P0701 to P0703 on the drive keypad before wiring anything |
| 4 | Robot continuous motion | Diagnosed and **mitigated**: run as a host-side teach-and-repeat cell with per-point speed/I/O and sensor-triggered single passes (section 2.10). A fully *blended* path still needs a controller-resident program |
| 5 | Serial commands to upload or run a program on the CR1 | Not established. Must be read from the manual or discovered empirically, never guessed |
| 6 | DVT Modbus register map | **Mapped read-only** (addresses 0–255, unit 1): no Pass/Fail register found (section 7.5). A result register would have to be configured in FrameWork, or the result read from the VDX stream on port 7500. Not tried: addresses > 255, other unit ids |
| 7 | DVT protocol documentation | **Recovered** from the FrameWork 2.8.0 installer (section 7.4): VDX telegram format, terminal reply codes and image commands are documented, and image retrieval is confirmed end to end. Still missing: the full `#`/`*` system-terminal command list (not in the carved help files) |
| 8 | DVT camera has no lens | Root cause of the blank image (section 7.8). Needs a C/CS-mount lens before any inspection is meaningful |

### 8.1 Recovering the I/O map

Two routes, do both and make them agree.

**Route A, the binder.** Festo ships documentation in four parts: A system
manual, B exercises, **C circuit diagram**, D data sheets. **Part C lists every
sensor and valve against its terminal and AS-i slave.** The binder exists,
since one page of it was photographed. Finding Part C could save a week.

**Route B, bus scan plus tracing.** Mount the CM 1243-2, go online, run
**ACTUAL to PLANNED**. Every slave that answers appears with its address.
Known so far: addresses 6 and 7 at Station 2, 10 at Station 3, 14, 15, 18 and
19 seen on labels.

Then identify inputs by walking the line and triggering things by hand while
watching a watch table, which is safe because it is read-only. Identify outputs
**with the air off first**, forcing one bit at a time and watching the slave
and valve LEDs. Never force more than one output at a time and never leave a
forced output set.

---

## 9. Lessons learned

**Diagnose before you build.** Significant time was lost on the E.IPC pursuing
recovery paths before establishing that the controller was unrecoverable. The
same pattern nearly repeated with the DVT camera until the order was reversed:
read the documentation, then scan.

**Confirm the manual's claim, not the folklore.** Re-entering origin DATA codes
after a battery loss does not work, despite being widely suggested. The ABS
method is what the manufacturer documents.

**A negative result is a result.** Two exhaustive passes through all 16 output
states proved the robot cannot control the conveyor. That closed a line of
enquiry permanently and is worth more than an untested assumption.

**Never guess robot motion syntax.** On a PLC a wrong guess is a compile error.
On a 5-axis arm it is a collision.

**Boot capture finds undocumented devices.** `tcpdump` plus a power cycle
revealed a camera that no ping sweep could find, because it was on a subnet
nobody knew about.

**A reply is not an acknowledgement.** The camera "answered" `#YI`, which looked
like a working trigger until the reply-code table was read and `% 1` turned out
to mean *general error*. Read the status code, do not infer success from the
fact that the device replied at all.

**Unpack, do not run, an untrusted installer.** The FrameWork protocol was
recovered by carving the Wise overlay's raw-deflate streams out of the installer
offline; nothing from the vendor binary was executed. The documents needed were
inside it.

**A documented channel beats a guessed register map.** Mapping Modbus 0–255
read-only found no Pass/Fail bit; the result actually lives in the documented
VDX stream on port 7500. Time spent diffing registers would have been wasted.

**Confirm the optical path before chasing a software bug.** Days of "no picture"
were, in the end, a missing lens. The transfer, decode and exposure control were
all working; the sensor was simply bare. Check the physics first.

**Stay read-only on someone else's device.** Every probe of the camera was
read-only except one operator-authorised, RAM-only exposure change that a power
cycle reverts. Nothing was written to flash, and no control bit was asserted.

**The two terminals.** `1M` on the input side and `3L+`/`3M` on the output
side account for most first-day failures on an S7-1200. Both produce symptoms
that look like software faults.

**A cylinder is not an actuator you wire.** It has air ports. The valve is the
electrical component. This confusion cost an afternoon.

**Check coil voltage before connecting.** A 12 V coil on a 24 V output
dissipates four times its rated power and fails within minutes. Measure the
resistance: roughly 30 ohm means 12 V, roughly 120 ohm or more means 24 V.

---

## Repository layout

```
.
├── README.md                  this file
├── plc/
│   ├── SensorRead.scl         FB + instance DB + OB1, importable into TIA
│   ├── FB_SafetyMonitor.scl   safety chain monitoring (status only)
│   ├── FB_Station.scl         generic station state machine
│   ├── FB_RobotStation.scl    Station 4 robot handshake
│   ├── FB_Conveyor.scl        MICROMASTER 420 terminal control
│   ├── DB_Line.scl            line-wide state
│   ├── InstanceDBs.scl        instance data blocks
│   └── OB_Main.scl            six-station orchestration
├── tags/
│   ├── PLC_Tags.csv           tag table, placeholders marked
│   └── PLC_Tags.xlsx          TIA-importable version
└── docs/
    ├── ROBOT_CONTROL.md       full treatment of RV-2AJ control methods
    └── IO_SURVEY.md           procedure for recovering the real I/O map
```

**Import order for the SCL files:** function blocks first, then `DB_Line.scl`,
then `InstanceDBs.scl`, then `OB_Main.scl`. TIA Portal cannot generate an
instance DB for a function block it has not seen yet.

---

## Safety

The **Pilz PNOZ X9** safety chain is hardwired and was not modified. The PLC
program **monitors** it and does not implement it. `FB_SafetyMonitor` is
deliberately written so that after the chain opens, `SafeToRun` stays FALSE
until the chain closes **and** an operator presses Reset. There is no
auto-restart.

Standing rules observed throughout:

- Air off for every first electrical test
- Air on last, at low pressure, hands clear
- Mains unplugged before touching any 230 V terminal
- One output forced at a time, never left forced

---

## References

### Mitsubishi

- [Mitsubishi CR1 Controller Instruction Manual](https://www.roboex.com/Mitsubishi%20Movemaster%20Robot%20Manual/RV-1A/CR1%20Controller%20Instruction%20Manual.pdf)
- [MELFA-BASIC IV command list, RV-3SJ Series](https://www.manualslib.com/manual/1949566/Mitsubishi-Rv-3sj-Series.html?page=98)

### Festo

- [Compact cylinders ADVU / AEVU, type code and function](https://ftp.festo.com/public/pneumatic/SOFTWARE_SERVICE/Documentation/2010/EN/ADVU_EN.PDF)
- [Proximity sensors SMT / SME-8, pin allocation and electrical data](https://ftp.festo.com/Public/PNEUMATIC/SOFTWARE_SERVICE/Documentation/2012/US/SMX8_ENUS.PDF)

### Siemens and power

- [Delta DVPPS01 power supply instruction sheet](https://www.famcocorp.com/uploads/product/Automation/Power%20Supply/Delta/Catalog/Delta-Power-Supply-DVP-PS01-M-Catalogue.pdf)

### DVT / Cognex

- [Cognex DVT Intellect manual](https://www.hvssystem.com/documentations/Cognex/DVT/intellectManualEN.pdf)
- [DVT SmartImage Sensor Installation and User Guide](https://gab.wallawalla.edu/~ralph.stirling/classes/engr480/docs/Vision/dvt_framework.pdf)
- [IAI integration note, source of the `#YI` trigger command](https://www.intelligentactuator.com/pdf/DVT_integ.pdf)
- [Cognex DVT Intellect 1.5.1 download](https://support.cognex.com/en/downloads/detail/dvt/689/1033)

### Linux tooling

- [python-snap7, TIA Portal configuration requirements](https://python-snap7.readthedocs.io/en/latest/tia-portal-config.html)
- [pnio-dcp, PROFINET discovery and IP assignment](https://pypi.org/project/pnio-dcp/)
