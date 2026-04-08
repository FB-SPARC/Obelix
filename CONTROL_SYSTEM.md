# Robot Control System Documentation

## Overview

This document describes the **Superstructure** state machine and control flow for the Obeliks FRC robot. The system coordinates all non-drive subsystems (bed, feeder, hood, intake, rack, shooter) through a centralized state machine with clean trigger-based button bindings.

## State Machine

The superstructure operates as a finite state machine with 6 discrete states. All state transitions are handled through `Superstructure.setState(State)`.

### States

#### **IDLE** — Emergency Stop
- **Purpose**: Panic stop / emergency shutdown
- **Behavior**: All motors stop, no motion
- **When to use**: Touchpad (operator panic button)
- **Transitions**: Only from user input

#### **ACTIVE** — Default Ready State ⭐
- **Purpose**: Robot is ready to drive, pass the trench, and respond to intake/shoot inputs
- **Behavior**:
  - Rack deployed at MAX_POSITION (280 mm)
  - Intake at 4V hold voltage (maintains game pieces without aggressive spin)
  - All other subsystems idle
- **When entered**: Teleop start, button releases (R1 release, L1 release, right-stick cancel)
- **Key insight**: This is the new default state. The robot **can always pass the trench** because the rack is deployed. This solves the original problem where IDLE would retract the rack, blocking trench passage.

#### **INTAKING** — Aggressive Intake
- **Purpose**: Rapidly pull game pieces from the trench
- **Behavior**:
  - Intake at 8V (full power)
  - Rack deploys 40× faster than normal (aggressive motion)
  - All other subsystems idle
- **When entered**: R1 press
- **When exited**: R1 release → returns to ACTIVE

#### **SHOOTING** — Shooting Mode
- **Purpose**: Spin up and fire the game piece
- **Behavior**:
  - Hood angle & shooter RPM controlled by polynomial shot-solution (distance-based)
  - Rack stays deployed
  - Intake holds at 4V
  - **Bed/feeder are gated by the shooter-ready latch** (see below)
- **When entered**: L1 press
- **When exited**: L1 release → returns to ACTIVE
- **Drive override**: Drive command switches to auto-aim (PID rotation to target)

#### **INTAKE_CLOSED** — Manual Rack Retraction
- **Purpose**: Edge case; retract intake mechanism (rarely used in normal flow)
- **Behavior**: Rack retracts to MIN_POSITION, all else idle
- **When entered**: Options button press
- **Note**: Not used in typical teleop flow; exists for manual override if needed

#### **COAST** — Soft Motors (Manual Positioning)
- **Purpose**: Position mechanisms by hand (tuning/diagnostics)
- **Behavior**: All motors switch to coast mode, brake mode disengaged
- **Note**: Not user-accessible; used only for tuning

---

## Shooter-Ready Latch

The **shooter-ready latch** is a key innovation that prevents the bed/feeder from jamming during shooting.

### The Problem
Shooter RPM can flicker momentarily during spin-up (vibration, traction loss, etc.). If bed/feeder are directly gated on `shooter.isAtSetpoint()`, they stutter: feed → stop → feed → stop, causing jams.

### The Solution
```java
private boolean shooterWasReady = false;

// In SHOOTING state:
if (shooter.isAtSetpoint()) {
  shooterWasReady = true;  // Latch goes HIGH
}
if (shooterWasReady) {
  bed.setBedRPM(2000);     // Stay ON even if RPM dips
  feeder.setFeederRPM(2500);
} else {
  bed.stop();              // Don't feed until ready
  feeder.stop();
}
```

**Key behavior**:
- Once the shooter reaches setpoint, the latch locks TRUE
- Feed rate stays on continuously, even if RPM flickers
- Latch resets to FALSE each time we enter SHOOTING (fresh shot sequence)

This is robust and prevents the race condition that plagued the old system.

---

## Button Bindings (Teleop Control)

All button bindings are in `RobotContainer.configureButtonBindings()`. The system is designed to **eliminate state races** by using clean trigger logic.

### Controller Layout (PS5)

| Button | Action | Effect |
|--------|--------|--------|
| **R1** | Press | → INTAKING (full intake, fast rack deploy) |
| **R1** | Release | → ACTIVE (if was intaking) |
| **L1** | Press | → SHOOTING (spins up shooter, enables drive auto-aim) |
| **L1** | Release | → ACTIVE (if was shooting) |
| **L1** (held) | **+ Left Stick** | Drive auto-aim: translation + PID rotation to target |
| **Right Stick** | Any input (while shooting) | → ACTIVE (cancel shot) |
| **Touchpad** | Press | → IDLE (emergency stop) |
| **Options** | Press | → INTAKE_CLOSED (manual rack retract) |
| **Triangle** | Press | Reset rack encoder (calibration) |

### State Race Prevention

**Old system (broken)**:
- SHOOTING_STATIONARY vs. SHOOTING_MOVING toggle via left-stick triggers
- L1.onFalse and left-stick.onFalse could both fire, fighting over state
- AutoAim never ended (would hang auto)

**New system (fixed)**:
- Single SHOOTING state; drive command handles aim-while-moving/stationary
- L1 is the only authority for shooting state (no left-stick toggle)
- Drive command (`joystickDriveAimAtPoint`) handles both cases:
  - Stick released → stationary shot (X-lock wheels)
  - Stick input → moving shot (PID rotation + translational velocity)
- No races because triggers are clean and don't fight

---

## Drive Control During Shooting

When L1 is held (SHOOTING state), the drive command **automatically switches to auto-aim mode**:

```
L1.whileTrue(joystickDriveAimAtPoint(...))
```

This command:
1. **Reads left-stick input** to determine translational velocity
2. **PID rotates chassis** to face the target (via `angleController`)
3. **X-locks wheels** when on-target and stick is released

**Two shot modes** (handled automatically by stick input):
- **Stationary shot**: Release left stick → wheels X-lock, pure rotation to target
- **Moving shot**: Provide left-stick input → drives forward/back/sideways while rotating

No separate state needed; the stick controls the behavior.

---

## Autonomous Commands

All auto commands are in `AutoCommands.java` and registered as PathPlanner named commands.

### `intakeMode(superstructure, rack)`
Intake sequence that always completes:
1. Set INTAKING
2. Wait for rack to reach MAX_POSITION
3. Return to ACTIVE

Used in autos as:
```
NamedCommands.registerCommand("Intake Mode", intakeMode(...))
```

### `shootSequence(superstructure, drive, target, timeout)`
Complete shooting sequence with guaranteed termination:
1. Set SHOOTING
2. Aim at target (with timeout) — **no hanging**
3. Return to ACTIVE

The timeout ensures the command **always finishes** in auto (unlike the old AutoAim which ran forever).

Used in autos as:
```
NamedCommands.registerCommand("Shoot", shootSequence(...))
```

### `aimAtPoint(drive, target)`
Pure drive command for PID aiming. No superstructure dependency — can be used standalone or in sequences.

---

## Auto File Updates

All 6 PathPlanner auto files have been updated:
- Removed the broken `parallel(wait(4.0), "Shooting Mode", "AutoAim")` groups
- Replaced with single `"Shoot"` named command

Old pattern (broken):
```json
{
  "type": "parallel",
  "commands": [
    { "type": "wait", "waitTime": 4.0 },
    { "type": "named", "name": "Shooting Mode" },
    { "type": "named", "name": "AutoAim" }  // Never ends!
  ]
}
```

New pattern (fixed):
```json
{ "type": "named", "name": "Shoot" }  // Self-contained, always terminates
```

---

## Key Improvements

### 1. **No More State Races**
- Single SHOOTING state (no STATIONARY/MOVING toggle)
- L1 is the sole authority
- Drive command handles all aim modes
- Cleanest trigger logic possible

### 2. **Rack Always Deployed by Default**
- ACTIVE state keeps rack out
- Robot can always pass the trench
- No more "can't reach trench" problem

### 3. **Auto Commands Always Terminate**
- `shootSequence` has timeout
- No hanging in autonomous
- `intakeMode` waits for a real signal (rack position)
- All commands use `finallyDo()` for safe cleanup

### 4. **Shooter Stutter Fixed**
- Latch prevents bed/feeder from flickering
- Once ready, feeding stays on
- Solves the jam-on-RPM-flicker problem

### 5. **Clean Teleop Flow**
- Button releases return to ACTIVE (not IDLE)
- IDLE is panic-only (touchpad)
- One button per function; no complex triggers

---

## Tuning & Debugging

### Useful NetworkTables Entries
- `Superstructure/State` — current state (IDLE, ACTIVE, etc.)
- `ShotControl/DistanceToTarget` — distance to hub in meters
- `ShotControl/HoodAngleGoal` — desired hood angle (degrees)
- `ShotControl/ShooterRPMGoal` — desired shooter RPM
- `Auto Align/Pos error` — angular position error (radians)
- `Auto Align/Vel error` — angular velocity error

### Rack Calibration
- Press Triangle to reset rack encoder to 0
- Manually deploy/retract to find min/max positions
- Update `RackConstants.MIN_POSITION_METERS` and `MAX_POSITION_METERS`

### Shooter-Ready Timeout
If feeder still jams despite the latch, consider:
1. Increasing `SHOOT_TIMEOUT_SECONDS` in AutoCommands (gives more spin-up time)
2. Tuning shooter PID (faster spin-up)
3. Checking feeder motor direction

---

## Summary

This control system is designed for:
- ✅ **No state races** (clean trigger logic, single authorities)
- ✅ **Robustness** (shooter-ready latch, timeouts, finallyDo cleanup)
- ✅ **Simplicity** (no complex multi-state toggles)
- ✅ **Flexibility** (ACTIVE as default allows easy transitions)

All code is well-commented; see `Superstructure.java`, `RobotContainer.java`, and `AutoCommands.java` for inline documentation.
