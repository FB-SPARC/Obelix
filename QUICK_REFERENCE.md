# Quick Reference Guide

## State Transition Diagram

```
                     ┌─────────────┐
                     │    IDLE     │ ← Touchpad (panic)
                     │(emergency)  │
                     └─────────────┘
                           ↑
              ┌────────────┴────────────┐
              │                         │
         R1 Release              L1 Release
              │                         │
              ↓                         ↓
        ┌──────────────────────────────────────┐
        │         ACTIVE (Default)             │
        │  ✓ Rack deployed                     │
        │  ✓ Can pass trench                   │
        │  ✓ Ready for intake or shoot         │
        └──────────────────────────────────────┘
              ↑                    ↑
         R1 Press              L1 Press
              │                    │
              ↓                    ↓
        ┌────────────┐      ┌──────────────┐
        │ INTAKING   │      │   SHOOTING   │
        │ • 8V intake│      │ • Auto-aim   │
        │ • Fast rack│      │ • Latch feed │
        └────────────┘      └──────────────┘

    [Options] ──→ INTAKE_CLOSED (rare edge case)
```

---

## Button Reference

### Teleop (PS5 Controller)

| **Button** | **Action** | **From** | **To** | **Behavior** |
|:---:|:---:|:---:|:---:|:---|
| **R1** | Press | any | INTAKING | Full-power intake, fast rack deploy |
| **R1** | Release | INTAKING | ACTIVE | Return to ready state |
| **L1** | Press | any | SHOOTING | Spin shooter, enable drive auto-aim |
| **L1** | Release | SHOOTING | ACTIVE | Stop shooting, keep rack out |
| **L1 + Left Stick** | Hold + move | SHOOTING | (stays) | Drive + rotate to target |
| **Right Stick** | Any input | SHOOTING | ACTIVE | Cancel shot (quick abort) |
| **Touchpad** | Press | any | IDLE | **EMERGENCY STOP** |
| **Options** | Press | any | INTAKE_CLOSED | Manual rack retract (rare) |
| **Triangle** | Press | any | (no change) | Reset rack encoder |

---

## State Behaviors at a Glance

### ACTIVE (Default)
```
Rack:        Deployed (280mm) at normal speed
Intake:      Hold voltage (4V) — maintains game pieces
Bed/Feeder:  Stopped
Hood:        Angle 0°
Shooter:     Stopped
```
**Transitions from**: Teleop start, R1 release, L1 release, right-stick input
**Can transition to**: INTAKING (R1), SHOOTING (L1), INTAKE_CLOSED (Options), IDLE (Touchpad)

### INTAKING
```
Rack:        Deployed FAST (40× normal) — minimize stall time
Intake:      Full power (8V)
Bed/Feeder:  Stopped
Hood:        Angle 0°
Shooter:     Stopped
```
**Transitions from**: R1 press
**Auto-exits to**: ACTIVE when R1 released (if still in INTAKING)

### SHOOTING
```
Rack:        Deployed (maintains position)
Intake:      Hold voltage (4V)
Bed/Feeder:  Gated by shooter-ready latch
  ├─ While shooter spinning up: OFF
  └─ Once shooter at setpoint: ON (2000 rpm bed, 2500 rpm feeder)
Hood:        Angle from shot-solution (polynomial fit)
Shooter:     RPM from shot-solution (polynomial fit)
Drive:       Auto-aim to target via PID rotation
```
**Transitions from**: L1 press
**Auto-exits to**: ACTIVE when L1 released (if still in SHOOTING)
**Special**: Right-stick input → ACTIVE (abort shot)

### IDLE
```
All motors:  STOPPED
```
**When to use**: EMERGENCY STOP ONLY
**Transition**: Touchpad press (always reachable from any state)

---

## Shooting (L1) Breakdown

### Step 1: Enter Shooting (L1 Press)
```
Superstructure → SHOOTING state
  • Hood spins to match target distance
  • Shooter spins to match target distance
  • Shooter-ready latch resets to FALSE

Drive → Auto-aim (PID rotation to target)
  • If left stick released: stationary aim (X-lock wheels)
  • If left stick input: moving aim (drive + rotate)
```

### Step 2: Spin-up (0~1.5 seconds, typically)
```
Hood:        Ramping to angle
Shooter:     Ramping to RPM
Bed/Feeder:  STOPPED (latch is FALSE)
Rack:        Deployed, maintaining
Intake:      Hold (4V)
```

### Step 3: Ready to Fire (Shooter at Setpoint)
```
Shooter-ready latch → TRUE
Bed:  2000 rpm (spin game piece)
Feeder:  2500 rpm (push into shooter)
  • Stays ON until we leave SHOOTING
  • Immune to RPM flicker (that's the point of the latch!)
```

### Step 4: Exit Shooting (L1 Release)
```
Superstructure → ACTIVE
  • Bed/Feeder stop
  • Rack stays deployed
  • Hood relaxes to 0°
  • Shooter spins down (not actively stopped)

Drive → Normal joystick control resumes
```

**Quick abort**: Right stick input while shooting → immediately go ACTIVE

---

## Autonomous Commands

### Named Commands (for PathPlanner)

```
"Intake Mode"
  1. Set INTAKING
  2. Wait for rack at MAX_POSITION
  3. Set ACTIVE
  Result: Always completes, no hanging

"Shoot"
  1. Set SHOOTING
  2. Aim at target (2 second timeout)
  3. Set ACTIVE
  Result: Always completes within timeout, no hanging
```

### Example Auto Flow
```
[Intake Mode]  ← Picks up game piece
    ↓
[Drive path]   ← Navigate to shooting location
    ↓
[Shoot]        ← Spin up, aim, feed, return to ACTIVE
    ↓
[Optional] repeat for multi-game-piece cycles
```

---

## Useful Troubleshooting

### Feeder Jams During Shot
- **Problem**: Bed/feeder stutter → jam
- **Root cause**: Shooter RPM flickers, gates feeder on/off repeatedly
- **Fix**: Shooter-ready latch (already implemented)
  - If still jamming: check shooter motor direction or PID tuning
  - If intermittent: increase SHOOT_TIMEOUT_SECONDS in AutoCommands

### Rack Won't Deploy in INTAKING
- **Problem**: Rack moves slowly or not at all
- **Root cause**: Likely tuning or mechanical issue
- **Debug**:
  - Check rack encoder value (Triangle to reset)
  - Verify motor direction (should push rack out)
  - Check NetworkTables for rack position feedback

### Robot Can't Pass Trench
- **Old problem**: IDLE retracted rack
- **Solution**: Always use ACTIVE (not IDLE) as return state
  - ✓ R1 release → ACTIVE (not IDLE)
  - ✓ L1 release → ACTIVE (not IDLE)
  - ✓ Teleop start → ACTIVE (not INTAKING)
  - ✓ Only Touchpad → IDLE (emergency stop)

### Auto Hangs (Doesn't Complete)
- **Old problem**: AutoAim never ended
- **Solution**: All auto commands now have timeouts or explicit end conditions
  - "Shoot" has 2-second timeout
  - "Intake Mode" waits for rack setpoint (will end)
  - No infinite loops

---

## State Machine Guarantees

✅ **No hanging in auto** — all commands have timeouts or explicit end conditions
✅ **No state races in teleop** — clean trigger logic, single authorities
✅ **Robust feeding** — shooter-ready latch prevents jamming
✅ **Always recoverable** — Touchpad → IDLE stops everything immediately
✅ **Rack always deployed by default** — ACTIVE keeps intake ready

---

## Key Insights

1. **ACTIVE is the new default** (not IDLE)
   - Solves "can't pass trench" problem
   - Makes state transitions clean

2. **Single SHOOTING state** (not STATIONARY/MOVING)
   - Drive command handles both via joystick
   - Eliminates state toggle races

3. **Shooter-ready latch** (not direct gate)
   - Prevents feeder stutter on RPM flicker
   - Reset each shot for fresh sequence

4. **Auto commands are atomic** (not hanging)
   - Timeouts on aim
   - Explicit end conditions on intake
   - No state-dependency bugs
