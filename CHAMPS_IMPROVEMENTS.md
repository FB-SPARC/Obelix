# Champs Improvement Plan

Goals derived from gap analysis against Team 6328's 2026 codebase.
Excludes: Northstar custom vision pipeline, Choreo build-time trajectories, ML object detection.

---

## 1. LoggedTunableNumber

Replace hardcoded PID/feed-forward constants with runtime-tunable values via NetworkTables.
Only exposed when `Constants.tuningMode = true`.

**Files to change:**
- Add `util/LoggedTunableNumber.java`
- Add `Constants.tuningMode` boolean (default `false`)
- Replace gains in `ShooterIOTalonFX`, `Hood`, `Drive`, `Superstructure`

**Done when:** All PID gains can be changed from the dashboard without a redeploy.

---

## 2. LoggedTracer (Cycle Time Profiling)

Instrument every major periodic section so loop overruns can be attributed to a specific subsystem.

**Files to change:**
- Add `util/LoggedTracer.java`
- Add `LoggedTracer.record("subsystemName")` calls in `Robot.java` after each subsystem's `periodic()` invocation

**Done when:** AKit logs show per-section cycle times visible in AdvantageScope.

---

## 3. Command Lifecycle Logging

Log every command initialize/finish/interrupt event so auto and teleop sequences can be debugged in replay.

**Files to change:**
- `Robot.java` — register `CommandScheduler` callbacks in `robotInit()` using `Logger.recordOutput()`

**Done when:** AdvantageScope replay shows which commands were active at any point in the match.

---

## 4. Subsystem Simulation

Replace no-op sim IO implementations with real physics models so autonomous can be developed without the robot.

**Subsystems to simulate:**
- `ShooterIOSim` — `FlywheelSim` for flywheel velocity
- `HoodIOSim` — `SingleJointedArmSim` for hood angle
- `IntakeIOSim` — `DCMotorSim`
- `RackIOSim` — `DCMotorSim`
- `FeederIOSim` — `DCMotorSim`

**Files to change:**
- New `*IOSim.java` for each subsystem above
- `RobotContainer.java` — instantiate sim implementations when `Constants.currentMode == Mode.SIM`

**Done when:** Full superstructure cycle (intake → feed → shoot) works in simulation.

---

## 5. Mechanism2d Visualization

Add a logged 2D mechanism visualization for the shooter/hood so mechanism state is visible in AdvantageScope without the robot.

**Files to change:**
- Add `util/ShooterVisualizer.java` using WPILib `Mechanism2d`
- Publish via `Logger.recordOutput("Mechanism2d", mechanism)` in `Superstructure.periodic()`

**Done when:** Hood angle and shooter state appear in AdvantageScope's mechanism view during sim and replay.

---

## 6. IO Output Struct Pattern

Refactor IO interfaces to log outputs (not just inputs) so replay is fully deterministic.

**Pattern change:**
- Each IO interface gets an `@AutoLog`-annotated inner `XxxIOOutputs` class
- Replace direct output methods (`setVoltage()`, etc.) with `applyOutputs(XxxIOOutputs outputs)`
- Subsystem `periodic()` populates an outputs object and calls `io.applyOutputs(outputs)`

**Subsystems to refactor:** Shooter, Hood, Feeder, Intake, Rack, Bed

**Done when:** Log replay shows commanded outputs alongside measured inputs for all subsystems.

---

## 7. Parameterized AutoSelector

Replace the flat `buildAutoChooser()` with a two-question selector (start position + strategy) so autos are composable without duplicating code.

**Files to change:**
- Add `util/AutoSelector.java` (question/response system)
- Refactor `AutoCommands.java` to build the auto command from selector answers
- Update `RobotContainer.java` to use `AutoSelector` instead of `LoggedDashboardChooser`

**Questions to implement:**
- Start Position: `LEFT`, `CENTER`, `RIGHT`
- Strategy: `AGGRESSIVE`, `SAFE`

**Done when:** Driver can independently pick start position and strategy; auto command is built from the combination.

---

## 8. CheckDeploy Build Safety

Prevent accidentally deploying a sim-configured binary to the real robot.

**Files to change:**
- Add `Constants.RobotType` enum (`OBELIX`, `SIMBOT`)
- Add `Constants.checkDeploy()` main method — throws if `ROBOT == SIMBOT`
- `build.gradle` — add `checkConstantsDeploy` task, run before `deploy`

**Done when:** Running `./gradlew deploy` with `SIMBOT` selected fails with a clear error message.

---

## 9. Watchdog Loop Overrun Suppression

Use reflection to access WPILib's private `m_watchdog` on `IterativeRobotBase` and set a longer timeout than the actual loop period. This suppresses spurious Driver Station loop overrun warnings on heavy initialization frames while still catching genuinely broken loops.

**How it works:** Loop runs at 20ms (`loopPeriodSecs`), but the watchdog only fires at 200ms (`loopPeriodWatchdogSecs`). The `CommandScheduler` period is also set to the watchdog value.

**Files to change:**
- Add `Constants.loopPeriodWatchdogSecs = 0.2` alongside existing `loopPeriodSecs = 0.02`
- `Robot.java` constructor — add the reflection block after `Logger.start()`:
  ```java
  try {
    Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
    watchdogField.setAccessible(true);
    Watchdog watchdog = (Watchdog) watchdogField.get(this);
    watchdog.setTimeout(Constants.loopPeriodWatchdogSecs);
  } catch (Exception e) {
    DriverStation.reportWarning("Failed to set watchdog timeout.", false);
  }
  CommandScheduler.getInstance().setPeriod(Constants.loopPeriodWatchdogSecs);
  ```

**Done when:** Driver Station no longer shows loop overrun warnings on normal frames; warnings still appear for loops exceeding 200ms.

---

## 10. VirtualSubsystem Pattern

Give Vision (and any future non-command subsystem) an explicit lifecycle that opts out of the CommandScheduler while still being called deterministically.

**Files to change:**
- Add `util/VirtualSubsystem.java`
- Convert `Vision` subsystem to extend `VirtualSubsystem`
- Call `VirtualSubsystem.runAllPeriodic()` in `Robot.java`

**Done when:** Vision periodic runs on a deterministic schedule independent of the CommandScheduler.

---

## 11. LimelightThread (Vision Loop Overrun Fix)

Move all blocking Limelight NT operations off the main robot loop onto a dedicated background thread, matching the pattern of the existing `PhoenixOdometryThread`.

**Root cause:** `NetworkTableInstance.getDefault().flush()` in `VisionIOLimelight.updateInputs()` forces an immediate network send on the main thread, which blocks and causes loop overruns. 6328 avoids this because their Northstar IO only drains pre-buffered NT queues — no flush needed.

**How it works:**
- `LimelightThread` runs at ~100Hz using a `Notifier`
- On the background thread: publish robot orientation, set IMU mode, call `NT.flush()`
- Also on the background thread: drain `readQueue()` from megatag1/megatag2 subscribers, store observations in a `Lock`-protected list
- `VisionIOLimelight.updateInputs()` only acquires the lock, drains the buffered observations, and copies them into inputs — no NT operations at all on the main thread

**Files to change:**
- Add `subsystems/vision/LimelightThread.java`
  - Constructor takes camera NT table name and a `Supplier<Rotation2d>` for orientation
  - Exposes `List<TimestampedObservation> getObservationsAndClear()` (lock-protected drain)
  - Uses `edu.wpi.first.wpilibj.Notifier` for the background loop
- Refactor `VisionIOLimelight.java`
  - Remove `orientationPublisher`, `imuModePublisher`, `latencySubscriber`, `megatag1Subscriber`, `megatag2Subscriber` fields
  - Construct a `LimelightThread` in the constructor
  - `updateInputs()` calls `thread.getObservationsAndClear()` only — no NT operations

**Done when:** Limelight vision no longer causes loop overruns; orientation is still published to the camera at full rate; pose observations are still timestamped correctly.

---

## 12. Energy & Current Logging (BatteryLogger)

Track per-subsystem current draw, power, and cumulative energy consumption every loop cycle. Helps diagnose brownouts, identify power hogs, and understand battery state during replay.

**How it works:**
- `BatteryLogger` is a singleton on `Robot` (`public static final BatteryLogger batteryLogger`)
- Each subsystem calls `Robot.batteryLogger.reportCurrentUsage(key, isDrive, amps...)` in its `periodic()`, passing stator current readings directly from its `inputs` struct
- `BatteryLogger` accumulates totals per key and per hierarchical prefix (e.g. `"Drive/Module0-Drive"` rolls up into `"Drive"`)
- `periodicAfterScheduler()` logs everything to AKit under `EnergyLogger/Current/`, `EnergyLogger/Power/`, `EnergyLogger/Energy/` and resets cycle totals
- Battery voltage and RIO current are sampled in `Robot.robotPeriodic()` via `RobotController` and set on the logger each cycle
- Fixed overhead entries (radio, CANivore, Pigeon, CANcoders) are included as constants inside `periodicAfterScheduler()`

**Files to change:**
- Add `energy/BatteryLogger.java`
- Add `energy/CurrentLimits.java` (consolidates all motor current limits in one place — replaces scattered constants)
- `Robot.java` — add `public static final BatteryLogger batteryLogger`, `BatteryIOInputs` inner class, sample voltage/current each cycle, call `batteryLogger.periodicAfterScheduler()` after the scheduler
- Each subsystem `periodic()` — add one `Robot.batteryLogger.reportCurrentUsage(...)` call using motor stator currents already present in inputs:
  - `Drive/Module0-Drive`, `Drive/Module0-Turn`, … (per module, `isDrive=true` for drive motors)
  - `Shooter/Flywheel`
  - `Shooter/Hood`
  - `Intake`
  - `Rack`
  - `Feeder`
  - `Bed`

**Done when:** AdvantageScope shows live and replayed current/power/energy per subsystem and in total; `CurrentLimits.java` is the single source of truth for all motor current limits.

---

## 13. Interpolation Maps for Shot Control

Replace the 4th-degree polynomial in `ShotControl.java` with `InterpolatingDoubleTreeMap` data tables keyed by distance.

**Why:** When a shot at a specific distance is off at competition, you change one number in the table. With a polynomial you have to re-fit the entire curve. Interpolation maps also don't blow up at range edges (Runge's phenomenon) and trivially handle non-monotonic relationships. This is the single highest-impact change for shot tuning speed at an event.

**Files to change:**
- Rewrite `util/ShotControl.java` — replace polynomial coefficients with `InterpolatingDoubleTreeMap` for hood angle (degrees) and `InterpolatingDoubleTreeMap` for flywheel RPM, populated with your measured calibration points
- Add time-of-flight map (`InterpolatingDoubleTreeMap`) keyed by distance → seconds in flight, used for moving-target lead calculation when that is implemented

**Done when:** Shot parameters are data-point tables; adding/changing one distance point takes one line and does not affect neighboring distances.

---

## 14. RobotState Singleton (Centralized Pose + Velocity)

Extract pose estimation out of `Drive` into a dedicated `RobotState` singleton. Add field-relative velocity tracking.

**Why:** Currently any code needing the robot pose must hold a reference to `Drive`. Shot-while-moving lead compensation requires field-relative velocity, which `SwerveDrivePoseEstimator` does not expose. A `TimeInterpolatableBuffer` inside `RobotState` enables proper vision latency compensation (look up pose at camera timestamp rather than current time).

**Files to change:**
- Add `RobotState.java` singleton with:
  - `addOdometryObservation(timestamp, gyroAngle, modulePositions[])` called from `Drive.periodic()`
  - `addVisionMeasurement(pose, timestamp, stdDevs)` replacing the `VisionConsumer` in `Drive`
  - `getEstimatedPose()`, `getRotation()`, `getRobotVelocity()` (field-relative `ChassisSpeeds`)
  - `getPoseAtTimestamp(double t)` via `TimeInterpolatableBuffer`
- Remove `SwerveDrivePoseEstimator` from `Drive.java`, replace with calls to `RobotState`
- Update `Vision.java` consumer to call `RobotState.getInstance().addVisionMeasurement()`
- Update all callers of `drive.getPose()` to `RobotState.getInstance().getEstimatedPose()`

**Done when:** Pose is accessible without a `Drive` reference; velocity is logged and available for future lead calculation.

---

## 15. FullSubsystem — Deferred Output Application for Drive

Add a `FullSubsystem` base class that provides a `periodicAfterScheduler()` hook. Apply drive module setpoints there instead of during command execution.

**Why:** Currently `runVelocity()` writes module setpoints mid-scheduler-run, before all subsystems have updated. The clean pattern is: read inputs in `periodic()` → commands run → apply outputs in `periodicAfterScheduler()`. This is the same pattern `PhoenixOdometryThread` solves for odometry reads, applied to the output side.

**Files to change:**
- Add `util/FullSubsystem.java` — extends `SubsystemBase`, static `runAllPeriodicAfterScheduler()`, abstract `periodicAfterScheduler()`
- Refactor `Drive.java` to extend `FullSubsystem`, move `io[i].setDesiredState(...)` calls from `runVelocity()` into `periodicAfterScheduler()`
- `Robot.java` — call `FullSubsystem.runAllPeriodicAfterScheduler()` after `CommandScheduler.getInstance().run()`

**Done when:** Drive module setpoints are always applied after the full command scheduler cycle completes.

---

## 16. showHardwareAlerts() + Auto Duration Logging

Two small additions to `Robot.java` that have outsized competition value.

**showHardwareAlerts():**
```java
public static boolean showHardwareAlerts() {
  return Constants.currentMode != Mode.SIM && Timer.getTimestamp() > 30.0;
}
```
Update all `.set(!inputs.connected)` alert calls in `Drive` and `Module` to gate on `Robot.showHardwareAlerts()`. Prevents false-positive disconnect alerts during the CAN bus initialization window on every power cycle.

**Auto duration logging:**
- Add `autoStart` (double) and `autoMessagePrinted` (boolean) fields to `Robot`
- Set `autoStart = Timer.getTimestamp()` in `autonomousInit()`
- In `robotPeriodic()`, once the autonomous command ends, print `"*** Auto finished in X.XX secs ***"` (or "cancelled") exactly once

**Done when:** No false disconnect alerts on boot; console shows auto duration after every auto run.

---

## 17. Controller Disconnection Alerts + Rumble Feedback

**Controller alerts** — add `Alert` objects for each controller port and check `DriverStation.isJoystickConnected()` each cycle. Fires a persistent dashboard warning when the driver or operator controller is unplugged.

**Rumble** — add haptic feedback for key conditions:
- Trying to shoot while robot is not in a valid state (brief double-pulse)
- Shooter not ready while trigger held (continuous low rumble)

**Files to change:**
- `RobotContainer.java` — add `Alert driverControllerDisconnected`, check in `updateDashboardOutputs()` or a periodic trigger
- Add rumble commands via `new RumbleCommand(controller, RumbleType.kBothRumble, intensity)` triggered off existing state conditions in `Superstructure`

**Done when:** Driver sees a dashboard alert when their controller is unplugged; controller vibrates when attempting an invalid action.

---

## 18. Robot.java Quality-of-Life Additions

Small one-time additions to `Robot.java` constructor, each under 5 lines:

**Silence joystick warnings:**
```java
DriverStation.silenceJoystickConnectionWarning(true);
```

**Silence Rotation2d zero-vector spam** — wrap `MathSharedStore.getMathShared()` to filter `"x and y components of Rotation2d are zero"` errors while forwarding everything else.

**Hostname + platform metadata:**
```java
Logger.recordMetadata("Hostname", InetAddress.getLocalHost().getHostName());
Logger.recordMetadata("Platform", System.getProperty("os.name") + " " + System.getProperty("os.arch"));
```

**Sim alliance station:**
```java
if (Constants.currentMode == Mode.SIM) {
  DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
  DriverStationSim.notifyNewData();
}
```

**Done when:** DS console is clean on startup; logs identify which machine/OS they came from; sim alliance flip behavior is deterministic.

---

## 19. Utility Classes: ContinuousConditionalCommand, SuppliedWaitCommand, TriggerUtil

Three small utility classes to copy from 6328 and wire in:

**ContinuousConditionalCommand** — re-evaluates its condition every cycle and live-swaps between two running commands. Unlike `ConditionalCommand` which evaluates only at init. Useful for subsystem default behaviors that should switch based on live state.

**SuppliedWaitCommand** — `WaitCommand` that takes a `DoubleSupplier`. Combined with `LoggedTunableNumber`, auto sequence timings become tunable without redeploy.

**TriggerUtil.doublePress(trigger)** — returns a `Trigger` that only activates on a double-tap within 0.4s. Use this to gate any destructive actions (e.g., manual zero routines, force-reset commands) so they cannot be triggered by a single accidental button press.

**Files to change:**
- Add `util/ContinuousConditionalCommand.java`
- Add `util/SuppliedWaitCommand.java`
- Add `util/TriggerUtil.java`
- In `RobotContainer.java`, wrap any destructive trigger bindings with `TriggerUtil.doublePress()`

**Done when:** Destructive commands require deliberate double-tap; auto wait times are tunable.

---

## 20. Build System: deleteOldFiles + CheckPullRequest + License Headers

Three one-line changes to `build.gradle` / `Constants.java`:

**`deleteOldFiles = true`** in the deploy artifact block — prevents stale trajectory/config files from accumulating on the RoboRIO across deploys.

**CheckPullRequest** — add a `Constants.CheckPullRequest` inner class with a `main()` that throws if `tuningMode == true` or robot type is not the competition default. Wire as a `checkConstantsPullRequest` Gradle task. Add to GitHub Actions CI so PRs are blocked when constants are left in a non-competition state.

**Spotless license header** — add `licenseHeader("// Copyright...")` to the existing Spotless java block. Zero ongoing maintenance; enforced automatically on every `spotlessApply`.

**Done when:** Deploy directory stays clean; PRs with debug constants are caught in CI; all files have consistent headers.

---

## 21. GeomUtil Utility Class

Add `util/geometry/GeomUtil.java` with ~20 geometry helpers that are currently scattered as inline calculations throughout the codebase.

**Methods to include:**
- `toTransform2d(Translation2d)`, `toTransform2d(double x, double y)`, `toTransform2d(Rotation2d)`, `toTransform2d(Pose2d)`, `toTransform2d(Transform3d)`
- `toPose2d(Transform2d)`, `toPose2d(Translation2d)`, `toPose2d(Rotation2d)`
- `toTransform3d(Pose3d)`, `toPose3d(Transform3d)`
- `inverse(Pose2d)`
- `multiply(Twist2d, double factor)`
- `toTwist2d(ChassisSpeeds)`
- `withTranslation(Pose2d, Translation2d)`, `withRotation(Pose2d, Rotation2d)`
- `transformVelocity(ChassisSpeeds, Translation2d transform, Rotation2d currentRotation)` — adjusts chassis speeds for a launcher/mechanism mounted off-center from the robot

**Files to change:**
- Add `util/geometry/GeomUtil.java`
- Replace any inline equivalents throughout `Drive`, `ShotControl`, `Vision`

**Done when:** All geometry conversions go through one utility class; no more inline `new Transform2d(translation, Rotation2d.kZero)` patterns.

---

## 22. EqualsUtil + Container\<T\>

Two tiny utilities used heavily throughout 6328's codebase.

**EqualsUtil** — epsilon equality for doubles and WPILib geometry types:
```java
EqualsUtil.epsilonEquals(a, b)           // default epsilon 1e-9
EqualsUtil.epsilonEquals(a, b, epsilon)
EqualsUtil.GeomExtensions.epsilonEquals(Twist2d, Twist2d)
```
Eliminates the scattered `Math.abs(a - b) < 1e-9` patterns.

**Container\<T\>** — a simple one-field mutable wrapper:
```java
public class Container<T> { public T value; }
```
Used to pass mutable values into lambdas without the `final double[]` array hack.

**Files to change:**
- Add `util/EqualsUtil.java`
- Add `util/Container.java`

**Done when:** Epsilon comparisons and lambda-mutable values use named utilities instead of ad-hoc patterns.

---

## 23. SwitchableChooser

A `LoggedNetworkInput`-backed dashboard chooser whose option list can be updated at runtime (unlike `LoggedDashboardChooser` which is fixed at init). Required by the parameterized `AutoSelector` (item 7) so that the second question's options can change based on the first answer.

**Files to change:**
- Add `util/SwitchableChooser.java`
- Wire into `AutoSelector.java` (item 7)

**Done when:** Auto selector second-question options update live when start position changes.

---

## 24. AllianceFlipUtil Upgrades

Extend the existing `AllianceFlipUtil` with missing overloads and a HAL-aware guard.

**Missing overloads to add:**
- `apply(Rotation3d)` — needed for 3D mechanism/vision pose flipping
- `apply(Translation3d)`
- `apply(Pose3d)`
- `apply(Bounds)` — flips a rectangular field region (needed once `Bounds` is added)

**Guard fix:**
```java
public static boolean shouldFlip() {
  return !Constants.disableHAL
      && DriverStation.getAlliance().isPresent()
      && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
}
```
Without `!Constants.disableHAL`, the flip logic crashes in unit tests where HAL is not initialized.

**Files to change:**
- `util/AllianceFlipUtil.java`

**Done when:** All 2D and 3D geometry types can be alliance-flipped; HAL-disabled tests don't crash.

---

## 25. Bounds Record

A simple record for axis-aligned rectangular field regions with `contains()` and `clamp()` methods. Used to define no-shoot zones (e.g. under the tower, behind the hubs) in `ShotControl`.

```java
public record Bounds(double minX, double maxX, double minY, double maxY) {
  public boolean contains(Translation2d t) { ... }
  public Translation2d clamp(Translation2d t) { ... }
  public Translation2d[] sides() { ... }  // for AScope visualization
}
```

**Files to change:**
- Add `util/geometry/Bounds.java`
- Use in `ShotControl.java` to replace any manual range checks

**Done when:** Field-region reasoning uses named `Bounds` objects instead of raw comparisons.

---

## 26. DriveToPose Command

A full closed-loop drive-to-field-pose command using a trapezoidal motion profile + separate linear and theta PID. Enables teleop auto-align (e.g. align to speaker shot position) and structured autonomous positioning.

**Features:**
- `TrapezoidProfile` for linear motion with configurable max velocity/acceleration
- `ProfiledPIDController` for theta with continuous input
- Feedforward scaling by distance-to-target (`linearFFMinRadius` / `linearFFMaxRadius`)
- All gains backed by `LoggedTunableNumber`
- Full AKit logging: setpoint pose, goal pose, distance error, theta error
- `withinTolerance(driveTol, thetaTol)` predicate for sequencing

**Files to change:**
- Add `commands/DriveToPose.java`
- Add a `driveToPose(Supplier<Pose2d>)` factory method in `DriveCommands.java`

**Done when:** Robot can autonomously drive to any field pose from teleop or auto; gains are tunable live.

---

## 27. shouldThrottle() — Disabled-Mode CPU Throttling

After 5 seconds disabled, skip expensive periodic work to reduce idle power draw and CAN bus load during pit/inspection.

```java
private static final Timer disabledTimer = new Timer();

// In robotPeriodic(), reset when enabled:
if (DriverStation.isEnabled()) disabledTimer.restart();

public static boolean shouldThrottle() {
  return disabledTimer.hasElapsed(5.0);
}
```

Gate expensive disabled-mode work (odometry history trimming, vision processing, etc.) with `if (!Robot.shouldThrottle())`.

**Files to change:**
- `Robot.java` — add `disabledTimer`, `shouldThrottle()`, reset on enable

**Done when:** Robot idles quietly after 5s disabled; no change to enabled behavior.

---

## 28. LaunchPreset Record + Hood Angle Offset Trim

Two competition-day shot-tuning quality-of-life additions to `ShotControl`.

**LaunchPreset record** — wrap each fixed-distance shot preset (tower, trench, etc.) as a `LoggedTunableNumber`-backed record so preset values are tunable from the dashboard without a redeploy:
```java
public record LaunchPreset(LoggedTunableNumber hoodAngleDeg, LoggedTunableNumber flywheelRPM) {}
public static final LaunchPreset towerPreset = new LaunchPreset(...);
public static final LaunchPreset trenchPreset = new LaunchPreset(...);
```

**Hood angle offset** — add an `incrementHoodAngleOffset(double deltaDeg)` method and bind operator POV-right / POV-left to `+0.2°` / `-0.2°` increments (with repeat on hold). The offset is added on top of the map lookup every cycle. When shots are consistently 1° low, the operator trims +1.0° without entering tuning mode or redeploying.

**Files to change:**
- `util/ShotControl.java` — add `LaunchPreset` record, named preset constants, `hoodAngleOffsetDeg` field, `incrementHoodAngleOffset()`
- `RobotContainer.java` — bind POV buttons to increment calls; display current offset on SmartDashboard

**Done when:** Fixed-distance presets are dashboard-tunable; operator can trim shot angle mid-match with POV buttons.

---

## 29. OverrideSwitches Controller

A dedicated `GenericHID` wrapper (port 5) for physical toggle switches on an operator console, replacing dangerous "override" bindings on the main controllers.

```java
public class OverrideSwitches {
  public Trigger driverSwitch(int index) { ... }    // 0-2
  public Trigger operatorSwitch(int index) { ... }  // 0-4
  public Trigger multiDirectionSwitchLeft() { ... }
  public Trigger multiDirectionSwitchRight() { ... }
}
```

**Suggested mappings:**
- Driver switch 1 → robot-relative drive
- Driver switch 2 → superstructure coast (only takes effect when disabled)
- Operator switch 0 → disable auto flywheel spinup
- Multi-direction switch → lost/won auto override for game-state logic

**Files to change:**
- Add `util/controllers/OverrideSwitches.java`
- `RobotContainer.java` — instantiate `new OverrideSwitches(5)`, replace dangerous controller-button overrides

**Done when:** Critical overrides live on a dedicated toggle-switch device; accidental activation during a match is impossible.

---

## 30. RobotModeTriggers-Based Init Logic

Move all mode-transition logic out of `autonomousInit()` / `teleopInit()` / `disabledInit()` and into `RobotModeTriggers` bindings inside `configureButtonBindings()`.

**Why:** Keeps all robot-state transitions in one composable place. Enables patterns like "auto-deploy intake on teleop enable unless already deployed" that are awkward as imperative `init()` calls.

**Examples to migrate:**
```java
RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> intakeSolenoid.deploy()));
RobotModeTriggers.teleop().onTrue(hood.zeroCommand().unless(hood::isZeroed));
RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> autoStart = Timer.getTimestamp()));
RobotModeTriggers.disabled().onFalse(Commands.runOnce(() -> coastOverride = false).ignoringDisable(true));
```

**Files to change:**
- `RobotContainer.java` — add `RobotModeTriggers` bindings in `configureButtonBindings()`
- `Robot.java` — reduce `autonomousInit()` / `teleopInit()` to only what must stay there (`autonomousCommand.cancel()`)

**Done when:** All mode-transition side-effects are trigger-driven and visible in the command lifecycle log.

---

## 31. RoboRioSim Team Number + @ExtensionMethod

Two one-line additions.

**`RoboRioSim.setTeamNumber()`** — without this, NT topics in sim use team number 0, breaking any dashboard that filters by team number:
```java
RoboRioSim.setTeamNumber(YOUR_TEAM_NUMBER); // in Robot() constructor
```

**`@ExtensionMethod({TriggerUtil.class})`** — add this Lombok annotation to `RobotContainer` once `TriggerUtil` (item 19) is in place. Allows writing:
```java
secondary.b().doublePress()  // instead of TriggerUtil.doublePress(secondary.b())
```
Makes all double-press bindings read exactly like normal trigger chains.

**Files to change:**
- `Robot.java` — add `RoboRioSim.setTeamNumber(YOUR_TEAM_NUMBER)` in constructor (inside `if (getMode() == SIM)`)
- `RobotContainer.java` — add `@ExtensionMethod({TriggerUtil.class})` class annotation

**Done when:** Sim NT topics use correct team number; double-press bindings use fluent syntax.

---

## Completion Checklist

- [x] LoggedTunableNumber
- [x] LoggedTracer
- [x] Command lifecycle logging
- [x] ShooterIOSim
- [x] HoodIOSim
- [x] IntakeIOSim
- [x] RackIOSim
- [x] FeederIOSim
- [~] Mechanism2d visualization (skipped)
- [x] IO output struct refactor
- [~] Parameterized AutoSelector (skipped — keeping flat autoChooser)
- [x] CheckDeploy build task
- [x] Watchdog loop overrun suppression
- [x] VirtualSubsystem + Vision conversion
- [x] LimelightThread (vision loop overrun fix)
- [x] BatteryLogger + CurrentLimits (energy logging)
- [~] Interpolation maps for shot control (skipped — happy with polynomial)
- [x] RobotState singleton
- [x] FullSubsystem deferred output application
- [x] showHardwareAlerts() + auto duration logging
- [x] Controller disconnection alerts + rumble
- [x] Robot.java QoL additions (joystick silence, Rotation2d silence, hostname, sim alliance)
- [x] Utility classes (ContinuousConditionalCommand, SuppliedWaitCommand, TriggerUtil)
- [x] Build system (deleteOldFiles, CheckPullRequest, license headers)
- [x] GeomUtil utility class
- [x] EqualsUtil + Container\<T\>
- [x] SwitchableChooser
- [x] AllianceFlipUtil upgrades (3D overloads + HAL guard)
- [x] Bounds record
- [x] DriveToPose command
- [x] shouldThrottle() disabled-mode throttling
- [x] LaunchPreset record + hood angle offset trim
- [x] OverrideSwitches controller
- [x] RobotModeTriggers-based init logic
- [x] RoboRioSim team number + @ExtensionMethod
