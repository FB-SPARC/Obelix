# Implementation Order

Sorted by **impact × ease**. Do the top tiers first — they unlock everything below them.

---

## Tier 1 — Do These First (Tiny, No Dependencies, Immediate Value)

These are all under 20 lines each and have zero dependencies on other items.

| # | Item | Why First |
|---|------|-----------|
| 22 | `EqualsUtil` + `Container<T>` | 2 files, ~20 lines total. Used everywhere once other items land. |
| 25 | `Bounds` record | 1 file, ~20 lines. Needed by shot control and AllianceFlipUtil. |
| 24 | `AllianceFlipUtil` upgrades | Add 4 overloads + 1 guard to existing file. 10 lines. |
| 31 | `RoboRioSim` team number + `@ExtensionMethod` | 2 lines across 2 files. |
| 18 | Robot.java QoL (joystick silence, Rotation2d silence, hostname, sim alliance) | 4 blocks, each 1–5 lines in `Robot.java` constructor. |
| 13 | Watchdog loop overrun suppression | ~10 lines in `Robot.java` constructor. Already written in the doc. |
| 16 | `showHardwareAlerts()` + auto duration logging | ~15 lines in `Robot.java`. Pure upside. |
| 27 | `shouldThrottle()` disabled-mode throttling | 1 timer + 1 method + 1 reset in `Robot.java`. |

---

## Tier 2 — Quick Wins (Small Files, Self-Contained)

Each is one new file or a small edit. No subsystem changes needed.

| # | Item | Why Here |
|---|------|----------|
| 1  | `LoggedTunableNumber` | One new file. Unlocks items 6, 28, and all PID tuning. |
| 2  | `LoggedTracer` | One new file + calls in `Robot.java`. Zero risk. |
| 3  | Command lifecycle logging | 5 lines in `Robot.java`. Huge replay value. |
| 21 | `GeomUtil` utility class | One new file. Needed cleanly by DriveToPose and RobotState. |
| 23 | `SwitchableChooser` | One new file. Needed by AutoSelector (item 7). |
| 10 | `VirtualSubsystem` + Vision conversion | One new file + convert Vision. Prerequisite for item 11. |
| 15 | `FullSubsystem` deferred output | One new file + small Drive refactor. |
| 8  | `CheckDeploy` build safety | One class + one Gradle task. Prevents disasters. |
| 20 | Build system cleanup (`deleteOldFiles`, license headers) | 1–2 lines in `build.gradle`. |

---

## Tier 3 — Medium Effort (New Subsystem/Singleton Pattern)

Require touching multiple files but follow a clear pattern.

| # | Item | Notes |
|---|------|-------|
| 14 | `RobotState` singleton | Prerequisite for `DriveToPose` and shot-while-moving. Do before item 26. |
| 19 | Utility classes (`ContinuousConditionalCommand`, `SuppliedWaitCommand`, `TriggerUtil`) | 3 small files. `TriggerUtil` needed before item 31's `@ExtensionMethod`. |
| 17 | Controller disconnection alerts + rumble | Mostly `RobotContainer.java` edits. |
| 30 | `RobotModeTriggers`-based init logic | Refactor of existing `autonomousInit` / `teleopInit`. |
| 7  | Parameterized `AutoSelector` | Needs `SwitchableChooser` (item 23) first. |
| 28 | `LaunchPreset` + hood angle offset trim | Needs `LoggedTunableNumber` (item 1) first. High competition value. |
| 29 | `OverrideSwitches` controller | One new file + `RobotContainer` wiring. Needs physical hardware to be useful. |
| 12 | `BatteryLogger` + `CurrentLimits` | Larger but self-contained. Do after `LoggedTracer`. |
| 9  | `CheckPullRequest` + CI | Needs `RobotType` enum from item 8 first. |

---

## Tier 4 — Larger Refactors (Touch Many Files)

Save these for when the Tier 1–3 foundations are in place.

| # | Item | Notes |
|---|------|-------|
| 26 | `DriveToPose` command | Needs `RobotState` (item 14) and `GeomUtil` (item 21) first. |
| 6  | IO output struct refactor | Touches every subsystem. Do in one sitting. |
| 11 | `LimelightThread` vision fix | Needs `VirtualSubsystem` (item 10) first. |
| 5  | `Mechanism2d` visualization | Needs sim IOs partially done first. |
| 4  | Subsystem simulation (`*IOSim`) | Do per-subsystem. Start with `ShooterIOSim`, then `HoodIOSim`. |

---

## Suggested First Session (≈ 2–3 hours)

Complete all of Tier 1 in one go — they're all in `Robot.java`, `AllianceFlipUtil.java`, and two tiny new files. You'll finish with:
- No more spurious loop overrun warnings
- No more false disconnect alerts on boot
- Hostname + platform in every log
- Sim alliance station fixed
- Disabled throttling working
- Auto duration printed to console

Then add `LoggedTunableNumber` (item 1) and `LoggedTracer` (item 2) as the first Tier 2 items — everything else builds on those.
