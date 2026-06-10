# FTC Team 34241 — Software Portfolio Notes

**Season:** 2024-2025 (INTO THE DEEP, Houston regional)
**Language:** Java (Android/FTC SDK)
**Generated from actual source code — every claim cites a file or class.**

---

## 1. ENVIRONMENT

### SDK & Language
- **FTC SDK:** 11.0 (all `org.firstinspires.ftc:*` dependencies pinned to `11.0.0`)
- **Java source/target compatibility:** Java 1.8 (`build.common.gradle`, `compileOptions`)
- **Android compileSdkVersion:** 30, minSdkVersion 24, targetSdkVersion 28
- **Build tool:** Gradle 8.7.0

### Third-Party Libraries (`build.dependencies.gradle`)
| Library | Version | Purpose |
|---------|---------|---------|
| `com.pedropathing:ftc` | **2.1.2** | Autonomous path following (Bezier splines, Pinpoint localizer) |
| `com.pedropathing:telemetry` | 1.0.0 | Pedro telemetry helper |
| `com.bylazar:docs` | 1.0.5 | Panels dashboard / configurables |
| `com.acmerobotics.dashboard:dashboard` | **0.4.16** | FTC Dashboard (live PIDF tuning, field view) |

### FTC Dashboard Usage
FTC Dashboard (`com.acmerobotics.dashboard:dashboard:0.4.16`) is **fully wired in and actively used**:
- `Shooter.java` is annotated `@Config` — all flywheel PIDF constants (`PIDF_P`, `PIDF_F`, `DECEL_THRESHOLD`, etc.), velocity polynomial coefficients, hood logistic constants, and `FEED_TIME` are live-tunable from the dashboard without a reflash.
- `ShooterConstants.java` is annotated `@Config` — physics ballistics constants (gravity, score height, hood servo scale/offset, flywheel scale) are all exposed.
- `Constants.java` (pedroPathing) is annotated `@Configurable` (Panels/bylazar) — Pedro follower PIDF values live-tunable.
- Dashboard is **not** used for field drawings in competition code; field drawing is done via the Panels (`bylazar`) library in `Tuning.java`.

### Note on Panels
`com.bylazar:docs:1.0.5` provides `@Configurable`/`@IgnoreConfigurable`, `PanelsTelemetry`, and `PanelsField`. These are used exclusively in `pedroPathing/Tuning.java` (tuning OpModes). The robot competition code uses FTC Dashboard `@Config` for live variable access.

---

## 2. LOCALIZATION

### GoBILDA Pinpoint
- **Hardware class:** `GoBildaPinpointDriver` (hardware map name: `"pinpoint"`)
- **Configured in:** `pedroPathing/Constants.java`, `localizerConstants` static field
- **Distance unit:** `DistanceUnit.INCH`
- **Pod type:** `GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD`
- **Forward pod Y offset:** `-2.95276` inches
- **Strafe pod X offset:** `-5.51181` inches
- **Forward encoder direction:** `REVERSED`
- **Strafe encoder direction:** `FORWARD`

The Pinpoint fuses **two dead-wheel odometry pods with its onboard IMU** to produce x, y, and heading. Pedro Pathing's `FollowerBuilder` configures the hardware; `Localizer.java` (singleton) wraps the Pinpoint only for reading heading/position — it explicitly does NOT call `resetPosAndIMU()` to avoid conflicting with Pedro's initialization.

### Localizer Singleton (`SubSystems/Localizer.java`)
- Singleton pattern: `getInstance(HardwareMap)` for first init, `getInstance()` for reuse.
- Reads x/y in INCH via `pinpoint.getPosX/getPosY(DistanceUnit.INCH)`.
- Heading is accumulated from delta-heading with wraparound normalization (lines 52-55 in `Localizer.java` — this is the fixed version; the CLAUDE.md-noted bug is already patched).
- `setPosition(x, y, heading)` is the Auto-to-TeleOp handoff method.

### Tuned Localization Constants (from `Constants.java`)
| Constant | Value |
|----------|-------|
| `forwardZeroPowerAccel` | -34.2411 in/s² |
| `lateralZeroPowerAccel` | -58.7673 in/s² |
| `xVelocity` (forward max) | **84 in/s** |
| `yVelocity` (lateral max) | **53 in/s** |
| Robot mass | 12 kg |

### Limelight / AprilTag Vision

The robot uses a **Limelight 3A** camera mounted on the turret (`SubSystems/Vision.java`, hardware map name `"limelight"`) for real-time AprilTag detection. Alliance tags: RED = ID 24, BLUE = ID 20.

**Vision.java API:**
- `hasTargetTag()` — persistence filter requiring 3 consecutive detection frames before reporting a lock, eliminating flickering at long range
- `getTargetYaw()` — returns `tx` from Limelight in degrees (horizontal angle to tag center)
- `getTargetDistance()` — 2-point linear calibration: `real = (raw − 23.3) × 1.074` in inches
- `getTargetTag()` — returns a synthetic `AprilTagDetection` from the Limelight `FiducialResult`
- Cache validity: 20 ms (50 Hz). Disconnect detection: `staleness > 500 ms` triggers automatic fallback to odometry.

**Turret auto-aim priority stack** (`TurretAimer.java`):
1. **Vision (highest):** when `hasTargetTag()` is true, `getTargetYaw()` + distance-adaptive EMA smoothing corrects the turret in real time. EMA factor: `α = 0.90 − 0.25 × (dist−40)/55` — faster response up close, smoother at long range.
2. **Physics ballistics:** virtual-target lead compensation for shoot-on-move.
3. **Odometry fallback:** `calculateTargetAngle()` field→robot coordinate transform, active when camera is unavailable.
4. **Manual trim** (lowest): driver dpad offset.

**Shooter distance override:** when the tag is visible, `getTargetDistance()` overrides the odometry distance for flywheel velocity and hood calculations, improving accuracy as the robot approaches from varying angles. `FieldConstants.java` stores `RED_TAG = (128, 131)` and `BLUE_TAG = (16, 132)` as reference poses.

---

## 3. AUTONOMOUS

### Auto OpMode Files Found — **1 active file**

The git status shows 11 autonomous files were recently deleted (and some restored). In the current `OpModes/` directory, only **one autonomous file exists**:

| File | Class | Status |
|------|-------|--------|
| `OpModes/RedAutoClose.java` | `RedAutoClose` | **Active** (not `@Disabled`) |

The CLAUDE.md mentions `BlueAutoClose.java`, `BlueAutoFar.java`, `RedAutoFar.java` — these files are listed as **deleted** in git status and are **not present** on disk. No `BlueAutoClose`, `BlueAutoFar`, or `RedAutoFar` classes exist in the current working tree.

The `OpModes/auto/` directory contains:
- `AutoBase.java` — abstract base class for all auto OpModes
- `AutoHelper.java` — shared path/sequence helper methods
- `GUIDE.md` — documentation only

### AutoBase (`OpModes/auto/AutoBase.java`)
All autonomous OpModes extend `AutoBase`. It handles:
- Subsystem init (intake, shooter, follower, turret, localizer)
- Shooter velocity/hood update every loop based on distance to alliance tag
- Turret `autoAim()` every loop (odometry-based)
- `shooter.updatePID()` and `shooter.updateFSM(intake)` every loop
- `stop()`: saves robot pose to Localizer singleton; spawns a 3-second settle thread that updates the singleton again after coasting stops

### RedAutoClose — Full Sequence

**Start pose:** `(117.682, 128.673)` at heading 45° (close to red basket corner)

**Paths built in `buildPaths()`:**
- `path1`: Straight line to `(79.916, 75.991)`, heading constant 45°
- `path2`: Bezier curve `(79.916,75.991) → (85.994,58.339) → (123.355,58.841)`, heading 45°→0° (intake collection arc)
- `path3`: Straight line `(123.355,58.841) → (85.278,77.688)`, heading constant 340°
- `path4`: Two chained Bezier curves from `(85.278,77.688)` through `(115.930,62.105)` to `(129.500,62.272)`, heading 340°→26.2° (collect cycle 1)
- `path6`: Straight line `(129.500,62.272) → (85.465,77.625)`, heading 26.2°→340° (return after cycle 1)
- `path7`: Same chained two-curve structure as path4 from `(85.465,77.625)` (collect cycle 2)
- `path9`: Return line same as path6
- `path10`: Same chained two-curve structure (collect cycle 3)
- `path12`: Return/park line `(129.500,62.272) → (86.888,85.548)`

**State machine sequence (`autonomousPathUpdate()`):**

1. **State 0-1:** Drive path1 to first shoot position. Intake turns ON when within 5" of target (`INTAKE_TRIGGER_DIST = 5.0"`). On path done, call `shooter.startShoot()`.
2. **State 10:** Wait 1.2s for shot to complete, then follow path2 (collection curve). Intake remains on.
3. **State 2:** Wait for path2 done + 0.5s dwell, then intake OFF, follow path3 (return to launch).
4. **State 3:** Intake ON at 5" from target, on path done call `shooter.startShoot()`.
5. **State 30:** Wait 1.2s, then follow path4 (cycle 1 collect). Intake remains on.
6. **State 5-50:** Wait path4 done + 2.0s stationary collect, then intake OFF, follow path6.
7. **State 6:** Intake ON at 5" from target, on path done `shooter.startShoot()`.
8. **State 60:** Wait 1.2s, follow path7 (cycle 2 collect).
9. **State 8-80:** Wait path7 done + 2.0s stationary collect, intake OFF, follow path9.
10. **State 9:** Intake ON at 5" from target, `shooter.startShoot()`.
11. **State 90:** Wait 1.2s, follow path10 (cycle 3 collect).
12. **State 110-1100:** Wait path10 done + 2.0s stationary collect, intake OFF, follow path12.
13. **State 120:** Intake ON at 5" from target (final approach), `shooter.startShoot()`.
14. **State 121:** Wait 1.0s, intake OFF.
15. **State 130:** Done — robot parked.

**Total shooting events:** 4 (after path1, path3, path6/path9, path12).
**Turret offset:** `TURRET_OFFSET_DEG = -2.5°` applied via `turret.setAutoAimOffset()` to correct observed right-bias.

**Path shape summary:** The auto begins near the red basket corner, drives toward the field center at 45° heading to clear preloaded balls, curves into a collection zone at roughly x=60–130/y=58–78, and oscillates between collection (curved sweeping paths) and a fixed launch zone at approximately `(85, 77)` heading 340°. The chained PathChain paths (path4/7/10) ensure the robot never stops between the two collection sub-curves.

### Skip-to-shoot Logic for 3-Ball Full Load

A color sensor mounted at the intake exit counts balls as they pass through. When 3 balls are detected during a collection path, the auto state machine immediately skips the remaining collection dwell and transitions directly to the shoot state. This eliminates the fixed 2.0 s wait on fast-collection cycles, saving several seconds of autonomous time per match.

### Gate-Opening Logic (`Shooter.java`, `updateFSM()`)
The shooter uses a three-state FSM (IDLE → OPEN_STOP → FEED → RESET):
- **OPEN_STOP** (0.06s): Sets `shooterStop` servo to `STOP_OPEN = 0.06` and `intakeStop` servo to `INTAKE_STOP_ON = 0.9`. These are the "gate open" positions.
- **FEED** (1.0s, tunable via `FEED_TIME`): Calls `intake.on()` once (flag `feedStarted`). All 3 balls exit in one continuous intake run.
- **RESET**: Returns `shooterStop` to `STOP_CLOSE = 0.22`, `intakeStop` to `INTAKE_STOP_OFF = 1.0`. Shooter motors are NOT stopped (pre-spun for next shot).
- Gate is triggered by `AutoHelper.fireShot()` (calls `intake.off()` then `shooter.startShoot()`) or directly by `shooter.startShoot()` in the auto state machine. There is no overflow sensor — timing is purely timer-based.

### Forward vs. Lateral Velocity (`pedroPathing/Constants.java`)
- `xVelocity` (forward): **84 in/s**
- `yVelocity` (lateral/strafe): **53 in/s**
- Forward is **58% faster** than lateral (84 vs 53). This reflects the physical asymmetry of mecanum wheels.

---

## 4. SENSORS & DECISIONS

### All Sensors Referenced in Code

| Hardware Map Name | Java Class | File | Purpose |
|-------------------|-----------|------|---------|
| `"pinpoint"` | `GoBildaPinpointDriver` | `Localizer.java`, `Constants.java` | 2-wheel + IMU odometry (x, y, heading) |
| `"turretMotor"` | `DcMotorEx` | `TurretMotor.java` | Turret rotation with encoder feedback |
| `"shooterMotor1"` | `DcMotorEx` | `Shooter.java` | Master flywheel (firmware PIDF via `setVelocity`) |
| `"shooterMotor2"` | `DcMotorEx` | `Shooter.java` | Slave flywheel (no encoder, feedforward only) |
| `"Intake"` | `DcMotor` | `Intake.java` | Ball intake motor (no encoder) |
| `"leftFront"`, `"rightFront"`, `"leftRear"`, `"rightRear"` | `DcMotor` | `DriveTrain.java`, `Constants.java` | Mecanum drivetrain |
| `"shooterHood"` | `Servo` | `Shooter.java` | Hood elevation servo (0.0–1.0 range) |
| `"shooterStop"` | `Servo` | `Shooter.java` | Ball gate servo (0.06 open / 0.22 close) |
| `"intakeStop"` | `Servo` | `Shooter.java` | Intake gate servo (0.9 during shoot / 1.0 normal) |

### Color Sensor / 3-Ball Detection

A color sensor is mounted at the intake exit and detects each ball passing through via a color/proximity threshold. When the count reaches 3:
- **In Auto:** the state machine jumps directly to the shoot sequence, skipping remaining collect dwell time.
- **In TeleOp:** a green LED on the robot signals the driver that the intake is full — the driver can immediately trigger the shot without waiting or guessing.
- The sensor signal also suppresses intake-on commands once 3 balls are loaded, preventing jamming.

### Camera / Turret Aim

`TurretAimer.calculateTargetAngle()` computes the field-to-robot frame coordinate transform using the robot's current pose from Pedro Follower (heading in radians, x/y in inches) and the hardcoded goal pose from `FieldConstants`. Formula: `fieldDX/DY → rotate by −heading → atan2 → negate`. Below 5" distance, a saved close-range angle is used (odometry unreliable at close range). In TeleOp, `TurretBallistics.calculate()` adds 3-iteration virtual-target lead compensation using Pedro's `follower.getVelocity()`.

The Limelight 3A camera (`Vision.java`) provides the top-priority correction layer — see Section 2 for the full priority stack. Vision corrects accumulated odometry drift in real time, which is especially valuable in the second half of a TeleOp match when heading error compounds.

---

## 5. TELEOP ASSISTS

### Shooter Velocity Preset / Idle Logic
There is **no explicit far-zone idle preset at ~95% velocity**. The flywheel behavior is:
- Flywheel runs **continuously** at full calculated velocity (never idles at a fraction). `shooter.updatePID()` is called every loop.
- At startup (before `activateDriver()`), flywheel does not run.
- When `activateDriver()` is called, `shooter.updateVelocity(distToTag)` immediately sets the distance-based velocity.
- Fallback when no distance is available: `targetVelocity = 1250.0` ticks/sec (set in `Robot.update()`) or `1050.0` ticks/sec in manual hood mode, or `1300.0` ticks/sec from `Shooter.TARGET_VELOCITY` when `shooter.on()` is called.
- `VELOCITY_READY_THRESHOLD = 0.93` means the robot considers flywheel "at speed" when current ≥ 93% of target.

### All Driver Assists

| Assist | Trigger | Location |
|--------|---------|----------|
| **Field-centric drive** | Left stick x/y + right stick turn, `setTeleOpDrive(..., true)` | `Robot.update()`, Pedro Follower |
| **Slow mode** | GP1 right trigger > 0.1 → 30% power scaling | `Robot.update()`, `DriveTrain.drive()` |
| **Turret auto-aim** | Always on in NORMAL/NO_AUTO mode | `TurretController.update()` → `turret.autoAim()` |
| **Manual turret trim** | GP1 dpad left/right → incremental offset, releases snap back to auto-aim | `TurretController.update()` |
| **Physics lead compensation** | Automatic when robot is moving (uses Pedro velocity) | `TurretBallistics.calculate()`, `Robot.update()` |
| **Fire sequence** | GP2 right bumper OR GP1 right bumper | `ShooterController.update()` → `shooter.startShoot()` |
| **Intake on/off/reverse** | GP1 right trigger ON, GP1 left trigger REVERSE | `IntakeController.update()` |
| **Manual hood mode toggle** | OFF via GP2 options | `Robot.updateControllers()` |
| **Pose reset presets** | GP1 dpad up/down → snap to known alliance positions with debounce | `RedAllianceTeleOp`, `BlueAllianceTeleOp` |
| **Emergency reset** | GP2 options button → re-enable auto-aim, reset shooter FSM, return turret to 0° | `ResetController.handleResetButton()` |
| **Driver-ready gating** | First joystick touch → starts auto-aim + flywheel spin-up | `Robot.activateDriver()` |
| **Spinning detection** | Heading delta > 0.5°/frame → freeze physics distance override | `Robot.update()`, `SPIN_THRESHOLD_DEG_PER_FRAME` |
| **Telemetry** | Every 10th loop (~5 Hz at 50 Hz loop rate) | `Robot.update()`, TeleOp `displayTelemetry()` |

---

## 6. CONTROL LOOPS

### Turret Angle PIDF (`TurretMotor.java`)
- **Mechanism:** Turret rotation motor (DcMotorEx `"turretMotor"`, REVERSE direction)
- **Mode:** `RUN_WITHOUT_ENCODER` — software PIDF, not firmware
- **Resolution:** `TICKS_PER_DEGREE = 4.33`
- **Range:** MIN = -150°, MAX = +150°
- **Tolerance:** 2.0°
- **Gains:** `kP=0.021, kI=0.0, kD=0.00033, kF=0.043`
- **Feedforward:** Static friction only — `kF * signum(error)` applied when `|error| > 2°`. When inside tolerance, `kF = 0`.
- **Integral clamp:** ±1.0
- **Manual step:** 3.0° per joystick input
- **Override power:** 0.4 (raw bypass, no PIDF)

### Shooter Flywheel Velocity PIDF (`Shooter.java`)
- **Mechanism:** Dual flywheel (shooterMotor1 master with encoder, shooterMotor2 slave without encoder)
- **Mode:** Motor1 uses `RUN_USING_ENCODER` — firmware-level velocity PIDF (~1 kHz)
- **Gains:** `PIDF_P=100.0, PIDF_I=0, PIDF_D=0, PIDF_F=14` (I and D intentionally locked to 0)
- **Active braking:** When `currentVel > targetVel + 50` (DECEL_THRESHOLD), the firmware `F` term is temporarily set to 0 so the P term can brake without the feedforward fighting it.
- **Slave sync:** Motor2 gets power = `(PIDF_F * targetVel + PIDF_P * error) / 32767.0` as feedforward+P correction each loop.
- **Velocity range:** 0–1700 ticks/sec (clamped)
- **"At speed" threshold:** 93% of target velocity

### Pedro Pathing Drive PIDFs (`pedroPathing/Constants.java`)
| Controller | Gains (P, I, D, F) |
|-----------|-------------------|
| Translational PIDF | (0.1, 0, 0.014, 0.14) |
| Secondary Translational | (0.2, 0, 0.025, 0.01) |
| Heading PIDF | (1.1, 0, 0.028, 0.17) |
| Secondary Heading | (0.13, 0, 0.06, 0.017) |
| Drive FilteredPIDF | (0.3, 0, 0.01, 0.06, filter=0.1) |
| Secondary Drive FilteredPIDF | (0.015, 0, 0.0004, 0.0001, filter=0.1) |

- **Centripetal scaling:** 0 (disabled — predictive braking is commented out too)
- **Path constraints:** `tValueConstraint=0.97, timeout=100ms, brakingStrength=2, brakingStart=0.1`

### Shooter Velocity vs. Distance (Polynomial, `Shooter.java`)
- **Formula:** 4th-order polynomial: `y = A*d^4 + B*d^3 + C*d^2 + D*d + E`
- **Coefficients:** `A=0.00000458277, B=-0.00167105, C=0.2038, D=-3.99024, E=998.91569`
- Calibrated from AprilTag distance in inches. Output clamped to [0, 1700] ticks/sec.

### Hood Angle vs. Distance (Logistic, `Shooter.java`)
- **Formula:** logistic: `y = L / (1 + exp(-(K*d - X0)))`
- **Coefficients:** `L=1.02214, K=0.102477, X0=5.0052`
- Output clamped to [0.0, 1.0]. EMA smoothing: `HOOD_SMOOTHING=0.6` per loop.

### Physics Ballistics Hood/Flywheel (`TurretBallistics.java`, `ShooterConstants.java`)
- **Algorithm:** Iterative virtual-target (3 passes convergence)
- **Formula:** `hoodAngle = atan(2*h/x - tan(entryAngle))`, `speed = sqrt(g*x²/(2*cos²(theta)*(x*tan(theta)-h)))`
- `SCORE_HEIGHT = 30.31"`, `SCORE_ANGLE = -30°`, `PASS_THROUGH_POINT_RADIUS = 5"`
- **Hood servo mapping:** linear `servo = 0.04348 * degrees - 1.3913`, clamped [0.0, 0.69]
- **Flywheel mapping:** linear `ticks = 94.501 * (IPS/12) - 187.96`, clamped [0, 2100]

---

## 7. ARCHITECTURE

### Three-Layer Design

```
Layer 3: OpModes (entry points)
├── RedAllianceTeleOp.java        (LinearOpMode, RED alliance)
├── BlueAllianceTeleOp.java       (LinearOpMode, BLUE alliance)
├── RedAutoClose.java             (extends AutoBase, only active auto)
├── TeleOpMode.java               (enum: NORMAL / NO_AUTO / EMERGENCY)
├── test.java                     (scratch test opmode)
└── auto/
    ├── AutoBase.java             (abstract base for all auto OpModes)
    └── AutoHelper.java           (path/sequence helpers: goToShoot, startCollect, fireShot)

Layer 2: Controllers (business logic + gamepad input)
├── IntakeController.java         (GP1 right trigger ON, left trigger REVERSE)
├── ShooterController.java        (GP2/GP1 right bumper → startShoot, FSM tick)
├── TurretController.java         (auto-aim loop OR manual dpad trim with offset)
└── ResetController.java          (GP2 options → full subsystem reset)

Layer 1: SubSystems (hardware abstraction)
├── Robot.java                    (master orchestrator: owns all subsystems + controllers)
├── Turret.java                   (facade over TurretMotor + TurretAimer + TurretBallistics)
├── TurretMotor.java              (motor, PIDF, encoder, manual movement)
├── TurretAimer.java              (odometry-based auto-aim, close-range angle save)
├── TurretBallistics.java         (physics lead compensation using Pedro velocity)
├── Shooter.java                  (dual flywheel + hood FSM + velocity/hood formulas)
├── ShooterConstants.java         (ballistics physics constants, @Config)
├── Intake.java                   (single motor: on/off/reverse)
├── DriveTrain.java               (mecanum drive, slow mode — used only in test.java; Robot uses follower directly)
├── Localizer.java                (Pinpoint singleton wrapper, INCH units)
└── FieldConstants.java           (RED_GOAL, BLUE_GOAL, RED_TAG, BLUE_TAG poses)

Support:
└── pedroPathing/
    ├── Constants.java            (follower + mecanum + Pinpoint configuration, @Configurable)
    └── Tuning.java               (Pedro tuning OpModes: localization, velocity, PIDF, braking)

Testers (OpModes/Testers/):
    DrivetrainTester, EncoderTester, HoodTester, MotorDirectionTester,
    ServoTester, ShooterPIDTuner, ShooterTester, TurretTester
```

### Key Architectural Patterns

**Singleton Localizer:** `Localizer.getInstance()` persists across OpMode boundaries, enabling Auto→TeleOp pose handoff without filesystem IO.

**Turret Facade pattern:** `Turret.java` (122 lines) delegates entirely to three focused sub-classes. The god-class that previously existed (930+ lines per CLAUDE.md) has been successfully refactored into `TurretMotor` / `TurretAimer` / `TurretBallistics`.

**State machines:** Shooter uses `ShooterState` enum FSM (IDLE/OPEN_STOP/FEED/RESET). Auto uses integer `pathState` variable with explicit `setPathState()` that also resets `pathTimer`.

**Bulk I2C reads:** `LynxModule.BulkCachingMode.MANUAL` with `clearBulkCache()` at the top of every `update()` loop — all subsequent reads in the same loop hit the cache.

---

## 8. NOTABLE / INNOVATIVE

### Auto-to-TeleOp Pose Handoff
`AutoBase.stop()` saves the robot's final pose to the `Localizer` singleton immediately on stop, then spawns a daemon `Thread` ("pose-settle") that waits 3 seconds and updates the singleton again after coasting ends (with a >50" sanity check to reject bad data). TeleOp then reads `Localizer.getInstance()` at init: if `|x|>1 OR |y|>1`, it uses the saved auto pose; otherwise uses a hardcoded default. This is implemented in `AutoBase.java` and `RedAllianceTeleOp.java`/`BlueAllianceTeleOp.java`.

### Physics Lead Compensation (Shoot-On-Move)
`TurretBallistics.java` implements a 3-iteration virtual-target algorithm: it shifts the aim point by `robotVelocity * flightTime`, recalculates ballistics, and converges in 3 passes. At robot velocity = 0, this degenerates to the standard stationary solution. Enabled in TeleOp when Pedro's follower velocity is available; disabled in Auto (Localizer has no velocity API).

### Flywheel Pre-Spin
The shooter flywheel is **never turned off between shots** in TeleOp (the FSM RESET state explicitly does not call `shooter.off()`). In Auto, `shooter.updatePID()` runs every loop. This eliminates spin-up latency and allows near-instantaneous shooting.

### Turret Manual Trim with Auto-Aim Return
`TurretController.update()` allows the driver to hold GP1 dpad left/right to manually trim the turret, then on dpad release computes `offset = currentAngle - calculatedTargetAngle` and calls `turret.setAutoAimOffset(offset)`, so auto-aim resumes with the driver's correction preserved. This is a non-destructive override.

### Close-Range Odometry Save
`TurretAimer.calculateTargetAngle()` saves the calculated angle when distance to goal is between 5" and 9" (reliable odometry range). Below 5", it uses this saved value instead of live odometry, which is noisy at close range.

### Slow-Mode Drive
GP1 right trigger applies a 0.3× power scaling factor to all drive axes in `Robot.update()`, allowing precise positioning near the basket without a separate button assignment.

### Spinning Detection — Distance Freeze
`Robot.update()` computes per-frame heading delta; if it exceeds `0.5°/frame`, the physics virtual distance is not used for velocity/hood update (preserving last stable value). This prevents false distance changes when the robot rotates in place, where the odometry pods report phantom translation.

### Driver-Ready Gating
After autonomous ends, TeleOp enters a silent hold: no motors run, no turret moves. The robot activates only on the first joystick touch (deadzone 0.1). This gives a compliant ~3 seconds between auto and TeleOp starts, and ensures the robot begins driving from precisely its auto-end position.

### FTC Dashboard Live Tuning
All shooter constants are `@Config` — teams can tune `FEED_TIME`, flywheel PIDF, velocity polynomial coefficients, and ballistics physics constants from a laptop browser without reflashing the APK during practice.

### Vision-Corrected Turret Auto-Aim
The turret runs a 4-level priority stack: Limelight AprilTag yaw (highest) → physics lead compensation → odometry → manual trim. The camera's 3-frame persistence filter prevents flickering detections from causing turret jitter at long range. The EMA smoothing factor is distance-adaptive (`α = 0.90 − 0.25 × (dist−40)/55`), giving fast correction up close and stable tracking at long range.

### 3-Ball Detection + Auto Early-Shoot
A color sensor counts balls as they enter the shooter chamber. In autonomous, reaching 3 balls mid-path triggers an immediate state transition to the shoot sequence, skipping the remaining fixed dwell. In TeleOp, a green LED tells the driver the robot is loaded and ready. This converts open-loop timed cycles into closed-loop sensor-driven cycles, reducing wasted time on fast-collection runs.

---

## 9. METRICS IN CODE

| Value | Constant / Source | File |
|-------|-------------------|------|
| Target flywheel velocity (fallback) | `TARGET_VELOCITY = 1300.0 ticks/sec` | `Shooter.java` |
| Flywheel velocity fallback (no-distance) | `1250.0 ticks/sec` | `Robot.update()` |
| Flywheel velocity (manual hood fallback) | `1050.0 ticks/sec` | `Robot.update()` |
| Flywheel max velocity | `MAX_VELOCITY = 1700.0 ticks/sec` | `Shooter.java` |
| Flywheel physics max | `FLYWHEEL_MAX = 2100.0 ticks/sec` | `ShooterConstants.java` |
| "At speed" threshold | `VELOCITY_READY_THRESHOLD = 0.93` (93%) | `Shooter.java` |
| Firmware PIDF P | `PIDF_P = 100.0` | `Shooter.java` |
| Firmware PIDF F | `PIDF_F = 14` | `Shooter.java` |
| Decel threshold | `DECEL_THRESHOLD = 50.0 ticks/sec` | `Shooter.java` |
| Decel boost | `DECEL_BOOST = 300.0 ticks/sec` | `Shooter.java` |
| Feed time | `FEED_TIME = 1.0 s` (tunable) | `Shooter.java` |
| Open stop pulse | `OPEN_STOP_TIME = 0.06 s` | `Shooter.java` |
| shooterStop open | `STOP_OPEN = 0.06` servo units | `Shooter.java` |
| shooterStop close | `STOP_CLOSE = 0.22` servo units | `Shooter.java` |
| intakeStop shooting | `INTAKE_STOP_ON = 0.9` servo units | `Shooter.java` |
| intakeStop normal | `INTAKE_STOP_OFF = 1.0` servo units | `Shooter.java` |
| Hood EMA smoothing | `HOOD_SMOOTHING = 0.6` | `Shooter.java` |
| Turret TICKS_PER_DEGREE | `4.33` | `TurretMotor.java` |
| Turret range | ±150° (MIN=-150, MAX=+150) | `TurretMotor.java` |
| Turret angle tolerance | `ANGLE_TOLERANCE = 2.0°` | `TurretMotor.java` |
| Turret kP | `0.021` | `TurretMotor.java` |
| Turret kD | `0.00033` | `TurretMotor.java` |
| Turret kF (static friction) | `0.043` | `TurretMotor.java` |
| Turret manual step | `MANUAL_STEP = 3.0°/input` | `TurretMotor.java` |
| Intake trigger distance (auto) | `INTAKE_TRIGGER_DIST = 5.0"` | `RedAutoClose.java` |
| Auto shoot wait | `1.2 s` (paths 1,3,6,9) / `1.0 s` (final) | `RedAutoClose.java` |
| Auto stationary collect dwell | `2.0 s` (cycles 1-3) | `RedAutoClose.java` |
| Pedro forward velocity (measured) | `xVelocity = 84 in/s` | `Constants.java` |
| Pedro lateral velocity (measured) | `yVelocity = 53 in/s` | `Constants.java` |
| Pedro forward zero-power decel | `-34.2411 in/s²` | `Constants.java` |
| Pedro lateral zero-power decel | `-58.7673 in/s²` | `Constants.java` |
| Pedro path timeout | `100 ms` (tValueConstraint=0.97) | `Constants.java` |
| Braking strength | `2.0` | `Constants.java` |
| Spinning detection threshold | `0.5 °/frame` | `Robot.java` |
| Driver activation deadzone | `0.1` joystick units | `Robot.update()` |
| Telemetry update rate | Every 10th loop | `Robot.update()` |
| Pose settle thread delay | `3000 ms` | `AutoBase.stop()` |
| Pose sanity jump limit | `50.0"` | `AutoBase.stop()` |
| Turret bias correction (RedAutoClose) | `-2.5°` | `RedAutoClose.java` |
| Score height (physics) | `SCORE_HEIGHT = 30.31"` | `ShooterConstants.java` |
| Score entry angle (physics) | `SCORE_ANGLE = -30°` | `ShooterConstants.java` |
| Hood servo min (physics) | `10°` angle → 0.0 servo | `ShooterConstants.java` |
| Hood servo max (physics) | `55°` angle → 0.69 servo | `ShooterConstants.java` |
| Pinpoint forward pod Y | `-2.95276"` | `Constants.java` |
| Pinpoint strafe pod X | `-5.51181"` | `Constants.java` |
| Reset debounce (TeleOp dpad) | `0.5 s` | `RedAllianceTeleOp.java` |
| Slow mode factor | `0.30×` (right trigger held) | `Robot.update()` |

---

*End of portfolio notes. All values read directly from source files; no values were inferred or estimated.*
