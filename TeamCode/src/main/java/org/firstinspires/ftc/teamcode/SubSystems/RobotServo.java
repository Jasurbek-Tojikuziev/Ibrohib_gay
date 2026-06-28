package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.Controllers.ShooterController;
import org.firstinspires.ftc.teamcode.OpModes.TeleOpMode;

/**
 * Robot orchestrator for servo-based turret (TurretServo instead of Turret motor + PIDF).
 * Everything else (drive, intake, shooter, vision, localizer) is identical to Robot.java.
 *
 * Auto-aim is done inline: odometry field→robot transform + optional vision EMA nudge.
 * Manual control: gamepad1 dpad_left / dpad_right adjust servo angle directly.
 */
public class RobotServo {

    private List<LynxModule> allHubs;

    public Follower      follower;
    public Vision        vision;
    public Intake        intake;
    public Shooter       shooter;
    public TurretServo   turretServo;

    public IntakeController  intakeController;
    public ShooterController shooterController;

    public String distanceSource   = "N/A";
    public double effectiveDistance = 0;
    public boolean manualHoodMode  = false;

    private TeleOpMode teleOpMode;
    private boolean    isRedAlliance;
    private boolean    driverReady    = false;
    private boolean    prevFireButton = false;

    // ── Turret servo aim state ───────────────────────────────────────────────

    private Pose    goalPose              = null;
    private boolean autoAimEnabled        = true;
    private double  autoAimTrim           = 0.0;  // driver trim kept across auto→manual→auto
    private double  visionSmoothedTy      = 0.0;  // EMA of camera ty (sideways → left/right)
    private double  savedCloseRangeAngle  = Double.NaN;

    // Mirror constants from TurretAimer so they stay in sync
    private static final double AIM_TRIM_RED  =  1.0;
    private static final double AIM_TRIM_BLUE = -0.5;
    private static final double VISION_TX_EMA =  0.4;  // weight on new camera sample

    // Manual speed: degrees per update-loop tick
    private static final double MANUAL_DEG_SLOW = 0.5;
    private static final double MANUAL_DEG_FAST = 2.0;
    private static final double MANUAL_HOLD_SEC = 1.0;

    private boolean     wasManualActive = false;
    private ElapsedTime dpadHoldTimer   = new ElapsedTime();
    private double      manualAngle     = 0.0;

    // ── Spinning detection ───────────────────────────────────────────────────

    private static final double SPIN_THRESHOLD_DEG_PER_FRAME = 0.5;
    private double prevHeading = Double.NaN;

    // ── Loop timing ─────────────────────────────────────────────────────────

    private ElapsedTime loopTimer  = new ElapsedTime();
    private double      avgLoopMs  = 0;
    private int         loopCount  = 0;

    // ── Constructor ──────────────────────────────────────────────────────────

    public RobotServo(HardwareMap hardwareMap, Telemetry telemetry,
                      boolean isRedAlliance, TeleOpMode mode) {
        this.teleOpMode   = mode;
        this.isRedAlliance = isRedAlliance;

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        follower = Constants.createFollower(hardwareMap);
        follower.update();

        Localizer.getInstance(hardwareMap);

        vision = new Vision(hardwareMap, isRedAlliance);

        intake  = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooter.setFollower(follower);

        turretServo = new TurretServo(hardwareMap); // centers servo at 0° on construction

        // Consume the motor-turret handoff flag so it doesn't interfere if motor TeleOp
        // is run later in the same power cycle.
        Turret.consumeAutoHandoff();

        intakeController  = new IntakeController(null, intake);
        shooterController = new ShooterController(null, shooter);

        if (mode == TeleOpMode.EMERGENCY) autoAimEnabled = false;
    }

    // ── Goal setup ───────────────────────────────────────────────────────────

    public void setGoalPose(Pose goal) { this.goalPose = goal; }

    // ── Lifecycle ────────────────────────────────────────────────────────────

    public void start() {
        follower.startTeleopDrive();
        intake.off();
    }

    /** Called on first joystick input. Spins up shooter and fires first auto-aim command. */
    public void activateDriver() {
        if (driverReady) return;
        driverReady = true;

        if (teleOpMode == TeleOpMode.NORMAL || teleOpMode == TeleOpMode.NO_AUTO) {
            double aimTrim   = isRedAlliance ? AIM_TRIM_RED : AIM_TRIM_BLUE;
            double initAngle = calculateAimAngle() + autoAimTrim + aimTrim;
            turretServo.setTargetAngle(initAngle);
            manualAngle = turretServo.getTargetAngle();

            // Shooter spin-up at distance-appropriate velocity
            double tagX = FieldConstants.getTag(isRedAlliance).getX();
            double tagY = FieldConstants.getTag(isRedAlliance).getY();
            Pose   p    = follower.getPose();
            double dx   = tagX - p.getX(), dy = tagY - p.getY();
            double dist = Math.sqrt(dx * dx + dy * dy);
            if (dist > 0) { shooter.updateVelocity(dist); shooter.updateHood(dist); }
            else          { shooter.on(); }
        } else {
            shooter.on();
        }
    }

    public boolean isDriverReady() { return driverReady; }

    // ── Main loop ────────────────────────────────────────────────────────────

    public void update(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        if (loopCount > 0) avgLoopMs = avgLoopMs * 0.9 + loopMs * 0.1;
        loopCount++;

        for (LynxModule hub : allHubs) hub.clearBulkCache();

        follower.update();
        vision.update();

        if (!driverReady) {
            boolean driveInput = Math.abs(gamepad1.left_stick_x)  > 0.1
                              || Math.abs(gamepad1.left_stick_y)  > 0.1
                              || Math.abs(gamepad1.right_stick_x) > 0.1;
            if (driveInput) activateDriver();
        }

        if (!driverReady) {
            if (loopCount % 10 == 0)
                telemetry.addData("Loop", String.format("%.1fms", avgLoopMs));
            return;
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x / 1.25,
                true);

        // ── Distances ────────────────────────────────────────────────────────
        double tagX = FieldConstants.getTag(isRedAlliance).getX();
        double tagY = FieldConstants.getTag(isRedAlliance).getY();
        Pose   curPose = follower.getPose();
        double dx = tagX - curPose.getX(), dy = tagY - curPose.getY();
        double odometryDistance = Math.sqrt(dx * dx + dy * dy);

        // Spinning detection — prevent stale distance from thrashing shooter params
        double currentHeading = curPose.getHeading();
        boolean isSpinning = false;
        if (!Double.isNaN(prevHeading)) {
            double rawDelta     = Math.toDegrees(currentHeading - prevHeading);
            double headingDelta = Math.abs(rawDelta - Math.round(rawDelta / 360.0) * 360.0);
            isSpinning = headingDelta > SPIN_THRESHOLD_DEG_PER_FRAME;
        }
        prevHeading = currentHeading;

        double visionDistance = vision.hasTargetTag() ? vision.getTargetDistance() : 0;

        if (visionDistance > 0) {
            effectiveDistance = visionDistance;
            distanceSource    = "Vision";
        } else if (odometryDistance > 0) {
            effectiveDistance = odometryDistance;
            distanceSource    = "Odometry";
        } else {
            effectiveDistance = 0;
            distanceSource    = "No distance";
        }

        // ── Shooter velocity / hood ──────────────────────────────────────────
        if (!manualHoodMode) {
            if (effectiveDistance <= 0) {
                if (!shooter.isShooting()) {
                    shooter.setTargetVelocity(1250.0);
                    shooter.setHoodPosition(0.0);
                    distanceSource = "No distance (fallback)";
                } else {
                    distanceSource += " (hold last)";
                }
            } else {
                shooter.updateVelocity(effectiveDistance);
                shooter.updateHood(effectiveDistance);
            }
        } else {
            if (effectiveDistance > 0) {
                shooter.updateVelocity(effectiveDistance);
                distanceSource += " (manual hood)";
            } else {
                if (!shooter.isShooting()) {
                    shooter.setTargetVelocity(1050.0);
                    distanceSource = "No distance (manual hood)";
                } else {
                    distanceSource += " (shooting, manual hood)";
                }
            }
        }
        shooter.updatePID();

        // ── Turret servo ─────────────────────────────────────────────────────
        updateServoTurret(gamepad1);

        // ── Intake + shooter controllers ─────────────────────────────────────
        updateControllers(gamepad1, gamepad2);

        if (loopCount % 10 == 0) {
            telemetry.addData("Loop", String.format("%.1fms (%.0f Hz)",
                    avgLoopMs, avgLoopMs > 0 ? 1000.0 / avgLoopMs : 0));
            telemetry.addData("Spinning",    isSpinning ? "YES (dist frozen)" : "no");
            telemetry.addData("Dist source", distanceSource);
            telemetry.addData("Vision",      vision.hasTargetTag()
                    ? String.format("LOCK  ty=%.1f°  dist=%.1f\"",
                            vision.getTargetPitch(), visionDistance)
                    : (vision.isConnected() ? "searching..." : "DISCONNECTED"));
        }

        if (gamepad2.a && !prevFireButton) { /* unused */ }
        prevFireButton = gamepad2.a;
    }

    // ── Turret servo auto-aim / manual ───────────────────────────────────────

    /**
     * Called every loop. Gamepad1 dpad_left/right = manual angle trim (mirrors TurretController
     * behavior but in degrees instead of motor power). Auto-aim uses the same field→robot
     * coordinate transform as TurretAimer.calculateTargetAngle(), plus a vision EMA nudge.
     */
    private void updateServoTurret(Gamepad gamepad1) {
        boolean dpadLeft    = gamepad1.dpad_left;
        boolean dpadRight   = gamepad1.dpad_right;
        boolean manualActive = dpadLeft || dpadRight;

        if (manualActive) {
            if (!wasManualActive) {
                manualAngle = turretServo.getTargetAngle(); // latch current angle on first press
                dpadHoldTimer.reset();
                autoAimEnabled = false;
            }
            double degPerLoop = dpadHoldTimer.seconds() >= MANUAL_HOLD_SEC
                    ? MANUAL_DEG_FAST : MANUAL_DEG_SLOW;
            if (dpadLeft)  manualAngle -= degPerLoop;
            if (dpadRight) manualAngle += degPerLoop;
            turretServo.setTargetAngle(manualAngle);
            manualAngle = turretServo.getTargetAngle(); // keep clamped copy in sync

        } else if (wasManualActive) {
            // Dpad just released: store residual trim, re-enable auto-aim (except EMERGENCY)
            autoAimTrim = turretServo.getTargetAngle() - calculateAimAngle();
            if (teleOpMode != TeleOpMode.EMERGENCY) autoAimEnabled = true;
        }

        if (!manualActive && autoAimEnabled) {
            double aimTrim = isRedAlliance ? AIM_TRIM_RED : AIM_TRIM_BLUE;
            double angle   = calculateAimAngle() + autoAimTrim + aimTrim;

            // Camera ty → left/right angle (camera is 90° sideways)
            if (vision.hasTargetTag()) {
                visionSmoothedTy += VISION_TX_EMA * (vision.getTargetPitch() - visionSmoothedTy);
                angle += visionSmoothedTy;
            } else {
                visionSmoothedTy = 0.0;
            }

            turretServo.setTargetAngle(angle);
        }

        wasManualActive = manualActive;
    }

    /**
     * Field→robot coordinate transform to the goal. Same math as TurretAimer.calculateTargetAngle().
     * Saves angle at 5–9" and holds it below 5" (odometry unreliable at very close range).
     */
    private double calculateAimAngle() {
        if (goalPose == null) return 0.0;
        Pose p = follower.getPose();
        if (p == null) return 0.0;

        double fieldDX = goalPose.getX() - p.getX();
        double fieldDY = goalPose.getY() - p.getY();
        double distance = Math.sqrt(fieldDX * fieldDX + fieldDY * fieldDY);

        if (distance >= 5.0 && distance <= 9.0) {
            double h  = p.getHeading();
            double rX =  fieldDX * Math.cos(h) + fieldDY * Math.sin(h);
            double rY = -fieldDX * Math.sin(h) + fieldDY * Math.cos(h);
            double ang = -Math.toDegrees(Math.atan2(rY, rX));
            savedCloseRangeAngle = Math.max(TurretServo.MIN_ANGLE, Math.min(TurretServo.MAX_ANGLE, ang));
        }

        if (distance < 5.0) {
            return Double.isNaN(savedCloseRangeAngle)
                    ? turretServo.getTargetAngle()
                    : savedCloseRangeAngle;
        }

        double h  = p.getHeading();
        double rX =  fieldDX * Math.cos(h) + fieldDY * Math.sin(h);
        double rY = -fieldDX * Math.sin(h) + fieldDY * Math.cos(h);
        double angle = -Math.toDegrees(Math.atan2(rY, rX));
        return Math.max(TurretServo.MIN_ANGLE, Math.min(TurretServo.MAX_ANGLE, angle));
    }

    /** Re-enable auto-aim and clear all manual trim. Call after a position reset. */
    public void enableAutoAim() {
        autoAimEnabled   = true;
        autoAimTrim      = 0.0;
        visionSmoothedTy = 0.0;
        savedCloseRangeAngle = Double.NaN;
    }

    public boolean isAutoAimEnabled() { return autoAimEnabled; }

    // ── Controllers (intake + shooter) ───────────────────────────────────────

    private void updateControllers(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad2 == null) return;

        shooterController.gamepad  = gamepad2;
        shooterController.gamepad1 = gamepad1;
        shooterController.update(intake);

        intakeController.gamepad  = gamepad1;
        intakeController.gamepad1 = gamepad1;
        if (!shooterController.isShooting()) intakeController.update();

        // GP2 Options: emergency reset — shooter FSM + turret center + re-enable auto-aim
        if (gamepad2.options) {
            intake.off();
            shooter.reset();
            turretServo.returnToCenter();
            enableAutoAim();
            manualHoodMode = false;
            shooter.resetDeadzones();
        }
    }

    // ── Stop ─────────────────────────────────────────────────────────────────

    public void stop() {
        intake.off();
        shooter.off();
        vision.stop();
    }
}
