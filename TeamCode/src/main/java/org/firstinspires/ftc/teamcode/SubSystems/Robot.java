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
import org.firstinspires.ftc.teamcode.Controllers.TurretController;
import org.firstinspires.ftc.teamcode.Controllers.ResetController;
import org.firstinspires.ftc.teamcode.OpModes.TeleOpMode;

public class Robot {
    private List<LynxModule> allHubs;

    public Follower follower;
    public Vision vision;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;

    public IntakeController intakeController;
    public ShooterController shooterController;
    public TurretController turretController;
    public ResetController resetController;

    private boolean prevFireButton = false;

    public String distanceSource = "N/A";
    public double effectiveDistance = 0;

    private TeleOpMode teleOpMode;

    public boolean manualHoodMode = false;

    private boolean driverReady = false;

    // Spinning detection — freeze physics when robot rotates to prevent odometry drift
    private static final double SPIN_THRESHOLD_DEG_PER_FRAME = 0.5;
    private double prevHeading = Double.NaN;

    private boolean isRedAlliance;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean isRedAlliance, TeleOpMode mode) {
        this.teleOpMode = mode;
        this.isRedAlliance = isRedAlliance;
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.update();

        Localizer.getInstance(hardwareMap);

        vision = new Vision(hardwareMap, isRedAlliance);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooter.setFollower(follower); // for the in-zone feed-power check
        turret = new Turret(hardwareMap, follower, vision);
        // Auto→TeleOp turret handoff: keep Auto's final turret position exactly once.
        // Any other start (fresh power-on, standalone or repeat TeleOp) → current physical becomes 0°.
        if (!Turret.consumeAutoHandoff()) {
            turret.resetEncoder();
        }
        Pose goal = FieldConstants.getGoal(isRedAlliance);
        Pose tag = FieldConstants.getTag(isRedAlliance);
        turret.setGoalPose(goal);
        turret.setTagPose(tag.getX(), tag.getY());
        turret.setAimTrimAlliance(isRedAlliance); // red/blue use different aim trims

        intakeController = new IntakeController(null, intake);
        shooterController = new ShooterController(null, shooter);
        turretController = new TurretController(null, turret);
        resetController = new ResetController(intakeController, shooterController, turretController, intake, shooter, turret, follower);

        if (mode == TeleOpMode.EMERGENCY) {
            turretController.disableAutoAim();
        }
    }

    public void start() {
        follower.startTeleopDrive();
        intake.off();
    }

    /**
     * Call when driver is ready (first joystick input).
     * Starts turret auto-aim and flywheel spin-up.
     */
    public void activateDriver() {
        if (driverReady) return;
        driverReady = true;

        if (teleOpMode == TeleOpMode.NORMAL || teleOpMode == TeleOpMode.NO_AUTO) {
            turret.autoAim();

            double tagX = FieldConstants.getTag(isRedAlliance).getX();
            double tagY = FieldConstants.getTag(isRedAlliance).getY();
            Pose startPos = follower.getPose();
            double dtx = tagX - startPos.getX();
            double dty = tagY - startPos.getY();
            double distToTag = Math.sqrt(dtx * dtx + dty * dty);
            if (distToTag > 0) {
                shooter.updateVelocity(distToTag);
                shooter.updateHood(distToTag);
            } else {
                shooter.on();
            }
        } else {
            shooter.on();
        }
    }

    public boolean isDriverReady() {
        return driverReady;
    }

    private ElapsedTime loopTimer = new ElapsedTime();
    private double avgLoopMs = 0;
    private int loopCount = 0;

    public void update(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        if (loopCount > 0) {
            avgLoopMs = avgLoopMs * 0.9 + loopMs * 0.1;
        }
        loopCount++;

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        follower.update();
        vision.update();

        // Activate everything the moment driver touches any drive stick on gamepad1.
        // Before this: robot holds position silently (no driving, no turret, no shooter).
        // This ensures the robot starts from the exact auto end-position.
        if (!driverReady) {
            boolean driveInput = Math.abs(gamepad1.left_stick_x)  > 0.1
                              || Math.abs(gamepad1.left_stick_y)  > 0.1
                              || Math.abs(gamepad1.right_stick_x) > 0.1;
            if (driveInput) activateDriver();
        }

        if (!driverReady) {
            if (loopCount % 10 == 0) {
                telemetry.addData("Loop", String.format("%.1fms (%.0f Hz)", avgLoopMs, avgLoopMs > 0 ? 1000.0 / avgLoopMs : 0));
            }
            return;
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x / 1.25,
                true
        );

        // Distance to TAG — for velocity/hood (formulas calibrated from tag, not goal)
        double tagX = FieldConstants.getTag(isRedAlliance).getX();
        double tagY = FieldConstants.getTag(isRedAlliance).getY();
        Pose curPose = follower.getPose();
        double dtx = tagX - curPose.getX();
        double dty = tagY - curPose.getY();
        double odometryDistance = Math.sqrt(dtx * dtx + dty * dty);

        // Spinning detection — prevent physics override when robot rotates in place
        double currentHeading = curPose.getHeading();
        boolean isSpinning = false;
        if (!Double.isNaN(prevHeading)) {
            double rawDelta = Math.toDegrees(currentHeading - prevHeading);
            double headingDeltaDeg = Math.abs(rawDelta - Math.round(rawDelta / 360.0) * 360.0);
            isSpinning = headingDeltaDeg > SPIN_THRESHOLD_DEG_PER_FRAME;
        }
        prevHeading = currentHeading;

        double visionDistance = vision.hasTargetTag() ? vision.getTargetDistance() : 0;

        double distanceToGoal;
        if (visionDistance > 0) {
            distanceToGoal = visionDistance;
            distanceSource = "Vision";
        } else if (odometryDistance > 0) {
            distanceToGoal = odometryDistance;
            distanceSource = "Odometry";
        } else {
            distanceToGoal = 0;
            distanceSource = "No distance";
        }

        // Physics virtual distance — always active (at vel=0 equals normal tag distance)
        double effectiveDist = distanceToGoal;
        if (!isSpinning && turret.hasPhysicsShot() && turret.getPhysicsVirtualDistanceInches() > 0) {
            effectiveDist = turret.getPhysicsVirtualDistanceInches();
        }
        effectiveDistance = effectiveDist;

        if (!manualHoodMode) {
            if (effectiveDist <= 0) {
                if (!shooter.isShooting()) {
                    shooter.setTargetVelocity(1250.0);
                    shooter.setHoodPosition(0.0);
                    distanceSource = "No distance (fallback)";
                } else {
                    distanceSource += " (hold last)";
                }
            } else {
                shooter.updateVelocity(effectiveDist);
                shooter.updateHood(effectiveDist);
            }
        } else {
            if (effectiveDist > 0) {
                shooter.updateVelocity(effectiveDist);
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

        updateControllers(gamepad1, gamepad2);

        if (loopCount % 10 == 0) {
            telemetry.addData("Loop", String.format("%.1fms (%.0f Hz)", avgLoopMs, avgLoopMs > 0 ? 1000.0 / avgLoopMs : 0));
            telemetry.addData("Spinning", isSpinning ? "YES (dist frozen)" : "no");
            telemetry.addData("Effective dist", String.format("%.1f\"", effectiveDist));
            telemetry.addData("Dist source", distanceSource);
            telemetry.addData("Vision", turret.hasVisionTarget()
                    ? String.format("LOCK  tx=%.1f°  dist=%.1f\"", vision.getTargetYaw(), visionDistance)
                    : (vision.isConnected() ? "searching..." : "DISCONNECTED"));
            if (turret.hasPhysicsShot()) {
                telemetry.addData("Physics", "ACTIVE");
                telemetry.addData("Turret correction", String.format("%.1f°",
                        turret.getCalculatedTurretAngleDeg() - turret.getTargetAngle()));
            }
        }

        handleFireButton(gamepad2, telemetry);
    }

    private void updateControllers(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad2 == null) return;
        shooterController.gamepad = gamepad2;
        shooterController.gamepad1 = gamepad1;
        shooterController.update(intake);

        boolean prevAutoAim = turretController.autoAimEnabled;
        turretController.gamepad = gamepad2;
        turretController.gamepad1 = gamepad1;
        turretController.update();

        if (!prevAutoAim && turretController.autoAimEnabled) {
            shooter.resetDeadzones();
        }

        intakeController.gamepad = gamepad1;
        intakeController.gamepad1 = gamepad1;
        if (!shooterController.isShooting()) {
            intakeController.update();
        }

        resetController.handleResetButton(gamepad2);

        if (gamepad2.options) {
            manualHoodMode = false;
            shooter.resetDeadzones();
        }
    }

    private void handleFireButton(Gamepad gamepad2, Telemetry telemetry) {
        if (gamepad2.a && !prevFireButton) {
//            fireBalls(telemetry);
        }
        prevFireButton = gamepad2.a;
    }

    public void stop() {
        intake.off();
        shooter.off();
        turret.stop();
        vision.stop();
    }
}
