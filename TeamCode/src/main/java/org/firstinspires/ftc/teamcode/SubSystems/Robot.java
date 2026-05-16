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

public class
Robot {
    // Bulk reads — all I2C reads per loop cached in one call
    private List<LynxModule> allHubs;

    // SubSystems
    public Follower follower;
    public DriveTrain driveTrain;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;

    // Controllers
    public IntakeController intakeController;
    public ShooterController shooterController;
    public TurretController turretController;
    public ResetController resetController;

    // Fire button state
    private boolean prevFireButton = false;

    // Distance source для debug телеметрии
    public String distanceSource = "N/A";
    // Actual distance used for velocity/hood (distance to TAG, not GOAL)
    public double effectiveDistance = 0;

    // TeleOp mode (NORMAL or EMERGENCY)
    private TeleOpMode teleOpMode;

    // Manual hood control flag
    public boolean manualHoodMode = false;

    // Driver ready flag — моторы не запускаются пока водитель не тронет джойстик
    private boolean driverReady = false;

    // Hood jitter prevention — единственная deadzone теперь в Shooter.java (3cm ≈ 1.18in)

    private boolean isRedAlliance;

    // Spinning detection — freeze distanceToGoal when robot spins in place to prevent odometry drift
    // GoBilda Pinpoint pods are not at center of rotation → spinning causes fake X/Y drift
    private static final double SPIN_THRESHOLD_DEG_PER_FRAME = 0.5; // degrees/frame (~15°/s at 30Hz) — catches slow turns too
    private double stableDistance = 0;
    private double prevHeading = Double.NaN;

    // Field coordinates — single source of truth in FieldConstants.java

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean isRedAlliance, TeleOpMode mode) {
        this.teleOpMode = mode;
        this.isRedAlliance = isRedAlliance;
        // Bulk reads — one I2C read per hub per loop instead of per-device
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Pedro Pathing Follower (одометрия)
        follower = Constants.createFollower(hardwareMap);
        follower.update(); // CRITICAL: Initialize before setting pose

        // Localizer singleton (нужен для relocalization)
        Localizer.getInstance(hardwareMap);

        // SubSystems
        driveTrain = new DriveTrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap, follower);
        Pose goal = FieldConstants.getGoal(isRedAlliance);
        Pose tag = FieldConstants.getTag(isRedAlliance);
        turret.setGoalPose(goal);
        turret.setTagPose(tag.getX(), tag.getY());

        // Controllers (на gamepad2)
        intakeController = new IntakeController(null, intake); // gamepad передадим в update
        shooterController = new ShooterController(null, shooter);
        turretController = new TurretController(null, turret);
        resetController = new ResetController(intakeController, shooterController, turretController, intake, shooter, turret);

        // Set initial auto-aim state based on mode
        if (mode == TeleOpMode.EMERGENCY) {
            turretController.disableAutoAim();
        }
        // NORMAL mode keeps default autoAimEnabled=true
    }

    public void start() {
        // Начальная настройка - убедимся что intake выключен
        intake.off();
        // Турель и flywheel НЕ запускаются — ждём activateDriver()
    }

    /**
     * Вызвать когда водитель готов (первый ввод с джойстика).
     * Запускает турель auto-aim и flywheel.
     */
    public void activateDriver() {
        if (driverReady) return; // уже активирован
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

    // Loop timing
    private ElapsedTime loopTimer = new ElapsedTime();
    private double avgLoopMs = 0;
    private int loopCount = 0;

    public void update(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        if (loopCount > 0) {
            avgLoopMs = avgLoopMs * 0.9 + loopMs * 0.1; // EMA
        }
        loopCount++;

        // Clear bulk cache once per loop — all subsequent I2C reads use cached data
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        follower.update();

        driveTrain.drive(gamepad1, gamepad2, telemetry);

        // Distance to TAG — для velocity/hood (формулы калиброваны от тега)
        double tagX = FieldConstants.getTag(isRedAlliance).getX();
        double tagY = FieldConstants.getTag(isRedAlliance).getY();
        Pose curPose = follower.getPose();
        double dtx = tagX - curPose.getX();
        double dty = tagY - curPose.getY();
        double odometryDistance = Math.sqrt(dtx * dtx + dty * dty);

        // Spinning detection — freeze distance when robot rotates in place to prevent odometry drift
        double currentHeading = curPose.getHeading();
        boolean isSpinning = false;
        if (!Double.isNaN(prevHeading)) {
            // Normalize heading delta to [-180, 180] degrees
            double rawDelta = Math.toDegrees(currentHeading - prevHeading);
            double headingDeltaDeg = Math.abs(rawDelta - Math.round(rawDelta / 360.0) * 360.0);
            isSpinning = headingDeltaDeg > SPIN_THRESHOLD_DEG_PER_FRAME;
        }
        prevHeading = currentHeading;

        double distanceToGoal = odometryDistance;
        if (odometryDistance > 0) {
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
        effectiveDistance = effectiveDist; // Expose for telemetry

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
            // Manual hood mode - only update velocity, not hood
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

        // Автоматическая регулировка Hood и Turret происходит внутри контроллеров
        updateControllers(gamepad1, gamepad2);

        // Telemetry — каждый 3-й loop чтобы не тормозить
        if (loopCount % 10 == 0) {
            telemetry.addData("Loop", String.format("%.1fms (%.0f Hz)", avgLoopMs, avgLoopMs > 0 ? 1000.0 / avgLoopMs : 0));
            telemetry.addData("Spinning", isSpinning ? "YES (dist frozen)" : "no");
            telemetry.addData("Effective dist", String.format("%.1f\"", effectiveDist));
            if (turret.hasPhysicsShot()) {
                telemetry.addData("Physics", "ACTIVE");
                telemetry.addData("Turret correction", String.format("%.1f°",
                        turret.getCalculatedTurretAngleDeg() - turret.getTargetAngle()));
            }
        }

        // Fire button
        handleFireButton(gamepad2, telemetry);
    }

private void updateControllers(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad2 == null) return;
        shooterController.gamepad = gamepad2;
        shooterController.update(intake);

        boolean prevAutoAim = turretController.autoAimEnabled;
        turretController.gamepad = gamepad2;
        turretController.gamepad1 = gamepad1; // Для dpad calibration
        turretController.update();

        // Если auto-aim только что включился — сбрасываем deadzones чтобы hood/velocity обновились сразу
        if (!prevAutoAim && turretController.autoAimEnabled) {
            shooter.resetDeadzones();
        }


        // IntakeController управляет intake только если Shooter НЕ активен
        intakeController.gamepad = gamepad2;
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
    }
}
