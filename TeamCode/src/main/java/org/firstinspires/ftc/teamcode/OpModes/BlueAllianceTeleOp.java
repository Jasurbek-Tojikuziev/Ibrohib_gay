package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.SubSystems.FieldConstants;
import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="BLUE Alliance TeleOp", group="TeleOp")
public class BlueAllianceTeleOp extends LinearOpMode {

    private Robot robot;

    // Mode selection
    private TeleOpMode selectedMode = TeleOpMode.NORMAL;  // Default
    private boolean modeConfirmed = false;
    private boolean prevRightBumper = false;

    // Position reset debounce (non-blocking замена sleep(500))
    private ElapsedTime resetDebounceTimer = new ElapsedTime();
    private static final double RESET_DEBOUNCE_SEC = 0.5;


    @Override
    public void runOpMode() {
        // BLUE Alliance
        boolean isRedAlliance = false;

        // Mode selection during init phase
        while (!isStarted() && !isStopRequested()) {
            // Allow mode switching with dpad (always, even after confirmation)
            if (gamepad1.dpad_up) {
                selectedMode = TeleOpMode.NORMAL;
            } else if (gamepad1.dpad_left) {
                selectedMode = TeleOpMode.NO_AUTO;
            } else if (gamepad1.dpad_down) {
                selectedMode = TeleOpMode.EMERGENCY;
            }

            // Confirm selection with right bumper
            if (gamepad1.right_bumper && !prevRightBumper) {
                modeConfirmed = true;
            }
            prevRightBumper = gamepad1.right_bumper;

            // Display
            telemetry.addLine("=== BLUE ALLIANCE TELEOP ===");
            telemetry.addLine();
            telemetry.addLine("=== SELECT MODE ===");
            telemetry.addData("Dpad Up", selectedMode == TeleOpMode.NORMAL ? ">>> NORMAL <<<" : "NORMAL");
            telemetry.addData("Dpad Left", selectedMode == TeleOpMode.NO_AUTO ? ">>> NO AUTO <<<" : "NO AUTO");
            telemetry.addData("Dpad Down", selectedMode == TeleOpMode.EMERGENCY ? ">>> EMERGENCY <<<" : "EMERGENCY");
            telemetry.addLine();

            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addData("Status", modeConfirmed ? "CONFIRMED ✓" : "Press START to begin");
            telemetry.addLine();
            telemetry.update();
        }

        waitForStart();

        // Initialize robot with selected mode
        robot = new Robot(hardwareMap, telemetry, isRedAlliance, selectedMode);

        // Check if Auto set a position - if so, use it. Otherwise use default.
        org.firstinspires.ftc.teamcode.SubSystems.Localizer localizer =
            org.firstinspires.ftc.teamcode.SubSystems.Localizer.getInstance();

        // Consume the one-time Auto→TeleOp pose handoff (mirrors the turret handoff):
        // Auto's final pose is used exactly once; any later run falls back to a default start.
        boolean usedAutoPose = org.firstinspires.ftc.teamcode.SubSystems.Localizer.consumeAutoPoseHandoff();

        double lastX = localizer.getX();
        double lastY = localizer.getY();
        double lastHeading = localizer.getHeading();

        Pose startPose;
        if (usedAutoPose && (Math.abs(lastX) > 1.0 || Math.abs(lastY) > 1.0)) {
            // Auto ran and set position - use it
            startPose = new Pose(lastX, lastY, Math.toRadians(lastHeading));
            telemetry.addLine("✓ Using position from Auto");
            telemetry.addData("Auto End Pos", "(%.1f, %.1f, %.0f°)", lastX, lastY, lastHeading);
        } else if (selectedMode == TeleOpMode.NO_AUTO) {
            // NO_AUTO mode - fixed start position
            startPose = new Pose(26, 129, Math.toRadians(135));
            telemetry.addLine("Using NO AUTO start position");
        } else {
            // No Auto ran - use default Blue start
            startPose = new Pose(14, 78, Math.toRadians(180));
            telemetry.addLine("Using default Blue start position");
        }

        robot.follower.setStartingPose(startPose);
        robot.follower.update();

        // Synchronize Localizer with Follower (in case we used default)
        Pose syncPose = robot.follower.getPose();
        localizer.setPosition(
            syncPose.getX(),
            syncPose.getY(),
            Math.toDegrees(syncPose.getHeading())
        );


        // Set goal (basket) for turret auto-aim
        Pose blueGoalPose = FieldConstants.BLUE_GOAL;
        robot.turret.setGoalPose(blueGoalPose);

        robot.start();

        while (opModeIsActive()) {

            // Position reset + activation on dpad (gamepad1)
            boolean didReset = false;
            if (gamepad1.dpad_up && resetDebounceTimer.seconds() >= RESET_DEBOUNCE_SEC) {
                if (!robot.isDriverReady()) robot.activateDriver();
                // Reset to Blue alliance preset position (far side)
                Pose resetPose = new Pose(135.8758815232722, 8.124118476727789, Math.toRadians(180));
                robot.follower.setPose(resetPose);
                org.firstinspires.ftc.teamcode.SubSystems.Localizer.getInstance().setPosition(
                    resetPose.getX(),
                    resetPose.getY(),
                    Math.toDegrees(resetPose.getHeading())
                );
                robot.turret.resetEncoder();         // current physical turret position becomes 0° (driver centered it)
                robot.turret.setAutoAimOffset(0.0);  // clear manual offset — position is now known
                robot.turret.clearRelocalization();  // position reset → drop stale camera correction
                robot.turretController.enableAutoAim();
                robot.turret.autoAim();              // drive to calculated target for this reset pose
                didReset = true;
                resetDebounceTimer.reset();
            }

            if (gamepad1.dpad_down && resetDebounceTimer.seconds() >= RESET_DEBOUNCE_SEC) {
                if (!robot.isDriverReady()) robot.activateDriver();
                Pose resetPose = new Pose(14, 78, Math.toRadians(180));
                robot.follower.setPose(resetPose);
                org.firstinspires.ftc.teamcode.SubSystems.Localizer.getInstance().setPosition(
                    resetPose.getX(), resetPose.getY(), Math.toDegrees(resetPose.getHeading()));
                robot.turret.resetEncoder();         // current physical turret position becomes 0° (driver centered it)
                robot.turret.setAutoAimOffset(0.0);  // clear manual offset — position is now known
                robot.turret.clearRelocalization();  // position reset → drop stale camera correction
                robot.turretController.enableAutoAim();
                robot.turret.autoAim();              // drive to calculated target for this reset pose
                didReset = true;
                resetDebounceTimer.reset();
            }

            // Обновление робота
            robot.update(gamepad1, gamepad2, telemetry);

            // После robot.update() resetEncoder() мог очистить goalPose — восстанавливаем
            if (didReset) {
                robot.turret.setGoalPose(blueGoalPose);
            }

            // Телеметрия
            displayTelemetry();

            telemetry.update();
        }

        robot.stop();
    }

    private void displayTelemetry() {
        if (!robot.isDriverReady()) {
            telemetry.addLine(">>> Touch GP1 left stick to activate from auto position <<<");
            telemetry.addLine();
        }

        // Mode indicator at top
        telemetry.addLine("=== MODE ===");
        telemetry.addData("TeleOp Mode", selectedMode);
        if (selectedMode == TeleOpMode.EMERGENCY) {
            telemetry.addLine("⚠️ EMERGENCY: Turret manual only");
        }
        telemetry.addLine();

        // Odometry
        Pose currentPose = robot.follower.getPose();
        telemetry.addLine("=== ODOMETRY (BLUE) ===");
        telemetry.addData("Robot X", "%.2f in", currentPose.getX());
        telemetry.addData("Robot Y", "%.2f in", currentPose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));

        // Turret
        telemetry.addLine();
        telemetry.addLine("=== TURRET ===");
        telemetry.addData("Current Angle", "%.1f°", robot.turret.getCurrentAngle());
        telemetry.addData("Target Angle", "%.1f°", robot.turret.getTargetAngle());

        // Vision / aim status
        telemetry.addLine();
        telemetry.addLine("=== VISION / AIM ===");
        telemetry.addData("Tag seen", robot.turret.hasVisionTarget() ? "YES — LOCK" : "no");
        telemetry.addData("Cam raw", robot.vision.getRawTagDebug());
        telemetry.addData("Auto-aim", robot.turretController.isAutoAimEnabled() ? "ON (aiming)" : "OFF (manual)");
        telemetry.addData("Reloc", robot.turret.didReloc() ? "*** correcting ***"
                : (robot.turret.isSettledForReloc() ? "settled" : "moving"));
        telemetry.addData("Reloc offset", "%.1f°", robot.turret.getRelocOffset());

        // Shooter
        telemetry.addLine();
        telemetry.addLine("=== SHOOTER ===");

        // Distances
        double odometryDist = robot.turret.getDistanceToGoal();
        telemetry.addData("Odometry Distance", "%.1f in", odometryDist);
        telemetry.addData("Effective Dist (vel/hood)", "%.1f in", robot.effectiveDistance);
        telemetry.addData("Distance Source", robot.distanceSource);
//        telemetry.addData("Is Shooting", robot.shooterController.isShooting() ? "YES" : "NO");
//        telemetry.addData("Sample Count", "%d / 3", robot.shooter.getSampleCount());
        telemetry.addData("Hood Servo Position", "%.2f", robot.shooter.getHoodServoPosition());
        telemetry.addData("Target Velocity", "%.0f ticks/sec", robot.shooter.getTargetVelocity());
        telemetry.addData("Current Velocity", "%.0f ticks/sec", robot.shooter.getCurrentVelocity());

        // Controls
//        telemetry.addLine();
//        telemetry.addLine("=== CONTROLS ===");
//        telemetry.addData("GP1 Dpad Left", "TURRET CALIBRATION (rotate left, release=reset encoder)");
//        telemetry.addData("GP1 Dpad Right", "TURRET CALIBRATION (rotate right, release=reset encoder)");
//        telemetry.addData("GP1 Dpad Down", "RESET POSITION (104, 135)");
//        telemetry.addData("GP2 Right Bumper", "Start Shoot");
//        telemetry.addData("GP2 Dpad Up", "Manual Open ShooterStop");
//        telemetry.addData("GP2 Dpad Down", "Manual Close ShooterStop");
//        telemetry.addData("GP2 Right Stick X", "Manual Turret (holds position)");
//        telemetry.addData("GP2 Left Bumper", "Re-enable Auto-Aim");
//        telemetry.addData("GP2 Options", "RESET ALL");
    }
}
