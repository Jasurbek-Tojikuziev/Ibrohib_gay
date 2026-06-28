package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.FieldConstants;
import org.firstinspires.ftc.teamcode.SubSystems.Localizer;
import org.firstinspires.ftc.teamcode.SubSystems.RobotServo;

@TeleOp(name = "RED Alliance TeleOp SERVO", group = "TeleOp")
public class RedAllianceTeleOpServo extends LinearOpMode {

    private RobotServo robot;

    private TeleOpMode selectedMode    = TeleOpMode.NORMAL;
    private boolean    modeConfirmed   = false;
    private boolean    prevRightBumper = false;

    private final ElapsedTime resetDebounceTimer = new ElapsedTime();
    private static final double RESET_DEBOUNCE_SEC = 0.5;

    @Override
    public void runOpMode() {
        final boolean isRedAlliance = true;

        // ── Mode selection (init phase) ──────────────────────────────────────
        while (!isStarted() && !isStopRequested()) {
            if      (gamepad1.dpad_up)   selectedMode = TeleOpMode.NORMAL;
            else if (gamepad1.dpad_left) selectedMode = TeleOpMode.NO_AUTO;
            else if (gamepad1.dpad_down) selectedMode = TeleOpMode.EMERGENCY;

            if (gamepad1.right_bumper && !prevRightBumper) modeConfirmed = true;
            prevRightBumper = gamepad1.right_bumper;

            telemetry.addLine("=== RED ALLIANCE TELEOP — SERVO TURRET ===");
            telemetry.addLine();
            telemetry.addData("Dpad Up",   selectedMode == TeleOpMode.NORMAL    ? ">>> NORMAL <<<"    : "NORMAL");
            telemetry.addData("Dpad Left",  selectedMode == TeleOpMode.NO_AUTO   ? ">>> NO AUTO <<<"   : "NO AUTO");
            telemetry.addData("Dpad Down",  selectedMode == TeleOpMode.EMERGENCY ? ">>> EMERGENCY <<<" : "EMERGENCY");
            telemetry.addLine();
            telemetry.addData("Status", modeConfirmed ? "CONFIRMED" : "Press START to begin");
            telemetry.update();
        }

        waitForStart();

        // ── Robot init ───────────────────────────────────────────────────────
        robot = new RobotServo(hardwareMap, telemetry, isRedAlliance, selectedMode);

        // Auto→TeleOp pose handoff (one-time, same logic as RedAllianceTeleOp)
        Localizer localizer  = Localizer.getInstance();
        boolean usedAutoPose = Localizer.consumeAutoPoseHandoff();
        double lastX         = localizer.getX();
        double lastY         = localizer.getY();
        double lastHeading   = localizer.getHeading();

        Pose startPose;
        if (usedAutoPose && (Math.abs(lastX) > 1.0 || Math.abs(lastY) > 1.0)) {
            startPose = new Pose(lastX, lastY, Math.toRadians(lastHeading));
            telemetry.addLine("Using position from Auto");
        } else if (selectedMode == TeleOpMode.NO_AUTO) {
            startPose = new Pose(118, 129, Math.toRadians(45));
            telemetry.addLine("Using NO AUTO start position");
        } else {
            startPose = new Pose(128.9090909090909, 80.36363636363635, Math.toRadians(0));
            telemetry.addLine("Using default Red start position");
        }

        robot.follower.setStartingPose(startPose);
        robot.follower.update();
        Pose syncPose = robot.follower.getPose();
        localizer.setPosition(syncPose.getX(), syncPose.getY(),
                Math.toDegrees(syncPose.getHeading()));

        Pose redGoalPose = FieldConstants.RED_GOAL;
        robot.setGoalPose(redGoalPose);

        robot.start();

        // ── Main loop ────────────────────────────────────────────────────────
        while (opModeIsActive()) {

            boolean didReset = false;

            // Dpad UP — far-zone position reset (same coordinates as RedAllianceTeleOp)
            if (gamepad1.dpad_up && resetDebounceTimer.seconds() >= RESET_DEBOUNCE_SEC) {
                if (!robot.isDriverReady()) robot.activateDriver();
                Pose resetPose = new Pose(11.598, 10.885, Math.toRadians(0));
                robot.follower.setPose(resetPose);
                localizer.setPosition(resetPose.getX(), resetPose.getY(),
                        Math.toDegrees(resetPose.getHeading()));
                robot.enableAutoAim();
                didReset = true;
                resetDebounceTimer.reset();
            }

            // Dpad DOWN — close-zone position reset
            if (gamepad1.dpad_down && resetDebounceTimer.seconds() >= RESET_DEBOUNCE_SEC) {
                if (!robot.isDriverReady()) robot.activateDriver();
                Pose resetPose = new Pose(128.9090909090909, 80.36363636363635, Math.toRadians(0));
                robot.follower.setPose(resetPose);
                localizer.setPosition(resetPose.getX(), resetPose.getY(),
                        Math.toDegrees(resetPose.getHeading()));
                robot.enableAutoAim();
                didReset = true;
                resetDebounceTimer.reset();
            }

            robot.update(gamepad1, gamepad2, telemetry);

            // Re-assert goal after reset (update() may overwrite internal state)
            if (didReset) robot.setGoalPose(redGoalPose);

            displayTelemetry();
            telemetry.update();
        }

        robot.stop();
    }

    private void displayTelemetry() {
        if (!robot.isDriverReady()) {
            telemetry.addLine(">>> Touch GP1 left stick to activate <<<");
            telemetry.addLine();
        }

        telemetry.addLine("=== MODE ===");
        telemetry.addData("TeleOp Mode", selectedMode);
        if (selectedMode == TeleOpMode.EMERGENCY)
            telemetry.addLine("EMERGENCY: manual turret only (GP1 dpad L/R)");
        telemetry.addLine();

        Pose currentPose = robot.follower.getPose();
        telemetry.addLine("=== ODOMETRY (RED) ===");
        telemetry.addData("Robot X",  "%.2f in", currentPose.getX());
        telemetry.addData("Robot Y",  "%.2f in", currentPose.getY());
        telemetry.addData("Heading",  "%.1f°",   Math.toDegrees(currentPose.getHeading()));

        telemetry.addLine();
        telemetry.addLine("=== TURRET SERVO ===");
        telemetry.addData("Target Angle",   "%.1f°", robot.turretServo.getTargetAngle());
        telemetry.addData("Servo Position", "%.3f",  robot.turretServo.getCommandedPosition());
        telemetry.addData("Auto-aim",       robot.isAutoAimEnabled() ? "ON" : "OFF (manual)");
        telemetry.addData("Tag seen",       robot.vision.hasTargetTag() ? "YES — LOCK" : "no");
        telemetry.addData("Cam raw",        robot.vision.getRawTagDebug());

        telemetry.addLine();
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Effective Dist",   "%.1f in",     robot.effectiveDistance);
        telemetry.addData("Distance Source",  robot.distanceSource);
        telemetry.addData("Hood Position",    "%.2f",        robot.shooter.getHoodServoPosition());
        telemetry.addData("Target Velocity",  "%.0f tick/s", robot.shooter.getTargetVelocity());
        telemetry.addData("Current Velocity", "%.0f tick/s", robot.shooter.getCurrentVelocity());
    }
}
