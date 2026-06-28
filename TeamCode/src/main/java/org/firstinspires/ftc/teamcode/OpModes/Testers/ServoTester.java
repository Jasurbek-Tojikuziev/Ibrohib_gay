package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="[TEST] Servo Tester", group="Testers")
public class ServoTester extends LinearOpMode {

    private Servo shooterHood;
    private Servo shooterStop;

    private double stopPos  = 0.55; // start closed
    private boolean prevUp, prevDown, prevLeft, prevRight, prevX, prevY;

    @Override
    public void runOpMode() {
        shooterHood = hardwareMap.get(Servo.class, "shooterHood");
        shooterStop = hardwareMap.get(Servo.class, "shooterStop");

        shooterStop.setPosition(stopPos);

        telemetry.addLine("Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ── Hood (dpad up / down) ────────────────────────────────────────
            boolean up   = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            if (up   && !prevUp)   shooterHood.setPosition(1.0);
            if (down && !prevDown) shooterHood.setPosition(0.0);

            // ── ShooterStop presets (X = closed, Y = open) ──────────────────
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;
            if (x && !prevX) { stopPos = 0.55;  shooterStop.setPosition(stopPos); }
            if (y && !prevY) { stopPos = 0.35;  shooterStop.setPosition(stopPos); }

            // ── ShooterStop step (dpad left = -0.1, dpad right = +0.1) ─────
            boolean left  = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;
            if (left  && !prevLeft)  { stopPos = Math.max(0.0, stopPos - 0.1); shooterStop.setPosition(stopPos); }
            if (right && !prevRight) { stopPos = Math.min(1.0, stopPos + 0.1); shooterStop.setPosition(stopPos); }

            prevUp = up; prevDown = down; prevLeft = left; prevRight = right; prevX = x; prevY = y;

            telemetry.addLine("=== SERVO TESTER ===");
            telemetry.addLine();
            telemetry.addLine("--- Hood ---");
            telemetry.addLine("  Dpad UP   = 1.0 (open)");
            telemetry.addLine("  Dpad DOWN = 0.0 (closed)");
            telemetry.addData("  Hood pos", "%.3f", shooterHood.getPosition());
            telemetry.addLine();
            telemetry.addLine("--- ShooterStop ---");
            telemetry.addLine("  X  = 0.55 (closed)");
            telemetry.addLine("  Y  = 0.35 (shooting/open)");
            telemetry.addLine("  Dpad LEFT = -0.1   Dpad RIGHT = +0.1");
            telemetry.addData("  Stop pos", "%.3f", stopPos);
            telemetry.update();
        }
    }
}
