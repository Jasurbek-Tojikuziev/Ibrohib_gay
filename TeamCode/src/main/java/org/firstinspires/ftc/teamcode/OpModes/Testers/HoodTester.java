package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Hood Tester", group = "Testers")
public class HoodTester extends LinearOpMode {

    private static final double STEP     = 0.01;
    private static final double MIN_POS  = 0.0;
    private static final double MAX_POS  = 0.6;

    @Override
    public void runOpMode() {
        Servo hood = hardwareMap.get(Servo.class, "shooterHood");

        double position = 0.0;
        hood.setPosition(position);

        telemetry.addLine("Hood Tester — Ready");
        telemetry.addData("Dpad Up", "increase 0.01");
        telemetry.addData("Dpad Down", "decrease 0.01");
        telemetry.update();

        waitForStart();

        boolean prevUp   = false;
        boolean prevDown = false;

        while (opModeIsActive()) {
            boolean upPressed   = gamepad1.dpad_up   && !prevUp;
            boolean downPressed = gamepad1.dpad_down && !prevDown;
            prevUp   = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;

            if (upPressed) {
                position = Math.min(MAX_POS, position + STEP);
                hood.setPosition(position);
            }
            if (downPressed) {
                position = Math.max(MIN_POS, position - STEP);
                hood.setPosition(position);
            }

            telemetry.addLine("=== HOOD TESTER ===");
            telemetry.addData("Position", "%.2f", position);
            telemetry.addData("Dpad Up",   "+ 0.01");
            telemetry.addData("Dpad Down", "- 0.01");
            telemetry.update();
        }
    }
}
