package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="[TEST] Intake", group="Testers")
public class IntakeTester extends LinearOpMode {

    private DcMotor intake;

    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "intake");

        telemetry.addLine("=== INTAKE TESTER ===");
        telemetry.addLine("D-pad Up:   Forward (intake in)");
        telemetry.addLine("D-pad Down: Reverse (eject)");
        telemetry.addLine("Release:    Stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                intake.setPower(1.0);
                telemetry.addData("Status", "FORWARD");
            } else if (gamepad1.dpad_down) {
                intake.setPower(-1.0);
                telemetry.addData("Status", "REVERSE");
            } else {
                intake.setPower(0.0);
                telemetry.addData("Status", "STOPPED");
            }

            telemetry.update();
        }
    }
}
