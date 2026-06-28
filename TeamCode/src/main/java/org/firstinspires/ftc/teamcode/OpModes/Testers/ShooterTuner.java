package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "Shooter Tuner", group = "Testers")
public class ShooterTuner extends LinearOpMode {

    private DcMotorEx motor1, motor2;
    private DcMotor intake, intake2;
    private Servo hood;

    private double targetVelocity = 1000;
    private double hoodPosition   = 0.5;
    private double pidfP          = 190.0;
    private double pidfF          = 13.0;
    private boolean motorsRunning = true;

    private boolean prevLSB = false;

    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        intake  = hardwareMap.get(DcMotor.class, "Intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2 = hardwareMap.get(DcMotor.class, "Intake2");
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        hood   = hardwareMap.get(Servo.class, "shooterHood");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hood.setDirection(Servo.Direction.FORWARD);
        ((ServoImplEx) hood).setPwmRange(new PwmControl.PwmRange(500, 2500));
        applyPIDF();
        hood.setPosition(hoodPosition);

        telemetry.addLine("Shooter Tuner — Ready. Press Start.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean rb = gamepad1.right_bumper;
            boolean lb = gamepad1.left_bumper;

            // Toggle motors — edge only
            boolean lsbPressed = gamepad1.left_stick_button && !prevLSB;
            if (lsbPressed) motorsRunning = !motorsRunning;
            prevLSB = gamepad1.left_stick_button;

            // Velocity — hold to repeat, RB = large step
            if (gamepad1.dpad_up)   targetVelocity += rb ? 100 : 10;
            if (gamepad1.dpad_down) targetVelocity -= rb ? 100 : 10;
            targetVelocity = Math.max(0, targetVelocity);

            // Hood — hold to repeat, RB = large step
            if (gamepad1.dpad_left)  hoodPosition -= rb ? 0.1 : 0.01;
            if (gamepad1.dpad_right) hoodPosition += rb ? 0.1 : 0.01;
            hoodPosition = Math.max(0.0, Math.min(1.0, hoodPosition));
            if (gamepad1.dpad_left || gamepad1.dpad_right) hood.setPosition(hoodPosition);

            // PIDF P — hold to repeat, LB = large step
            boolean pidfChanged = false;
            if (gamepad1.y) { pidfP += lb ? 10 : 1;   pidfChanged = true; }
            if (gamepad1.a) { pidfP -= lb ? 10 : 1;   pidfChanged = true; }

            // PIDF F — hold to repeat, LB = large step
            if (gamepad1.b) { pidfF += lb ? 5 : 0.5;  pidfChanged = true; }
            if (gamepad1.x) { pidfF -= lb ? 5 : 0.5;  pidfChanged = true; }

            if (pidfChanged) {
                pidfP = Math.max(0, pidfP);
                pidfF = Math.max(0, pidfF);
                applyPIDF();
            }

            // Intake — hold right bumper + right trigger
            double intakePower = gamepad1.right_trigger > 0.5 ? 1.0 : 0.0;
            intake.setPower(intakePower);
            intake2.setPower(intakePower);

            // Drive motors
            double currentVel = motor1.getVelocity();
            if (motorsRunning) {
                motor1.setVelocity(targetVelocity);
                motor2.setPower(motor1.getPower());
            } else {
                motor1.setPower(0);
                motor2.setPower(0);
            }

            // Telemetry
            telemetry.addLine("=== SHOOTER TUNER ===");
            telemetry.addData("Motors", motorsRunning ? "ON" : "OFF  (LSB to toggle)");
            telemetry.addLine("--- Velocity ---");
            telemetry.addData("Target ", "%.0f ticks/s", targetVelocity);
            telemetry.addData("Current", "%.0f ticks/s", currentVel);
            telemetry.addLine("--- Hood ---");
            telemetry.addData("Position", "%.2f", hoodPosition);
            telemetry.addLine("--- PIDF ---");
            telemetry.addData("P", "%.1f", pidfP);
            telemetry.addData("F", "%.2f", pidfF);
            telemetry.addLine("--- Controls (hold to repeat) ---");
            telemetry.addData("dpad up/down  ", rb ? "vel ±100" : "vel ±10");
            telemetry.addData("dpad left/right", rb ? "hood ±0.1" : "hood ±0.01");
            telemetry.addData("Y / A         ", lb ? "P ±10"  : "P ±1");
            telemetry.addData("B / X         ", lb ? "F ±5"   : "F ±0.5");
            telemetry.addData("Right Trigger ", "intake (hold)");
            telemetry.update();

            sleep(100); // 10 Hz repeat rate while held
        }

        motor1.setPower(0);
        motor2.setPower(0);
        intake.setPower(0);
        intake2.setPower(0);
    }

    private void applyPIDF() {
        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(pidfP, 0, 0, pidfF));
    }
}
