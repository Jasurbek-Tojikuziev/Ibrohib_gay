package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

@Config
@TeleOp(name="[PID] Shooter Motors Only", group="Testers")
public class ShooterPIDTuner extends LinearOpMode {

    // SDK PIDF коэффициенты (firmware-level ~1kHz, настраиваются через Dashboard)
    public static double PIDF_P        = 100.0;
    public static double PIDF_I        = 0.0;
    public static double PIDF_D        = 0.0;
    public static double PIDF_F        = 14;
    public static double TARGET_VELOCITY = 1500.0; // ticks/sec

    // Моторы
    private DcMotorEx shooterMotor1, shooterMotor2;

    // Hood servo
    private Servo hood;

    // Intake subsystem
    private Intake intake;

    private double targetVelocity = 0;

    // Last-applied PIDF — для детектирования изменений через Dashboard (I и D заблокированы в 0)
    private double lastP = PIDF_P, lastF = PIDF_F;

    // Button states (gamepad1)
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;

    // Button states (gamepad2 - fine control)
    private boolean prevDpadUp2 = false;
    private boolean prevDpadDown2 = false;
    private boolean prevDpadLeft2 = false;
    private boolean prevDpadRight2 = false;

    @Override
    public void runOpMode() {
        // FTC Dashboard телеметрия
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Инициализация моторов
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");

        // Motor1 — master, firmware-level velocity PIDF (~1kHz)
        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(PIDF_P, 0, 0, PIDF_F));

        // Motor2 — slave, no encoder, synced via feedforward
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Инициализация Hood
        hood = hardwareMap.get(Servo.class, "shooterHood");
        hood.setPosition(0.0); // Start at CLOSE position

        // Инициализация Intake
        intake = new Intake(hardwareMap);

        telemetry.addLine("=== SHOOTER PID TUNER ===");
        telemetry.addLine("Motors + Hood + Intake Test");
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("A: Start Motors");
        telemetry.addLine("B: Stop Motors");
        telemetry.addLine("Right Trigger: Intake ON");
        telemetry.addLine();
        telemetry.addLine("HOOD GAMEPAD1 (шаг 0.3):");
        telemetry.addLine("Dpad Down: 0.0");
        telemetry.addLine("Dpad Left: 0.3");
        telemetry.addLine("Right Bumper: 0.5");
        telemetry.addLine("Dpad Right: 0.6");
        telemetry.addLine("Dpad Up: 0.9");
        telemetry.addLine("Left Bumper: 1.0");
        telemetry.addLine();
        telemetry.addLine("HOOD GAMEPAD2 (шаг 0.1):");
        telemetry.addLine("Dpad Down: 0.1");
        telemetry.addLine("Dpad Left: 0.2");
        telemetry.addLine("Dpad Right: 0.4");
        telemetry.addLine("Dpad Up: 0.7");
        telemetry.addLine();
        telemetry.addLine("Tune PID via FTC Dashboard");
        telemetry.addData("Status", "Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleControls();
            updatePID();
            displayTelemetry();
            telemetry.update();
        }

        // Cleanup
        stopMotors();
        intake.off();
    }

    private void handleControls() {
        // A: Start motors
        if (gamepad1.a && !prevA) {
            targetVelocity = TARGET_VELOCITY;
        }

        // B: Stop motors
        if (gamepad1.b && !prevB) {
            stopMotors();
        }

        // Right Trigger: Intake control
        if (gamepad1.right_trigger > 0.1) {
            intake.on();
        } else {
            intake.off();
        }

        // Hood controls - gamepad1 (крупный шаг 0.3)
        if (gamepad1.dpad_down && !prevDpadDown) {
            hood.setPosition(0); // ≤30 cm
        }
        if (gamepad1.dpad_left && !prevDpadLeft) {
            hood.setPosition(0.3); // ≤50 cm
        }
        if (gamepad1.dpad_right && !prevDpadRight) {
            hood.setPosition(0.6); // ≤70 cm
        }
        if (gamepad1.dpad_up && !prevDpadUp) {
            hood.setPosition(0.9); // ≤100 cm
        }
        if (gamepad1.left_bumper && !prevLeftBumper) {
            hood.setPosition(1.0); // ≤150+ cm
        }
        if (gamepad1.right_bumper && !prevRightBumper) {
            hood.setPosition(0.5); // ≤150+ cm
        }

        // Hood controls - gamepad2 (точный шаг 0.1)
        if (gamepad2.dpad_down && !prevDpadDown2) {
            hood.setPosition(0.1); // Fine control
        }
        if (gamepad2.dpad_left && !prevDpadLeft2) {
            hood.setPosition(0.2); // Fine control
        }
        if (gamepad2.dpad_right && !prevDpadRight2) {
            hood.setPosition(0.4); // Fine control
        }
        if (gamepad2.dpad_up && !prevDpadUp2) {
            hood.setPosition(0.7); // Fine control
        }

        // Update button states
        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
        prevLeftBumper = gamepad1.left_bumper;
        prevRightBumper = gamepad1.right_bumper;

        prevDpadUp2 = gamepad2.dpad_up;
        prevDpadDown2 = gamepad2.dpad_down;
        prevDpadLeft2 = gamepad2.dpad_left;
        prevDpadRight2 = gamepad2.dpad_right;
    }

    private void updatePID() {
        if (targetVelocity == 0) {
            shooterMotor1.setVelocity(0);
            shooterMotor2.setPower(0);
            return;
        }

        // Переприменяем PIDF если изменили через FTC Dashboard
        if (PIDF_P != lastP || PIDF_F != lastF) {
            shooterMotor1.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(PIDF_P, 0, 0, PIDF_F));
            lastP = PIDF_P; lastF = PIDF_F;
        }

        shooterMotor1.setVelocity(targetVelocity);
        double error = targetVelocity - shooterMotor1.getVelocity();
        shooterMotor2.setPower((PIDF_F * targetVelocity + PIDF_P * error) / 32767.0);
    }

    private void stopMotors() {
        targetVelocity = 0;
        shooterMotor1.setVelocity(0);
        shooterMotor2.setPower(0);
        intake.off();
    }

    private void displayTelemetry() {
        // Motor1 в FORWARD, инверсия не нужна
        double currentVel = shooterMotor1.getVelocity();
        double error = targetVelocity - currentVel;

        telemetry.addLine("=== SHOOTER PID TUNER ===");
        telemetry.addData("Motors", "shooterMotor1, shooterMotor2");
        telemetry.addLine();

        // Status
        telemetry.addLine("--- STATUS ---");
        telemetry.addData("Shooter Running", targetVelocity > 0 ? "YES" : "NO");
        telemetry.addData("Intake", gamepad1.right_trigger > 0.1 ? "ON" : "OFF");
        telemetry.addLine();

        // Hood Position
        telemetry.addLine("--- HOOD ---");
        double hoodPos = hood.getPosition();
        telemetry.addData("Hood Servo Position", "%.2f", hoodPos);

        // Показываем какой дистанции соответствует текущая позиция
        String hoodDescription;
        if (Math.abs(hoodPos - 0.0) < 0.05) {
            hoodDescription = "≤30 cm (CLOSE)";
        } else if (Math.abs(hoodPos - 0.5) < 0.05) {
            hoodDescription = "≤50 cm";
        } else if (Math.abs(hoodPos - 0.7) < 0.05) {
            hoodDescription = "≤70 cm (MIDDLE)";
        } else if (Math.abs(hoodPos - 0.9) < 0.05) {
            hoodDescription = "≤100 cm";
        } else if (Math.abs(hoodPos - 1.0) < 0.05) {
            hoodDescription = "≤150+ cm (FAR)";
        } else {
            hoodDescription = "Custom";
        }
        telemetry.addData("Hood Angle", hoodDescription);
        telemetry.addLine();

        // Velocity
        telemetry.addLine("--- VELOCITY ---");
        telemetry.addData("Target", "%.0f ticks/sec", targetVelocity);
        telemetry.addData("Current", "%.0f ticks/sec", currentVel);
        telemetry.addData("Error", "%.0f ticks/sec", error);

        if (targetVelocity > 0) {
            double accuracy = (currentVel / targetVelocity) * 100.0;
            telemetry.addData("Accuracy", "%.1f%%", accuracy);
        }
        telemetry.addLine();

        // PIDF Gains
        telemetry.addLine("--- SDK PIDF (Dashboard) ---");
        telemetry.addData("PIDF_P", "%.2f", PIDF_P);
        telemetry.addData("PIDF_I", "%.2f", PIDF_I);
        telemetry.addData("PIDF_D", "%.2f", PIDF_D);
        telemetry.addData("PIDF_F", "%.4f", PIDF_F);
        telemetry.addLine();

        // Motor Power
        telemetry.addLine("--- MOTOR POWER ---");
        telemetry.addData("Motor1 Power", "%.3f", shooterMotor1.getPower());
        telemetry.addData("Motor2 Power", "%.3f", shooterMotor2.getPower());
        telemetry.addLine();

        // Controls
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addData("A", "Start motors");
        telemetry.addData("B", "Stop motors");
        telemetry.addData("Right Trigger", "Intake ON");
        telemetry.addLine();
        telemetry.addLine("HOOD GP1 (шаг 0.3):");
        telemetry.addLine("↓ 0.0 | ← 0.3 | → 0.6 | ↑ 0.9");
        telemetry.addLine("LB: 1.0 | RB: 0.5");
        telemetry.addLine();
        telemetry.addLine("HOOD GP2 (шаг 0.1):");
        telemetry.addLine("↓ 0.1 | ← 0.2 | → 0.4 | ↑ 0.7");
        telemetry.addLine();

        // Tuning Tips
        telemetry.addLine("--- TUNING TIPS ---");
        if (targetVelocity > 0) {
            if (Math.abs(error) > 200) {
                telemetry.addLine("Large error → Increase PIDF_P");
            } else if (Math.abs(error) > 50) {
                telemetry.addLine("Getting close → Add PIDF_D");
            } else if (Math.abs(error) > 20) {
                telemetry.addLine("Good! Fine tune PIDF_F");
            } else {
                telemetry.addLine("Excellent velocity control!");
            }

            if (currentVel > targetVelocity * 1.1) {
                telemetry.addLine("Overshoot → Reduce PIDF_P or add PIDF_D");
            }
        } else {
            telemetry.addLine("Press A to start");
        }
    }
}
