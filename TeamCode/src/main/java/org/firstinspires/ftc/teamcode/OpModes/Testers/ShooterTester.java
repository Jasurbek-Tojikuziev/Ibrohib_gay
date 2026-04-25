package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Shooter;

@Config
@TeleOp(name="[TEST] Shooter Tester", group="Testers")
public class ShooterTester extends LinearOpMode {

    // Целевая скорость для теста (A кнопка)
    // PIDF_P/I/D/F туните прямо в Shooter через FTC Dashboard
    public static double TARGET_VELOCITY = 1760; // ticks/sec

    private Shooter shooter;

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    @Override
    public void runOpMode() {
        // Настройка телеметрии для Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initializing Shooter...");
        telemetry.update();

        shooter = new Shooter(hardwareMap);

        telemetry.addData("Status", "Ready!");
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("A: Start Motors");
        telemetry.addLine("B: Stop Motors");
        telemetry.addLine("Dpad Up: +100 velocity");
        telemetry.addLine("Dpad Down: -100 velocity");
        telemetry.addLine();
        telemetry.addLine("Tune PIDF via Dashboard:");
        telemetry.addLine("Shooter.PIDF_P/I/D/F, Shooter.TARGET_VELOCITY");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            shooter.updatePID();

            handleControls();
            displayTelemetry();
            telemetry.update();
        }

        shooter.off();
    }

    private void handleControls() {
        // A: Включить моторы
        if (gamepad1.a && !prevA) {
            shooter.setTargetVelocity(TARGET_VELOCITY);
        }

        // B: Выключить моторы
        if (gamepad1.b && !prevB) {
            shooter.off();
        }

        // Dpad Up: Увеличить target velocity
        if (gamepad1.dpad_up && !prevDpadUp) {
            TARGET_VELOCITY += 100;
        }

        // Dpad Down: Уменьшить target velocity
        if (gamepad1.dpad_down && !prevDpadDown) {
            TARGET_VELOCITY -= 100;
            if (TARGET_VELOCITY < 0) TARGET_VELOCITY = 0;
        }

        // Сохраняем состояния кнопок
        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
    }

    private void displayTelemetry() {
        telemetry.addLine("=== SHOOTER TESTER ===");
        telemetry.addLine();

        // Status
        telemetry.addLine("--- STATUS ---");
        telemetry.addData("Motors Running", shooter.isShooting() ? "YES" : "NO");
        telemetry.addLine();

        // PID Status
        telemetry.addLine("--- PID ---");
        double targetVel = shooter.getTargetVelocity();
        double currentVel = shooter.getCurrentVelocity();
        double error = targetVel - currentVel;

        telemetry.addData("Target Velocity", "%.0f ticks/sec", targetVel);
        telemetry.addData("Current Velocity", "%.0f ticks/sec", currentVel);
        telemetry.addData("Error", "%.0f ticks/sec", error);

        // Процент точности
        if (targetVel > 0) {
            double accuracy = (currentVel / targetVel) * 100.0;
            telemetry.addData("Accuracy", "%.1f%%", accuracy);
        }

        telemetry.addLine();
        telemetry.addData("PIDF_P", "%.2f", Shooter.PIDF_P);
        telemetry.addData("PIDF_I", "%.2f", Shooter.PIDF_I);
        telemetry.addData("PIDF_D", "%.2f", Shooter.PIDF_D);
        telemetry.addData("PIDF_F", "%.4f", Shooter.PIDF_F);
        telemetry.addLine();

        // Controls
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("A", "Start Motors");
        telemetry.addData("B", "Stop Motors");
        telemetry.addData("Dpad Up", "Increase Velocity (+100)");
        telemetry.addData("Dpad Down", "Decrease Velocity (-100)");
        telemetry.addLine();

        // Tuning tips
        telemetry.addLine("=== TUNING TIPS ===");
        if (targetVel > 0) {
            if (Math.abs(error) > 200) {
                telemetry.addLine("Large error -> Increase kP");
            } else if (Math.abs(error) > 50) {
                telemetry.addLine("Getting close -> Fine tune kP or add kD");
            } else if (Math.abs(error) > 20) {
                telemetry.addLine("Good! Try adding small kI");
            } else {
                telemetry.addLine("Excellent velocity control!");
            }

            if (currentVel > targetVel * 1.1) {
                telemetry.addLine("Overshooting -> Reduce kP or add kD");
            }
        } else {
            telemetry.addLine("Press A to start motors");
        }
    }
}
