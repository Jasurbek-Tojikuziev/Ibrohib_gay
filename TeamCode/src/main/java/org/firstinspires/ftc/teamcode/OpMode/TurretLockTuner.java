package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Controllers.MecanumDriveController;
import org.firstinspires.ftc.teamcode.Subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.TurretSubsystem;

/**
 * Turret lock PID tuner.
 *
 * Locks the turret at a fixed angle (0°) and lets you drive the robot around
 * to test how well the turret holds position under disturbance.
 *
 * Controls (gamepad2):
 *   X button   — cycle active parameter: P → F → P ...
 *   D-Pad Up   — active param += step
 *   D-Pad Down — active param -= step
 *   B button   — cycle step size (0.1 → 0.01 → 0.001 → 0.0001)
 *
 * Drivetrain: gamepad1 (standard mecanum — left stick move, right stick rotate).
 */
@TeleOp(name = "Turret Lock Tuner", group = "Testers")
public class TurretLockTuner extends OpMode {

    private TurretSubsystem       turret;
    private MecanumDriveController driveController;

    private static final double LOCK_ANGLE = 0.0; // degrees — turret faces forward

    private enum Param { P, F }
    private Param activeParam = Param.P;

    private final double[] stepSizes = { 0.1, 0.01, 0.001, 0.0001 };
    private int stepIndex = 1; // default step = 0.01

    private boolean lastX, lastB, lastDpadUp, lastDpadDown;

    @Override
    public void init() {
        turret          = new TurretSubsystem(hardwareMap);
        driveController = new MecanumDriveController(new DriveSubsystem(hardwareMap));

        turret.setTargetAngle(LOCK_ANGLE);

        telemetry.addLine("Turret Lock Tuner ready");
        telemetry.addLine("Drive: gamepad1  |  Tune: gamepad2");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ── Drivetrain (gamepad1) ─────────────────────────────────────────────
        driveController.update(gamepad1);

        // ── Turret: always locked at LOCK_ANGLE using lock PID ───────────────
        turret.setPID(TurretSubsystem.kP_lock, 0, 0, TurretSubsystem.kF_lock);
        turret.setTargetAngle(LOCK_ANGLE);
        turret.update();

        // ── Cycle active parameter (X, rising edge) ───────────────────────────
        if (gamepad2.x && !lastX) {
            activeParam = (activeParam == Param.P) ? Param.F : Param.P;
        }
        lastX = gamepad2.x;

        // ── Cycle step size (B, rising edge) ─────────────────────────────────
        if (gamepad2.b && !lastB) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        lastB = gamepad2.b;

        double step = stepSizes[stepIndex];

        // ── Adjust active parameter (D-Pad, rising edge) ─────────────────────
        boolean dpadUp   = gamepad2.dpad_up;
        boolean dpadDown = gamepad2.dpad_down;

        if (dpadUp && !lastDpadUp) {
            if (activeParam == Param.P) TurretSubsystem.kP_lock += step;
            else                        TurretSubsystem.kF_lock += step;
        }
        if (dpadDown && !lastDpadDown) {
            if (activeParam == Param.P) TurretSubsystem.kP_lock = Math.max(0, TurretSubsystem.kP_lock - step);
            else                        TurretSubsystem.kF_lock = Math.max(0, TurretSubsystem.kF_lock - step);
        }
        lastDpadUp   = dpadUp;
        lastDpadDown = dpadDown;

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetry.addData("Active param", activeParam.name() + "  (X to cycle)");
        telemetry.addData("Step size",    step + "  (B to cycle)");
        telemetry.addData("kP_lock",      "%.4f", TurretSubsystem.kP_lock);
        telemetry.addData("kF_lock",      "%.4f", TurretSubsystem.kF_lock);
        telemetry.addData("Lock angle°",  LOCK_ANGLE);
        telemetry.addData("Actual angle°","%.2f", turret.getCurrentAngle());
        telemetry.addData("Error°",       "%.2f", LOCK_ANGLE - turret.getCurrentAngle());
        telemetry.update();
    }
}
