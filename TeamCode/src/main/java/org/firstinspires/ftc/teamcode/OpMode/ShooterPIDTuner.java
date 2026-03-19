package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;

/**
 * Shooter PIDF + Hood + Velocity tuner.
 *
 * All tunable values are ShooterSubsystem statics, so they are visible and
 * editable on FTC Dashboard under the "ShooterSubsystem" config panel.
 * Gamepad adjustments write directly to those same statics — both inputs stay
 * in sync automatically.
 *
 * Controls:
 *   X button       — cycle active parameter (P → F → Velocity → Hood → ...)
 *   D-Pad Up        — active param += step
 *   D-Pad Down      — active param -= step
 *   B button        — cycle step size
 *   Right Trigger   — manual intake (while not shooting)
 *   Right Bumper    — shoot sequence with hood drops (drop 1 + drop 2)
 *   Left Bumper     — shoot sequence without hood drops
 *   A button        — cut shooter power for 1 s, then spin back up (spin-down test)
 */
@TeleOp(name = "Shooter PIDF Tuner", group = "Testers")
public class ShooterPIDTuner extends OpMode {

    ShooterSubsystem shooter;
    IntakeSubsystem  intake;

    final FtcDashboard dashboard = FtcDashboard.getInstance();

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 1;

    private enum Param { P, F, VELOCITY, HOOD, HOOD_DROP_TIME, HOOD_DROP_TIME_2, HOOD_DROP_1, HOOD_DROP_2 }
    Param activeParam = Param.P;

    boolean lastX, lastB, lastDpadUp, lastDpadDown, lastRightBumper, lastLeftBumper, lastA;

    private enum ShootState { IDLE, SHOOTING }
    ShootState shootState = ShootState.IDLE;
    ElapsedTime shootTimer = new ElapsedTime();

    boolean powerCutActive = false;
    ElapsedTime powerCutTimer = new ElapsedTime();

    boolean hoodDropped  = false;
    boolean hoodDropped2 = false;
    boolean shootWithDrops = false;

    @Override
    public void init() {
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        intake  = new IntakeSubsystem(hardwareMap);
        shooter.startMotors();
        shooter.setHoodPosition(ShooterSubsystem.HOOD_POSITION);
        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {

        // X: cycle active parameter
        boolean curX = gamepad1.x;
        if (curX && !lastX) {
            switch (activeParam) {
                case P:             activeParam = Param.F;             break;
                case F:             activeParam = Param.VELOCITY;      break;
                case VELOCITY:      activeParam = Param.HOOD;          break;
                case HOOD:           activeParam = Param.HOOD_DROP_TIME;   break;
                case HOOD_DROP_TIME:   activeParam = Param.HOOD_DROP_TIME_2; break;
                case HOOD_DROP_TIME_2: activeParam = Param.HOOD_DROP_1;     break;
                case HOOD_DROP_1:      activeParam = Param.HOOD_DROP_2;     break;
                case HOOD_DROP_2:      activeParam = Param.P;               break;
            }
        }
        lastX = curX;

        // B: cycle step size
        boolean curB = gamepad1.b;
        if (curB && !lastB) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        lastB = curB;

        // D-Pad Up: active param += step
        boolean curDpadUp = gamepad1.dpad_up;
        if (curDpadUp && !lastDpadUp) {
            adjustParam(+stepSizes[stepIndex]);
        }
        lastDpadUp = curDpadUp;

        // D-Pad Down: active param -= step
        boolean curDpadDown = gamepad1.dpad_down;
        if (curDpadDown && !lastDpadDown) {
            adjustParam(-stepSizes[stepIndex]);
        }
        lastDpadDown = curDpadDown;

        // A: cut shooter power for 1 s then spin back up
        boolean curA = gamepad1.a;
        if (curA && !lastA && !powerCutActive) {
            shooter.stopMotors();
            powerCutTimer.reset();
            powerCutActive = true;
        }
        lastA = curA;

        if (powerCutActive && powerCutTimer.seconds() >= 1.0) {
            shooter.startMotors();
            powerCutActive = false;
        }

        // Right bumper: shoot with hood drops
        boolean curRightBumper = gamepad1.right_bumper;
        if (curRightBumper && !lastRightBumper && shootState == ShootState.IDLE) {
            shooter.setGatePosition(ShooterSubsystem.GATE_OPEN);
            intake.setPower(1.0);
            shootTimer.reset();
            shootWithDrops = true;
            shootState = ShootState.SHOOTING;
        }
        lastRightBumper = curRightBumper;

        // Left bumper: shoot without hood drops
        boolean curLeftBumper = gamepad1.left_bumper;
        if (curLeftBumper && !lastLeftBumper && shootState == ShootState.IDLE) {
            shooter.setGatePosition(ShooterSubsystem.GATE_OPEN);
            intake.setPower(1.0);
            shootTimer.reset();
            shootWithDrops = false;
            shootState = ShootState.SHOOTING;
        }
        lastLeftBumper = curLeftBumper;

        // Shoot sequence state machine
        if (shootState == ShootState.SHOOTING) {
            intake.setPower(1.0);

            if (shootWithDrops) {
                // First drop at HOOD_DROP_TIME
                if (!hoodDropped && shootTimer.seconds() >= ShooterSubsystem.HOOD_DROP_TIME) {
                    shooter.setHoodPosition(Math.max(0.0, ShooterSubsystem.HOOD_POSITION - ShooterSubsystem.HOOD_DROP_1));
                    hoodDropped = true;
                }

                // Second drop at HOOD_DROP_TIME_2
                if (!hoodDropped2 && shootTimer.seconds() >= ShooterSubsystem.HOOD_DROP_TIME_2) {
                    shooter.setHoodPosition(Math.max(0.0, ShooterSubsystem.HOOD_POSITION - ShooterSubsystem.HOOD_DROP_1 - ShooterSubsystem.HOOD_DROP_2));
                    hoodDropped2 = true;
                }
            }

            if (shootTimer.seconds() >= 1.5) {
                shooter.setGatePosition(ShooterSubsystem.GATE_CLOSED);
                intake.stop();
                shooter.setHoodPosition(ShooterSubsystem.HOOD_POSITION); // restore hood
                hoodDropped  = false;
                hoodDropped2 = false;
                shootState = ShootState.IDLE;
            }
        } else {
            // Manual intake via right trigger when not shooting
            if (gamepad1.right_trigger > 0.1) {
                intake.setPower(1.0);
            } else {
                intake.stop();
            }
        }

        // Apply statics to subsystem every loop (skip motor commands during power cut)
        shooter.updatePIDF();
        if (!powerCutActive) {
            shooter.setVelocity(ShooterSubsystem.TARGET_VELOCITY);
            shooter.updateMotorSync();
        }
        // Only apply global hood position when not shooting (shoot sequence owns it during SHOOTING)
        if (shootState != ShootState.SHOOTING) {
            shooter.setHoodPosition(ShooterSubsystem.HOOD_POSITION);
        }

        double curVelocity = shooter.getMasterVelocity();
        double error       = ShooterSubsystem.TARGET_VELOCITY - curVelocity;

        // FTC Dashboard: live velocity graph
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("velocity (ticks/s)", curVelocity);
        packet.put("target   (ticks/s)", ShooterSubsystem.TARGET_VELOCITY);
        packet.put("error    (ticks/s)", error);
        dashboard.sendTelemetryPacket(packet);

        // Driver Station telemetry
        telemetry.addData("Target Velocity",  "%.1f",  ShooterSubsystem.TARGET_VELOCITY);
        telemetry.addData("Current Velocity", "%.2f",  curVelocity);
        telemetry.addData("Error",            "%.2f",  error);
        telemetry.addLine("----------------------------");
        telemetry.addData("Active Param (X)", activeParam);
        telemetry.addData("P             (U/D)", "%.4f%s", ShooterSubsystem.PIDF_P,          activeParam == Param.P             ? " <--" : "");
        telemetry.addData("F             (U/D)", "%.4f%s", ShooterSubsystem.PIDF_F,          activeParam == Param.F             ? " <--" : "");
        telemetry.addData("Velocity      (U/D)", "%.1f%s",  ShooterSubsystem.TARGET_VELOCITY,  activeParam == Param.VELOCITY      ? " <--" : "");
        telemetry.addData("Hood          (U/D)", "%.4f%s", ShooterSubsystem.HOOD_POSITION,   activeParam == Param.HOOD          ? " <--" : "");
        telemetry.addData("Drop 1 Time   (U/D)", "%.3fs%s", ShooterSubsystem.HOOD_DROP_TIME,   activeParam == Param.HOOD_DROP_TIME   ? " <--" : "");
        telemetry.addData("Drop 1 Amount (U/D)", "%.4f%s",  ShooterSubsystem.HOOD_DROP_1,      activeParam == Param.HOOD_DROP_1      ? " <--" : "");
        telemetry.addData("Drop 2 Time   (U/D)", "%.3fs%s", ShooterSubsystem.HOOD_DROP_TIME_2, activeParam == Param.HOOD_DROP_TIME_2 ? " <--" : "");
        telemetry.addData("Drop 2 Amount (U/D)", "%.4f%s",  ShooterSubsystem.HOOD_DROP_2,      activeParam == Param.HOOD_DROP_2      ? " <--" : "");
        telemetry.addData("Step Size (B)",    "%.4f",   stepSizes[stepIndex]);
        telemetry.addLine("----------------------------");
        telemetry.addData("Shoot State",      shootState + (shootState == ShootState.SHOOTING ? (shootWithDrops ? " (drops)" : " (no drops)") : ""));
        telemetry.addData("Power Cut (A)",    powerCutActive ? String.format("CUTTING (%.1fs left)", 1.0 - powerCutTimer.seconds()) : "ready");
        telemetry.addData("Intake (R Trig)",  gamepad1.right_trigger > 0.1 ? "ON" : "OFF");
        telemetry.update();
    }

    private void adjustParam(double delta) {
        switch (activeParam) {
            case P:
                ShooterSubsystem.PIDF_P += delta;
                break;
            case F:
                ShooterSubsystem.PIDF_F += delta;
                break;
            case VELOCITY:
                ShooterSubsystem.TARGET_VELOCITY += delta;
                if (ShooterSubsystem.TARGET_VELOCITY < 0) ShooterSubsystem.TARGET_VELOCITY = 0;
                break;
            case HOOD:
                ShooterSubsystem.HOOD_POSITION += delta;
                ShooterSubsystem.HOOD_POSITION = Math.max(0.0, Math.min(1.0, ShooterSubsystem.HOOD_POSITION));
                break;
            case HOOD_DROP_TIME:
                ShooterSubsystem.HOOD_DROP_TIME += delta;
                if (ShooterSubsystem.HOOD_DROP_TIME < 0)   ShooterSubsystem.HOOD_DROP_TIME = 0;
                if (ShooterSubsystem.HOOD_DROP_TIME > 1.5) ShooterSubsystem.HOOD_DROP_TIME = 1.5;
                break;
            case HOOD_DROP_TIME_2:
                ShooterSubsystem.HOOD_DROP_TIME_2 += delta;
                if (ShooterSubsystem.HOOD_DROP_TIME_2 < 0)   ShooterSubsystem.HOOD_DROP_TIME_2 = 0;
                if (ShooterSubsystem.HOOD_DROP_TIME_2 > 1.5) ShooterSubsystem.HOOD_DROP_TIME_2 = 1.5;
                break;
            case HOOD_DROP_1:
                ShooterSubsystem.HOOD_DROP_1 += delta;
                ShooterSubsystem.HOOD_DROP_1 = Math.max(0.0, Math.min(1.0, ShooterSubsystem.HOOD_DROP_1));
                break;
            case HOOD_DROP_2:
                ShooterSubsystem.HOOD_DROP_2 += delta;
                ShooterSubsystem.HOOD_DROP_2 = Math.max(0.0, Math.min(1.0, ShooterSubsystem.HOOD_DROP_2));
                break;
        }
    }
}
