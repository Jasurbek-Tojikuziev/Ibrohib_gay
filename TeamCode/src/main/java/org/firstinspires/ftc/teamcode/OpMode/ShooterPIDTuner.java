package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.MecanumDriveController;
import org.firstinspires.ftc.teamcode.Subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;

/**
 * Shooter PIDF + Hood + Velocity tuner.
 *
 * Controls:
 *   X button      — cycle active parameter (P → F → Velocity → Hood → Intake Power → Gate)
 *   D-Pad Up/Down — active param += / -= step
 *   B button      — cycle step size
 *   Right Trigger — manual intake (while not shooting)
 *   Right Bumper  — shoot sequence
 *   A button      — cut shooter power for 1 s, then spin back up (spin-down test)
 */
@TeleOp(name = "Shooter PIDF Tuner", group = "Testers")
public class ShooterPIDTuner extends OpMode {

    ShooterSubsystem       shooter;
    IntakeSubsystem        intake;
    MecanumDriveController driveController;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 1;

    double intakePower  = 0.75;
    double gatePosition = ShooterSubsystem.GATE_CLOSED;

    private enum Param { P, F, VELOCITY, HOOD, INTAKE_POWER, GATE }
    Param activeParam = Param.P;
    boolean lastX, lastB, lastDpadUp, lastDpadDown, lastRightBumper, lastA;

    private enum ShootState { IDLE, SHOOTING }
    ShootState shootState = ShootState.IDLE;
    ElapsedTime shootTimer = new ElapsedTime();

    boolean powerCutActive = false;
    ElapsedTime powerCutTimer = new ElapsedTime();

    @Override
    public void init() {
        shooter         = new ShooterSubsystem(hardwareMap, telemetry);
        intake          = new IntakeSubsystem(hardwareMap);
        driveController = new MecanumDriveController(new DriveSubsystem(hardwareMap));
        shooter.startMotors();
        shooter.setHoodPosition(ShooterSubsystem.HOOD_POSITION);
        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {

        // Drive — gamepad1 sticks
        driveController.update(gamepad1);

        // X: cycle active parameter
        boolean curX = gamepad1.x;
        if (curX && !lastX) {
            switch (activeParam) {
                case P:           activeParam = Param.F;            break;
                case F:           activeParam = Param.VELOCITY;     break;
                case VELOCITY:    activeParam = Param.HOOD;         break;
                case HOOD:        activeParam = Param.INTAKE_POWER; break;
                case INTAKE_POWER: activeParam = Param.GATE;        break;
                case GATE:        activeParam = Param.P;            break;
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

        // Right bumper: shoot sequence
        boolean curRightBumper = gamepad1.right_bumper;
        if (curRightBumper && !lastRightBumper && shootState == ShootState.IDLE) {
            shooter.setGatePosition(ShooterSubsystem.GATE_OPEN);
            intake.setPower(intakePower);
            shootTimer.reset();
            shootState = ShootState.SHOOTING;
        }
        lastRightBumper = curRightBumper;

        // Shoot sequence state machine
        if (shootState == ShootState.SHOOTING) {
            intake.setPower(intakePower);
            if (shootTimer.seconds() >= 1.5) {
                shooter.setGatePosition(ShooterSubsystem.GATE_CLOSED);
                intake.stop();
                shooter.setHoodPosition(ShooterSubsystem.HOOD_POSITION);
                shootState = ShootState.IDLE;
            }
        } else {
            // Manual intake via right trigger when not shooting
            if (gamepad1.right_trigger > 0.1) {
                intake.setPower(intakePower);
            } else {
                intake.stop();
            }
        }

        // Apply statics every loop
        shooter.updatePIDF();
        if (!powerCutActive) {
            shooter.setVelocity(ShooterSubsystem.TARGET_VELOCITY);
            shooter.updateMotorSync();
        }
        if (shootState != ShootState.SHOOTING) {
            shooter.setHoodPosition(ShooterSubsystem.HOOD_POSITION);
            shooter.setGatePosition(gatePosition);
        }

        double curVelocity = shooter.getMasterVelocity();
        double error       = ShooterSubsystem.TARGET_VELOCITY - curVelocity;

        telemetry.addData("Target Velocity",  "%.1f",  ShooterSubsystem.TARGET_VELOCITY);
        telemetry.addData("Current Velocity", "%.2f",  curVelocity);
        telemetry.addData("Error",            "%.2f",  error);
        telemetry.addLine("----------------------------");
        telemetry.addData("Active Param (X)", activeParam);
        telemetry.addData("P            (U/D)", "%.4f%s", ShooterSubsystem.PIDF_P,         activeParam == Param.P            ? " <--" : "");
        telemetry.addData("F            (U/D)", "%.4f%s", ShooterSubsystem.PIDF_F,         activeParam == Param.F            ? " <--" : "");
        telemetry.addData("Velocity     (U/D)", "%.1f%s",  ShooterSubsystem.TARGET_VELOCITY, activeParam == Param.VELOCITY     ? " <--" : "");
        telemetry.addData("Hood         (U/D)", "%.4f%s", ShooterSubsystem.HOOD_POSITION,  activeParam == Param.HOOD         ? " <--" : "");
        telemetry.addData("Intake Power (U/D)", "%.4f%s", intakePower,                     activeParam == Param.INTAKE_POWER ? " <--" : "");
        telemetry.addData("Gate Pos     (U/D)", "%.4f%s", gatePosition,                    activeParam == Param.GATE         ? " <--" : "");
        telemetry.addData("Step Size (B)",      "%.4f",   stepSizes[stepIndex]);
        telemetry.addLine("----------------------------");
        telemetry.addData("Shoot State",     shootState);
        telemetry.addData("Power Cut (A)",   powerCutActive ? String.format("CUTTING (%.1fs left)", 1.0 - powerCutTimer.seconds()) : "ready");
        telemetry.addData("Intake (R Trig)", gamepad1.right_trigger > 0.1 ? "ON" : "OFF");
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
            case INTAKE_POWER:
                intakePower += delta;
                intakePower = Math.max(0.0, Math.min(1.0, intakePower));
                break;
            case GATE:
                gatePosition += delta;
                gatePosition = Math.max(0.0, Math.min(1.0, gatePosition));
                break;
        }
    }
}
