package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class TurretController {
    public Gamepad gamepad;
    public Gamepad gamepad1; // For calibration (dpad left/right/up)
    private Turret turret;

    public boolean autoAimEnabled = true;

    private static final double MANUAL_SENSITIVITY = 0.35;
    private static final double JOYSTICK_DEADZONE = 0.1;
    private static final double CALIBRATION_POWER = 0.3;

    private boolean prevDpadLeft  = false;
    private boolean prevDpadRight = false;
    private boolean prevDpadUp    = false;

    public TurretController(Gamepad gamepad, Turret turret) {
        this.gamepad = gamepad;
        this.turret = turret;
    }

    public void update() {
        if (gamepad == null) return;

        // Dpad Up (gamepad1) — finalise calibration, reset encoder, enable auto-aim
        boolean dpadUpPressed = gamepad1 != null && gamepad1.dpad_up && !prevDpadUp;
        prevDpadUp = gamepad1 != null && gamepad1.dpad_up;
        if (dpadUpPressed) {
            turret.resetEncoder();       // clears offset internally via onEncoderReset()
            turret.setAutoAimOffset(0.0);
            autoAimEnabled = true;
            prevDpadLeft  = false;
            prevDpadRight = false;
            return;
        }

        // Dpad Left (gamepad1) — rotate left raw; on release reset encoder
        if (gamepad1 != null && gamepad1.dpad_left) {
            turret.manualRotateRaw(-CALIBRATION_POWER);
            autoAimEnabled = false;
            prevDpadLeft = true;
            return;
        } else if (gamepad1 != null && prevDpadLeft && !gamepad1.dpad_left) {
            turret.manualRotateRaw(0.0);
            turret.resetEncoder();
            autoAimEnabled = false;
            prevDpadLeft = false;
            return;
        }

        // Dpad Right (gamepad1) — rotate right raw; on release reset encoder
        if (gamepad1 != null && gamepad1.dpad_right) {
            turret.manualRotateRaw(CALIBRATION_POWER);
            autoAimEnabled = false;
            prevDpadRight = true;
            return;
        } else if (gamepad1 != null && prevDpadRight && !gamepad1.dpad_right) {
            turret.manualRotateRaw(0.0);
            turret.resetEncoder();
            autoAimEnabled = false;
            prevDpadRight = false;
            return;
        }

        // Left bumper (gamepad2) — re-enable auto-aim, preserving manual offset
        if (gamepad.left_bumper) {
            if (!autoAimEnabled) {
                double offset = turret.getCurrentAngle() - turret.getCalculatedTargetAngle();
                turret.setAutoAimOffset(offset);
            }
            autoAimEnabled = true;
        }

        double manualInput = gamepad.right_stick_x;

        if (Math.abs(manualInput) > JOYSTICK_DEADZONE) {
            if (autoAimEnabled) {
                turret.syncManualTarget();
                autoAimEnabled = false;
            }
            turret.manualControl(manualInput * MANUAL_SENSITIVITY);
        } else {
            if (autoAimEnabled) {
                turret.autoAim();
            } else {
                turret.manualControl(0.0);
            }
        }
    }

    public boolean isAutoAimEnabled() { return autoAimEnabled; }
    public void enableAutoAim()       { autoAimEnabled = true; }
    public void disableAutoAim()      { autoAimEnabled = false; }
}
