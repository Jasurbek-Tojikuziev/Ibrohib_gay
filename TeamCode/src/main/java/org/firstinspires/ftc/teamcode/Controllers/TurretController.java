package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class TurretController {
    public Gamepad gamepad;
    public Gamepad gamepad1; // For calibration (dpad left/right/up)
    private Turret turret;

    public boolean autoAimEnabled = true;

    private static final double MANUAL_SENSITIVITY_SLOW = 0.2;   // hold < 1s: fine adjustment
    private static final double MANUAL_SENSITIVITY_FAST = 0.35;  // hold ≥ 1s: full speed
    private static final double HOLD_THRESHOLD_SEC      = 1.0;

    /** True when GP1 dpad_left or dpad_right was pressed last frame — detects release transition. */
    private boolean     wasManualActive = false;
    private ElapsedTime dpadHoldTimer   = new ElapsedTime();

    public TurretController(Gamepad gamepad, Turret turret) {
        this.gamepad = gamepad;
        this.turret = turret;
    }

    public void update() {
        if (gamepad == null) return;

        // GP1 Dpad Left/Right — manual turret with seamless return to auto-aim on release
        boolean dpadLeft  = gamepad1 != null && gamepad1.dpad_left;
        boolean dpadRight = gamepad1 != null && gamepad1.dpad_right;
        boolean manualActive = dpadLeft || dpadRight;
        double  manualInput  = dpadLeft ? -1.0 : (dpadRight ? 1.0 : 0.0);

        if (manualActive) {
            // First frame: sync PID target to current angle to prevent jerk
            if (!wasManualActive) {
                turret.syncManualTarget();
                dpadHoldTimer.reset();
            }
            autoAimEnabled = false;
            double sensitivity = dpadHoldTimer.seconds() >= HOLD_THRESHOLD_SEC
                    ? MANUAL_SENSITIVITY_FAST
                    : MANUAL_SENSITIVITY_SLOW;
            turret.manualControl(manualInput * sensitivity);
            // Background: getCalculatedTargetAngle() (odometry) keeps updating silently
        } else if (wasManualActive) {
            // Dpad just released — manual trim = how far we moved from where auto-aim (odometry +
            // camera relocOffset) would have pointed. Subtract relocOffset so the manual trim is
            // independent of the camera correction and they don't fight.
            double offset = turret.getCurrentAngle() - turret.getCalculatedTargetAngle() - turret.getRelocOffset();
            turret.setAutoAimOffset(offset);
            autoAimEnabled = true;
            turret.autoAim();   // snap immediately to corrected target
        } else {
            if (autoAimEnabled) {
                turret.autoAim();
            } else {
                turret.manualControl(0.0);
            }
        }

        wasManualActive = manualActive;
    }

    public boolean isAutoAimEnabled() { return autoAimEnabled; }
    public void enableAutoAim()       { autoAimEnabled = true; }
    public void disableAutoAim()      { autoAimEnabled = false; }
}
