package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Test OpMode for the dual-Axon turret (two servos, 1:1 with turret, both same direction).
 *
 * Full 360° travel: position 0.0–1.0 is mapped to 0–360° by widening the PWM range to the
 * full Axon pulse window (500–2500 µs). Without that, 0.0–1.0 would only cover part of the travel.
 *   - Center (180°)              = 0.5
 *   - Full RIGHT (+180° / 360°)  = 1.0
 *   - Full LEFT  (-180° / 0°)    = 0.0
 *
 * Controls (gamepad1):
 *   - Left stick X : sweep continuously at max speed (hold) — runs until physical limit or release
 *   - Dpad RIGHT   : step +90°  (press twice from center → full right)
 *   - Dpad LEFT    : step -90°  (press twice from center → full left)
 *   - A            : recenter to 180°
 *
 * Hardware config: two servos named "turretServo1" and "turretServo2".
 */
@TeleOp(name = "[TEST] Axon Turret Servos", group = "Testers")
public class AxonTurretTester extends LinearOpMode {

    private ServoImplEx turretServo1;
    private ServoImplEx turretServo2;

    private static final double CENTER  = 0.5;   // 180°
    private static final double STEP_90 = 0.25;  // 90° in 0..1 units (full range = 360°)

    // Left-stick sweep: position units per second. Set high so the servo runs at its own max speed.
    // If it overshoots/keeps moving after you release, lower this to match the servo's real slew rate.
    private static final double SWEEP_RATE     = 2.0;
    private static final double STICK_DEADZONE = 0.1;

    private double targetPos = CENTER;
    private boolean prevRight = false, prevLeft = false, prevA = false;
    private final ElapsedTime sweepTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        turretServo1 = hardwareMap.get(ServoImplEx.class, "turretServo1");
        turretServo2 = hardwareMap.get(ServoImplEx.class, "turretServo2");

        // Widen PWM to the full Axon range so 0.0–1.0 spans the full ~360°.
        turretServo1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turretServo2.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Both servos rotate the SAME direction → same position value.
        // (If one is physically mirrored, set turretServo2.setDirection(Servo.Direction.REVERSE).)

        targetPos = CENTER;
        applyPosition();

        telemetry.addLine("Ready — both servos centered at 180°");
        telemetry.addLine("Dpad RIGHT = +90°, Dpad LEFT = -90°, A = recenter");
        telemetry.update();

        waitForStart();
        sweepTimer.reset();

        while (opModeIsActive()) {
            double dt = sweepTimer.seconds();
            sweepTimer.reset();

            // Left stick X — continuous max-speed sweep while held; stops at limit (0/1) or on release.
            double sx = gamepad1.left_stick_x;            // right = +, left = -
            if (Math.abs(sx) > STICK_DEADZONE) {
                targetPos += Math.signum(sx) * SWEEP_RATE * dt;   // constant max speed (direction only)
                targetPos = Math.max(0.0, Math.min(1.0, targetPos));
            }

            boolean right = gamepad1.dpad_right;
            boolean left  = gamepad1.dpad_left;
            boolean a     = gamepad1.a;

            // Rising-edge detection → exactly one 90° step per press.
            if (right && !prevRight) targetPos = Math.min(1.0, targetPos + STEP_90);
            if (left  && !prevLeft)  targetPos = Math.max(0.0, targetPos - STEP_90);
            if (a     && !prevA)     targetPos = CENTER;

            prevRight = right;
            prevLeft  = left;
            prevA     = a;

            applyPosition();

            double angle      = targetPos * 360.0;   // 0..360°
            double fromCenter = angle - 180.0;       // -180..+180°

            telemetry.addLine("=== AXON TURRET TEST ===");
            telemetry.addData("Position (0..1)", "%.3f", targetPos);
            telemetry.addData("Angle", "%.0f°  (%+.0f° from center)", angle, fromCenter);
            telemetry.addData("Left stick X", "%.2f", sx);
            telemetry.addLine();
            telemetry.addLine("L-stick X = sweep   Dpad R/L = +/-90°   A = center");
            telemetry.update();
        }
    }

    private void applyPosition() {
        turretServo1.setPosition(targetPos);
        turretServo2.setPosition(targetPos);
    }
}
