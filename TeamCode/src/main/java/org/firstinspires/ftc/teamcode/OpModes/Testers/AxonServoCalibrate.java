package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Calibration tester for the dual-Axon turret — measure the real travel and how much servo
 * position (0..1) equals one degree.
 *
 * How to use:
 *   1. Press B  → position 1.0 (one end). Measure the turret angle with a protractor → call it A°.
 *   2. Press A  → position 0.0 (other end). Measure the turret angle → call it B°.
 *   3. range_deg = |A - B|   (degrees of travel across a full 0.0→1.0 position change)
 *        - degrees per 1.0 position = range_deg
 *        - position per 1 degree    = 1.0 / range_deg
 *        - degrees per 0.01 step     = range_deg / 100
 *        - degrees per 0.1  step     = range_deg / 10
 *   Use the fine/coarse steps to land on exact reference marks while measuring.
 *
 * Controls (gamepad1):
 *   Dpad UP / DOWN    = +/- 0.10  (coarse)
 *   Dpad RIGHT / LEFT = +/- 0.01  (fine)
 *   A = 0.0   |   B = 1.0   |   X = 0.5 (center)
 *
 * Hardware config: two servos named "turretServo1" and "turretServo2".
 */
@TeleOp(name = "[TEST] Axon Servo Calibrate", group = "Testers")
public class AxonServoCalibrate extends LinearOpMode {

    private ServoImplEx turretServo1;
    private ServoImplEx turretServo2;

    private static final double COARSE = 0.10;
    private static final double FINE   = 0.01;

    private double pos = 0.5;
    private boolean pUp, pDown, pRight, pLeft, pA, pB, pX;

    @Override
    public void runOpMode() {
        turretServo1 = hardwareMap.get(ServoImplEx.class, "turretServo1");
        turretServo2 = hardwareMap.get(ServoImplEx.class, "turretServo2");
        turretServo1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turretServo2.setPwmRange(new PwmControl.PwmRange(500, 2500));

        apply();
        telemetry.addLine("Ready. UP/DN=+/-0.10  R/L=+/-0.01  A=0.0 B=1.0 X=0.5");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up,    dn = gamepad1.dpad_down;
            boolean ri = gamepad1.dpad_right, le = gamepad1.dpad_left;
            boolean a  = gamepad1.a, b = gamepad1.b, x = gamepad1.x;

            // Rising-edge → one step per press
            if (up && !pUp)    pos += COARSE;
            if (dn && !pDown)  pos -= COARSE;
            if (ri && !pRight) pos += FINE;
            if (le && !pLeft)  pos -= FINE;
            if (a  && !pA)     pos = 0.0;
            if (b  && !pB)     pos = 1.0;
            if (x  && !pX)     pos = 0.5;

            pos = Math.max(0.0, Math.min(1.0, pos));

            pUp = up; pDown = dn; pRight = ri; pLeft = le; pA = a; pB = b; pX = x;

            apply();

            telemetry.addLine("=== AXON SERVO CALIBRATE ===");
            telemetry.addData("Position", "%.3f", pos);
            telemetry.addLine();
            telemetry.addLine("UP/DN = +/-0.10   R/L = +/-0.01");
            telemetry.addLine("A = 0.0   B = 1.0   X = 0.5");
            telemetry.addLine();
            telemetry.addLine("Measure angle at 0.0 and 1.0, then:");
            telemetry.addLine("  range_deg = |angle@1.0 - angle@0.0|");
            telemetry.addLine("  pos per 1 deg = 1.0 / range_deg");
            telemetry.addLine("  deg per 0.01  = range_deg / 100");
            telemetry.update();
        }
    }

    private void apply() {
        turretServo1.setPosition(pos);
        turretServo2.setPosition(pos);
    }
}
