package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.TurretServo;

/**
 * Drives the dual-Axon turret BY ANGLE (degrees) to verify the calibration, direction, and limits
 * before wiring it into auto-aim.
 *
 * Controls (gamepad1):
 *   Dpad UP / DOWN    = +/- 10°
 *   Dpad RIGHT / LEFT = +/- 1°
 *   A            = center (0°)
 *   Right bumper = full right (MAX_ANGLE)
 *   Left bumper  = full left  (MIN_ANGLE)
 *
 * Check: command +90 → turret should point 90° to the RIGHT; -90 → 90° LEFT; 0 → straight/center.
 */
@TeleOp(name = "[TEST] Turret Servo Degrees", group = "Testers")
public class TurretServoDegreesTester extends LinearOpMode {

    private TurretServo turret;
    private double angle = 0.0;
    private boolean pUp, pDown, pRight, pLeft, pA, pRB, pLB;

    @Override
    public void runOpMode() {
        turret = new TurretServo(hardwareMap);

        telemetry.addLine("Ready — turret centered (0°)");
        telemetry.addLine("Dpad UP/DN = +/-10°, R/L = +/-1°, A = center, bumpers = full ends");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up,    dn = gamepad1.dpad_down;
            boolean ri = gamepad1.dpad_right, le = gamepad1.dpad_left;
            boolean a  = gamepad1.a;
            boolean rb = gamepad1.right_bumper, lb = gamepad1.left_bumper;

            if (up && !pUp)    angle += 10.0;
            if (dn && !pDown)  angle -= 10.0;
            if (ri && !pRight) angle += 1.0;
            if (le && !pLeft)  angle -= 1.0;
            if (a  && !pA)     angle = 0.0;
            if (rb && !pRB)    angle = TurretServo.MAX_ANGLE;
            if (lb && !pLB)    angle = TurretServo.MIN_ANGLE;

            pUp = up; pDown = dn; pRight = ri; pLeft = le; pA = a; pRB = rb; pLB = lb;

            turret.setTargetAngle(angle);   // clamps internally
            angle = turret.getTargetAngle(); // keep our local copy clamped too

            telemetry.addLine("=== TURRET SERVO (degrees) ===");
            telemetry.addData("Target angle", "%.1f°  (right = +)", turret.getTargetAngle());
            telemetry.addData("Servo position", "%.3f", turret.getCommandedPosition());
            telemetry.addLine();
            telemetry.addLine("UP/DN = +/-10°   R/L = +/-1°   A = center");
            telemetry.addLine("RB = full right   LB = full left");
            telemetry.update();
        }
    }
}
