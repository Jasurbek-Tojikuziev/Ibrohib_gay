package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Dual-Axon turret actuator — two servos, 1:1 with the turret, both same direction.
 * Position mode, open-loop. This replaces the old motor + encoder + PIDF actuation:
 * you give it a target angle in degrees and it commands both servos to the matching position.
 *
 * Calibration (measured on the robot):
 *   - center position 0.5  =  0°
 *   - 0.0026667 position per degree   (±90° measured as positions 0.26 / 0.74; new gear ratio)
 *   - right = positive angle  (matches the existing turret convention)
 *
 * Soft limits currently ±125°. Physical travel is now ~±187.5° (full 0.0–1.0 ≈ 375°), so there is
 * extra range available beyond ±125° if you want to use it.
 */
public class TurretServo {

    private final ServoImplEx servo1;
    private final ServoImplEx servo2;

    public static final double CENTER_POS     = 0.5;
    public static final double POS_PER_DEGREE = 0.0026667; // measured: 0.24 / 90° (new gear ratio, ~375° total)
    public static final double MIN_ANGLE      = -180.0;
    public static final double MAX_ANGLE      =  180.0;

    private double targetAngle = 0.0;

    public TurretServo(HardwareMap hw) {
        servo1 = hw.get(ServoImplEx.class, "turretServo1");
        servo2 = hw.get(ServoImplEx.class, "turretServo2");

        // Full Axon pulse window for max travel.
        servo1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servo2.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Both servos rotate the SAME direction → same position value.
        // (If one is mounted mirrored, set servo2.setDirection(Servo.Direction.REVERSE) here.)

        setTargetAngle(0.0); // start centered
    }

    /** Convert a turret angle (deg, right = +) to a clamped servo position 0..1. */
    public static double angleToPosition(double angleDeg) {
        double pos = CENTER_POS + angleDeg * POS_PER_DEGREE;
        return Math.max(0.0, Math.min(1.0, pos));
    }

    /** Command a turret angle in degrees (right = +). Clamped to soft limits, then to 0..1. */
    public void setTargetAngle(double angleDeg) {
        targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angleDeg));
        double pos = angleToPosition(targetAngle);
        servo1.setPosition(pos);
        servo2.setPosition(pos);
    }

    public void   returnToCenter()        { setTargetAngle(0.0); }
    public double getTargetAngle()        { return targetAngle; }
    public double getCommandedPosition()  { return angleToPosition(targetAngle); }
    public boolean isCentered()           { return Math.abs(targetAngle) < 1.0; }
}
