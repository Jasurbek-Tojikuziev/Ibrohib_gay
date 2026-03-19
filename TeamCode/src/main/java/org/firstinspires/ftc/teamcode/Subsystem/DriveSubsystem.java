package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Mecanum drivetrain subsystem.
 *
 * Motor configuration (hardware map names):
 *   "leftFront"  – front-left  (left  mecanum wheel)
 *   "leftRear"   – back-left   (left  mecanum wheel)
 *   "rightFront" – front-right (right mecanum wheel)
 *   "rightRear"  – back-right  (right mecanum wheel)
 *
 * Left-side motors are reversed so that positive power = forward on both sides.
 */
public class DriveSubsystem {

    private final DcMotor leftFront;
    private final DcMotor leftRear;
    private final DcMotor rightFront;
    private final DcMotor rightRear;

    public DriveSubsystem(HardwareMap hardwareMap) {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        // Left-side motors are mounted mirrored, so reverse them
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        // Brake when power is zero to hold position
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Drive the mecanum drivetrain.
     *
     * @param axial   forward/backward power  (+1 = forward, -1 = backward)
     * @param lateral strafe power            (+1 = right,   -1 = left)
     * @param yaw     turning power           (+1 = turn right, -1 = turn left)
     */
    public void drive(double axial, double lateral, double yaw) {
        double fl = axial + lateral + yaw;
        double fr = axial - lateral - yaw;
        double bl = axial - lateral + yaw;
        double br = axial + lateral - yaw;

        // Normalize so no value exceeds 1.0
        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                Math.max(Math.abs(fr),
                Math.max(Math.abs(bl), Math.abs(br)))));

        leftFront.setPower(fl / max);
        rightFront.setPower(fr / max);
        leftRear.setPower(bl / max);
        rightRear.setPower(br / max);
    }

    /** Stop all drive motors. */
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
}
