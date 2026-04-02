package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Shooter subsystem.
 *
 * Motor configuration (hardware map names):
 *   "shooterMotor1" – shooter motor with encoder (master)
 *   "shooterMotor2" – shooter motor without encoder (slave, synced to master)
 *
 * Servo configuration (hardware map names):
 *   "shooterStop" – gate servo        (closed = 0.0,  open = 0.29)
 *   "intakeStop"  – compression servo (closed = 1.0, open = 0.87)
 *
 * PIDF coefficients (RUN_USING_ENCODER) should be tuned for your specific motor.
 * Default target velocity: 1500 ticks/sec.
 */
public class ShooterSubsystem {

    // Tunable via FTC Dashboard
    public static double PIDF_P         = 130;
    public static double PIDF_I         = 0.0;
    public static double PIDF_D         = 0.0;
    public static double PIDF_F         = 13.4;
    public static double TARGET_VELOCITY = 1000.0; // ticks per second
    public static double HOOD_POSITION   = 0.3;    // servo position [0.0, 1.0]
    public static double HOOD_DROP_TIME   = 0.3;   // seconds into shoot sequence for first hood drop
    public static double HOOD_DROP_TIME_2 = 0.45;  // seconds into shoot sequence for second hood drop
    public static double HOOD_DROP_1      = 0.12;  // first drop amount
    public static double HOOD_DROP_2      = 0.2;   // second drop amount (cumulative from base)

    public static final double GATE_CLOSED        = 1.0;
    public static final double GATE_OPEN          = 0.74;
    public static final double COMPRESSION_CLOSED = 1.0;
    public static final double COMPRESSION_OPEN   = 0.87;

    private final DcMotorEx shooterMaster; // has encoder – velocity PIDF
    private final DcMotorEx shooterSlave;  // no encoder  – mirrors master

    private final Servo     gateServo;
    private final Servo     compressionServo;
    private final Servo     hoodServo;
    private final Telemetry telemetry;

    private double lastP = PIDF_P, lastF = PIDF_F;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        shooterMaster = hardwareMap.get(DcMotorEx.class, "shooterMotor1"); // encoder plugged here
        shooterSlave  = hardwareMap.get(DcMotorEx.class, "shooterMotor2");

        shooterMaster.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterSlave.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMaster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterSlave.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Master uses encoder for closed-loop velocity control
        shooterMaster.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMaster.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMaster.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(PIDF_P, 0, 0, PIDF_F)
        );

        // Slave has no encoder – driven open-loop, power synced to master each loop
        shooterSlave.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gateServo        = hardwareMap.get(Servo.class, "shooterStop");
        compressionServo = hardwareMap.get(Servo.class, "intakeStop");

        Servo tempHood = null;
        try { tempHood = hardwareMap.get(Servo.class, "shooterHood"); } catch (Exception ignored) {}
        hoodServo = tempHood;

        // Park servos at default positions
        gateServo.setPosition(GATE_CLOSED);
        compressionServo.setPosition(COMPRESSION_CLOSED);
    }

    /** Reverse both shooter motor directions. Call after construction when wiring is opposite. */
    public void reverseMotors() {
        shooterMaster.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterSlave.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /** Start shooter motors at target velocity. Call once on op-mode start. */
    public void startMotors() {
        setVelocity(TARGET_VELOCITY);
    }

    /** Stop both shooter motors. */
    public void stopMotors() {
        shooterMaster.setVelocity(0);
        shooterSlave.setPower(0);
    }

    /**
     * Sync slave power to whatever the master's PIDF controller is outputting.
     * This prevents slave from overpowering master, keeping both at the same speed.
     * Must be called every loop() while motors are running.
     */
    public void updateMotorSync() {
        double currentVelocity = shooterMaster.getVelocity();
        double error           = TARGET_VELOCITY - currentVelocity;
        double computedPower   = (PIDF_F * TARGET_VELOCITY + PIDF_P * error) / 32767.0;

        shooterSlave.setPower(computedPower);

        telemetry.addData("motor1 velocity", currentVelocity);
        telemetry.addData("motor2 velocity", shooterSlave.getVelocity());
        telemetry.addData("computed power",  computedPower);
        telemetry.addData("motor2 power",    shooterSlave.getPower());
    }

    /** Returns master encoder velocity in ticks/sec. */
    public double getMasterVelocity() {
        return shooterMaster.getVelocity();
    }

    /** Returns the current power being sent to the slave motor. */
    public double getSlavePower() {
        return shooterSlave.getPower();
    }

    /** Set master motor to a specific velocity (ticks/sec). */
    public void setVelocity(double velocity) {
        shooterMaster.setVelocity(velocity);
    }

    /**
     * Re-applies PIDF coefficients if they were changed via FTC Dashboard.
     * Call every loop().
     */
    public void updatePIDF() {
        if (PIDF_P != lastP || PIDF_F != lastF) {
            shooterMaster.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(PIDF_P, 0, 0, PIDF_F)
            );
            lastP = PIDF_P;
            lastF = PIDF_F;
        }
    }

    public void setGatePosition(double position) {
        gateServo.setPosition(position);
    }

    public void setCompressionPosition(double position) {
        compressionServo.setPosition(position);
    }

    public void setHoodPosition(double position) {
        if (hoodServo != null) hoodServo.setPosition(position);
    }
}

