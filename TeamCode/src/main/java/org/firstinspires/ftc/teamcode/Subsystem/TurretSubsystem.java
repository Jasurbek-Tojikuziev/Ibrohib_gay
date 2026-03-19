package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * TurretSubsystem controls the turret rotation motor using a manual PID loop.
 *
 * IMPORTANT: Manually aim turret forward (toward robot front) before starting
 * any OpMode. The encoder is zeroed at init — wherever the turret physically
 * points at that moment becomes angle 0 degrees.
 *
 * Hardware: GoBilda 435 RPM motor (5203-2402-0014) with 3:1 external gear reduction.
 *   Motor output ticks/rev : 384.5 (theoretical) — physical measurement = 3 ticks/degree
 *   External reduction     : 3:1  (35-tooth → 105-tooth gear)
 *   Turret ticks/rev       : 1080.0  (3 ticks/deg × 360 deg, physically measured)
 *   Ticks per degree       : 3.0    (physically measured on robot)
 *
 * PID coefficients kP, kI, kD are tunable live via FTC Dashboard without
 * redeploying code. Open Dashboard in a browser while connected to the robot.
 */
@Config
public class TurretSubsystem {

    // ── Hardware map device name ─────────────────────────────────────────────
    private static final String DEVICE_NAME = "turretMotor";

    // ── Mechanical constants ─────────────────────────────────────────────────
    static final double TICKS_PER_REV    = 1080.0;
    static final double TICKS_PER_DEGREE =    3.15;

    // ── Soft limits (degrees). Clamped in setTargetAngle(). ─────────────────
    private static final double TURRET_MIN_ANGLE = -135.0;
    private static final double TURRET_MAX_ANGLE =  135.0;

    // ── PID coefficients — public static so FTC Dashboard @Config can edit them live ──
    public static double kP = 0.035;
    public static double kI = 0.0;
    public static double kD = 0.001;
    public static double kF = 0.0;

    // ── Motor output clamp ───────────────────────────────────────────────────
    private static final double MAX_POWER     =  1.0;
    public static double INTEGRAL_MAX  =  1.0;

    // ── Runtime state ────────────────────────────────────────────────────────
    private final DcMotorEx motor;
    private final FtcDashboard dashboard;

    private double targetAngle    = 0.0; // degrees
    private double integral       = 0.0;
    private double lastError      = 0.0;
    private long   lastUpdateTime;       // nanoseconds, set in constructor

    /**
     * Initializes the turret motor, resets the encoder, and prepares the PID state.
     * Robot must be stationary and turret must be pointed forward when this runs.
     *
     * @param hardwareMap the OpMode's HardwareMap
     */
    public TurretSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, DEVICE_NAME);

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Zero the encoder. Current physical position becomes angle 0 (robot-forward).
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Manual PID drives this motor — RUN_WITHOUT_ENCODER lets us set raw power.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dashboard       = FtcDashboard.getInstance();
        lastUpdateTime  = System.nanoTime();
    }

    /**
     * Sets the desired turret angle in degrees relative to robot-forward (encoder zero).
     * Clamped to [TURRET_MIN_ANGLE, TURRET_MAX_ANGLE] to protect mechanical stops.
     *
     * @param degrees target angle; positive = counterclockwise (verify direction on robot)
     */
    public void setTargetAngle(double degrees) {
        targetAngle = Math.max(TURRET_MIN_ANGLE, Math.min(TURRET_MAX_ANGLE, degrees));
    }

    /**
     * Runs one PID iteration. Must be called every loop().
     *
     * Calculates dt using System.nanoTime() so PID behaves consistently
     * regardless of loop speed. Sends live data to FTC Dashboard.
     */
    public void update() {
        // ── Delta time ───────────────────────────────────────────────────────
        long   currentTime = System.nanoTime();
        double dt          = (currentTime - lastUpdateTime) / 1e9; // seconds
        lastUpdateTime     = currentTime;

        // Guard against dt = 0 on first call or after a stall
        if (dt <= 0) return;

        // ── PID math ─────────────────────────────────────────────────────────
        double currentAngle = getCurrentAngle();
        double error        = targetAngle - currentAngle;

        integral += error * dt;
        integral  = Math.max(-INTEGRAL_MAX, Math.min(INTEGRAL_MAX, integral));

        double derivative = (error - lastError) / dt;
        lastError         = error;

        double output = (kP * error)
                      + (kI * integral)
                      + (kD * derivative)
                      + (kF * targetAngle);
        output = Math.max(-MAX_POWER, Math.min(MAX_POWER, output));

        motor.setPower(output);

        // ── FTC Dashboard telemetry ───────────────────────────────────────────
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Turret Current Angle (deg)", currentAngle);
        packet.put("Turret Target Angle  (deg)", targetAngle);
        packet.put("Turret PID Error     (deg)", error);
        packet.put("Turret Motor Power",         output);
        packet.put("Turret Integral",            integral);
        packet.put("kF term",                    kF * targetAngle);
        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * Returns the current turret angle in degrees, calculated from the motor encoder.
     * 0 degrees = robot-forward (encoder zero set at init).
     */
    public double getCurrentAngle() {
        return motor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    /**
     * Returns the currently stored target angle in degrees.
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Re-zeros the encoder at the current physical position and clears PID state.
     * Call this after manually rotating the turret to its forward (0°) position.
     */
    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngle    = 0.0;
        integral       = 0.0;
        lastError      = 0.0;
        lastUpdateTime = System.nanoTime();
    }
}
