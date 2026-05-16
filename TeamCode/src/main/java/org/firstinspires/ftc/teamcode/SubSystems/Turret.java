package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

/**
 * Turret facade — preserves the original public API.
 *
 * Internally delegates to three focused classes:
 *   TurretMotor      — motor, PIDF, encoder, manual control
 *   TurretAimer      — auto-aim: odometry-based
 *   TurretBallistics — while-moving physics (null when no Follower available)
 *
 * Robot.java and TurretController.java are unchanged.
 */
public class Turret {

    // Internal components — exposed as public for direct debugging/testing
    public final TurretMotor      motor;
    public final TurretAimer      aimer;
    public final TurretBallistics ballistics; // null in Auto / basic constructors

    // Preset angles (convenience constants)
    public static final double RED_TARGET  = -90.0;
    public static final double BLUE_TARGET =  90.0;
    public static final double ZERO        =   0.0;

    // ── Constructors (same signatures as before) ──────────────────────────────

    /** Auto: odometry via Localizer singleton, resets encoder. */
    public Turret(HardwareMap hardwareMap, Localizer localizer) {
        motor      = new TurretMotor(hardwareMap, true);
        ballistics = null; // Localizer has no velocity API → physics disabled
        aimer      = new TurretAimer(motor, ballistics, localizer);
    }

    /**
     * TeleOp: odometry via Pedro Follower.
     * Does NOT reset encoder — preserves turret position from Auto.
     */
    public Turret(HardwareMap hardwareMap, Follower follower) {
        motor      = new TurretMotor(hardwareMap, false);
        ballistics = new TurretBallistics(follower);
        aimer      = new TurretAimer(motor, ballistics, follower);
    }

    /** Basic motor test: no odometry, no physics. */
    public Turret(HardwareMap hardwareMap) {
        motor      = new TurretMotor(hardwareMap, true);
        ballistics = null;
        aimer      = new TurretAimer(motor);
    }

    // ── Goal / tag setup ─────────────────────────────────────────────────────

    public void setGoalPose(Pose goal)           { aimer.setGoalPose(goal); }
    public Pose getGoalPose()                    { return aimer.getGoalPose(); }
    public void setTagPose(double x, double y)   { aimer.setTagPose(x, y); }
    public boolean hasGoal()                     { return aimer.hasGoal(); }
    public double getDistanceToGoal()            { return aimer.getDistanceToGoal(); }

    // ── Physics results ──────────────────────────────────────────────────────

    public boolean hasPhysicsShot() {
        return ballistics != null && ballistics.isValid();
    }
    public double getCalculatedHoodServo() {
        return ballistics != null ? ballistics.getHoodServo() : 0.0;
    }
    public double getCalculatedFlywheelTicks() {
        return ballistics != null ? ballistics.getFlywheelTicks() : 0.0;
    }
    public double getCalculatedTurretAngleDeg() {
        return ballistics != null ? ballistics.getTurretAngleDeg() : 0.0;
    }
    public double getPhysicsVirtualDistanceInches() {
        return ballistics != null ? ballistics.getVirtualDistanceInches() : 0.0;
    }

    // ── Auto-aim ─────────────────────────────────────────────────────────────

    /** Full auto-aim loop: odometry → physics lead. Drives motor. Call every loop(). */
    public void autoAim()                      { aimer.autoAim(); }

    /** Hold current targetAngle with PIDF (no aim recalculation). */
    public void maintainTarget()               { motor.maintainTarget(); }

    // ── Motor control ────────────────────────────────────────────────────────

    public void   setAutoTarget(double angle)      { motor.setTargetAngle(angle); }
    public void   setTargetAngle(double angle)     { motor.setTargetAngle(angle); }
    public double getTargetAngle()                 { return motor.getTargetAngle(); }
    public double getCurrentAngle()                { return motor.getCurrentAngle(); }
    public double getCurrentPosition()             { return motor.getCurrentPosition(); }
    public double getTargetPosition()              { return motor.getTargetPosition(); }
    public boolean atTarget()                      { return motor.atTarget(); }
    public boolean isCentered()                    { return motor.isCentered(); }
    public double getMotorPower()                  { return motor.getMotorPower(); }
    public void returnToCenter()                   { motor.returnToCenter(); }
    public void syncManualTarget()                 { motor.syncManualTarget(); }
    public void manualControl(double input)        { motor.manualControl(input); }
    public void manualOverride(double direction)   { motor.manualOverride(direction); }
    public void manualRotateRaw(double power)      { motor.manualRotateRaw(power); }
    public void setPIDF(double p, double i, double d, double f) { motor.setPIDF(p, i, d, f); }
    public void setPID(double p, double i, double d)            { motor.setPID(p, i, d); }

    // ── Lifecycle ────────────────────────────────────────────────────────────

    public void resetEncoder() {
        motor.resetEncoder();
        aimer.onEncoderReset(); // resets smoothedTargetAngle + smoothedGoalX/Y
    }

    public void resetGoalSmoothing() { aimer.resetGoalSmoothing(); }

    public void stop() { motor.stop(); }
}
