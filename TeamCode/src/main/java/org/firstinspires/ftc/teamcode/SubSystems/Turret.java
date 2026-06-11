package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

/**
 * Turret facade — preserves the original public API.
 *
 * Internally delegates to three focused classes:
 *   TurretMotor      — motor, PIDF, encoder, manual control
 *   TurretAimer      — auto-aim: odometry-based targeting
 *   TurretBallistics — while-moving physics (null when no Follower available)
 */
public class Turret {

    public final TurretMotor      motor;
    public final TurretAimer      aimer;
    public final TurretBallistics ballistics;

    public static final double RED_TARGET  = -90.0;
    public static final double BLUE_TARGET =  90.0;
    public static final double ZERO        =   0.0;

    // ── Constructors ─────────────────────────────────────────────────────────

    /** Auto: odometry via Localizer singleton, resets encoder. */
    public Turret(HardwareMap hardwareMap, Localizer localizer) {
        motor      = new TurretMotor(hardwareMap, true);
        ballistics = null;
        aimer      = new TurretAimer(motor, ballistics, localizer);
    }

    /**
     * TeleOp: odometry via Pedro Follower + Limelight vision.
     * Vision overrides odometry when tag is visible.
     * Does NOT reset encoder — preserves turret position from Auto.
     */
    public Turret(HardwareMap hardwareMap, Follower follower, Vision vision) {
        motor      = new TurretMotor(hardwareMap, false);
        ballistics = new TurretBallistics(follower);
        aimer      = new TurretAimer(motor, ballistics, follower, vision);
    }

    /**
     * TeleOp: odometry via Pedro Follower, no vision.
     * Does NOT reset encoder — preserves turret position from Auto.
     */
    public Turret(HardwareMap hardwareMap, Follower follower) {
        motor      = new TurretMotor(hardwareMap, false);
        ballistics = new TurretBallistics(follower);
        aimer      = new TurretAimer(motor, ballistics, follower);
    }

    /**
     * Vision-only aim test: turret tracks the tag via Limelight tx + parallax.
     * No odometry, no physics, no flywheel. Resets encoder at construction.
     */
    public Turret(HardwareMap hardwareMap, Vision vision) {
        motor      = new TurretMotor(hardwareMap, true);
        ballistics = null;
        aimer      = new TurretAimer(motor, vision);
    }

    /** Basic motor test: no odometry, no physics. */
    public Turret(HardwareMap hardwareMap) {
        motor      = new TurretMotor(hardwareMap, true);
        ballistics = null;
        aimer      = new TurretAimer(motor);
    }

    // ── Goal / tag setup ─────────────────────────────────────────────────────

    public void setGoalPose(Pose goal)           { aimer.setGoalPose(goal); }
    public void setAimTrimAlliance(boolean isRed){ aimer.setAimTrimAlliance(isRed); }
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
    public void setAutoAimOffset(double offset){ aimer.setAutoAimOffset(offset); }
    /** (vision-only test mode) set aim direction, chase lead cap, and parallax sign (+1/-1/0). */
    public void setVisionTuning(double sign, double maxLead, double parallaxSign) {
        aimer.setVisionTuning(sign, maxLead, parallaxSign);
    }

    /** Configure camera relocalization: sign (+1/-1), threshold (deg), settle limits (deg/s, in/s). */
    public void setRelocalization(double sign, double thresholdDeg, double settleAngVel, double settleTransVel) {
        aimer.setRelocalization(sign, thresholdDeg, settleAngVel, settleTransVel);
    }
    /** Clear the camera drift correction (on a manual position reset). */
    public void clearRelocalization() { aimer.clearRelocalization(); }
    public double  getAutoAimOffset()  { return aimer.getAutoAimOffset(); }
    public double  getRelocOffset()    { return aimer.getRelocOffset(); }
    public double  getRelocResidual()  { return aimer.getRelocResidual(); }
    public boolean isSettledForReloc() { return aimer.isSettledForReloc(); }
    public double  getAngularVel()     { return aimer.getAngularVelDeg(); }
    public double  getTranslationVel() { return aimer.getTranslationVel(); }
    public boolean didReloc()          { return aimer.didReloc(); }
    public double getCalculatedTargetAngle()   { return aimer.getCalculatedTargetAngle(); }

    /** Hold current targetAngle with PIDF (no aim recalculation). */
    public void maintainTarget()               { motor.maintainTarget(); }

    /** Auto: hold hardcoded angle with PIDF. */
    public void maintainWithVisionCorrection() { aimer.maintainWithVisionCorrection(); }

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
    public void manualMove(double power)           { motor.manualMove(power); }
    public void manualOverride(double direction)   { motor.manualOverride(direction); }
    public void manualRotateRaw(double power)      { motor.manualRotateRaw(power); }
    public void setPIDF(double p, double i, double d, double f) { motor.setPIDF(p, i, d, f); }
    public void setPID(double p, double i, double d)            { motor.setPID(p, i, d); }

    // ── Tracking ─────────────────────────────────────────────────────────────

    public boolean isTracking()      { return aimer.isTracking(); }
    public boolean hasVisionTarget() { return aimer.hasVisionTarget(); }
    public Vision  getVision()       { return aimer.getVision(); }

    // ── Lifecycle ────────────────────────────────────────────────────────────

    public void resetEncoder() {
        motor.resetEncoder();
        aimer.onEncoderReset();
    }

    public void stop() { motor.stop(); }
}
