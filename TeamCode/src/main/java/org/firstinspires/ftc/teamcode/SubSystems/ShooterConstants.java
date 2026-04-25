package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

/**
 * Physics constants for the Moving Shooter system.
 * All values tunable via FTC Dashboard (@Config).
 *
 * Algorithm (Nishant Vikramaditya, youtube.com/watch?v=oSVNTER_37A):
 *
 * Step 1 - Stationary ballistic trajectory:
 *   hoodAngle     = atan(2*y/x - tan(a))
 *   flywheelSpeed = sqrt(g*x^2 / (2*cos^2(theta)*(x*tan(theta)-y)))
 *
 * Step 2 - Decompose robot velocity into parallel / perpendicular to goal vector:
 *   coordinateTheta      = velTheta - goalTheta
 *   parallelComponent    = -cos(coordinateTheta) * |vel|
 *   perpendicularComponent = sin(coordinateTheta) * |vel|
 *
 * Step 3 - Compensate with Pythagorean theorem:
 *   vz   = flywheelSpeed * sin(hoodAngle)
 *   time = x / (flywheelSpeed * cos(hoodAngle))
 *   ivr  = x / time + parallelComponent
 *   nvr  = sqrt(ivr^2 + perpendicularComponent^2)
 *   ndr  = nvr * time
 *
 * Step 4 - Recalculate launch parameters:
 *   hoodAngle     = atan(vz / nvr)
 *   flywheelSpeed = sqrt(g*ndr^2 / (2*cos^2(theta)*(ndr*tan(theta)-y)))
 *
 * Step 5 - Turret lead angle:
 *   turretVelCompOffset = atan(perpendicularComponent / ivr)
 *   turretAngle = toDegrees(robotHeading - goalTheta + turretVelCompOffset)
 */
@Config
public class ShooterConstants {

    // ---- Goal positions (Pedro Pathing field coordinates, inches) ----
    public static Pose GOAL_POS_RED  = new Pose(130, 135, Math.toRadians(270));
    public static Pose GOAL_POS_BLUE = new Pose(12, 136, Math.toRadians(270));

    // ---- Gravity ----
    /** Gravitational acceleration in inches/s^2 (32.174 ft/s^2 * 12 in/ft). */
    public static double GRAVITY = 32.174 * 12; // 386.088 in/s^2

    // ---- Goal geometry ----
    /** Vertical height of scoring target above the shooter launch point (inches). 106cm - 29cm = 77cm */
    public static double SCORE_HEIGHT = 30.31;

    /** Desired ball entry angle at the goal (radians, NEGATIVE = ball descends into goal). CALIBRATE. */
    public static double SCORE_ANGLE = Math.toRadians(-30.0);

    /** Effective goal radius to reduce horizontal distance (inches). */
    public static double PASS_THROUGH_POINT_RADIUS = 5.0;

    // ---- Hood angle limits (radians) ----
    public static double HOOD_MAX_ANGLE = Math.toRadians(55.0);
    public static double HOOD_MIN_ANGLE = Math.toRadians(10.0);

    // ---- Hood servo conversion ----
    /**
     * Linear mapping: servoPosition = HOOD_SERVO_SCALE * hoodAngleDeg + HOOD_SERVO_OFFSET
     *
     * Same as Decode_Robot: getHoodTicksFromDegrees(degrees) = 0.0226 * degrees - 0.7443
     * CALIBRATE for your robot.
     */
    public static double HOOD_SERVO_SCALE  = 0.04348;  // servo units / degree (0→32°, 1→55°)
    public static double HOOD_SERVO_OFFSET = -1.3913;  // servo units (0→32°, 1→55°)
    public static double HOOD_SERVO_MIN    = 0.0;      // Min servo position
    public static double HOOD_SERVO_MAX    = 0.69;     // Max servo position

    // ---- Flywheel conversion ----
    /**
     * Linear mapping: motorTicks = FLYWHEEL_SCALE * (ballSpeedIPS / 12) + FLYWHEEL_OFFSET
     *
     * Same as Decode_Robot: getFlywheelTicksFromVelocity(v) = 94.501 * v/12 - 187.96 + offset
     * ballSpeedIPS = ball launch speed in inches/sec from physics formula.
     * CALIBRATE for your robot.
     */
    public static double FLYWHEEL_SCALE  = 94.501;  // ticks per (in/s / 12) - CALIBRATE
    public static double FLYWHEEL_OFFSET = -187.96;  // ticks/sec offset - CALIBRATE
    public static double FLYWHEEL_MIN    = 0.0;      // Min motor ticks/sec
    public static double FLYWHEEL_MAX    = 2100.0;   // Max motor ticks/sec

    // ---- Safety / stability ----
    /** Maximum turret velocity-compensation offset (degrees). Hard safety clamp. */
    public static double MAX_COMPENSATION_ANGLE = 30.0;

    // ---- Conversion helpers ----

    /** Convert physics hood angle (degrees) to servo position. */
    public static double getHoodServoFromDegrees(double degrees) {
        double servo = HOOD_SERVO_SCALE * degrees + HOOD_SERVO_OFFSET;
        return Math.max(HOOD_SERVO_MIN, Math.min(HOOD_SERVO_MAX, servo));
    }

    /** Convert physics hood angle (radians) to servo position. */
    public static double getHoodServoFromRadians(double radians) {
        return getHoodServoFromDegrees(Math.toDegrees(radians));
    }

    /** Convert physics ball speed (in/s) to motor ticks/sec. */
    public static double getFlywheelTicksFromVelocity(double velocityIPS) {
        double ticks = FLYWHEEL_SCALE * velocityIPS / 12.0 + FLYWHEEL_OFFSET;
        return Math.max(FLYWHEEL_MIN, Math.min(FLYWHEEL_MAX, ticks));
    }
}
