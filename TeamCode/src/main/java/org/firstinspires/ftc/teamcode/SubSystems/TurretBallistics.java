package org.firstinspires.ftc.teamcode.SubSystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

/**
 * Physics-based while-moving shot calculation.
 *
 * Algorithm (iterative virtual-target):
 *   The ball inherits robot velocity at launch. By aiming at
 *   virtualTarget = realTarget - robotVelocity * flightTime,
 *   the ball lands on the real target despite robot movement.
 *
 * Results are stored after each call to calculate().
 * Check isValid() before using any result.
 */
public class TurretBallistics {

    private final Follower follower;

    // Results from last calculate() call
    private boolean physicsValid          = false;
    private double  calculatedTurretAngleDeg = 0.0;
    private double  calculatedHoodServo      = 0.0;
    private double  calculatedFlywheelTicks  = 0.0;

    // Debug telemetry (public — readable from Robot.java or Dashboard)
    public double debugPhysicsHoodAngleDeg    = 0;
    public double debugPhysicsFlywheelIPS     = 0;
    public double debugPhysicsHorizontalDist  = 0;
    public double debugPhysicsVelMag          = 0;

    public TurretBallistics(Follower follower) {
        this.follower = follower;
    }

    /**
     * Run the iterative virtual-target ballistics calculation.
     * Stores results internally — call isValid() then the getters.
     *
     * @param goalPose  Field-frame basket position (for turret lead angle).
     * @param tagX      Field-frame AprilTag X (physics distance calibrated from tag). Null → use goalPose.
     * @param tagY      Field-frame AprilTag Y.
     */
    public void calculate(Pose goalPose, Double tagX, Double tagY) {
        physicsValid = false;
        if (follower == null || goalPose == null) return;

        Pose   robotPose = follower.getPose();
        Vector robotVel  = follower.getVelocity();

        double robotX    = robotPose.getX();
        double robotY    = robotPose.getY();
        double velFieldX = robotVel.getXComponent(); // field frame, in/s
        double velFieldY = robotVel.getYComponent();
        double velMag    = robotVel.getMagnitude();

        // Physics target: tag position (formulas calibrated from tag), fall back to goal
        double realTargetX = (tagX != null) ? tagX : goalPose.getX();
        double realTargetY = (tagY != null) ? tagY : goalPose.getY();

        double heightY    = ShooterConstants.SCORE_HEIGHT;
        double entryAngle = ShooterConstants.SCORE_ANGLE;

        double virtualTargetX = realTargetX;
        double virtualTargetY = realTargetY;
        double hoodAngle = 0, flywheelSpeed = 0, shotTime = 0;

        // 3 iterations converge shot solution (vel=0 → no shift → identical to stationary)
        for (int pass = 0; pass < 3; pass++) {
            if (pass > 0) {
                virtualTargetX = realTargetX - velFieldX * shotTime;
                virtualTargetY = realTargetY - velFieldY * shotTime;
            }

            double dx            = virtualTargetX - robotX;
            double dy            = virtualTargetY - robotY;
            double horizontalDist = Math.sqrt(dx * dx + dy * dy);

            double x = horizontalDist - ShooterConstants.PASS_THROUGH_POINT_RADIUS;
            if (x <= 0) return; // too close

            hoodAngle = Math.atan(2.0 * heightY / x - Math.tan(entryAngle));
            hoodAngle = Math.max(ShooterConstants.HOOD_MIN_ANGLE,
                                 Math.min(ShooterConstants.HOOD_MAX_ANGLE, hoodAngle));

            double cosHood = Math.cos(hoodAngle);
            double tanHood = Math.tan(hoodAngle);
            double denom   = 2.0 * cosHood * cosHood * (x * tanHood - heightY);
            if (denom <= 0) return; // invalid trajectory

            flywheelSpeed = Math.sqrt(ShooterConstants.GRAVITY * x * x / denom);
            shotTime      = x / (flywheelSpeed * cosHood);
            if (shotTime <= 0) return;
        }

        // Distance for hood/velocity lookup — tag-based (empirical formulas calibrated from tag)
        double tagDx = virtualTargetX - robotX;
        double tagDy = virtualTargetY - robotY;
        debugPhysicsHorizontalDist = Math.sqrt(tagDx * tagDx + tagDy * tagDy);

        // Turret lead angle — goal-based (actual basket, shifted by velocity * shotTime)
        double aimX       = goalPose.getX() - velFieldX * shotTime;
        double aimY       = goalPose.getY() - velFieldY * shotTime;
        double finalDeltaX = aimX - robotX;
        double finalDeltaY = aimY - robotY;

        double heading = robotPose.getHeading();
        double cos     = Math.cos(heading);
        double sin     = Math.sin(heading);

        // Field → robot frame rotation
        double robotDeltaX =  finalDeltaX * cos + finalDeltaY * sin;
        double robotDeltaY = -finalDeltaX * sin + finalDeltaY * cos;

        double turretAngleRad = Math.atan2(robotDeltaY, robotDeltaX);
        double turretAngleDeg = -Math.toDegrees(turretAngleRad);
        turretAngleDeg = Math.max(TurretMotor.MIN_ANGLE, Math.min(TurretMotor.MAX_ANGLE, turretAngleDeg));

        // Store results
        calculatedTurretAngleDeg = turretAngleDeg;
        calculatedHoodServo      = ShooterConstants.getHoodServoFromRadians(hoodAngle);
        calculatedFlywheelTicks  = ShooterConstants.getFlywheelTicksFromVelocity(flywheelSpeed);
        physicsValid             = true;

        // Debug
        debugPhysicsHoodAngleDeg = Math.toDegrees(hoodAngle);
        debugPhysicsFlywheelIPS  = flywheelSpeed;
        debugPhysicsVelMag       = velMag;
    }

    // ── Getters ───────────────────────────────────────────────────────────────

    /** Whether the last calculate() produced a valid result. */
    public boolean isValid()                  { return physicsValid; }

    /** Turret angle with lead compensation (degrees, clamped to motor limits). */
    public double getTurretAngleDeg()         { return calculatedTurretAngleDeg; }

    /** Physics-calculated hood servo position (0.0–1.0). */
    public double getHoodServo()              { return calculatedHoodServo; }

    /** Physics-calculated flywheel speed (motor ticks/sec). */
    public double getFlywheelTicks()          { return calculatedFlywheelTicks; }

    /** Virtual distance to physics-adjusted target (inches) — feed to updateVelocity/updateHood. */
    public double getVirtualDistanceInches()  { return debugPhysicsHorizontalDist; }
}
