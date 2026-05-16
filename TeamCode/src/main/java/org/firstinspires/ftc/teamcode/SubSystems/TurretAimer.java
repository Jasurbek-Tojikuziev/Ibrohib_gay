package org.firstinspires.ftc.teamcode.SubSystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

/**
 * Turret auto-aim: odometry-based targeting.
 * Applies physics lead angle from TurretBallistics when available.
 */
public class TurretAimer {

    private final TurretMotor      motor;
    private final TurretBallistics ballistics; // null if no follower
    private final Follower         follower;   // null in Auto/test
    private final Localizer        localizer;  // null in TeleOp/test

    // Goal / tag coordinates
    private Pose    goalPose = null;
    private Double  goalX    = null;
    private Double  goalY    = null;
    private Double  tagX     = null;
    private Double  tagY     = null;

    // EMA smoothing state
    private double smoothedTargetAngle  = 0.0;
    private double savedCloseRangeAngle = Double.NaN;
    private double smoothedGoalX        = Double.NaN;
    private double smoothedGoalY        = Double.NaN;

    private static final double GOAL_POSE_SMOOTHING    = 0.1;  // ~200 ms at 50 Hz

    // ── Constructors ─────────────────────────────────────────────────────────

    /** TeleOp: Pedro Follower for odometry. Physics ballistics enabled. */
    public TurretAimer(TurretMotor motor, TurretBallistics ballistics, Follower follower) {
        this.motor      = motor;
        this.ballistics = ballistics;
        this.follower   = follower;
        this.localizer  = null;
        this.smoothedTargetAngle = motor.getCurrentAngle();
    }

    /** Auto: Localizer for odometry. No physics (Localizer has no velocity API). */
    public TurretAimer(TurretMotor motor, TurretBallistics ballistics, Localizer localizer) {
        this.motor      = motor;
        this.ballistics = ballistics;
        this.follower   = null;
        this.localizer  = localizer;
        this.smoothedTargetAngle = motor.getCurrentAngle();
    }

    /** Basic test: no odometry, no physics. */
    public TurretAimer(TurretMotor motor) {
        this.motor      = motor;
        this.ballistics = null;
        this.follower   = null;
        this.localizer  = null;
    }

    // ── Goal setup ───────────────────────────────────────────────────────────

    public void setGoalPose(Pose goal) {
        this.goalPose = goal;
        if (goal != null) {
            this.goalX = goal.getX();
            this.goalY = goal.getY();
        }
    }

    public Pose getGoalPose() { return goalPose; }

    public void setTagPose(double x, double y) {
        this.tagX = x;
        this.tagY = y;
    }

    public boolean hasGoal() {
        return goalPose != null || (goalX != null && goalY != null);
    }

    public double getDistanceToGoal() {
        if (!hasGoal()) return 0.0;
        Pose p = getCurrentPose();
        if (p == null) return 0.0;
        double tx = (goalPose != null) ? goalPose.getX() : goalX;
        double ty = (goalPose != null) ? goalPose.getY() : goalY;
        double dx = tx - p.getX();
        double dy = ty - p.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    // ── Auto-aim (main entry) ────────────────────────────────────────────────

    /**
     * Full auto-aim: vision or odometry sets base angle, physics applies lead compensation.
     * Runs PIDF and drives motor each call — invoke every loop().
     */
    public void autoAim() {
        autoAimLegacy(); // sets motor targetAngle via odometry

        // Physics overrides with lead-compensated angle (at vel=0 → same as odometry)
        if (ballistics != null) {
            ballistics.calculate(goalPose, tagX, tagY);
            if (ballistics.isValid()) {
                motor.setTargetAngle(ballistics.getTurretAngleDeg());
                smoothedTargetAngle = motor.getTargetAngle();
            }
        }

        double power = motor.calculatePIDF(motor.getTargetAngle(), motor.getCurrentAngle());
        motor.applyPower(power);
    }

    /** Odometry-based targeting. Updates motor.targetAngle. */
    private void autoAimLegacy() {
        if (hasGoal() && (follower != null || localizer != null)) {
            double angle = calculateTargetAngle();
            smoothedTargetAngle = angle;
            motor.setTargetAngle(angle);
        }
    }

    /** Field coordinates → robot-frame turret angle via heading rotation. */
    private double calculateTargetAngle() {
        if (!hasGoal()) return 0.0;
        Pose p = getCurrentPose();
        if (p == null) return 0.0;

        double targetX = (goalPose != null) ? goalPose.getX() : goalX;
        double targetY = (goalPose != null) ? goalPose.getY() : goalY;

        double fieldDX = targetX - p.getX();
        double fieldDY = targetY - p.getY();
        double distance = Math.sqrt(fieldDX * fieldDX + fieldDY * fieldDY);

        // Save angle at 5-9" — reliable range before odometry gets noisy up close
        if (distance >= 5.0 && distance <= 9.0) {
            double h   = p.getHeading();
            double rX  =  fieldDX * Math.cos(h) + fieldDY * Math.sin(h);
            double rY  = -fieldDX * Math.sin(h) + fieldDY * Math.cos(h);
            double ang = -Math.toDegrees(Math.atan2(rY, rX));
            savedCloseRangeAngle = Math.max(TurretMotor.MIN_ANGLE, Math.min(TurretMotor.MAX_ANGLE, ang));
        }

        // < 5": odometry unreliable — use saved angle
        if (distance < 5.0) {
            return Double.isNaN(savedCloseRangeAngle) ? motor.getTargetAngle() : savedCloseRangeAngle;
        }

        // Field → robot frame
        double h   = p.getHeading();
        double cos = Math.cos(h);
        double sin = Math.sin(h);
        double rX  =  fieldDX * cos + fieldDY * sin;
        double rY  = -fieldDX * sin + fieldDY * cos;

        double angle = -Math.toDegrees(Math.atan2(rY, rX));
        return Math.max(TurretMotor.MIN_ANGLE, Math.min(TurretMotor.MAX_ANGLE, angle));
    }

    // ── Misc ─────────────────────────────────────────────────────────────────

    public boolean isTracking() {
        return hasGoal();
    }

    /** Reset goal EMA smoothing (call after sudden robot pose reset). */
    public void resetGoalSmoothing() {
        if (goalPose != null) {
            smoothedGoalX = goalPose.getX();
            smoothedGoalY = goalPose.getY();
        } else {
            smoothedGoalX = Double.NaN;
            smoothedGoalY = Double.NaN;
        }
    }

    /**
     * Full smoothing reset after encoder reset.
     * Mirrors original Turret.resetEncoder() behaviour:
     * smoothedTargetAngle → 0, smoothedGoalX/Y → NaN.
     */
    public void onEncoderReset() {
        smoothedTargetAngle = 0.0;
        smoothedGoalX       = Double.NaN;
        smoothedGoalY       = Double.NaN;
    }

    // ── Internal helpers ─────────────────────────────────────────────────────

    private Pose getCurrentPose() {
        if (follower  != null) return follower.getPose();
        if (localizer != null) {
            return new Pose(localizer.getX(), localizer.getY(),
                            Math.toRadians(localizer.getHeading()));
        }
        return null;
    }
}
