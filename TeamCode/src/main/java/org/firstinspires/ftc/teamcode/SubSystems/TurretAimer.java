package org.firstinspires.ftc.teamcode.SubSystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

/**
 * Turret auto-aim: odometry-based targeting with optional physics lead compensation.
 */
public class TurretAimer {

    private final TurretMotor      motor;
    private final TurretBallistics ballistics;
    private final Follower         follower;
    private final Localizer        localizer;

    private Pose    goalPose = null;
    private Double  goalX    = null;
    private Double  goalY    = null;
    private Double  tagX     = null;
    private Double  tagY     = null;

    private double smoothedTargetAngle  = 0.0;
    private double savedCloseRangeAngle = Double.NaN;
    private double autoAimOffset        = 0.0;

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
     * Full auto-aim: odometry sets base angle, physics applies lead compensation.
     * Runs PIDF and drives motor each call — invoke every loop().
     */
    public void setAutoAimOffset(double offset) {
        this.autoAimOffset = offset;
    }

    public double getAutoAimOffset() { return autoAimOffset; }

    public double getCalculatedTargetAngle() {
        return calculateTargetAngle();
    }

    public double getSmoothedTargetAngle() { return smoothedTargetAngle; }

    public void autoAim() {
        if (hasGoal() && (follower != null || localizer != null)) {
            double angle = calculateTargetAngle() + autoAimOffset;
            angle = Math.max(TurretMotor.MIN_ANGLE, Math.min(TurretMotor.MAX_ANGLE, angle));
            smoothedTargetAngle = angle;
            motor.setTargetAngle(angle);
        }

        if (ballistics != null) {
            ballistics.calculate(goalPose, tagX, tagY);
            if (ballistics.isValid()) {
                double angle = ballistics.getTurretAngleDeg() + autoAimOffset;
                angle = Math.max(TurretMotor.MIN_ANGLE, Math.min(TurretMotor.MAX_ANGLE, angle));
                motor.setTargetAngle(angle);
                smoothedTargetAngle = motor.getTargetAngle();
            }
        }

        double power = motor.calculatePIDF(motor.getTargetAngle(), motor.getCurrentAngle());
        motor.applyPower(power);
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

        double h   = p.getHeading();
        double cos = Math.cos(h);
        double sin = Math.sin(h);
        double rX  =  fieldDX * cos + fieldDY * sin;
        double rY  = -fieldDX * sin + fieldDY * cos;

        double angle = -Math.toDegrees(Math.atan2(rY, rX));
        return Math.max(TurretMotor.MIN_ANGLE, Math.min(TurretMotor.MAX_ANGLE, angle));
    }

    // ── Autonomous aim ───────────────────────────────────────────────────────

    /** Auto: holds targetAngle with PIDF. */
    public void maintainWithVisionCorrection() {
        double power = motor.calculatePIDF(motor.getTargetAngle(), motor.getCurrentAngle());
        motor.applyPower(power);
    }

    // ── Misc ─────────────────────────────────────────────────────────────────

    public boolean isTracking() {
        return hasGoal();
    }

    public void onEncoderReset() {
        smoothedTargetAngle  = 0.0;
        savedCloseRangeAngle = Double.NaN;
        autoAimOffset        = 0.0;
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
