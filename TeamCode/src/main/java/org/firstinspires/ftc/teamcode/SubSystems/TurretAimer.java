package org.firstinspires.ftc.teamcode.SubSystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * Turret auto-aim: vision (priority) → odometry fallback.
 * Applies physics lead angle from TurretBallistics when available.
 */
public class TurretAimer {

    private final TurretMotor      motor;
    private final TurretBallistics ballistics; // null if no follower
    private final Vision           vision;
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
    private int    visionConfirmationFrames = 0;

    // Tunable constants
    public static double VISION_YAW_OFFSET             = 1.5;  // degrees, compensates camera offset
    public static double VISION_CORRECTION_THRESHOLD   = 8.0;  // degrees
    public static int    VISION_CONFIRMATION_REQUIRED  = 5;    // consecutive frames
    private static final double SMOOTHING_FACTOR       = 0.25;
    private static final double GOAL_POSE_SMOOTHING    = 0.1;  // ~200 ms at 50 Hz

    // ── Constructors ─────────────────────────────────────────────────────────

    /** TeleOp: Pedro Follower for odometry. Physics ballistics enabled. */
    public TurretAimer(TurretMotor motor, TurretBallistics ballistics,
                       Vision vision, Follower follower) {
        this.motor      = motor;
        this.ballistics = ballistics;
        this.vision     = vision;
        this.follower   = follower;
        this.localizer  = null;
        this.smoothedTargetAngle = motor.getCurrentAngle();
    }

    /** Auto: Localizer for odometry. No physics (Localizer has no velocity API). */
    public TurretAimer(TurretMotor motor, TurretBallistics ballistics,
                       Vision vision, Localizer localizer) {
        this.motor      = motor;
        this.ballistics = ballistics;
        this.vision     = vision;
        this.follower   = null;
        this.localizer  = localizer;
        this.smoothedTargetAngle = motor.getCurrentAngle();
    }

    /** Vision-only or basic test (no odometry, no physics). */
    public TurretAimer(TurretMotor motor, Vision vision) {
        this.motor      = motor;
        this.ballistics = null;
        this.vision     = vision;
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
        autoAimLegacy(); // sets motor targetAngle via vision or odometry

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

    /** Vision (priority 1) → odometry (priority 2) targeting. Updates motor.targetAngle. */
    private void autoAimLegacy() {
        if (vision != null && vision.hasTargetTag()) {
            double tx = vision.getTargetYaw() + VISION_YAW_OFFSET;
            if (!Double.isNaN(tx)) {
                double currentAngle = motor.getCurrentAngle();
                double newTarget    = currentAngle + tx;
                newTarget = Math.max(TurretMotor.MIN_ANGLE, Math.min(TurretMotor.MAX_ANGLE, newTarget));

                // EMA: closer target = faster response, farther = smoother
                double dist = getDistanceToGoal();
                double emaFactor = 0.90 - 0.25 * Math.min(1.0, Math.max(0.0, (dist - 40.0) / 55.0));
                smoothedTargetAngle = smoothedTargetAngle + emaFactor * (newTarget - smoothedTargetAngle);
                smoothedTargetAngle = Math.max(TurretMotor.MIN_ANGLE, Math.min(TurretMotor.MAX_ANGLE, smoothedTargetAngle));
                motor.setTargetAngle(smoothedTargetAngle);
            }
        } else if (hasGoal() && (follower != null || localizer != null)) {
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

    // ── Autonomous aim ───────────────────────────────────────────────────────

    /**
     * Auto: holds hardcoded targetAngle, applies small vision correction once settled.
     * Vision loss after correction → holds corrected angle (does not roll back).
     */
    public void maintainWithVisionCorrection() {
        double currentAngle = motor.getCurrentAngle();

        if (vision != null && vision.hasTargetTag()) {
            visionConfirmationFrames++;
        } else {
            visionConfirmationFrames = 0;
        }

        // Resync EMA buffer if it drifted from motor target (e.g. after setTargetAngle())
        if (Math.abs(smoothedTargetAngle - motor.getTargetAngle()) > 5.0) {
            smoothedTargetAngle = motor.getTargetAngle();
        }

        // Only correct once turret is settled AND camera has confirmed N consecutive frames
        if (motor.atTarget() && visionConfirmationFrames >= VISION_CONFIRMATION_REQUIRED) {
            double tx = vision.getTargetYaw() + VISION_YAW_OFFSET;
            if (!Double.isNaN(tx)) {
                double newTarget = currentAngle + tx;
                newTarget = Math.max(TurretMotor.MIN_ANGLE, Math.min(TurretMotor.MAX_ANGLE, newTarget));
                smoothedTargetAngle = smoothedTargetAngle + SMOOTHING_FACTOR * (newTarget - smoothedTargetAngle);
                smoothedTargetAngle = Math.max(TurretMotor.MIN_ANGLE, Math.min(TurretMotor.MAX_ANGLE, smoothedTargetAngle));
                motor.setTargetAngle(smoothedTargetAngle);
            }
        }

        double power = motor.calculatePIDF(motor.getTargetAngle(), currentAngle);
        motor.applyPower(power);
    }

    // ── Vision ───────────────────────────────────────────────────────────────

    /**
     * Updates goalPose from a live tag observation (camera frame → field frame).
     * Camera is mounted on turret → two rotations needed: camera→robot, robot→field.
     * EMA-filtered to suppress Limelight noise.
     */
    public void updateGoalFromVision(Pose3D tagCameraPose) {
        if (tagCameraPose == null) return;
        if (follower == null && localizer == null) return;

        Position pos = tagCameraPose.getPosition();
        if (pos == null) return;

        double cameraX = pos.x; // lateral: right = + (meters)
        double cameraZ = pos.z; // forward from camera (meters)

        // Apply same distance correction as Vision.getTargetDistance()
        double rawInches = Math.sqrt(cameraX * cameraX + cameraZ * cameraZ) * 39.3701;
        double scaleFactor = (rawInches > 0)
                ? ((rawInches - Vision.DISTANCE_OFFSET) * Vision.DISTANCE_SCALE) / rawInches
                : 1.0;
        cameraX *= scaleFactor;
        cameraZ *= scaleFactor;

        // Camera → robot frame (turret angle = rotation from robot forward)
        double alpha     = Math.toRadians(motor.getCurrentAngle());
        double robotRight =  cameraX * Math.cos(alpha) + cameraZ * Math.sin(alpha);
        double robotFwd   = -cameraX * Math.sin(alpha) + cameraZ * Math.cos(alpha);

        // Robot frame (meters) → inches
        double tagFwdIn   = robotFwd   * 39.3701;
        double tagRightIn = robotRight * 39.3701;

        // Robot → field frame
        double robotX, robotY, heading;
        if (follower != null) {
            Pose pose = follower.getPose();
            robotX  = pose.getX();
            robotY  = pose.getY();
            heading = pose.getHeading();
        } else {
            robotX  = localizer.getX();
            robotY  = localizer.getY();
            heading = Math.toRadians(localizer.getHeading());
        }

        double tagFieldX = robotX + tagFwdIn * Math.cos(heading) + tagRightIn * Math.sin(heading);
        double tagFieldY = robotY + tagFwdIn * Math.sin(heading) - tagRightIn * Math.cos(heading);

        if (Double.isNaN(smoothedGoalX)) {
            smoothedGoalX = tagFieldX;
            smoothedGoalY = tagFieldY;
        } else {
            smoothedGoalX = smoothedGoalX + GOAL_POSE_SMOOTHING * (tagFieldX - smoothedGoalX);
            smoothedGoalY = smoothedGoalY + GOAL_POSE_SMOOTHING * (tagFieldY - smoothedGoalY);
        }
        setGoalPose(new Pose(smoothedGoalX, smoothedGoalY));
    }

    /**
     * Inverse of updateGoalFromVision: given a tag observation and its known field position,
     * compute the robot's field position. Used for vision re-localization.
     */
    public Pose computeRobotPoseFromVision(Pose3D tagCameraPose, double tagFieldX, double tagFieldY) {
        if (tagCameraPose == null) return null;
        if (follower == null && localizer == null) return null;

        Position pos = tagCameraPose.getPosition();
        if (pos == null) return null;

        double cameraX = pos.x;
        double cameraZ = pos.z;

        double rawInches = Math.sqrt(cameraX * cameraX + cameraZ * cameraZ) * 39.3701;
        double scaleFactor = (rawInches > 0)
                ? ((rawInches - Vision.DISTANCE_OFFSET) * Vision.DISTANCE_SCALE) / rawInches
                : 1.0;
        double correctedX = cameraX * scaleFactor;
        double correctedZ = cameraZ * scaleFactor;

        double alpha      = Math.toRadians(motor.getCurrentAngle());
        double robotRight =  correctedX * Math.cos(alpha) + correctedZ * Math.sin(alpha);
        double robotFwd   = -correctedX * Math.sin(alpha) + correctedZ * Math.cos(alpha);

        double tagFwdIn   = robotFwd   * 39.3701;
        double tagRightIn = robotRight * 39.3701;

        double heading;
        if (follower != null) {
            heading = follower.getPose().getHeading();
        } else {
            heading = Math.toRadians(localizer.getHeading());
        }

        double robotX = tagFieldX - tagFwdIn * Math.cos(heading) - tagRightIn * Math.sin(heading);
        double robotY = tagFieldY - tagFwdIn * Math.sin(heading) + tagRightIn * Math.cos(heading);

        return new Pose(robotX, robotY, heading);
    }

    // ── Misc ─────────────────────────────────────────────────────────────────

    public boolean isTracking() {
        return (vision != null && vision.hasTargetTag()) || hasGoal();
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
     * smoothedTargetAngle → 0, smoothedGoalX/Y → NaN (forces re-init from vision).
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
