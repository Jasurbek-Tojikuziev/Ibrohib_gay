package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

/**
 * Turret auto-aim: odometry-based targeting with optional physics lead compensation.
 */
@Config
public class TurretAimer {

    // Camera→flywheel parallax correction for relocalization (dashboard-tunable).
    // The camera sits CAMERA_RIGHT_OFFSET_IN beside the flywheel, so without this the
    // relocalization aims the camera (not the flywheel) at the goal. Flip the sign if the
    // side-miss gets worse; 0 disables. Auto-scales with distance.
    public static double RELOC_PARALLAX_SIGN = 1.0;

    // Constant aim trim (deg) added to the goal aim — manual bias correction. + / - to shift sides.
    // Alliance-specific: red and blue goals are on opposite sides, so the SAME encoder trim shifts
    // them toward opposite sides — they need separate values. Selected by setAimTrimAlliance().
    public static double RED_AIM_TRIM_DEG  =  1.0;  // red is correct at +1°
    public static double BLUE_AIM_TRIM_DEG = -0.5;  // tune on dash (+ = aim more right, - = more left)
    private boolean isRedForTrim = true;

    // Relocalization ONLY happens inside this field zone (inches). Outside it, the turret aims on
    // pure odometry/Pinpoint and never relocalizes, even if the error is large.
    public static double RELOC_ZONE_X_MIN = 35.0;
    public static double RELOC_ZONE_X_MAX = 110.0;
    public static double RELOC_ZONE_Y_MIN = 0.0;
    public static double RELOC_ZONE_Y_MAX = 36.0;

    private final TurretMotor      motor;
    private final TurretBallistics ballistics;
    private final Follower         follower;
    private final Localizer        localizer;
    private final Vision           vision;

    private Pose    goalPose = null;
    private Double  goalX    = null;
    private Double  goalY    = null;
    private Double  tagX     = null;
    private Double  tagY     = null;

    private double smoothedTargetAngle  = 0.0;
    private double savedCloseRangeAngle = Double.NaN;
    private double autoAimOffset        = 0.0; // vision-relocalization correction added to the odometry aim
    private double smoothedTx           = 0.0; // EMA of camera horizontal angle to tag (ty, sideways cam)

    // ── Aiming model ───────────────────────────────────────────────────────────
    // PRIMARY aim is ODOMETRY (zero latency, smooth). The camera is used only to
    // RELOCALIZE: when the robot is settled and a tag is stably visible, if the camera's
    // measured angle to the tag disagrees with odometry by more than a threshold, snap that
    // difference into autoAimOffset. The camera is never in the fast loop (its latency would
    // make the turret oscillate), so this avoids that entirely.
    //
    // Camera is mounted 90° sideways → the LEFT/RIGHT angle to the tag is in ty
    // (getTargetPitch), not tx. visionSign (+1/-1) maps ty to turret-angle direction.
    private double visionSign        = 1.0;   // +1/-1: camTagAngle = currentAngle + visionSign*ty (flipped after 180° cam rotation)
    private double relocThresholdDeg = 1.0;   // only correct when residual exceeds this (deg)
    private double relocOffset       = 0.0;   // camera drift correction (separate from manual autoAimOffset)
    private double settleAngVel      = 8.0;   // deg/s — below this counts as "not rotating"
    private double settleTransVel    = 4.0;   // in/s  — below this counts as "not translating"
    private double visionMaxLead     = 6.0;   // (vision-only test mode) cap on chase lead
    private double visionParallaxSign = 2.0;  // aim-shift strength: covers 7cm parallax + tag→goal offset (tuned)

    private static final double VISION_TX_EMA        = 0.4;  // camera-angle smoothing
    private static final int    SETTLE_FRAMES        = 5;    // consecutive still loops before trusting
    private static final double RELOC_MAX_CORRECTION = 45.0; // reject ongoing reads bigger than this (bad frame)
    private boolean firstLockDone = false;                   // first settled lock snaps fully (initial localize)
    private long    lastCorrectionTime = 0L;                 // (unused) legacy cooldown timestamp
    private static final double RELOC_COOLDOWN_SEC = 0.4;    // (unused)
    private boolean relocArmed = true;                       // fire ONE full correction per settle event (instant)

    // Motion estimate (for "settled" detection)
    private double prevHeadingRad = Double.NaN, prevX = 0.0, prevY = 0.0;
    private long   prevMotionTime = 0L;
    private double angVelDeg = 0.0, transVel = 0.0;
    private int    settledFrames = 0;

    // Telemetry
    private double  lastResidual = 0.0;
    private boolean didCorrect   = false;

    /** Clear the camera drift correction (call on a manual position reset — odometry is now known). */
    public void clearRelocalization() {
        relocOffset   = 0.0;
        firstLockDone = false;
        relocArmed    = false; // after a reset, trust the reset pose; re-correct only after driving
    }

    /** Configure camera-relocalization: sign (+1/-1), correction threshold (deg), settle limits. */
    public void setRelocalization(double sign, double thresholdDeg,
                                  double settleAngVelDegPerSec, double settleTransVelInPerSec) {
        this.visionSign     = sign;
        this.relocThresholdDeg = thresholdDeg;
        this.settleAngVel   = settleAngVelDegPerSec;
        this.settleTransVel = settleTransVelInPerSec;
    }

    /** (vision-only test mode) set aim direction, chase lead cap, and parallax sign (+1/-1/0). */
    public void setVisionTuning(double sign, double maxLead, double parallaxSign) {
        this.visionSign         = sign;
        this.visionMaxLead      = maxLead;
        this.visionParallaxSign = parallaxSign;
    }

    // Telemetry getters
    public double  getRelocResidual()    { return lastResidual; }
    public double  getRelocOffset()      { return relocOffset; }
    public boolean isSettledForReloc()   { return isSettled(); }
    public double  getAngularVelDeg()    { return angVelDeg; }
    public double  getTranslationVel()   { return transVel; }
    public boolean didReloc()            { return didCorrect; }

    // ── Constructors ─────────────────────────────────────────────────────────

    /** TeleOp: Pedro Follower + Limelight vision. Vision overrides odometry when tag visible. */
    public TurretAimer(TurretMotor motor, TurretBallistics ballistics, Follower follower, Vision vision) {
        this.motor      = motor;
        this.ballistics = ballistics;
        this.follower   = follower;
        this.localizer  = null;
        this.vision     = vision;
        this.smoothedTargetAngle = motor.getCurrentAngle();
    }

    /** TeleOp: Pedro Follower for odometry only, no vision. */
    public TurretAimer(TurretMotor motor, TurretBallistics ballistics, Follower follower) {
        this.motor      = motor;
        this.ballistics = ballistics;
        this.follower   = follower;
        this.localizer  = null;
        this.vision     = null;
        this.smoothedTargetAngle = motor.getCurrentAngle();
    }

    /** Auto: Localizer for odometry. No physics, no vision. */
    public TurretAimer(TurretMotor motor, TurretBallistics ballistics, Localizer localizer) {
        this.motor      = motor;
        this.ballistics = ballistics;
        this.follower   = null;
        this.localizer  = localizer;
        this.vision     = null;
        this.smoothedTargetAngle = motor.getCurrentAngle();
    }

    /** Vision-only aim test: Limelight tx tracking, no odometry, no physics. */
    public TurretAimer(TurretMotor motor, Vision vision) {
        this.motor      = motor;
        this.ballistics = null;
        this.follower   = null;
        this.localizer  = null;
        this.vision     = vision;
        this.smoothedTargetAngle = motor.getCurrentAngle();
    }

    /** Basic test: no odometry, no physics, no vision. */
    public TurretAimer(TurretMotor motor) {
        this.motor      = motor;
        this.ballistics = null;
        this.follower   = null;
        this.localizer  = null;
        this.vision     = null;
    }

    // ── Goal setup ───────────────────────────────────────────────────────────

    /** Select which alliance's aim trim to use (red and blue need different values). */
    public void setAimTrimAlliance(boolean isRed) { this.isRedForTrim = isRed; }

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
        boolean haveOdometry = hasGoal() && (follower != null || localizer != null);

        if (haveOdometry) {
            // Camera relocalization updates relocOffset (only when settled + tag stable + in-zone).
            updateVisionRelocalization();

            // Apply the camera correction ONLY inside the zone — outside it, aim on pure odometry
            // (the correction was tuned for the zone's geometry and is wrong elsewhere).
            double appliedReloc = isInRelocZone() ? relocOffset : 0.0;
            double aimTrim = isRedForTrim ? RED_AIM_TRIM_DEG : BLUE_AIM_TRIM_DEG;

            double angle = calculateTargetAngle() + appliedReloc + autoAimOffset + aimTrim;
            angle = wrapToReachable(angle, motor.getCurrentAngle()); // short way across the rear seam
            smoothedTargetAngle = angle;
            motor.setTargetAngle(angle);

            // Physics lead (shoot-on-move).
            if (ballistics != null) {
                ballistics.calculate(goalPose, tagX, tagY);
                if (ballistics.isValid()) {
                    double a = ballistics.getTurretAngleDeg() + appliedReloc + autoAimOffset + aimTrim;
                    a = wrapToReachable(a, motor.getCurrentAngle());
                    motor.setTargetAngle(a);
                    smoothedTargetAngle = motor.getTargetAngle();
                }
            }
        } else if (vision != null && vision.hasTargetTag()) {
            // Vision-only test mode (no odometry): continuous tracking off ty with a lead cap.
            smoothedTx += VISION_TX_EMA * (vision.getTargetPitch() - smoothedTx);
            double lead = visionSign * smoothedTx;
            lead = Math.max(-visionMaxLead, Math.min(visionMaxLead, lead));
            // Parallax: the camera sits CAMERA_RIGHT_OFFSET_IN beside the flywheel, so centering
            // the camera on the tag leaves the flywheel pointing off to the side. Shift the aim by
            // atan(offset/distance) so the FLYWHEEL lands on target. Auto-scales with distance.
            double dist = vision.getTargetDistance();
            double parallax = (dist > 1.0)
                    ? visionParallaxSign * Math.toDegrees(Math.atan2(Vision.CAMERA_RIGHT_OFFSET_IN, dist))
                    : 0.0;
            double target = motor.getCurrentAngle() + lead + parallax;
            target = Math.max(TurretMotor.MIN_ANGLE, Math.min(TurretMotor.MAX_ANGLE, target));
            motor.setTargetAngle(target);
            smoothedTargetAngle = target;
        } else if (vision != null) {
            smoothedTx = 0.0;
        }

        double power = motor.calculatePIDF(motor.getTargetAngle(), motor.getCurrentAngle());
        motor.applyPower(power);
    }

    /**
     * Camera relocalization: when the robot is settled and a tag is stably visible, compare the
     * camera's measured angle to the tag against odometry's prediction. If they disagree by more
     * than relocThresholdDeg, snap that residual into autoAimOffset to correct odometry drift/skips.
     */
    private void updateVisionRelocalization() {
        updateMotionEstimate(); // keep the settled-state fresh every loop

        if (vision == null || tagX == null || tagY == null) return;
        if (!vision.hasTargetTag()) { smoothedTx = 0.0; didCorrect = false; return; }

        smoothedTx += VISION_TX_EMA * (vision.getTargetPitch() - smoothedTx);

        // Moving → don't relocalize (camera lag), and ARM so the next time it settles it gets
        // exactly one clean shot. (Prevents relocalizing while driving fast.)
        if (!isSettled()) { relocArmed = true; didCorrect = false; return; }

        // Zone gate: only relocalize inside the shooting zone. Everywhere else, trust odometry
        // entirely — no correction, even if the error is large.
        if (!isInRelocZone()) { didCorrect = false; return; }

        double rdist = vision.getTargetDistance();
        double camParallax = (rdist > 1.0)
                ? RELOC_PARALLAX_SIGN * Math.toDegrees(Math.atan2(Vision.CAMERA_RIGHT_OFFSET_IN, rdist))
                : 0.0;
        double camTagAngle = motor.getCurrentAngle() + visionSign * smoothedTx - camParallax;
        double odoTagAngle = fieldAngleTo(tagX, tagY);
        double residual = normalizeDeg(camTagAngle - (odoTagAngle + relocOffset));
        lastResidual = residual;

        if (Math.abs(residual) > RELOC_MAX_CORRECTION) { didCorrect = false; return; } // bad frame

        // ONE full correction per settle event → the corrected target is applied instantly (no
        // step-wise cooldown), so the turret jumps straight to the new aim. Re-arms only when the
        // robot moves again. This makes the in-zone relocalize-and-shoot fast.
        if (relocArmed && Math.abs(residual) > relocThresholdDeg) {
            relocOffset  += residual;
            relocArmed    = false;
            firstLockDone = true;
            didCorrect    = true;
        } else {
            didCorrect = false; // odometry good enough, or already corrected this settle
        }
    }

    private void updateMotionEstimate() {
        Pose p = getCurrentPose();
        if (p == null) return;
        long now = System.nanoTime();
        if (Double.isNaN(prevHeadingRad)) {
            prevHeadingRad = p.getHeading(); prevX = p.getX(); prevY = p.getY(); prevMotionTime = now;
            return;
        }
        double dt = (now - prevMotionTime) / 1e9;
        if (dt < 1e-3) return;
        double instAng   = Math.abs(Math.toDegrees(normalizeRad(p.getHeading() - prevHeadingRad))) / dt;
        double dx = p.getX() - prevX, dy = p.getY() - prevY;
        double instTrans = Math.sqrt(dx * dx + dy * dy) / dt;
        angVelDeg = 0.5 * angVelDeg + 0.5 * instAng;   // light smoothing
        transVel  = 0.5 * transVel  + 0.5 * instTrans;
        prevHeadingRad = p.getHeading(); prevX = p.getX(); prevY = p.getY(); prevMotionTime = now;

        if (angVelDeg < settleAngVel && transVel < settleTransVel) {
            settledFrames = Math.min(settledFrames + 1, SETTLE_FRAMES);
        } else {
            settledFrames = 0;
        }
    }

    private boolean isSettled() { return settledFrames >= SETTLE_FRAMES; }

    /** True when the robot is inside the relocalization/shooting zone (field inches). */
    private boolean isInRelocZone() {
        Pose p = getCurrentPose();
        if (p == null) return false;
        double rx = p.getX(), ry = p.getY();
        return rx >= RELOC_ZONE_X_MIN && rx <= RELOC_ZONE_X_MAX
            && ry >= RELOC_ZONE_Y_MIN && ry <= RELOC_ZONE_Y_MAX;
    }

    /** Raw odometry turret angle (encoder frame) that points at a field point. No clamp. */
    private double fieldAngleTo(double fx, double fy) {
        Pose p = getCurrentPose();
        if (p == null) return 0.0;
        double dx = fx - p.getX(), dy = fy - p.getY();
        double h  = p.getHeading();
        double rX =  dx * Math.cos(h) + dy * Math.sin(h);
        double rY = -dx * Math.sin(h) + dy * Math.cos(h);
        return -Math.toDegrees(Math.atan2(rY, rX));
    }

    private static double normalizeDeg(double a) {
        while (a >  180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    // Margin kept inside the hard ±MAX_ANGLE limit when choosing a wrapped target.
    private static final double LIMIT_MARGIN_DEG = 5.0;

    /**
     * Pick the target representation (target, target±360) closest to the current turret angle and
     * reachable within the limits MINUS a 5° margin. This uses the turret's past-180° overlap so
     * that when the goal is directly behind (the ±180 seam), the turret nudges the SHORT way
     * (e.g. to +185°) instead of swinging ~350° the long way.
     */
    private double wrapToReachable(double target, double current) {
        double lo = TurretMotor.MIN_ANGLE + LIMIT_MARGIN_DEG;   // -185
        double hi = TurretMotor.MAX_ANGLE - LIMIT_MARGIN_DEG;   // +185
        double best = Double.NaN;
        double bestDist = Double.MAX_VALUE;
        for (double cand : new double[]{ target - 360, target, target + 360 }) {
            if (cand < lo || cand > hi) continue;       // not reachable (with margin)
            double d = Math.abs(cand - current);
            if (d < bestDist) { bestDist = d; best = cand; }
        }
        // Fallback if nothing was in range: just clamp into the usable band.
        return Double.isNaN(best) ? Math.max(lo, Math.min(hi, target)) : best;
    }

    private static double normalizeRad(double a) {
        while (a >  Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
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

    public boolean isTracking()       { return hasGoal(); }
    public boolean hasVisionTarget()  { return vision != null && vision.hasTargetTag(); }
    public Vision  getVision()        { return vision; }

    public void onEncoderReset() {
        smoothedTargetAngle  = 0.0;
        savedCloseRangeAngle = Double.NaN;
        autoAimOffset        = 0.0;
        relocOffset          = 0.0;
        smoothedTx           = 0.0;
        firstLockDone        = false; // re-acquire localization after a re-zero
        relocArmed           = false; // trust the reset; re-correct only after driving
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
