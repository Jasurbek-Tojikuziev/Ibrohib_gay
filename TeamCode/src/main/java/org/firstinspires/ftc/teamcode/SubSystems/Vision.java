package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/**
 * Limelight 3A AprilTag vision wrapper.
 *
 * Camera mounting on this robot:
 *   - Height above ground : 34 cm (13.4 in)
 *   - Lateral offset      : 7 cm (2.756 in) RIGHT of flywheel center
 *   - Mount               : on turret — rotates with it
 *
 * IMPORTANT: the camera is mounted 90° SIDEWAYS on the turret. As a result the
 * LEFT/RIGHT angle to the tag (what aiming needs) is reported in ty
 * (getTargetPitch), NOT tx. tx reads the up/down angle on this robot. The turret
 * aimer therefore drives ty to zero. Distance uses the 3D pose magnitude, which is
 * orientation-independent, so it is unaffected by the rotation.
 */
public class Vision {

    private final Limelight3A limelight;
    private int targetTagId;

    // Camera geometry — used by TurretAimer for parallax correction
    public static final double CAMERA_RIGHT_OFFSET_IN = 2.756; // 7 cm → inches

    // ty-based distance: physical geometry of the camera mount
    // distance = (TAG_HEIGHT_IN - CAMERA_HEIGHT_IN) / tan(CAMERA_TILT_DEG + ty)
    private static final double CAMERA_HEIGHT_IN = 13.4;  // 34 cm above floor
    private static final double TAG_HEIGHT_IN     = 29.5;  // official FTC SDK: DECODE goal tags (20/24) Z height
    public  static       double CAMERA_TILT_DEG   = 18.7; // derived from measurement; tune if needed

    // Persistence: require N consecutive frames before trusting tag
    private static final int PERSISTENCE_THRESHOLD = 3;
    private int consecutiveFrames = 0;

    // Staleness — treat camera as disconnected if data is older than this
    private static final long STALENESS_MS = 500;

    private LLResult cachedResult = null;

    public Vision(HardwareMap hardwareMap, boolean isRedAlliance) {
        limelight   = hardwareMap.get(Limelight3A.class, "limelight");
        targetTagId = isRedAlliance ? 24 : 20;
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100); // request fresh data 100x/sec so tx doesn't lag the turret
        limelight.start();
    }

    /**
     * Call once per loop BEFORE reading hasTargetTag() / getTargetYaw() / getTargetDistance().
     * Updates persistence counter and refreshes cached results.
     */
    public void update() {
        LLResult result = limelight.getLatestResult();
        if (result == null || result.getStaleness() > STALENESS_MS) {
            consecutiveFrames = 0;
            return;
        }
        cachedResult = result;
        if (findTarget(result) != null) {
            consecutiveFrames = Math.min(consecutiveFrames + 1, PERSISTENCE_THRESHOLD + 10);
        } else {
            consecutiveFrames = 0;
        }
    }

    /** True when the target tag has been seen in 3+ consecutive frames. */
    public boolean hasTargetTag() {
        return consecutiveFrames >= PERSISTENCE_THRESHOLD;
    }

    /**
     * Horizontal angle to target tag (Limelight tx), degrees.
     * Positive = tag is to the right of the camera crosshair.
     * Returns 0 if no target.
     */
    public double getTargetYaw() {
        LLResultTypes.FiducialResult t = findTarget(cachedResult);
        return t != null ? t.getTargetXDegrees() : 0.0;
    }

    /**
     * Horizontal ground distance to tag, in inches.
     *
     * Primary source: Limelight 3D pose (targetPoseCameraSpace). This requires the
     * AprilTag size (6.5" for DECODE goal tags) to be set in the Limelight pipeline.
     * The 3D straight-line distance is tilt-independent (it is a vector magnitude),
     * so we convert to horizontal range using only the fixed tag/camera height
     * difference — no CAMERA_TILT_DEG needed and no inversion possible.
     *
     * Fallback: ty-trig, used only when 3D pose is unavailable (returns near-zero).
     * Formula: distance = (TAG_HEIGHT_IN - CAMERA_HEIGHT_IN) / tan(CAMERA_TILT_DEG + ty)
     */
    public double getTargetDistance() {
        LLResultTypes.FiducialResult t = findTarget(cachedResult);
        if (t == null) return 0.0;

        double heightDiff = TAG_HEIGHT_IN - CAMERA_HEIGHT_IN;

        // Primary: 3D pose → straight-line distance → horizontal ground range.
        Pose3D pose = t.getTargetPoseCameraSpace();
        if (pose != null) {
            Position p = pose.getPosition(); // meters
            double straightIn = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z) * 39.3701;
            if (straightIn > 1.0) {
                double horiz2 = straightIn * straightIn - heightDiff * heightDiff;
                return horiz2 > 0 ? Math.sqrt(horiz2) : straightIn;
            }
        }

        // Fallback: ty-trig (only when 3D pose is not being produced).
        double ty = t.getTargetYDegrees();
        double angleRad = Math.toRadians(CAMERA_TILT_DEG + ty);
        if (Math.abs(angleRad) < 0.01) return 0.0;
        double dist = heightDiff / Math.tan(angleRad);
        return Math.max(0.0, dist);
    }

    /** True if limelight is returning fresh data (not stale / disconnected). */
    public boolean isConnected() {
        LLResult r = limelight.getLatestResult();
        return r != null && r.getStaleness() <= STALENESS_MS;
    }

    /** Switch alliance mid-run if needed (e.g. test modes). */
    public void setAlliance(boolean isRedAlliance) {
        targetTagId = isRedAlliance ? 24 : 20;
    }

    /** Vertical angle to tag (Limelight ty), degrees. For diagnosing camera rotation. */
    public double getTargetPitch() {
        LLResultTypes.FiducialResult t = findTarget(cachedResult);
        return t != null ? t.getTargetYDegrees() : 0.0;
    }

    public int getTargetTagId()        { return targetTagId; }
    public int getConsecutiveFrames()  { return consecutiveFrames; }

    public void stop() { limelight.stop(); }

    // ── Internal ─────────────────────────────────────────────────────────────

    private LLResultTypes.FiducialResult findTarget(LLResult result) {
        if (result == null) return null;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == targetTagId) return f;
        }
        return null;
    }
}
