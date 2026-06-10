package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/**
 * Limelight distance calibration tester.
 * Shows all available distance sources so you can see which one works.
 */
@TeleOp(name = "Limelight Tester", group = "Testers")
public class LimelightTester extends LinearOpMode {

    // Change to 20 for BLUE alliance
    private static final int TARGET_TAG_ID = 24;

    // Camera geometry for ty-based distance
    private static final double CAMERA_HEIGHT_IN = 13.4;  // 34 cm above floor
    private static final double TAG_HEIGHT_IN     = 29.5;  // official FTC SDK: DECODE goal tags (20/24) Z height
    private static final double CAMERA_TILT_DEG   = 18.7; // derived from measurement at 220 cm

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Limelight Tester ready. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result == null) {
                telemetry.addLine("Status: NO RESULT (limelight not responding)");
                telemetry.update();
                continue;
            }

            long staleness = result.getStaleness();
            telemetry.addData("Status", staleness <= 500 ? "CONNECTED" : "STALE (" + staleness + "ms)");

            LLResultTypes.FiducialResult target = findTag(result, TARGET_TAG_ID);

            if (target != null) {
                double tx = target.getTargetXDegrees();
                double ty = target.getTargetYDegrees();
                double ta = target.getTargetArea();

                telemetry.addLine("=== Tag " + TARGET_TAG_ID + " FOUND ===");
                telemetry.addData("tx", String.format("%.2f deg", tx));
                telemetry.addData("ty", String.format("%.2f deg", ty));
                telemetry.addData("ta", String.format("%.3f %%", ta));

                // -- Pose source 1: targetPoseCameraSpace (tag in camera frame)
                Pose3D camPose = target.getTargetPoseCameraSpace();
                if (camPose != null) {
                    Position p = camPose.getPosition();
                    double straight = Math.sqrt(p.x*p.x + p.y*p.y + p.z*p.z) * 39.3701;
                    double heightDiff = TAG_HEIGHT_IN - CAMERA_HEIGHT_IN;
                    double horiz2 = straight*straight - heightDiff*heightDiff;
                    double horiz = horiz2 > 0 ? Math.sqrt(horiz2) : straight;
                    telemetry.addData("POSE straight-line", String.format("%.2f in", straight));
                    telemetry.addData("POSE horizontal  <-- use this", String.format("%.2f in", horiz));
                    telemetry.addData("  raw (m)", String.format("x=%.3f y=%.3f z=%.3f", p.x, p.y, p.z));
                } else {
                    telemetry.addLine("targetPoseCameraSpace: null");
                }

                // -- Pose source 2: cameraPoseTargetSpace (camera in tag frame)
                Pose3D tagPose = target.getCameraPoseTargetSpace();
                if (tagPose != null) {
                    Position p = tagPose.getPosition();
                    double d = Math.sqrt(p.x*p.x + p.y*p.y + p.z*p.z) * 39.3701;
                    telemetry.addData("cameraPoseTargetSpace dist", String.format("%.2f in  (x=%.3f y=%.3f z=%.3f m)", d, p.x, p.y, p.z));
                } else {
                    telemetry.addLine("cameraPoseTargetSpace: null");
                }

                // ty-based: distance = (TAG_HEIGHT - CAM_HEIGHT) / tan(TILT + ty)
                double angleRad = Math.toRadians(CAMERA_TILT_DEG + ty);
                double tyDist = (Math.abs(angleRad) > 0.01)
                        ? (TAG_HEIGHT_IN - CAMERA_HEIGHT_IN) / Math.tan(angleRad)
                        : 0;
                telemetry.addData("ty-based dist", String.format("%.2f in", Math.max(0, tyDist)));

            } else {
                telemetry.addLine("Tag " + TARGET_TAG_ID + " NOT visible");
                List<LLResultTypes.FiducialResult> all = result.getFiducialResults();
                if (all != null && !all.isEmpty()) {
                    StringBuilder ids = new StringBuilder("Seen tag IDs: ");
                    for (LLResultTypes.FiducialResult f : all) ids.append(f.getFiducialId()).append(" ");
                    telemetry.addLine(ids.toString());
                } else {
                    telemetry.addLine("No fiducials in frame");
                }
            }

            telemetry.update();
        }

        limelight.stop();
    }

    private LLResultTypes.FiducialResult findTag(LLResult result, int id) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == id) return f;
        }
        return null;
    }
}
