package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.OpModes.auto.AutoBase;

//@Disabled
@Autonomous(name = "red_close", group = "Autonomous", preselectTeleOp = "RED Alliance TeleOp")
public class RedClose extends AutoBase {

    private PathChain path1, path2, path3, path4, path5, path6,
                      path7, path8, path9,     // gate cycle 1: out / collect-curve / back
                      path10, path11, path12,  // gate cycle 2
                      path13, path14, path15,  // gate cycle 3
                      path16;                  // final reposition

    @Override protected boolean isRedAlliance() { return true; }
    @Override protected Pose getStartPose() { return new Pose(118.036, 127.200, Math.toRadians(224)); }

    // Turret bias correction (positive = right, negative = left). Tune for first-shot center.
    private static final double TURRET_OFFSET_DEG = -2.5;

    @Override
    protected void onStart() {
        turret.setAutoAimOffset(TURRET_OFFSET_DEG);
    }

    @Override
    protected void buildPaths() {

        // Path 1 — start (118.036, 127.200) → (100.309, 106.100), heading 224°
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(118.036, 127.200), new Pose(100.309, 106.100)))
                .setLinearHeadingInterpolation(Math.toRadians(224), Math.toRadians(224))
                .build();

        // Path 2 — curve (100.309, 106.100) → (119.291, 90.273), heading 290° → 270°
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(100.309, 106.100), new Pose(121.318, 102.336), new Pose(119.291, 90.273)))
                .setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(270))
                .build();

        // Path 3 — (119.291, 90.273) → (107.818, 106.055), heading 290°
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(119.291, 90.273), new Pose(107.818, 106.055)))
                .setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(290))
                .build();

        // Path 4 — (107.818, 106.055) → (119.891, 70.100), heading 295° → 270°
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(107.818, 106.055), new Pose(119.891, 70.100)))
                .setLinearHeadingInterpolation(Math.toRadians(295), Math.toRadians(270))
                .build();

        // Path 5 — (119.891, 70.100) → (128.455, 71.100), heading 270°
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(119.891, 70.100), new Pose(128.455, 71.100)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                .build();

        // Path 6 — (128.455, 71.100) → (84.345, 86.273), heading 270° → 330°
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(128.455, 71.100), new Pose(84.345, 86.273)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(330))
                .build();

        // ── Gate cycle 1 ──────────────────────────────────────────────────────
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(84.345, 86.273), new Pose(117.436, 67.582)))
                .setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(325))
                .build();
        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(117.436, 67.582), new Pose(125.136, 64.773), new Pose(126.618, 68.891)))
                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(25))
                .build();
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(126.618, 68.891), new Pose(84.364, 86.800)))
                .setLinearHeadingInterpolation(Math.toRadians(332), Math.toRadians(332))
                .build();

        // ── Gate cycle 2 ──────────────────────────────────────────────────────
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(84.364, 86.800), new Pose(117.436, 67.582)))
                .setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(325))
                .build();
        path11 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(117.436, 67.582), new Pose(125.136, 64.773), new Pose(126.618, 68.891)))
                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(25))
                .build();
        path12 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(126.618, 68.891), new Pose(84.364, 86.700)))
                .setLinearHeadingInterpolation(Math.toRadians(332), Math.toRadians(332))
                .build();

        // ── Gate cycle 3 ──────────────────────────────────────────────────────
        path13 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(84.364, 86.700), new Pose(117.436, 67.582)))
                .setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(325))
                .build();
        path14 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(117.436, 67.582), new Pose(125.136, 64.773), new Pose(126.618, 68.891)))
                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(25))
                .build();
        path15 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(126.618, 68.891), new Pose(85.000, 86.800)))
                .setLinearHeadingInterpolation(Math.toRadians(332), Math.toRadians(332))
                .build();

        // Path 16 — final reposition (85.000, 86.800) → (99.236, 77.545), heading 332°
        path16 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(85.000, 86.800), new Pose(99.236, 77.545)))
                .setLinearHeadingInterpolation(Math.toRadians(332), Math.toRadians(332))
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {

            case 0:  follower.followPath(path1, true); setPathState(1); break;
            case 1:  if (h.pathDone(0)) { shooter.startShoot(); setPathState(2); } break;        // SHOOT (preload)
            case 2:  if (h.timePassed(1.0)) { follower.followPath(path2, true); setPathState(3); } break;

            case 3:  if (h.pathDone(0)) { intake.on(); follower.followPath(path3, true); setPathState(4); } break;
            case 4:  if (h.pathDone(0)) { intake.off(); shooter.startShoot(); setPathState(5); } break; // SHOOT
            case 5:  if (h.timePassed(1.0)) { follower.followPath(path4, true); setPathState(6); } break;

            case 6:  if (h.pathDone(0)) { intake.on(); follower.followPath(path5, true); setPathState(7); } break;
            case 7:  if (h.pathDone(0)) { follower.followPath(path6, true); setPathState(8); } break;
            case 8:  if (h.pathDone(0)) { intake.off(); shooter.startShoot(); setPathState(9); } break; // SHOOT
            case 9:  if (h.timePassed(1.0)) { follower.followPath(path7, true); setPathState(10); } break;

            // Gate cycle 1
            case 10: if (h.pathDone(0)) { intake.on(); follower.followPath(path8, true); setPathState(11); } break;
            case 11: if (h.pathDone(0)) { follower.followPath(path9, true); setPathState(12); } break;
            case 12: if (h.pathDone(0)) { intake.off(); shooter.startShoot(); setPathState(13); } break; // SHOOT
            case 13: if (h.timePassed(1.0)) { follower.followPath(path10, true); setPathState(14); } break;

            // Gate cycle 2
            case 14: if (h.pathDone(0)) { intake.on(); follower.followPath(path11, true); setPathState(15); } break;
            case 15: if (h.pathDone(0)) { follower.followPath(path12, true); setPathState(16); } break;
            case 16: if (h.pathDone(0)) { intake.off(); shooter.startShoot(); setPathState(17); } break; // SHOOT
            case 17: if (h.timePassed(1.0)) { follower.followPath(path13, true); setPathState(18); } break;

            // Gate cycle 3
            case 18: if (h.pathDone(0)) { intake.on(); follower.followPath(path14, true); setPathState(19); } break;
            case 19: if (h.pathDone(0)) { follower.followPath(path15, true); setPathState(20); } break;
            case 20: if (h.pathDone(0)) { intake.off(); shooter.startShoot(); setPathState(21); } break; // SHOOT
            case 21: if (h.timePassed(1.0)) { follower.followPath(path16, true); setPathState(22); } break;

            case 22: if (h.pathDone(0)) { intake.off(); setPathState(23); } break; // final reposition → parked
            case 23: break; // done
        }
    }
}
