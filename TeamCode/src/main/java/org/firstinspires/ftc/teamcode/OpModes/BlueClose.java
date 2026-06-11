package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.OpModes.auto.AutoBase;

/**
 * Mirror of {@link RedClose} across the field center (x → 144−x, y unchanged, heading → 180−heading).
 */
//@Disabled
@Autonomous(name = "blue_close", group = "Autonomous", preselectTeleOp = "BLUE Alliance TeleOp")
public class BlueClose extends AutoBase {

    private PathChain path1, path2, path3, path4, path5, path6,
                      path7, path8, path9,     // gate cycle 1: out / collect-curve / back
                      path10, path11, path12,  // gate cycle 2
                      path13, path14, path15,  // gate cycle 3
                      path16;                  // final reposition

    @Override protected boolean isRedAlliance() { return false; }
    @Override protected Pose getStartPose() { return new Pose(25.964, 127.200, Math.toRadians(316)); }

    // Turret bias correction — HARDCODED (no @Config so the dashboard can't override it).
    // Auto-only, no TeleOp effect. More negative = aim more LEFT.
    private static final double TURRET_OFFSET_DEG = -2.5;

    @Override
    protected void onStart() {
        turret.setAutoAimOffset(TURRET_OFFSET_DEG);
    }

    @Override
    protected void buildPaths() {

        // Path 1 — start (25.964, 127.200) → (43.691, 106.100), heading -44°
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(25.964, 127.200), new Pose(43.691, 106.100)))
                .setLinearHeadingInterpolation(Math.toRadians(-44), Math.toRadians(-44))
                .build();

        // Path 2 — curve (43.691, 106.100) → (28.200, 83.745), heading -110° → -90°
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(43.691, 106.100), new Pose(25.518, 102.336), new Pose(28.200, 83.745)))
                .setLinearHeadingInterpolation(Math.toRadians(-110), Math.toRadians(-90))
                .build();

        // Path 3 — (28.200, 83.745) → (33.564, 99.945), heading -110°
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(28.200, 83.745), new Pose(33.564, 99.945)))
                .setLinearHeadingInterpolation(Math.toRadians(-110), Math.toRadians(-110))
                .build();

        // Path 4 — (33.564, 99.945) → (24.109, 63.100), heading -115° → -90°
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(33.564, 99.945), new Pose(24.109, 63.100)))
                .setLinearHeadingInterpolation(Math.toRadians(-115), Math.toRadians(-90))
                .build();

        // Path 5 — (24.109, 63.100) → (17.545, 63.100), heading -90°
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(24.109, 63.100), new Pose(17.545, 63.100)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();

        // Path 6 — (17.545, 63.100) → (49.836, 75.236), heading -90° → -150°
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(17.545, 63.100), new Pose(49.836, 75.236)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-150))
                .build();

        // ── Gate cycle 1 ──────────────────────────────────────────────────────
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(49.836, 75.236), new Pose(15.636, 46.873)))
                .setLinearHeadingInterpolation(Math.toRadians(-145), Math.toRadians(155))
                .build();
        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(15.636, 46.873), new Pose(12.118, 47.318), new Pose(6.040, 50.091)))
                .setLinearHeadingInterpolation(Math.toRadians(154), Math.toRadians(154))
                .build();
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(6.040, 50.091), new Pose(49.600, 75.018)))
                .setLinearHeadingInterpolation(Math.toRadians(-152), Math.toRadians(-152))
                .build();

        // ── Gate cycle 2 (same as cycle 1) ─────────────────────────────────────
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(49.836, 75.236), new Pose(15.636, 46.873)))
                .setLinearHeadingInterpolation(Math.toRadians(-145), Math.toRadians(155))
                .build();
        path11 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(15.636, 46.873), new Pose(12.118, 47.318), new Pose(6.040, 50.091)))
                .setLinearHeadingInterpolation(Math.toRadians(154), Math.toRadians(154))
                .build();
        path12 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(6.040, 50.091), new Pose(49.600, 75.018)))
                .setLinearHeadingInterpolation(Math.toRadians(-152), Math.toRadians(-152))
                .build();

        // ── Gate cycle 3 (same as cycle 1) ─────────────────────────────────────
        path13 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(49.836, 75.236), new Pose(15.636, 46.873)))
                .setLinearHeadingInterpolation(Math.toRadians(-145), Math.toRadians(155))
                .build();
        path14 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(15.636, 46.873), new Pose(12.118, 47.318), new Pose(6.5, 50.091)))
                .setLinearHeadingInterpolation(Math.toRadians(154), Math.toRadians(154))
                .build();
        path15 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(6.540, 50.091), new Pose(49.600, 75.018)))
                .setLinearHeadingInterpolation(Math.toRadians(-152), Math.toRadians(-152))
                .build();

        // Path 16 — final reposition (49.600, 75.018) → (37.345, 69.255), heading -152°
        path16 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(49.600, 75.018), new Pose(37.345, 69.255)))
                .setLinearHeadingInterpolation(Math.toRadians(-152), Math.toRadians(-152))
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
            case 11: if (h.pathDone(0)) { setPathState(111); } break;                       // arrived at gate
            case 111: if (h.timePassed(1.5)) { follower.followPath(path9, true); setPathState(12); } break; // dwell 1.5s collecting
            case 12: if (h.pathDone(0)) { intake.off(); shooter.startShoot(); setPathState(13); } break; // SHOOT
            case 13: if (h.timePassed(1.0)) { follower.followPath(path10, true); setPathState(14); } break;

            // Gate cycle 2
            case 14: if (h.pathDone(0)) { intake.on(); follower.followPath(path11, true); setPathState(15); } break;
            case 15: if (h.pathDone(0)) { setPathState(151); } break;                       // arrived at gate
            case 151: if (h.timePassed(1.5)) { follower.followPath(path12, true); setPathState(16); } break; // dwell 1.5s collecting
            case 16: if (h.pathDone(0)) { intake.off(); shooter.startShoot(); setPathState(17); } break; // SHOOT
            case 17: if (h.timePassed(1.0)) { follower.followPath(path13, true); setPathState(18); } break;

            // Gate cycle 3
            case 18: if (h.pathDone(0)) { intake.on(); follower.followPath(path14, true); setPathState(19); } break;
            case 19: if (h.pathDone(0)) { setPathState(191); } break;                       // arrived at gate
            case 191: if (h.timePassed(1.5)) { follower.followPath(path15, true); setPathState(20); } break; // dwell 1.5s collecting
            case 20: if (h.pathDone(0)) { intake.off(); shooter.startShoot(); setPathState(21); } break; // SHOOT
            case 21: if (h.timePassed(1.0)) { follower.followPath(path16, true); setPathState(22); } break;

            case 22: if (h.pathDone(0)) { intake.off(); setPathState(23); } break; // final reposition → parked
            case 23: break; // done
        }
    }
}
