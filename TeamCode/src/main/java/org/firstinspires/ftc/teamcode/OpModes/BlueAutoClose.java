package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.OpModes.auto.AutoBase;

//@Disabled
@Autonomous(name="Blue AutoClose", group="Autonomous", preselectTeleOp = "BLUE Alliance TeleOp")
public class BlueAutoClose extends AutoBase {

    private PathChain path1, path2, path3, path4, path6,
                      path7, path9, path10, path12;

    @Override protected boolean isRedAlliance() { return false; }
    @Override protected Pose getStartPose() { return new Pose(26.318, 128.673, Math.toRadians(135)); }

    private static final double TURRET_OFFSET_DEG = -2.5;
    private static final double INTAKE_TRIGGER_DIST = 5.0;

    @Override
    protected void onStart() {
        autoAimOffsetDeg = TURRET_OFFSET_DEG;
    }

    @Override
    protected void buildPaths() {

        // Path 1 — start → first position (heading constant 135°)
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(26.318, 128.673),
                        new Pose(64.084, 75.991)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        // Path 2 — first position → collection curve (135° → 180°)
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(64.084, 75.991),
                        new Pose(70.442, 54.630),
                        new Pose(20.645, 60)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        // Path 3 — scoring → launch position (heading constant 200°)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(20.645, 58.841),
                        new Pose(58.722, 77.688)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(200))
                .build();

        // Path 4 — LongCurve + ShortCurve chained (no stop between curves)
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(          // LongCurve: 200° → 152.3°
                        new Pose(58.722, 77.688),
                        new Pose(52.765, 63.326),
                        new Pose(28.070, 61.572)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(149))
                .addPath(new BezierCurve(
                        new Pose(28.070, 58.105),
                        new Pose(20.237, 56.869),
                        new Pose(15.2, 61.572)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(149), Math.toRadians(149))
                .build();

        // Path 6 — extended scoring → launch position (152.3° → 200°)
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(15.2, 61.572),
                        new Pose(58.535, 77.625)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(149), Math.toRadians(200))
                .build();

        // Path 7 — LongCurve + ShortCurve chained (no stop between curves)
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(          // LongCurve: 200° → 152.3°
                        new Pose(58.535, 77.625),
                        new Pose(52.765, 63.326),
                        new Pose(28.070, 61.572)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(149))
                .addPath(new BezierCurve(
                        new Pose(28.070, 61.572),
                        new Pose(20.237, 56.869),
                        new Pose(15.2, 61.572)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(149), Math.toRadians(149))
                .build();

        // Path 9 — extended scoring → launch position (152.3° → 200°)
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(15.2, 61.572),
                        new Pose(58.535, 77.625)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(149), Math.toRadians(200))
                .build();

        // Path 10 — LongCurve + ShortCurve chained (no stop between curves)
        path10 = follower.pathBuilder()
                .addPath(new BezierCurve(          // LongCurve: 200° → 152.3°
                        new Pose(58.535, 77.625),
                        new Pose(52.765, 63.326),
                        new Pose(28.070, 61.572)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(149))
                .addPath(new BezierCurve(
                        new Pose(28.070, 61.572),
                        new Pose(20.237, 56.869),
                        new Pose(15.2, 61.572)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(149), Math.toRadians(149))
                .build();

        // Path 12 — extended scoring → park (152.3° → 200°)
        path12 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(1.2, 61.572),
                        new Pose(57.112, 85.548)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(149), Math.toRadians(200))
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {

            // ── Path 1: start → first shoot position ───────────────────────────

            case 0:
                follower.followPath(path1, true);
                setPathState(1);
                break;

            case 1:
                if (distTo(64.084, 75.991) < INTAKE_TRIGGER_DIST) intake.on();
                if (h.pathDone(0)) { shooter.startShoot(); setPathState(10); }
                break;

            case 10:
                if (h.timePassed(1.2)) {
                    follower.followPath(path2, true);
                    setPathState(2);
                }
                break;

            // ── Path 2: collection curve ────────────────────────────────────────

            case 2:
                if (h.pathDone(0.5)) {
                    intake.off();
                    follower.followPath(path3, true);
                    setPathState(3);
                }
                break;

            // ── Path 3: return → second shoot position ──────────────────────────

            case 3:
                if (distTo(58.722, 77.688) < INTAKE_TRIGGER_DIST) intake.on();
                if (h.pathDone(0)) { shooter.startShoot(); setPathState(30); }
                break;

            case 30:
                if (h.timePassed(1.2)) {
                    follower.followPath(path4, true);
                    setPathState(5);
                }
                break;

            // ── Cycle 1: path4 collect → golaunch ──────────────────────────────

            case 5:
                if (h.pathDone(0)) setPathState(50);
                break;

            case 50:
                if (h.timePassed(2.0)) {
                    intake.off();
                    follower.followPath(path6, true);
                    setPathState(6);
                }
                break;

            // ── Path 6: return → third shoot position ───────────────────────────

            case 6:
                if (distTo(58.535, 77.625) < INTAKE_TRIGGER_DIST) intake.on();
                if (h.pathDone(0)) { shooter.startShoot(); setPathState(60); }
                break;

            case 60:
                if (h.timePassed(1.2)) {
                    follower.followPath(path7, true);
                    setPathState(8);
                }
                break;

            // ── Cycle 2: path7 collect → goLaunch ──────────────────────────────

            case 8:
                if (h.pathDone(0)) setPathState(80);
                break;

            case 80:
                if (h.timePassed(2.0)) {
                    intake.off();
                    follower.followPath(path9, true);
                    setPathState(9);
                }
                break;

            // ── Path 9: return → fourth shoot position ───────────────────────────

            case 9:
                if (distTo(58.535, 77.625) < INTAKE_TRIGGER_DIST) intake.on();
                if (h.pathDone(0)) { shooter.startShoot(); setPathState(90); }
                break;

            case 90:
                if (h.timePassed(1.2)) {
                    follower.followPath(path10, true);
                    setPathState(110);
                }
                break;

            // ── Cycle 3: path10 collect → goLaunch ─────────────────────────────

            case 110:
                if (h.pathDone(0)) setPathState(1100);
                break;

            case 1100:
                if (h.timePassed(2.0)) {
                    intake.off();
                    follower.followPath(path12, true);
                    setPathState(120);
                }
                break;

            // ── Path 12: return → final shoot + park ────────────────────────────

            case 120:
                if (distTo(57.112, 85.548) < INTAKE_TRIGGER_DIST) intake.on();
                if (h.pathDone(0)) { shooter.startShoot(); setPathState(121); }
                break;

            case 121:
                if (h.timePassed(1.0)) {
                    intake.off();
                    setPathState(130);
                }
                break;

            case 130:
                break;
        }
    }

    private double distTo(double x, double y) {
        Pose p = follower.getPose();
        return Math.hypot(x - p.getX(), y - p.getY());
    }
}
