package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.OpModes.auto.AutoBase;

//@Disabled
@Autonomous(name="Red AutoClose", group="Autonomous", preselectTeleOp = "RED Alliance TeleOp")
public class RedAutoClose extends AutoBase {

    // path4 / path7 / path10 are each LongCurve+shortcurve chained into one PathChain
    // so the robot never stops between the two curves
    private PathChain path1, path2, path3, path4, path6,
                      path7, path9, path10, path12;

    @Override protected boolean isRedAlliance() { return true; }
    @Override protected Pose getStartPose() { return new Pose(117.6822429, 128.6728971, Math.toRadians(45)); }

    // Turret calibration: positive = right, negative = left.
    // Correct the systematic right-bias observed on this robot.
    // Tune in 0.5° steps until first-shot rings the center of the goal.
    private static final double TURRET_OFFSET_DEG = -2.5;

    // Distance from shoot endpoint at which intake starts spinning (inches).
    private static final double INTAKE_TRIGGER_DIST = 5.0;

    @Override
    protected void onStart() {
        autoAimOffsetDeg = TURRET_OFFSET_DEG;
    }

    @Override
    protected void buildPaths() {

        // Path 1 — start → first position (heading constant 45°)
        path1 = follower.pathBuilder()

                .addPath(new BezierLine(
                        new Pose(117.682, 128.673),
                        new Pose(79.916, 75.991)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        // Path 2 (Firstintakecurve) — first position → scoring (45° → 0°)
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(79.916, 75.991),
                        new Pose(85.994, 58.339),
                        new Pose(123.355, 58.841)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Path 3 — scoring → launch position (heading constant 340°)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(123.355, 58.841),
                        new Pose(85.278, 77.688)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(340))
                .build();

        // Path 4 — LongCurve + shortcurve chained (no stop between curves)
        // launch → mid-scoring → extended scoring
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(          // LongCurve: 340° → 0°
                        new Pose(85.278, 77.688),
                        new Pose(91.235, 63.326),
                        new Pose(115.930, 61.572)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(27.7))
                .addPath(new BezierCurve(
                        new Pose(115.930, 58.105),
                        new Pose(123.763, 56.869),
                        new Pose(130.000, 61.572)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(27.7), Math.toRadians(27.7))
                .build();

        // Path 6 (golaunch) — extended scoring → launch position (25.7° → 340°)
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(130.000, 61.572),
                        new Pose(85.465, 77.625)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(27.7), Math.toRadians(340))
                .build();

        // Path 7 — Longcurve + ShortCurve chained (no stop between curves)
        // launch → mid-scoring → extended scoring
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(          // Longcurve: 340° → 0°
                        new Pose(85.465, 77.625),
                        new Pose(91.235, 63.326),
                        new Pose(115.930, 61.572)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(27.7))
                .addPath(new BezierCurve(
                        new Pose(115.930, 61.572),
                        new Pose(123.763, 56.869),
                        new Pose(130.000, 60.572)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(27.7), Math.toRadians(27.7))
                .build();

        // Path 9 (goLaunch) — extended scoring → launch position (25.7° → 340°)
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(130.000, 61.572),
                        new Pose(85.465, 77.625)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(27.7), Math.toRadians(340))
                .build();

        // Path 10 — Longcurve + shortcurve chained (no stop between curves)
        // launch → mid-scoring → extended scoring
        path10 = follower.pathBuilder()
                .addPath(new BezierCurve(          // Longcurve: 340° → 0°
                        new Pose(85.465, 77.625),
                        new Pose(91.235, 63.326),
                        new Pose(115.930, 61.572)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(27.7))
                .addPath(new BezierCurve(
                        new Pose(115.930, 61.572),
                        new Pose(123.763, 56.869),
                        new Pose(130.000, 61.572)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(27.7), Math.toRadians(27.7))
                .build();

        // Path 12 (goLaunch) — extended scoring → park (25.7° → 340°)
        path12 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(130.000, 61.572),
                        new Pose(86.888, 85.548)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(27.7), Math.toRadians(340))
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {

            // ── Path 1: start → first shoot position ───────────────────────────

            case 0: // Drive to first shoot position
                follower.followPath(path1, true);
                setPathState(1);
                break;

            case 1: // intake on at 5" from target, shoot on arrival
                if (distTo(79.916, 75.991) < INTAKE_TRIGGER_DIST) intake.on();
                if (h.pathDone(0)) { shooter.startShoot(); setPathState(10); }
                break;

            case 10: // 1.2s shoot → collection curve (intake already on)
                if (h.timePassed(1.2)) {
                    follower.followPath(path2, true);
                    setPathState(2);
                }
                break;

            // ── Path 2: collection curve ────────────────────────────────────────

            case 2: // path2 done + 0.5s dwell → intake off, return to launch
                if (h.pathDone(0.5)) {
                    intake.off();
                    follower.followPath(path3, true);
                    setPathState(3);
                }
                break;

            // ── Path 3: return → second shoot position ──────────────────────────

            case 3: // intake on at 5" from target, shoot on arrival
                if (distTo(85.278, 77.688) < INTAKE_TRIGGER_DIST) intake.on();
                if (h.pathDone(0)) { shooter.startShoot(); setPathState(30); }
                break;

            case 30: // 1.2s shoot → cycle 1 collect (intake already on)
                if (h.timePassed(1.2)) {
                    follower.followPath(path4, true);
                    setPathState(5);
                }
                break;

            // ── Cycle 1: path4 collect → golaunch ──────────────────────────────

            case 5: // path4 done → start 2500ms stationary collect
                if (h.pathDone(0)) setPathState(50);
                break;

            case 50: // 2500ms stationary → intake off, golaunch
                if (h.timePassed(2.0)) {
                    intake.off();
                    follower.followPath(path6, true);
                    setPathState(6);
                }
                break;

            // ── Path 6: return → third shoot position ───────────────────────────

            case 6: // intake on at 5" from target, shoot on arrival
                if (distTo(85.465, 77.625) < INTAKE_TRIGGER_DIST) intake.on();
                if (h.pathDone(0)) { shooter.startShoot(); setPathState(60); }
                break;

            case 60: // 1.2s shoot → cycle 2 collect (intake already on)
                if (h.timePassed(1.2)) {
                    follower.followPath(path7, true);
                    setPathState(8);
                }
                break;

            // ── Cycle 2: path7 collect → goLaunch ──────────────────────────────

            case 8: // path7 done → start 2500ms stationary collect
                if (h.pathDone(0)) setPathState(80);
                break;

            case 80: // 2500ms stationary → intake off, goLaunch
                if (h.timePassed(2.0)) {
                    intake.off();
                    follower.followPath(path9, true);
                    setPathState(9);
                }
                break;

            // ── Path 9: return → fourth shoot position ───────────────────────────

            case 9: // intake on at 5" from target, shoot on arrival
                if (distTo(85.465, 77.625) < INTAKE_TRIGGER_DIST) intake.on();
                if (h.pathDone(0)) { shooter.startShoot(); setPathState(90); }
                break;

            case 90: // 1.2s shoot → cycle 3 collect (intake already on)
                if (h.timePassed(1.2)) {
                    follower.followPath(path10, true);
                    setPathState(110);
                }
                break;

            // ── Cycle 3: path10 collect → goLaunch ─────────────────────────────

            case 110: // path10 done → start 2500ms stationary collect
                if (h.pathDone(0)) setPathState(1100);
                break;

            case 1100: // 2500ms stationary → intake off, goLaunch final
                if (h.timePassed(2.0)) {
                    intake.off();
                    follower.followPath(path12, true);
                    setPathState(120);
                }
                break;

            // ── Path 12: return → final shoot + park ────────────────────────────

            case 120: // intake on at 5" from target, shoot on arrival
                if (distTo(86.888, 85.548) < INTAKE_TRIGGER_DIST) intake.on();
                if (h.pathDone(0)) { shooter.startShoot(); setPathState(121); }
                break;

            case 121: // 1.2s final shoot → intake off → parked
                if (h.timePassed(1.0)) {
                    intake.off();
                    setPathState(130);
                }
                break;

            case 130: // Done — robot parked
                break;
        }
    }

    /** Euclidean distance from current robot pose to target point (inches). */
    private double distTo(double x, double y) {
        Pose p = follower.getPose();
        return Math.hypot(x - p.getX(), y - p.getY());
    }
}
