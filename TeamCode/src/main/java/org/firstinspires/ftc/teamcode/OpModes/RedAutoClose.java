package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.OpModes.auto.AutoBase;

//@Disabled
@Autonomous(name="Red AutoClose", group="Autonomous", preselectTeleOp = "RED Alliance TeleOp")
public class RedAutoClose extends AutoBase {

    private PathChain path1, path2, path3, path4, path5, path6;

    @Override protected boolean isRedAlliance() { return true; }
    @Override protected Pose getStartPose() { return new Pose(117.6822429, 128.6728971, Math.toRadians(45)); }

    @Override
    protected void buildPaths() {

        // Path 1 — start → first collection position (heading constant 45°)
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(117.682, 128.673),
                        new Pose(79.916, 75.991)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        // Path 2 — collection → scoring, Bezier curve (45° → 0°)
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(80.916, 70.991),
                        new Pose(85.994, 58.339),
                        new Pose(123.355, 58.841)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Path 3 — scoring → second collection position (heading constant 340°)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(123.355, 58.841),
                        new Pose(85.278, 77.688)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(340))
                .build();

        // Path 4 — collection → scoring, Bezier curve (340° → 0°)
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(85.278, 77.688),
                        new Pose(91.235, 63.326),
                        new Pose(115.930, 61.105)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(0))
                .build();

        // Path 5 — scoring → extended scoring, Bezier curve (heading constant 32°)
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(115.930, 61.105),
                        new Pose(123.763, 60.869),
                        new Pose(129.50, 61.272)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(25.7), Math.toRadians(25.7))
                .build();

        // Path 6 — extended scoring → return/park (25° → 340°)
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(131.000, 61.472),
                        new Pose(85.465, 77.625)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(25.7), Math.toRadians(340))
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {

            // ── First cycle ────────────────────────────────────────────────────

            case 0: // Drive to first collection position
                follower.followPath(path1, true);
                setPathState(1);
                break;

            case 1: // Wait for path1 to finish → intake ON
                if (h.pathDone(0)) {
                    intake.on();
                    setPathState(11);
                }
                break;

            case 11: // 500 ms intake collection before shooting
                if (h.timePassed(0.5)) {
                    shooter.startShoot();
                    setPathState(12);
                }
                break;

            case 12: // 1200 ms shooting (intake continues) → intake OFF, drive to scoring
                if (h.timePassed(1.2)) {
                    intake.off();
                    follower.followPath(path2, true);
                    setPathState(2);
                }
                break;

            case 2: // path2 done + 500 ms at scoring → return to collection
                if (h.pathDone(0.5)) {
                    follower.followPath(path3, true);
                    setPathState(3);
                }
                break;

            // ── Second cycle ───────────────────────────────────────────────────

            case 3: // Wait for path3 to finish → intake ON
                if (h.pathDone(0)) {
                    intake.on();
                    setPathState(31);
                }
                break;

            case 31: // 500 ms intake collection before shooting
                if (h.timePassed(0.5)) {
                    shooter.startShoot();
                    setPathState(32);
                }
                break;

            case 32: // 1200 ms shooting (intake continues) → intake OFF, drive to scoring
                if (h.timePassed(1.2)) {
                    intake.off();
                    follower.followPath(path4, true);
                    setPathState(4);
                }
                break;

            case 4: // path4 done → immediately drive to extended scoring
                if (h.pathDone(0)) {
                    follower.followPath(path5, true);
                    setPathState(5);
                }
                break;

            case 5: // path5 done + 2000 ms at extended scoring → return/park
                if (h.pathDone(2.0)) {
                    follower.followPath(path6, true);
                    setPathState(6);
                }
                break;

            case 6: // Done — robot parked
                break;
        }
    }
}
