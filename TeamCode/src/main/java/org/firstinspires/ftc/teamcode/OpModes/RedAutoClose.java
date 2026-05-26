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

        // Path 1 — start → intake position (heading constant 45°)
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(117.682, 128.673),
                        new Pose(82.916, 70.991)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        // Path 2 — intake → scoring position, Bezier curve (45° → 0°)
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(82.916, 70.991),
                        new Pose(85.994, 58.339),
                        new Pose(123.355, 58.841)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Path 3 — scoring → intake position (heading constant 340°)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(123.355, 58.841),
                        new Pose(82.888, 70.916)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(340))
                .build();

        // Path 4 — intake → scoring position, Bezier curve (340° → 0°)
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(82.888, 70.916),
                        new Pose(86.255, 59.541),
                        new Pose(118.718, 59.304)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(0))
                .build();

        // Path 5 — scoring → extended scoring position, Bezier curve (0° → 30°)
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(118.718, 59.304),
                        new Pose(123.763, 58.679),
                        new Pose(128.219, 62.277)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                .build();

        // Path 6 — extended scoring → return/park (25° → 340°)
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(128.219, 62.277),
                        new Pose(83.075, 70.654)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(340))
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {

            case 0: // Start → intake position
                follower.followPath(path1, true);
                setPathState(1);
                break;

            case 1: // Wait 1000 ms at intake, then go score
                if (h.pathDone(1.0)) {
                    follower.followPath(path2, true);
                    setPathState(2);
                }
                break;

            case 2: // Scoring → immediately return to intake
                if (h.pathDone(0)) {
                    follower.followPath(path3, true);
                    setPathState(3);
                }
                break;

            case 3: // Wait 1000 ms at intake (second cycle), then go score
                if (h.pathDone(1.0)) {
                    follower.followPath(path4, true);
                    setPathState(4);
                }
                break;

            case 4: // Second scoring → immediately go to extended scoring (Bezier)
                if (h.pathDone(0)) {
                    follower.followPath(path5, true);
                    setPathState(5);
                }
                break;

            case 5: // Wait 2000 ms at extended scoring, then return/park
                if (h.pathDone(2.0)) {
                    follower.followPath(path6, true);
                    setPathState(6);
                }
                break;

            case 6: // Done — robot parks at intake area
                break;
        }
    }
}
