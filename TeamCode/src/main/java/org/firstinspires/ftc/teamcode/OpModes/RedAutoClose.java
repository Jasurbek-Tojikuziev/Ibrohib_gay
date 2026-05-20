package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.OpModes.auto.AutoBase;

//@Disabled
@Autonomous(name="Red AutoClose", group="Autonomous", preselectTeleOp = "RED Alliance TeleOp")
public class RedAutoClose extends AutoBase {

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;

    @Override protected boolean isRedAlliance() { return true; }
    @Override protected Pose getStartPose() { return new Pose(117.757, 128.748, Math.toRadians(45)); }

    @Override
    protected void onStart() {
        shooter.setTargetVelocity(1260);
    }

    @Override
    protected void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(117.757, 128.748),
                        new Pose(94.879, 107.215)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(94.879, 107.215),
                        new Pose(102.028, 91.374)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(102.028, 91.374),
                        new Pose(121.579, 90.850)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(121.579, 90.850),
                        new Pose(81.318, 75.327)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(81.318, 75.327),
                        new Pose(102.402, 60)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(102.402, 60),
                        new Pose(124.561, 60)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(124.561, 60),
                        new Pose(82.187, 75.131)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(82.187, 75.131),
                        new Pose(129, 64)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(38))
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(128, 61.5),
                        new Pose(81.636, 75.290)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Drive to first shoot position
                follower.followPath(path1, true);
                intake.off();
                setPathState(1);
                break;

            case 1: // Wait + shoot 1
                if (h.pathDone(1.5)) { h.fireShot(); setPathState(2); }
                break;

            case 2: // Drive path2, no intake
                if (shooter.isIdle()) { follower.followPath(path2, true); setPathState(3); }
                break;

            case 3: // Path2 done — intake on, collect via path3
                if (h.pathDone(0)) { h.startCollect(path3, 1.0); setPathState(4); }
                break;

            case 4: // Path3 done — intake off, return via path4
                if (h.pathDone(0)) { intake.off(); follower.followPath(path4, true); setPathState(5); }
                break;

            case 5: // Wait + shoot 2
                if (h.pathDone(1.5)) { h.fireShot(); setPathState(6); }
                break;

            case 6: // Wait + shoot 3 (same position)
                if (shooter.isIdle() && h.timePassed(1.5)) { h.fireShot(); setPathState(7); }
                break;

            case 7: // Drive path5, no intake
                if (shooter.isIdle()) { follower.followPath(path5, true); setPathState(8); }
                break;

            case 8: // Path5 done — intake on, collect via path6
                if (h.pathDone(0)) { h.startCollect(path6, 1.0); setPathState(9); }
                break;

            case 9: // Path6 done — intake off, return via path7
                if (h.pathDone(0)) { intake.off(); follower.followPath(path7, true); setPathState(10); }
                break;

            case 10: // Wait + shoot 4
                if (h.pathDone(1.5)) { h.fireShot(); setPathState(11); }
                break;

            case 11: // Drive path8, intake on (collect)
                if (shooter.isIdle()) { h.startCollect(path8, 1.0); setPathState(12); }
                break;

            case 12: // Path8 done — intake on during 2500ms wait, then off, return via path9
                if (h.pathDone(2.5)) { intake.off(); follower.followPath(path9, true); setPathState(13); }
                break;

            case 13: // Wait + shoot 5
                if (h.pathDone(1.5)) { h.fireShot(); setPathState(14); }
                break;

            case 14: // Done — pose saved in AutoBase.stop()
                break;
        }
    }
}
