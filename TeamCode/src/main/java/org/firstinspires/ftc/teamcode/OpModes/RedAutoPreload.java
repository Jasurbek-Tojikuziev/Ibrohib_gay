package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.auto.AutoBase;

@Autonomous(name = "Red Auto Preload", group = "Autonomous", preselectTeleOp = "RED Alliance TeleOp")
public class RedAutoPreload extends AutoBase {

    private PathChain path1, path2, path3, path4, path5;

    private static final Pose START_POSE        = new Pose(120.37,              126.87,              Math.toRadians(36));
    private static final Pose SHOOT_POSE_1      = new Pose(89.42056074766356,   105.53271028037383,  Math.toRadians(36));
    private static final Pose COLLECT_APPROACH  = new Pose(95.21495327102804,   87.44859813084112,   Math.toRadians(0));
    private static final Pose COLLECT_POSE      = new Pose(128.392523364486,    87.44859813084112,   Math.toRadians(0));
    private static final Pose SHOOT_POSE_2      = new Pose(89.24299065420561,   105.57009345794393,  Math.toRadians(36));
    private static final Pose PARK_POSE         = new Pose(101.74766355140187,  78.72897196261682,   Math.toRadians(0));

    @Override protected boolean isRedAlliance() { return true; }

    @Override
    protected Pose getStartPose() { return START_POSE; }

    @Override
    protected void updateShooter() {
        double dist = turret.getDistanceToGoal();
        if (dist > 0) {
            shooter.updateVelocity(dist);
            shooter.updateHood(dist);
        } else {
            shooter.setTargetVelocity(1250);
            shooter.setHoodPosition(0.4);
        }
    }

    @Override
    protected void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOT_POSE_1))
                .setConstantHeadingInterpolation(Math.toRadians(36))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POSE_1, COLLECT_APPROACH))
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(COLLECT_APPROACH, COLLECT_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(COLLECT_POSE, SHOOT_POSE_2))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POSE_2, PARK_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // drive to first shoot position
                follower.followPath(path1, true);
                setPathState(1);
                break;

            case 1: // wait for arrival
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2: // shoot preloaded balls for 1.5s
                if (shooter.isIdle()) {
                    shooter.startShoot();
                }
                if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    follower.followPath(path2, true);
                    setPathState(3);
                }
                break;

            case 3: // drive to collect approach, wait 0.1s after arrival
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    intake.on();
                    follower.followPath(path3, true);
                    setPathState(4);
                }
                break;

            case 4: // drive forward collecting
                if (!follower.isBusy()) {
                    intake.off();
                    follower.followPath(path4, true);
                    setPathState(5);
                }
                break;

            case 5: // drive to second shoot position
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;

            case 6: // shoot collected balls for 1.5s
                if (shooter.isIdle()) {
                    shooter.startShoot();
                }
                if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    intake.off();
                    follower.followPath(path5, true);
                    setPathState(7);
                }
                break;

            case 7: // park — wait until auto ends
                break;
        }
    }
}
