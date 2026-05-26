package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.auto.AutoBase;

@Autonomous(name="Blue AutoClose", group="Autonomous", preselectTeleOp="BLUE Alliance TeleOp")
public class BlueAutoClose extends AutoBase {

    private PathChain path1, path2, path3;

    @Override protected boolean isRedAlliance() { return false; }

    @Override
    protected Pose getStartPose() {
        return new Pose(25.495, 129.570, Math.toRadians(135));
    }

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
                .addPath(new BezierLine(
                        new Pose(25.495, 129.570),
                        new Pose(47.028, 113.607)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(144))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(47.028, 113.607),
                        new Pose(38.710, 84.234)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(38.710, 84.234),
                        new Pose(14.374, 84.075)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // drive to shoot position, turret auto-aims throughout
                follower.followPath(path1, true);
                setPathState(1);
                break;

            case 1: // wait 1.5s then shoot
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    shooter.startShoot();
                    setPathState(2);
                }
                break;

            case 2: // wait for shot to finish, then drive toward intake zone
                if (shooter.isIdle() && pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    follower.followPath(path2, true);
                    setPathState(3);
                }
                break;

            case 3: // wait to reach intake position, then start intake and continue
                if (!follower.isBusy()) {
                    intake.on();
                    follower.followPath(path3, true);
                    setPathState(4);
                }
                break;

            case 4: // wait to reach final position
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;

            case 5: // done — pose saved by AutoBase.stop()
                break;
        }
    }
}
