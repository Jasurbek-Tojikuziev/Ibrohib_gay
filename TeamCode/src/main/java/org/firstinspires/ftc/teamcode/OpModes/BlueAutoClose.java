package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.OpModes.auto.AutoBase;

@Disabled
@Autonomous(name="Blue AutoClose", group="Autonomous", preselectTeleOp="BLUE Alliance TeleOp")
public class BlueAutoClose extends AutoBase {

    private PathChain path1, path2, path4, path6, path8;
    private final Pose shootPose = new Pose(46.102, 87.079, Math.toRadians(180));

    @Override protected boolean isRedAlliance() { return false; }
    @Override protected Pose getStartPose() { return new Pose(26, 129, Math.toRadians(135)); }
    @Override protected void updateShooter() { shooter.setTargetVelocity(1250); shooter.setHoodPosition(0.4); }

    @Override
    protected void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(26.000, 129.000),
                                new Pose(47.922, 104.702)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        path2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(47.922, 104.702),
                                new Pose(57.297, 60.042),
                                new Pose(8.712, 62.430)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180), 0.15)
                .build();

        path4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(46.102, 87.079),
                                new Pose(68.068, 27.477),
                                new Pose(10.663, 38.908)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(45.987, 87.079),
                                new Pose(20.693, 87.011)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(45.975, 87.147),
                                new Pose(25.553, 71.819)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1, true);
                setPathState(50);
                break;

            case 50:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.5) {
                    shooter.startShoot();
                    setPathState(1);
                }
                break;

            case 1: // едет брать 6 + доезжает
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.5) {
                    intake.on();
                    follower.followPath(path2, 0.8, true);
                    turret.setTargetAngle(48);
                    setPathState(200);
                }
                break;

            case 200: // возвращается стрелять
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    turret.setTargetAngle(48);
                    intake.off();
                    follower.followPath(
                        follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .build(),
                        0.8, true);
                    setPathState(2);
                }
                break;

            case 2: // стреляет 6й
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    shooter.startShoot();
                    setPathState(50);
                }
                break;

            case 33:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0 && shooter.isIdle()) {
                    intake.on();
                    follower.followPath(path2, 0.6, true);
                    setPathState(1333);
                }
                break;

            case 1333:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    turret.setTargetAngle(48);
                    intake.off();
                    follower.followPath(
                        follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .build(),
                        0.8, true);
                    setPathState(1133);
                }
                break;

            case 1133:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    shooter.startShoot();
                    setPathState(333);
                }
                break;

            case 333:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0 && shooter.isIdle()) {
                    intake.on();
                    follower.followPath(path2, 0.6, true);
                    setPathState(1113);
                }
                break;

            case 1113:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    turret.setTargetAngle(48);
                    intake.off();
                    follower.followPath(
                        follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .build(),
                        0.8, true);
                    setPathState(1111);
                }
                break;

            case 1111:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    shooter.startShoot();
                    setPathState(3333);
                }
                break;

            case 3333:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0 && shooter.isIdle()) {
                    intake.on();
                    follower.followPath(path2, 0.6, true);
                    setPathState(33333);
                }
                break;

            case 3: // едет за 9м
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    intake.off();
                    shooter.startShoot();
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5 && shooter.isIdle()) {
                    intake.on();
                    turret.setTargetAngle(48);
                    follower.followPath(path4, 0.8, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    follower.followPath(
                        follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .build(),
                        true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    intake.off();
                    shooter.startShoot();
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5 && shooter.isIdle()) {
                    intake.on();
                    follower.followPath(path6, 0.6, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    follower.followPath(
                        follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .build(),
                        true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    intake.off();
                    shooter.startShoot();
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.9) {
                    follower.followPath(path8, true);
                    turret.setTargetAngle(0);
                    setPathState(13);
                }
                break;

            case 13: // Done — pose saved in AutoBase.stop()
                break;
        }
    }
}
