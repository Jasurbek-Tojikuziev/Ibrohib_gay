package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.OpModes.auto.AutoBase;

@Disabled
@Autonomous(name="Blue AutoFar", group="Autonomous", preselectTeleOp="BLUE Alliance TeleOp")
public class BlueAutoFar extends AutoBase {

    private PathChain path1, path3, path5;
    private final Pose shootPose = new Pose(57.684, 8.357, Math.toRadians(-180));

    @Override protected boolean isRedAlliance() { return false; }
    @Override protected Pose getStartPose() { return new Pose(57.684, 8.357, Math.toRadians(-180)); }
    @Override protected void updateShooter() { shooter.setTargetVelocity(1250); shooter.setHoodPosition(0.4); }

    @Override
    protected void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(57.684, 8.357),
                                new Pose(17, 9.169)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                .build();

        path3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(57.681, 8.327),
                                new Pose(61.719, 39.423),
                                new Pose(19.480, 37.850)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                .build();

        path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(57.449, 10.740),
                                new Pose(44.539, 13.272)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(90))
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                turret.setTargetAngle(70);
                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    shooter.startShoot();
                    setPathState(50);
                }
                break;

            case 50:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.3 && shooter.isIdle()) {
                    intake.on();
                    follower.followPath(path1, 0.8, true);
                    setPathState(1);
                }
                break;

            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    turret.setTargetAngle(70);
                    intake.off();
                    follower.followPath(
                        follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setConstantHeadingInterpolation(Math.toRadians(-180))
                            .build(),
                        0.8, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    shooter.startShoot();
                    setPathState(50);
                }
                break;

            case 33:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0 && shooter.isIdle()) {
                    intake.on();
                    follower.followPath(path1, 0.6, true);
                    setPathState(1333);
                }
                break;

            case 1333:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    turret.setTargetAngle(70);
                    intake.off();
                    follower.followPath(
                        follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setConstantHeadingInterpolation(Math.toRadians(-180))
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
                    follower.followPath(path1, 0.6, true);
                    setPathState(1113);
                }
                break;

            case 1113:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    turret.setTargetAngle(70);
                    intake.off();
                    follower.followPath(
                        follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setConstantHeadingInterpolation(Math.toRadians(-180))
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
                    follower.followPath(path1, 0.6, true);
                    setPathState(33333);
                }
                break;

            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0 && shooter.isIdle()) {
                    intake.on();
                    follower.followPath(path3, 0.8, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5 && shooter.isIdle()) {
                    follower.followPath(
                        follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setConstantHeadingInterpolation(Math.toRadians(-180))
                            .build(),
                        0.8, true);
                    turret.setTargetAngle(70);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    intake.off();
                    shooter.startShoot();
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    follower.followPath(path5, true);
                    turret.setTargetAngle(0);
                    setPathState(13);
                }
                break;

            case 13: // Done — pose saved in AutoBase.stop()
                break;
        }
    }
}
