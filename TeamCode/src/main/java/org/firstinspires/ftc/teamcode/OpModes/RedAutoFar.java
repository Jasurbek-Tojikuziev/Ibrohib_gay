package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.OpModes.auto.AutoBase;

@Disabled
@Autonomous(name="Red AutoFar", group="Autonomous", preselectTeleOp = "RED Alliance TeleOp")
public class RedAutoFar extends AutoBase {

    private PathChain path1, path3, path33, path333, path5;
    private final Pose shootPose = new Pose(87.798, 8.951, Math.toRadians(0));

    @Override protected boolean isRedAlliance() { return true; }
    @Override protected Pose getStartPose() { return new Pose(87.914, 8.100, Math.toRadians(0)); }
    @Override protected void updateShooter() { shooter.setTargetVelocity(1250); shooter.setHoodPosition(0.4); }

    @Override
    protected void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(87.914, 8.100),
                                new Pose(83.009, 42.290),
                                new Pose(124.928, 38.735)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.798, 8.951),
                                new Pose(128.061, 8.031)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path33 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.798, 8.951),
                                new Pose(128.061, 8.031)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))
                .build();

        path333 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.798, 8.951),
                                new Pose(128.061, 8.031)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-20))
                .build();

        path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.805, 8.017),
                                new Pose(100, 45)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Initial shot from start
                turret.setTargetAngle(-70);
                if (h.timePassed(1)) { shooter.startShoot(); setPathState(50); }
                break;

            case 50: // Drive to collect first ball
                if (h.pathDoneAndIdle(1.3)) {
                    intake.on();
                    follower.followPath(path1, 0.8, true);
                    setPathState(1);
                }
                break;

            case 1: // Return to shoot
                if (h.pathDone(2)) {
                    turret.setTargetAngle(-62);
                    intake.off();
                    h.goToShoot(shootPose, 0.8);
                    setPathState(2);
                }
                break;

            case 2: // Shoot
                if (h.pathDone(1.0)) { shooter.startShoot(); setPathState(3); }
                break;

            case 3: // Sweep pass 1
                if (h.pathDoneAndIdle(1.0)) {
                    intake.on();
                    follower.followPath(path3, 0.6, true);
                    setPathState(33);
                }
                break;

            case 33: // Sweep pass 2
                if (h.pathDone(0.5)) {
                    intake.on();
                    follower.followPath(path33, 0.5, true);
                    setPathState(333);
                }
                break;

            case 333: // Sweep pass 3
                if (h.pathDone(0.5)) {
                    intake.on();
                    follower.followPath(path333, 0.5, true);
                    setPathState(4);
                }
                break;

            case 4: // Return to shoot after sweep
                if (h.pathDoneAndIdle(3.5)) {
                    h.goToShoot(shootPose, 0.8);
                    turret.setTargetAngle(-62);
                    setPathState(5);
                }
                break;

            case 5: // Shoot
                if (h.pathDone(1.5)) { h.fireShot(); setPathState(6); }
                break;

            case 6: // Park
                if (h.pathDone(2.0)) {
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
