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

    private int collectCycle = 0;
    private PathChain path1, path2, path3, path3slow, path4, path6, path8;
    private final Pose shootPose = new Pose(79, 76, Math.toRadians(-30));
    private static final double SHOOT_TURRET_ANGLE = -85;

    @Override protected boolean isRedAlliance() { return true; }
    @Override protected Pose getStartPose() { return new Pose(118.006, 128.797, Math.toRadians(45)); }
    @Override protected void updateShooter() { shooter.setTargetVelocity(1250); shooter.setHoodPosition(0.4); }

    @Override
    protected void onStart() {
        collectCycle = 0;
        shooter.setTargetVelocity(1260); // pre-spin flywheel before first loop
    }

    @Override
    protected void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(118.006, 128.797),
                                new Pose(79, 86)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .setBrakingStrength(0.6)
                .build();

        path2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(79, 86),
                                new Pose(88.000, 58.921),
                                new Pose(119.000, 57.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0), 0.15)
                .setBrakingStrength(0.8)
                .build();

        path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(79.000, 76.000),
                                new Pose(122.35, 63)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(20), 0.3)
                .build();

        path3slow = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(122.35, 63),
                                new Pose(131, 60.5)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(20))
                .build();

        path4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(79, 76),
                                new Pose(115, 86)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(79.000, 76.000),
                                new Pose(79.705, 36.174),
                                new Pose(123.000, 35.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0), 0.1)
                .build();

        path8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(79, 76),
                                new Pose(98, 76)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start: begin driving, flywheel already pre-spinning from start()
                follower.followPath(path1, 0.9, true);
                intake.on();
                setPathState(50);
                break;

            case 50: // After 0.35s flywheel is at speed
                if (h.timePassed(1.8)) { shooter.startShoot(); setPathState(1); }
                break;

            case 1: // Collect ball 6 (part 1)
                if (h.pathDone(1.0) && shooter.isIdle()) { h.startCollect(path2, 1); setPathState(2); }
                break;

            case 2: // Return to shoot ball 6
                if (h.pathDone(2.0)) { h.goToShoot(shootPose, 1.0, 0.2, -95); setPathState(3); }
                break;

            case 3: // Shoot ball 6
                if (h.pathDone(1.3)) { h.fireShot(); setPathState(300); }
                break;

            case 300: // Fast part — drive toward sample
                if (h.pathDone(1.0)) { intake.on(); follower.followPath(path3, 1.0, true); setPathState(301); }
                break;

            case 301: // Slow part — approach sample gently
                if (h.pathDone(0.1) && !follower.isBusy()) { follower.followPath(path3slow, 0.5, true); setPathState(3000); }
                break;

            case 3000:
                if (h.pathDone(2.0)) { h.goToShoot(shootPose, 1.0, 0.05, -95); setPathState(4000); }
                break;

            case 4000: // Shoot, then repeat or continue
                if (h.pathDone(1.3)) {
                    h.fireShot();
                    collectCycle++;
                    if (collectCycle < 2) {
                        setPathState(300); // repeat collect cycle
                    } else {
                        setPathState(4); // all cycles done → continue
                    }
                }
                break;

            case 4: // Collect ball 9
                if (h.pathDoneAndIdle(0.1)) { h.startCollect(path4, 1.0, -60); setPathState(5); }
                break;

            case 5: // Return to shoot ball 9
                if (h.pathDone(1.2)) { h.goToShootAtHeading(shootPose, -60, 0); setPathState(6); }
                break;

            case 6: // Shoot ball 9
                if (h.pathDone(1.0)) { h.fireShot(); setPathState(7); }
                break;

            case 7: // Collect ball 12
                if (h.pathDoneAndIdle(1.0)) { h.startCollect(path6, 1.0); setPathState(8); }
                break;

            case 8: // Return to shoot ball 12
                if (h.pathDone(2.0)) { h.goToShoot(shootPose, SHOOT_TURRET_ANGLE); setPathState(9); }
                break;

            case 9: // Shoot ball 12
                if (h.pathDone(1.0)) { h.fireShot(); setPathState(10); }
                break;

            case 10: // Park
                if (h.pathDone(1.0)) {
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
