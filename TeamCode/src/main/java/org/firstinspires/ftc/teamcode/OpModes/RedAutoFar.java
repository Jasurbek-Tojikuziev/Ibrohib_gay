package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.auto.AutoBase;

@Autonomous(name="Red AutoFar", group="Autonomous", preselectTeleOp="RED Alliance TeleOp")
public class RedAutoFar extends AutoBase {

    private static final Pose START   = new Pose(78.3,             8.523,                Math.toRadians(0));
    private static final Pose COLLECT = new Pose(132.271028037383,   8.523364485981281,    Math.toRadians(0));
    private static final Pose PARK    = new Pose(107.355140186916,   8.523364485981281,    Math.toRadians(0));

    private PathChain pathToCollect, pathToStart, pathToPark;

    @Override protected boolean isRedAlliance() { return true; }
    @Override protected Pose getStartPose() { return START; }

    @Override
    protected void updateShooter() {
        double dist = turret.getDistanceToGoal();
        if (dist > 0) {
            shooter.updateVelocity(dist);
            shooter.updateHood(dist);
        } else {
            shooter.setTargetVelocity(1576);
            shooter.setHoodPosition(1.0);
        }
    }

    @Override
    protected void buildPaths() {
        pathToCollect = follower.pathBuilder()
                .addPath(new BezierLine(START, COLLECT))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pathToStart = follower.pathBuilder()
                .addPath(new BezierLine(COLLECT, START))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pathToPark = follower.pathBuilder()
                .addPath(new BezierLine(START, PARK))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // wait for flywheel to reach target velocity, then fire ball 1
                if (shooter.atSpeed()) { shooter.startShoot(); setPathState(1); }
                break;

            case 1: // fire ball 2
                if (shooter.isIdle() && h.timePassed(0.1)) { shooter.startShoot(); setPathState(2); }
                break;

            case 2: // fire ball 3
                if (shooter.isIdle() && h.timePassed(0.1)) { shooter.startShoot(); setPathState(3); }
                break;

            case 3: // all preloaded balls fired — drive to collect zone with intake on
                if (shooter.isIdle()) {
                    intake.on();
                    follower.followPath(pathToCollect, true);
                    setPathState(4);
                }
                break;

            case 4: // wait for arrival at collect zone
                if (!follower.isBusy()) { setPathState(40); }
                break;

            case 40: // intake for 3 seconds, then return to start
                if (h.timePassed(3.0)) {
                    intake.off();
                    follower.followPath(pathToStart, true);
                    setPathState(5);
                }
                break;

            case 5: // arrived at start — fire ball 1
                if (!follower.isBusy()) { shooter.startShoot(); setPathState(6); }
                break;

            case 6: // fire ball 2
                if (shooter.isIdle() && h.timePassed(0.1)) { shooter.startShoot(); setPathState(7); }
                break;

            case 7: // fire ball 3
                if (shooter.isIdle() && h.timePassed(0.1)) { shooter.startShoot(); setPathState(8); }
                break;

            case 8: // wait for last shot, then park
                if (shooter.isIdle()) {
                    intake.off();
                    follower.followPath(pathToPark, true);
                    setPathState(9);
                }
                break;

            case 9: // done — pose saved in AutoBase.stop()
                break;
        }
    }
}
