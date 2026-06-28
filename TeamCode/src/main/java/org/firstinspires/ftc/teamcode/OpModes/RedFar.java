package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.OpModes.auto.AutoBase;

//@Disabled
@Autonomous(name = "red_far", group = "Autonomous", preselectTeleOp = "RED Alliance TeleOp")
public class RedFar extends AutoBase {

    private PathChain path1, path2, path3, path4, path5, path6, path7,
                      path8, path9, path10, path11, path12, path13, path14;

    @Override protected boolean isRedAlliance() { return true; }

    // NOTE: start heading was null in the source paths — set to 0° as a placeholder.
    // Change this to however the robot is physically placed at the start.
    @Override protected Pose getStartPose() { return new Pose(87.684, 8.203, Math.toRadians(0)); }

    // Turret bias correction (positive = right, negative = left). Tune for first-shot center.
    private static final double TURRET_OFFSET_DEG = -2.5;

    @Override
    protected void onStart() {
        autoAimOffsetDeg = TURRET_OFFSET_DEG;
    }

    @Override
    protected void buildPaths() {

        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(87.684, 8.203), new Pose(110.75, 11.607), new Pose(129.976, 8.190)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)) // start heading was null → 0
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(132.976, 9.190), new Pose(90.803, 7.966)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(90.803, 7.966), new Pose(88.850, 42.083), new Pose(126.989, 37.609)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(126.989, 37.609), new Pose(91.010, 8.155)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(91.010, 8.155), new Pose(131.254, 7.848)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(131.254, 7.848), new Pose(90.836, 8.037)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(90.836, 8.037), new Pose(88.511, 19.632), new Pose(122.156, 17.241), new Pose(132.993, 18.291)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(132.993, 18.291), new Pose(90.956, 8.027)))
                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(90.956, 8.027), new Pose(130.070, 3.318), new Pose(127.806, 11), new Pose(127.829, 41.128)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(135.829, 45.128), new Pose(91.004, 8.827)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        path11 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(91.004, 8.827), new Pose(113.718, 38.300)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        path12 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(113.718, 38.300), new Pose(135.365, 38.825)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path13 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(135.365, 38.825), new Pose(91.193, 8.733)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        path14 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(91.193, 8.733), new Pose(106.611, 9.090)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        // Pattern (TUNE AS NEEDED): outward paths = collect (intake on),
        // return-to-home paths (~x90,y8) = shoot. Shooter velocity/hood are auto-set by AutoBase.
        switch (pathState) {

            case 0:  if (h.timePassed(2.0)) { shooter.startShoot(); setPathState(1); } break; // 2s flywheel spin-up, then preload SHOOT
            case 1:  if (h.timePassed(1.0)) { intake.on(); follower.followPath(path1, true); setPathState(2); } break; // out + collect

            case 2:  if (h.pathDone(0)) { follower.followPath(path2, true); setPathState(3); } break;     // return
            case 3:  if (h.pathDone(0)) { intake.off(); shooter.startShoot(); setPathState(4); } break;   // SHOOT
            case 4:  if (h.timePassed(1.0)) { intake.on(); follower.followPath(path3, true); setPathState(5); } break;

            case 5:  if (h.pathDone(0)) { follower.followPath(path4, true); setPathState(6); } break;     // return
            case 6:  if (h.pathDone(0)) { intake.off(); shooter.startShoot(); setPathState(7); } break;   // SHOOT
            case 7:  if (h.timePassed(1.0)) { intake.on(); follower.followPath(path5, true); setPathState(8); } break;

            case 8:  if (h.pathDone(0)) { follower.followPath(path6, true); setPathState(9); } break;     // return
            case 9:  if (h.pathDone(0)) { intake.off(); shooter.startShoot(); setPathState(10); } break;  // SHOOT
            case 10: if (h.timePassed(1.0)) { intake.on(); follower.followPath(path7, true); setPathState(11); } break;

            case 11: if (h.pathDone(0)) { follower.followPath(path8, true); setPathState(12); } break;    // return
            case 12: if (h.pathDone(0)) { intake.off(); shooter.startShoot(); setPathState(13); } break;  // SHOOT
            case 13: if (h.timePassed(1.0)) { intake.on(); follower.followPath(path9, true); setPathState(14); } break;

            case 14: if (h.pathDone(0)) { follower.followPath(path10, true); setPathState(15); } break;   // return
            case 15: if (h.pathDone(0)) { intake.off(); shooter.startShoot(); setPathState(16); } break;  // SHOOT
            case 16: if (h.timePassed(1.0)) { intake.on(); follower.followPath(path11, true); setPathState(17); } break;

            case 17: if (h.pathDone(0)) { follower.followPath(path12, true); setPathState(18); } break;   // keep collecting
            case 18: if (h.pathDone(0)) { follower.followPath(path13, true); setPathState(19); } break;   // return
            case 19: if (h.pathDone(0)) { intake.off(); shooter.startShoot(); setPathState(20); } break;  // SHOOT
            case 20: if (h.timePassed(1.0)) { follower.followPath(path14, true); setPathState(21); } break; // reposition / park

            case 21: if (h.pathDone(0)) { intake.off(); setPathState(22); } break; // parked
            case 22: break; // done
        }
    }
}
