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

    private PathChain path1, path2, path3, path4;

    @Override protected boolean isRedAlliance() { return true; }
    @Override protected Pose getStartPose() { return new Pose(117.6822429, 128.6728971, Math.toRadians(45)); }

    @Override
    protected void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(117.6822429, 128.6728971),
                        new Pose(91.4392523, 91.1775700)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .setGlobalDeceleration()
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(91.4392523, 91.1775700),
                        new Pose(94.9813, 62.6261),
                        new Pose(103.7663551, 62.9158878)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .setGlobalDeceleration()
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(103.7663551, 62.9158878),
                        new Pose(128.7, 58.6)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(25))
                .setGlobalDeceleration()
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(128.7, 58.6),
                        new Pose(91.6168224, 91.0467289)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(22), Math.toRadians(0))
                .setGlobalDeceleration()
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Drive to path1 end
                follower.followPath(path1, true);
                setPathState(1);
                break;

            case 1: // Wait 1450ms
                if (h.pathDone(1.45)) { follower.followPath(path2, true); setPathState(2); }
                break;

            case 2: // Path2 done — turn intake on, wait 2000ms
                if (h.pathDone(0)) { intake.on(); setPathState(3); }
                break;

            case 3: // Intake running, follow path3 immediately
                if (h.pathDone(0)) { follower.followPath(path3, true); setPathState(4); }
                break;

            case 4: // Path3 done — wait 2000ms then follow path4
                if (h.pathDone(2.0)) { follower.followPath(path4, true); setPathState(5); }
                break;

            case 5: // Done
                break;
        }
    }
}
