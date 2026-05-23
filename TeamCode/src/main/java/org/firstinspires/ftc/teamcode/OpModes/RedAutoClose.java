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

    private PathChain path1, path2, path3;

    @Override protected boolean isRedAlliance() { return true; }
    @Override protected Pose getStartPose() { return new Pose(117.6822429, 128.6728971, Math.toRadians(45)); }

    @Override
    protected void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(117.6822429, 128.6728971),
                        new Pose(82.6915887, 78.6168224)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(82.6915887, 78.6168224),
                        new Pose(94.9813, 62.6261),
                        new Pose(118.2056074, 59.8130841)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(118.2056074, 59.8130841),
                        new Pose(134.6261682, 61.5887850)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(25))
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

            case 3: // Intake running, wait 2000ms then follow path3
                if (h.timePassed(2.0)) { follower.followPath(path3, true); setPathState(4); }
                break;

            case 4: // Done
                break;
        }
    }
}
