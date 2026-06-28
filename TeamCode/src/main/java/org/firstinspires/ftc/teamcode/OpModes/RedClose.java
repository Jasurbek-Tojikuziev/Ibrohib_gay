package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.OpModes.auto.AutoBase;

//@Disabled
@Autonomous(name = "red_close", group = "Autonomous", preselectTeleOp = "RED Alliance TeleOp")
public class RedClose extends AutoBase {

    // path45  = path4 + path5  chained (cycle 2 outbound, no stop in the middle)
    // path78  = path7 + path8  chained (cycle 3 outbound, no stop in the middle)
    // path910 = path9 + path10 chained (cycle 3 return,   no stop in the middle)
    private PathChain path1, path2, path3, path45, path6, path78, path910;

    @Override protected boolean isRedAlliance() { return true; }
    @Override protected Pose getStartPose() { return new Pose(117.743, 128.697, Math.toRadians(45)); }

    private static final double TURRET_OFFSET_DEG = -2.5;

    private static final double GATE_STALL_VEL     = 4.0;
    private static final double GATE_PUSH_MIN_TIME = 0.6;
    private static final double GATE_PUSH_MAX_TIME = 3.0;

    @Override
    protected void onStart() {
        autoAimOffsetDeg = TURRET_OFFSET_DEG;
    }

    @Override
    protected void buildPaths() {

        // Path 1 — start → first shoot position
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(117.743, 128.697), new Pose(88.066, 84.398)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        // Path 2 — shoot → gate (cycle 1)
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(88.066, 84.398), new Pose(125.768, 83.390)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 3 — gate → shoot (cycle 1 return)
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(125.768, 83.390), new Pose(87.834, 84.344)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 45 — shoot → lower → gate (cycle 2 outbound, chained = no stop in middle)
        path45 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(87.834, 84.344), new Pose(88.112, 60.689)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(new Pose(88.112, 60.689), new Pose(124.888, 59.506)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 6 — gate → shoot (cycle 2 return)
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(124.888, 59.506), new Pose(86.942, 84.340)))
                .setLinearHeadingInterpolation(Math.toRadians(335), Math.toRadians(335))
                .build();

        // Path 78 — shoot → mid → gate (cycle 3 outbound, chained = no stop in middle)
        path78 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(86.942, 84.340), new Pose(118.830, 63.307)))
                .setLinearHeadingInterpolation(Math.toRadians(335), Math.toRadians(335))
                .addPath(new BezierLine(new Pose(118.830, 63.307), new Pose(124.759, 59.610)))
                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(25))
                .build();

        // Path 910 — gate → mid → shoot (cycle 3 return, chained = no stop in middle)
        path910 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(124.759, 59.610), new Pose(118.734, 63.299)))
                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(335))
                .addPath(new BezierLine(new Pose(118.734, 63.299), new Pose(87.041, 84.207)))
                .setLinearHeadingInterpolation(Math.toRadians(335), Math.toRadians(335))
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {

            // ── Preload ──────────────────────────────────────────────────────────
            case 0: follower.followPath(path1, true); setPathState(1); break;
            case 1: if (h.pathDone(0)) { shooter.startShoot(); setPathState(2); } break;
            case 2: if (h.timePassed(1.0)) { intake.on(); follower.followPath(path2, true); setPathState(3); } break;

            // ── Cycle 1: path2 → gate stall → path3 → shoot ─────────────────────
            case 3: if ((h.timePassed(GATE_PUSH_MIN_TIME) && follower.getVelocity().getMagnitude() < GATE_STALL_VEL)
                        || h.timePassed(GATE_PUSH_MAX_TIME)) { setPathState(31); } break;
            case 31: if (h.timePassed(1.5)) { follower.followPath(path3, true); setPathState(4); } break;
            case 4: if (h.pathDone(0)) { intake.off(); shooter.startShoot(); setPathState(5); } break;
            case 5: if (h.timePassed(1.0)) { intake.on(); follower.followPath(path45, true); setPathState(6); } break;

            // ── Cycle 2: path45 → gate stall → path6 → shoot ────────────────────
            case 6: if ((h.timePassed(GATE_PUSH_MIN_TIME) && follower.getVelocity().getMagnitude() < GATE_STALL_VEL)
                        || h.timePassed(GATE_PUSH_MAX_TIME)) { setPathState(61); } break;
            case 61: if (h.timePassed(1.5)) { follower.followPath(path6, true); setPathState(7); } break;
            case 7: if (h.pathDone(0)) { intake.off(); shooter.startShoot(); setPathState(8); } break;
            case 8: if (h.timePassed(1.0)) { intake.on(); follower.followPath(path78, true); setPathState(9); } break;

            // ── Cycle 3: path78 → gate stall → path910 → shoot ──────────────────
            case 9: if ((h.timePassed(GATE_PUSH_MIN_TIME) && follower.getVelocity().getMagnitude() < GATE_STALL_VEL)
                        || h.timePassed(GATE_PUSH_MAX_TIME)) { setPathState(91); } break;
            case 91: if (h.timePassed(1.5)) { follower.followPath(path910, true); setPathState(10); } break;
            case 10: if (h.pathDone(0)) { intake.off(); shooter.startShoot(); setPathState(11); } break;
            case 11: if (h.timePassed(1.0)) { setPathState(12); } break;
            case 12: break; // done
        }
    }
}
