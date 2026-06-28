package org.firstinspires.ftc.teamcode.OpModes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SubSystems.FieldConstants;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Localizer;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.SubSystems.TurretServo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Base class for all autonomous OpModes.
 *
 * Subclasses only implement:
 *   - getStartPose()           — unique starting position
 *   - buildPaths()             — unique path definitions
 *   - autonomousPathUpdate()   — unique state machine
 *
 * Optionally override:
 *   - isRedAlliance()          — default true
 *   - updateShooter()          — default odometry-based velocity/hood
 *   - onStart()                — extra init after super.start()
 */
public abstract class AutoBase extends OpMode {

    protected Follower    follower;
    protected Intake      intake;
    protected Shooter     shooter;
    protected TurretServo turretServo;
    protected Localizer   localizer;
    protected AutoHelper  h;
    protected Timer       pathTimer;
    protected int         pathState = 0;

    protected Pose   goalPose;
    protected double autoAimOffsetDeg = 0;

    protected abstract Pose getStartPose();
    protected abstract void buildPaths();
    protected abstract void autonomousPathUpdate();

    protected boolean isRedAlliance() { return true; }

    protected void onStart() {}

    protected void updateShooter() {
        Pose tag = FieldConstants.getTag(isRedAlliance());
        Pose cur = follower.getPose();
        double dist = Math.hypot(tag.getX() - cur.getX(), tag.getY() - cur.getY());
        if (dist > 0) {
            shooter.updateVelocity(dist);
            shooter.updateHood(dist);
        } else {
            shooter.setTargetVelocity(1250);
            shooter.setHoodPosition(0.4);
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        localizer = Localizer.getInstance(hardwareMap);

        intake      = new Intake(hardwareMap);
        shooter     = new Shooter(hardwareMap);
        turretServo = new TurretServo(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.update();

        follower.setStartingPose(getStartPose());

        pathTimer = new Timer();
        goalPose  = FieldConstants.getGoal(isRedAlliance());

        h = new AutoHelper(follower, shooter, intake, turretServo, pathTimer);

        buildPaths();

        shooter.setFollower(follower);
        shooter.setGoal(goalPose.getX(), goalPose.getY());
    }

    @Override
    public void start() {
        Turret.markAutoHandoff();
        Localizer.markAutoPoseHandoff();
        pathTimer.resetTimer();
        turretServo.returnToCenter();
        setPathState(0);
        onStart();
    }

    @Override
    public void loop() {
        follower.update();
        localizer.update();

        // Servo turret auto-aim (odometry only) — same transform as TurretAimer.calculateTargetAngle()
        Pose cur = follower.getPose();
        double dx  = goalPose.getX() - cur.getX();
        double dy  = goalPose.getY() - cur.getY();
        double h   = cur.getHeading();
        double rX  =  dx * Math.cos(h) + dy * Math.sin(h);
        double rY  = -dx * Math.sin(h) + dy * Math.cos(h);
        turretServo.setTargetAngle(-Math.toDegrees(Math.atan2(rY, rX)) + autoAimOffsetDeg);

        updateShooter();

        shooter.updatePID();
        shooter.updateFSM(intake);

        autonomousPathUpdate();

        Pose tag = FieldConstants.getTag(isRedAlliance());
        double distTelem = Math.hypot(tag.getX() - cur.getX(), tag.getY() - cur.getY());

        telemetry.addData("Auto",             getClass().getSimpleName());
        telemetry.addData("Path state",       pathState);
        telemetry.addData("Dist to tag",      "%.1f\"",       distTelem);
        telemetry.addData("Target velocity",  "%.0f ticks/s", shooter.getTargetVelocity());
        telemetry.addData("Current velocity", "%.0f ticks/s", shooter.getCurrentVelocity());
        telemetry.addData("Hood position",    "%.3f",         shooter.getHoodServoPosition());
        telemetry.addData("Turret target°",   "%.2f",         turretServo.getTargetAngle());
        telemetry.addData("Turret pos",       "%.4f",         turretServo.getCommandedPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (intake      != null) intake.off();
        if (shooter     != null) shooter.off();
        if (turretServo != null) turretServo.returnToCenter();

        if (follower != null) {
            follower.breakFollowing();

            final Pose snap = follower.getPose();
            if (localizer != null) {
                localizer.setPosition(
                    snap.getX(),
                    snap.getY(),
                    Math.toDegrees(snap.getHeading())
                );
            }

            final Follower  f   = follower;
            final Localizer loc = localizer;

            Thread settleThread = new Thread(() -> {
                try {
                    Thread.sleep(3000);
                    f.update();
                    Pose settled = f.getPose();
                    double jump = Math.hypot(settled.getX() - snap.getX(),
                                            settled.getY() - snap.getY());
                    if (jump < 50.0) {
                        loc.setPosition(
                            settled.getX(),
                            settled.getY(),
                            Math.toDegrees(settled.getHeading())
                        );
                    }
                } catch (Exception ignored) {}
            }, "pose-settle");
            settleThread.setDaemon(true);
            settleThread.start();
        }
    }
}
