package org.firstinspires.ftc.teamcode.OpModes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.SubSystems.FieldConstants;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Localizer;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
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
 *   - updateShooter()          — default fixed velocity/hood values
 *   - onStart()                — extra init after super.start()
 */
public abstract class AutoBase extends OpMode {

    protected Follower follower;
    protected Intake intake;
    protected Shooter shooter;
    protected Turret turret;
    protected Localizer localizer;
    protected AutoHelper h;
    protected Timer pathTimer;
    protected int pathState = 0;

    protected abstract Pose getStartPose();
    protected abstract void buildPaths();
    protected abstract void autonomousPathUpdate();

    protected boolean isRedAlliance() { return true; }

    protected void onStart() {}

    /**
     * Called every loop to update shooter velocity/hood based on distance to tag.
     */
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

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.update();

        turret = new Turret(hardwareMap, follower);

        follower.setStartingPose(getStartPose());

        pathTimer = new Timer();
        h = new AutoHelper(follower, shooter, intake, turret, pathTimer);

        buildPaths();

        Pose goal = FieldConstants.getGoal(isRedAlliance());
        shooter.setFollower(follower);
        shooter.setGoal(goal.getX(), goal.getY());
        turret.setGoalPose(goal);
    }

    @Override
    public void start() {
        turret.resetEncoder();
        pathTimer.resetTimer();
        turret.setTargetAngle(0.0);
        setPathState(0);
        onStart();
    }

    @Override
    public void loop() {
        follower.update();
        localizer.update();
        turret.autoAim();

        updateShooter();

        shooter.updatePID();
        shooter.updateFSM(intake);

        autonomousPathUpdate();

        Pose tag = FieldConstants.getTag(isRedAlliance());
        Pose cur = follower.getPose();
        double distTelem = Math.hypot(tag.getX() - cur.getX(), tag.getY() - cur.getY());

        telemetry.addData("Auto", getClass().getSimpleName());
        telemetry.addData("Path state", pathState);
        telemetry.addData("Dist to tag", "%.1f\"", distTelem);
        telemetry.addData("Target velocity", "%.0f ticks/s", shooter.getTargetVelocity());
        telemetry.addData("Current velocity", "%.0f ticks/s", shooter.getCurrentVelocity());
        telemetry.addData("Hood position", "%.3f", shooter.getHoodServoPosition());

        // ── Turret diagnostics ────────────────────────────────────────────────
        telemetry.addData("Turret target°", "%.2f", turret.getTargetAngle());
        telemetry.addData("Turret current°", "%.2f", turret.getCurrentAngle());
        telemetry.addData("Turret odom°", "%.2f", turret.getCalculatedTargetAngle());
        telemetry.addData("Turret physics?", turret.hasPhysicsShot());
        // ─────────────────────────────────────────────────────────────────────

        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop subsystems immediately
        if (intake  != null) intake.off();
        if (shooter != null) shooter.off();
        if (turret  != null) turret.stop();

        if (follower != null) {
            follower.breakFollowing();

            // Save position immediately as fallback
            final Pose snap = follower.getPose();
            if (localizer != null) {
                localizer.setPosition(
                    snap.getX(),
                    snap.getY(),
                    Math.toDegrees(snap.getHeading())
                );
            }

            // 3-second settle thread: robot may still be coasting when auto ends.
            // After 3 s, update Localizer with the true resting position.
            // TeleOp reads this singleton at init, so it starts from where the robot actually stopped.
            final Follower  f   = follower;
            final Localizer loc = localizer;

            Thread settleThread = new Thread(() -> {
                try {
                    Thread.sleep(3000);
                    f.update();
                    Pose settled = f.getPose();
                    // Sanity check: if Pinpoint was reset by TeleOp starting early,
                    // the position would jump > 50". In that case keep the snap position.
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
