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
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
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
 *   - getShooterVelocity()     — default 1250
 *   - getHoodPosition()        — default 0.4
 *   - onStart()                — extra init after super.start()
 */
public abstract class AutoBase extends OpMode {

    // === Shared subsystems (accessible by subclasses) ===
    protected Follower follower;
    protected Intake intake;
    protected Shooter shooter;
    protected Turret turret;
    protected Vision vision;
    protected Localizer localizer;
    protected AutoHelper h;
    protected Timer pathTimer;
    protected int pathState = 0;

    // === Abstract — each Auto MUST implement ===
    protected abstract Pose getStartPose();
    protected abstract void buildPaths();
    protected abstract void autonomousPathUpdate();

    // === Overrideable defaults ===
    protected boolean isRedAlliance() { return true; }

    /** Called after super.start(). Override for per-auto init (e.g. collectCycle = 0). */
    protected void onStart() {}

    /**
     * Called every loop to update shooter velocity/hood.
     * Default: fixed values (good for CloseAuto).
     * Override for different fixed values or dynamic behavior.
     */
    protected void updateShooter() {
        shooter.setTargetVelocity(1250);
        shooter.setHoodPosition(0.4);
    }

    // === State machine helper ===
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // === Lifecycle ===

    @Override
    public void init() {
        localizer = Localizer.getInstance(hardwareMap);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        vision = new Vision();
        vision.init(hardwareMap);
        vision.setAlliance(isRedAlliance());

        follower = Constants.createFollower(hardwareMap);
        follower.update(); // CRITICAL: init Pinpoint before setting pose

        turret = new Turret(hardwareMap, vision, follower);

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
        vision.start();
        pathTimer.resetTimer();

        turret.setTargetAngle(0.0);

        setPathState(0);

        onStart(); // subclass-specific init
    }

    @Override
    public void loop() {
        follower.update();
        localizer.update();
        turret.maintainWithVisionCorrection();

        updateShooter();

        shooter.updatePID();
        shooter.updateFSM(intake);

        autonomousPathUpdate();

        telemetry.addData("Auto", getClass().getSimpleName());
        telemetry.addData("Path state", pathState);
        telemetry.addData("Target velocity", "%.0f ticks/s", shooter.getTargetVelocity());
        telemetry.addData("Current velocity", "%.0f ticks/s", shooter.getCurrentVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            Pose finalPose = follower.getPose();
            localizer.setPosition(
                finalPose.getX(),
                finalPose.getY(),
                Math.toDegrees(finalPose.getHeading())
            );
            follower.breakFollowing();
        }
        if (intake != null) intake.off();
        if (shooter != null) shooter.off();
        if (turret != null) turret.stop();
        if (vision != null) vision.stop();
    }
}
