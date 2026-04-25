package org.firstinspires.ftc.teamcode.OpModes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class AutoHelper {
    private final Follower follower;
    private final Shooter shooter;
    private final Intake intake;
    private final Turret turret;
    private final Timer pathTimer;

    public AutoHelper(Follower follower, Shooter shooter,
                      Intake intake, Turret turret, Timer pathTimer) {
        this.follower = follower;
        this.shooter = shooter;
        this.intake = intake;
        this.turret = turret;
        this.pathTimer = pathTimer;
    }

    public boolean pathDone(double minTime) {
        return !follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= minTime;
    }

    public boolean pathDoneAndIdle(double minTime) {
        return pathDone(minTime) && shooter.isIdle();
    }

    public boolean timePassed(double seconds) {
        return pathTimer.getElapsedTimeSeconds() >= seconds;
    }

    public void goToShootAtHeading(Pose shootPose, double turretAngle, double customHeading) {
        turret.setTargetAngle(turretAngle);
        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), shootPose))
                        .setConstantHeadingInterpolation(customHeading)
                        .build(),
                true);
    }

    // --- goToShoot: turretAngle всегда первым, потом followPath (одновременно) ---

    public void goToShoot(Pose shootPose, double turretAngle) {
        turret.setTargetAngle(turretAngle); // сначала турель
        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), shootPose))
                        .setConstantHeadingInterpolation(shootPose.getHeading())
                        .build(),
                true);
    }

    public void goToShoot(Pose shootPose, double speed, double turretAngle) {
        turret.setTargetAngle(turretAngle); // сначала турель
        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), shootPose))
                        .setConstantHeadingInterpolation(shootPose.getHeading())
                        .build(),
                speed, true);
    }

    public void goToShoot(Pose shootPose, double speed, double endTime, double turretAngle) {
        turret.setTargetAngle(turretAngle); // сначала турель
        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), shootPose))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), shootPose.getHeading(), endTime)
                        .build(),
                speed, true);
    }

    // --- startCollect: turretAngle опциональный ---

    public void startCollect(PathChain path, double speed) {
        intake.on();
        follower.followPath(path, speed, true);
    }

    public void startCollect(PathChain path, double speed, double turretAngle) {
        turret.setTargetAngle(turretAngle); // сначала турель
        intake.on();
        follower.followPath(path, speed, true);
    }

    // startCollect динамический (Pose) — без endTime, без turretAngle
    public void startCollect(Pose collectPose, double speed) {
        intake.on();
        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), collectPose))
                        .setConstantHeadingInterpolation(collectPose.getHeading())
                        .build(),
                speed, true);
    }

    // startCollect с динамическим построением пути (как goToShoot) — позволяет задать endTime
    public void startCollect(Pose collectPose, double speed, double endTime) {
        intake.on();
        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), collectPose))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), collectPose.getHeading(), endTime)
                        .build(),
                speed, true);
    }

    public void startCollect(Pose collectPose, double speed, double endTime, double turretAngle) {
        turret.setTargetAngle(turretAngle);
        intake.on();
        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), collectPose))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), collectPose.getHeading(), endTime)
                        .build(),
                speed, true);
    }

    public void fireShot() {
        intake.off();
        shooter.startShoot();
    }
}
