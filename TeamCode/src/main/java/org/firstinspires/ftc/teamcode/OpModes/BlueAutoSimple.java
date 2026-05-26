package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.FieldConstants;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Localizer;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Auto Simple (3 balls)", group = "Autonomous", preselectTeleOp = "BLUE Alliance TeleOp")
public class BlueAutoSimple extends LinearOpMode {

    private static final int    BALLS_TO_SHOOT   = 3;
    private static final double SPINUP_SECONDS   = 2.5;
    private static final double DRIVE_POWER      = 0.35;
    private static final double DRIVE_DISTANCE   = 30.0; // inches
    private static final double DRIVE_TIMEOUT    = 4.0;  // seconds safety timeout

    private DcMotor  leftFront, leftRear, rightFront, rightRear;
    private Shooter  shooter;
    private Turret   turret;
    private Intake   intake;
    private Localizer localizer;

    @Override
    public void runOpMode() {
        intake   = new Intake(hardwareMap);
        shooter  = new Shooter(hardwareMap);

        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(25.495, 129.570, Math.toRadians(144)));
        follower.update();

        // Motors have directions set by Pedro follower init above
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        localizer = Localizer.getInstance(hardwareMap);
        turret    = new Turret(hardwareMap, follower);
        turret.setGoalPose(FieldConstants.BLUE_GOAL);

        telemetry.addLine("Blue Auto Simple — Ready");
        telemetry.addData("Balls", BALLS_TO_SHOOT);
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // ── Phase 1: drive backward 30 inches (odometry stop) ────────────────
        driveBackward();

        // ── Phase 2: spin up flywheel and aim turret ──────────────────────────
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < SPINUP_SECONDS) {
            follower.update();
            turret.maintainWithVisionCorrection();
            updateShooter();
            telemetry.addLine("Spinning up...");
            telemetry.addData("Velocity", "%.0f / %.0f", shooter.getCurrentVelocity(), shooter.getTargetVelocity());
            telemetry.addData("Turret",   "%.1f°", turret.getCurrentAngle());
            telemetry.update();
        }

        // ── Phase 3: shoot 3 balls ────────────────────────────────────────────
        int ballsShot = 0;
        boolean shotInProgress = false;

        while (opModeIsActive() && ballsShot < BALLS_TO_SHOOT) {
            follower.update();
            turret.maintainWithVisionCorrection();
            updateShooter();

            if (!shotInProgress && shooter.isIdle()) {
                shooter.startShoot();
                shotInProgress = true;
            } else if (shotInProgress && shooter.isIdle()) {
                ballsShot++;
                shotInProgress = false;
            }

            telemetry.addData("Balls shot", ballsShot + " / " + BALLS_TO_SHOOT);
            telemetry.addData("Velocity",   "%.0f / %.0f", shooter.getCurrentVelocity(), shooter.getTargetVelocity());
            telemetry.addData("Turret",     "%.1f°", turret.getCurrentAngle());
            telemetry.update();
        }

        // ── Phase 4: wait for last shot FSM to finish ─────────────────────────
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 2.5) {
            shooter.updateFSM(intake);
            shooter.updatePID();
        }

        // ── Save pose for TeleOp handoff ──────────────────────────────────────
        Pose finalPose = follower.getPose();
        Localizer.getInstance(hardwareMap).setPosition(
                finalPose.getX(),
                finalPose.getY(),
                Math.toDegrees(finalPose.getHeading())
        );

        shooter.off();
        turret.stop();
        intake.off();
    }

    private void driveBackward() {
        localizer.update();
        double startX = localizer.getX();
        double startY = localizer.getY();

        leftFront.setPower(-DRIVE_POWER);
        leftRear.setPower(-DRIVE_POWER);
        rightFront.setPower(-DRIVE_POWER);
        rightRear.setPower(-DRIVE_POWER);

        ElapsedTime timeout = new ElapsedTime();

        while (opModeIsActive() && timeout.seconds() < DRIVE_TIMEOUT) {
            localizer.update();
            double dx   = localizer.getX() - startX;
            double dy   = localizer.getY() - startY;
            double dist = Math.sqrt(dx * dx + dy * dy);

            turret.maintainWithVisionCorrection();

            telemetry.addLine("Driving backward...");
            telemetry.addData("Distance", "%.2f / %.1f in", dist, DRIVE_DISTANCE);
            telemetry.update();

            if (dist >= DRIVE_DISTANCE) break;
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        sleep(150); // settle
    }

    private void updateShooter() {
        double dist = turret.getDistanceToGoal();
        if (dist > 0) {
            shooter.updateVelocity(dist);
            shooter.updateHood(dist);
        } else {
            shooter.setTargetVelocity(1250);
            shooter.setHoodPosition(0.4);
        }
        shooter.updatePID();
        shooter.updateFSM(intake);
    }
}
