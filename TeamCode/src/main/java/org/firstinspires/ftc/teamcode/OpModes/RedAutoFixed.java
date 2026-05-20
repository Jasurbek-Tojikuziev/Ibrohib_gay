package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Auto Fixed", group = "Autonomous", preselectTeleOp = "RED Alliance TeleOp")
public class RedAutoFixed extends LinearOpMode {

    private static final double FIXED_TURRET_ANGLE = 0.0;
    private static final double TARGET_VELOCITY    = 1250;
    private static final double HOOD_POSITION      = 0.4;
    private static final double DRIVE_BACK_POWER   = 0.4;
    private static final double DRIVE_BACK_SECONDS = 0.7;
    private static final double SHOOT_SECONDS      = 1.5;
    private static final double STRAFE_POWER       = 0.35;
    private static final double STRAFE_SECONDS     = 1.0;

    @Override
    public void runOpMode() {
        Intake  intake  = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);

        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(130, 76, Math.toRadians(0)));
        follower.update();
        follower.startTeleopDrive(true);

        Turret turret = new Turret(hardwareMap, follower);
        turret.setTargetAngle(FIXED_TURRET_ANGLE);

        shooter.setTargetVelocity(TARGET_VELOCITY);
        shooter.setHoodPosition(HOOD_POSITION);

        telemetry.addLine("Red Auto Fixed — Ready");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        ElapsedTime timer = new ElapsedTime();

        // ── Phase 1: drive backward 0.7s ─────────────────────────────────────
        while (opModeIsActive() && timer.seconds() < DRIVE_BACK_SECONDS) {
            follower.setTeleOpDrive(-DRIVE_BACK_POWER, 0, 0, false);
            follower.update();
            shooter.updatePID();
            shooter.updateFSM(intake);
            turret.maintainTarget();
            telemetry.addLine("Driving backward...");
            telemetry.addData("Velocity", "%.0f / %.0f", shooter.getCurrentVelocity(), shooter.getTargetVelocity());
            telemetry.update();
        }
        follower.setTeleOpDrive(0, 0, 0, false);
        follower.update();
        sleep(100);

        // ── Phase 2: shoot for 1.5s ───────────────────────────────────────────
        timer.reset();
        while (opModeIsActive() && timer.seconds() < SHOOT_SECONDS) {
            follower.setTeleOpDrive(0, 0, 0, false);
            follower.update();
            shooter.updatePID();
            shooter.updateFSM(intake);
            turret.maintainTarget();
            if (shooter.isIdle()) shooter.startShoot();
            telemetry.addLine("Shooting...");
            telemetry.addData("Time left", "%.1f s", SHOOT_SECONDS - timer.seconds());
            telemetry.addData("Velocity", "%.0f / %.0f", shooter.getCurrentVelocity(), shooter.getTargetVelocity());
            telemetry.update();
        }

        // ── Phase 3: strafe RIGHT (Red) — negative = right in Pedro convention ─
        timer.reset();
        while (opModeIsActive() && timer.seconds() < STRAFE_SECONDS) {
            follower.setTeleOpDrive(0, -STRAFE_POWER, 0, false);
            follower.update();
            shooter.updatePID();
            shooter.updateFSM(intake);
            turret.maintainTarget();
            telemetry.addLine("Strafing right...");
            telemetry.update();
        }
        follower.setTeleOpDrive(0, 0, 0, false);
        follower.update();

        shooter.off();
        turret.stop();
        intake.off();
    }
}
