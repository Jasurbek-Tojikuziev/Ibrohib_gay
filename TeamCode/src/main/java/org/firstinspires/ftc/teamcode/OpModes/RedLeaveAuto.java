package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Leave Auto", group = "Autonomous", preselectTeleOp = "RED Alliance TeleOp")
public class RedLeaveAuto extends LinearOpMode {

    private static final double DRIVE_BACK_POWER   = 0.7;
    private static final double DRIVE_BACK_SECONDS = 0.7;
    private static final double FIXED_TURRET_ANGLE = 0.0;

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(130, 76, Math.toRadians(0)));
        follower.update();
        follower.startTeleopDrive(true);

        Turret turret = new Turret(hardwareMap, follower);
        turret.setTargetAngle(FIXED_TURRET_ANGLE);

        telemetry.addLine("Red Leave Auto — Ready");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < DRIVE_BACK_SECONDS) {
            follower.setTeleOpDrive(-DRIVE_BACK_POWER, 0, 0, false);
            follower.update();
            turret.maintainTarget();
            telemetry.addLine("Leaving starting zone...");
            telemetry.addData("Time", "%.1f / %.1f s", timer.seconds(), DRIVE_BACK_SECONDS);
            telemetry.update();
        }

        follower.setTeleOpDrive(0, 0, 0, false);
        follower.update();
        turret.stop();
    }
}
