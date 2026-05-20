package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "[TEST] Drivetrain (Pedro)", group = "Testers")
public class DrivetrainTester extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();

        telemetry.addLine("Pedro Pathing drivetrain test");
        telemetry.addLine("Left stick: forward/strafe  |  Right stick X: turn");
        telemetry.addLine("Field-centric ON");
        telemetry.update();

        waitForStart();

        follower.startTeleopDrive();

        double startX = 0, startY = 0;
        boolean positionRecorded = false;

        while (opModeIsActive()) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );
            follower.update();

            Pose pose = follower.getPose();

            // Record position when driver first touches left_stick_x (strafing)
            if (!positionRecorded && Math.abs(gamepad1.left_stick_x) > 0.1) {
                startX = pose.getX();
                startY = pose.getY();
                positionRecorded = true;
            }
            if (gamepad1.a) {
                startX = pose.getX();
                startY = pose.getY();
                positionRecorded = true;
            }

            telemetry.addLine("=== DRIVETRAIN ENCODER DEBUG ===");
            telemetry.addLine("Left stick: forward/strafe | Right stick X: turn");
            telemetry.addLine();

            telemetry.addLine("--- POSE ---");
            telemetry.addData("X (strafe axis)", "%.3f in", pose.getX());
            telemetry.addData("Y (forward axis)", "%.3f in", pose.getY());
            telemetry.addData("Heading", "%.2f deg", Math.toDegrees(pose.getHeading()));
            telemetry.addLine();

            if (positionRecorded) {
                double dX = pose.getX() - startX;
                double dY = pose.getY() - startY;
                telemetry.addLine("--- DELTA from strafe start (A=reset) ---");
                telemetry.addData("dX (strafe encoder)", "%.3f in", dX);
                telemetry.addData("dY (forward encoder)", "%.3f in", dY);
                telemetry.addLine();

                if (Math.abs(gamepad1.left_stick_x) > 0.1) {
                    if (Math.abs(dX) < 0.5 && Math.abs(dY) < 0.5)
                        telemetry.addLine("!! STRAFE ENCODER NOT COUNTING !!");
                    else if (Math.abs(dX) > Math.abs(dY))
                        telemetry.addLine("OK: X axis counting (strafe encoder works)");
                    else
                        telemetry.addLine("WRONG AXIS: Y counting during strafe — check pod orientation");
                }
            } else {
                telemetry.addLine("Strafe left_stick_x to record start, or press A");
            }

            telemetry.addLine();
            telemetry.addLine("--- STICKS ---");
            telemetry.addData("left_stick_y  (forward)", "%.2f", -gamepad1.left_stick_y);
            telemetry.addData("left_stick_x  (strafe)",  "%.2f", -gamepad1.left_stick_x);
            telemetry.addData("right_stick_x (turn)",    "%.2f", -gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}
