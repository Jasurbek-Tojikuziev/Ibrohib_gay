package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Leave Autonomous — timed drive only (no encoders).
 *
 * Drives straight forward for 0.7 seconds, then waits until 27 seconds
 * have elapsed from Start.
 */
@Autonomous(name = "Leave", group = "Auto")
public class LeaveAuto extends LinearOpMode {

    private static final double DRIVE_DURATION = 0.7;   // seconds to drive forward
    private static final double AUTO_DURATION  = 27.0;  // seconds total from St+  art
    private static final double DRIVE_POWER    = 0.8;

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Leave Auto ready");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // ── Drive forward 0.7 s ───────────────────────────────────────────────
        setPower(DRIVE_POWER);
        while (opModeIsActive() && runtime.seconds() < DRIVE_DURATION) {
            telemetry.addData("Status", "Driving");
            telemetry.addData("Time", "%.2fs", runtime.seconds());
            telemetry.update();
        }
        setPower(0);

        // ── Wait until 27 s ───────────────────────────────────────────────────
        while (opModeIsActive() && runtime.seconds() < AUTO_DURATION) {
            telemetry.addData("Status", "Waiting");
            telemetry.addData("Time", "%.1fs / %.0fs", runtime.seconds(), AUTO_DURATION);
            telemetry.update();
        }
    }

    private void setPower(double power) {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }
}
