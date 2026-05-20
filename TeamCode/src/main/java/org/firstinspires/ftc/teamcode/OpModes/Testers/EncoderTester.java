package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="[TEST] Encoder Ticks", group="Testers")
public class EncoderTester extends LinearOpMode {

    private DcMotorEx turretMotor;
    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    private DcMotorEx intake;
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightRear;

    @Override
    public void runOpMode() {
        turretMotor   = hardwareMap.get(DcMotorEx.class, "turretMotor");
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        intake        = hardwareMap.get(DcMotorEx.class, "Intake");
        leftFront     = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront    = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear      = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear     = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Keep encoders intact — do not reset
        for (DcMotorEx m : new DcMotorEx[]{turretMotor, shooterMotor1, shooterMotor2,
                intake, leftFront, rightFront, leftRear, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addLine("=== ENCODER TICKS TESTER ===");
        telemetry.addLine("B: Reset all encoder counts to zero");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.b) {
                for (DcMotorEx m : new DcMotorEx[]{turretMotor, shooterMotor1, shooterMotor2,
                        intake, leftFront, rightFront, leftRear, rightRear}) {
                    m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }

            telemetry.addLine("--- TURRET ---");
            telemetry.addData("turretMotor ticks", turretMotor.getCurrentPosition());
            telemetry.addData("turretMotor velocity", "%.1f t/s", turretMotor.getVelocity());

            telemetry.addLine();
            telemetry.addLine("--- SHOOTER ---");
            telemetry.addData("shooterMotor1 ticks", shooterMotor1.getCurrentPosition());
            telemetry.addData("shooterMotor1 velocity", "%.1f t/s", shooterMotor1.getVelocity());
            telemetry.addData("shooterMotor2 ticks", shooterMotor2.getCurrentPosition());
            telemetry.addData("shooterMotor2 velocity", "%.1f t/s", shooterMotor2.getVelocity());

            telemetry.addLine();
            telemetry.addLine("--- INTAKE ---");
            telemetry.addData("intake ticks", intake.getCurrentPosition());

            telemetry.addLine();
            telemetry.addLine("--- DRIVE ---");
            telemetry.addData("leftFront  ticks", leftFront.getCurrentPosition());
            telemetry.addData("rightFront ticks", rightFront.getCurrentPosition());
            telemetry.addData("leftRear   ticks", leftRear.getCurrentPosition());
            telemetry.addData("rightRear  ticks", rightRear.getCurrentPosition());

            telemetry.addLine();
            telemetry.addData("B", "Reset all encoders");

            telemetry.update();
        }
    }
}
