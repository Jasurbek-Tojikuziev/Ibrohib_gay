package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Motor Test — runs each motor at 0.3 power one at a time.
 *
 * Controls:
 *   X button (rising edge) — cycle to next motor
 *
 * The active motor runs at 0.3 power. All others are stopped.
 */
@TeleOp(name = "Motor Test", group = "Testers")
public class MotorTest extends OpMode {

    private DcMotor[] motors;
    private String[]  names;
    private int       activeIndex = 0;
    private boolean   lastX       = false;

    private static final double TEST_POWER = 0.3;

    @Override
    public void init() {
        names = new String[]{
            "leftFront",
            "leftRear",
            "rightFront",
            "rightRear",
            "shooterMotor1",
            "shooterMotor2",
            "turretMotor",
            "Intake"
        };

        motors = new DcMotor[names.length];
        for (int i = 0; i < names.length; i++) {
            motors[i] = hardwareMap.get(DcMotor.class, names[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setPower(0);
        }

        telemetry.addLine("Motor Test ready — press X to cycle");
        telemetry.update();
    }

    @Override
    public void loop() {

        // X (rising edge): cycle to next motor
        if (gamepad1.x && !lastX) {
            motors[activeIndex].setPower(0);
            activeIndex = (activeIndex + 1) % motors.length;
        }
        lastX = gamepad1.x;

        // Give power only to the active motor
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(i == activeIndex ? TEST_POWER : 0);
        }

        // Telemetry
        telemetry.addLine("Press X to cycle motor");
        telemetry.addLine("----------------------------");
        for (int i = 0; i < names.length; i++) {
            telemetry.addData(names[i], i == activeIndex ? "<-- RUNNING (0.3)" : "stopped");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        for (DcMotor m : motors) m.setPower(0);
    }
}
