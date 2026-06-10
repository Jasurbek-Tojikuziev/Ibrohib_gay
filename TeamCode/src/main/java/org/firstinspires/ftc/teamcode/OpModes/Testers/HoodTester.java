package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Hood servo tester.
 *   Hold Dpad Up   → hood position increases (continuous, smooth)
 *   Hold Dpad Down → hood position decreases
 *   Bumpers        → fine single step (±0.01)
 */
@Config
@TeleOp(name = "Hood Tester", group = "Testers")
public class HoodTester extends LinearOpMode {

    public static double RATE     = 0.4;  // position units per second while holding dpad
    public static double FINE     = 0.01; // bumper single-step size
    public static double START    = 0.30; // start position (within usable range)
    public static double MIN_POS  = 0.0;
    public static double MAX_POS  = 1.0;

    @Override
    public void runOpMode() {
        Servo hood = hardwareMap.get(Servo.class, "shooterHood");
        hood.setDirection(Servo.Direction.FORWARD);
        ((ServoImplEx) hood).setPwmRange(new PwmControl.PwmRange(500, 2500));

        double position = START;
        hood.setPosition(position);

        telemetry.addLine("Hood Tester — HOLD Dpad Up/Down to move, bumpers = fine step");
        telemetry.update();
        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        boolean prevRB = false, prevLB = false;

        while (opModeIsActive()) {
            double dt = timer.seconds();
            timer.reset();

            if (gamepad1.dpad_up)   position += RATE * dt;
            if (gamepad1.dpad_down) position -= RATE * dt;

            if (gamepad1.right_bumper && !prevRB) position += FINE;
            if (gamepad1.left_bumper  && !prevLB) position -= FINE;
            prevRB = gamepad1.right_bumper;
            prevLB = gamepad1.left_bumper;

            position = Math.max(MIN_POS, Math.min(MAX_POS, position));
            hood.setPosition(position);

            telemetry.addLine("=== HOOD TESTER ===");
            telemetry.addData("Position", "%.3f", position);
            telemetry.addLine("Hold Dpad Up = up, Dpad Down = down");
            telemetry.addLine("RB / LB = fine ±0.01");
            telemetry.update();
        }
    }
}
