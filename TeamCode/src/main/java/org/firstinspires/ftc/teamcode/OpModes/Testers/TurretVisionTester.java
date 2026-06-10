package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

/**
 * Turret vision-aim tester — NO flywheel, NO odometry.
 *
 * STEP 1 — FIND THE SIGN with the open-loop probe (cannot run away):
 *   Hold RIGHT BUMPER. The turret turns at a fixed slow power (+). Watch "tx":
 *     - tx goes DOWN while holding RB  → set VISION_SIGN = 1
 *     - tx goes UP   while holding RB  → set VISION_SIGN = -1
 *   (LEFT BUMPER turns the other way if you need it.)
 *
 * STEP 2 — Let go of the bumpers. With the right VISION_SIGN, the turret should now
 *   creep onto the tag and HOLD (tx -> ~0). If it creeps the wrong way, flip VISION_SIGN.
 *   If it's too slow, raise VISION_MAX_LEAD; too aggressive, lower it.
 *
 * All knobs live on FTC Dashboard (192.168.43.1:8080). Controls (gamepad1):
 *   RB hold = probe +power   LB hold = probe -power
 *   Dpad R = RED(24)   Dpad L = BLUE(20)   A = stop   B = reset encoder
 */
@Config
@TeleOp(name = "Turret Vision Aim Test", group = "Testers")
public class TurretVisionTester extends LinearOpMode {

    public static double VISION_SIGN     = -1.0; // +1 or -1 — flip if turret drives AWAY from tag
    public static double VISION_MAX_LEAD = 6.0;  // deg the target may lead current (chase speed)
    public static double PROBE_POWER     = 0.15; // open-loop probe power

    private boolean isRed = false;   // start on BLUE (tag 20); Dpad Right switches to RED

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Vision vision = new Vision(hardwareMap, isRed);
        Turret turret = new Turret(hardwareMap, vision);

        telemetry.addLine("Turret Vision Aim Test (no flywheel)");
        telemetry.addLine("STEP 1: hold RB, watch tx. tx DOWN -> SIGN=1, tx UP -> SIGN=-1");
        telemetry.addLine("STEP 2: release bumpers -> it should lock on the tag.");
        telemetry.addLine("Tune at 192.168.43.1:8080. Press START.");
        telemetry.update();
        waitForStart();

        turret.resetEncoder();

        while (opModeIsActive()) {
            if (gamepad1.dpad_right) { isRed = true;  vision.setAlliance(true);  }
            if (gamepad1.dpad_left)  { isRed = false; vision.setAlliance(false); }
            if (gamepad1.b)          { turret.resetEncoder(); }

            turret.setVisionTuning(VISION_SIGN, VISION_MAX_LEAD, 0.0); // no parallax in pure aim test

            vision.update();

            String mode;
            if (gamepad1.right_bumper) {
                turret.manualRotateRaw(PROBE_POWER);   // open-loop probe (+)
                mode = "PROBE +" + PROBE_POWER;
            } else if (gamepad1.left_bumper) {
                turret.manualRotateRaw(-PROBE_POWER);  // open-loop probe (-)
                mode = "PROBE -" + PROBE_POWER;
            } else if (gamepad1.a) {
                turret.stop();
                mode = "STOP";
            } else {
                turret.autoAim();                      // closed-loop vision aim
                mode = "AIM";
            }

            telemetry.addData("Mode",      mode);
            telemetry.addData("Alliance",  isRed ? "RED (24)" : "BLUE (20)");
            telemetry.addData("Tag lock",  turret.hasVisionTarget() ? "LOCK" : "searching...");
            telemetry.addData("tx",        "%.2f deg", vision.getTargetYaw());
            telemetry.addData("ty",        "%.2f deg", vision.getTargetPitch());
            telemetry.addData("angle",     "%.1f deg", turret.getCurrentAngle());
            telemetry.addData("target",    "%.1f deg", turret.getTargetAngle());
            telemetry.addData("power",     "%.3f",     turret.getMotorPower());
            telemetry.addData("SIGN",      "%.0f", VISION_SIGN);
            telemetry.update();
        }

        turret.stop();
        vision.stop();
    }
}
