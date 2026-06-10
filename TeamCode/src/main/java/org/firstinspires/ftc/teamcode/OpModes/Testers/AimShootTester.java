package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

/**
 * Camera-only Aim + Shoot test — NO odometry, NO Pinpoint, NO follower.
 *
 * The turret aims purely from the camera: it centers the tag (ty), so the flywheel points at
 * the tag. Distance comes from the camera (3D pose). Drive is robot-centric (no heading needed).
 *
 * Controls (gamepad1):
 *   Left stick / Right stick X = robot-centric drive   (right trigger = slow mode)
 *   Left bumper (hold)         = run intake (load balls)
 *   Right bumper (press)       = FIRE
 *
 * Tunables (FTC Dashboard, 192.168.43.1:8080): VISION_SIGN, VISION_MAX_LEAD.
 *
 * NOTE: this aims at the TAG (camera centered), not the separate goal coordinate. If your tag
 * is not exactly at the goal, shots land offset by the tag→goal distance.
 */
@Config
@TeleOp(name = "Aim + Shoot Test (camera only)", group = "Testers")
public class AimShootTester extends LinearOpMode {

    public static boolean IS_RED            = true;  // tag 24 (red) / 20 (blue)
    public static double  VISION_SIGN       = 1.0;   // tracking gain/sign (1 = converges on tag)
    public static double  VISION_MAX_LEAD   = 6.0;   // deg the target may lead current (chase speed)
    public static double  PARALLAX_SIGN     = 2.0;   // aim-shift strength: covers 7cm parallax + tag→goal offset
    public static double  FALLBACK_VELOCITY = 1250.0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DriveTrain drive   = new DriveTrain(hardwareMap, telemetry);
        Vision     vision  = new Vision(hardwareMap, IS_RED);
        Turret     turret  = new Turret(hardwareMap, vision);   // vision-only: no odometry
        Shooter    shooter = new Shooter(hardwareMap);
        Intake     intake  = new Intake(hardwareMap);

        telemetry.addLine("Camera-only Aim + Shoot  —  " + (IS_RED ? "RED 24" : "BLUE 20"));
        telemetry.addLine("LB = intake, RB = FIRE.  Drive = sticks (robot-centric).");
        telemetry.addLine("Press START.");
        telemetry.update();
        waitForStart();

        boolean prevRB = false;
        boolean haveValidDistance = false;  // have we ever gotten a real distance?
        double  lastDist = 0.0;             // last good distance (held through tag dropouts)

        while (opModeIsActive()) {
            drive.drive(gamepad1, gamepad2, telemetry);
            vision.update();

            // Distance from the camera. On tag loss, HOLD the last good velocity/hood (just
            // don't update) so a flicker doesn't drop the shot to default. Only fall back to
            // default if we've never seen a tag yet.
            boolean tagSeen = vision.hasTargetTag();
            if (tagSeen) {
                double dist = vision.getTargetDistance();
                if (dist > 0) {
                    shooter.updateVelocity(dist);
                    shooter.updateHood(dist);
                    lastDist = dist;
                    haveValidDistance = true;
                }
            } else if (!haveValidDistance && !shooter.isShooting()) {
                shooter.setTargetVelocity(FALLBACK_VELOCITY);
                shooter.setHoodPosition(0.0);
            }
            // tag lost but we had a distance → leave velocity/hood as-is (held).
            shooter.updatePID();

            // Turret aim: camera-only continuous tracking (centers the tag via ty).
            turret.setVisionTuning(VISION_SIGN, VISION_MAX_LEAD, PARALLAX_SIGN);
            turret.autoAim();

            // Intake: LB loads; otherwise off unless the shot FSM is feeding.
            if (gamepad1.left_bumper)       intake.on();
            else if (!shooter.isShooting()) intake.off();

            // Fire on RB rising edge.
            boolean rb = gamepad1.right_bumper;
            if (rb && !prevRB) shooter.startShoot();
            prevRB = rb;
            shooter.updateFSM(intake);

            telemetry.addData("Tag lock", turret.hasVisionTarget() ? "LOCK" : "searching...");
            telemetry.addData("ty (aim error)", "%.2f deg", vision.getTargetPitch());
            telemetry.addData("Turret cur/tgt", "%.1f / %.1f deg",
                    turret.getCurrentAngle(), turret.getTargetAngle());
            telemetry.addLine();
            telemetry.addData("Dist", "%.1f in %s", lastDist, tagSeen ? "" : "(held)");
            telemetry.addData("Vel", "%.0f / %.0f  %s",
                    shooter.getCurrentVelocity(), shooter.getTargetVelocity(),
                    shooter.atSpeed() ? "READY" : "spinning");
            telemetry.addData("Hood", "%.3f", shooter.getHoodServoPosition());
            telemetry.addData("State", shooter.getCurrentState());
            telemetry.update();
        }

        shooter.off();
        intake.off();
        turret.stop();
        vision.stop();
    }
}
