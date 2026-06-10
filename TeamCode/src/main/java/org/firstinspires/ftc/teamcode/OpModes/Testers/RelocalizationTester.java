package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.FieldConstants;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Relocalization tester — odometry aims the turret; the camera only corrects odometry
 * drift/skips when the robot is settled and a tag is stably visible.
 *
 * HOW TO TEST:
 *   1. Place the robot at START_X/START_Y/START_HEADING_DEG (set them to match reality),
 *      turret pointed straight forward. Press START (encoder zeros to "forward").
 *   2. The turret should aim at the goal from odometry (smooth, no camera lag).
 *   3. Drive around and stop abruptly to trigger an odometry skip. When it settles with the
 *      tag visible, watch "offset" jump and the aim snap back onto the goal ("reloc!" flashes).
 *
 * Tunables (FTC Dashboard, 192.168.43.1:8080):
 *   VISION_SIGN, RELOC_THRESHOLD (deg), SETTLE_ANG_VEL (deg/s), SETTLE_TRANS_VEL (in/s).
 *   Drive: left stick = translate, right stick X = turn (field-centric).
 */
@Config
@TeleOp(name = "Relocalization Tester", group = "Testers")
public class RelocalizationTester extends LinearOpMode {

    public static boolean IS_RED           = true;  // set BEFORE pressing INIT
    public static double  VISION_SIGN      = -1.0;  // +1/-1 (same sign you found for aiming)
    public static double  RELOC_THRESHOLD  = 3.0;   // deg — correct only when residual exceeds this
    public static double  SETTLE_ANG_VEL   = 8.0;   // deg/s — below = "not rotating"
    public static double  SETTLE_TRANS_VEL = 4.0;   // in/s  — below = "not translating"

    public static double  START_X           = 0.0;
    public static double  START_Y           = 0.0;
    public static double  START_HEADING_DEG = 0.0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));
        follower.update();

        Vision vision = new Vision(hardwareMap, IS_RED);
        Turret turret = new Turret(hardwareMap, follower, vision);

        Pose goal = FieldConstants.getGoal(IS_RED);
        Pose tag  = FieldConstants.getTag(IS_RED);
        turret.setGoalPose(goal);
        turret.setTagPose(tag.getX(), tag.getY());

        telemetry.addLine("Relocalization Tester");
        telemetry.addLine(IS_RED ? "RED (tag 24)" : "BLUE (tag 20)");
        telemetry.addLine("Point turret FORWARD, place robot at START pose, press START.");
        telemetry.update();
        waitForStart();

        turret.resetEncoder();          // 0 = turret pointing robot-forward
        follower.startTeleopDrive();

        while (opModeIsActive()) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true);
            follower.update();
            vision.update();

            turret.setRelocalization(VISION_SIGN, RELOC_THRESHOLD, SETTLE_ANG_VEL, SETTLE_TRANS_VEL);
            turret.autoAim();

            Pose p = follower.getPose();
            telemetry.addData("Alliance", IS_RED ? "RED 24" : "BLUE 20");
            telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f deg",
                    p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
            telemetry.addData("Tag lock", turret.hasVisionTarget() ? "LOCK" : "searching...");
            telemetry.addData("Settled", turret.isSettledForReloc() ? "YES" : "moving");
            telemetry.addData("Ang vel", "%.1f deg/s", turret.getAngularVel());
            telemetry.addData("Trans vel", "%.1f in/s", turret.getTranslationVel());
            telemetry.addLine();
            telemetry.addData("Odo aim (raw)", "%.1f deg", turret.getCalculatedTargetAngle());
            telemetry.addData("Reloc offset", "%.1f deg", turret.getRelocOffset());
            telemetry.addData("Residual", "%.1f deg", turret.getRelocResidual());
            telemetry.addData("RELOC!", turret.didReloc() ? "*** correcting ***" : "-");
            telemetry.addLine();
            telemetry.addData("Turret target", "%.1f deg", turret.getTargetAngle());
            telemetry.addData("Turret current", "%.1f deg", turret.getCurrentAngle());
            telemetry.addData("Distance", "%.1f in", vision.getTargetDistance());
            telemetry.update();
        }

        turret.stop();
        vision.stop();
    }
}
