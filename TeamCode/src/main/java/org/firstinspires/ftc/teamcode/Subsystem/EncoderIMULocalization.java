package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Dead-reckoning localization using the four drive motor encoders and the
 * Control Hub IMU (BHI260AP).
 *
 * Motor hardware map names must match DriveSubsystem:
 *   "leftFront"  (direction: FORWARD)
 *   "leftRear"   (direction: REVERSE)
 *   "rightFront" (direction: REVERSE)
 *   "rightRear"  (direction: FORWARD)
 *
 * Because of those direction settings, encoder signs when the robot moves
 * forward are: lf=+, lr=−, rf=−, rr=+
 * Forward ticks = (lf − lr − rf + rr) / 4
 *
 * Motor: REV HD Hex Motor + UltraPlanetary 4:1 × 3:1 = 12:1 total
 *   Encoder: 28 PPR × 4 (quadrature) × 12 (gearbox) = 1344 ticks/rev
 * Wheel: 96 mm diameter → circumference = 11.874 in
 * → TICKS_PER_INCH ≈ 113.18  (calibrate by driving a known distance and adjusting)
 *
 * IMU orientation: Control Hub logo facing UP, USB port facing FORWARD.
 * Change LogoFacingDirection / UsbFacingDirection if your hub is mounted differently.
 */
public class EncoderIMULocalization {

    // ── Calibration constant — adjust after physical test ─────────────────────
    public static double TICKS_PER_INCH = 113.18;

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final DcMotor leftFront;
    private final DcMotor leftRear;
    private final DcMotor rightFront;
    private final DcMotor rightRear;
    private final IMU     imu;

    // ── Tracked pose ──────────────────────────────────────────────────────────
    private double x;       // inches
    private double y;       // inches
    private double heading; // radians — 0 = direction robot faces at start

    // ── Last encoder snapshot ─────────────────────────────────────────────────
    private int lastLF;
    private int lastLR;
    private int lastRF;
    private int lastRR;

    public EncoderIMULocalization(HardwareMap hardwareMap) {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        // Change orientation constants if your Control Hub is mounted differently
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        resetPose(0, 0, 0);
    }

    /**
     * Set a known starting pose and zero the IMU yaw.
     * Call once at the beginning of autonomous.
     *
     * @param startX       starting X in inches
     * @param startY       starting Y in inches
     * @param startHeading starting heading in radians (0 = direction the robot faces)
     */
    public void resetPose(double startX, double startY, double startHeading) {
        x       = startX;
        y       = startY;
        heading = startHeading;

        imu.resetYaw();

        lastLF = leftFront.getCurrentPosition();
        lastLR = leftRear.getCurrentPosition();
        lastRF = rightFront.getCurrentPosition();
        lastRR = rightRear.getCurrentPosition();
    }

    /**
     * Update position estimate. Call every loop iteration during autonomous.
     */
    public void update() {
        int curLF = leftFront.getCurrentPosition();
        int curLR = leftRear.getCurrentPosition();
        int curRF = rightFront.getCurrentPosition();
        int curRR = rightRear.getCurrentPosition();

        int dLF = curLF - lastLF;
        int dLR = curLR - lastLR;
        int dRF = curRF - lastRF;
        int dRR = curRR - lastRR;

        lastLF = curLF;
        lastLR = curLR;
        lastRF = curRF;
        lastRR = curRR;

        // lf=+, lr=−, rf=−, rr=+ when moving forward (matches DriveSubsystem directions)
        double forwardInches = (dLF - dLR - dRF + dRR) / 4.0 / TICKS_PER_INCH;

        // Heading from IMU — more reliable than encoder-derived heading on mecanum
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        heading = angles.getYaw(AngleUnit.RADIANS);

        // Project forward movement onto field X/Y axes using current heading
        x += forwardInches * Math.cos(heading);
        y += forwardInches * Math.sin(heading);
    }

    public double getX()              { return x; }
    public double getY()              { return y; }
    public double getHeading()        { return heading; }
    public double getHeadingDegrees() { return Math.toDegrees(heading); }
}
