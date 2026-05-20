package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {

    // ── Translational PIDF ────────────────────────────────────────────────────
    public static double transP = 0.02,  transI = 0, transD = 0.002, transF = 0.06;
    public static double trans2P = 0, trans2I = 0, trans2D = 0, trans2F = 0;

    // ── Heading PIDF ──────────────────────────────────────────────────────────
    public static double headP = 0.45, headI = 0, headD = 0.005, headF = 0.06;
    public static double head2P = 0,  head2I = 0, head2D = 0, head2F = 0;

    // ── Drive PIDF ────────────────────────────────────────────────────────────
    public static double driveP = 0.35,   driveI = 0, driveD = 0.045,   driveF = 0.6, driveFilter = 0.04;
    public static double drive2P = 0, drive2I = 0, drive2D = 0, drive2F = 0, drive2Filter = 0;

    // ── Centripetal & Robot ───────────────────────────────────────────────────
    public static double centripetalScaling = 0.00058;
    public static double mass = 12;

    // ── Zero Power Accelerations ──────────────────────────────────────────────
    public static double forwardZeroPowerAccel = -29.44;
    public static double lateralZeroPowerAccel = -60.49;

    // ── Drive Velocity ────────────────────────────────────────────────────────
    public static double xVelocity = 78.28;
    public static double yVelocity = 56.19;

    // ── Localizer ─────────────────────────────────────────────────────────────
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.95276)
            .strafePodX(-5.51181)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.4, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        FollowerConstants followerConstants = new FollowerConstants()
                .forwardZeroPowerAcceleration(forwardZeroPowerAccel)
                .lateralZeroPowerAcceleration(lateralZeroPowerAccel)
                .translationalPIDFCoefficients(new PIDFCoefficients(transP, transI, transD, transF))
                .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(trans2P, trans2I, trans2D, trans2F))
                .headingPIDFCoefficients(new PIDFCoefficients(headP, headI, headD, headF))
                .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(head2P, head2I, head2D, head2F))
                .drivePIDFCoefficients(new FilteredPIDFCoefficients(driveP, driveI, driveD, driveF, driveFilter))
                .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(drive2P, drive2I, drive2D, drive2F, drive2Filter))
                .centripetalScaling(centripetalScaling)
                .mass(mass);

        MecanumConstants driveConstants = new MecanumConstants()
                .maxPower(1)
                .rightFrontMotorName("rightFront")
                .rightRearMotorName("rightRear")
                .leftRearMotorName("leftRear")
                .leftFrontMotorName("leftFront")
                .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                .xVelocity(xVelocity)
                .yVelocity(yVelocity);

        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
