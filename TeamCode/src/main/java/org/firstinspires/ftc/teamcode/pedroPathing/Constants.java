package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
    public static double transP = 0.095,  transI = 0, transD = 0.0095,  transF = 0.0076;
    public static double trans2P = 0.19,  trans2I = 0, trans2D = 0.0209, trans2F = 0;
    public static double translationalPIDFSwitch = 6;

    // ── Heading PIDF ──────────────────────────────────────────────────────────
    public static double headP = 1.615,  headI = 0, headD = 0.1425,  headF = 0.019;
    public static double head2P = 2.1,   head2I = 0, head2D = 0.095,  head2F = 0;

    // ── Drive PIDF ────────────────────────────────────────────────────────────
    public static double driveP = 0.03325,  driveI = 0, driveD = 0.000002,   driveF = 0.6,  driveFilter = 0.19;
    public static double drive2P = 0.04275, drive2I = 0, drive2D = 0.00000665, drive2F = 0.6, drive2Filter = 0.0;

    // ── Predictive Braking ────────────────────────────────────────────────────
    public static double predictiveKP         = 0.15;
    public static double predictiveKLinear    = 0.07519223906924728;
    public static double predictiveKQuadratic = 0.0019185831783996089;

    // ── Centripetal & Robot ───────────────────────────────────────────────────
    public static double centripetalScaling = 0;  // disabled — predictive braking accounts for this
    public static double mass = 11.0;

    // ── Zero Power Accelerations ──────────────────────────────────────────────
    public static double forwardZeroPowerAccel = -34.41482226756982;
    public static double lateralZeroPowerAccel = -59.02340458740202;

    // ── Drive Velocity ────────────────────────────────────────────────────────
    public static double xVelocity = 86.636;
    public static double yVelocity = 66.908;

    // ── Localizer ─────────────────────────────────────────────────────────────
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.95276)
            .strafePodX(-5.51181)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static double brakingStrength = 2;
    public static double brakingStart = 0.1;
    public static PathConstraints pathConstraints = new PathConstraints(0.96, 100, brakingStrength, brakingStart);

    public static Follower createFollower(HardwareMap hardwareMap) {
        FollowerConstants followerConstants = new FollowerConstants()
                .forwardZeroPowerAcceleration(forwardZeroPowerAccel)
                .lateralZeroPowerAcceleration(lateralZeroPowerAccel)
                .translationalPIDFCoefficients(new PIDFCoefficients(transP, transI, transD, transF))
                .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(trans2P, trans2I, trans2D, trans2F))
                .translationalPIDFSwitch(translationalPIDFSwitch)
                .headingPIDFCoefficients(new PIDFCoefficients(headP, headI, headD, headF))
                .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(head2P, head2I, head2D, head2F))
                .drivePIDFCoefficients(new FilteredPIDFCoefficients(driveP, driveI, driveD, driveF, driveFilter))
                .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(drive2P, drive2I, drive2D, drive2F, drive2Filter))
                .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(predictiveKP, predictiveKLinear, predictiveKQuadratic))
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
                .yVelocity(yVelocity)
                .useVoltageCompensation(true)
                .useBrakeModeInTeleOp(true);

        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
