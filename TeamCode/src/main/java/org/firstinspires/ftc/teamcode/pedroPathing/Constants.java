package org.firstinspires.ftc.teamcode.pedroPathing;

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

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-29.7)
            .lateralZeroPowerAcceleration(-74.43)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.015,0,0.0001,0.015))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.007,0,0.0001,0.012 ))
            .headingPIDFCoefficients(new PIDFCoefficients(1.95,0,0.05,0.013))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.2,0,0.001 ,0.013))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.7,0,0.01,0.6,0.04))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.3,0,0.001,0.6,0.04))
            .centripetalScaling(0.00058)
            .mass(12);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(92.8)
            .yVelocity(71);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.95276)
            .strafePodX(-5.51181) //it was 0 inch
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // CRITICAL: These encoder directions MUST match Localizer.java!
            // Forward encoder: mounted parallel to robot front/back → controls Y-axis (vertical movement)
            // Strafe encoder: mounted perpendicular to robot → controls X-axis (horizontal movement)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)    // Y-axis (forward/backward)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);   // X-axis (left/right)

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.8, 1);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}