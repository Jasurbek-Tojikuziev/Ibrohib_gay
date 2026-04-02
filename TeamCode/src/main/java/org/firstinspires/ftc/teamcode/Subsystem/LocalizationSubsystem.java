package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D; // used by setPosition()
import org.firstinspires.ftc.teamcode.Util.PoseStorage;

/**
 * LocalizationSubsystem wraps the GoBilda Pinpoint odometry computer.
 *
 * MATCH SETUP REQUIREMENTS:
 *  1. Robot must be physically oriented the same direction every time it
 *     powers on (heading 90 degrees = facing Blue wall / +Y direction).
 *  2. Turret must be manually pointed forward before OpMode starts.
 *  3. These are hardware requirements — code cannot fix them.
 *
 * Pod wiring convention (GoBilda Pinpoint):
 *  - X pod  = forward/backward odometry pod (parallel to robot forward axis)
 *             X offset: left of center = positive, right of center = negative
 *  - Y pod  = strafe odometry pod (perpendicular to robot forward axis)
 *             Y offset: forward of center = positive, behind center = negative
 *
 * If an axis reads backwards during testing, change the corresponding
 * EncoderDirection below — do NOT change the offset signs or the math elsewhere.
 */
@Config
public class LocalizationSubsystem {

    // Vibration filter thresholds — tunable via FTC Dashboard
    public static double POSITION_THRESHOLD = 0.005; // inches
    public static double HEADING_THRESHOLD  = 0.001; // radians

    // Hardware map device name for the Pinpoint odometry computer
    private static final String DEVICE_NAME = "pinpoint";

    // Red Alliance fixed starting position (inches, heading in radians)
    private static final double RED_START_X       = 129.121;
    private static final double RED_START_Y       =  79.103;
    private static final double RED_START_HEADING = Math.toRadians(0.0);

    // Blue Alliance fixed starting position (inches, heading in radians)
    private static final double BLUE_START_X       =  15.402;
    private static final double BLUE_START_Y       =  79.103;
    private static final double BLUE_START_HEADING = Math.toRadians(180.0);

    // Pod physical offsets from robot center (inches).
    // X pod is 2.756 in to the right of center → negative per Pinpoint convention.
    // Y pod is 3.602 in behind center          → negative per Pinpoint convention.
    // If an axis reads reversed during testing, flip the sign of that offset here.
    private static final double POD_OFFSET_X = -2.756;
    private static final double POD_OFFSET_Y = -3.602;

    private final GoBildaPinpointDriver pinpoint;

    // Starting pose — saved so resetToStart() can restore it
    private double startX;
    private double startY;
    private double startHeading;

    // Last confirmed positions — updated only when change exceeds threshold
    private double lastX;
    private double lastY;
    private double lastHeading;

    /**
     * Initializes the Pinpoint odometry computer and sets the robot's starting pose.
     *
     * If Autonomous ran before this TeleOp (PoseStorage.poseWasSaved == true),
     * the robot's final Auto pose is restored from PoseStorage.
     * Otherwise, the fixed alliance starting position is used.
     *
     * @param hardwareMap  the OpMode's HardwareMap
     * @param isRedAlliance true = Red Alliance start, false = Blue Alliance start
     */
    public LocalizationSubsystem(HardwareMap hardwareMap, boolean isRedAlliance) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, DEVICE_NAME);

        // Pod offsets (inches). Right/rear offsets are negative per Pinpoint convention.
        pinpoint.setOffsets(POD_OFFSET_X, POD_OFFSET_Y, DistanceUnit.INCH);

        // goBILDA 4-bar linkage odometry pods
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set encoder directions. Flip FORWARD → REVERSED here if an axis reads backwards.
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // Determine starting pose: restore from Auto, or use fixed alliance start.
        // Note: resetPosAndIMU() is intentionally NOT called here — it executes
        // asynchronously and wipes the position set by setPosition() before the
        // first loop() runs. IMU is already calibrated from robot power-on.
        if (PoseStorage.poseWasSaved) {
            startX       = PoseStorage.x;
            startY       = PoseStorage.y;
            startHeading = PoseStorage.heading;
        } else if (isRedAlliance) {
            startX       = RED_START_X;
            startY       = RED_START_Y;
            startHeading = RED_START_HEADING;
        } else {
            startX       = BLUE_START_X;
            startY       = BLUE_START_Y;
            startHeading = BLUE_START_HEADING;
        }

        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH, startX, startY,
                AngleUnit.RADIANS, startHeading));

        // Force an immediate update so the starting position is committed before
        // any async side effects of resetPosAndIMU() can overwrite it.
        // Without this, getX()/getY() return 0 on the first loop() call.
        pinpoint.update();

        lastX       = startX;
        lastY       = startY;
        lastHeading = startHeading;
    }

    /**
     * Must be called every loop(). Updates the Pinpoint's internal position
     * estimate using the latest encoder and IMU readings.
     */
    public void update() {
        pinpoint.update();
    }

    /**
     * Resets the robot's position to the starting pose chosen at init().
     * Call when gamepad2 left bumper is pressed to tell the robot it is
     * back at its starting position.
     */
    public void resetToStart() {
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH, startX, startY,
                AngleUnit.RADIANS, startHeading));
        pinpoint.update();
        lastX       = startX;
        lastY       = startY;
        lastHeading = startHeading;
    }

    /**
     * Returns the robot's current X position on the field in inches.
     * Origin (0,0) = bottom-left corner (Red alliance human player zone corner).
     * X increases to the right.
     * Changes smaller than POSITION_THRESHOLD are suppressed to filter shooter vibration.
     */
    public double getX() {
        double current = pinpoint.getPosX(DistanceUnit.INCH);
        if (Math.abs(current - lastX) > POSITION_THRESHOLD) {
            lastX = current;
        }
        return lastX;
    }

    /**
     * Returns the robot's current Y position on the field in inches.
     * Y increases toward the Blue alliance wall.
     * Changes smaller than POSITION_THRESHOLD are suppressed to filter shooter vibration.
     */
    public double getY() {
        double current = pinpoint.getPosY(DistanceUnit.INCH);
        if (Math.abs(current - lastY) > POSITION_THRESHOLD) {
            lastY = current;
        }
        return lastY;
    }

    /**
     * Returns the robot's current heading in radians.
     * setPosition() in the constructor sets the correct starting heading (π/2),
     * so no offset adjustment is needed here.
     * 0 rad = facing +X (right). π/2 rad = facing +Y (Blue wall).
     * Changes smaller than HEADING_THRESHOLD are suppressed to filter shooter vibration.
     */
    public double getHeading() {
        double current = pinpoint.getHeading(AngleUnit.RADIANS);
        if (Math.abs(current - lastHeading) > HEADING_THRESHOLD) {
            lastHeading = current;
        }
        return lastHeading;
    }
}






