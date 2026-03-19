package org.firstinspires.ftc.teamcode.Util;

/**
 * PoseStorage bridges Autonomous → TeleOp.
 *
 * At the end of any Autonomous OpMode, write:
 *
 *     PoseStorage.x           = localization.getX();
 *     PoseStorage.y           = localization.getY();
 *     PoseStorage.heading     = localization.getHeading();
 *     PoseStorage.poseWasSaved = true;
 *
 * TeleOp reads poseWasSaved on init. If true, it restores the robot's
 * position from here instead of using the fixed alliance starting position.
 * No changes to TeleOp are needed when Auto is added later.
 */
public class PoseStorage {

    /** X position in inches (field coordinates, origin = bottom-left corner). */
    public static double x = 0.0;

    /** Y position in inches (field coordinates, origin = bottom-left corner). */
    public static double y = 0.0;

    /** Robot heading in radians. 0 = facing +X (right). π/2 = facing +Y (Blue wall). */
    public static double heading = 0.0;

    /**
     * Set to true by Autonomous when it saves a pose.
     * TeleOp checks this flag — if false, it falls back to the
     * fixed alliance starting position instead of reading x/y/heading above.
     */
    public static boolean poseWasSaved = false;
}
