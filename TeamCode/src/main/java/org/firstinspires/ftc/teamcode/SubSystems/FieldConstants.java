package org.firstinspires.ftc.teamcode.SubSystems;

import com.pedropathing.geometry.Pose;

/**
 * Single source of truth for field coordinates.
 * All files reference these instead of hardcoding numbers.
 */
public class FieldConstants {
    // Goal (basket) — turret aims here
    public static final Pose RED_GOAL  = new Pose(144, 144);
    public static final Pose BLUE_GOAL = new Pose(0, 144);

    // AprilTag — shooter velocity/hood calibrated from tag distance
    public static final Pose RED_TAG  = new Pose(128, 131);
    public static final Pose BLUE_TAG = new Pose(16, 132);

    // Helpers
    public static Pose getGoal(boolean isRed) { return isRed ? RED_GOAL : BLUE_GOAL; }
    public static Pose getTag(boolean isRed)  { return isRed ? RED_TAG  : BLUE_TAG;  }
}
