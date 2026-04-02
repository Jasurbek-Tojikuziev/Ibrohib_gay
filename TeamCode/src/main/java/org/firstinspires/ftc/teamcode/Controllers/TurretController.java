package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.TurretSubsystem;

/**
 * TurretController runs the auto-aim math every loop.
 *
 * It reads the robot's field position from LocalizationSubsystem, calculates
 * the angle from the robot to the goal basket, converts that to a turret angle
 * relative to the robot's current heading, then commands TurretSubsystem to
 * rotate to that angle.
 *
 * Approach: compute the field-frame vector from robot to goal, rotate it into
 * the robot's coordinate frame using the heading rotation matrix, then call
 * atan2 on the rotated vector to get the turret angle. Negated so the turret
 * holds its aim as the robot rotates rather than spinning with it.
 *
 * Goal coordinates (inches, PedroPathing coordinate system):
 *   Origin = bottom-left corner of field (Red alliance human player zone corner)
 *   X increases to the right, Y increases toward Blue alliance wall
 *
 * Adjust RED_GOAL_X/Y and BLUE_GOAL_X/Y after the first field test if aim is off.
 */
public class TurretController {

    // ── Goal basket coordinates (inches) ─────────────────────────────────────
    private static final double RED_GOAL_X  = 127.0;
    private static final double RED_GOAL_Y  = 132.0;
    private static final double BLUE_GOAL_X =  16.5;
    private static final double BLUE_GOAL_Y = 132.0;

    // ── Shooter regression coefficients ──────────────────────────────────────
    private static final double HOOD_SLOPE     = 0.00698901;
    private static final double HOOD_INTERCEPT = 0.032967;
    private static final double VEL_MIN  =  900.0;
    private static final double VEL_MAX  = 2000.0;
    private static final double HOOD_MIN =    0.0;
    private static final double HOOD_MAX =    1.0;

    private final LocalizationSubsystem localization;
    private final TurretSubsystem       turret;
    private final ShooterSubsystem      shooter;
    private final Telemetry             telemetry;
    private final double                goalX;
    private final double                goalY;

    // Manual joystick control constants
    private static final double JOYSTICK_DEADBAND    = 0.05;
    private static final double DEGREES_PER_SECOND   = 90.0; // max speed at full deflection

    private long    lastManualTime = 0;
    /** Accumulated joystick trim added on top of auto-aim target. Reset via resetZero(). */
    private double  trimOffset     = 0.0;
    /** True = right stick took over; auto-aim resumes only after resetZero(). */
    private boolean manualMode     = false;
    /** Last auto-aim angle computed while inside a launch zone. Held when outside. */
    private double  lockedAngle    = 0.0;

    /**
     * Constructs the controller and selects the correct goal coordinates
     * based on the alliance color.
     *
     * @param localization  the subsystem providing robot field position
     * @param turret        the subsystem controlling turret motor and PID
     * @param isRedAlliance true = aim at Red basket, false = aim at Blue basket
     * @param telemetry     OpMode telemetry for live debug output on Driver Hub
     */
    public TurretController(LocalizationSubsystem localization,
                            TurretSubsystem turret,
                            ShooterSubsystem shooter,
                            boolean isRedAlliance,
                            Telemetry telemetry) {
        this.localization = localization;
        this.turret       = turret;
        this.shooter      = shooter;
        this.telemetry    = telemetry;
        this.goalX        = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;
        this.goalY        = isRedAlliance ? RED_GOAL_Y : BLUE_GOAL_Y;
    }

    /**
     * Runs the full auto-aim calculation and commands the turret. Call every loop().
     *
     * Steps:
     *   1. Compute vector from robot to goal in field coordinates.
     *   2. Rotate that vector into the robot's coordinate system using heading.
     *   3. Calculate turret angle from the rotated vector using atan2.
     *      Negative sign so turret holds aim instead of rotating with the robot.
     *   4. Normalize result to [-180, 180] degrees using modulo math.
     *   5. Send normalized angle to turret PID via setTargetAngle().
     *   6. Run turret PID update().
     */
    public void update() {
        // Step 1 — robot pose and vector to goal in field coordinates
        double robotX = localization.getX();
        double robotY = localization.getY();
        double heading = localization.getHeading(); // radians

        double fieldDeltaX = goalX - robotX;
        double fieldDeltaY = goalY - robotY;

        // Distance-based shooter calculation
        double distance = Math.sqrt(fieldDeltaX * fieldDeltaX + fieldDeltaY * fieldDeltaY);
        double velocity = Math.max(VEL_MIN, Math.min(VEL_MAX,
                  0.00000601176 * Math.pow(distance, 4)
                - 0.00175879   * Math.pow(distance, 3)
                + 0.186341     * Math.pow(distance, 2)
                - 2.69796      * distance
                + 1031.69331));
        double hood     = Math.max(HOOD_MIN, Math.min(HOOD_MAX, HOOD_SLOPE * distance + HOOD_INTERCEPT));
        shooter.setVelocity(velocity);
        shooter.setHoodPosition(hood);

        // Step 2 — rotate vector into robot coordinate system
        double robotDeltaX =  fieldDeltaX * Math.cos(heading)
                            + fieldDeltaY * Math.sin(heading);
        double robotDeltaY = -fieldDeltaX * Math.sin(heading)
                            + fieldDeltaY * Math.cos(heading);

        // Step 3 — calculate turret angle from robot-frame vector
        double turretAngle = Math.toDegrees(
                Math.atan2(robotDeltaY, robotDeltaX)
        );

        // Step 4 — normalize to [-180, 180] using modulo (safe for any input value)
        turretAngle = ((turretAngle + 180) % 360 + 360) % 360 - 180;

        // Step 5 — command the turret (setTargetAngle also clamps to ±180°)
        turret.setTargetAngle(turretAngle);

        // Step 6 — run one PID iteration with real dt
        turret.update();

        // ── Debug telemetry ───────────────────────────────────────────────────
        telemetry.addData("robotX",        robotX);
        telemetry.addData("robotY",        robotY);
        telemetry.addData("rawHeading",    heading);
        telemetry.addData("robotHeading°", Math.toDegrees(heading));
        telemetry.addData("goalX",         goalX);
        telemetry.addData("goalY",         goalY);
        telemetry.addData("distance (in)", "%.1f", distance);
        telemetry.addData("velocity",      "%.0f", velocity);
        telemetry.addData("hood",          "%.4f", hood);
        telemetry.addData("turretTarget°", turretAngle);
        telemetry.addData("turretActual°", turret.getCurrentAngle());
        telemetry.update();
    }

    /**
     * Combined auto-aim + joystick trim update. Call every loop() in alliance opmodes.
     *
     * Computes the auto-aim target angle exactly as {@link #update()}, then adds a
     * persistent trim offset that is adjusted incrementally by the gamepad2 right
     * joystick each loop. This lets the driver nudge the aim without disabling
     * auto-tracking. The trim resets to zero whenever {@link #resetZero()} is called.
     *
     * @param gp2 gamepad2 — right_stick_x drives trim; left_trigger zero handled in opmode
     */
    public void update(Gamepad gp2) {
        long now = System.nanoTime();
        if (lastManualTime == 0) lastManualTime = now;
        double dt = (now - lastManualTime) / 1e9;
        lastManualTime = now;

        double robotX = localization.getX();
        double robotY = localization.getY();
        boolean inZone = isInsideLaunchZone(robotX, robotY);

        double joystickX = gp2.right_stick_x;

        if (Math.abs(joystickX) > JOYSTICK_DEADBAND) {
            // ── Manual mode: right stick rotates turret freely ────────────────
            manualMode = true;
            turret.setTargetAngle(turret.getTargetAngle() + joystickX * DEGREES_PER_SECOND * dt);
            turret.update();
            telemetry.addData("In Launch Zone", inZone);
            telemetry.addData("Turret Mode",   "MANUAL");
            telemetry.addData("turretTarget°", turret.getTargetAngle());
            telemetry.addData("turretActual°", turret.getCurrentAngle());
        } else if (!manualMode) {
            // ── Always compute auto-aim angle ─────────────────────────────────
            double heading = localization.getHeading();

            double fieldDeltaX = goalX - robotX;
            double fieldDeltaY = goalY - robotY;

            double distance = Math.sqrt(fieldDeltaX * fieldDeltaX + fieldDeltaY * fieldDeltaY);
            double velocity = Math.max(VEL_MIN, Math.min(VEL_MAX,
                  0.00000601176 * Math.pow(distance, 4)
                - 0.00175879   * Math.pow(distance, 3)
                + 0.186341     * Math.pow(distance, 2)
                - 2.69796      * distance
                + 1031.69331));
            double hood = Math.max(HOOD_MIN, Math.min(HOOD_MAX, HOOD_SLOPE * distance + HOOD_INTERCEPT));
            shooter.setVelocity(velocity);
            shooter.setHoodPosition(hood);

            double robotDeltaX =  fieldDeltaX * Math.cos(heading)
                                + fieldDeltaY * Math.sin(heading);
            double robotDeltaY = -fieldDeltaX * Math.sin(heading)
                                + fieldDeltaY * Math.cos(heading);

            double autoAimAngle = Math.toDegrees(Math.atan2(robotDeltaY, robotDeltaX));
            autoAimAngle = ((autoAimAngle + 180) % 360 + 360) % 360 - 180;

            if (inZone) {
                // Inside zone: track goal, update lockedAngle
                lockedAngle = autoAimAngle;
                turret.setTargetAngle(autoAimAngle);
                turret.setPID(TurretSubsystem.kP, TurretSubsystem.kI,
                              TurretSubsystem.kD, TurretSubsystem.kF);
            } else {
                // Outside zone: hold last angle from inside zone
                turret.setTargetAngle(lockedAngle);
                turret.setPID(TurretSubsystem.kP_lock, 0,
                              0, TurretSubsystem.kF_lock);
            }
            turret.update();

            telemetry.addData("In Launch Zone", inZone);
            telemetry.addData("Turret Mode",   inZone ? "AUTO-AIM" : "LOCKED");
            telemetry.addData("turretTarget°", inZone ? autoAimAngle : lockedAngle);
            telemetry.addData("lockedAngle°",  lockedAngle);
            telemetry.addData("turretActual°", turret.getCurrentAngle());
            telemetry.addData("robotX",        robotX);
            telemetry.addData("robotY",        robotY);
            telemetry.addData("robotHeading°", Math.toDegrees(heading));
            telemetry.addData("distance (in)", "%.1f", distance);
            telemetry.addData("velocity",      "%.0f", velocity);
            telemetry.addData("hood",          "%.4f", hood);
        } else {
            // ── Manual mode, stick released: hold current position ────────────
            turret.update();
            telemetry.addData("In Launch Zone", inZone);
            telemetry.addData("Turret Mode",   "MANUAL (holding)");
            telemetry.addData("turretTarget°", turret.getTargetAngle());
            telemetry.addData("turretActual°", turret.getCurrentAngle());
        }
    }

    /**
     * Manual turret control via right joystick X axis. Call every loop() instead of
     * update() when running in manual mode (TeleOp).
     *
     * Right stick right = turret rotates right; left = turret rotates left.
     * Target angle is adjusted incrementally each loop based on joystick deflection and dt.
     * Angle is clamped by TurretSubsystem.setTargetAngle().
     *
     * @param gp gamepad to read right_stick_x from
     */
    public void updateManual(Gamepad gp) {
        long now = System.nanoTime();
        if (lastManualTime == 0) {
            lastManualTime = now;
        }
        double dt = (now - lastManualTime) / 1e9; // seconds
        lastManualTime = now;

        double joystickX = gp.right_stick_x;
        if (Math.abs(joystickX) > JOYSTICK_DEADBAND) {
            double delta = joystickX * DEGREES_PER_SECOND * dt;
            turret.setTargetAngle(turret.getTargetAngle() + delta);
        }

        turret.update();

        telemetry.addData("turretTarget° (manual)", turret.getTargetAngle());
        telemetry.addData("turretActual° (manual)", turret.getCurrentAngle());
    }

    /**
     * Re-zeros the turret encoder at the current physical position and clears PID state.
     * Call this after manually rotating the turret to its forward (0°) position.
     */
    // ── Launch zone helpers ───────────────────────────────────────────────────

    /**
     * Returns true if (x, y) is within 10 inches of either launch zone triangle.
     *
     * Big triangle:   A(0,144)  B(72,72)  C(144,144)
     * Small triangle: A(72,24)  B(48,0)   C(96,0)
     *
     * Both triangles are wound counter-clockwise. Proximity is checked by expanding
     * each triangle's edges outward by 10 inches using signed cross-product distances.
     */
    private boolean isInsideLaunchZone(double x, double y) {
        return isInsideExpandedTriangle(x, y,
                    0, 144,  72, 72, 144, 144, 10)
            || isInsideExpandedTriangle(x, y,
                   72,  24,  48,  0,  96,   0, 10);
    }

    /**
     * Returns true if point (px, py) is inside the triangle ABC expanded outward
     * by {@code margin} inches on all sides.
     *
     * Triangle vertices must be in counter-clockwise order.
     * Uses the signed distance from each edge: a point is inside the expanded
     * triangle when its signed distance from every edge is >= -margin.
     * Signed distance = cross(edge, point - edge_start) / |edge|.
     */
    private boolean isInsideExpandedTriangle(double px, double py,
                                              double ax, double ay,
                                              double bx, double by,
                                              double cx, double cy,
                                              double margin) {
        double abx = bx - ax, aby = by - ay;
        double bcx = cx - bx, bcy = cy - by;
        double cax = ax - cx, cay = ay - cy;

        double d1 = abx * (py - ay) - aby * (px - ax); // cross(AB, AP)
        double d2 = bcx * (py - by) - bcy * (px - bx); // cross(BC, BP)
        double d3 = cax * (py - cy) - cay * (px - cx); // cross(CA, CP)

        double len1 = Math.sqrt(abx * abx + aby * aby);
        double len2 = Math.sqrt(bcx * bcx + bcy * bcy);
        double len3 = Math.sqrt(cax * cax + cay * cay);

        return d1 >= -margin * len1
            && d2 >= -margin * len2
            && d3 >= -margin * len3;
    }

    public void resetZero() {
        turret.resetEncoder();
        trimOffset = 0.0;
        manualMode = false; // resume auto-aim

        // Immediately aim at the goal so the PID starts tracking right away.
        double heading     = localization.getHeading();
        double fieldDeltaX = goalX - localization.getX();
        double fieldDeltaY = goalY - localization.getY();
        double robotDeltaX =  fieldDeltaX * Math.cos(heading) + fieldDeltaY * Math.sin(heading);
        double robotDeltaY = -fieldDeltaX * Math.sin(heading) + fieldDeltaY * Math.cos(heading);
        double autoAimAngle = Math.toDegrees(Math.atan2(robotDeltaY, robotDeltaX));
        autoAimAngle = ((autoAimAngle + 180) % 360 + 360) % 360 - 180;
        turret.setTargetAngle(autoAimAngle);
    }
}
