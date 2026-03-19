package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystem.LocalizationSubsystem;
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
    private static final double RED_GOAL_X  = 143.0;
    private static final double RED_GOAL_Y  = 143.0;
    private static final double BLUE_GOAL_X =   0.0;
    private static final double BLUE_GOAL_Y = 143.0;

    private final LocalizationSubsystem localization;
    private final TurretSubsystem       turret;
    private final Telemetry             telemetry;
    private final double                goalX;
    private final double                goalY;

    // Manual joystick control constants
    private static final double JOYSTICK_DEADBAND    = 0.05;
    private static final double DEGREES_PER_SECOND   = 90.0; // max speed at full deflection

    private long   lastManualTime = 0;
    /** Accumulated joystick trim added on top of auto-aim target. Reset via resetZero(). */
    private double trimOffset     = 0.0;

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
                            boolean isRedAlliance,
                            Telemetry telemetry) {
        this.localization = localization;
        this.turret       = turret;
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
        telemetry.addData("fieldDeltaX",   fieldDeltaX);
        telemetry.addData("fieldDeltaY",   fieldDeltaY);
        telemetry.addData("robotDeltaX",   robotDeltaX);
        telemetry.addData("robotDeltaY",   robotDeltaY);
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
        // ── Auto-aim math (identical to update()) ────────────────────────────
        double robotX   = localization.getX();
        double robotY   = localization.getY();
        double heading  = localization.getHeading(); // radians

        double fieldDeltaX = goalX - robotX;
        double fieldDeltaY = goalY - robotY;

        double robotDeltaX =  fieldDeltaX * Math.cos(heading)
                            + fieldDeltaY * Math.sin(heading);
        double robotDeltaY = -fieldDeltaX * Math.sin(heading)
                            + fieldDeltaY * Math.cos(heading);

        double autoAimAngle = Math.toDegrees(Math.atan2(robotDeltaY, robotDeltaX));
        autoAimAngle = ((autoAimAngle + 180) % 360 + 360) % 360 - 180;

        // ── Joystick trim accumulation ────────────────────────────────────────
        long now = System.nanoTime();
        if (lastManualTime == 0) lastManualTime = now;
        double dt = (now - lastManualTime) / 1e9;
        lastManualTime = now;

        double joystickX = gp2.right_stick_x;
        if (Math.abs(joystickX) > JOYSTICK_DEADBAND) {
            trimOffset += joystickX * DEGREES_PER_SECOND * dt;
        }

        // ── Command PID with combined angle ───────────────────────────────────
        double finalTarget = autoAimAngle + trimOffset;
        turret.setTargetAngle(finalTarget);
        turret.update();

        // ── Debug telemetry ───────────────────────────────────────────────────
        telemetry.addData("autoAim°",      autoAimAngle);
        telemetry.addData("trimOffset°",   trimOffset);
        telemetry.addData("turretTarget°", finalTarget);
        telemetry.addData("turretActual°", turret.getCurrentAngle());
        telemetry.addData("robotX",        robotX);
        telemetry.addData("robotY",        robotY);
        telemetry.addData("robotHeading°", Math.toDegrees(heading));
        telemetry.update();
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
    public void resetZero() {
        turret.resetEncoder();
        trimOffset = 0.0;

        // Immediately aim at the goal so the PID never holds 0° after a reset.
        double heading    = localization.getHeading();
        double fieldDeltaX = goalX - localization.getX();
        double fieldDeltaY = goalY - localization.getY();
        double robotDeltaX =  fieldDeltaX * Math.cos(heading) + fieldDeltaY * Math.sin(heading);
        double robotDeltaY = -fieldDeltaX * Math.sin(heading) + fieldDeltaY * Math.cos(heading);
        double autoAimAngle = Math.toDegrees(Math.atan2(robotDeltaY, robotDeltaX));
        autoAimAngle = ((autoAimAngle + 180) % 360 + 360) % 360 - 180;
        turret.setTargetAngle(autoAimAngle);
    }
}
