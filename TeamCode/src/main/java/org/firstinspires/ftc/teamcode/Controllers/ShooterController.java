package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;

/**
 * Shooter controller for gamepad 2.
 *
 * Right bumper (click) → shoot sequence:
 *   1. Gate opens (0.29), intake runs at full power — simultaneously.
 *   2. After 1.5 s  → gate closes (0), intake stops.
 *   Compression servo is NOT used; it stays at its default position (1.0) at all times.
 *
 * Shooter motors spin at TARGET_VELOCITY the moment the op-mode starts.
 * PIDF on the master motor keeps velocity stable through all three balls.
 *
 * Call order in loop() matters: ShooterController must be called AFTER IntakeController
 * so that it can override intake power during the shoot sequence.
 */
public class ShooterController {

    private static final double SHOOT_DURATION = 1.5; // seconds

    private enum State { IDLE, SHOOTING }

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem  intake;
    private final Telemetry        telemetry;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private State             state           = State.IDLE;
    private final ElapsedTime shootTimer      = new ElapsedTime();
    private boolean           lastRightBumper = false;

    public ShooterController(ShooterSubsystem shooter, IntakeSubsystem intake, Telemetry telemetry) {
        this.shooter   = shooter;
        this.intake    = intake;
        this.telemetry = telemetry;
    }

    /** Call once from MainOpMode.start() to spin up shooter motors. */
    public void onStart() {
        shooter.startMotors();
    }

    /** Adjust shooter velocity (ticks/sec). */
    public void setVelocity(double velocity) {
        shooter.setVelocity(velocity);
    }

    /** Call every loop() from MainOpMode. Must be called after IntakeController.update(). */
    public void update(Gamepad gp1, Gamepad gp2) {
        // Keep slave motor synced to master encoder every loop
        shooter.updateMotorSync();

        // Re-apply PIDF if changed via dashboard
        shooter.updatePIDF();

        // Send velocity graph to FTC Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("velocity (ticks/s)", shooter.getMasterVelocity());
        packet.put("target (ticks/s)",   ShooterSubsystem.TARGET_VELOCITY);
        dashboard.sendTelemetryPacket(packet);

        // Rising-edge detection for right bumper on either gamepad
        boolean rightBumper        = gp1.right_bumper || gp2.right_bumper;
        boolean rightBumperPressed = rightBumper && !lastRightBumper;
        lastRightBumper            = rightBumper;

        telemetry.addData("Shooter1 velocity (ticks/s)", "%.0f / %.0f", shooter.getMasterVelocity(), ShooterSubsystem.TARGET_VELOCITY);
        telemetry.addData("Shooter2 power",              "%.3f", shooter.getSlavePower());
        telemetry.addData("Shooter state",               state);

        switch (state) {
            case IDLE:
                if (rightBumperPressed) {
                    // Begin shoot sequence – all actions are simultaneous
                    shooter.setGatePosition(ShooterSubsystem.GATE_OPEN);
                    intake.setPower(1.0);
                    shootTimer.reset();
                    state = State.SHOOTING;
                }
                break;

            case SHOOTING:
                // Hold intake on throughout the sequence, overriding IntakeController
                intake.setPower(1.0);

                if (shootTimer.seconds() >= SHOOT_DURATION) {
                    shooter.setGatePosition(ShooterSubsystem.GATE_CLOSED);
                    intake.stop();
                    state = State.IDLE;
                }
                break;
        }
    }
}
