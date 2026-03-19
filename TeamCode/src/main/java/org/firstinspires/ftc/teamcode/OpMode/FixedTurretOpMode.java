package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.Controllers.MecanumDriveController;
import org.firstinspires.ftc.teamcode.Subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.TurretSubsystem;

/**
 * Fixed-Turret TeleOp.
 *
 * Identical to the alliance opmodes except:
 *   - No localization / auto-aim. Turret is controlled manually via gamepad2 right stick X.
 *   - Shooter PIDF locked to P=70.4, F=12.24 at velocity 1400 ticks/s.
 *   - Right bumper (click) → gate servo opens. Click again → gate servo closes.
 *   - Compression servo is never touched (stays at default 1.0).
 */
@TeleOp(name = "Fixed Turret TeleOp", group = "TeleOp")
public class FixedTurretOpMode extends OpMode {

    private static final double FIXED_P          = 70.400;
    private static final double FIXED_F          = 12.2400;
    private static final double FIXED_VELOCITY   = 1400.0;

    private static final double JOYSTICK_DEADBAND  = 0.05;
    private static final double DEGREES_PER_SECOND = 90.0;

    // ── Subsystems ────────────────────────────────────────────────────────────
    private ShooterSubsystem       shooterSubsystem;
    private IntakeSubsystem        intakeSubsystem;
    private TurretSubsystem        turretSubsystem;

    // ── Controllers ───────────────────────────────────────────────────────────
    private MecanumDriveController driveController;
    private IntakeController       intakeController;

    // ── Gate toggle state ─────────────────────────────────────────────────────
    private boolean gateOpen          = false;
    private boolean lastRightBumper   = false;

    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        ShooterSubsystem.PIDF_P          = FIXED_P;
        ShooterSubsystem.PIDF_F          = FIXED_F;
        ShooterSubsystem.TARGET_VELOCITY = FIXED_VELOCITY;

        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
        intakeSubsystem  = new IntakeSubsystem(hardwareMap);
        turretSubsystem  = new TurretSubsystem(hardwareMap);
        driveController  = new MecanumDriveController(new DriveSubsystem(hardwareMap));
        intakeController = new IntakeController(intakeSubsystem);

        telemetry.addLine("Init complete — Fixed Turret TeleOp");
        telemetry.update();
    }

    @Override
    public void start() {
        shooterSubsystem.startMotors();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // ── Drive ─────────────────────────────────────────────────────────────
        driveController.update(gamepad1);

        // ── Shooter motors ────────────────────────────────────────────────────
        shooterSubsystem.updateMotorSync();
        shooterSubsystem.updatePIDF();

        // ── Intake ────────────────────────────────────────────────────────────
        intakeController.update(gamepad1, gamepad2);

        // ── Gate servo toggle (right bumper click) ────────────────────────────
        boolean rightBumper        = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean rightBumperPressed = rightBumper && !lastRightBumper;
        lastRightBumper            = rightBumper;

        if (rightBumperPressed) {
            gateOpen = !gateOpen;
            shooterSubsystem.setGatePosition(
                    gateOpen ? ShooterSubsystem.GATE_OPEN : ShooterSubsystem.GATE_CLOSED);
        }

        // ── Turret: manual control via gamepad2 right stick X ────────────────
        double joystickX = gamepad2.right_stick_x;
        if (Math.abs(joystickX) > JOYSTICK_DEADBAND) {
            turretSubsystem.setTargetAngle(
                    turretSubsystem.getTargetAngle() + joystickX * DEGREES_PER_SECOND * dt);
        }
        turretSubsystem.update();

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetry.addData("Gate",                gateOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Shooter velocity",    "%.0f / %.0f",
                shooterSubsystem.getMasterVelocity(), ShooterSubsystem.TARGET_VELOCITY);
        telemetry.addData("Turret Target (deg)", "%.1f", turretSubsystem.getTargetAngle());
        telemetry.addData("Turret Actual (deg)", "%.1f", turretSubsystem.getCurrentAngle());
        telemetry.update();
    }
}
