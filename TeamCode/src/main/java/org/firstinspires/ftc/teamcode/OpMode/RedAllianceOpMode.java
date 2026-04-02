package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.Controllers.MecanumDriveController;
import org.firstinspires.ftc.teamcode.Controllers.ShooterController;
import org.firstinspires.ftc.teamcode.Controllers.TurretController;
import org.firstinspires.ftc.teamcode.Subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.TurretSubsystem;

/**
 * Red Alliance TeleOp.
 *
 * Identical to BlueAllianceOpMode except:
 *   - LocalizationSubsystem initialized with isRedAlliance = true
 *   - TurretController aims at the Red basket (143.0, 143.0 in)
 *
 * MATCH SETUP CHECKLIST (before pressing Init):
 *   1. Robot physically oriented facing Blue wall (heading = 90 degrees)
 *   2. Turret manually pointed forward (toward robot front)
 *   3. Robot placed at Red Alliance starting position
 */
@TeleOp(name = "Red Alliance TeleOp", group = "TeleOp")
public class RedAllianceOpMode extends OpMode {

    // ── Subsystems ────────────────────────────────────────────────────────────
    private LocalizationSubsystem localization;
    private TurretSubsystem       turretSubsystem;
    private IntakeSubsystem       intakeSubsystem;

    // ── Controllers ───────────────────────────────────────────────────────────
    private MecanumDriveController driveController;
    private IntakeController       intakeController;
    private ShooterController      shooterController;
    private TurretController       turretController;

    // Rising-edge state for gamepad2 right trigger (full reset)
    private boolean lastRightTrigger2 = false;

    @Override
    public void init() {
        ShooterSubsystem.TARGET_VELOCITY = 1350.0;

        // Localization — reads PoseStorage if Auto ran, otherwise uses Red start position
        localization = new LocalizationSubsystem(hardwareMap, true);

        // Turret — encoder zeroed here; turret must be physically forward at this moment
        turretSubsystem = new TurretSubsystem(hardwareMap);

        // Existing subsystems — identical to MainOpMode
        intakeSubsystem   = new IntakeSubsystem(hardwareMap);
        driveController   = new MecanumDriveController(new DriveSubsystem(hardwareMap));
        intakeController  = new IntakeController(intakeSubsystem);
        ShooterSubsystem shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
        shooterController = new ShooterController(shooterSubsystem, intakeSubsystem, telemetry);

        // Turret controller — aims at Red basket (143.0, 143.0 in)
        turretController = new TurretController(localization, turretSubsystem, shooterSubsystem, true, telemetry);

        // Confirm Pinpoint received the starting position.
        // initX should be ≈ 87.620, initY ≈ 8.298, initHeading° ≈ 0 (raw, before offset)
        telemetry.addData("initX",        localization.getX());
        telemetry.addData("initY",        localization.getY());
        telemetry.addData("initHeading°", Math.toDegrees(localization.getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        // Spin up shooter motors the moment the start button is pressed
        shooterController.onStart();
    }

    @Override
    public void loop() {
        // ── Localization ─────────────────────────────────────────────────────
        localization.update();

        // ── Drive (gamepad 1) ─────────────────────────────────────────────────
        driveController.update(gamepad1);

        // ── Full reset (gamepad2 right trigger, rising edge) ─────────────────
        // Resets odometry to Red alliance start position and re-zeros turret encoder.
        boolean rightTrigger2 = gamepad2.right_trigger > 0.5;
        if (rightTrigger2 && !lastRightTrigger2) {
            localization.resetToStart();
            turretController.resetZero();
        }
        lastRightTrigger2 = rightTrigger2;

        // ── Intake & Shooter (gamepad 1 + 2) ─────────────────────────────────
        // IntakeController runs first; ShooterController runs second so it can
        // override intake power during the shoot sequence.
        intakeController.update(gamepad1, gamepad2);
        shooterController.update(gamepad1, gamepad2);

        // ── Turret: auto-aim + gamepad2 right joystick trim ──────────────────
        turretController.update(gamepad2);

        // ── Driver Station telemetry ──────────────────────────────────────────
        telemetry.addData("Alliance",         "RED");
        telemetry.addData("Robot X (in)",     "%.2f", localization.getX());
        telemetry.addData("Robot Y (in)",     "%.2f", localization.getY());
        telemetry.addData("Heading  (deg)",   "%.1f", Math.toDegrees(localization.getHeading()));
        telemetry.addData("Turret Target (deg)", "%.1f", turretSubsystem.getTargetAngle());
        telemetry.addData("Turret Actual (deg)", "%.1f", turretSubsystem.getCurrentAngle());
        telemetry.update();
    }
}
