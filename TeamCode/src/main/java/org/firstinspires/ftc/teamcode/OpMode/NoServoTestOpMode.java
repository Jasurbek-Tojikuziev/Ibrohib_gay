package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.MecanumDriveController;
import org.firstinspires.ftc.teamcode.Subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.TurretSubsystem;

/**
 * No-Servo Test TeleOp.
 *
 * Same as Fixed Turret TeleOp except:
 *   - No servos are moved at any point (gate and compression stay at default).
 *   - Right trigger (click) → toggles intake on/off. No timer, no auto-stop.
 *   - Shooter motors spin at FIXED_VELOCITY the whole time.
 */
@TeleOp(name = "No Servo Test TeleOp", group = "TeleOp")
public class NoServoTestOpMode extends OpMode {

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

    // ── Intake toggle state ───────────────────────────────────────────────────
    private boolean intakeOn           = false;
    private boolean lastRightTrigger   = false;

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

        telemetry.addLine("Init complete — No Servo Test TeleOp");
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

        // ── Intake toggle (right trigger click) ───────────────────────────────
        boolean rightTrigger        = gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1;
        boolean rightTriggerPressed = rightTrigger && !lastRightTrigger;
        lastRightTrigger            = rightTrigger;

        if (rightTriggerPressed) {
            intakeOn = !intakeOn;
            intakeSubsystem.setPower(intakeOn ? 1.0 : 0.0);
        }

        // ── Turret: manual control via gamepad2 right stick X ────────────────
        double joystickX = gamepad2.right_stick_x;
        if (Math.abs(joystickX) > JOYSTICK_DEADBAND) {
            turretSubsystem.setTargetAngle(
                    turretSubsystem.getTargetAngle() + joystickX * DEGREES_PER_SECOND * dt);
        }
        turretSubsystem.update();

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetry.addData("Intake",          intakeOn ? "ON" : "OFF");
        telemetry.addData("Shooter velocity","%.0f / %.0f",
                shooterSubsystem.getMasterVelocity(), ShooterSubsystem.TARGET_VELOCITY);
        telemetry.addData("Turret Target (deg)", "%.1f", turretSubsystem.getTargetAngle());
        telemetry.addData("Turret Actual (deg)", "%.1f", turretSubsystem.getCurrentAngle());
        telemetry.update();
    }
}
