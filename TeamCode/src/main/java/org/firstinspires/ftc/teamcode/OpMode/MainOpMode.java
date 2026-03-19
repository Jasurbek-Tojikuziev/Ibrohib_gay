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

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainOpMode extends OpMode {

    private MecanumDriveController driveController;
    private IntakeController       intakeController;
    private ShooterController      shooterController;
    private TurretController       turretController;

    private IntakeSubsystem       intakeSubsystem;
    private LocalizationSubsystem localization;

    // Rising-edge state for gamepad2 left bumper (position reset)
    private boolean lastLeftBumper2    = false;
    // Rising-edge state for gamepad2 left trigger (turret zero reset)
    private boolean lastLeftTrigger2   = false;

    @Override
    public void init() {
        intakeSubsystem   = new IntakeSubsystem(hardwareMap);
        localization      = new LocalizationSubsystem(hardwareMap, false);
        driveController   = new MecanumDriveController(new DriveSubsystem(hardwareMap));
        intakeController  = new IntakeController(intakeSubsystem);
        shooterController = new ShooterController(new ShooterSubsystem(hardwareMap, telemetry), intakeSubsystem, telemetry);
        turretController  = new TurretController(localization, new TurretSubsystem(hardwareMap), false, telemetry);
    }

    @Override
    public void start() {
        // Spin up shooter motors the moment the start button is pressed
        shooterController.onStart();
    }

    @Override
    public void loop() {
        localization.update();

        if (localization.getY() > 80 && localization.getX() > 76) {
            shooterController.setVelocity(1350);
        } else {
            shooterController.setVelocity(ShooterSubsystem.TARGET_VELOCITY);
        }

        driveController.update(gamepad1);

        // Gamepad2 left bumper (rising edge) — tell robot it is at its starting position
        boolean leftBumper2 = gamepad2.left_bumper;
        if (leftBumper2 && !lastLeftBumper2) {
            localization.resetToStart();
        }
        lastLeftBumper2 = leftBumper2;

        // IntakeController runs first; ShooterController runs second so it can
        // override intake power during the shoot sequence.
        // Both gamepads can trigger intake and shoot.
        intakeController.update(gamepad1, gamepad2);
        shooterController.update(gamepad1, gamepad2);

        // Gamepad2 left trigger (rising edge) — re-zero turret encoder at current position
        boolean leftTrigger2 = gamepad2.left_trigger > 0.5;
        if (leftTrigger2 && !lastLeftTrigger2) {
            turretController.resetZero();
        }
        lastLeftTrigger2 = leftTrigger2;

        // Turret: gamepad2 right joystick left/right for manual rotation
        turretController.updateManual(gamepad2);

        telemetry.update();
    }
}
