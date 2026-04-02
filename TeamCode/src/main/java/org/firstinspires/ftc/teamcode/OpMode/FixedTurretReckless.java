package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.TurretSubsystem;

/**
 * Fixed-Turret TeleOp — drivetrain-corrected copy of FixedTurretOpMode.
 *
 * Drive fix vs FixedTurretOpMode / MecanumDriveController:
 *   - Motors wired with standard FTC convention: left=REVERSE, right=FORWARD
 *     (was FL=FWD, LR=REV, RF=REV, RR=FWD — incorrect for this robot)
 *   - Gamepad mapping corrected:
 *       axial   = -left_stick_y   (forward / backward)
 *       lateral =  left_stick_x   (strafe left / right)
 *       yaw     =  right_stick_x  (rotate left / right)
 *     (MecanumDriveController had lateral and yaw swapped)
 *
 * Everything else (shooter, intake, turret) is identical to FixedTurretOpMode.
 */
@TeleOp(name = "Fixed Turret Reckless", group = "TeleOp")
public class FixedTurretReckless extends OpMode {

    private static final double FIXED_P           = 70.400;
    private static final double FIXED_F           = 13.00;

    private static final double VELOCITY_IDLE      = 1175.0;
    private static final double VELOCITY_RIGHT     = 1000.0;
    private static final double VELOCITY_LEFT      = 1350.0;
    private static final double VELOCITY_TOLERANCE = 0.01;   // fire only when within 1% of target (accel AND decel)

    private static final double HOOD_RIGHT         = 0.0;   // hood for right bumper (1000 ticks/s)
    private static final double HOOD_LEFT          = 0.4;   // hood for left bumper (1350 ticks/s)
    private static final double HOOD_DEFAULT       = 0.0;   // hood resting position

    private static final double SHOOT_DURATION     = 1.5;

    private enum ShootState { IDLE, SPOOLING, SHOOTING }

    // ── Drive motors (managed directly — not through DriveSubsystem) ──────────
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;

    // ── Subsystems ────────────────────────────────────────────────────────────
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem  intakeSubsystem;
    private TurretSubsystem  turretSubsystem;
    private IntakeController intakeController;

    // ── Shoot sequence state ──────────────────────────────────────────────────
    private ShootState        shootState      = ShootState.IDLE;
    private double            targetVelocity  = VELOCITY_IDLE;
    private double            targetHood      = HOOD_DEFAULT;
    private boolean           lastRightBumper = false;
    private boolean           lastLeftBumper  = false;
    private final ElapsedTime shootTimer      = new ElapsedTime();

    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        // ── Drive motors: standard FTC mecanum directions ─────────────────────
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ── Shooter / intake / turret ─────────────────────────────────────────
        ShooterSubsystem.PIDF_P          = FIXED_P;
        ShooterSubsystem.PIDF_F          = FIXED_F;
        ShooterSubsystem.TARGET_VELOCITY = VELOCITY_IDLE;

        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
        intakeSubsystem  = new IntakeSubsystem(hardwareMap);
        turretSubsystem  = new TurretSubsystem(hardwareMap);
        intakeController = new IntakeController(intakeSubsystem);

        telemetry.addLine("Init complete — Fixed Turret Reckless");
        telemetry.update();
    }

    @Override
    public void start() {
        shooterSubsystem.startMotors();
        turretSubsystem.brake(); // no active PID — BRAKE mode holds position passively
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // ── Drive: corrected mapping ───────────────────────────────────────────
        //   axial   = -left_stick_y  (push up = forward)
        //   lateral =  left_stick_x  (push right = strafe right)
        //   yaw     =  right_stick_x (push right = rotate right)
        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        double fl = axial + lateral + yaw;
        double fr = axial - lateral - yaw;
        double bl = axial - lateral + yaw;
        double br = axial + lateral - yaw;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        leftFront.setPower(fl / max);
        rightFront.setPower(fr / max);
        leftRear.setPower(bl / max);
        rightRear.setPower(br / max);

        // ── Shooter motors ────────────────────────────────────────────────────
        shooterSubsystem.updateMotorSync();
        shooterSubsystem.updatePIDF();

        // ── Shoot sequence state machine ──────────────────────────────────────
        boolean rightBumperPressed = gamepad2.right_bumper && !lastRightBumper;
        boolean leftBumperPressed  = gamepad2.left_bumper  && !lastLeftBumper;
        lastRightBumper = gamepad2.right_bumper;
        lastLeftBumper  = gamepad2.left_bumper;

        switch (shootState) {

            case IDLE:
                if (rightBumperPressed) {
                    targetVelocity = VELOCITY_RIGHT;
                    shooterSubsystem.setHoodPosition(HOOD_RIGHT);  // immediate — no waiting
                    shooterSubsystem.setVelocity(targetVelocity);
                    ShooterSubsystem.TARGET_VELOCITY = targetVelocity;
                    shootState = ShootState.SPOOLING;
                } else if (leftBumperPressed) {
                    targetVelocity = VELOCITY_LEFT;
                    shooterSubsystem.setHoodPosition(HOOD_LEFT);   // immediate — no waiting
                    shooterSubsystem.setVelocity(targetVelocity);
                    ShooterSubsystem.TARGET_VELOCITY = targetVelocity;
                    shootState = ShootState.SPOOLING;
                }
                break;

            case SPOOLING:
                // Gate + intake wait for velocity — hood is already set
                if (Math.abs(shooterSubsystem.getMasterVelocity() - targetVelocity) <= targetVelocity * VELOCITY_TOLERANCE) {
                    shooterSubsystem.setGatePosition(ShooterSubsystem.GATE_OPEN);
                    intakeSubsystem.setPower(1.0);
                    shootTimer.reset();
                    shootState = ShootState.SHOOTING;
                }
                break;

            case SHOOTING:
                if (shootTimer.seconds() >= SHOOT_DURATION) {
                    shooterSubsystem.setGatePosition(ShooterSubsystem.GATE_CLOSED);
                    shooterSubsystem.setHoodPosition(HOOD_DEFAULT);
                    intakeSubsystem.stop();
                    ShooterSubsystem.TARGET_VELOCITY = VELOCITY_IDLE;
                    shooterSubsystem.setVelocity(VELOCITY_IDLE);
                    shootState = ShootState.IDLE;
                }
                break;
        }

        // ── Intake manual control (gamepad2, blocked during sequence) ─────────
        if (shootState == ShootState.IDLE) {
            intakeController.update(gamepad2, gamepad2);
        }

        // ── Turret: passive BRAKE hold — no active PID ───────────────────────
        // (brake() already called in start(); no update() needed)

        // ── Telemetry ─────────────────────────────────────────────────────────
        double currentVel = shooterSubsystem.getMasterVelocity();
        telemetry.addData("Shoot state",         shootState.name());
        telemetry.addData("Shooter velocity",    "%.0f / %.0f (%.0f%%)",
                currentVel, ShooterSubsystem.TARGET_VELOCITY,
                ShooterSubsystem.TARGET_VELOCITY > 0
                        ? (currentVel / ShooterSubsystem.TARGET_VELOCITY * 100) : 0.0);
        telemetry.addData("Gate",                shootState == ShootState.SHOOTING ? "OPEN" : "CLOSED");
        if (shootState == ShootState.SHOOTING) {
            telemetry.addData("Time left",       "%.1fs", SHOOT_DURATION - shootTimer.seconds());
        }
        telemetry.update();
    }
}
