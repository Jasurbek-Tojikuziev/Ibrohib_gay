package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.TurretSubsystem;

/**
 * NazarX OpMode — simplified single-velocity teleop.
 *
 *   - Shooter runs at VELOCITY (1000 ticks/s) the entire match — no idle speed change.
 *   - Right bumper: open gate + run intake immediately (no velocity wait).
 *     Left bumper does the same.
 *   - Shoot sequence runs for 1.5 s then closes gate + stops intake.
 *   - Everything else (drive, intake triggers, turret) identical to FixedTurretNazarX.
 */
@TeleOp(name = "NazarX OpMode", group = "TeleOp")
public class NazarXOpMode extends OpMode {

    private static final double FIXED_P           = 70.400;
    private static final double FIXED_F           = 13.00;

    private static final double VELOCITY          = 1000.0; // constant shoot speed

    private static final double GATE_OPEN         = 0.0;
    private static final double GATE_CLOSED       = 0.27;
    private static final double HOOD              = 0.1;    // fixed hood position

    private static final double SHOOT_DURATION    = 1.5;

    private static final double YAW_SCALE         = 0.8;
    private static final double JOYSTICK_DEADBAND = 0.05;
    private static final double DEGREES_PER_SECOND = 30.0;

    private enum ShootState { IDLE, SHOOTING }

    // ── Drive motors ──────────────────────────────────────────────────────────
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;

    // ── Subsystems ────────────────────────────────────────────────────────────
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem  intakeSubsystem;
    private TurretSubsystem  turretSubsystem;

    // ── Shoot sequence state ──────────────────────────────────────────────────
    private ShootState        shootState      = ShootState.IDLE;
    private boolean           lastRightBumper = false;
    private boolean           lastLeftBumper  = false;
    private final ElapsedTime shootTimer      = new ElapsedTime();

    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        ShooterSubsystem.PIDF_P          = FIXED_P;
        ShooterSubsystem.PIDF_F          = FIXED_F;
        ShooterSubsystem.TARGET_VELOCITY = VELOCITY;

        // ── Drive motors — NazarX original directions ─────────────────────────
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
        shooterSubsystem.reverseMotors();
        intakeSubsystem  = new IntakeSubsystem(hardwareMap);
        turretSubsystem  = new TurretSubsystem(hardwareMap);

        shooterSubsystem.setHoodPosition(HOOD);
        shooterSubsystem.setGatePosition(GATE_CLOSED);

        telemetry.addLine("Init complete — NazarX OpMode");
        telemetry.update();
    }

    @Override
    public void start() {
        shooterSubsystem.startMotors();
        turretSubsystem.setTargetAngle(0);
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // ── Drive (gamepad1 only) ─────────────────────────────────────────────
        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.right_stick_x * YAW_SCALE;
        double yaw     =  gamepad1.left_stick_x;

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

        // ── Shoot sequence state machine (no velocity wait) ───────────────────
        boolean rightBumperPressed = gamepad2.right_bumper && !lastRightBumper;
        boolean leftBumperPressed  = gamepad2.left_bumper  && !lastLeftBumper;
        lastRightBumper = gamepad2.right_bumper;
        lastLeftBumper  = gamepad2.left_bumper;

        switch (shootState) {

            case IDLE:
                if (rightBumperPressed || leftBumperPressed) {
                    shooterSubsystem.setGatePosition(GATE_OPEN);
                    intakeSubsystem.setPower(-1.0);
                    shootTimer.reset();
                    shootState = ShootState.SHOOTING;
                }
                break;

            case SHOOTING:
                if (shootTimer.seconds() >= SHOOT_DURATION) {
                    shooterSubsystem.setGatePosition(GATE_CLOSED);
                    intakeSubsystem.stop();
                    shootState = ShootState.IDLE;
                }
                break;
        }

        // ── Intake manual control (gamepad2, blocked during sequence) ─────────
        if (shootState == ShootState.IDLE) {
            if (gamepad2.right_trigger > 0.05) {
                intakeSubsystem.setPower(-1.0);
            } else if (gamepad2.left_trigger > 0.05) {
                intakeSubsystem.setPower(1.0);
            } else {
                intakeSubsystem.stop();
            }
        }

        // ── Turret: slow trim via gamepad2 right stick X ──────────────────────
        double joystickX = gamepad2.right_stick_x;
        if (Math.abs(joystickX) > JOYSTICK_DEADBAND) {
            turretSubsystem.setTargetAngle(
                    turretSubsystem.getTargetAngle() + joystickX * DEGREES_PER_SECOND * dt);
        }
        turretSubsystem.update();

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
        telemetry.addData("Turret Target (deg)", "%.1f", turretSubsystem.getTargetAngle());
        telemetry.addData("Turret Actual (deg)", "%.1f", turretSubsystem.getCurrentAngle());
        telemetry.update();
    }
}
