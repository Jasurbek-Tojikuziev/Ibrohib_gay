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
 * Fixed-Turret TeleOp.
 *
 *   - gamepad1: drivetrain only.
 *   - gamepad2: intake (triggers), shoot (bumpers), turret trim (right stick X).
 *   - Right bumper: shoot at 1200 ticks/s. Left bumper: shoot at 1400 ticks/s.
 *   - Shoot sequence waits until motor reaches 97% of target velocity, then
 *     opens gate + runs intake for 1.5 s, then closes/stops. Idle speed is 1300.
 *   - Turret locks to 0° on start. Gamepad2 right stick X rotates it slowly;
 *     releasing the stick holds the current angle via PID.
 *   - Shooter PIDF locked to P=70.4, F=12.24.
 *   - Compression servo is never touched (stays at default 1.0).
 */
@TeleOp(name = "Fixed Turret TeleOp", group = "TeleOp")
public class FixedTurretNazarX extends OpMode {

    private static final double FIXED_P           = 70.400;
    private static final double FIXED_F           = 13.00;

    private static final double VELOCITY_IDLE      = 1175.0; // standby speed
    private static final double VELOCITY_RIGHT     = 1000.0; // right bumper shoot speed
    private static final double VELOCITY_LEFT      = 1350.0; // left bumper shoot speed
    private static final double VELOCITY_TOLERANCE = 0.01;   // fire only when within 1% of target (accel AND decel)

    private static final double GATE_OPEN          = 1.0;    // shooterStop open position
    private static final double GATE_CLOSED        = 0.74;   // shooterStop closed position
    private static final double HOOD_RIGHT         = 0.1;    // hood for right bumper (1000 ticks/s)
    private static final double HOOD_LEFT          = 0.9;    // hood for left bumper (1350 ticks/s)
    private static final double HOOD_DEFAULT       = 0.1;    // hood resting position

    private static final double SHOOT_DURATION    = 1.5;    // seconds

    private static final double YAW_SCALE          = 0.8;    // rotation 20% slower than full speed
    private static final double JOYSTICK_DEADBAND  = 0.05;
    private static final double DEGREES_PER_SECOND = 30.0;  // slow trim speed

    private enum ShootState { IDLE, SPOOLING, SHOOTING }

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
    private ShootState          shootState       = ShootState.IDLE;
    private double              targetVelocity   = VELOCITY_IDLE;
    private boolean             lastRightBumper  = false;
    private boolean             lastLeftBumper   = false;
    private final ElapsedTime   shootTimer       = new ElapsedTime();

    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        ShooterSubsystem.PIDF_P          = FIXED_P;
        ShooterSubsystem.PIDF_F          = FIXED_F;
        ShooterSubsystem.TARGET_VELOCITY = VELOCITY_IDLE;

        // ── Drive motors — NazarX original directions ─────────────────────────
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
        shooterSubsystem.reverseMotors();
        intakeSubsystem  = new IntakeSubsystem(hardwareMap);
        turretSubsystem  = new TurretSubsystem(hardwareMap);

        shooterSubsystem.setHoodPosition(HOOD_DEFAULT);
        shooterSubsystem.setGatePosition(GATE_CLOSED);

        telemetry.addLine("Init complete — Fixed Turret NazarX");
        telemetry.update();
    }

    @Override
    public void start() {
        shooterSubsystem.startMotors();
        turretSubsystem.setTargetAngle(0);  // lock to 0° on start
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // ── Drive (gamepad1 only) — right stick X is rotation on NazarX ─────
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
                // Fire only when velocity is within 3% of target — handles both acceleration and deceleration
                if (Math.abs(shooterSubsystem.getMasterVelocity() - targetVelocity) <= targetVelocity * VELOCITY_TOLERANCE) {
                    shooterSubsystem.setGatePosition(GATE_OPEN);
                    intakeSubsystem.setPower(-1.0);
                    shootTimer.reset();
                    shootState = ShootState.SHOOTING;
                }
                break;

            case SHOOTING:
                if (shootTimer.seconds() >= SHOOT_DURATION) {
                    // End sequence: close gate, stop intake, reset hood, return to idle speed
                    shooterSubsystem.setGatePosition(GATE_CLOSED);
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
