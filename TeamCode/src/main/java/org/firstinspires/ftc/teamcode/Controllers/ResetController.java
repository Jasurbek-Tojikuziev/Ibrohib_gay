package org.firstinspires.ftc.teamcode.Controllers;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Localizer;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class ResetController {
    private IntakeController intakeController;
    private ShooterController shooterController;
    private TurretController turretController;

    private Intake   intake;
    private Shooter  shooter;
    private Turret   turret;
    private Follower follower;  // needed to snapshot current pose into Localizer on reset

    private boolean wasResetPressed = false;

    public ResetController(IntakeController intakeController,
                           ShooterController shooterController,
                           TurretController turretController,
                           Intake intake,
                           Shooter shooter,
                           Turret turret,
                           Follower follower) {
        this.intakeController  = intakeController;
        this.shooterController = shooterController;
        this.turretController  = turretController;
        this.intake   = intake;
        this.shooter  = shooter;
        this.turret   = turret;
        this.follower = follower;
    }

    private boolean resetInProgress = false;

    public void handleResetButton(Gamepad gamepad2) {
        // Снимаем флаг как только турель вернулась в центр
        if (resetInProgress && turret.isCentered()) {
            resetInProgress = false;
        }

        if (gamepad2.options && !wasResetPressed && !resetInProgress) {
            wasResetPressed = true;
            resetInProgress = true;
            resetControls();
        }
        if (!gamepad2.options) wasResetPressed = false;
    }

    private void resetControls() {
        // Сброс контроллеров - включаем auto-aim обратно
        turretController.enableAutoAim();

        // Сброс подсистем
        intake.off();
        shooter.reset(); // Полный сброс shooter (моторы, servos, FSM)

        // Снова запускаем моторы с правильной velocity на основе расстояния
        double distanceToGoal = turret.getDistanceToGoal();
        if (distanceToGoal > 0) {
            shooter.updateVelocity(distanceToGoal);
            shooter.updateHood(distanceToGoal);
        } else {
            shooter.on(); // Fallback: стандартная velocity
        }

        turret.returnToCenter(); // Return turret to 0° with PID

        // Forget auto values: overwrite Localizer with current physical position.
        // If TeleOp is restarted now, it starts from the reset position, not auto.
        if (follower != null) {
            Pose cur = follower.getPose();
            Localizer loc = Localizer.getInstance();
            if (loc != null) {
                loc.setPosition(cur.getX(), cur.getY(), Math.toDegrees(cur.getHeading()));
            }
        }

        wasResetPressed = false;
    }

}
