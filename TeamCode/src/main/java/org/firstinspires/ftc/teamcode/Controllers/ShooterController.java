package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;

public class ShooterController {
    public Gamepad gamepad;
    public Gamepad gamepad1;
    private Shooter shooter;

    private boolean prevRightBumper = false;
    private boolean prevGp1RightBumper = false;

    public ShooterController(Gamepad gamepad, Shooter shooter) {
        this.gamepad = gamepad;
        this.shooter = shooter;
    }

    public void update(Intake intake) {
        if (gamepad == null) return;

        // Right Bumper (gp2) or Right Bumper (gp1) — fire
        boolean gp1Fire = gamepad1 != null && gamepad1.right_bumper && !prevGp1RightBumper;
        if ((gamepad.right_bumper && !prevRightBumper) || gp1Fire) {
            shooter.startShoot();
        }

        // Hood and Velocity updated by Robot.update() via odometry distance
        shooter.updateFSM(intake);

        prevRightBumper = gamepad.right_bumper;
        prevGp1RightBumper = gamepad1 != null ? gamepad1.right_bumper : false;
    }

    public boolean isShooting() {
        return shooter.isShooting();
    }

    public Shooter.ShooterState getCurrentState() {
        return shooter.getCurrentState();
    }
}
