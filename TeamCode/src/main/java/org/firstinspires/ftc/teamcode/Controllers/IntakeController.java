package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeController {
    public Gamepad gamepad;
    public Gamepad gamepad1;
    private Intake intake;

    public IntakeController(Gamepad gamepad, Intake intake) {
        this.gamepad = gamepad;
        this.intake = intake;
    }

    public void update() {
        if (gamepad == null) return;

        boolean gp1Intake = gamepad1 != null && gamepad1.left_trigger > 0.5;

        if (gamepad.right_trigger > 0.5 || gp1Intake) {
            intake.on();
        } else if (gamepad.left_trigger > 0.5) {
            intake.reverse();
        } else {
            intake.off();
        }
    }
}