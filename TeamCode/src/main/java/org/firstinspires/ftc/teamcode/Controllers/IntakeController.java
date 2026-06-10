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

        // GP1 right trigger → intake ON, GP1 left trigger → intake REVERSE
        boolean gp1RightTrigger = gamepad.right_trigger > 0.5;
        boolean gp1LeftTrigger  = gamepad.left_trigger  > 0.5;

        if (gp1RightTrigger) {
            intake.on();
        } else if (gp1LeftTrigger) {
            intake.reverse();
        } else {
            intake.off();
        }
    }
}