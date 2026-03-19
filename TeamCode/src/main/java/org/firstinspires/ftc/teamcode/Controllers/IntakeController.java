package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;

public class IntakeController {

    private final IntakeSubsystem intake;

    public IntakeController(IntakeSubsystem intake) {
        this.intake = intake;
    }

    /**
     * Update intake based on either gamepad's triggers.
     * Right trigger = intake (1.0), left trigger = outtake (-1.0).
     * Right trigger takes priority if both are pressed.
     * Either gamepad can trigger — whichever has the larger input wins.
     */
    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp1.right_trigger > 0.1 || gp2.right_trigger > 0.1) {
            intake.setPower(1.0);
        } else if (gp1.left_trigger > 0.1 || gp2.left_trigger > 0.1) {
            intake.setPower(-1.0);
        } else {
            intake.stop();
        }
    }
}
