package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Subsystem.DriveSubsystem;

public class MecanumDriveController {

    private final DriveSubsystem drive;

    public MecanumDriveController(DriveSubsystem drive) {
        this.drive = drive;
    }

    public void update(Gamepad gamepad) {
        drive.drive(
                -gamepad.left_stick_y,
                gamepad.left_stick_x,
                gamepad.right_stick_x
        );
    }
}