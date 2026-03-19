package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Intake subsystem.
 *
 * Motor configuration (hardware map name):
 *   "Intake" – intake motor
 */
public class IntakeSubsystem {

    private final DcMotor intakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Set intake motor power. Positive = intake, negative = outtake. */
    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}
