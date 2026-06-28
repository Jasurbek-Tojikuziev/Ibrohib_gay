package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intake;
    private DcMotor intake2;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "Intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake2 = hardwareMap.get(DcMotor.class, "Intake2");
        // VERIFY DIRECTION: if the two motors are mounted MIRRORED (facing each other), this must be
        // the OPPOSITE of intake (FORWARD) so they pull together. If mounted/geared the same way,
        // keep it REVERSE. Wrong direction = the two motors fight each other.
        intake2.setDirection(DcMotor.Direction.REVERSE);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void on() {
        setPower(1.0);
    }

    public void off() {
        setPower(0.0);
    }

    public void reverse() {
        setPower(-1.0);
    }

    public void setPower(double power) {
        intake.setPower(power);
        intake2.setPower(power);
    }
}
