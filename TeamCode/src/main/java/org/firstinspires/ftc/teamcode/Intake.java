package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Intake {
    private final DcMotor intake;

    public static double power = 1.0;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
    }

    public void pullIn() {
        intake.setPower(power);
    }

    public void pushOut() {
        intake.setPower(-power);
    }

    public void stop() {
        intake.setPower(0.0);
    }
}
