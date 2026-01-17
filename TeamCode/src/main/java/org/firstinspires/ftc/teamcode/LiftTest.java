package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Configurable
@TeleOp(name = "Lift Test", group = "Main")
public class LiftTest extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx liftLeft;
    private DcMotorEx liftRight;

    private TelemetryManager telemetryM;

    public static double liftPowerUp = 0.75;
    public static double liftPowerDown = 0.1;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        liftLeft = hardwareMap.get(DcMotorEx.class, "lift_left");
        liftRight = hardwareMap.get(DcMotorEx.class, "lift_right");



        // CHANGE
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setDirection(DcMotor.Direction.FORWARD);





        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {

        telemetryM.update();


        if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            liftLeft.setPower(liftPowerUp);
            liftRight.setPower(liftPowerUp);
        } else if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            liftLeft.setPower(-liftPowerDown);
            liftRight.setPower(-liftPowerDown);
        } else {
            liftLeft.setPower(0);
            liftRight.setPower(0);
        }



        telemetryM.debug("Run Time", runtime);

        telemetryM.debug("left lift Position", liftLeft.getCurrentPosition());
        telemetryM.debug("right lift Position", liftRight.getCurrentPosition());
        telemetryM.debug("lift power left/Right", "%4.2f, %4.2f", liftLeft.getPower(), liftRight.getPower());
    }

    @Override
    public void stop() {
    }
}
