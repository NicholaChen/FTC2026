package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class BallLaunch {
    public static String OUTTAKE_MOTOR_NAME = "outtake";

    public static double revperticks = 28;
    private Servo launchServo;
    private DcMotorEx outtake;
    private Telemetry telemetry;

    public BallLaunch(HardwareMap hardwareMap, Telemetry telemetry_) {
        telemetry = telemetry_;
        outtake = hardwareMap.get(DcMotorEx.class, OUTTAKE_MOTOR_NAME);
        launchServo = hardwareMap.get(Servo.class, "servoName");
        outtake.setDirection(DcMotor.Direction.FORWARD);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("BallLaunch", "Initialized");
    }

    public int try_launch() {
        // check if the motor is at correct speed

        launch();

        telemetry.addData("BallLaunch", "Launched");
        return 0; // success
    }

    public void launch() {

    }

    public void start() {
        telemetry.addData("BallLaunch", "Starting");
        outtake.setPower(1.0);
    }

    public void stop() {
        telemetry.addData("BallLaunch", "Stopping");
        outtake.setPower(0.0);
    }

    public int calculatePower(double distance) {
        // kinematics?

        return 0;

    }
    public double getTicksPerSec(){
        double ticksVelocity = outtake.getVelocity();
        return ticksVelocity*60/revperticks;
    }
}
