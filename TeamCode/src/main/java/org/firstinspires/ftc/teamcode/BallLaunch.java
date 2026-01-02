package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class BallLaunch {
    public static String OUTTAKE_MOTOR_NAME = "outtake";
    public double targetVelocity = 70;
    public static double revperticks = 28;
    private Servo launchServo;
    public DcMotorEx outtake;
    private Telemetry telemetry;
    public BallLaunch(HardwareMap hardwareMap, Telemetry telemetry_) {
        telemetry = telemetry_;
        outtake = hardwareMap.get(DcMotorEx.class, OUTTAKE_MOTOR_NAME);
        launchServo = hardwareMap.get(Servo.class, "servoName");
        outtake.setDirection(DcMotor.Direction.FORWARD);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("BallLaunch", "Initialized");
    }
    public void start() {
        telemetry.addData("BallLaunch", "Starting");
        outtake.setPower(1.0);
    }
    public void stop() {
        telemetry.addData("BallLaunch", "Stopping");
        outtake.setPower(0.0);
        launchServo.setPosition(0);
        telemetry.addData("BallLaunch", "Stopped");
    }
    public int calculatePower(double distance) {
        // kinematics?
        return 0;
    }
    public double getRPM(){
        return outtake.getVelocity()*60/revperticks;
    }
    public double getRadPerSec(){
        return outtake.getVelocity()*2.0*Math.PI/revperticks;
    }
    public void launch() {
        launchServo.setPosition(0.4);
        telemetry.addData("BallLaunch", "Launched");
        outtake.setVelocity(0);
    }
}
