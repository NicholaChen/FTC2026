package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends OpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor intake;
    private BallLaunch ballLaunch;
    private DcMotor liftLeft;
    private DcMotor liftRight;

    private ElapsedTime timerA = new ElapsedTime(); //timer for the ball launch

    private ElapsedTime timerB = new ElapsedTime(); //timer for another mystery thing
    double maxLiftPower = 0.50;
    boolean BallLaunchStatus;

    double currentTime;




    private CameraVision cameraVision;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        telemetry.addData("Status", "Initializing");

        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setDirection(DcMotor.Direction.REVERSE);

        ballLaunch = new BallLaunch(hardwareMap, telemetry);
        //cameraVision = new CameraVision(hardwareMap, telemetry);
        liftLeft = hardwareMap.get(DcMotor.class, "lift_left");
        liftRight = hardwareMap.get(DcMotor.class, "lift_right");
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setDirection(DcMotor.Direction.FORWARD);


        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BallLaunchStatus = false;


        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
        //variables
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);

        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double frontLeftPower = power * cos / max + turn;
        double frontRightPower = power * sin / max - turn;
        double backLeftPower = power * sin / max + turn;
        double backRightPower = power * cos / max - turn;

        //
        if ((power + Math.abs(turn)) > 1) {
            frontLeftPower /= power + Math.abs(turn);
            frontRightPower /= power + Math.abs(turn);
            backLeftPower /= power + Math.abs(turn);
            backRightPower /= power + Math.abs(turn);
        }
        // controller keybinds
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        if (gamepad1.dpad_left) {
            intake.setPower(1.0);
        } else if (gamepad1.dpad_right) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }

        if (gamepad1.dpad_up) {
            if (!BallLaunchStatus){
                timerA.reset();
                ballLaunch.outtake.setVelocity(ballLaunch.targetVelocity);
                BallLaunchStatus = true;
            }
            if (timerA.seconds() >= 2.0 && ballLaunch.targetVelocity <= ballLaunch.getRadPerSec()){
                ballLaunch.launch();
            }
        } else {
            ballLaunch.stop();
        }

        //linear slides
        if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            liftLeft.setPower(maxLiftPower);
            liftRight.setPower(maxLiftPower);
            telemetry.addData("Lift", "up");
        } else if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            liftLeft.setPower(-maxLiftPower);
            liftRight.setPower(-maxLiftPower);
            telemetry.addData("Lift", "down");
        } else {
            liftLeft.setPower(0.0);
            liftRight.setPower(0.0);
            telemetry.addData("Lift", "");
        }
        if (liftRight.getCurrentPosition() == 767.2) {
            liftLeft.setPower(0);
            liftRight.setPower(0);
        }



        // List<AprilTagDetection> detections = cameraVision.detect();
        // telemetry.addData("aprilTags", detections.size());

        telemetry.addData("Run Time", runtime);
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
