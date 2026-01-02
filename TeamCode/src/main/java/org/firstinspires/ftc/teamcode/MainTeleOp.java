package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

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

    private DcMotorEx liftLeft;
    private DcMotorEx liftRight;

    double maxLiftPower = 0.75;



    private CameraVision cameraVision;

    private Servo launchServo;

    private double lastFrontLeftPower;
    private double lastFrontRightPower;
    private double lastBackLeftPower;
    private double lastBackRightPower;

    private MecanumDrive drive;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        telemetry.addData("Status", "Initializing");

        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setDirection(DcMotor.Direction.REVERSE);

        ballLaunch = new BallLaunch(hardwareMap);
        //cameraVision = new CameraVision(hardwareMap, telemetry);
        liftLeft = hardwareMap.get(DcMotorEx.class, "lift_left");
        liftRight = hardwareMap.get(DcMotorEx.class, "lift_right");

        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setDirection(DcMotor.Direction.FORWARD);

        //liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        launchServo = hardwareMap.get(Servo.class, "launch");
        launchServo.setDirection(Servo.Direction.REVERSE);


        drive = new MecanumDrive(hardwareMap, new Pose2d(Globals.PoseX, Globals.PoseY, Globals.PoseHeading));
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

    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

        drive.updatePoseEstimate();

        Pose2d pose = drive.localizer.getPose();

        if (gamepad1.dpad_right) {
            intake.setPower(0.75);
        } else if (gamepad1.dpad_left) {
            intake.setPower(-0.75);
        } else {
            intake.setPower(0.0);
        }

        if (gamepad1.dpad_up) {
            ballLaunch.start();
        } else {
            ballLaunch.stop();
        }

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
        if (liftRight.getPower() == maxLiftPower) {
            liftLeft.setPower(0);
            liftRight.setPower(0);
        }

        if (gamepad1.aWasPressed()) {
            launchServo.setPosition(0.7);
        } else if (gamepad1.aWasReleased()) {
            launchServo.setPosition(0.36);
        }

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);

        telemetry.addData("Run Time", runtime);
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", drive.leftFront.getPower(), drive.rightFront.getPower());
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", drive.leftBack.getPower(), drive.rightBack.getPower());
        telemetry.update();
        dash.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
    }
}
