package org.firstinspires.ftc.teamcode;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;
@Configurable
@TeleOp
public class MainTeleOp extends OpMode {
    private Follower follower;
    private boolean parking;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor intake;
    private DcMotorEx outtake;

    private Servo launchServo;

    private Lift lift;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(Globals.PoseX, Globals.PoseY, Globals.PoseHeading));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setDirection(DcMotor.Direction.REVERSE);

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotor.Direction.FORWARD);

        lift = new Lift(hardwareMap);

        launchServo = hardwareMap.get(Servo.class, "launch");
        launchServo.setDirection(Servo.Direction.REVERSE);
    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!parking) {

            if (gamepad1.left_trigger > 0.05 && gamepad1.right_trigger <= 0.05) { // Strafe left using left trigger
                follower.setTeleOpDrive(
                        0,
                        gamepad1.left_trigger / 1.5,
                        0,
                        true // Robot Centric
                );

            } else if (gamepad1.right_trigger > 0.05 && gamepad1.left_trigger <= 0.05) { // Strafe right using right trigger
                follower.setTeleOpDrive(
                        0,
                        -gamepad1.right_trigger / 1.5,
                        0,
                        true // Robot Centric
                );
            } else { // Both triggers are pressed, normal driving
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true // Robot Centric
                );
            }
        }

        if (gamepad2.dpad_right) {
            intake.setPower(0.75);
        } else if (gamepad2.dpad_left) {
            intake.setPower(-0.75);
        } else {
            intake.setPower(0.0);
        }

        if (gamepad1.dpadUpWasPressed()) {
            outtake.setPower(1.0);
        }
        if (gamepad1.dpadUpWasReleased()) {
            outtake.setPower(0.0);
        }

        if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            lift.up();
        } else if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            lift.down();
        }

        lift.update(); // Update lift control loop


        final double LAUNCH_SERVO_EXTENDED = 0.7;
        final double LAUNCH_SERVO_RETRACTED = 0.31;

        if (gamepad1.aWasPressed()) {
            launchServo.setPosition(LAUNCH_SERVO_EXTENDED);
        } else if (gamepad1.aWasReleased()) {
            launchServo.setPosition(LAUNCH_SERVO_RETRACTED);
        }

        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            parking = true;
        }
        //Stop automated following if the follower is done
        if (parking && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            parking = false;
        }

//
//        if (!parking) {
//            //Make the last parameter false for field-centric
//            //In case the drivers want to use a "slowMode" you can scale the vectors
//            //This is the normal version to use in the TeleOp
//            if (!slowMode) follower.setTeleOpDrive(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x,
//                    -gamepad1.right_stick_x,
//                    true // Robot Centric
//            );
//                //This is how it looks with slowMode on
//            else follower.setTeleOpDrive(
//                    -gamepad1.left_stick_y * slowModeMultiplier,
//                    -gamepad1.left_stick_x * slowModeMultiplier,
//                    -gamepad1.right_stick_x * slowModeMultiplier,
//                    true // Robot Centric
//            );
//        }

        telemetryM.debug("lift target", lift.targetTicks);
        telemetryM.debug("lift left pos", lift.liftLeft.getCurrentPosition());
        telemetryM.debug("lift right pos", lift.liftRight.getCurrentPosition());

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("heading", Math.toDegrees(follower.getHeading()));

        telemetryM.debug("velocity", follower.getVelocity());
    }
}