package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.LaunchCalculator;

import java.util.function.Supplier;

@Autonomous (name="Test", group="Red")
public class AutoTest extends OpMode {
    private final Pose startPose = new Pose(117, 130, Math.toRadians(-324));
    private final Pose launchPose = new Pose(83, 84, LaunchCalculator.heading(83, 84, true));
    public enum STATES {
        INIT,
        SPIN_1,
        SPIN_2,
        SPIN_3,
        SPIN_4,
        SPIN_5,
        TURN_1,
        TURN_2,
        END

    }

    private STATES currentState = STATES.INIT;
    private Follower follower;

    private Lift lift;
    private BallLaunch ballLaunch;
    private Intake intake;

    private PathChain Spin1;
    private PathChain Spin2;
    private PathChain Spin3;
    private PathChain Spin4;
    private PathChain Spin5;
    private PathChain Turn1;
    private PathChain Turn2;
    private Supplier<PathChain> EndPathChain;
    private Timer opmodeTimer;

    public void buildPaths() {
      Spin1 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(117.000, 130.000),
            new Pose(83.000, 105.300)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(-324), Math.toRadians(-324))
        .build();

        Spin2 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(83.000, 105.300),
            new Pose(83.000, 84.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
        .build();

        Spin3 = follower.pathBuilder().addPath(
          new Path(
            new Pose(83.000, 84.000, follower.getHeading()),
            new Pose(83.000, 84.000, Math.toRadians(136))
          )
        ).setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(136))
        .build();

        Spin4 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(83.000, 84.000),
            new Pose(83.000, 105.300)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
        .build();

        Spin5 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(83.000, 105.300),
            new Pose(117.000, 130.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36))
        .build();

        Turn1 = follower.pathBuilder().addPath(
          new BezierCurve(
            new Pose(117.000, 130.000),
            new Pose(83.000, 105.300),
            new Pose(92.810, 67.123)
          )
        ).setTangentHeadingInterpolation()
        .setReversed(true)
        .build();
      
        Turn2 = follower.pathBuilder().addPath(
          new BezierCurve(
            new Pose(92.810, 67.123),
            new Pose(90.593, 76.415),
            new Pose(83.000, 84.000)
          )
        ).setTangentHeadingInterpolation()
        .build();

        EndPathChain = () -> follower.pathBuilder().addPath(
                        new Path(new BezierLine(
                                follower.getPose(),
                                new Pose(96, 72.000)
                        ))
                ).setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(-224))
                .build();
    }

    @Override
    public void init() {
        Globals.isRed = true;

        opmodeTimer = new Timer();

        ballLaunch = new BallLaunch(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();

        lift.setTargetTicks(Lift.minTicks);
    }

    private void update() {
        switch (currentState) {
            case INIT:
                currentState = STATES.SPIN_1;

//                ballLaunch.setTargetVelocity(Globals.SHORT_LAUNCH_VELOCITY);
//                ballLaunch.launchCount = 3;

                break;
            case SPIN_1:
                if (!follower.isBusy()) {
                    follower.followPath(Spin1);
                    currentState = STATES.SPIN_2;
                }
                break;
            case SPIN_2:
                if (!follower.isBusy()) {
                    follower.followPath(Spin2);
                    currentState = STATES.SPIN_3;
                }
                break;
            case SPIN_3:
                if (!follower.isBusy()) {
                    follower.followPath(Spin3);
                    currentState = STATES.SPIN_4;
                }
                break;
            case SPIN_4:
                if (!follower.isBusy()) {
                    follower.followPath(Spin4);
                    currentState = STATES.SPIN_5;
                }
                break;
            case SPIN_5:
                if (!follower.isBusy()) {
                    follower.followPath(Spin5);
                    currentState = STATES.TURN_1;
                }
                break;
            case TURN_1:
                if (!follower.isBusy()) {
                    follower.followPath(Turn1);
                    currentState = STATES.TURN_2;
                }
                break;
            case TURN_2:
                if (!follower.isBusy()) { 
                    follower.followPath(Turn2);
                    currentState = STATES.END;
                }
                break;
            case END:
                if (!follower.isBusy()) {
                    follower.followPath(EndPathChain.get());
                }
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        if (opmodeTimer.getElapsedTime() >= 25000 && currentState != STATES.END) {
            currentState = STATES.END;
            follower.followPath(EndPathChain.get());
        }

        update();

        ballLaunch.update();
        lift.update();


        telemetry.addData("ball launch", ballLaunch.currentState);
        telemetry.addData("launch count", ballLaunch.launchCount);
        telemetry.addData("STATE", currentState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }

    @Override
    public void stop() {
        Globals.PoseX = follower.getPose().getX();
        Globals.PoseY = follower.getPose().getY();
        Globals.PoseHeading = follower.getPose().getHeading();
    }
}
