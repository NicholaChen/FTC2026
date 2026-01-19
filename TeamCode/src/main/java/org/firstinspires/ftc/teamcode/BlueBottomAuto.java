package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name="Blue Bottom", group="Blue")
public class BlueBottomAuto extends OpMode {
    private final Pose startPose = new Pose(56.000, 9.000, Math.toRadians(-90));
    private final Pose launchPose1 = new Pose(61.000, 25.000, Math.toRadians(-154));
    private final Pose launchPose2 = new Pose(61.000, 84.000, Math.toRadians(-136));
    public enum STATES {
        INIT,
        TO_INITIAL_LAUNCH,
        INITIAL_LAUNCH,
        TO_INTAKE_1,
        INTAKE_1,
        TO_LAUNCH_1,
        LAUNCH_1,
        TO_INTAKE_2,
        INTAKE_2,
        TO_LAUNCH_2,
        LAUNCH_2,
        END
    }

    private STATES currentState = STATES.INIT;
    private Follower follower;

    private BallLaunch ballLaunch;
    private Intake intake;

    private PathChain ToInitialLaunch;
    private PathChain ToIntake1;
    private PathChain Intake1;
    private PathChain ToLaunch1;
    private PathChain ToIntake2;
    private PathChain Intake2;
    private PathChain ToLaunch2;
    private Supplier<PathChain> EndPathChain;
    private Timer opmodeTimer;

    public void buildPaths() {
        ToInitialLaunch = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                launchPose1
                        )
                ).setLinearHeadingInterpolation(startPose.getHeading(), launchPose1.getHeading())
                .build();

        ToIntake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                launchPose1,
                                new Pose(launchPose1.getX(), 36.000),
                                new Pose(44.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(launchPose1.getHeading(), Math.toRadians(180))
                .setNoDeceleration()
                .build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 36.000),
                                new Pose(9.000, 36.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        ToLaunch1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(9.000, 36.000),
                                new Pose(launchPose1.getX(), 36.000),
                                launchPose1
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), launchPose1.getHeading())
                .build();
        
        ToIntake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                launchPose1,
                                new Pose(launchPose1.getX(), 60.000),
                                new Pose(44.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(launchPose1.getHeading(), Math.toRadians(180))
                .setNoDeceleration()
                .build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 60.000),
                                new Pose(9.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        ToLaunch2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(9.000, 60.000),
                                new Pose(launchPose2.getX(), 60.000),
                                launchPose2
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), launchPose2.getHeading())
                .build();

        EndPathChain = () -> follower.pathBuilder().addPath(
                        new Path(new BezierLine(
                                follower.getPose(),
                                new Pose(48.000, 72.000)
                        ))
                ).setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(180))
                .build();
    }

    @Override
    public void init() {
        Globals.isRed = false;

        opmodeTimer = new Timer();

        ballLaunch = new BallLaunch(hardwareMap);
        intake = new Intake(hardwareMap);

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
    }

    private void update() {
        switch (currentState) {
            case INIT:
                follower.followPath(ToInitialLaunch);
                currentState = STATES.TO_INITIAL_LAUNCH;

                ballLaunch.setTargetVelocity(2000); // TODO
                ballLaunch.launchCount = 3;

                break;
            case TO_INITIAL_LAUNCH:
                if (!follower.isBusy()) {
                    currentState = STATES.INITIAL_LAUNCH;
                }
                break;
            case INITIAL_LAUNCH:
                if (ballLaunch.currentState == BallLaunch.STATES.IDLE) {
                    follower.followPath(ToIntake1);
                    currentState = STATES.TO_INTAKE_1;
                } else {
                    ballLaunch.launch(); // continuously try launching
                }
                break;
            case TO_INTAKE_1:
                if (!follower.isBusy()) {
                    follower.followPath(Intake1);
                    currentState = STATES.INTAKE_1;
                    intake.pullIn();
                }
                break;
            case INTAKE_1:
                if (!follower.isBusy()) {
                    intake.stop();
                    follower.followPath(ToLaunch1);
                    currentState = STATES.TO_LAUNCH_1;

                    ballLaunch.setTargetVelocity(2000); // TODO
                    ballLaunch.launchCount = 3;
                }
                break;
            case TO_LAUNCH_1:
                if (!follower.isBusy()) {
                    currentState = STATES.LAUNCH_1;
                }
                break;
            case LAUNCH_1:
                if (ballLaunch.currentState == BallLaunch.STATES.IDLE) {
                    follower.followPath(ToIntake2);
                    currentState = STATES.TO_INTAKE_2;
                } else {
                    ballLaunch.launch(); // continuously try launching
                }
                break;
            case TO_INTAKE_2:
                if (!follower.isBusy()) {
                    follower.followPath(Intake2);
                    currentState = STATES.INTAKE_2;
                    intake.pullIn();
                }
                break;
            case INTAKE_2:
                if (!follower.isBusy()) {
                    intake.stop();
                    follower.followPath(ToLaunch2);
                    currentState = STATES.TO_LAUNCH_2;

                    ballLaunch.setTargetVelocity(2000); // TODO
                    ballLaunch.launchCount = 3;
                }
                break;
            case TO_LAUNCH_2:
                if (!follower.isBusy()) {
                    currentState = STATES.LAUNCH_2;
                }
                break;
            case LAUNCH_2:
                if (ballLaunch.currentState == BallLaunch.STATES.IDLE) {
                    currentState = STATES.END;
                    follower.followPath(EndPathChain.get());
                } else {
                    ballLaunch.launch(); // continuously try launching
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
