// NC 2026

package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class BallLaunch {
    public enum STATES {
        IDLE,
        SPINNING_UP, // outtake is spinning up to velocity
        READY_TO_LAUNCH, // outtake is at velocity and ready to launch using launch()
        LAUNCHING // currently launching a ball (busy)
    }

    public STATES currentState = STATES.IDLE;

    public int launchCount = 0;
    public boolean forceLaunch = false; // If true, prepares launch even if launchCount is 0
    private double velocity = 2000;

    public static int reloadTime = 500; // minimum time (ms) between launches

    public static double servoStartPosition = 0.3;
    public static double servoLaunchPosition = 0.7;

    public static double velocityTolerance = 40.0;

    private final Timer launchTimer = new Timer();
    private final DcMotorEx outtake;
    private final Servo launchServo;

    public BallLaunch(HardwareMap hardwareMap) {
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotor.Direction.FORWARD);

        outtake.setVelocity(0);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launchServo = hardwareMap.get(Servo.class, "launch");
        launchServo.setDirection(Servo.Direction.REVERSE);

        launchServo.setPosition(servoStartPosition);
    }

    public void update() {
        switch (currentState) {
            case IDLE:
                if (launchCount > 0 || forceLaunch) {
                    currentState = STATES.SPINNING_UP;

                    outtake.setVelocity(velocity);
                }
                break;
            case SPINNING_UP:
                if (launchCount == 0 && !forceLaunch) {
                    currentState = STATES.IDLE;
                    outtake.setVelocity(0);

                    break;
                }

                if (Math.abs(outtake.getVelocity() - velocity) < velocityTolerance) {
                    currentState = STATES.READY_TO_LAUNCH;
                }
                break;
            case READY_TO_LAUNCH:
                if (launchCount == 0 && !forceLaunch) {
                    currentState = STATES.IDLE;
                    outtake.setVelocity(0);
                }

                break;
            case LAUNCHING:
                if (launchCount == 0 && !forceLaunch) {
                    currentState = STATES.IDLE;
                    outtake.setVelocity(0);

                    break;
                }

                if (launchTimer.getElapsedTime() > reloadTime) {
                    launchServo.setPosition(servoStartPosition);
                    if (launchCount > 0 || forceLaunch) {
                        if (Math.abs(outtake.getVelocity() - velocity) < velocityTolerance) {
                            currentState = STATES.READY_TO_LAUNCH;
                        } else {
                            currentState = STATES.SPINNING_UP;
                        }
                    } else {
                        currentState = STATES.IDLE;
                        outtake.setVelocity(0);
                    }
                }
                break;
        }
    }

    public boolean launch() {
        // Returns true if launch initiated

        if (currentState == STATES.READY_TO_LAUNCH) {
            currentState = STATES.LAUNCHING;
            launchServo.setPosition(servoLaunchPosition);
            launchTimer.resetTimer();

            if (launchCount > 0) {
                launchCount--;
            }

            return true;
        }
        return false;
    }

    public void setTargetVelocity(double targetVelocity) {
        velocity = targetVelocity;

        if (currentState == STATES.SPINNING_UP || currentState == STATES.READY_TO_LAUNCH || currentState == STATES.LAUNCHING) {
            outtake.setVelocity(velocity);

            if (Math.abs(outtake.getVelocity() - velocity) < velocityTolerance && currentState != STATES.LAUNCHING) {
                currentState = STATES.READY_TO_LAUNCH;
            } else if (currentState != STATES.LAUNCHING) {
                currentState = STATES.SPINNING_UP;
            }
        }
    }

    public double getTargetVelocity() {
        return velocity;
    }

    public double getCurrentVelocity() {
        return outtake.getVelocity();
    }
}
