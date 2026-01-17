package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Configurable
public class Lift {
    public DcMotorEx liftLeft;
    public DcMotorEx liftRight;

    public static double posP = 0.01;
    public static double posI = 0.0;
    public static double posD = 0.0001;
    public static double syncP = 0.01;
    public static double syncI = 0.0;
    public static double syncD = 0.0001;
    public static double kHold = 0;

    public static int upIncrement = 15;
    public static int downIncrement = 6;

    private final PIDController posPID  = new PIDController(posP, posI, posD);
    private final PIDController syncPID = new PIDController(syncP, syncI, syncD);

    public int targetTicks = 0;
    public double currentTicks = 0;
    private int prevTargetTicks = Integer.MIN_VALUE;

    public static int maxTicks = 2000;
    public static int minTicks = -60;

    public Lift(HardwareMap hardwareMap) {
        liftLeft = hardwareMap.get(DcMotorEx.class, "lift_left");
        liftRight = hardwareMap.get(DcMotorEx.class, "lift_right");

        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setDirection(DcMotor.Direction.FORWARD);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void up() {
         targetTicks += upIncrement;


        if (targetTicks > maxTicks) {
            targetTicks = maxTicks;
        }
    }

    public void down() {
        targetTicks -= downIncrement;

        if (targetTicks < minTicks) {
            targetTicks = minTicks;
        }
    }

    public void update() {
        if (targetTicks != prevTargetTicks) {
            posPID.reset();
            prevTargetTicks = targetTicks;
        }

        int x1 = liftLeft.getCurrentPosition();
        int x2 = liftRight.getCurrentPosition();

        currentTicks = (x1 + x2) / 2.0;

        double avgPos = (x1 + x2) / 2.0;
        double ePos  = targetTicks - avgPos;
        double eSync = x1 - x2;

        double uPos  = posPID.update(ePos);
        double uSync = syncPID.update(eSync);

        double hold1 = (x1 >= 0) ? kHold : 0.0;
        double hold2 = (x2 >= 0) ? kHold : 0.0;

        double power1 = uPos + uSync + hold1;
        double power2 = uPos - uSync + hold2;

        power1 = Math.max(-1.0, Math.min(1.0, power1));
        power2 = Math.max(-1.0, Math.min(1.0, power2));

        liftLeft.setPower(power1);
        liftRight.setPower(power2);
    }

    public void reset() {
        posPID.reset();
        syncPID.reset();
    }
}
