package org.firstinspires.ftc.teamcode;

public class PIDController {
    private double kP, kI, kD;
    private double I = 0;
    private double lastError = 0;
    private long lastTime = 0;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double update(double error) {
        long now = System.nanoTime();

        if (lastTime == 0) {
            lastTime = now;
            lastError = error;
            return kP * error;
        }

        double dt = (now - lastTime) * 1e-9;
        lastTime = now;

        if (dt <= 0) return 0;

        // Integral
        I += error * dt;
        //integral = clamp(integral, -integralLimit, integralLimit);

        // Derivative
        double derivative = (error - lastError) / dt;
        lastError = error;

        return kP * error + kI * I + kD * derivative;
    }

    public void reset() {
        I = 0;
        lastError = 0;
        lastTime = 0;
    }
}