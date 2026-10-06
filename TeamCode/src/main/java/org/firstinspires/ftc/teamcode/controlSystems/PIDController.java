package org.firstinspires.ftc.teamcode.controlSystems;

import com.qualcomm.robotcore.util.Range;

public class PIDController {
    private double kP, kI, kD, kF, integral, lastError, lastTime;
    private boolean isVelocity;
    private double deadband = 0.15; 
    private double ffRampRange = 5.0; // Angle range over which kF is linearly ramped

    public PIDController(double p, double i, double d, double f, boolean velocityMode){
        kP = p;
        kI = i;
        kD = d;
        kF = f;
        lastTime = System.nanoTime() / 1e9;
        isVelocity = velocityMode;
        reset();
    }

    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    public double update(double target, double current) {
        return update(target, current, 12.0);
    }

    public double update(double target, double current, double voltage){
        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;
        if (dt <= 0) return 0;

        lastTime = currentTime;

        double error = target - current;
        double absError = Math.abs(error);
        
        // --- DEADBAND LOGIC ---
        if (!isVelocity && absError < deadband) {
            integral = 0;
            lastError = error;
            return 0;
        }

        if (kI != 0) {
            integral += error * dt;
            integral = Range.clip(integral, -1, 1);
        } else {
            integral = 0;
        }

        double derivative = (error - lastError) / dt;
        lastError = error;

        double ff = 0;
        if (isVelocity) {
            ff = kF * (target / 5200);
        } else {
            // --- RAMPED FEEDFORWARD ---
            // Instead of a hard jump, we ramp kF from 0 to full over 'ffRampRange'
            // This removes the "kick" that causes micro-oscillations/chatter.
            double ffScale = Math.min(absError / ffRampRange, 1.0);
            ff = Math.signum(error) * kF * ffScale;
        }

        double voltageComp = 12.0 / Math.max(voltage, 1.0);
        double output = (ff + (kP * error) + (kI * integral) + (kD * derivative)) * voltageComp;
        
        return Range.clip(output, -1, 1);
    }

    public void reset() {
        lastTime = System.nanoTime() / 1e9;
        integral = 0;
        lastError = 0;
    }

    public void setGains(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
    }
}
