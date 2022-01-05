package org.frogforce503.lib.control;

import org.frogforce503.lib.math.MathUtils;

public class PidController {
    private final PidConstants constants;
    public double lastError = Double.NaN;
    public double integralAccum = 0.0;
    private double setpoint;
    private boolean continuous = false;
    private double inputRange = Double.POSITIVE_INFINITY;
    private double minOutput = Double.NEGATIVE_INFINITY;
    private double maxOutput = Double.POSITIVE_INFINITY;

    public PidController(PidConstants constants) {
        this.constants = constants;
    }

    public double calculate(double current, double dt) {
        double error = setpoint - current;
        if (continuous) {
            error %= inputRange;
            if (Math.abs(error) > inputRange / 2.0) {
                if (error > 0.0) {
                    error -= inputRange;
                } else {
                    error += inputRange;
                }
            }
        }

        double integral = 0.0;
        double integralRange = Double.POSITIVE_INFINITY;
        if (Math.abs(error) < integralRange / 2.0) {
            integral = integralAccum + error * dt;
        }
        integralAccum = integral;

        double derivative = 0.0;
        if (Double.isFinite(lastError)) {
            derivative = (error - lastError) / dt;
        }
        lastError = error;

        return MathUtils.clamp(constants.p * error + constants.i * integral + constants.d * derivative, minOutput,
                maxOutput);
    }

    public void reset() {
        lastError = Double.NaN;
        integralAccum = 0.0;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setContinuous(boolean continuous) {
        this.continuous = continuous;
    }

    public void setInputRange(double minInput, double maxInput) {
        this.inputRange = maxInput - minInput;
    }

}
