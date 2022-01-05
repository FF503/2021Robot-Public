package org.frogforce503.lib.util;

public class DrivetrainFeedforwardConstants {
    private final double velocityConstant;
    private final double accelerationConstant;
    private final double staticConstant;

    public DrivetrainFeedforwardConstants(double velocityConstant, double accelerationConstant, double staticConstant) {
        this.velocityConstant = velocityConstant;
        this.accelerationConstant = accelerationConstant;
        this.staticConstant = staticConstant;
    }

    public double getVelocityConstant() {
        return velocityConstant;
    }

    public double getAccelerationConstant() {
        return accelerationConstant;
    }

    public double getStaticConstant() {
        return staticConstant;
    }
}
