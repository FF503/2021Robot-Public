package org.frogforce503.lib.util;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class SwerveTranslationalPID {
    private final FrogPIDF x;
    private final FrogPIDF y;

    public SwerveTranslationalPID(FrogPIDF x, FrogPIDF y) {
        this.x = x;
        this.y = y;
    }

    public void updateSetpoint(Translation2d setpoint) {
        y.setSetpoint(setpoint.getY());
    }

    public Translation2d getOutput(Translation2d currentTranslation) {
        return new Translation2d(x.calculateOutput(currentTranslation.getX(), false), y.calculateOutput(currentTranslation.getY(), false));
    }

    public boolean onTarget() {
        return x.onTarget() && y.onTarget();
    }
}