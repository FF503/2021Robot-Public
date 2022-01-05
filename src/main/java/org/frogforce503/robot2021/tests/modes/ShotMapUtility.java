package org.frogforce503.robot2021.tests.modes;

import org.frogforce503.robot2021.subsystems.Turret;
import org.frogforce503.robot2021.subsystems.swerve.Swerve;
import org.frogforce503.robot2021.tests.RobotTest;

public class ShotMapUtility extends RobotTest {

    private double hoodAngle;
    private double shooterVelocity;

    @Override
    public void run() {

    }

    void drive() {

    }

    @Override
    public void stop() {
        Swerve.getInstance().stop();
        Turret.getInstance().stop();
    }

}
