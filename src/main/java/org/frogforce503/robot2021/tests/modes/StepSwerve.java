package org.frogforce503.robot2021.tests.modes;

import org.frogforce503.robot2021.subsystems.swerve.Swerve;
import org.frogforce503.robot2021.tests.RobotTest;

public class StepSwerve extends RobotTest {

    int iter = 0;
    int spacing = 25;

    public StepSwerve() {
        super();
        this.setName("step_swerve");
    }

    public void setAngle(double angle) {
        Swerve.getInstance().getModules().forEach((mod) -> {
            mod.drive(503.0, angle);
        });
    }

    @Override
    public void run() {
        if (iter < spacing)
            Swerve.getInstance().snapForward();
        else if (iter < spacing * 2)
            setAngle(90);
        else if (iter < spacing * 3)
            setAngle(180);
        else if (iter < spacing * 4)
            setAngle(270);
        else
            iter = 0;

        iter++;
    }

    @Override
    public void stop() {
        Swerve.getInstance().stop();
    }

}
