package org.frogforce503.robot2021.tests.modes;

import org.frogforce503.robot2021.subsystems.swerve.Swerve;
import org.frogforce503.robot2021.tests.RobotTest;
import org.frogforce503.robot2021.tests.TestInput;

public class SwerveSpin extends RobotTest {

    TestInput<Double> speed = new TestInput<Double>("speed", Double.valueOf(0.3));
    double angle = 0;
    boolean direction = false;

    @SuppressWarnings("unchecked")
    public SwerveSpin() {
        super();
        // this.setSubsystem(Swerve.getInstance());
        this.setName("spin");
        this.setInputs(new TestInput[] { speed });
    }

    @Override
    public void init() {
        Swerve.getInstance().snapForward();
        angle = 0;
    }

    @Override
    public void run() {
        // Swerve.getInstance().testModules(speed.getValue());
        // Swerve.getInstance().getModules().forEach((mod) -> {
        // mod.drive(0.5, angle);
        // System.out.println(mod.driveMotor.getEncoderVelocity());
        // });

        Swerve.getInstance().spinTest(speed.getValue(), angle);
        System.out.println(speed.getValue());

        angle += direction ? -1 : 1;

        if (angle >= 360 && !direction)
            direction = true;
        else if (angle <= 0 && direction) {
            angle = 0;
            direction = false;
        }

        this.writeOutput("You are currrently running Swerve Spin Test at Speed " + speed.getValue());
    }

    @Override
    public void stop() {
        Swerve.getInstance().stop();
    }
}
