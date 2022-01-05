package org.frogforce503.robot2021.tests.modes;

import org.frogforce503.robot2021.subsystems.swerve.Swerve;
import org.frogforce503.robot2021.tests.RobotTest;

public class ReZeroSwerve extends RobotTest {

    String output = "";
    int index = 0;
    Boolean hasRan = false;

    String[] motorNames = new String[] { "BR", "BL", "FL", "FR" };

    public ReZeroSwerve() {
        super();
        // this.setSubsystem(Swerve.getInstance());
        this.setName("rezero_swerve");
    }

    @Override
    public void run() {
        if (!hasRan) {
            // backRight, backLeft, frontLeft, frontRight
            // 2154 front left
            Swerve.getInstance().getModules().forEach((mod) -> {
                // mod.turnMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,
                // kSlotIdx, kTimeoutMs);
                int position = mod.turnMotor.getSensorCollection().getPulseWidthPosition();
                double offset = position % 4096;

                output += motorNames[index] + ": " + offset + (index < 3 ? ", " : "");
                index++;
            });
            /**
             * Set all swerve motors to absolute mode Get values for all of the encoders
             * (save to variables) print "new zero positions are: {position}
             * {position}...(and for the other ones)"
             */
            this.writeOutput("The new zero positions are: " + output);
            hasRan = true;
        }
    }

    @Override
    public void stop() {

    }

}
