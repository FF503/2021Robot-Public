package org.frogforce503.robot2021.subsystems.intake;

import org.frogforce503.robot2021.subsystems.Subsystem;
import org.frogforce503.robot2021.subsystems.intake.Intake.IntakeControlState;

public abstract class IntakeBase extends Subsystem {
    public boolean isIntakeRunning() {
        return false;
    }

    public void setIntakeState(boolean running) {
        // Do nothing
    }

    public void conformToState(IntakeControlState state) {
        // Do nothing
    }

    public IntakeControlState getState() {
        return null;
    }
}
