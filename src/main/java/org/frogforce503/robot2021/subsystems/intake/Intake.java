package org.frogforce503.robot2021.subsystems.intake;

import org.frogforce503.robot2021.Robot;

public class Intake {
    private static NewIntake newInstance = null;
    private static LegacyIntake legacyInstance = null;

    public static IntakeBase getInstance() {
        if (Robot.bot.hasMotorizedHood())
            return newInstance == null ? newInstance = NewIntake.getInstance() : newInstance;
        else
            return legacyInstance == null ? legacyInstance = LegacyIntake.getInstance() : legacyInstance;
    }

    public enum IntakeControlState {
        INTAKING(0.70, true), SHOOTING(0.80, true), EJECTING(-0.85, true), OUT(0.0, true), OFF(0, false);

        private double power = 0.0;
        private boolean solenoid = false;

        IntakeControlState(double power, boolean solenoid) {
            this.power = power;
            this.solenoid = solenoid;
        }

        public double getPower() {
            return this.power;
        }

        public boolean getSolenoid() {
            return this.solenoid;
        }

    }
}
