
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frogforce503.robot2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frogforce503.lib.drivers.TalonWrapper;

public class Feeder extends Subsystem {

    /* Feeder PWR / Spindexer RPM = 0.9 / 89.315*/

    private static Feeder instance = null;
    TalonWrapper feederMotor;
    double power = 0.5;
    LinearFilter powerFilter = LinearFilter.highPass(0.1, 0.02);
    private final PeriodicIO periodicIO = new PeriodicIO();

    private Feeder() {
        feederMotor = new TalonWrapper(3);
        feederMotor.setInverted(true);

        feederMotor.configVoltageCompSaturation(12.0, 10);
        feederMotor.enableVoltageCompensation(true);
        feederMotor.configNominalOutputForward(0.0 / 12.0, 10);
        feederMotor.configContinuousCurrentLimit(80, 10);
        feederMotor.configPeakCurrentLimit(80, 10);
        feederMotor.configPeakCurrentDuration(100, 10);
        feederMotor.enableCurrentLimit(true);
        // intakeMotor.restoreFactoryDefaults();

        // intakeMotor.setSmartCurrentLimit(Constants.kIntakeCurrentLimit);

        stop();
    }

    public static Feeder getInstance() {
        return instance == null ? instance = new Feeder() : instance;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Feeder Power", feederMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Feeder Power Filtered", powerFilter.calculate(feederMotor.getMotorOutputPercent()));
        SmartDashboard.putNumber("Feeder Current", feederMotor.getStatorCurrent());

        // SmartDashboard.putString("Intake State", controlState.toString());
        // SmartDashboard.putNumber("Intake Current", panel.getCurrent(6));
    }

    @Override
    public void stop() {
        // System.out.println("Stopping intake");
        // conformToState(IntakeControlState.OFF);
        periodicIO.feederPower = 0.0;
    }

    @Override
    public void onStart(double timestamp) {
        stop();
    }

    @Override
    public void onLoop(double timestamp) {

    }

    @Override
    public void writePeriodicOutputs() {

        // if (panel.getCurrent(6) > softwareMaxCurrent){
        // powerToSet *= overCurrentPowerProtectionKp;
        // }
        feederMotor.set(ControlMode.PercentOutput, periodicIO.feederPower);

    }

    @Override
    public void onStop(double timestamp) {
        stop();
    }

    // public boolean isIntakeRunning() {
    // return isRunning;
    // }

    // public void setIntakeState(boolean running) {
    // isRunning = running;
    // }

    // public void conformToState(IntakeControlState state) {
    // controlState = state;
    // }

    // public IntakeControlState getState() {
    // return controlState;
    // }

    public void setOpenLoop(double feederPower) {
        periodicIO.feederPower = feederPower;
    }

    public void setCurrentLimit(int limit) {
        // intakeMotor.setCurrent
    }

    public class PeriodicIO {
        public double feederPower;
    }

    // public Request stateRequest(IntakeControlState state) {
    // return new Request() {

    // @Override
    // public void act() {
    // conformToState(state);
    // }

    // @Override
    // public boolean isFinished() {
    // return controlState == state;
    // }
    // };
    // }

    // public Request stopRequest() {
    // return new Request() {

    // @Override
    // public void act() {
    // stop();
    // }

    // @Override
    // public boolean isFinished() {
    // return true;
    // }
    // };
    // }

}
