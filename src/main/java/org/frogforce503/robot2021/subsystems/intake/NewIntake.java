
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frogforce503.robot2021.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.frogforce503.robot2021.Robot;
import org.frogforce503.robot2021.subsystems.intake.Intake.IntakeControlState;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NewIntake extends IntakeBase {

    private static NewIntake instance = null;
    TalonFX intakeMotor;
    DoubleSolenoid intakeShifter;
    private boolean isRunning;
    private IntakeControlState controlState = IntakeControlState.OFF;

    private NewIntake() {
        intakeMotor = new TalonFX(Robot.bot.intakeID);

        // intakeMotor.configSupplyCurrentLimit(new
        // SupplyCurrentLimitConfiguration(true, 70, 71, 0), 10);
        intakeMotor.setNeutralMode(NeutralMode.Coast);

        if (Robot.bot.hasMotorizedHood()) {
            intakeShifter = new DoubleSolenoid(Robot.bot.shiftForwardID, Robot.bot.shiftReverseID);
        }

        // stop();
    }

    public static NewIntake getInstance() {
        return instance == null ? instance = new NewIntake() : instance;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Intake Power", intakeMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Intake Stator Current", intakeMotor.getStatorCurrent());
        SmartDashboard.putNumber("Intake Supply Current", intakeMotor.getSupplyCurrent());
        SmartDashboard.putString("Intake State", controlState.toString());
    }

    @Override
    public void stop() {
        System.out.println("Stopping intake");
        conformToState(IntakeControlState.OFF);
    }

    @Override
    public void writePeriodicOutputs() {
        double powerToSet = controlState.getPower();

        intakeMotor.set(ControlMode.PercentOutput, powerToSet);
        intakeShifter.set(controlState.getSolenoid() ? Value.kForward : Value.kReverse);

    }

    @Override
    public boolean isIntakeRunning() {
        return isRunning;
    }

    @Override
    public void setIntakeState(boolean running) {
        isRunning = running;
    }

    @Override
    public void conformToState(IntakeControlState state) {
        controlState = state;
    }

    @Override
    public IntakeControlState getState() {
        return controlState;
    }

    @Override
    public void onStart(double timestamp) {
        stop();
    }

    @Override
    public void onLoop(double timestamp) {

    }

    @Override
    public void onStop(double timestamp) {
        stop();
    }
}
