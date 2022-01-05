/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frogforce503.robot2021.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frogforce503.robot2021.Robot;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {

    private static Climber instance = null;
    TalonFX climber;
    DoubleSolenoid climberRelease;
    double startTime = -1.0;

    public Climber() {
        climber = new TalonFX(Robot.bot.climberID);
        climberRelease = new DoubleSolenoid(Robot.bot.climberForwardID, Robot.bot.climberReverseID);
        climber.getSensorCollection().setIntegratedSensorPosition(0.0, 10);
    }

    public static Climber getInstance() {
        return instance = instance == null ? new Climber() : instance;
    }

    public double getEncoderCounts() {
        return climber.getSensorCollection().getIntegratedSensorPosition();
    }

    public void setOpenLoop(double power) {
        if (Math.abs(power) <= 0.05) {
            startTime = -1.0;
            climber.set(TalonFXControlMode.PercentOutput, 0.0);
            climberRelease.set(Value.kForward);
        } else if (startTime == -1.0) {
            startTime = Timer.getFPGATimestamp();
            if (power > 0.05) {
                climberRelease.set(Value.kReverse);
            } else {
                climberRelease.set(Value.kForward);
            }
        } else {
            System.out.println("Starting climber motion");
            if (Timer.getFPGATimestamp() - startTime > 0.5) {
                climber.set(TalonFXControlMode.PercentOutput, power);
            }
        }

    }

    @Override
    public void readPeriodicInputs() {

    }

    @Override
    public void writePeriodicOutputs() {

    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Climber current", climber.getStatorCurrent());
        SmartDashboard.putNumber("Climber position", getEncoderCounts());
    }

    @Override
    public void stop() {

    }

    @Override
    public void onStart(double timestamp) {

    }

    @Override
    public void onLoop(double timestamp) {

    }

    @Override
    public void onStop(double timestamp) {
        stop();
    }

}
